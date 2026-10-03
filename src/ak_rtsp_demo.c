/*
 * ak_rtsp_demo.c
 *
 * Anyka VI + RTSP demo with:
 *   - H.264 main/sub RTSP streams
 *   - IR-cut day/night control
 *   - IR LED control
 *   - Anyka ISP DAY/NIGHT profile switching
 *   - automatic day/night detection
 *   - simple HTTP control page
 *
 * ISP modes:
 *
 *   mode 0 = DAY
 *   mode 1 = NIGHT
 *
 * The ISP mode is changed with:
 *
 *   ak_vi_switch_mode(vi_handle, mode)
 *
 * The actual ABI was verified from libplat_vi.so:
 *
 *   int ak_vi_switch_mode(void *vi_handle, int mode);
 *
 * DAY:
 *   IR-cut DAY
 *   IR LED OFF
 *   ISP DAY
 *
 * NIGHT:
 *   IR-cut NIGHT
 *   IR LED ON
 *   ISP NIGHT
 *
 * AUTO detection is intentionally asymmetric.
 *
 * Observed camera values:
 *
 *   DAY:
 *       luma ~= 54..60
 *       gain ~= 256..1538
 *
 *   NIGHT:
 *       luma ~= 26..29
 *       gain ~= 3968
 *
 * DAY -> NIGHT:
 *       gain >= IR_NIGHT_GAIN
 *
 * NIGHT -> DAY:
 *       the camera is temporarily switched to ISP DAY,
 *       then AE is allowed to settle.
 *
 *       DAY is confirmed when:
 *           luma >= IR_PROBE_DAY_LUM
 *           AND
 *           gain <= IR_PROBE_DAY_GAIN
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include <unistd.h>
#include <getopt.h>
#include <fcntl.h>
#include <errno.h>
#include <pthread.h>

#include <sys/socket.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#include "ak_common.h"
#include "ak_vi.h"
#include "ak_vpss.h"
#include "ak_rtsp.h"
#include "akuio.h"


/* ------------------------------------------------------------------------- */
/* IR hardware                                                               */
/* ------------------------------------------------------------------------- */

#define IRCUT_A_FILE_NAME       "/sys/user-gpio/gpio-ircut_a"
#define IRCUT_B_FILE_NAME       "/sys/user-gpio/gpio-ircut_b"
#define IRLED_FILE_NAME         "/sys/user-gpio/ir-led"

#define IRLED_ON_VAL            "1"
#define IRLED_OFF_VAL           "0"

/* ------------------------------------------------------------------------- */
/* Anyka ISP modes                                                           */
/* ------------------------------------------------------------------------- */

#define ISP_MODE_DAY             0
#define ISP_MODE_NIGHT           1

/*
 * Verified from the ARM disassembly of libplat_vi.so:
 *
 *     int ak_vi_switch_mode(void *vi_handle, int mode);
 */



/* ------------------------------------------------------------------------- */
/* AUTO day/night detection                                                  */
/* ------------------------------------------------------------------------- */

/*
 * Observed:
 *
 * DAY:
 *     luma 54..60
 *     gain 256..1538
 *
 * NIGHT:
 *     luma 26..29
 *     gain ~3968
 *
 * Gain is therefore the primary DAY -> NIGHT detector.
 */
#define IR_NIGHT_GAIN  2000
#define IR_NIGHT_DAY_LUMA  100   /* night->day when luma > this (real light) */

/*
 * The following values are retained for informational/debug purposes.
 */
#define IR_NIGHT_LUM            40
#define IR_DAY_LUM              52
#define IR_DAY_GAIN     400

/*
 * Polling interval.
 */
#define IR_POLL_SEC             2

/*
 * Number of consecutive NIGHT detections required.
 *
 * 5 x 2 seconds = approximately 10 seconds.
 */
#define IR_CONFIRM              3

/*
 * Wait after changing ISP profile before normal AUTO decisions resume.
 */
#define IR_SETTLE_POLLS         8

/*
 * Minimum time spent in NIGHT before probing for DAY.
 *
 * 15 x 2 seconds = approximately 30 seconds.
 */
#define IR_NIGHT_HOLD_POLLS  150

/*
 * Number of polls allowed for the DAY ISP profile to settle.
 *
 * 5 x 2 seconds = approximately 10 seconds.
 */
#define IR_PROBE_SETTLE_POLLS   5

/*
 * Observed real DAY luma is approximately 54..60.
 *
 * Use 54 instead of 70 because 70 was above the actual measured
 * daytime range and therefore prevented NIGHT -> DAY confirmation.
 */
#define IR_PROBE_DAY_LUM     45

/*
 * Observed DAY analogue gain is approximately 256..1538.
 */
#define IR_PROBE_DAY_GAIN       1200


/* ------------------------------------------------------------------------- */
/* Web interface                                                             */
/* ------------------------------------------------------------------------- */

#define WEBUI_PORT              80


/* ------------------------------------------------------------------------- */
/* RTSP configuration                                                        */
/* ------------------------------------------------------------------------- */

#define FIRST_PATH              "/etc/jffs2/isp_f37_mipi1lane.conf"

#define LEN_HINT                512

#define DEFAULT_MAIN_WIDTH      1280
#define DEFAULT_MAIN_HEIGHT     720
#define DEFAULT_SUB_WIDTH       640
#define DEFAULT_SUB_HEIGHT      480

#define DEFAULT_MAIN_FPS        25
#define DEFAULT_SUB_FPS         25

#define DEFAULT_MINQP           20
#define DEFAULT_MAXQP           51
#define DEFAULT_GOP             2

#define DEFAULT_MAIN_MODE       BR_MODE_CBR
#define DEFAULT_SUB_MODE        BR_MODE_CBR

#define DEFAULT_MAIN_TYPE       H264_ENC_TYPE
#define DEFAULT_SUB_TYPE        H264_ENC_TYPE

#define DEFAULT_MAIN_KBPS       2000
#define DEFAULT_SUB_KBPS       200

#define DEFAULT_MAIN_SUFFIX     "vs0"
#define DEFAULT_SUB_SUFFIX      "vs1"


/* ------------------------------------------------------------------------- */
/* AE structure                                                              */
/* ------------------------------------------------------------------------- */

typedef struct {
    unsigned char current_calc_avg_lumi;
    unsigned char current_calc_avg_compensation_lumi;
    unsigned char current_darked_flag;

    int current_a_gain;
    int current_d_gain;
    int current_isp_d_gain;
    int current_exp_time;

    unsigned int current_a_gain_step;
    unsigned int current_d_gain_step;
    unsigned int current_isp_d_gain_step;
    unsigned int current_exp_time_step;
} AK_AE_RUN;

extern int AK_ISP_get_ae_run_info(AK_AE_RUN *p);
/* AE attr for night brightness boost - matches ak_isp_drv.h AK_ISP_AE_ATTR */
typedef struct {
    unsigned int exp_time_max, exp_time_min;
    unsigned int d_gain_max, d_gain_min;
    unsigned int isp_d_gain_min, isp_d_gain_max;
    unsigned int a_gain_max, a_gain_min;
    unsigned int exp_step, exp_stable_range;
    unsigned int target_lumiance;
    unsigned int envi_gain_range[10][2];
    unsigned int hist_weight[16];
    unsigned int OE_suppress_en, OE_detect_scope, OE_rate_max, OE_rate_min;
} AK_AE_ATTR;
extern int AK_ISP_get_ae_attr(AK_AE_ATTR *p);
extern int AK_ISP_set_ae_attr(const AK_AE_ATTR *p);

/* ------------------------------------------------------------------------- */
/* Global state                                                              */
/* ------------------------------------------------------------------------- */

static volatile sig_atomic_t run_flag = AK_FALSE;

void *vi_handle = NULL;

static int g_ir_run = 0;
static int g_ir_status = 1;
static int g_ir_mode = 0;

static pthread_t g_ir_tid;
static pthread_mutex_t g_ir_mutex = PTHREAD_MUTEX_INITIALIZER;

static int g_probe_state = 0;
static int g_probe_polls = 0;
static int g_night_hold = 0;

static int g_web_run = 0;
static int g_web_sock = -1;
static pthread_t g_web_tid;

static int i_main_width = DEFAULT_MAIN_WIDTH;
static int i_main_height = DEFAULT_MAIN_HEIGHT;
static int i_sub_width = DEFAULT_SUB_WIDTH;
static int i_sub_height = DEFAULT_SUB_HEIGHT;
static int i_main_kbps = DEFAULT_MAIN_KBPS;
static int i_sub_kbps = DEFAULT_SUB_KBPS;
static int i_main_fps = DEFAULT_MAIN_FPS;
static int i_sub_fps = DEFAULT_SUB_FPS;
static int i_main_mode = DEFAULT_MAIN_MODE;
static int i_sub_mode = DEFAULT_SUB_MODE;
static int i_main_type = DEFAULT_MAIN_TYPE;
static int i_sub_type = DEFAULT_SUB_TYPE;

static char *pc_main_name = DEFAULT_MAIN_SUFFIX;
static char *pc_sub_name = DEFAULT_SUB_SUFFIX;
static char *pc_prog_name = NULL;

static int i_gop = DEFAULT_GOP;
static int i_minqp = DEFAULT_MINQP;
static int i_maxqp = DEFAULT_MAXQP;


/* ------------------------------------------------------------------------- */
/* CLI                                                                       */
/* ------------------------------------------------------------------------- */

static char ac_option_hint[][LEN_HINT] = {
    "       HELP",
    "[NUM]  ( DEFAULT: 1280 )",
    "[NUM]  ( DEFAULT: 720 )",
    "[NUM]  ( DEFAULT: 640 )",
    "[NUM]  ( DEFAULT: 480 )",
    "[NUM]  ( DEFAULT: 2000 )",
    "[NUM]  ( DEFAULT: 200 )",
    "[NUM]  ( DEFAULT: 25 )",
    "[NUM]  ( DEFAULT: 25 )",
    "[NUM]  ( DEFAULT: 0 )",
    "[NUM]  ( DEFAULT: 0 )",
    "[NUM]  ( DEFAULT: 0 )",
    "[NUM]  ( DEFAULT: 0 )",
    "[NAME] ( DEFAULT: 'vs0' )",
    "[NAME] ( DEFAULT: 'vs1' )",
    "[NUM]  ( DEFAULT: 2 )",
    "[NUM]  ( DEFAULT: 20 )",
    "[NUM]  ( DEFAULT: 51 )"
};

static struct option option_long[] = {
    { "help",          no_argument,       NULL, 'h' },
    { "main-width",    required_argument, NULL, 'a' },
    { "main-height",   required_argument, NULL, 'b' },
    { "sub-width",     required_argument, NULL, 'c' },
    { "sub-height",    required_argument, NULL, 'd' },
    { "main-kbps",     required_argument, NULL, 'e' },
    { "sub-kbps",      required_argument, NULL, 'f' },
    { "main-fps",      required_argument, NULL, 'g' },
    { "sub-fps",       required_argument, NULL, 'i' },
    { "main-mode",     required_argument, NULL, 'j' },
    { "sub-mode",      required_argument, NULL, 'k' },
    { "main-type",     required_argument, NULL, 'l' },
    { "sub-type",      required_argument, NULL, 'm' },
    { "main-chn-name", required_argument, NULL, 'n' },
    { "sub-chn-name",  required_argument, NULL, 'o' },
    { "gop",           required_argument, NULL, 'p' },
    { "minqp",         required_argument, NULL, 'q' },
    { "maxqp",         required_argument, NULL, 'r' }
};


/* ------------------------------------------------------------------------- */
/* Utilities                                                                 */
/* ------------------------------------------------------------------------- */

static int send_all(int fd, const char *buf, size_t len)
{
    size_t sent = 0;

    while (sent < len) {
        ssize_t n = send(fd, buf + sent, len - sent, 0);

        if (n < 0) {
            if (errno == EINTR)
                continue;

            return -1;
        }

        if (n == 0)
            return -1;

        sent += (size_t)n;
    }

    return 0;
}


static int parse_positive_int(const char *value, int *result)
{
    char *end = NULL;
    long number;

    if (value == NULL || result == NULL)
        return -1;

    errno = 0;

    number = strtol(value, &end, 10);

    if (errno != 0 || end == value || *end != '\0')
        return -1;

    if (number <= 0 || number > 1000000)
        return -1;

    *result = (int)number;

    return 0;
}


/* ------------------------------------------------------------------------- */
/* IR GPIO                                                                   */
/* ------------------------------------------------------------------------- */

static int ir_write(const char *path, const char *value)
{
    int fd;
    ssize_t len;
    ssize_t written;

    if (path == NULL || value == NULL)
        return -1;

    fd = open(path, O_WRONLY);

    if (fd < 0) {
        fprintf(stderr,
                "[ir] open(%s) failed: %s\n",
                path,
                strerror(errno));

        return -1;
    }

    len = (ssize_t)strlen(value);

    written = write(fd, value, (size_t)len);

    if (written != len) {
        fprintf(stderr,
                "[ir] write(%s) failed: %s\n",
                path,
                (written < 0) ? strerror(errno) : "short write");

        close(fd);

        return -1;
    }

    close(fd);

    return 0;
}


/* ------------------------------------------------------------------------- */
/* ISP switching                                                             */
/* ------------------------------------------------------------------------- */

static int ir_switch_isp_locked(int mode)
{
    int ret;

    if (vi_handle == NULL) {
        fprintf(stderr,
                "[ir] cannot switch ISP mode: vi_handle is NULL\n");

        return -1;
    }

    if (mode != ISP_MODE_DAY && mode != ISP_MODE_NIGHT) {
        fprintf(stderr,
                "[ir] invalid ISP mode: %d\n",
                mode);

        return -1;
    }

    ret = ak_vi_switch_mode(vi_handle, mode);

    fprintf(stderr,
            "[ir] ak_vi_switch_mode(%s) ret=%d\n",
            mode == ISP_MODE_DAY ? "DAY" : "NIGHT",
            ret);

    return ret;
}


/* ------------------------------------------------------------------------- */
/* IR transitions                                                            */
/* ------------------------------------------------------------------------- */

static int ir_set_day_locked(void)
{
    int result = 0;

    fprintf(stderr,
            "[ir] switching to DAY\n");

    /*
     * IR-cut DAY:
     *
     * A = 0
     * B = 1
     */
    if (ir_write(IRCUT_A_FILE_NAME, "0") != 0)
        result = -1;

    if (ir_write(IRCUT_B_FILE_NAME, "1") != 0)
        result = -1;

    usleep(120000);

    if (ir_write(IRCUT_A_FILE_NAME, "0") != 0)
        result = -1;

    if (ir_write(IRCUT_B_FILE_NAME, "0") != 0)
        result = -1;

    /*
     * IR illumination must be disabled before using the DAY ISP profile.
     */
    if (ir_write(IRLED_FILE_NAME, IRLED_OFF_VAL) != 0)
        result = -1;

    if (ir_switch_isp_locked(ISP_MODE_DAY) != 0)
        result = -1;

    if (result == 0) {
        fprintf(stderr,
                "[ir] DAY active: IR-cut=DAY LED=OFF ISP=DAY COLOR\n");
    } else {
        fprintf(stderr,
                "[ir] DAY transition failed\n");
    }

    return result;
}


static int ir_set_night_locked(void)
{
    int result = 0;

    fprintf(stderr,
            "[ir] switching to NIGHT\n");

    /*
     * IR-cut NIGHT:
     *
     * A = 1
     * B = 0
     */
    if (ir_write(IRCUT_A_FILE_NAME, "1") != 0)
        result = -1;

    if (ir_write(IRCUT_B_FILE_NAME, "0") != 0)
        result = -1;

    usleep(120000);

    if (ir_write(IRCUT_A_FILE_NAME, "0") != 0)
        result = -1;

    if (ir_write(IRCUT_B_FILE_NAME, "0") != 0)
        result = -1;

    /*
     * Enable IR illumination before the NIGHT ISP profile settles.
     */
    if (ir_write(IRLED_FILE_NAME, IRLED_ON_VAL) != 0)
        result = -1;

    if (ir_switch_isp_locked(ISP_MODE_NIGHT) != 0)
        result = -1;

    if (result == 0) {
        fprintf(stderr,
                "[ir] NIGHT active: IR-cut=NIGHT LED=ON ISP=NIGHT B&W\n");
    } else {
        fprintf(stderr,
                "[ir] NIGHT transition failed\n");
    }

    return result;
}


/* ------------------------------------------------------------------------- */
/* AE                                                                        */
/* ------------------------------------------------------------------------- */

static int ir_read_ae(AK_AE_RUN *ae)
{
    if (ae == NULL)
        return -1;

    memset(ae, 0, sizeof(*ae));

    if (AK_ISP_get_ae_run_info(ae) != 0)
        return -1;

    return 0;
}


static int ir_read_ae_values(int *luma, int *gain, int *darkflag)
{
    AK_AE_RUN ae;

    if (luma == NULL || gain == NULL || darkflag == NULL)
        return -1;

    if (ir_read_ae(&ae) != 0)
        return -1;

    *luma = (int)ae.current_calc_avg_lumi;
    *gain = ae.current_a_gain;
    *darkflag = (int)ae.current_darked_flag;

    return 0;
}


/* ------------------------------------------------------------------------- */
/* AUTO detector                                                             */
/* ------------------------------------------------------------------------- */

static void *ir_thread(void *arg)
{
    int previous_mode = -1;

    (void)arg;


    /* piotr-go's full auto day/night threshold setup.
       The FULL arrays (all 5 night_cnt, all 10 day_cnt) + lock_time
       are what the ISP needs to evaluate the switch - partial arrays
       don't work. Values from the GC2 vendor config. */
    {
        struct ak_auto_day_night_threshold threshold;
        memset(&threshold, 0, sizeof(threshold));
        threshold.day_to_night_lum = 3584;
        threshold.night_to_day_lum = 1536;
        threshold.lock_time = 900000;
        threshold.night_cnt[0] = 15000;
        threshold.night_cnt[1] = 15000;
        threshold.night_cnt[2] = 15000;
        threshold.night_cnt[3] = 15000;
        threshold.night_cnt[4] = 15000;
        threshold.day_cnt[0] = 300000;
        threshold.day_cnt[1] = 300000;
        threshold.day_cnt[2] = 300000;
        threshold.day_cnt[3] = 300000;
        threshold.day_cnt[4] = 300000;
        threshold.day_cnt[5] = 300000;
        threshold.day_cnt[6] = 300000;
        threshold.day_cnt[7] = 300000;
        threshold.day_cnt[8] = 300000;
        threshold.day_cnt[9] = 300000;
        ak_vpss_isp_set_auto_day_night_param(&threshold);
        fprintf(stderr, "[ir] auto d/n param set (piotr full arrays)\n");
    }

    while (1) {
        int mode;
        int cur;

        pthread_mutex_lock(&g_ir_mutex);
        if (!g_ir_run) {
            pthread_mutex_unlock(&g_ir_mutex);
            break;
        }
        mode = g_ir_mode;
        pthread_mutex_unlock(&g_ir_mutex);

        if (mode != previous_mode) {
            if (mode == 0)
                fprintf(stderr, "[ir] mode changed to AUTO\n");
            else if (mode == 1)
                fprintf(stderr, "[ir] mode changed to FORCE DAY\n");
            else
                fprintf(stderr, "[ir] mode changed to FORCE NIGHT\n");
            previous_mode = mode;
        }

        /* FORCE modes: handled by the web handler */
        if (mode != 0) {
            sleep(IR_POLL_SEC);
            continue;
        }

        /* AUTO: ask the ISP for the day/night level.
         * pre_ir_level: 1 = currently day, 0 = currently night.
         * return: 1 = day, 0 = night, -1 = failed. */
        cur = ak_vpss_isp_get_auto_day_night_level(g_ir_status ? 1 : 0);


        if (cur >= 0 && cur != g_ir_status) {
            pthread_mutex_lock(&g_ir_mutex);
            if (g_ir_run && g_ir_mode == 0) {
                if (cur == 1) {
                    if (ir_set_day_locked() == 0) {
                        g_ir_status = 1;
                        fprintf(stderr, "[ir] AUTO -> DAY\n");
                    }
                } else {
                    if (ir_set_night_locked() == 0) {
                        g_ir_status = 0;
                        fprintf(stderr, "[ir] AUTO -> NIGHT\n");
                }
            }
          }
            pthread_mutex_unlock(&g_ir_mutex);
        }

        sleep(IR_POLL_SEC);
    }

    return NULL;
} 
/*
------------------------------------------------------------------------- */
/* IR start/stop                                                             */
/* ------------------------------------------------------------------------- */

static int ir_start(void)
{
    int ret;
    pthread_mutex_lock(&g_ir_mutex);


    g_ir_status = 1;
    g_ir_mode = 0;

    g_probe_state = 0;
    g_probe_polls = 0;
    g_night_hold = 0;

    ret = ir_set_day_locked();

    if (ret != 0) {
        pthread_mutex_unlock(&g_ir_mutex);

        fprintf(stderr,
                "[ir] initial DAY setup failed\n");

        return -1;
    }

    g_ir_run = 1;

    pthread_mutex_unlock(&g_ir_mutex);

    ret = pthread_create(
        &g_ir_tid,
        NULL,
        ir_thread,
        NULL
    );

    if (ret != 0) {
        pthread_mutex_lock(&g_ir_mutex);

        g_ir_run = 0;

        pthread_mutex_unlock(&g_ir_mutex);

        fprintf(stderr,
                "[ir] pthread_create failed: %s\n",
                strerror(ret));

        return -1;
    }

    fprintf(stderr,
            "[ir] day/night thread started\n");

    return 0;
}


static void ir_stop(void)
{
    int should_join;

    pthread_mutex_lock(&g_ir_mutex);

    should_join = g_ir_run;
    g_ir_run = 0;

    pthread_mutex_unlock(&g_ir_mutex);

    if (should_join) {
        pthread_join(g_ir_tid, NULL);

        fprintf(stderr,
                "[ir] day/night thread stopped\n");
    }

    /*
     * Always turn the IR LED off when stopping.
     */
    ir_write(IRLED_FILE_NAME, IRLED_OFF_VAL);
}


/* ------------------------------------------------------------------------- */
/* Web HTML                                                                  */
/* ------------------------------------------------------------------------- */


static int web_page(int fd)
{
    char response[8192];
    int luma = -1, gain = -1, darkflag = -1;
    int status, mode;
    const char *state, *mode_name;
    int ret;

    if (ir_read_ae_values(&luma, &gain, &darkflag) != 0) {
        luma = -1; gain = -1; darkflag = -1;
    }

    pthread_mutex_lock(&g_ir_mutex);
    status = g_ir_status;
    mode = g_ir_mode;
    pthread_mutex_unlock(&g_ir_mutex);

    state = status ? "DAY" : "NIGHT";
    mode_name = (mode == 0) ? "AUTO" : (mode == 1) ? "FORCE DAY" : "FORCE NIGHT";

    ret = snprintf(response, sizeof(response),
        "HTTP/1.1 200 OK\r\n"
        "Content-Type: text/html; charset=utf-8\r\n"
        "Connection: close\r\n"
        "\r\n"
        "<!DOCTYPE html><html><head><meta charset=utf-8>"
        "<meta name=viewport content='width=device-width,initial-scale=1'>"
        "<title>Anyka Camera</title>"
        "<style>"
        "*{box-sizing:border-box}"
        "body{font-family:system-ui,sans-serif;background:#1a1d21;color:#e8eaed;margin:0}"
        ".bar{background:#12151a;padding:.8em 1.2em;border-bottom:1px solid #2a2f36;display:flex;align-items:center;gap:.6em}"
        ".bar h1{font-size:1.1em;margin:0;color:#ff8c2a}"
        ".bar .sub{color:#7a828c;font-size:.85em}"
        ".wrap{max-width:720px;margin:0 auto;padding:1.2em}"
        ".card{background:#22272e;border:1px solid #2a2f36;border-radius:10px;padding:1em 1.2em;margin-bottom:1em}"
        ".card h2{margin:0 0 .6em;font-size:.95em;color:#ff8c2a;text-transform:uppercase;letter-spacing:.05em}"
        ".row{display:flex;justify-content:space-between;padding:.35em 0;border-bottom:1px solid #2a2f36}"
        ".row:last-child{border:0}"
        ".row .k{color:#9aa3ad}.row .v{font-weight:600}"
        ".badge{display:inline-block;padding:.25em .7em;border-radius:6px;font-weight:700;font-size:.9em}"
        ".day{background:#2d6b3f;color:#d4f5dd}.night{background:#2a4a6b;color:#d4e4f5}"
        ".btns{display:flex;gap:.5em;flex-wrap:wrap}"
        ".btns a{flex:1;min-width:90px;text-decoration:none}"
        "button{width:100%%;font-size:1em;padding:.7em;border:0;border-radius:8px;"
        "background:#333a42;color:#e8eaed;cursor:pointer;font-weight:600}"
        "button:hover{background:#3d454e}"
        ".b-day{background:#2d6b3f}.b-night{background:#2a4a6b}.b-auto{background:#6b5a2d}"
        "code{background:#12151a;padding:.3em .6em;border-radius:6px;color:#6cc;font-size:.9em;"
        "word-break:break-all;display:inline-block}"
        "a.lnk{color:#6cc}"
        "</style></head><body>"

        "<div class=bar><h1>&#128247; Anyka Camera</h1>"
        "<span class=sub>AK3918 / JXF37</span></div>"

        "<div class=wrap>"

        "<div class=card>"
        "<h2>Status</h2>"
        "<div class=row><span class=k>State</span>"
        "<span class=v><span class='badge %s'>%s</span></span></div>"
        "<div class=row><span class=k>Mode</span><span class=v>%s</span></div>"
        "<div class=row><span class=k>Luma</span><span class=v>%d</span></div>"
        "<div class=row><span class=k>Gain</span><span class=v>%d</span></div>"
        "</div>"

        "<div class=card>"
        "<div class=card>"
        "<h2>Live</h2>"
        "<img id=pv style='width:100%%;border-radius:8px;display:block;background:#000;min-height:200px' src=/snapshot.jpg>"
        "<script>setInterval(function(){document.getElementById('pv').src='/snapshot.jpg?t='+Date.now();},1000);</script>"
        "</div>"

        "<h2>Day / Night</h2>"
        "<div class=btns>"
        "<a href=/set?mode=day><button class=b-day>&#9728; Day</button></a>"
        "<a href=/set?mode=night><button class=b-night>&#9790; Night</button></a>"
        "<a href=/set?mode=auto><button class=b-auto>&#9881; Auto</button></a>"
        "</div></div>"

        "<div class=card>"
        "<h2>Streams</h2>"
        "<div class=row><span class=k>Main (1080p)</span>"
        "<span class=v><code id=m></code></span></div>"
        "<div class=row><span class=k>Sub (VGA)</span>"
        "<span class=v><code id=s></code></span></div>"
        "<script>var h=location.hostname;"
        "document.getElementById('m').textContent='rtsp://'+h+':554/%s';"
        "document.getElementById('s').textContent='rtsp://'+h+':554/%s';</script>"
        "</div>"

        "<div class=card>"
        "<h2>Snapshot</h2>"
        "<p style='color:#9aa3ad;margin:.3em 0'>Grab a still with ffmpeg:</p>"
        "<code id=ff></code>"
        "<script>document.getElementById('ff').textContent="
        "'ffmpeg -rtsp_transport tcp -i rtsp://'+location.hostname+':554/%s -frames:v 1 out.jpg';"
        "</script></div>"

        "<p style='text-align:center;color:#5a626c;font-size:.8em'>"
        "auto refresh 5s &middot; day/night via ISP</p>"

        "</div></body></html>\r\n",
        status ? "day" : "night", state,   /* badge class + text */
        mode_name,
        luma, gain,
        pc_main_name, pc_sub_name,         /* stream JS */
        pc_main_name                       /* ffmpeg */
    );

    if (ret < 0 || (size_t)ret >= sizeof(response))
        return -1;

    return send_all(fd, response, (size_t)ret);
}
    

/* ------------------------------------------------------------------------- */
/* Web request handling                                                      */
/* ------------------------------------------------------------------------- */

static int snapshot_serve(int fd);
static int snapshot_init(void);
static void snapshot_exit(void);

static int web_handle(int fd)
{
    char request[2048];
    ssize_t len;

    len = recv(
        fd,
        request,
        sizeof(request) - 1,
        0
    );

    if (len <= 0)
        return -1;

    request[len] = '\0';

    if (strncmp(
            request,
            "GET /set?mode=day",
            18) == 0) {

        pthread_mutex_lock(&g_ir_mutex);

        g_ir_mode = 1;
        g_probe_state = 0;
        g_probe_polls = 0;
        g_night_hold = 0;

        if (ir_set_day_locked() == 0)
            g_ir_status = 1;

        pthread_mutex_unlock(&g_ir_mutex);

        return web_page(fd);
    }

    if (strncmp(
            request,
            "GET /set?mode=night",
            20) == 0) {

        pthread_mutex_lock(&g_ir_mutex);

        g_ir_mode = 2;
        g_probe_state = 0;
        g_probe_polls = 0;
        g_night_hold = 0;

        if (ir_set_night_locked() == 0)
            g_ir_status = 0;

        pthread_mutex_unlock(&g_ir_mutex);

        return web_page(fd);
    }

    if (strncmp(
            request,
            "GET /set?mode=auto",
            19) == 0) {

        pthread_mutex_lock(&g_ir_mutex);

        g_ir_mode = 0;

        g_probe_state = 0;
        g_probe_polls = 0;
        g_night_hold = 0;

        pthread_mutex_unlock(&g_ir_mutex);

        return web_page(fd);
    }

       if (strncmp(request, "GET /snapshot.jpg", 17) == 0) {
        return snapshot_serve(fd);
    }

    return web_page(fd);
}


/* ------------------------------------------------------------------------- */
/* Web thread                                                                */
/* ------------------------------------------------------------------------- */

static void *web_thread(void *arg)
{
    (void)arg;

    while (1) {
        int client_fd;

        struct sockaddr_in client_addr;
        socklen_t client_len = sizeof(client_addr);

        pthread_mutex_lock(&g_ir_mutex);

        if (!g_web_run) {
            pthread_mutex_unlock(&g_ir_mutex);
            break;
        }

        pthread_mutex_unlock(&g_ir_mutex);

        client_fd = accept(
            g_web_sock,
            (struct sockaddr *)&client_addr,
            &client_len
        );

        if (client_fd < 0) {
            if (errno == EINTR)
                continue;

            pthread_mutex_lock(&g_ir_mutex);

            if (!g_web_run) {
                pthread_mutex_unlock(&g_ir_mutex);
                break;
            }

            pthread_mutex_unlock(&g_ir_mutex);

            continue;
        }

        web_handle(client_fd);

        close(client_fd);
    }

    return NULL;
}


/* ------------------------------------------------------------------------- */
/* Web start/stop                                                            */
/* ------------------------------------------------------------------------- */

static int web_start(void)
{
    struct sockaddr_in addr;

    int opt = 1;
    int ret;

    g_web_sock = socket(
        AF_INET,
        SOCK_STREAM,
        0
    );

    if (g_web_sock < 0) {
        fprintf(stderr,
                "[web] socket failed: %s\n",
                strerror(errno));

        return -1;
    }

    if (setsockopt(
            g_web_sock,
            SOL_SOCKET,
            SO_REUSEADDR,
            &opt,
            sizeof(opt)) < 0) {

        fprintf(stderr,
                "[web] setsockopt failed: %s\n",
                strerror(errno));

        close(g_web_sock);
        g_web_sock = -1;

        return -1;
    }

    memset(
        &addr,
        0,
        sizeof(addr)
    );

    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    addr.sin_port = htons(WEBUI_PORT);

    if (bind(
            g_web_sock,
            (struct sockaddr *)&addr,
            sizeof(addr)) < 0) {

        fprintf(stderr,
                "[web] bind(:%d) failed: %s\n",
                WEBUI_PORT,
                strerror(errno));

        close(g_web_sock);
        g_web_sock = -1;

        return -1;
    }

    if (listen(g_web_sock, 8) < 0) {
        fprintf(stderr,
                "[web] listen failed: %s\n",
                strerror(errno));

        close(g_web_sock);
        g_web_sock = -1;

        return -1;
    }

    pthread_mutex_lock(&g_ir_mutex);

    g_web_run = 1;

    pthread_mutex_unlock(&g_ir_mutex);

    ret = pthread_create(
        &g_web_tid,
        NULL,
        web_thread,
        NULL
    );

    if (ret != 0) {
        pthread_mutex_lock(&g_ir_mutex);

        g_web_run = 0;

        pthread_mutex_unlock(&g_ir_mutex);

        close(g_web_sock);
        g_web_sock = -1;

        fprintf(stderr,
                "[web] pthread_create failed: %s\n",
                strerror(ret));

        return -1;
    }

    fprintf(stderr,
            "[web] UI thread started\n");

    fprintf(stderr,
            "[web] listening on :%d\n",
            WEBUI_PORT);

    return 0;
}


static void web_stop(void)
{
    int should_join;

    pthread_mutex_lock(&g_ir_mutex);

    should_join = g_web_run;
    g_web_run = 0;

    pthread_mutex_unlock(&g_ir_mutex);

    if (g_web_sock >= 0) {
        shutdown(
            g_web_sock,
            SHUT_RDWR
        );

        close(g_web_sock);

        g_web_sock = -1;
    }

    if (should_join) {
        pthread_join(
            g_web_tid,
            NULL
        );

        fprintf(stderr,
                "[web] UI thread stopped\n");
    }
}

/* ===== JPEG snapshot support (Thingino-style live preview) =====
   Opens an MJPEG encoder on the VI, serves /snapshot.jpg.
   Add externs (ak_venc.h / ak_global.h already included via ak_rtsp etc.,
   but declare if needed). Uses a mutex for thread-safe frame grab. */

/* globals (add with the other statics) */


/* call ONCE after ak_rtsp_start, with the global vi_handle valid */
/* ===== Snapshot via ak_venc_send_frame (vendor-style, no request_stream) =====
   Opens a persistent MJPEG encoder (no request_stream, so no continuous
   VI-capture binding that conflicts with the ISP day/night). On request,
   grabs a raw frame with ak_vi_get_frame and encodes it one-shot with
   ak_venc_send_frame. Single-mode: we free stream->data after sending.

   Replaces snapshot_init/exit/serve. */

#include "ak_global.h"   /* struct frame, video_stream - usually already via ak_vi.h */

static void *g_jpeg_enc = NULL;
static pthread_mutex_t g_jpeg_mutex = PTHREAD_MUTEX_INITIALIZER;

static int snapshot_init(void)
{
    struct encode_param ep;
    memset(&ep, 0, sizeof(ep));
    ep.width        = i_sub_width;
    ep.height       = i_sub_height;
    ep.minqp        = 20;
    ep.maxqp        = 51;
    ep.fps          = i_sub_fps;
    ep.goplen       = i_sub_fps;
    ep.bps          = i_sub_kbps;
    ep.profile      = PROFILE_BASE;
    ep.use_chn      = ENCODE_SUB_CHN;
    ep.enc_grp      = ENCODE_PICTURE;
    ep.br_mode      = BR_MODE_VBR;
    ep.enc_out_type = MJPEG_ENC_TYPE;

    /* open ONLY - do NOT request_stream (that binds VI capture -> ISP conflict) */
    g_jpeg_enc = ak_venc_open(&ep);
    if (!g_jpeg_enc) {
        fprintf(stderr, "[snap] ak_venc_open(jpeg) failed\n");
        return -1;
    }
    ak_venc_set_mjpeg_qlevel(g_jpeg_enc, 2);
    fprintf(stderr, "[snap] JPEG encoder ready (send_frame mode)\n");
    return 0;
}

static void snapshot_exit(void)
{
    if (g_jpeg_enc) { ak_venc_close(g_jpeg_enc); g_jpeg_enc = NULL; }
}

static int snapshot_serve(int fd)
{
    struct video_input_frame vif;
    struct video_stream vs;
    char hdr[128];
    int ok = -1;

    if (!g_jpeg_enc) {
        const char *r = "HTTP/1.1 503 Service Unavailable\r\n"
                        "Connection: close\r\n\r\nno encoder\n";
        send_all(fd, r, strlen(r));
        return -1;
    }

    pthread_mutex_lock(&g_jpeg_mutex);

    memset(&vif, 0, sizeof(vif));
    int _gr = ak_vi_get_frame(vi_handle, &vif);
    if (_gr == 0) {
        struct frame *f = &vif.vi_frame[VIDEO_CHN_SUB];
        if (f->data && f->len > 0) {
            memset(&vs, 0, sizeof(vs));
            ak_venc_send_frame(g_jpeg_enc, f->data, f->len, &vs);
            if (vs.data && vs.len > 0) {
                int hl = snprintf(hdr, sizeof(hdr),
                    "HTTP/1.1 200 OK\r\n"
                    "Content-Type: image/jpeg\r\n"
                    "Content-Length: %u\r\n"
                    "Cache-Control: no-store\r\n"
                    "Connection: close\r\n\r\n", vs.len);
                if (send_all(fd, hdr, hl) == 0 &&
                    send_all(fd, (const char *)vs.data, vs.len) == 0)
                    ok = 0;
                free(vs.data);
            }
        }
        ak_vi_release_frame(vi_handle, &vif);
    }

    pthread_mutex_unlock(&g_jpeg_mutex);

    if (ok != 0) {
        const char *r = "HTTP/1.1 503 Service Unavailable\r\n"
                        "Connection: close\r\n\r\nframe error\n";
        send_all(fd, r, strlen(r));
    }
    return ok;
}


/* ------------------------------------------------------------------------- */
/* VI initialization                                                         */
/* ------------------------------------------------------------------------- */

static void *ak_rtsp_vi_init(void)
{
    void *handle;

    struct video_resolution resolution;
    struct video_channel_attr attr;

    fprintf(stderr,
            "MARKER: entering ak_rtsp_vi_init\n");

    fprintf(stderr,
            "MARKER: before ak_vi_match_sensor(%s)\n",
            FIRST_PATH);

    if (ak_vi_match_sensor(FIRST_PATH) < 0) {
        ak_print_error_ex(
            "match sensor failed\n"
        );

        return NULL;
    }

    fprintf(stderr,
            "MARKER: after ak_vi_match_sensor OK\n");

    fprintf(stderr,
            "MARKER: before ak_vi_open\n");

    handle = ak_vi_open(VIDEO_DEV0);

    if (handle == NULL) {
        ak_print_error_ex(
            "vi open failed\n"
        );

        return NULL;
    }

    fprintf(stderr,
            "MARKER: after ak_vi_open OK handle=%p\n",
            handle);

    memset(
        &resolution,
        0,
        sizeof(resolution)
    );

    fprintf(stderr,
            "MARKER: before ak_vi_get_sensor_resolution\n");

    if (ak_vi_get_sensor_resolution(
            handle,
            &resolution)) {

        ak_print_error_ex(
            "get sensor resolution failed, "
            "using defaults %dx%d\n",
            DEFAULT_MAIN_WIDTH,
            DEFAULT_MAIN_HEIGHT
        );

        resolution.width = DEFAULT_MAIN_WIDTH;
        resolution.height = DEFAULT_MAIN_HEIGHT;
    } else {
        ak_print_normal(
            "sensor resolution height:%d, width:%d.\n",
            resolution.height,
            resolution.width
        );
    }

    fprintf(stderr,
            "MARKER: after ak_vi_get_sensor_resolution\n");

    memset(
        &attr,
        0,
        sizeof(attr)
    );

    attr.crop.left = 0;
    attr.crop.top = 0;

    attr.crop.width = resolution.width;
    attr.crop.height = resolution.height;

    attr.res[VIDEO_CHN_MAIN].width = i_main_width;
    attr.res[VIDEO_CHN_MAIN].height = i_main_height;
    attr.res[VIDEO_CHN_MAIN].max_width = resolution.width;
    attr.res[VIDEO_CHN_MAIN].max_height = resolution.height;

    attr.res[VIDEO_CHN_SUB].width = i_sub_width;
    attr.res[VIDEO_CHN_SUB].height = i_sub_height;
    attr.res[VIDEO_CHN_SUB].max_width = i_sub_width;
    attr.res[VIDEO_CHN_SUB].max_height = i_sub_height;

    fprintf(stderr,
            "MARKER: setting VI channels "
            "main=%dx%d sub=%dx%d\n",
            i_main_width,
            i_main_height,
            i_sub_width,
            i_sub_height);

    if (ak_vi_set_channel_attr(
            handle,
            &attr)) {

        ak_print_error_ex(
            "ak_vi_set_channel_attr failed\n"
        );

        return NULL;
    }

    ak_print_notice_ex(
        "start capture ...\n"
    );

    if (ak_vi_capture_on(handle)) {
        ak_print_error_ex(
            "ak_vi_capture_on failed\n"
        );

        return NULL;
    }

    fprintf(stderr,
            "MARKER: VI capture started\n");

    /*
     * Start explicitly in the DAY ISP profile.
     *
     * ir_start() also performs the complete DAY hardware transition,
     * but doing this here guarantees that the VI starts with the expected
     * profile before RTSP begins pulling frames.
     */
    if (ak_vi_switch_mode(
            handle,
            ISP_MODE_DAY) != 0) {

        fprintf(stderr,
                "[vi] initial ak_vi_switch_mode(DAY) failed\n");
    } else {
        fprintf(stderr,
                "[vi] initial ISP mode = DAY\n");
    }

    return handle;
}


/* ------------------------------------------------------------------------- */
/* CLI help                                                                  */
/* ------------------------------------------------------------------------- */

static int help_hint(void)
{
    size_t i;
    size_t count;

    count =
        sizeof(option_long) /
        sizeof(option_long[0]);

    printf(
        "%s\n",
        pc_prog_name
    );

    for (i = 0; i < count; i++) {
        printf(
            "\t--%-16s -%c %s\n",
            option_long[i].name,
            option_long[i].val,
            ac_option_hint[i]
        );
    }

    printf("\n");

    return 0;
}


/* ------------------------------------------------------------------------- */
/* Signals                                                                   */
/* ------------------------------------------------------------------------- */

static void process_signal(int sig)
{
    (void)sig;

    run_flag = AK_FALSE;
}


static int register_signal(void)
{
    signal(SIGINT, process_signal);
    signal(SIGTERM, process_signal);
    signal(SIGUSR1, process_signal);
    signal(SIGUSR2, process_signal);
    signal(SIGALRM, process_signal);
    signal(SIGHUP, process_signal);
    signal(SIGPIPE, SIG_IGN);
    signal(SIGCHLD, SIG_IGN);

    return 0;
}


/* ------------------------------------------------------------------------- */
/* Main                                                                      */
/* ------------------------------------------------------------------------- */

int main(int argc, char **argv)
{
    int ret;
    int option;

    struct rtsp_param param = {{{0}}};

    pc_prog_name = argv[0];

    register_signal();

    while ((option = getopt_long(
                argc,
                argv,
                "ha:b:c:d:e:f:g:i:j:k:l:m:n:o:p:q:r:",
                option_long,
                NULL)) != -1) {

        switch (option) {
        case 'h':
            help_hint();
            return 0;

        case 'a':
            if (parse_positive_int(
                    optarg,
                    &i_main_width) != 0)
                return -1;
            break;

        case 'b':
            if (parse_positive_int(
                    optarg,
                    &i_main_height) != 0)
                return -1;
            break;

        case 'c':
            if (parse_positive_int(
                    optarg,
                    &i_sub_width) != 0)
                return -1;
            break;

        case 'd':
            if (parse_positive_int(
                    optarg,
                    &i_sub_height) != 0)
                return -1;
            break;

        case 'e':
            if (parse_positive_int(
                    optarg,
                    &i_main_kbps) != 0)
                return -1;
            break;

        case 'f':
            if (parse_positive_int(
                    optarg,
                    &i_sub_kbps) != 0)
                return -1;
            break;

        case 'g':
            if (parse_positive_int(
                    optarg,
                    &i_main_fps) != 0)
                return -1;
            break;

        case 'i':
            if (parse_positive_int(
                    optarg,
                    &i_sub_fps) != 0)
                return -1;
            break;

        case 'j':
            if (parse_positive_int(
                    optarg,
                    &i_main_mode) != 0)
                return -1;
            break;

        case 'k':
            if (parse_positive_int(
                    optarg,
                    &i_sub_mode) != 0)
                return -1;
            break;

        case 'l':
            if (parse_positive_int(
                    optarg,
                    &i_main_type) != 0)
                return -1;
            break;

        case 'm':
            if (parse_positive_int(
                    optarg,
                    &i_sub_type) != 0)
                return -1;
            break;

        case 'n':
            pc_main_name = optarg;
            break;

        case 'o':
            pc_sub_name = optarg;
            break;

        case 'p':
            if (parse_positive_int(
                    optarg,
                    &i_gop) != 0)
                return -1;
            break;

        case 'q':
            if (parse_positive_int(
                    optarg,
                    &i_minqp) != 0)
                return -1;
            break;

        case 'r':
            if (parse_positive_int(
                    optarg,
                    &i_maxqp) != 0)
                return -1;
            break;

        default:
            help_hint();
            return -1;
        }
    }

    if (i_minqp > i_maxqp) {
        fprintf(stderr,
                "minqp must be <= maxqp\n");

        return -1;
    }

    fprintf(stderr,
            "[config] main=%dx%d %dfps %dkbps\n",
            i_main_width,
            i_main_height,
            i_main_fps,
            i_main_kbps);

    fprintf(stderr,
            "[config] sub=%dx%d %dfps %dkbps\n",
            i_sub_width,
            i_sub_height,
            i_sub_fps,
            i_sub_kbps);

    fprintf(stderr,
            "[config] qp=%d..%d gop=%d\n",
            i_minqp,
            i_maxqp,
            i_gop);

    fprintf(stderr,
            "[ir] ISP modes: DAY=%d NIGHT=%d\n",
            ISP_MODE_DAY,
            ISP_MODE_NIGHT);

    fprintf(stderr,
            "[ir] DAY -> NIGHT: gain >= %d\n",
            IR_NIGHT_GAIN);

    fprintf(stderr,
            "[ir] NIGHT probe: luma >= %d AND gain <= %d\n",
            IR_PROBE_DAY_LUM,
            IR_PROBE_DAY_GAIN);

    fprintf(stderr,
            "[ir] confirm=%d poll=%ds settle=%d "
            "night_hold=%d probe_settle=%d\n",
            IR_CONFIRM,
            IR_POLL_SEC,
            IR_SETTLE_POLLS,
            IR_NIGHT_HOLD_POLLS,
            IR_PROBE_SETTLE_POLLS);

    /*
     * This SDK exposes the PMEM initialization routine used by the original
     * application, but not akuio_pmem_exit(). Do not call a nonexistent
     * cleanup symbol during shutdown.
     */
    ret = akuio_pmem_init();

    if (ret != 0) {
        fprintf(stderr,
                "akuio_pmem_init failed: %d\n",
                ret);

        return -1;
    }

    vi_handle = ak_rtsp_vi_init();

    if (vi_handle == NULL) {
        fprintf(stderr,
                "VI initialization failed\n");

        return -1;
    }

    fprintf(stderr,
            "VI initialization OK handle=%p\n",
            vi_handle);

    ret = ak_vi_set_flip_mirror(
        vi_handle,
        1,
        1
    );

    if (ret != 0) {
        fprintf(stderr,
                "ak_vi_set_flip_mirror failed: %d\n",
                ret);
    }

    /*
     * The rtsp_param structure used by this Anyka SDK contains rtsp_chn[],
     * not video[].
     */
    param.rtsp_chn[0].current_channel =
        VIDEO_CHN_MAIN;

    param.rtsp_chn[0].width =
        i_main_width;

    param.rtsp_chn[0].height =
        i_main_height;

    param.rtsp_chn[0].fps =
        i_main_fps;

    param.rtsp_chn[0].max_kbps =
        i_main_kbps;

    param.rtsp_chn[0].min_qp =
        i_minqp;

    param.rtsp_chn[0].max_qp =
        i_maxqp;

    param.rtsp_chn[0].gop_len =
        i_gop;

    param.rtsp_chn[0].video_enc_type =
        i_main_type;

    param.rtsp_chn[0].video_br_mode =
        i_main_mode;

    param.rtsp_chn[0].vi_handle =
        vi_handle;

    strncpy(
        param.rtsp_chn[0].suffix_name,
        pc_main_name,
        sizeof(param.rtsp_chn[0].suffix_name) - 1
    );

    param.rtsp_chn[0].suffix_name[
        sizeof(param.rtsp_chn[0].suffix_name) - 1
    ] = '\0';

    param.rtsp_chn[1].current_channel =
        VIDEO_CHN_SUB;

    param.rtsp_chn[1].width =
        i_sub_width;

    param.rtsp_chn[1].height =
        i_sub_height;

    param.rtsp_chn[1].fps =
        i_sub_fps;

    param.rtsp_chn[1].max_kbps =
        i_sub_kbps;

    param.rtsp_chn[1].min_qp =
        i_minqp;

    param.rtsp_chn[1].max_qp =
        i_maxqp;

    param.rtsp_chn[1].gop_len =
        i_gop;

    param.rtsp_chn[1].video_enc_type =
        i_sub_type;

    param.rtsp_chn[1].video_br_mode =
        i_sub_mode;

    param.rtsp_chn[1].vi_handle =
        vi_handle;

    strncpy(
        param.rtsp_chn[1].suffix_name,
        pc_sub_name,
        sizeof(param.rtsp_chn[1].suffix_name) - 1
    );

    param.rtsp_chn[1].suffix_name[
        sizeof(param.rtsp_chn[1].suffix_name) - 1
    ] = '\0';

    ret = ak_rtsp_init(&param);

    if (ret != 0) {
        fprintf(stderr,
                "ak_rtsp_init failed: %d\n",
                ret);

        return -1;
    }

    fprintf(stderr,
            "RTSP initialization OK\n");

    ret = ak_rtsp_start(
        VIDEO_CHN_MAIN
    );

    if (ret != 0) {
        fprintf(stderr,
                "ak_rtsp_start(main) failed: %d\n",
                ret);

        ak_rtsp_exit();

        return -1;
    }

    ret = ak_rtsp_start(
        VIDEO_CHN_SUB
    );

    if (ret != 0) {
        fprintf(stderr,
                "ak_rtsp_start(sub) failed: %d\n",
                ret);

        ak_rtsp_stop(
            VIDEO_CHN_MAIN
        );

        ak_rtsp_exit();

        return -1;
    }

    fprintf(stderr,
            "RTSP main/sub started\n");

    if (ir_start() != 0) {
        fprintf(stderr,
                "[ir] failed to start automatic IR control\n");
    }

    if (web_start() != 0) {
        fprintf(stderr,
                "[web] failed to start UI; "
                "continuing without web UI\n");
    }

     if (snapshot_init() != 0)
        fprintf(stderr, "[snap] init failed; no live preview\n");

    run_flag = AK_TRUE;

    while (run_flag)
        ak_sleep_ms(1000);

    fprintf(stderr,
            "Shutting down...\n");

    web_stop();

    ir_stop();
    snapshot_exit();

    ak_rtsp_stop(
        VIDEO_CHN_MAIN
    );

    ak_rtsp_stop(
        VIDEO_CHN_SUB
    );

    ak_rtsp_exit();

    vi_handle = NULL;

    return 0;
}
