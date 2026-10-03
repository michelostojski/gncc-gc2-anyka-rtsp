# GNCC GC2 (Anyka AK3918) — Local RTSP Camera Firmware

Custom, cloud-free firmware for the **GNCC GC2** indoor camera
(Anyka **AK3918EV**, JXF37 sensor, RTL8188FU Wi‑Fi). Built from the
vendor SDK demo, it runs **entirely from internal flash — no SD card
required** — and provides:

- **RTSP** streaming — main `1280x720` (`/vs0`) + sub `640x480` (`/vs1`)
- **Automatic day/night** via the ISP's native auto‑switch (no pink cast,
  no oscillation, works indoors where ambient light changes at any hour)
- Clean **grayscale night** vision (IR‑cut + IR LED + ISP night profile)
- A **web UI** on port 80 — live JPEG preview, day/night control, status,
  stream links — a dark, single‑page layout inspired by [Thingino](https://thingino.com/)
- **Wi‑Fi** (stock association flow, credentials on flash)
- **telnet** for maintenance, front **LED off** (no cloud‑connect blink)

> No cloud. No app. No SD card. Everything local, on flash.


---

## Hardware

| | |
|---|---|
| SoC | Anyka AK3918EV (ARM926EJ‑S), kernel 4.4.x, uClibc 0.9.33 |
| Sensor | SOI **JXF37** (`f37_mipi`, 1‑lane MIPI) |
| Wi‑Fi | Realtek **RTL8188FU** (USB, via OTG host) |
| Flash | 8 MB SPI‑NOR |
| IR‑cut | `/sys/user-gpio/gpio-ircut_a` + `gpio-ircut_b` (two‑wire latch) |
| IR‑LED | `/sys/user-gpio/ir-led` (on/off) |

### Flash layout (`/proc/mtd`)

| mtd | offset | size | name | contents |
|----|--------|------|------|----------|
| mtd1 | — | 0x200000 | KERNEL | |
| mtd4 | 0x23b000 | 0x180000 | A | rootfs (squashfs) |
| mtd5 | — | 0x7d000 | B | `/etc/jffs2` (rw config) |
| mtd6 | 0x438000 | 0x3b6000 | C | `/usr` (squashfs, XZ) ← this firmware |

---

## What was hard (and how it was solved)

This firmware is the result of reverse‑engineering several non‑obvious
behaviours of the Anyka SDK. The notes below may save others a lot of time.

### 1. Day/night — use the ISP's own auto switch, with FULL arrays

Gain, average luma, the "dark flag", and the photo‑resistor
(`ak_drv_ir_get_input_level`) are all **unreliable** on this unit:

- In night mode the IR LED floods the scene, so **gain and luma are frozen**
  and can't tell day from night.
- The LDR returns a constant value.

The reliable method is the SDK's built‑in ISP auto day/night, **but it only
works if the threshold arrays are filled completely** (the detail that made
it finally work, from piotr‑go's app):

```c
struct ak_auto_day_night_threshold th;
memset(&th, 0, sizeof(th));
th.day_to_night_lum = 3584;     /* vendor config values   */
th.night_to_day_lum = 1536;
th.lock_time        = 900000;
for (i = 0; i < 5;  i++) th.night_cnt[i] = 15000;   /* ALL 5  */
for (i = 0; i < 10; i++) th.day_cnt[i]   = 300000;  /* ALL 10 */
ak_vpss_isp_set_auto_day_night_param(&th);

/* each poll: 1 = day, 0 = night, -1 = fail */
int level = ak_vpss_isp_get_auto_day_night_level(cur_is_day ? 1 : 0);
```

Partially‑filled arrays silently fail (the level never changes). With the
full arrays the ISP switches cleanly in both directions, no probe, no pink.

### 2. Night pink → `ak_vi_switch_mode(VI_MODE_NIGHT)`

Zeroing ISP effect/saturation did **not** remove the magenta IR cast.
The fix is the VI day/night mode switch, which loads the proper night ISP
profile (grayscale): `ak_vi_switch_mode(vi_handle, VI_MODE_NIGHT)` for night
and `VI_MODE_DAY` for day.

### 3. Web‑UI live preview — `ak_venc_send_frame`, not `request_stream`

A persistent MJPEG encoder opened with `ak_venc_request_stream` on the VI
**crashes** `ak_vpss_isp_get_auto_day_night_level` (floating‑point
exception). The vendor's method is a one‑shot encode that doesn't bind a
continuous capture:

```c
/* once: open an MJPEG encoder, do NOT request_stream */
jpeg = ak_venc_open(&ep /* enc_out_type = MJPEG_ENC_TYPE */);

/* per /snapshot.jpg request: */
ak_vi_get_frame(vi_handle, &vif);
ak_venc_send_frame(jpeg, vif.vi_frame[SUB].data,
                   vif.vi_frame[SUB].len, &vs);   /* returns 0 on success */
/* send vs.data (vs.len) as image/jpeg, then free(vs.data) */
ak_vi_release_frame(vi_handle, &vif);
```

Note `ak_venc_send_frame` returns **0** on success (not the length) —
check `vs.data && vs.len`, not the return value.

### 4. SD‑free install — rebuild `/usr`, flash from U‑Boot

`/usr` (mtd6) is a **XZ** squashfs (not LZO — a byte in the superblock was
misread early on and cost a bricked boot + recovery). To fit the app + its
extra libs in the 3.6 MB partition, remove `bin/anyka_ipc` (2.2 MB) and the
stale `partC/` duplicate, then repack with XZ and flash from U‑Boot.

---

## Build

Toolchain: `arm-anykav200-linux-uclibcgnueabi` (uClibc 0.9.33, `.so.0`).

```sh
cd src
sh build.sh          # produces ak_rtsp_demo_v104
```

The demo links the Anyka SDK libs. Four are not in the stock `/usr/lib`
and must ship with the firmware: `libakae.so`, `libplat_common_one.so`,
`libplat_mem.so`, `libplat_osal.so`.

---

## SD‑free install (overwrites `/usr`)

> Requires **serial console** (UART, 115200) for U‑Boot access and recovery.
> **Back up the original `/usr` first** — this is your rescue image.

### 1. Back up the stock `/usr`
```sh
# on the camera
cat /dev/mtdblock6 > /mnt/app_current.bin      # 3,891,200 bytes, keep safe!
```

### 2. Build the new `/usr` image (on a PC)
```sh
unsquashfs -d usr_new app_current.bin
cd usr_new
rm -f bin/anyka_ipc bin/ak_*_demo              # free space
rm -rf partC                                   # stale duplicate (~7.8 MB)
cp  .../ak_rtsp_demo_v104 bin/                 # the app
cp  .../libakae.so .../libplat_common_one.so \
    .../libplat_mem.so .../libplat_osal.so lib/
cp  .../sbin/anyka_ipc.sh sbin/                # boot script (see scripts/)
# stop the cloud LED blink:
sed -i 's|/usr/sbin/red_led.sh blink|#&|g; s|/usr/sbin/blue_led.sh blink|#&|g' sbin/wifi_run.sh
cd ..
mksquashfs usr_new app.sqsh -comp xz -b 131072 -noappend -all-root
ls -l app.sqsh                                 # MUST be <= 3,891,200 bytes
```

### 3. Put your Wi‑Fi credentials on flash
Edit `/etc/jffs2/wpa_supplicant.conf` on the camera with your SSID/PSK
(and/or the `[wireless]` section of `/etc/jffs2/anyka_cfg.ini`, which the
stock association flow reads).

### 4. Flash from U‑Boot (piotr‑go's method)
Copy `app.sqsh` to the SD's FAT partition, reboot, interrupt to U‑Boot:
```
fatload mmc 0:1 0x82000000 app.sqsh
sf probe 0
sf erase 438000 3B6000
sf write 0x82000000 438000 3B6000
reset
```

### Recovery
If boot fails, re‑flash the backup the same way:
```
fatload mmc 0:1 0x82000000 app_current.bin
sf probe 0 ; sf erase 438000 3B6000 ; sf write 0x82000000 438000 3B6000 ; reset
```

---

## Usage

- Web UI: `http://<camera-ip>/`
- RTSP main: `rtsp://<camera-ip>:554/vs0` · sub: `rtsp://<camera-ip>:554/vs1`
- Snapshot: `http://<camera-ip>/snapshot.jpg`
- Day/Night/Auto: buttons in the UI, or `GET /set?mode=day|night|auto`
- Maintenance: `telnet <camera-ip>`  *(open shell — trusted LAN only)*

Works out of the box with VLC, ffmpeg, go2rtc and Frigate.

---

## Credits

- **[Thingino](https://thingino.com/)** — the open‑source IP‑camera firmware
  whose web UI inspired this one. Thingino targets Ingenic SoCs (not Anyka),
  but its clean, single‑page preview‑and‑control layout is the model the web
  UI here follows.
- **piotr‑go** — [GNCC_GC2_ak3918ev300_RTSP](https://github.com/piotr-go/GNCC_GC2_ak3918ev300_RTSP):
  the U‑Boot `sf` flash procedure and the complete‑array day/night setup.
- The Anyka hacking community (TECKIN/GNCC/Nooie AK3918 work).

## Disclaimer

Flashing SPI‑NOR can brick the device. You need serial‑console recovery.
Provided as‑is, for your own hardware, at your own risk. Not affiliated with
GNCC or Anyka.
