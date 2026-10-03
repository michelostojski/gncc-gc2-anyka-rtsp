# Engineering notes

Condensed findings from building this firmware — the non‑obvious bits.

## Day / night
- `ak_vpss_isp_get_auto_day_night_level()` is the only reliable detector on
  this unit. It **requires the full threshold arrays** via
  `ak_vpss_isp_set_auto_day_night_param()`:
  `night_cnt[0..4] = 15000`, `day_cnt[0..9] = 300000`,
  `day_to_night_lum = 3584`, `night_to_day_lum = 1536`, `lock_time = 900000`.
  Partial arrays → the level never changes.
- Gain and average luma are **frozen in night mode** (IR LED floods the
  sensor) — unusable for night→day. The LDR (`ak_drv_ir_get_input_level`)
  returns a constant. Don't rely on any of them.
- Night grayscale / no pink: `ak_vi_switch_mode(vi_handle, VI_MODE_NIGHT)`
  (and `VI_MODE_DAY` for day) loads the correct ISP profile.
- IR‑cut is a two‑wire latch: pulse A/B ~120 ms then release both to 0.
  Day = A0 B1, Night = A1 B0. IR‑LED via `/sys/user-gpio/ir-led` (on/off).

## Live preview (web UI)
- A persistent MJPEG encoder via `ak_venc_request_stream` **crashes** the ISP
  day/night call (FP exception). Use the one‑shot path instead:
  `ak_vi_get_frame` → `ak_venc_send_frame` → `free(vs.data)` →
  `ak_vi_release_frame`, per request.
- `ak_venc_send_frame` returns **0** on success — test `vs.data && vs.len`.

## Flash / SD‑free
- `/usr` (mtd6) squashfs is **XZ**, block 131072, squashfs 4.0.
  Rebuild with `mksquashfs ... -comp xz -b 131072`. (LZO is not supported by
  the on‑device kernel squashfs driver and will fail to mount.)
- Partition size limit: `app.sqsh` ≤ 0x3B6000 (3,891,200 bytes).
- Free space by removing `bin/anyka_ipc` (2.2 MB) and the stale `partC/`.
- Flash from U‑Boot with `sf erase/write` at offset 0x438000 (piotr‑go).

## Boot flow
- Stock `service.sh`: if `/mnt/wifitest` exists (SD inserted) it runs the SD
  test path; otherwise it runs `anyka_ipc.sh start` + `net_manage.sh`. So for
  SD‑free, the replaced `/usr/sbin/anyka_ipc.sh` is what launches the app.
