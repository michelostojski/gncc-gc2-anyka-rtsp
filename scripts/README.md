# scripts

## `anyka_ipc.sh`
Boot script placed at `/usr/sbin/anyka_ipc.sh` inside the repacked `/usr`
squashfs. It replaces the stock cloud app (`anyka_ipc`) launch and instead:

1. brings up Wi‑Fi (RTL8188FU: `sdio_wifi` → `otg-hs` → `8188fu`,
   then `wpa_supplicant` with `/etc/jffs2/wpa_supplicant.conf` + DHCP),
2. pulses the IR‑cut to day,
3. starts `telnetd` (open shell — trusted LAN only),
4. launches `/usr/bin/ak_rtsp_demo_v104`,
5. turns the front LED off (no cloud‑connect blink).

The stock association flow (`wifi_run.sh` / `wifi_station.sh`) still runs and
reads credentials from `/etc/jffs2/anyka_cfg.ini` `[wireless]`; the explicit
Wi‑Fi here is belt‑and‑suspenders.

## `wifi_run.sh` patch
The stock `wifi_run.sh` blinks the front LED while trying to reach the cloud
(which no longer exists). Comment out its blink calls before repacking:

```sh
sed -i 's|/usr/sbin/red_led.sh blink|#&|g; \
        s|/usr/sbin/blue_led.sh blink|#&|g' usr_new/sbin/wifi_run.sh
```
