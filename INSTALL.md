# Install — SD‑free firmware on the GNCC GC2 (Anyka AK3918)

> **You need a serial console (UART, 115200 baud)** connected to the camera's
> debug pads for U‑Boot access and recovery. Flashing SPI‑NOR can brick the
> device; without serial you cannot recover.

## 0. Prerequisites
- UART‑to‑USB adapter wired to the camera's TX/RX/GND pads
- `squashfs-tools` on your PC (`unsquashfs`, `mksquashfs`)
- An SD card (FAT) — used only to transfer the image, not at runtime
- The built `ak_rtsp_demo_v104` and the 4 extra libs (see `src/README.md`)

## 1. Back up the stock `/usr` (your rescue image)
On the camera (serial or telnet shell):
```sh
cat /dev/mtdblock6 > /mnt/app_current.bin      # 3,891,200 bytes
```
Copy `app_current.bin` to your PC and keep it safe.

## 2. Build the new `/usr` image (PC)
```sh
unsquashfs -d usr_new app_current.bin
cd usr_new

# free space
rm -f  bin/anyka_ipc            # 2.2 MB cloud app, not needed
rm -f  bin/ak_*_demo            # SDK demos, not needed
rm -rf partC                    # stale ~7.8 MB duplicate of /usr

# add the app + its extra libs
cp /path/to/ak_rtsp_demo_v104 bin/ ; chmod +x bin/ak_rtsp_demo_v104
cp /path/to/libakae.so /path/to/libplat_common_one.so \
   /path/to/libplat_mem.so /path/to/libplat_osal.so lib/

# boot script
cp /path/to/scripts/anyka_ipc.sh sbin/ ; chmod +x sbin/anyka_ipc.sh

# stop the cloud LED blink
sed -i 's|/usr/sbin/red_led.sh blink|#&|g; \
        s|/usr/sbin/blue_led.sh blink|#&|g' sbin/wifi_run.sh

cd ..
# IMPORTANT: compression is XZ (the stock /usr is XZ, not LZO)
mksquashfs usr_new app.sqsh -comp xz -b 131072 -noappend -all-root
ls -l app.sqsh                  # MUST be <= 3,891,200 bytes (0x3B6000)
```

## 3. Wi‑Fi credentials (on flash, persist)
On the camera, set your network in `/etc/jffs2/wpa_supplicant.conf`:
```
ctrl_interface=/var/run/wpa_supplicant
update_config=1
network={
    ssid="YOUR_SSID"
    psk="YOUR_PASSWORD"
    scan_ssid=1
}
```
(The stock association flow also reads the `[wireless]` section of
`/etc/jffs2/anyka_cfg.ini` — set it there too if needed.)

## 4. Flash from U‑Boot
Copy `app.sqsh` to the SD's FAT partition. Reboot and hit a key to stop at
the U‑Boot prompt, then:
```
fatload mmc 0:1 0x82000000 app.sqsh
sf probe 0
sf erase 438000 3B6000
sf write 0x82000000 438000 3B6000
reset
```

## 5. Verify
Boot **with** the SD still in to check it comes up, then **remove the SD**
and reboot to confirm SD‑free operation:
- `http://<ip>/` shows the UI + live preview
- `rtsp://<ip>:554/vs0` streams in VLC/ffmpeg/Frigate
- `telnet <ip>` gives a shell
- front LED is off

## Recovery
If the new `/usr` fails to boot, re‑flash the backup from U‑Boot:
```
fatload mmc 0:1 0x82000000 app_current.bin
sf probe 0
sf erase 438000 3B6000
sf write 0x82000000 438000 3B6000
reset
```

## Offsets reference (`/proc/mtd`)
| partition | offset | size |
|-----------|--------|------|
| A (rootfs)| 0x23b000 | 0x180000 |
| C (/usr)  | 0x438000 | 0x3b6000 |
