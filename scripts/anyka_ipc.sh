#! /bin/sh
### BEGIN INIT INFO
# File:             camera.sh
# Provides:         RTSP camera + WiFi + telnet + LED off
### END INIT INFO

MODE=$1
PATH=$PATH:/bin:/sbin:/usr/bin:/usr/sbin

usage() { echo "Usage: $0 start|stop"; exit 3; }

start_wifi()
{
	if ifconfig wlan0 2>/dev/null | grep -q "inet addr:"; then
		echo "wifi already up"
		return 0
	fi
	echo "loading wifi driver (RTL8188FU)..."
	insmod /usr/modules/sdio_wifi.ko 2>/dev/null; sleep 2
	insmod /usr/modules/otg-hs.ko   2>/dev/null; sleep 2
	insmod /usr/modules/8188fu.ko   2>/dev/null; sleep 3
	ifconfig wlan0 up 2>/dev/null
	killall wpa_supplicant 2>/dev/null; sleep 1
	wpa_supplicant -B -iwlan0 -Dwext -c /etc/jffs2/wpa_supplicant.conf
	sleep 6
	killall udhcpc 2>/dev/null
	udhcpc -i wlan0 -t 10 -n &
	i=0
	while [ $i -lt 15 ]; do
		ifconfig wlan0 | grep -q "inet addr:" && break
		sleep 1; i=`expr $i + 1`
	done
}

led_off()
{
	echo none > /sys/class/leds/red_led/trigger 2>/dev/null
	echo none > /sys/class/leds/blue_led/trigger 2>/dev/null
	echo 0 > /sys/class/leds/red_led/brightness 2>/dev/null
	echo 0 > /sys/class/leds/blue_led/brightness 2>/dev/null
	echo 0 > /sys/class/leds/system_state_led/brightness 2>/dev/null
}

stop()
{
	killall -15 ak_rtsp_demo_v104 2>/dev/null
	echo "camera stopped"
}

start()
{
	echo "start camera service......"

	start_wifi

	# IR-cut day pulse (clear pink)
	echo 0 > /sys/user-gpio/gpio-ircut_a 2>/dev/null
	echo 1 > /sys/user-gpio/gpio-ircut_b 2>/dev/null
	sleep 1
	echo 0 > /sys/user-gpio/gpio-ircut_a 2>/dev/null
	echo 0 > /sys/user-gpio/gpio-ircut_b 2>/dev/null

	# maintenance shell
	busybox telnetd -l /bin/sh -b 0.0.0.0 2>/dev/null &
	echo "telnetd started"

	pid=`pgrep ak_rtsp_demo_v104`
	if [ "$pid" = "" ]; then
		export LD_LIBRARY_PATH=/usr/lib:/lib
		cd /usr/bin
		./ak_rtsp_demo_v104 > /tmp/rtsp.log 2>&1 &
		echo "ak_rtsp_demo_v104 started"
	fi

	# front LED off (local-only, no cloud) - after a delay so it sticks
	sleep 3
	led_off
	# keep it off (in case something re-asserts it)
	( sleep 15; led_off ) &
}

case "$MODE" in
	start)   start ;;
	stop)    stop  ;;
	restart) stop; start ;;
	*)       usage ;;
esac
exit 0
