#!/bin/sh
CC=/home/ostingo/Téléchargements/arm-anykav200-crosstool/usr/bin/arm-anykav200-linux-uclibcgnueabi-gcc
HJ=/home/ostingo/Analyse_kernel/Anyka_ak3918_hacking_journey-main/cross-compile
AKUIO=/home/ostingo/qiwen/anycloud39ev300/SDK/platform/libplat/src/vi/include_vi
ISPINNER=/home/ostingo/qiwen/anycloud39ev300/SDK/platform/libplat/include_inner
LIBD1=$HJ/IOT-ANYKA-PTZdaemon/libs
LIBD2=/home/ostingo/anyka-backup/gc2/partC/lib
LIBD3=/home/ostingo/qiwen/anycloud39ev300/SDK/platform/libplat/lib

$CC -I$HJ/rtsp/include/libplat/include -I$HJ/rtsp/include/libmpi/include \
    -I$HJ/libre_anyka_app/include -I$AKUIO -I$ISPINNER \
    -Wall -g -O0 -std=gnu99 -pthread ak_rtsp_demo.c -o ak_rtsp_demo_v104 \
    -L$LIBD1 -L$LIBD2 -L$LIBD3 \
    -Wl,-rpath-link,$LIBD1 -Wl,-rpath-link,$LIBD2 -Wl,-rpath-link,$LIBD3 \
    -Wl,--start-group \
      -lplat_vi -lplat_vpss -lakispsdk -lakuio -lmpi_venc -lakv_encode \
      -lakmedialib -lplat_common -lplat_common_one -lplat_thread -lplat_drv -lplat_ai \
      -lplat_ipcsrv -lplat_venc_cb -lakae -lapp_rtsp -lapp_net \
      -lplat_mem -lplat_osal -lakaudiofilter \
    -Wl,--end-group -ldl -lrt -lpthread
