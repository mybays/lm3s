#这是一个为LM3S系列Cortex-M3单片机编写的Makefile配置文件
#Auther:mybays@qq.com
#Date:Nov-18-2012


#单片机的型号
PART=lm3s9b96

#单片机类型,cortex-m3架构
VARIANT=cm3

#开发环境的根目录
ROOT=../../..

#编译源文件时的搜索路径,VPATH是Makefile里有效文件路径
VPATH = ./
VPATH += ${ROOT}/boot_loader
VPATH += ${ROOT}/driverlib
VPATH += ${ROOT}/myproject
VPATH += ${ROOT}/myproject/drivers
VPATH += ${ROOT}/myproject/drivers/${PART}
VPATH += ${ROOT}/grlib
VPATH += ${ROOT}/usblib
VPATH += ${ROOT}/utils
VPATH += ${ROOT}/myproject/templates
VPATH += ${ROOT}/third_party/uip-1.0
VPATH += ${ROOT}/third_party/uip-1.0/apps/
VPATH += ${ROOT}/third_party/uip-1.0/apps/httpd

VPATH += ${ROOT}/boards/dk-${PART}/
VPATH += ${ROOT}/boards/dk-${PART}/drivers/


#需要编译的C源文件
SRC = $(TARGET).c
SRC += startup_${PART}.c
SRC += httpd.c
SRC += kitronix320x240x16_ssd2119_8bit.c
SRC += set_pinout.c
SRC += uartstdio.c
SRC += uip.c
SRC += uip_arp.c
SRC += uip_timer.c
SRC += ustdlib.c

#额外的CFLAG
CFLAGSgcc=-DUART_BUFFERED -DTARGET_IS_DUSTDEVIL_RA0
#需要编译的C++源文件
CPPSRC = 
CPPSRC += 

#需要编译的汇编源文件
ASRC = 
ASRC += 

#include要搜索的文件夹
IPATH = ./
IPATH += ${ROOT}
IPATH += ${ROOT}/boot_loader
IPATH += ${ROOT}/driverlib
IPATH += ${ROOT}/myproject
IPATH += ${ROOT}/myproject/drivers
IPATH += ${ROOT}/myproject/drivers/${PART}
IPATH += ${ROOT}/grlib
IPATH += ${ROOT}/inc
IPATH += ${ROOT}/IQmath
IPATH += ${ROOT}/usblib
IPATH += ${ROOT}/utils
IPATH += ${ROOT}/third_party/uip-1.0
IPATH += ${ROOT}/third_party/uip-1.0/uip
IPATH += ${ROOT}/third_party/uip-1.0/apps/
IPATH += ${ROOT}/third_party/uip-1.0/apps/httpd
IPATH += ${ROOT}/boards/dk-${PART}/
IPATH += ${ROOT}/boards/dk-${PART}/drivers
#复位向量表
ENTRY=ResetHandler

#打包库文件路径
EXTRA_LIBDIRS = ${ROOT}/myproject/lib
#额外的库文件
EXTRA_LIBS =
#静态库文件
LIBA = ${EXTRA_LIBDIRS}/libdriver-cm3.a
#LIBA += ${EXTRA_LIBDIRS}/libusb.a


include ${ROOT}/myproject/templates/makedefs
