#这是一个为LM3S系列Cortex-M3单片机编写的Makefile配置文件
#Auther:mybays@qq.com
#Date:Nov-18-2012

#单片机的型号
PART=lm3s8962


#需要编译的C源文件
SRC = $(TARGET).c
SRC += startup_${PART}.c
SRC += mydelay.c
SRC += ds18b20.c
SRC += uartstdio.c
SRC += rit128x96x4.c
SRC += ustdlib.c
SRC += crc8.c
SRC += 1wire.c
#额外的CFLAG
#CFLAGSgcc=-DUART_BUFFERED -DTARGET_IS_DUSTDEVIL_RA0
#需要编译的C++源文件
CPPSRC = 
CPPSRC += 

#需要编译的汇编源文件
ASRC = 
ASRC += 

#开发环境的根目录
ROOT=../../..

include ${ROOT}/myproject/templates/makedefs
