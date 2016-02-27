#这是一个为LM3S系列Cortex-M3单片机编写的Makefile配置文件
#Auther:mybays@qq.com
#Date:Nov-18-2012

#单片机的型号
PART=LM3S8962


#需要编译的C源文件
SRC = $(TARGET).c
SRC += startup_${PART}.c
SRC += mydelay.c
SRC += ustdlib.c
SRC += oled.c
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

#编译源文件时的搜索路径,VPATH是Makefile里有效文件路径
VPATH = ./

#定义CFLAGS
CFLAGS = -mthumb


include ${ROOT}/myproject/templates/makedefs
