#ifndef _1WIRE_
#define _1WIRE_

#include "inc/hw_gpio.h"
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/sysctl.h"
#include "driverlib/adc.h"
#include "driverlib/timer.h"
#include "uartstdio.h"
#include "driverlib/uart.h"
#include <stdint.h>
#include <stdio.h>

//配置文件
#include "dsconfig.h"






//定义one-wire总线命令
#define OW_READ_ROM 0x33
#define OW_MATCH_ROM 0x55
#define OW_SKIP_ROM 0xCC
#define OW_SEARCH_ROM 0xF0
#define OW_ALARM_SEARCH 0xEC

#define OW_SEARCH_FIRST 0xFF
#define OW_PRESENCE_ERR 0xFF
#define OW_DATA_ERR 0xFE
#define OW_LAST_DEVICE 0x00

//rom-code size including CRC
#define OW_ROMCODE_SIZE 8

	//将端口设置为输入
#define OW_DIR_IN() (HWREG(OW_BUS_BASE+GPIO_O_DIR)&=~(1<<OW_BUS))
	//将端口设置为输出
#define OW_DIR_OUT GPIOPinTypeGPIOOutput(GPIO_PORTC_BASE,(1<<OW_BUS))
	//端口写1
#define OW_OUT_HIGH() (HWREG(OW_BUS_BASE+GPIO_O_DATA+(0xff<<2))|=(1<<OW_BUS))
	//端口写0
#define OW_OUT_LOW() (HWREG(OW_BUS_BASE+GPIO_O_DATA+(0xff<<2))&=~(1<<OW_BUS))
	//返回总线的状态
#define OW_GET_IN() (HWREG(OW_BUS_BASE+GPIO_O_DATA+(0xff<<2))&(1<<OW_BUS))

//onewire总线拉低
extern void ow_out_low(void);
//onewire总线拉高
extern void ow_out_high(void);
//onewire总线设置为输出
extern void ow_dir_out(void);
//onewire总线设置为输入
extern void ow_dir_in(void);
extern uint8_t ow_get_in(void);
extern uint8_t ow_bit_io(uint8_t b);
extern uint8_t ow_byte_wr(uint8_t b);
extern uint8_t ow_rom_search(uint8_t diff,uint8_t *id);
extern void ow_command(uint8_t command,const uint8_t *id);
extern uint8_t ow_reset(void);
extern uint8_t ow_byte_rd(void);
extern uint8_t ow_input_pin_state(void);
extern void ow_set_bus();

#endif