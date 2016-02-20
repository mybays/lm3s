#ifndef _DS18B20_
#define _DS18B20_
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
//定义单线总线要使用的GPIO,PC7
#define OW_BUS_BASE GPIO_PORTC_BASE
#define OW_BUS GPIO_PIN7
#define GPIO_PIN7 7

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
#define OW_DIR_IN() (HWREG(OW_BUS_BASE+GPIO_O_DIR)&=~(0x80))
	//将端口设置为输出
#define OW_DIR_OUT GPIOPinTypeGPIOOutput(GPIO_PORTC_BASE,(1<<OW_BUS))
	//端口写1
#define OW_OUT_HIGH() (HWREG(OW_BUS_BASE+GPIO_O_DATA+(0xff<<2))|=(1<<OW_BUS))
	//端口写0
#define OW_OUT_LOW() (HWREG(OW_BUS_BASE+GPIO_O_DATA+(0xff<<2))&=~(0x80))
	//返回总线的状态
#define OW_GET_IN() (HWREG(OW_BUS_BASE+GPIO_O_DATA+(0xff<<2))&(1<<OW_BUS))
#define DS18S20_ID 0x10				//DS18S20的主标识号
#define DS18B20_ID 0x28				//DS18B20的主标识号


#define DS18X20_CONVERT_T	0x44		//开始温度转换
#define DS18X20_OK          0x00
#define DS18X20_START_FAIL  0x02
#define DS18X20_SP_SIZE  9
#define DS18X20_READ		0xBE		//读取暂存器和CRC字节,读数据直到9字节
#define DS18X20_WRITE		0x4E		//写两个字节到暂存器,地址为2和3
#define DS18X20_COPY		0x48		//把暂存器的内容拷贝到非易失性存储器中
#define DS18X20_RECALL		0xB8	//把非易失性存储器中的值读取到暂存器
#define DS18X20_POWER		0xB4	//标识DS18B20的供电模式,0为寄生电源,1为外部电源
#define DS18X20_ERROR_CRC   0x03
#define DS18B20_CONF_REG    4
#define DS18B20_9_BIT       0
#define DS18B20_10_BIT      (1<<5)
#define DS18B20_11_BIT      (1<<6)
#define DS18B20_12_BIT      ((1<<6)|(1<<5))
// indefined bits in LSB if 18B20 != 12bit
#define DS18B20_9_BIT_UNDF       ((1<<0)|(1<<1)|(1<<2))
#define DS18B20_10_BIT_UNDF      ((1<<0)|(1<<1))
#define DS18B20_11_BIT_UNDF      ((1<<0))
#define DS18B20_12_BIT_UNDF      0
#define DS18B20_TCONV_12BIT      750
#define DS18X20_FRACCONV         625

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
extern void ow_command(uint8_t command,uint8_t *id);
extern void DS18X20_find_sensor(uint8_t *diff,uint8_t id[]);

extern uint8_t DS18X20_start_meas();
extern uint8_t DS18X20_read_meas(uint8_t id[],uint8_t *subzero,uint8_t *cel,uint16_t *cel_frac_bits);
extern uint8_t DS18X20_meas_to_cel(uint8_t fc,uint8_t *sp,uint8_t* subzero,uint8_t* cel,uint16_t* cel_frac_bits);
extern uint8_t ow_reset(void);


extern uint8_t ow_byte_rd(void);


extern uint8_t ow_input_pin_state(void);

extern void ow_set_bus();

//CRC8校验部分
extern uint8_t crc8(uint8_t* data_in,uint16_t number_of_bytes_to_read);

uint8_t DS18X20_get_temp(uint8_t id[],uint8_t *subzero,uint8_t *cel,uint16_t *cel_frac_bits);

#endif

