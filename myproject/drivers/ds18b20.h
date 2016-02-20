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


extern void DS18X20_find_sensor(uint8_t *diff,uint8_t id[]);
extern uint8_t DS18X20_start_meas();
extern uint8_t DS18X20_read_meas(uint8_t id[],uint8_t *subzero,uint8_t *cel,uint16_t *cel_frac_bits);
extern uint8_t DS18X20_meas_to_cel(uint8_t fc,uint8_t *sp,uint8_t* subzero,uint8_t* cel,uint16_t* cel_frac_bits);
uint8_t DS18X20_get_temp(uint8_t id[],uint8_t *subzero,uint8_t *cel,uint16_t *cel_frac_bits);
uint8_t search_sensors(uint64_t gid[],uint8_t maxnum);
#endif

