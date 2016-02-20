#ifndef _CRC8_
#define _CRC8_


#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include <stdint.h>
#include <stdio.h>

#define CRC8INIT 0x00
#define CRC8POLY 0x18
//CRC8校验部分
extern uint8_t crc8(uint8_t* data_in,uint16_t number_of_bytes_to_read);

#endif