/* nRF24L01.h
 * Register definitions for manipulating the Nordic Semiconductor
 * nRF24L01+ RF transceiver chipsets.
 *

    Copyright (c) 2007 Stefan Engelke <mbox@stefanengelke.de>
    Some parts copyright (c) 2012 Eric Brundick <spirilis [at] linux dot com>

    Permission is hereby granted, free of charge, to any person 
    obtaining a copy of this software and associated documentation 
    files (the "Software"), to deal in the Software without 
    restriction, including without limitation the rights to use, copy, 
    modify, merge, publish, distribute, sublicense, and/or sell copies 
    of the Software, and to permit persons to whom the Software is 
    furnished to do so, subject to the following conditions:

    The above copyright notice and this permission notice shall be 
    included in all copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, 
    EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF 
    MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND 
    NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT 
    HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, 
    WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, 
    OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER 
    DEALINGS IN THE SOFTWARE.
 */
#ifndef _NRF24L01_H
#define _NRF24L01_H

#include "inc/hw_types.h"
#include <inttypes.h>
#include <stdbool.h>

/* Register Map */
#define RF24_CONFIG      0x00
#define RF24_EN_AA       0x01
#define RF24_EN_RXADDR   0x02
#define RF24_SETUP_AW    0x03
#define RF24_SETUP_RETR  0x04
#define RF24_RF_CH       0x05
#define RF24_RF_SETUP    0x06
#define RF24_STATUS      0x07
#define RF24_OBSERVE_TX  0x08
#define RF24_CD          0x09
#define RF24_RPD         0x09
#define RF24_RX_ADDR_P0  0x0A
#define RF24_RX_ADDR_P1  0x0B
#define RF24_RX_ADDR_P2  0x0C
#define RF24_RX_ADDR_P3  0x0D
#define RF24_RX_ADDR_P4  0x0E
#define RF24_RX_ADDR_P5  0x0F
#define RF24_TX_ADDR     0x10
#define RF24_RX_PW_P0    0x11
#define RF24_RX_PW_P1    0x12
#define RF24_RX_PW_P2    0x13
#define RF24_RX_PW_P3    0x14
#define RF24_RX_PW_P4    0x15
#define RF24_RX_PW_P5    0x16
#define RF24_FIFO_STATUS 0x17
#define RF24_DYNPD       0x1C
#define RF24_FEATURE     0x1D

/* Register Bits */
#define RF24_MASK_RX_DR  BIT6
#define RF24_MASK_TX_DS  BIT5
#define RF24_MASK_MAX_RT BIT4
#define RF24_EN_CRC      BIT3
#define RF24_CRCO        BIT2
#define RF24_PWR_UP      BIT1
#define RF24_PRIM_RX     BIT0
#define RF24_ENAA_P5     BIT5
#define RF24_ENAA_P4     BIT4
#define RF24_ENAA_P3     BIT3
#define RF24_ENAA_P2     BIT2
#define RF24_ENAA_P1     BIT1
#define RF24_ENAA_P0     BIT0
#define RF24_ERX_P5      BIT5
#define RF24_ERX_P4      BIT4
#define RF24_ERX_P3      BIT3
#define RF24_ERX_P2      BIT2
#define RF24_ERX_P1      BIT1
#define RF24_ERX_P0      BIT0
#define RF24_AW          BIT0
#define RF24_ARD         BIT4
#define RF24_ARC         BIT0
#define RF24_PLL_LOCK    BIT4
#define RF24_CONT_WAVE   BIT7
#define RF24_RF_DR       BIT3
#define RF24_RF_DR_LOW   BIT5
#define RF24_RF_DR_HIGH  BIT3
#define RF24_RF_PWR      BIT1
#define RF24_LNA_HCURR   BIT0
#define RF24_RX_DR       BIT6
#define RF24_TX_DS       BIT5
#define RF24_MAX_RT      BIT4
#define RF24_RX_P_NO     BIT1
#define RF24_TX_FULL     BIT0
#define RF24_PLOS_CNT    BIT4
#define RF24_ARC_CNT     BIT0
#define RF24_TX_REUSE    BIT6
#define RF24_FIFO_FULL   BIT5
#define RF24_TX_EMPTY    BIT4
#define RF24_RX_FULL     BIT1
#define RF24_RX_EMPTY    BIT0
#define RF24_EN_DPL      BIT2
#define RF24_EN_ACK_PAY  BIT1
#define RF24_EN_DYN_ACK  BIT0

/* Instructions */
#define RF24_R_REGISTER    0x00
#define RF24_W_REGISTER    0x20
#define RF24_REGISTER_MASK 0x1F
#define RF24_R_RX_PAYLOAD  0x61
#define RF24_W_TX_PAYLOAD  0xA0
#define RF24_FLUSH_TX      0xE1
#define RF24_FLUSH_RX      0xE2
#define RF24_REUSE_TX_PL   0xE3
#define RF24_R_RX_PL_WID   0x60
#define RF24_W_ACK_PAYLOAD 0xA8
#define RF24_W_TX_PAYLOAD_NOACK 0xB0
#define RF24_NOP           0xFF






//****************************************************************//
// SPI(nRF24L01) commands
#define R_REGISTER      0x00
#define W_REGISTER      0x20
#define REGISTER_MASK   0x1F
#define READ_REG        0x00  // Define read command to register
#define WRITE_REG       0x20  // Define write command to register
#define RD_RX_PLOAD     0x61  // Define RX payload register address
#define WR_TX_PLOAD     0xA0  // Define TX payload register address
#define FLUSH_TX        0xE1  // Define flush TX register command
#define FLUSH_RX        0xE2  // Define flush RX register command
#define REUSE_TX_PL     0xE3  // Define reuse TX payload register command
#define NOP             0xFF  // Define No Operation, might be used to read status register

//***************************************************//
// SPI(nRF24L01) registers(addresses)
#define CONFIG          0x00  // 'Config' register address
#define NRF_CONFIG          0x00  // 'Config' register address
#define EN_AA           0x01  // 'Enable Auto Acknowledgment' register address
#define EN_RXADDR       0x02  // 'Enabled RX addresses' register address
#define SETUP_AW        0x03  // 'Setup address width' register address
#define SETUP_RETR      0x04  // 'Setup Auto. Retrans' register address
#define RF_CH           0x05  // 'RF channel' register address
#define RF_SETUP        0x06  // 'RF setup' register address
#define STATUS          0x07  // 'Status' register address
#define OBSERVE_TX      0x08  // 'Observe TX' register address
#define CD              0x09  // 'Carrier Detect' register address
#define RX_ADDR_P0      0x0A  // 'RX address pipe0' register address
#define RX_ADDR_P1      0x0B  // 'RX address pipe1' register address
#define RX_ADDR_P2      0x0C  // 'RX address pipe2' register address
#define RX_ADDR_P3      0x0D  // 'RX address pipe3' register address
#define RX_ADDR_P4      0x0E  // 'RX address pipe4' register address
#define RX_ADDR_P5      0x0F  // 'RX address pipe5' register address
#define TX_ADDR         0x10  // 'TX address' register address
#define RX_PW_P0        0x11  // 'RX payload width, pipe0' register address
#define RX_PW_P1        0x12  // 'RX payload width, pipe1' register address
#define RX_PW_P2        0x13  // 'RX payload width, pipe2' register address
#define RX_PW_P3        0x14  // 'RX payload width, pipe3' register address
#define RX_PW_P4        0x15  // 'RX payload width, pipe4' register address
#define RX_PW_P5        0x16  // 'RX payload width, pipe5' register address
#define FIFO_STATUS     0x17  // 'FIFO Status Register' register address
#define DYNPD       0x1C

/* Bit Mnemonics */
#define MASK_RX_DR  6
#define MASK_TX_DS  5
#define MASK_MAX_RT 4
#define EN_CRC      3
#define CRCO        2
#define PWR_UP      1
#define PRIM_RX     0
#define ENAA_P5     5
#define ENAA_P4     4
#define ENAA_P3     3
#define ENAA_P2     2
#define ENAA_P1     1
#define ENAA_P0     0
#define ERX_P5      5
#define ERX_P4      4
#define ERX_P3      3
#define ERX_P2      2
#define ERX_P1      1
#define ERX_P0      0
#define AW          0
#define ARD         4
#define ARC         0
#define PLL_LOCK    4
#define RF_DR       3
#define RF_PWR      6
#define RX_DR       6
#define TX_DS       5
#define MAX_RT      4
#define RX_P_NO     1
#define TX_FULL     0
#define PLOS_CNT    4
#define ARC_CNT     0
#define TX_REUSE    6
#define FIFO_FULL   5
#define TX_EMPTY    4
#define RX_FULL     1
#define RX_EMPTY    0
#define DPL_P5      5
#define DPL_P4      4
#define DPL_P3      3
#define DPL_P2      2
#define DPL_P1      1
#define DPL_P0      0
#define EN_DPL      2
#define EN_ACK_PAY  1
#define EN_DYN_ACK  0

/* P model bit Mnemonics */
#define RF_DR_LOW   5
#define RF_DR_HIGH  3
#define RF_PWR_LOW  1
#define RF_PWR_HIGH 2


#define RX_PLOAD_WIDTH  32      // 20 uints TX payload
#define TX_PLOAD_WIDTH  32      // 20 uints TX payload


#define _BV(x) (1<<(x))

extern const unsigned char TX_ADDRESS[]; // Define a static TX address ;
extern const unsigned char RX_ADDRESS[]; // Define
typedef enum { RF24_1MBPS = 0, RF24_2MBPS, RF24_250KBPS } rf24_datarate_e;
typedef enum { RF24_CRC_DISABLED = 0, RF24_CRC_8, RF24_CRC_16 } rf24_crclength_e;



void set_nrf24l01_ce_h();
void set_nrf24l01_ce_l();
void set_nrf24l01_csn_h();
void set_nrf24l01_csn_l();
//SPI操作层
uint8_t spi_send_byte(uint8_t data);

//
uint8_t spi_write_reg(const uint8_t reg,const uint8_t value);
uint8_t spi_read_reg(uint8_t reg);

//
uint8_t nrf_write_reg(uint8_t reg,uint8_t value);
uint8_t nrf_read_reg(uint8_t reg);

uint8_t nrf_write_buf(uint8_t reg,const uint8_t buf[], const uint8_t len);
uint8_t nrf_read_buf(uint8_t reg,uint8_t buf[], uint8_t len);


//接收
uint8_t nrf_rx_packet(uint8_t *rxbuf);
//发送
uint8_t nrf_tx_packet(uint8_t *txbuf);


void write_multi_register(uint8_t reg,const uint8_t buf[], uint8_t len);
void read_register(unsigned char reg, uint8_t buf[], unsigned char len);



//清除TX FIFO寄存器.发射模式下用
uint8_t flush_tx(void);

//清除RX FIFO寄存器.接收模式下用
uint8_t flush_rx(void);


//void spi_write_buf(unsigned char reg, unsigned char *pBuf, unsigned char bytes);
void set_module_tx();
void init_NRF24L01(void);

void setPALevel(uint8_t level);
bool setDataRate(rf24_datarate_e speed);
void setAutoAckAll(bool enable);
void setAutoAck(uint8_t pipe,bool enable);
void setRetries(uint8_t delay, uint8_t count);
void setCRCLength(rf24_crclength_e length);



rf24_datarate_e getDataRate( void );
rf24_crclength_e getCRCLength(void);

void print_address_register(const char* name, uint8_t reg, uint8_t qty);
void print_byte_register(const char* name, uint8_t reg, uint8_t qty);



void print_status(uint8_t status);
uint8_t get_status(void);

void openWritingPipe(uint64_t value);
void openReadingPipe(uint8_t child, uint64_t address);

void writeFast( const void* buf, uint8_t len );

void setChannel(uint8_t channel);
uint8_t getChannel();
uint8_t getPALevel(void);
bool isPVariant(void);




#define pgm_read_byte(addr) (*(const uint8_t *)(addr))


#endif
