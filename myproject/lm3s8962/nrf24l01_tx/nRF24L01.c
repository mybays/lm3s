#include "nRF24L01.h"
#include "stdint.h"
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/debug.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "driverlib/ssi.h"
#include "driverlib/pin_map.h"
#include "drivers/rit128x96x4.h"
#include "drivers/mydelay.h"
#include "inc/hw_types.h"
#include "inc/hw_memmap.h"
#include "inc/lm3s8962.h"
#include "driverlib/gpio.h"
#include "driverlib/sysctl.h"
#include "mydelay.h"
#include <inttypes.h>
#include "uartstdio.h"

#define TX_ADR_WIDTH    5   	// 5 uints TX address width
#define RX_ADR_WIDTH    5   	// 5 uints RX address width
#define RX_PLOAD_WIDTH  32  	// 20 uints TX payload
#define TX_PLOAD_WIDTH  32  	// 20 uints TX payload


bool p_variant=false;

const uint8_t child_pipe_enable[] =
{
  ERX_P0, ERX_P1, ERX_P2, ERX_P3, ERX_P4, ERX_P5
};

static const uint8_t child_pipe[] =
{
  RX_ADDR_P0, RX_ADDR_P1, RX_ADDR_P2, RX_ADDR_P3, RX_ADDR_P4, RX_ADDR_P5
};

const uint8_t child_payload_size[] =
{
  RX_PW_P0, RX_PW_P1, RX_PW_P2, RX_PW_P3, RX_PW_P4, RX_PW_P5
};

const unsigned char TX_ADDRESS[5] = { 'T', 'I', 'M', 'C', 'U' }; // 定义一个发送地址（发送数据的地址） ;
const unsigned char RX_ADDRESS[5] = { 'B', 'O', 'O', 'S', 'T' }; // 定义一个接收地址（接收应答信号的地址）;


typedef enum { RF24_PA_MIN = 0,RF24_PA_LOW, RF24_PA_HIGH, RF24_PA_MAX, RF24_PA_ERROR } rf24_pa_dbm_e ;


//uint8_t  RX_ADDRESS[RX_ADR_WIDTH]= {0xE6,0xE6,0xE6,0xE6,0xE6};	//本地地址
uint8_t  RX_ADDRESS1[RX_ADR_WIDTH]= {0xc2,0xc2,0xc2,0xc2,0xE6};	//本地地址

//uint8_t  TX_ADDRESS[TX_ADR_WIDTH]= {0xE6,0xE6,0xE6,0xE6,0xE6};	//接收地址

void set_nrf24l01_ce_h()
{
	GPIO_PORTC_DATA_R|=(1<<7);
}

void set_nrf24l01_ce_l()
{
	GPIO_PORTC_DATA_R&=~(1<<7);
}

void set_nrf24l01_csn_h()
{
	GPIO_PORTC_DATA_R|=(1<<5);
}

void set_nrf24l01_csn_l()
{
	GPIO_PORTC_DATA_R&=~(1<<5);
}


//SPI操作层

//spi写/读一字节
uint8_t spi_send_byte(uint8_t data)
{
	uint32_t status;
	//while(SSIBusy(SSI0_BASE)){}
	SSIDataPut(SSI0_BASE,data);
	SSIDataGet(SSI0_BASE,&status);
	//while(SSIBusy(SSI0_BASE)){}
	return (uint8_t)status;
}

//spi写nrf24寄存器
uint8_t spi_write_reg(const uint8_t reg,const uint8_t value)
{
	uint8_t status;
	
	set_nrf24l01_csn_l();
	status=spi_send_byte(reg);
	spi_send_byte(value);
	set_nrf24l01_csn_h();

    return status;
}

//spi读nrf24寄存器
uint8_t spi_read_reg(uint8_t reg)
{
	uint8_t value=0;

	set_nrf24l01_csn_l();
	spi_send_byte(reg);
	value=spi_send_byte(0xff);
	set_nrf24l01_csn_h();

	return value;
}


void set_module_tx()
{
	spi_write_reg(CONFIG, ( spi_read_reg(CONFIG) ) & ~_BV(PRIM_RX) );
}

uint8_t nrf_write_reg(uint8_t reg,uint8_t value)
{
	return spi_write_reg(W_REGISTER|(REGISTER_MASK & reg),value);
}

uint8_t nrf_read_reg(uint8_t reg)
{
	return spi_read_reg(R_REGISTER | ( REGISTER_MASK & reg ));
}


//uint8_t nrf_write_buf(uint8_t reg,const uint8_t * buf,uint8_t len)
uint8_t nrf_write_buf(uint8_t reg,const uint8_t buf[], const uint8_t len)
{
	uint8_t status;
	set_nrf24l01_csn_l();
	status=spi_send_byte(W_REGISTER | ( REGISTER_MASK & reg ));

	for(int i=0;i<len;i++)
	{
		status=spi_send_byte(*buf);
		buf++;
	}
	set_nrf24l01_csn_h();
	return status;
}

uint8_t nrf_read_buf(uint8_t reg,uint8_t buf[], uint8_t len)
{
	uint8_t status,i;
	set_nrf24l01_csn_l();
	status=spi_send_byte(R_REGISTER | ( REGISTER_MASK & reg ));
	for(i=0;i<len;i++)
	{
		status=spi_send_byte(0xff);
		buf[i]=status;
	}
	set_nrf24l01_csn_h();
	return status;
}


/*
void write_multi_register(uint8_t reg,const uint8_t buf[], uint8_t len)
{
	set_nrf24l01_csn_l();
	uint8_t test;
	int i;
	SSIDataPut(SSI0_BASE, W_REGISTER | ( REGISTER_MASK & reg ));
	SSIDataGet(SSI0_BASE, &test);
	for(i=0;i<len;i++)
	{
		SSIDataPut(SSI0_BASE, buf[i]);
		SSIDataGet(SSI0_BASE, &test);
	}
	set_nrf24l01_csn_h();
}

void read_register(unsigned char reg, uint8_t buf[], unsigned char len)
{
	set_nrf24l01_csn_l();
	unsigned char test;
	int i;
	SSIDataPut(SSI0_BASE, R_REGISTER | ( REGISTER_MASK & reg ));
	SSIDataGet(SSI0_BASE, &test);
	for(i=0;i<len;i++)
	{
		SSIDataPut(SSI0_BASE, 0xff);
		SSIDataGet(SSI0_BASE, &buf[i]);
	}
	set_nrf24l01_csn_h();
}
*/








/*
void spi_write_buf(unsigned char reg, unsigned char *pBuf, unsigned char bytes)
{
	set_nrf24l01_csn_l();
	SSIDataPut(SSI0_BASE, reg);
	for(;bytes>0;bytes--)
	{
		while(SSIBusy(SSI0_BASE)){}
		SSIDataPut(SSI0_BASE, *pBuf);
		pBuf++;
	}
	while(SSIBusy(SSI0_BASE)){}
	set_nrf24l01_csn_h();
}
*/
/*
void set_module_tx()
{
	set_nrf24l01_ce_l();
	spi_write_reg(WRITE_REG+CONFIG,0x5e);
	spi_write_reg(WRITE_REG+EN_AA,0x00);				//关闭自动重发
	spi_write_reg(WRITE_REG+EN_RXADDR,0x00);			//关闭接收
	spi_write_reg(WRITE_REG+RF_CH,89);					//设置频率88
	spi_write_buf(WRITE_REG+TX_ADDR,TX_ADDRESS,5);
	spi_write_buf(WRITE_REG+RX_ADDR_P0,TX_ADDRESS,5);
	spi_write_buf(WRITE_REG+RX_ADDR_P1,RX_ADDRESS,5);
}

void transfer_string(unsigned char *string, unsigned char length)
{
	set_nrf24l01_ce_l();
	spi_write_buf(WR_TX_PLOAD,string,length);
	spi_write_reg(WRITE_REG+STATUS,0x70);
	set_nrf24l01_ce_h();
	_delay_us(100);
	set_nrf24l01_ce_l();
}
*/



void setChannel(uint8_t channel)
{
	nrf_write_reg(RF_CH,channel);
}
uint8_t getChannel()
{
	return spi_read_reg(RF_CH);
}

void setPALevel(uint8_t level)
{
	uint8_t setup=nrf_read_reg(RF_SETUP) & 0b11111000;
	if(level >3)
	{
		level = (RF24_PA_MAX << 1) + 1;
	}
	else
	{
		level = (level << 1) + 1;
	}
	nrf_write_reg(RF_SETUP, setup |= level);
}

bool setDataRate(rf24_datarate_e speed)
{
	bool result=false;
	uint8_t setup=nrf_read_reg(RF_SETUP);

	setup &= ~(_BV(RF_DR_LOW) | _BV(RF_DR_HIGH)) ;
	if( speed == RF24_250KBPS )
	{
		setup |= _BV( RF_DR_LOW ) ;
	}
	else
	{
		if(speed == RF24_2MBPS)
		{
			setup |= _BV(RF_DR_HIGH);
		}
	}
	nrf_write_reg(RF_SETUP,setup);
	if(nrf_read_reg(RF_SETUP) == setup)
	{
		result=true;
	}
	return result;
}

void setAutoAckAll(bool enable)
{
	if(enable)
	{
		nrf_write_reg(EN_AA,0b111111);
	}
	else
	{
		nrf_write_reg(EN_AA,0);
	}
}
void setAutoAck(uint8_t pipe,bool enable)
{
	if(pipe<=6)
	{
		uint8_t en_aa=nrf_read_reg(EN_AA);
		if(enable)
		{
			en_aa|=_BV(pipe);
		}
		else
		{
			en_aa &=~_BV(pipe);
		}
		nrf_write_reg(EN_AA,en_aa);
	}
}

void setRetries(uint8_t delay, uint8_t count)
{
	nrf_write_reg(SETUP_RETR,(delay&0xf)<<ARD | (count&0xf)<<ARC);
}

void setCRCLength(rf24_crclength_e length)
{
	uint8_t config = nrf_read_reg(CONFIG) & ~( _BV(CRCO) | _BV(EN_CRC)) ;

	// switch uses RAM (evil!)
	if ( length == RF24_CRC_DISABLED )
	{
 		// Do nothing, we turned it off above.
	}
	else if ( length == RF24_CRC_8 )
	{
		config |= _BV(EN_CRC);
	}
	else
	{
		config |= _BV(EN_CRC);
		config |= _BV( CRCO );
	}
	nrf_write_reg( CONFIG, config ) ;
}


void init_NRF24L01(void)
{
	set_nrf24l01_ce_l();
	set_nrf24l01_csn_h();

	// Reset NRF_CONFIG and enable 16-bit CRC.
	//nrf_write_reg( NRF_CONFIG, 0b00001100 ) ;
	//setRetries(5,15);
	//nrf_write_reg(SETUP_RETR,(5&0xf)<<ARD | (15&0xf)<<ARC);
	
	setChannel(74);
	setPALevel(RF24_PA_MAX);
	setDataRate(RF24_2MBPS);
	setAutoAckAll(1);
	setRetries(2,15);
	setCRCLength(RF24_CRC_8);
	
	

	/*
	spi_write_buf(WRITE_REG + TX_ADDR, TX_ADDRESS, TX_ADR_WIDTH);
	spi_write_buf(WRITE_REG + RX_ADDR_P0, RX_ADDRESS, RX_ADR_WIDTH);
	spi_write_buf(WRITE_REG + RX_ADDR_P1, RX_ADDRESS1, RX_ADR_WIDTH);
	
	spi_write_reg(WRITE_REG + EN_RXADDR, 0x03);
	
	
	spi_write_reg(WRITE_REG + RX_PW_P0, RX_PLOAD_WIDTH);
	spi_write_reg(WRITE_REG + RX_PW_P1, RX_PLOAD_WIDTH);
	*/
	
	set_nrf24l01_ce_h();
	_delay_us(200);

}

void print_address_register(const char* name, uint8_t reg, uint8_t qty)
{
	UARTprintf("%s\t =",name);
	//UARTprintf("test1:%d\r\n",qty);

	while(qty--)
	{
		uint8_t buffer[5]={0};
		nrf_read_buf(reg++,&buffer[0],5);
		UARTprintf(" 0x");
		unsigned char* bufptr = buffer + sizeof buffer;
		while( --bufptr >= buffer )
			UARTprintf("%02x",*bufptr);
	}
	UARTprintf("\r\n");
}

void print_byte_register(const char* name, uint8_t reg, uint8_t qty)
{
	UARTprintf("%s\t =",name);
	while(qty--)
	{
		UARTprintf(" 0x%02x",nrf_read_reg(reg++));
	}
	UARTprintf("\r\n");
}

void testprint_address_register(const char* name, uint8_t reg, uint8_t qty)
{
	UARTprintf("%s\t =",name);
	UARTprintf("test1:%d\r\n",qty);

	while(qty--)
	{
		UARTprintf("test:%d\r\n",qty);
		uint8_t buffer[5]={0};
		nrf_read_buf(reg++,&buffer[0],sizeof buffer);
		UARTprintf(" 0x");
		unsigned char* bufptr = buffer + sizeof buffer;
		while( --bufptr >= buffer )
			UARTprintf("%02x",*bufptr);
	}
	UARTprintf("\r\nend......\r\n");
}

void openWritingPipe(uint64_t value)
{
	nrf_write_buf(RX_ADDR_P0, (uint8_t*)(&value), 5);
	nrf_write_buf(TX_ADDR, (uint8_t*)(&value), 5);
	nrf_write_reg(RX_PW_P0,32);
}

void openReadingPipe(uint8_t child, uint64_t address)
{
	if(child == 0)
	{

	}
	if(child <=6)
	{
		if(child <2)
		{
			nrf_write_buf(pgm_read_byte(&child_pipe[child]), (const uint8_t*)(&address), 5);
		}
		else
		{
			nrf_write_buf(pgm_read_byte(&child_pipe[child]), (const uint8_t*)(&address), 1);
		}
		//
		nrf_write_reg(pgm_read_byte(&child_payload_size[child]),5);
		//
		nrf_write_reg(EN_RXADDR,nrf_read_reg(EN_RXADDR) | _BV(pgm_read_byte(&child_pipe_enable[child])));
	}
}

rf24_crclength_e getCRCLength(void)
{
  rf24_crclength_e result = RF24_CRC_DISABLED;
  
  uint8_t config = nrf_read_reg(CONFIG) & ( _BV(CRCO) | _BV(EN_CRC)) ;
  uint8_t AA = nrf_read_reg(EN_AA);
  
  if ( config & _BV(EN_CRC ) || AA)
  {
    if ( config & _BV(CRCO) )
      result = RF24_CRC_16;
    else
      result = RF24_CRC_8;
  }

  return result;
}

void print_status(uint8_t status)
{
  UARTprintf("STATUS\t\t = 0x%02x RX_DR=%x TX_DS=%x MAX_RT=%x RX_P_NO=%x TX_FULL=%x\r\n",
           status,
           (status & _BV(RX_DR))?1:0,
           (status & _BV(TX_DS))?1:0,
           (status & _BV(MAX_RT))?1:0,
           ((status >> RX_P_NO) & 0b111),
           (status & _BV(TX_FULL))?1:0
          );
}

uint8_t get_status(void)
{
	set_nrf24l01_csn_l();
	return spi_send_byte(NOP);
	set_nrf24l01_csn_h();
}

uint8_t getPALevel(void)
{

  return (nrf_read_reg(RF_SETUP) & (_BV(RF_PWR_LOW) | _BV(RF_PWR_HIGH))) >> 1 ;
}

bool isPVariant(void)
{
  return p_variant ;
}


rf24_datarate_e getDataRate( void )
{
  rf24_datarate_e result ;
  uint8_t dr = nrf_read_reg(RF_SETUP) & (_BV(RF_DR_LOW) | _BV(RF_DR_HIGH));

  // switch uses RAM (evil!)
  // Order matters in our case below
  if ( dr == _BV(RF_DR_LOW) )
  {
    // '10' = 250KBPS
    result = RF24_250KBPS ;
  }
  else if ( dr == _BV(RF_DR_HIGH) )
  {
    // '01' = 2MBPS
    result = RF24_2MBPS ;
  }
  else
  {
    // '00' = 1MBPS
    result = RF24_1MBPS ;
  }
  return result ;
}


//清除TX FIFO寄存器.发射模式下用
uint8_t flush_tx(void)
{
	set_nrf24l01_csn_l();
	return spi_send_byte(FLUSH_TX);
	set_nrf24l01_csn_h();
}
//清除RX FIFO寄存器.接收模式下用
uint8_t flush_rx(void)
{
	set_nrf24l01_csn_l();
	return spi_send_byte(FLUSH_RX);
	set_nrf24l01_csn_h();
}


uint8_t nrf_rx_packet(uint8_t *rxbuf)
{
	uint8_t state;
	state=spi_send_byte(STATUS);
	nrf_write_reg(STATUS,state);
	if(state&RX_DR)
	{
		nrf_read_buf(RD_RX_PLOAD,rxbuf,RX_PLOAD_WIDTH);
		nrf_write_reg(FLUSH_RX,0xFF);
		return 0;
	}
	return 1;
}

uint8_t nrf_tx_packet(uint8_t *txbuf)
{
	uint8_t state,status;
	set_nrf24l01_ce_l();
	set_nrf24l01_csn_l();
	status=spi_send_byte(WR_TX_PLOAD);
	for(int i=0;i<TX_PLOAD_WIDTH;i++)
	{
		status=spi_send_byte(txbuf);
		txbuf++;
	}
	set_nrf24l01_csn_h();
	set_nrf24l01_ce_h();
	//等待IRQ信号,HOW? IRQ信号有无替代寄存器?
	//while();
	state=nrf_read_reg(STATUS);
	nrf_write_reg(STATUS,state);
	if(state&MAX_RT)
	{
		nrf_write_reg(FLUSH_TX,0xff);
		return MAX_RT;
	}
	if(state&TX_DS)
	{
		return TX_DS;
	}
	return 0xFF;
}
