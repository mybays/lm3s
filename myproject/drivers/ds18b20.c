#include "inc/hw_gpio.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/gpio.h"
#include "driverlib/sysctl.h"
#include "ds18b20.h"
#include "mydelay.h"
#include <inttypes.h>


//onewire总线拉低
void ow_out_low(void)
{
	GPIOPinWrite(OW_BUS_BASE,(1<<OW_BUS),0x00);
}

//onewire总线输出高电平
void ow_out_high(void)
{
	GPIOPinWrite(OW_BUS_BASE,(1<<OW_BUS),(1<<OW_BUS));
}

//onewire总线设置为输出
void ow_dir_out(void)
{
	GPIOPinTypeGPIOOutput(OW_BUS_BASE,(1<<OW_BUS));
}

//onewire总线设置为输入
void ow_dir_in(void)
{
	GPIOPinTypeGPIOInput(OW_BUS_BASE,(1<<OW_BUS));
}

//返回总线状态,0或者1
uint8_t ow_get_in(void)
{
	return HWREG(OW_BUS_BASE+GPIO_O_DATA+(0xff<<2))&(1<<OW_BUS);
}

//总线复位,如果一切正常,函数返回0
uint8_t ow_reset(void)
{
	uint8_t err;

	ow_dir_out();		//确保之前为高电平
	ow_out_low();
	_delay_us(480);		//复位总线
	ow_dir_in();
	_delay_us(66);
	err=ow_get_in();	//如果总线上存在DS18B20,此时总线要被期间拉低,如果没有被拉低,说明有错误,err=1
	_delay_us(480-66);
	if(ow_get_in()==0)	//此时总线应该恢复为高电平
		err=1;
	return err;
}

//总线读写1bit
uint8_t ow_bit_io(uint8_t b)
{
	ow_dir_out();
	ow_out_low();
	_delay_us(1);
	if(b)
		ow_dir_in();
	_delay_us(15-1);
	if(ow_get_in()==0)
		b=0;
	_delay_us(60-15);
	ow_dir_in();
	return b;
}

//总线写一字节
uint8_t ow_byte_wr(uint8_t b)
{
	uint8_t i=8,j;
	do
	{
		j=ow_bit_io(b&1);
		b>>=1;
		if(j)
			b|=0x80;
	}	while(--i);
	return b;
}

//总线读一字节
uint8_t ow_byte_rd(void)
{
	return ow_byte_wr(0xFF);
}

//总线上搜索传感器
uint8_t ow_rom_search(uint8_t diff,uint8_t *id)
{
	uint8_t i,j,next_diff;
	uint8_t b;

	if(ow_reset())
	{
		UARTprintf("reset error.\n");
	 	return OW_PRESENCE_ERR;
	}
	ow_byte_wr(OW_SEARCH_ROM);
	next_diff=OW_LAST_DEVICE;

	i=OW_ROMCODE_SIZE*8;

	do
	{
		j=8;
		do
		{
			b=ow_bit_io(1);
			if(ow_bit_io(1))
			{
				if(b)
					return OW_DATA_ERR;
			}
			else
			{
				if(!b)
				{
					if(diff>i||((*id&1)&&diff!=i))
					{
						b=1;
						next_diff=i;
					}
				}
			}
			ow_bit_io(b);
			*id>>=1;
			if(b)
				*id|=0x80;
			i--;
		}while(--j);
		id++;

	}while(i);

	return next_diff;	
}

//向总线发一个命令
void ow_command(uint8_t command,uint8_t *id)
{
	uint8_t i;
	ow_reset();
	if(id)
	{
		ow_byte_wr(OW_MATCH_ROM);
		i=OW_ROMCODE_SIZE;
		do
		{
			ow_byte_wr(*id);
			id++;
		}while(--i);
	}
	else
	{
		ow_byte_wr(OW_SKIP_ROM);
	}
	ow_byte_wr(command);
}

void DS18X20_find_sensor(uint8_t *diff,uint8_t id[])
{
	for(;;)
	{
		*diff=ow_rom_search(*diff,&id[0]);
		if(*diff==OW_PRESENCE_ERR||*diff==OW_DATA_ERR||*diff==OW_LAST_DEVICE)
			return;
		if(id[0]==DS18B20_ID||id[0]==DS18S20_ID)
			return;
	}
}

//DS18X20开始转换
uint8_t DS18X20_start_meas(uint8_t id[])
{
	ow_reset();
	if(ow_get_in())
	{
		ow_command(DS18X20_CONVERT_T,id);
		return DS18X20_OK;
	}
	else
	{
		return DS18X20_START_FAIL;
	}
}

uint8_t DS18X20_read_meas(uint8_t id[],uint8_t *subzero,uint8_t *cel,uint16_t *cel_frac_bits)
{
	uint8_t i;
	uint8_t sp[DS18X20_SP_SIZE];
	ow_reset();
	ow_command(DS18X20_READ,id);
	for(i=0;i<DS18X20_SP_SIZE;i++)
		sp[i]=ow_byte_rd();
		if(crc8(&sp[0],DS18X20_SP_SIZE))
			return DS18X20_ERROR_CRC;
		DS18X20_meas_to_cel(id[0],sp,subzero,cel,cel_frac_bits);
		return DS18X20_OK;
}

#define CRC8INIT 0x00
#define CRC8POLY 0x18
//CRC部分
uint8_t crc8(uint8_t* data_in,uint16_t number_of_bytes_to_read)
{
	uint8_t crc;
	uint16_t loop_count;
	uint8_t bit_counter;
	uint8_t data;
	uint8_t feedback_bit;

	crc=CRC8INIT;

	for(loop_count=0;loop_count!=number_of_bytes_to_read;loop_count++)
	{
		data=data_in[loop_count];

		bit_counter=8;
		do
		{
			feedback_bit=(crc^data)&0x01;
			if(feedback_bit==0x01)
			{
				crc=crc^CRC8POLY;
			}
			crc=(crc>>1)&0x7F;
			if(feedback_bit==0x01)
			{
				crc=crc|0x80;
			}
			data=data>>1;
			bit_counter--;
		}while(bit_counter>0);
	}
	return crc;
}

extern uint8_t DS18X20_meas_to_cel(uint8_t fc,uint8_t *sp,uint8_t* subzero,uint8_t* cel,uint16_t* cel_frac_bits)
{
	uint16_t meas;
	uint8_t i;

	meas=sp[0];
	meas|=((uint16_t)sp[1])<<8;

	if(fc==DS18S20_ID)
	{
		meas&=(uint16_t)0xfffe;
		meas<<=3;
		meas+=(16-sp[6])-4;
	}
	if(meas&0x8000)		//如果最高位为1,说明温度为负
	{
		*subzero=1;
		meas^=0xffff;
		meas++;
	}
	else *subzero=0;
	if(fc==DS18B20_ID)
	{
		i=sp[DS18B20_CONF_REG];
		if((i&DS18B20_12_BIT)==DS18B20_12_BIT)
			;
		else if((i&DS18B20_11_BIT)==DS18B20_11_BIT)
			meas&=~(DS18B20_11_BIT_UNDF);
		else if((i&DS18B20_10_BIT)==DS18B20_10_BIT)
			meas&=~(DS18B20_10_BIT_UNDF);
		else
			meas&=~(DS18B20_9_BIT_UNDF);
	}
	UARTprintf("meas:%d.\n",meas);
	*cel=(uint8_t)(meas>>4);
	*cel_frac_bits=(uint8_t)(meas&0x000F);
	return DS18X20_OK;
}



//获取一个传感器的温度,参数1为指定的id,参数2存放是否零下标志,参数3为整数部分,参数4为小数部分
uint8_t DS18X20_get_temp(uint8_t id[],uint8_t *subzero,uint8_t *cel,uint16_t *cel_frac_bits)
{
	//保存临时的数据
	uint8_t sp[DS18X20_SP_SIZE];
	//保存16bit的温度raw数据
	uint16_t meas;
	//临时变量
	uint8_t i;

	ow_reset();
	if(ow_get_in())
	{
		//发送转换命令
		ow_command(DS18X20_CONVERT_T,id);
		//转换12bit温度需要的时间
		_delay_ms(DS18B20_TCONV_12BIT);

		ow_reset();

		//读出临时数据到sp[]
		ow_command(DS18X20_READ,id);
		for(i=0;i<DS18X20_SP_SIZE;i++)
			sp[i]=ow_byte_rd();

		//校验数据的完整性
		if(crc8(&sp[0],DS18X20_SP_SIZE))
			return -1;

		//转换到16bit的raw数据
		meas=sp[0];
		meas|=((uint16_t)sp[1])<<8;

		//看是否为零下温度
		if(meas&0x8000)
		{
			*subzero=1;
			meas^=0xffff;
			meas++;
		}
		else *subzero=0;

		//bit4
		i=sp[DS18B20_CONF_REG];
		if((i&DS18B20_12_BIT)==DS18B20_12_BIT)
			;
		else if((i&DS18B20_11_BIT)==DS18B20_11_BIT)
			meas&=~(DS18B20_11_BIT_UNDF);
		else if((i&DS18B20_10_BIT)==DS18B20_10_BIT)
			meas&=~(DS18B20_10_BIT_UNDF);
		else
			meas&=~(DS18B20_9_BIT_UNDF);

		//获取整数部分
		*cel=(uint8_t)(meas>>4);
		//获取小数部分
		*cel_frac_bits=(uint8_t)(meas&0x000F);

		return 0;
	}
	else
	{
		return -1;
	}
}
