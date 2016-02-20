#include "1wire.h"







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
void ow_command(uint8_t command,const uint8_t *id)
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


