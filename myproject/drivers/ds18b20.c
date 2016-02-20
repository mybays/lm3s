#include "inc/hw_gpio.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/gpio.h"
#include "driverlib/sysctl.h"
#include "ds18b20.h"
#include "mydelay.h"
#include <inttypes.h>
#include "crc8.h"
#include "1wire.h"



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
	//UARTprintf("meas:%d.\n",meas);
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



uint8_t search_sensors(uint64_t gid[],uint8_t maxnum)
{
	uint8_t i;
	uint8_t id[OW_ROMCODE_SIZE];
	uint8_t diff,nSensors=0;
	uint8_t (*pid)[];
	
	UARTprintf("Scanning Bus for DS18X20.\n");
	for(diff=OW_SEARCH_FIRST;diff != OW_LAST_DEVICE && nSensors<MAXSENSORS;)
	{
		pid=(uint8_t *)&gid[nSensors];
		DS18X20_find_sensor(&diff,&id[0]);
		if(diff==OW_PRESENCE_ERR)
		{
			UARTprintf("No Sensors found.\n");
			break;
		}
		if(diff==OW_DATA_ERR)
		{
			UARTprintf("Bus Error.\n");
			break;
		}
		for(i=0;i<OW_ROMCODE_SIZE;i++)
		{
			//gid[nSensors]|=((uint64_t)id[i]<<(8*i));
			(*pid)[i]=id[i];
		}
		nSensors++;
		//UARTprintf("test:%lx\r\n",gid[nSensors]);
		if(nSensors>=maxnum)
		{
			break;
		}
	}
	return nSensors;
}

