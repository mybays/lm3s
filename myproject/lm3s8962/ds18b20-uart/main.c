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
#include <string.h>

#include "mydelay.h"
#include "ds18b20.h"
#ifdef DEBUG
void
__error__(char *pcFilename, unsigned long ulLine)
{
}
#endif


#define MAXSENSORS 1
uint8_t gSensorIDs[OW_ROMCODE_SIZE];
uint8_t subzero,cel,cel_frac_bits;

uint8_t buffer[sizeof(int)*8+1];
uint8_t search_sensors(void)
{
	uint8_t i;
	uint8_t id[OW_ROMCODE_SIZE];
	uint8_t diff,nSensors=0;
	UARTprintf("Scanning Bus for DS18X20.\n");
	for(diff=OW_SEARCH_FIRST;diff != OW_LAST_DEVICE && nSensors<MAXSENSORS;)
	{
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
			gSensorIDs[i]=id[i];
		nSensors++;
	}
	return nSensors;
}

//若总线上存在DS18B20,值为真,否则为0
int ds18b20_present=0;

int main()
{
	uint8_t i,j;

	unsigned long SysClock=0;
	//使能PLL,系统时钟为50MHz
	SysCtlClockSet(SYSCTL_SYSDIV_4 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_8MHZ);
	//使能串口0
	SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
	//使能GPIOA
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
	//把PA0和PA1转换为功能引脚(串口)
	GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
	//配置串口格式,波特率115200,8n1
	UARTConfigSetExpClk(UART0_BASE, SysCtlClockGet(), 115200,
			(UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE|
			UART_CONFIG_PAR_NONE));
	//使能串口的中断
	IntEnable(INT_UART0);
	//使能串口中断中的收和发
	UARTIntEnable(UART0_BASE, UART_INT_RX | UART_INT_RT);
	//初始化标准输入输出
	UARTStdioInit(0);
	SysClock=SysCtlClockGet();
	UARTprintf("Uart works fine.\nSystem frequency is %d.\n",SysClock);

	//使能PORTC,PC7为单线总线
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
	//SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
	//test PC7
	GPIOPinTypeGPIOOutput(GPIO_PORTC_BASE,GPIO_PIN_7);
	GPIOPadConfigSet(GPIO_PORTC_BASE,GPIO_PIN_7,GPIO_STRENGTH_4MA,GPIO_PIN_TYPE_STD_WPU);
	ow_dir_out();
	ow_out_high();
/*
	while(1)
	{
		ow_out_high();
		_delay_ms(1000);
		ow_out_low();
		_delay_ms(1000);
	}
*/
/*
	ow_dir_out();
	GPIOPinTypeGPIOOutput(GPIO_PORTC_BASE,0x80);
//	while(1)
	{
		ow_out_high();
		_delay_ms(1000);
		ow_out_low();
		_delay_ms(1000);
	}
*/

	if(ow_reset())
		UARTprintf("one-wire bus error.\n");
	else
		UARTprintf("one-wire bus ok.\n");
	_delay_ms(10);


	uint8_t nSensors;
	nSensors=search_sensors();
	UARTprintf("%d Sensors found.\n",nSensors);

		UARTprintf("# %d:",i);
		for(j=0;j<OW_ROMCODE_SIZE;j++)
		UARTprintf("0x%x ",gSensorIDs[j]);
		UARTprintf("\n");

	while(1)
	{
		if(DS18X20_get_temp(&gSensorIDs[0],&subzero,&cel,&cel_frac_bits))
			return -1;
		else
		{
					if(subzero)
						UARTprintf("-");
					else
						UARTprintf("+");
					UARTprintf("%d.",(int8_t)cel);
//					itoa(cel_frac_bits*DS18X20_FRACCONV,(char)buffer,10);
//					for(i=0;i<4-strlen(buffer);i++)
//					UARTprintf("0");
					UARTprintf("%4d C`\n",cel_frac_bits*DS18X20_FRACCONV);
			
		}
	}

	while(1)
	{
		if(DS18X20_start_meas()==DS18X20_OK)
		{
			_delay_ms(DS18B20_TCONV_12BIT);
			for(i=0;i<nSensors;i++)
			{
				UARTprintf("Sensor# %d =",i);
				if(DS18X20_read_meas(&gSensorIDs[0],&subzero,&cel,&cel_frac_bits)==DS18X20_OK)
				{
					if(subzero)
						UARTprintf("-");
					else
						UARTprintf("+");
					UARTprintf("%d.",(int8_t)cel);
//					itoa(cel_frac_bits*DS18X20_FRACCONV,(char)buffer,10);
//					for(i=0;i<4-strlen(buffer);i++)
//					UARTprintf("0");
					UARTprintf("%o4dC`\n",cel_frac_bits*DS18X20_FRACCONV);
					
				}
				else
					UARTprintf("CRC Error.\n");
			}
		}
		_delay_ms(10);
	}



/*
	//test gpio,led:PF0,高电平点亮,select key:PF1,按下为低电平
	//使能PORTF
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
	//PF0,发光二极管,设置为输出
//	HWREG(GPIO_PORTF_BASE+GPIO_O_DIR)|=0x01;
	//PF1,按键,设置为输入
//	HWREG(GPIO_PORTF_BASE+GPIO_O_DIR)&=0xFD;
	GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE,0x01);
	GPIOPinTypeGPIOInput(GPIO_PORTF_BASE,0x02);
	while(1)
	{
		if(HWREG(GPIO_PORTF_BASE+GPIO_O_DATA+(0xff<<2))&0x02)
			GPIOPinWrite(GPIO_PORTF_BASE,0x01,0x01);
		else
			GPIOPinWrite(GPIO_PORTF_BASE,0x01,0x00);
	}
	
*/
	//test uart and sysclock
	while(1)
	{
		UARTprintf("%d\n",SysClock);
		_delay_ms(1000);
	};
  

}
