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
#include "1wire.h"
#ifdef DEBUG
void
__error__(char *pcFilename, unsigned long ulLine)
{
}
#endif

uint8_t subzero,cel[MAXSENSORS],cel_frac_bits[MAXSENSORS];


int main()
{
	uint8_t i,j;
	uint64_t u_sensorid[MAXSENSORS];

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


	if(ow_reset())
		UARTprintf("one-wire bus error.\n");
	else
		UARTprintf("one-wire bus ok.\n");
	_delay_ms(10);


	uint8_t nSensors;
	nSensors=search_sensors(&u_sensorid[0],MAXSENSORS);
	uint8_t (*pid)[];
	pid=(uint8_t *)&u_sensorid[0];

	UARTprintf("%d Sensors found.\n",nSensors);
	for(i=0;i<nSensors;i++)
	{
		UARTprintf("# %d:",i);
		for(j=0;j<OW_ROMCODE_SIZE;j++)
		{
			UARTprintf("0x%x ",(*pid+i)[j]);
		}
		
		UARTprintf("\n");
	}
		

	while(1)
	{
		for(i=0;i<nSensors;i++)
		{
			UARTprintf("#%d: ",i);
			pid=(uint8_t *)&u_sensorid[0];

			//if(DS18X20_get_temp((uint8_t *)&u_sensorid[i],&subzero,&cel[i],&cel_frac_bits[i]))
			if(DS18X20_get_temp(pid,&subzero,&cel[i],&cel_frac_bits[i]))
			return -1;
			else
			{
				if(subzero)
					UARTprintf("-");
				else
					UARTprintf("+");
				UARTprintf("%d.",(int8_t)cel[i]);
//					itoa(cel_frac_bits*DS18X20_FRACCONV,(char)buffer,10);
//					for(i=0;i<4-strlen(buffer);i++)
//					UARTprintf("0");
				UARTprintf("%4d C`\n",cel_frac_bits[i]*DS18X20_FRACCONV);
			}
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
				if(DS18X20_read_meas((uint8_t *)&u_sensorid[i],&subzero,&cel[i],&cel_frac_bits[i])==DS18X20_OK)
				{
					if(subzero)
						UARTprintf("-");
					else
						UARTprintf("+");
					UARTprintf("%d.",(int8_t)cel[i]);
//					itoa(cel_frac_bits*DS18X20_FRACCONV,(char)buffer,10);
//					for(i=0;i<4-strlen(buffer);i++)
//					UARTprintf("0");
					UARTprintf("%04dC`\n",cel_frac_bits[i]*DS18X20_FRACCONV);
					
				}
				else
					UARTprintf("CRC Error.\n");
			}
		}
	}
}
