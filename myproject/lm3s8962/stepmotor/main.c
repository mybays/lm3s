/*********************************************
LED:PF0
*********************************************/
#include "inc/hw_gpio.h"
#include"inc/hw_types.h"
#include"inc/hw_memmap.h"
#include"inc/lm3s8962.h"
#include"driverlib/gpio.h"
#include"driverlib/sysctl.h"
#include"mydelay.h"

#define STEP_MOTOR_PORT_BASE GPIO_PORTB_BASE

int main(void)
{
	//时钟配置,50MHz
	SysCtlClockSet(SYSCTL_SYSDIV_4|SYSCTL_USE_PLL|SYSCTL_OSC_MAIN | SYSCTL_XTAL_8MHZ);
	//PF0LED接口
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
	GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE,0x0F);

	int step1[4]={0x01,0x02,0x04,0x08};
	int step2[4]={0x03,0x06,0x0c,0x09};
	int step3[8]={0x01,0x03,0x02,0x06,0x04,0x0c,0x08,0x0d};

	while(1)
	{
		for(int i=0;i<4;i++)
		{
			HWREG(STEP_MOTOR_PORT_BASE+GPIO_O_DATA+(0xff<<2))=step2[3-i];
			_delay_ms(2);
		}
	}
}
