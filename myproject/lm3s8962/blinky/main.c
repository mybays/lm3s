/*********************************************
LED:PF0
*********************************************/
#include"inc/hw_types.h"
#include"inc/hw_memmap.h"
#include"inc/lm3s8962.h"
#include"driverlib/gpio.h"
#include"driverlib/sysctl.h"
#include"mydelay.h"


int main(void)
{
	//时钟配置,50MHz
	SysCtlClockSet(SYSCTL_SYSDIV_4|SYSCTL_USE_PLL|SYSCTL_OSC_MAIN | SYSCTL_XTAL_8MHZ);
	//PF0LED接口
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
	GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE,0x01);
	while(1)
	{
		GPIO_PORTF_DATA_R|=0x01;
		_delay_ms(300);
		GPIO_PORTF_DATA_R&=~(0x01);
		_delay_ms(300);
	}
}
