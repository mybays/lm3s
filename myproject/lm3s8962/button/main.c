#include "inc/lm3s8962.h"
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/ethernet.h"
#include "driverlib/flash.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/sysctl.h"
#include "driverlib/systick.h"
#include "utils/locator.h"
#include "utils/uartstdio.h"
#include "utils/ustdlib.h"


//GPIOF中断服务函数
void GPIOFIntHandler(void);





int main(void)
{

	// Enable the GPIO pin for the LED (PF0).  Set the direction as output, and
	// enable the GPIO pin for digital function.
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
	GPIOPinTypeGPIOInput(GPIO_PORTF_BASE,GPIO_PIN_1);
	GPIOPadConfigSet(GPIO_PORTF_BASE,GPIO_PIN_1,GPIO_STRENGTH_2MA,GPIO_PIN_TYPE_STD_WPU);
	GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE,GPIO_PIN_0);
	GPIOIntTypeSet(GPIO_PORTF_BASE,GPIO_PIN_1,GPIO_FALLING_EDGE);
	GPIOPinIntEnable(GPIO_PORTF_BASE,GPIO_PIN_1);
	IntEnable(INT_GPIOF);
	IntMasterEnable();

	//
	// Loop forever.
	//
	while(1)
	{
	}

	return 0;
}

//GPIOF中断服务函数
void GPIOFIntHandler(void)
{
	unsigned long ulStatus=GPIOPinIntStatus(GPIO_PORTF_BASE,true);
	GPIOPinIntClear(GPIO_PORTF_BASE,ulStatus);
	if(ulStatus&0x02)
	{
		GPIOPinWrite(GPIO_PORTF_BASE,GPIO_PIN_0,(~GPIOPinRead(GPIO_PORTF_BASE,GPIO_PIN_0)));
	}
}
