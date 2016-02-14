//ADC+OLED的简易示波器

#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"

#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/sysctl.h"
#include "driverlib/adc.h"
#include "driverlib/timer.h"




#ifdef DEBUG
void
__error__(char *pcFilename, unsigned long ulLine)
{
}
#endif


int main()
{
  //设置时钟
  SysCtlClockSet(SYSCTL_SYSDIV_4 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_8MHZ); 
  //使能GPIOA
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
  GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE,GPIO_PIN_3);
  GPIOPadConfigSet(GPIO_PORTB_BASE,GPIO_PIN_3,GPIO_STRENGTH_4MA,GPIO_PIN_TYPE_STD);
  GPIOPinWrite(GPIO_PORTB_BASE,GPIO_PIN_3,0x08);

  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
  //设置PF1为输入,select按键
  GPIOPinTypeGPIOInput(GPIO_PORTF_BASE,GPIO_PIN_1);
    GPIOPadConfigSet(GPIO_PORTF_BASE, GPIO_PIN_1, GPIO_STRENGTH_2MA,
                     GPIO_PIN_TYPE_STD_WPU);
  unsigned long read;
   GPIOPinWrite(GPIO_PORTB_BASE,GPIO_PIN_3,0x08);
   while(1)
   ;
   /*
  while(1)
  {	read=0;
	read=GPIOPinRead(GPIO_PORTB_BASE,GPIO_PIN_1);
	if(read)
	  GPIOPinWrite(GPIO_PORTB_BASE,GPIO_PIN_3,0x08);
	else
	  GPIOPinWrite(GPIO_PORTB_BASE,GPIO_PIN_3,0x00);
  };
  */

}
