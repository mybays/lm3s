//SysCtlDelay
//系统延时函数SysCtlDelay延时3个时钟周期

//#include "inc/hw_ints.h"
#include "inc/hw_types.h"
#include "driverlib/sysctl.h"

void _delay_ms(unsigned long count)
{
	unsigned long ulClockMS;
	ulClockMS=SysCtlClockGet()/(3*1000);
	SysCtlDelay(count*ulClockMS);
}
void _delay_us(unsigned long count)
{
	unsigned long ulClockMS;
	ulClockMS=SysCtlClockGet()/(3*1000000);
	SysCtlDelay(count*ulClockMS);
}
