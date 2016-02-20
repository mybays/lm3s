//SysCtlDelay
//系统延时函数SysCtlDelay延时3个时钟周期

//#include "inc/hw_ints.h"
#include "inc/hw_types.h"
#include "driverlib/sysctl.h"

//RAMRUN要比romrun慢一半
#ifdef RAMRUN
#define MSCOUNT 2000
#define USCOUNT 2000000
#else
#define MSCOUNT 1000
#define USCOUNT 1000000
#endif

void _delay_ms(unsigned long count)
{
	unsigned long ulClockMS;
	ulClockMS=SysCtlClockGet()/(3*MSCOUNT);
	SysCtlDelay(count*ulClockMS);
}
void _delay_us(unsigned long count)
{
	unsigned long ulClockMS;
	ulClockMS=SysCtlClockGet()/(3*USCOUNT);
	SysCtlDelay(count*ulClockMS);
}
