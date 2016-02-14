#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/timer.h"
#include "driverlib/pwm.h"
#include "driverlib/interrupt.h"
#include "drivers/rit128x96x4.h"

extern void Timer1ATimeoutIntHandlerUP(void);                                                 //脉宽增大方向中断服务
extern void Timer1ATimeoutIntHandlerDOWN(void);                                                  //脉宽减小方向中断服务
//void RIT128x96x4UnsignedLongDraw(unsigned long figure, unsigned long ulX,                      //以十进制方式显示unsignedlong
//                      unsigned long ulY, unsigned char ucLevel);

int main()
{
    SysCtlClockSet(SYSCTL_SYSDIV_1|SYSCTL_USE_OSC|SYSCTL_OSC_MAIN|SYSCTL_XTAL_8MHZ);  //设置系统时钟8Mhz
    //配置GPIOF引脚
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);                                      //使能GPIOF并将PF0设为PWM输出引脚
    GPIOPinTypePWM(GPIO_PORTF_BASE,GPIO_PIN_0);   
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOG);                                      //使能GPIOG并将PG1设为PWM输出引脚
    GPIOPinTypePWM(GPIO_PORTG_BASE,GPIO_PIN_1);
    //配置PWM模块
    SysCtlPWMClockSet(SYSCTL_PWMDIV_1);                                                  //设置PWM分频
    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM);                                          //使能PWM模块
    PWMGenConfigure(PWM_BASE, PWM_GEN_0,PWM_GEN_MODE_UP_DOWN | PWM_GEN_MODE_NO_SYNC); //配置PWMGEN0,计数器为先增后减计数，非同步更新
    PWMGenPeriodSet(PWM_BASE, PWM_GEN_0,SysCtlClockGet()/500);                      //将PWMGEN0频率设为500HZ
    //PWMGenPeriodSet(PWM_BASE, PWM_GEN_0,SysCtlClockGet()/1000);                      //将PWMGEN0频率设为1000HZ
    PWMPulseWidthSet(PWM_BASE,PWM_OUT_0,0);                                              //设置PWM0脉宽
    PWMPulseWidthSet(PWM_BASE,PWM_OUT_1,0);                                              //设置PWM1脉宽
    PWMOutputState(PWM_BASE,PWM_OUT_0_BIT,true);                          //使能PWM0引脚PWM1引脚
    PWMGenEnable(PWM_BASE,PWM_GEN_0);                                                  //使能PWMGEN0
   
	RIT128x96x4Init(1000000);
	RIT128x96x4StringDraw("hello world",1,1,15);    //配置定时器
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER1);                                     
    TimerConfigure(TIMER1_BASE,TIMER_CFG_32_BIT_PER);
    TimerLoadSet(TIMER1_BASE,TIMER_A,0x0000ffff);
    TimerIntEnable(TIMER1_BASE,TIMER_TIMA_TIMEOUT);   
    TimerIntRegister(TIMER1_BASE,TIMER_A,Timer1ATimeoutIntHandlerUP);                   //设置TIMER1ATIMEOUT中断向量为Timer1ATimeoutIntHandlerUP
    TimerEnable(TIMER1_BASE,TIMER_A);

    while(1);
}

//脉宽增大方向中断服务
void Timer1ATimeoutIntHandlerUP(void)
{
    TimerIntClear(TIMER1_BASE,TIMER_TIMA_TIMEOUT);                //清除中断标志位
    if(PWMGenPeriodGet(PWM_BASE,PWM_GEN_0)/8 < PWMPulseWidthGet(PWM_BASE,PWM_OUT_0))     //当占空比大于12.5%时改变中断向量
    {
        TimerIntRegister(TIMER1_BASE,TIMER_A,Timer1ATimeoutIntHandlerDOWN);                 //改变中断向量为Timer1ATimeoutIntHandlerDOWN
    }
    else
    {
        PWMPulseWidthSet(PWM_BASE,PWM_OUT_0,PWMPulseWidthGet(PWM_BASE,PWM_OUT_0)+10);      //增大脉宽
        PWMPulseWidthSet(PWM_BASE,PWM_OUT_1,PWMPulseWidthGet(PWM_BASE,PWM_OUT_1)+10);
    }
}
//脉宽减小方向中断服务
void Timer1ATimeoutIntHandlerDOWN(void)
{
    TimerIntClear(TIMER1_BASE,TIMER_TIMA_TIMEOUT);                //清除中断标志位
    if(PWMPulseWidthGet(PWM_BASE,PWM_OUT_0)<10)                                            //脉宽小于10时改变中断向量
    {                                                                                    //改变中断向量为Timer1ATimeoutIntHandlerUP
        TimerIntRegister(TIMER1_BASE,TIMER_A,Timer1ATimeoutIntHandlerUP);
    }
    else
    {
        PWMPulseWidthSet(PWM_BASE,PWM_OUT_0,PWMPulseWidthGet(PWM_BASE,PWM_OUT_0)-10);      //减小脉宽
        PWMPulseWidthSet(PWM_BASE,PWM_OUT_1,PWMPulseWidthGet(PWM_BASE,PWM_OUT_1)-10);
    }
}
