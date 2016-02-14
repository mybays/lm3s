// 本例程演示了如何使用LM3S8962EK评估板来测试片内ADC模块和Timer模块. 为了方便
// 测试,ADC的输入使用内部的温度传感器. Timer0被配置成2个16位定时器,其中TimerA
// 被用来产生一个定时器中断,在定时器中断中触发ADC采样/转换,转换结果经过计算后得到
// 芯片内核温度并进一步显示在EK板上的OLED上.如果你有可变电阻,也可以接在ADC的通道0,
// 程序需做简单调整,更方便ADC的测试和评估.
// ADC也可以配置成由TimerA直接触发.
//

//*****************************************************************************

#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"

#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/sysctl.h"
#include "driverlib/adc.h"
#include "driverlib/timer.h"

//#include "utils/rit128x96x4.h"
//#include "utils/ustdlib.h"

#ifdef DEBUG
void
__error__(char *pcFilename, unsigned long ulLine)
{
}
#endif


unsigned char adc_busy=0;

void Timer0AIntHandler()
{
  // 清中断标志
  TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);    
  
  //定时时间到,检查上一次AD转换是否完成,如果完成,进行新的转换
  if(adc_busy==0)
  {
      // 触发AD转换,序列0
      // ADCProcessorTrigger (unsigned long ulBase, unsigned long ulSequenceNum)
      ADCProcessorTrigger(ADC0_BASE, 0);
      adc_busy=1;
  }
  
  
}


void ADC0IntHandler()
{

    unsigned long adc0Value;   // Holds the ADC result
    char adc0String[5];        // Holds the string-converted ADC result

    // 清ADC0中断标志.
    // ADCIntClear (unsigned long ulBase, unsigned long ulSequenceNum) 
    ADCIntClear(ADC0_BASE, 0);
    
    //从SSO读出转换结果 (FIFO0),本例中只有一个采样.如果要使用多个转换源,需要使用数组做为参数传递给API函数,保存FIFO转换结果.
    // long ADCSequenceDataGet (unsigned long ulBase,unsigned long ulSequenceNum,unsigned long *pulBuffer) 
    ADCSequenceDataGet(ADC0_BASE, 0, &adc0Value);
    adc0Value= ((59960 - (adc0Value * 100)) / 356);
    
    
    // 在OLED上显示当前温度
    usprintf(adc0String, "%d", adc0Value);    
    IntMasterDisable();
    RIT128x96x4StringDraw("Current temp :         ", 6, 48, 15);
    RIT128x96x4StringDraw(adc0String, 94, 48, 15);
    IntMasterEnable();  

    // ADC模块空闲,可以进行下一次转换
    adc_busy=0;    
 
}


void Init_Timer_A(void)
{
  // 使能TIMER0模块
  SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0); 
  
  // 禁止定时器A
  // TimerDisable (unsigned long ulBase, unsigned long ulTimer)
  TimerDisable(TIMER0_BASE, TIMER_A);
  
  // 配置TIMER0模块工作在双16位定时器,周期工作模式
  // TimerConfigure (unsignedlongulBase,unsignedlongulConfig)
  TimerConfigure(TIMER0_BASE, TIMER_CFG_16_BIT_PAIR | TIMER_CFG_A_PERIODIC);
  
  // 配置8位分频器
  // TimerPrescaleSet (unsigned long ulBase, unsigned long ulTimer, unsigned long ulValue)
  TimerPrescaleSet( TIMER0_BASE, TIMER_A,100);
  
  // 配置定时器周期
  // TimerLoadSet (unsigned long ulBase, unsigned long ulTimer, unsigned long ulValue)
  TimerLoadSet(TIMER0_BASE, TIMER_A, 50000);
  
  // 使能时钟溢出中断
  // TimerIntEnable (unsigned long ulBase, unsigned long ulIntFlags) 
  TimerIntEnable(TIMER0_BASE,TIMER_TIMA_TIMEOUT);
  
  // 在NVIC中使能时钟中断
  IntEnable(INT_TIMER0A);
  
  // 使能TIMER A
  // TimerEnable (unsigned long ulBase, unsigned long ulTimer)
  TimerEnable(TIMER0_BASE, TIMER_A);  
  
}

void Init_ADC0(void)
{
  
 
  // 使能ADC0模块
  SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
  
  // 配置前先关闭序列0
  // ADCSequenceDisable (unsigned long ulBase, unsigned long ulSequenceNum) 
  ADCSequenceDisable(ADC0_BASE, 0); 
  
  // 配置SS0: 由CPU软件触发,优先级0(最高)
  // ADCSequenceConfigure(unsigned long ulBase,unsigned long ulSequenceNum, unsigned long ulTrigger, unsigned long ulPriority)
  ADCSequenceConfigure(ADC0_BASE, 0,ADC_TRIGGER_PROCESSOR,0); 
  
  // 配置序列里面的每一个成员配置输入通道,是否中断,是否最后采样.本例序列中只有一个采样:ulstep=0,来自内置温度传感器TS
  // ADCSequenceStepConfigure(unsigned long ulBase,unsigned long ulSequenceNum,unsigned long ulStep,unsigned long ulConfig)
  ADCSequenceStepConfigure(ADC0_BASE, 0, 0, ADC_CTL_TS |ADC_CTL_IE| ADC_CTL_END ); //
  
  // 使能模块中断
  // ADCIntEnable (unsigned long ulBase, unsigned long ulSequenceNum) 
  ADCIntEnable(ADC0_BASE,0);
  
  // 在NVIC中使能ADC中断
  IntEnable(INT_ADC0);
  
  // 使能序列0
  // ADCSequenceEnable (unsigned long ulBase, unsigned long ulSequenceNum) 
  ADCSequenceEnable(ADC0_BASE, 0);
  
  
  
}

Init_OLED(void)
{
    //*****************************************************************************
    // Display Setup
    //*****************************************************************************  

    // Setup the OLED controller interface clock to 1MHz
    RIT128x96x4Init(1000000);

    // Print the header "Periodic ADC" on the display
    RIT128x96x4StringDraw("Periodic ADC", 18, 24, 15);
}

int main()
{
  //系统时钟设置.
  //对于LM3S8962来说,外部晶体只能在1-8MHZ之间.但是ADC需要14-18Mhz才能正常工作,所以必须使用PLL
  //PLL频率400MHZ,预先被2分频输出,所以SYSDIV_4之后得到50MHZ.
  //该函数配置系统使用8M晶体,使能PLL,系统时钟50M. ADC的时钟直接由PLL/25(400/25=18Mhz)获得
  
  
  SysCtlClockSet(SYSCTL_SYSDIV_4 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_8MHZ); 
  //SysCtlClockSet(SYSCTL_SYSDIV_1 | SYSCTL_USE_OSC | SYSCTL_OSC_MAIN | SYSCTL_XTAL_8MHZ); 
  
  Init_OLED();
  Init_ADC0();
  Init_Timer_A();
  
 
  while(1)
  {

  };
  

}
