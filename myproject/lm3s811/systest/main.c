#include "inc/lm3s811.h"
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/debug.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "driverlib/pwm.h"
#include "driverlib/timer.h"
#include "driverlib/systick.h"
#include "utils/uartstdio.h"



#ifdef DEBUG
void
__error__(char *pcFilename, unsigned long ulLine)
{
}
#endif


//*****************************************************************************
//
// The UART interrupt handler.
//
//*****************************************************************************
void UART0IntHandler(void)
{
    unsigned long ulStatus;

    //
    // Get the interrrupt status.
    //
    ulStatus = UARTIntStatus(UART0_BASE, true);

    //
    // Clear the asserted interrupts.
    //
    UARTIntClear(UART0_BASE, ulStatus);

    //
    // Loop while there are characters in the receive FIFO.
    //
    while(UARTCharsAvail(UART0_BASE))
    {
        //
        // Read the next character from the UART and write it back to the UART.
        //
        UARTCharPutNonBlocking(UART0_BASE, UARTCharGetNonBlocking(UART0_BASE));
    }
}

int test=1;

void SysTickIntHandler(void)
{
    UARTprintf("systickint.\r\n");
    if(test)
    {
        GPIO_PORTC_DATA_R |= 0x20;
        test=0;
    }
    else
    {
        GPIO_PORTC_DATA_R &= ~(0x20);
        test=1;
    }
    
    

}

void
Timer0AIntHandler(void)
{
    //
    // 清除 Timer 中断
    //
    TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
}

//*****************************************************************************
//
// This example demonstrates how to send a string of data to the UART.
//
//*****************************************************************************
int
main(void)
{

    //直接使用外部晶振:6MHz
    //SysCtlClockSet(SYSCTL_SYSDIV_1 | SYSCTL_USE_OSC | SYSCTL_OSC_MAIN | SYSCTL_XTAL_6MHZ);
    //使用外部晶振,6MHz,锁相环4分频:50MHz
    SysCtlClockSet(SYSCTL_SYSDIV_4 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_6MHZ);
    //使用内部晶振
    //SysCtlClockSet(SYSCTL_SYSDIV_1 | SYSCTL_USE_OSC| SYSCTL_OSC_INT| SYSCTL_XTAL_12MHZ);

    //
    // Enable the peripherals used by this example.
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

    //
    // Enable processor interrupts.
    //
    IntMasterEnable();

    //
    // Set GPIO A0 and A1 as UART pins.
    //
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    //
    // Configure the UART for 115,200, 8-N-1 operation.
    //
    UARTConfigSetExpClk(UART0_BASE, SysCtlClockGet(), 115200,
                        (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
                         UART_CONFIG_PAR_NONE));

    //
    // Enable the UART interrupt.
    //
    IntEnable(INT_UART0);
    UARTIntEnable(UART0_BASE, UART_INT_RX | UART_INT_RT);


    unsigned long clock;
    clock=SysCtlClockGet();
    UARTStdioInit(0);
    UARTprintf("clockfrequnecy:%d Hz.\r\n",clock);

    //
    // Loop forever echoing data through the UART.
    //

    //pwm test
    unsigned long ulPeriod;
    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);

    GPIOPinTypePWM(GPIO_PORTD_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    ulPeriod = SysCtlClockGet() / 50000;

    PWMGenConfigure(PWM0_BASE, PWM_GEN_0,
                    PWM_GEN_MODE_UP_DOWN | PWM_GEN_MODE_NO_SYNC);
    PWMGenPeriodSet(PWM0_BASE, PWM_GEN_0, ulPeriod);

    //
    // Set PWM0 to a duty cycle of 25% and PWM1 to a duty cycle of 75%.
    //
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0, ulPeriod / 4);
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_1, ulPeriod * 3 / 4);

    //
    // Enable the PWM0 and PWM1 output signals.
    //
    PWMOutputState(PWM0_BASE, PWM_OUT_0_BIT | PWM_OUT_1_BIT, true);
    PWMGenEnable(PWM0_BASE, PWM_GEN_0);

    //RED LED
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);//启用 GPIO C
    GPIOPinTypeGPIOOutput(GPIO_PORTC_BASE, GPIO_PIN_5 );//配置PC5为输出

    //SysTick
    SysTickPeriodSet(SysCtlClockGet() / 2);
    SysTickIntEnable();
    SysTickEnable();
    while(1)
    {
/*
        //Turn on LED
        GPIO_PORTC_DATA_R |= 0x20;
        //DELAY
        for(ulLoop = 0; ulLoop < 800000; ulLoop++)
        {
        }
        //Turn off LED
        GPIO_PORTC_DATA_R &= ~(0x20);
        //DELAY
        for(ulLoop = 0; ulLoop < 200000; ulLoop++)
        {
        }
*/
    }
}
