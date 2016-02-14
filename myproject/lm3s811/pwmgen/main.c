//*****************************************************************************
//
// pwmgen.c - PWM signal generation example.
//
// Copyright (c) 2005-2013 Texas Instruments Incorporated.  All rights reserved.
// Software License Agreement
// 
// Texas Instruments (TI) is supplying this software for use solely and
// exclusively on TI's microcontroller products. The software is owned by
// TI and/or its suppliers, and is protected under applicable copyright
// laws. You may not combine this software with "viral" open-source
// software in order to form a larger program.
// 
// THIS SOFTWARE IS PROVIDED "AS IS" AND WITH ALL FAULTS.
// NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT
// NOT LIMITED TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. TI SHALL NOT, UNDER ANY
// CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR CONSEQUENTIAL
// DAMAGES, FOR ANY REASON WHATSOEVER.
// 
// This is part of revision 10636 of the EK-LM3S811 Firmware Package.
//
//*****************************************************************************
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
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/debug.h"
#include "driverlib/gpio.h"
#include "driverlib/pwm.h"
#include "driverlib/sysctl.h"
//#include "drivers/display96x16x1.h"

//*****************************************************************************
//
//! \addtogroup example_list
//! <h1>PWM (pwmgen)</h1>
//!
//! This example application utilizes the PWM peripheral to output a 25% duty
//! cycle PWM signal and a 75% duty cycle PWM signal, both at 50 kHz.  Once
//! configured, the application enters an infinite loop, doing nothing while
//! the PWM peripheral continues to output its signals.
//
//*****************************************************************************

//*****************************************************************************
//
// The error routine that is called if the driver library encounters an error.
//
//*****************************************************************************
#ifdef DEBUG
void
__error__(char *pcFilename, unsigned long ulLine)
{
}
#endif

//*****************************************************************************
//
// This example demonstrates how to setup the PWM block to generate signals.
//
//*****************************************************************************
int
main(void)
{
    unsigned long ulPeriod;

    //
    // Set the clocking to run directly from the crystal.
    //
    SysCtlClockSet(SYSCTL_SYSDIV_1 | SYSCTL_USE_OSC | SYSCTL_OSC_MAIN |
                   SYSCTL_XTAL_6MHZ);
    SysCtlPWMClockSet(SYSCTL_PWMDIV_1);

    //
    // Initialize the OLED display.
    //
    //Display96x16x1Init(false);

    //
    // Clear the screen and thell the user what is happening.
    //
    //Display96x16x1StringDraw("Generating PWM", 6, 0);
    //Display96x16x1StringDraw("on PD0 and PD1", 6, 1);

    

    //
    // Enable the peripherals used by this example.
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);
    //SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM1);
    //SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM2);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
    
    //
    // Set GPIO D0 and D1 as PWM pins.  They are used to output the PWM0 and
    // PWM1 signals.
    //
    GPIOPinTypePWM(GPIO_PORTD_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    GPIOPinTypePWM(GPIO_PORTB_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    GPIOPinTypePWM(GPIO_PORTE_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    
    //
    // Compute the PWM period based on the system clock.
    //
    //ulPeriod = SysCtlClockGet() / 50000;

    ulPeriod = 10000;

    //
    // Set the PWM period to 50 kHz.
    //
    PWMGenConfigure(PWM0_BASE, PWM_GEN_0,
                    PWM_GEN_MODE_UP_DOWN | PWM_GEN_MODE_NO_SYNC);
    PWMGenPeriodSet(PWM0_BASE, PWM_GEN_0, ulPeriod);
    PWMGenConfigure(PWM0_BASE, PWM_GEN_1,
                    PWM_GEN_MODE_UP_DOWN | PWM_GEN_MODE_NO_SYNC);
    PWMGenPeriodSet(PWM0_BASE, PWM_GEN_1, ulPeriod);
    PWMGenConfigure(PWM0_BASE, PWM_GEN_2,
                    PWM_GEN_MODE_UP_DOWN | PWM_GEN_MODE_NO_SYNC);
    PWMGenPeriodSet(PWM0_BASE, PWM_GEN_2, ulPeriod);
    
    //
    // Set PWM0 to a duty cycle of 25% and PWM1 to a duty cycle of 75%.
    //
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0, 0);
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_1, 0);
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2, 0);
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_3, 0);
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_4, 0);
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_5, 0);
    

    //
    // Enable the PWM0 and PWM1 output signals.
    //
    PWMOutputState(PWM0_BASE, PWM_OUT_0_BIT | PWM_OUT_1_BIT, true);
    PWMOutputState(PWM0_BASE, PWM_OUT_2_BIT | PWM_OUT_3_BIT, true);
    PWMOutputState(PWM0_BASE, PWM_OUT_4_BIT | PWM_OUT_5_BIT, true);
    
    //
    // Enable the PWM generator.
    //
    PWMGenEnable(PWM0_BASE, PWM_GEN_0);
    PWMGenEnable(PWM0_BASE, PWM_GEN_1);
    PWMGenEnable(PWM0_BASE, PWM_GEN_2);
    
    //
    // Loop forever while the PWM signals are generated.
    //
    unsigned long count=1;
    //unsigned long ulLoop;


    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);//启用 GPIO C
    GPIOPinTypeGPIOOutput(GPIO_PORTC_BASE, GPIO_PIN_5 );//配置PC5为输出
    volatile unsigned long ulLoop;

    SYSCTL_RCGC2_R = SYSCTL_RCGC2_GPIOC;
    ulLoop = SYSCTL_RCGC2_R;
    GPIO_PORTC_DIR_R |= 0x20;
    GPIO_PORTC_DEN_R |= 0x20;

    int dir=0;

    while(1)
    {
        if(count==ulPeriod)
        {
            dir=0;
        }
        if(count==0)
        {
            dir=1;
        }
        if(dir==0)
        {
            count--;
        }
        if(dir==1)
        {
            count++;
        }
        PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0, count%ulPeriod);
        PWMPulseWidthSet(PWM0_BASE, PWM_OUT_1, count%ulPeriod);
        PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2, count%ulPeriod);
        PWMPulseWidthSet(PWM0_BASE, PWM_OUT_3, count%ulPeriod);
        PWMPulseWidthSet(PWM0_BASE, PWM_OUT_4, count%ulPeriod);
        PWMPulseWidthSet(PWM0_BASE, PWM_OUT_5, count%ulPeriod);
        
        GPIO_PORTC_DATA_R &= ~(0x20);
        //DELAY
        for(ulLoop = 0; ulLoop < 100; ulLoop++)
        {
        }
        //Turn off LED
        GPIO_PORTC_DATA_R |= 0x20;
        //DELAY
        for(ulLoop = 0; ulLoop < 100; ulLoop++)
        {
        }
    }
}
