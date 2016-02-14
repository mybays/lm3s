//*****************************************************************************
//
// uart_echo.c - Example for reading data from and writing data to the UART in
//               an interrupt driven fashion.
//
// Copyright (c) 2005-2010 Texas Instruments Incorporated.  All rights reserved.
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
// This is part of revision 6459 of the EK-LM3S8962 Firmware Package.
//
//*****************************************************************************
#include "inc/hw_sysctl.h"
#include "inc/hw_types.h"


#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"


#include "driverlib/debug.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "inc/lm3s8962.h"
//*****************************************************************************
//
//! \addtogroup example_list
//! <h1>UART Echo (uart_echo)</h1>
//!
//! This example application utilizes the UART to echo text.  The first UART
//! (connected to the FTDI virtual serial port on the evaluation board) will be
//! configured in 115,200 baud, 8-n-1 mode.  All characters received on the
//! UART are transmitted back to the UART.
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

void systemprobe(void);

//*****************************************************************************
//
// The UART interrupt handler.
//
//*****************************************************************************
void
UART0IntHandler(void)
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

//*****************************************************************************
//
// Send a string to the UART.
//
//*****************************************************************************
void
UARTSend(const unsigned char *pucBuffer, unsigned long ulCount)
{
    //
    // Loop while there are more characters to send.
    //
    while(ulCount--)
    {
        //
        // Write the next character to the UART.
        //
        UARTCharPutNonBlocking(UART0_BASE, *pucBuffer++);
    }
}

//*****************************************************************************
//
// This example demonstrates how to send a string of data to the UART.
//
//*****************************************************************************
//FILE uart_str = FDEV_SETUP_STREAM(uart_putchar, uart_getchar, _FDEV_SETUP_RW);

int
main(void)
{
     SysCtlClockSet(SYSCTL_SYSDIV_4 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_8MHZ);


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

    //
    // Prompt for text to be entered.
    //
    UARTSend((unsigned char *)"Enter text: ", 12);
    UARTStdioInit(0);
    UARTprintf("Hello,world\n");
    	unsigned long ulClockMS,ulCount;
	unsigned short usController;

	{
	ulClockMS=SysCtlClockGet();
    UARTprintf("%d\n",ulClockMS);

    SysCtlDelay(2000000);
    }
    //
    // Loop forever echoing data through the UART.
    //
    systemprobe();
    while(1)
    {
//        UARTprintf("%d\n",ulClockMS);
    }
}

void systemprobe(void)
{
//	UARTprintf("0x%x",HWREG
	//hw_types.h
	if(CLASS_IS_SANDSTORM)
		UARTprintf("System is sandstorm.\n");
	if(CLASS_IS_FURY)
		UARTprintf("System is fury.\n");
	if(CLASS_IS_DUSTDEVIL)
		UARTprintf("System is dustdevil.\n");
	if(CLASS_IS_TEMPEST)
		UARTprintf("System is tempest.\n");
	if(CLASS_IS_FIRESTORM)
	if(REVISION_IS_A0)
		UARTprintf("System is A0.\n");
	if(REVISION_IS_A1)
		UARTprintf("System is A1.\n");
	if(REVISION_IS_A2)
		UARTprintf("System is A2.\n");
	if(REVISION_IS_B0)
		UARTprintf("System is B0.\n");
	if(REVISION_IS_B1)
		UARTprintf("System is B1.\n");
	if(REVISION_IS_C0)
		UARTprintf("System is C0.\n");
	if(REVISION_IS_C1)
		UARTprintf("System is C1.\n");
	if(REVISION_IS_C2)
		UARTprintf("System is C2.\n");
	if(REVISION_IS_C3)
		UARTprintf("System is C3.\n");

}
