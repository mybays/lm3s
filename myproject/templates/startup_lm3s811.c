/*
Startup file for LM3S8962.
Modified in Mar.28.2010
Template file,need modify for interupt function.
*/

//forward declaration of the default fault handlers
void ResetHandler(void);
static void NmiHandler(void);
static void FaultHandler(void);
static void DefaultHandler(void);

#define WEAK __attribute__ ((weak))

void WEAK MPUFaultHandler(void);
void WEAK BusFaultHandler(void);
void WEAK UsageFaultHandler(void);
void WEAK SVCallHandler(void);
void WEAK DebugMonHandler(void);
void WEAK PendSVHandler(void);
void WEAK SysTickIntHandler(void);

void WEAK GPIOAIntHandler(void);
void WEAK GPIOBIntHandler(void);
void WEAK GPIOCIntHandler(void);
void WEAK GPIODIntHandler(void);
void WEAK GPIOEIntHandler(void);
void WEAK UART0IntHandler(void);
void WEAK UART1IntHandler(void);
void WEAK SSI0IntHandler(void);
void WEAK I2C0IntHandler(void);
void WEAK PWMFaultIntHandler(void);
void WEAK PWMGen0IntHandler(void);
void WEAK PWMGen1IntHandler(void);
void WEAK PWMGen2IntHandler(void);
void WEAK QE0IntHandler(void);
void WEAK ADC0IntHandler(void);
void WEAK ADC1IntHandler(void);
void WEAK ADC2IntHandler(void);
void WEAK ADC3IntHandler(void);
void WEAK WatchdogIntHandler(void);
void WEAK Timer0AIntHandler(void);
void WEAK Timer0BIntHandler(void);
void WEAK Timer1AIntHandler(void);
void WEAK Timer1BIntHandler(void);
void WEAK Timer2AIntHandler(void);
void WEAK Timer2BIntHandler(void);
void WEAK Comp0IntHandler(void);
void WEAK Comp1IntHandler(void);
void WEAK Comp2IntHandler(void);
void WEAK SysCtrlIntHandler(void);
void WEAK FlashCtrlIntHandler(void);
//*****************************************************************************
//
// The entry point for the application.
//
//*****************************************************************************
extern int main(void);

//*****************************************************************************
//
// Reserve space for the system stack.
//
//*****************************************************************************
static unsigned long pulStack[64];

//*****************************************************************************
//
// The vector table.  Note that the proper constructs must be placed on this to
// ensure that it ends up at physical address 0x0000.0000.
//
//*****************************************************************************
__attribute__ ((section(".vectors")))
void (* const g_pfnVectors[])(void) =
{
    (void (*)(void))((unsigned long)pulStack + sizeof(pulStack)),
                                            // The initial stack pointer
    ResetHandler,                               // The reset handler
    NmiHandler,                                  // The NMI handler
    FaultHandler,                               // The hard fault handler
    MPUFaultHandler,                      // The MPU fault handler
    BusFaultHandler,                      // The bus fault handler
    UsageFaultHandler,                      // The usage fault handler
    0,                                      // Reserved
    0,                                      // Reserved
    0,                                      // Reserved
    0,                                      // Reserved
    SVCallHandler,                      // SVCall handler
    DebugMonHandler,                      // Debug monitor handler
    0,                                      // Reserved
    PendSVHandler,                      // The PendSV handler
    SysTickIntHandler,                      // The SysTick handler

	//
	//External Interrupts
	//

    GPIOAIntHandler,			// GPIO Port A
	GPIOBIntHandler,			// GPIO Port B
	GPIOCIntHandler,			// GPIO Port C
	GPIODIntHandler,			// GPIO Port D
	GPIOEIntHandler,			// GPIO Port E
	UART0IntHandler,			// UART0 Rx and Tx
	UART1IntHandler,			// UART1 Rx and Tx
	SSI0IntHandler,				// SSI0 Rx and Tx
	I2C0IntHandler,				// I2C0 Master and Slave
	PWMFaultIntHandler,		// PWM Fault
	PWMGen0IntHandler,		// PWM Generator 0
	PWMGen1IntHandler,		// PWM Generator 1
	PWMGen2IntHandler,		// PWM Generator 2
	QE0IntHandler,				// Quadrature Encoder 0
	ADC0IntHandler,			// ADC Sequence 0
	ADC1IntHandler,			// ADC Sequence 1
	ADC2IntHandler,			// ADC Sequence 2
	ADC3IntHandler,			// ADC Sequence 3
	WatchdogIntHandler,		// Watchdog timer
	Timer0AIntHandler,			// Timer 0 subtimer A
	Timer0BIntHandler,			// Timer 0 subtimer B
	Timer1AIntHandler,			// Timer 1 subtimer A
	Timer1BIntHandler,			// Timer 1 subtimer B
	Timer2AIntHandler,			// Timer 2 subtimer A
	Timer2BIntHandler,			// Timer 2 subtimer B
	Comp0IntHandler,			// Analog Comparator 0
	Comp1IntHandler,			// Analog Comparator 1
	Comp2IntHandler,			// Analog Comparator 2
	SysCtrlIntHandler,			// System Control (PLL, OSC, BO)
	FlashCtrlIntHandler,			// FLASH Control
};

//*****************************************************************************
//
// The following are constructs created by the linker, indicating where the
// the "data" and "bss" segments reside in memory.  The initializers for the
// for the "data" segment resides immediately following the "text" segment.
//
//*****************************************************************************
extern unsigned long _etext;
extern unsigned long _data;
extern unsigned long _edata;
extern unsigned long _bss;
extern unsigned long _ebss;

//*****************************************************************************
//
// This is the code that gets called when the processor first starts execution
// following a reset event.  Only the absolutely necessary set is performed,
// after which the application supplied entry() routine is called.  Any fancy
// actions (such as making decisions based on the reset cause register, and
// resetting the bits in that register) are left solely in the hands of the
// application.
//
//*****************************************************************************
void
ResetHandler(void)
{
    unsigned long *pulSrc, *pulDest;

    //
    // Copy the data segment initializers from flash to SRAM.
    //
    pulSrc = &_etext;
    for(pulDest = &_data; pulDest < &_edata; )
    {
        *pulDest++ = *pulSrc++;
    }

    //
    // Zero fill the bss segment.
    //
    __asm("    ldr     r0, =_bss\n"
          "    ldr     r1, =_ebss\n"
          "    mov     r2, #0\n"
          "    .thumb_func\n"
          "zero_loop:\n"
          "        cmp     r0, r1\n"
          "        it      lt\n"
          "        strlt   r2, [r0], #4\n"
          "        blt     zero_loop");

    //
    // Call the application's entry point.
    //
    main();
}

//*****************************************************************************
//
// This is the code that gets called when the processor receives a NMI.  This
// simply enters an infinite loop, preserving the system state for examination
// by a debugger.
//
//*****************************************************************************
static void
NmiHandler(void)
{
    //
    // Enter an infinite loop.
    //
    while(1)
    {
    }
}

//*****************************************************************************
//
// This is the code that gets called when the processor receives a fault
// interrupt.  This simply enters an infinite loop, preserving the system state
// for examination by a debugger.
//
//*****************************************************************************
static void
FaultHandler(void)
{
    //
    // Enter an infinite loop.
    //
    while(1)
    {
    }
}

//*****************************************************************************
//
// This is the code that gets called when the processor receives an unexpected
// interrupt.  This simply enters an infinite loop, preserving the system state
// for examination by a debugger.
//
//*****************************************************************************
static void
DefaultHandler(void)
{
    //
    // Go into an infinite loop.
    //
    while(1)
    {
    }
}

#pragma weak MPUFaultHandler=DefaultHandler
#pragma weak BusFaultHandler=DefaultHandler
#pragma weak UsageFaultHandler=DefaultHandler
#pragma weak SVCallHandler=DefaultHandler
#pragma weak DebugMonHandler=DefaultHandler
#pragma weak PendSVHandler=DefaultHandler
#pragma weak SysTickIntHandler=DefaultHandler
// External Interrupts
#pragma weak GPIOAIntHandler=DefaultHandler
#pragma weak GPIOBIntHandler=DefaultHandler
#pragma weak GPIOCIntHandler=DefaultHandler
#pragma weak GPIODIntHandler=DefaultHandler
#pragma weak GPIOEIntHandler=DefaultHandler
#pragma weak UART0IntHandler=DefaultHandler
#pragma weak UART1IntHandler=DefaultHandler
#pragma weak SSI0IntHandler=DefaultHandler
#pragma weak I2C0IntHandler=DefaultHandler
#pragma weak PWMFaultIntHandler=DefaultHandler
#pragma weak PWMGen0IntHandler=DefaultHandler
#pragma weak PWMGen1IntHandler=DefaultHandler
#pragma weak PWMGen2IntHandler=DefaultHandler
#pragma weak QE0IntHandler=DefaultHandler
#pragma weak ADC0IntHandler=DefaultHandler
#pragma weak ADC1IntHandler=DefaultHandler
#pragma weak ADC2IntHandler=DefaultHandler
#pragma weak ADC3IntHandler=DefaultHandler
#pragma weak WatchdogIntHandler=DefaultHandler
#pragma weak Timer0AIntHandler=DefaultHandler
#pragma weak Timer0BIntHandler=DefaultHandler
#pragma weak Timer1AIntHandler=DefaultHandler
#pragma weak Timer1BIntHandler=DefaultHandler
#pragma weak Timer2AIntHandler=DefaultHandler
#pragma weak Timer2BIntHandler=DefaultHandler
#pragma weak Comp0IntHandler=DefaultHandler
#pragma weak Comp1IntHandler=DefaultHandler
#pragma weak Comp2IntHandler=DefaultHandler
#pragma weak SysCtrlIntHandler=DefaultHandler
#pragma weak FlashCtrlIntHandler=DefaultHandler