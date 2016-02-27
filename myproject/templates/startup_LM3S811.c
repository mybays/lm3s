/*
Startup file for LM3S811.
Modified in Feb.12.2016
Template file,need modify for interupt function.
*/

//forward declaration of the default fault handlers
void ResetHandler(void);
static void NmiHandler(void);
static void FaultHandler(void);
static void DefaultHandler(void);

#define WEAK __attribute__ ((weak))


//void WEAK ResetHandler(void);
//void WEAK NmiHandler(void);
//void WEAK FaultHandler(void);


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



//entry point for app
extern int main(void);

static unsigned long pulStack[64];


//constructs from linker
extern unsigned long _text;
extern unsigned long _etext;
extern unsigned long _data;
extern unsigned long _edata;
extern unsigned long _bss;
extern unsigned long _ebss;
extern unsigned long _heap;
extern unsigned long _eheap;
extern unsigned long _stack;
extern unsigned long _estack;


//vectors table
__attribute__ ((section(".vectors")))
void (* const _vectorstable[])(void) =
{
//	&_estack,					// initial stack pointer
	(void (*)(void))((unsigned long)pulStack + sizeof(pulStack)),
	ResetHandler,				// Reset handler
	NmiHandler,				// NMI handler
	FaultHandler,				// hard fault handler
	MPUFaultHandler,			// MPU fault handler
	BusFaultHandler,			// bus fault handler
	UsageFaultHandler,			// usage fault handler
	0,						// reserved
	0,						// reserved
	0,						// reserved
	0,						// reserved
	SVCallHandler,				// SVCall handler
	DebugMonHandler,			// Debug monitor handler
	0,						// reserved
	PendSVHandler,			// PendSV handler
	SysTickIntHandler,			// SysTick handler

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
	FlashCtrlIntHandler			// FLASH Control
};




//Reset Handler
__attribute__ ((section(".init")))
__attribute__ ((naked))
void ResetHandler(void)
{
	unsigned long *src,*dst;

	//set stack pointer
	__asm(	"ldr r0,=%0\n"
			"mov sp,r0\n"
			:
			: "i" (&_estack)
		);

	//set vector table address
	__asm(	"ldr r0,=%0\n"
			"ldr r1,=0xe000ed08\n"
			"str r0,[r1]\n"
			:
			: "i" (&_text)
		);

	//copy the data segment initializers from flash to RAM
	src=&_etext;
	for(dst=(unsigned long*)&_data;dst<(unsigned long*)&_edata;)
	{
		*dst++=*src++;
	}

	//zero fill the bss segment
	__asm(	"ldr r0,=_bss\n"
			"ldr r1,=_ebss\n"
			"mov r2,#0\n"
			".thumb_func\n"
			"zero_loop:\n"
			"cmp r0,r1\n"
			"it lt\n"
			"strlt r2,[r0],#4\n"
			"blt zero_loop"
		);

	//jump to app`s entry point
	__asm(	"mov r0,#0\n"
			"mov r1,#0\n"
			"ldr r2,=%0\n"
			"blx r2\n"
			:
			: "i" (main)
		);

	//dead loop,never reach this point
	while(1);
}




//NMI Handler
static void NmiHandler(void)
{
	while(1);
}

//Fault Handler
static void FaultHandler(void)
{
	while(1);
}

//Default Handler
static void DefaultHandler(void)
{
	while(1);
}


//*****************************************************************************
//
// Provide weak aliases for each Exception handler to the Default_Handler.
// As they are weak aliases, any function with the same name will override
// this definition.
//
//*****************************************************************************
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