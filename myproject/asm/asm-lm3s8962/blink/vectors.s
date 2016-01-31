NmiHandler:
	b NmiHandler					/* NMI handler */
FaultHandler:
	b FaultHandler				/* hard fault handler */
MPUFaultHandler:
	b MPUFaultHandler			/* MPU fault handler */
BusFaultHandler:
	b BusFaultHandler				/* bus fault handler */
UsageFaultHandler:
	b UsageFaultHandler			/* usage fault handler */
SVCallHandler:
	b SVCallHandler				/* SVCall handler */
DebugMonHandler:
	b DebugMonHandler			/* Debug monitor handler */
PendSVHandler:
	b PendSVHandler				/* PendSV handler */
SysTickIntHandler:
	b SysTickIntHandler			/* SysTick handler */
GPIOAIntHandler:
	b GPIOAIntHandler			/* GPIO Port A */
GPIOBIntHandler:
	b GPIOBIntHandler				/* GPIO Port B */
GPIOCIntHandler:
	b GPIOCIntHandler			/* GPIO Port C */
GPIODIntHandler:
	b GPIODIntHandler			/* GPIO Port D */
GPIOEIntHandler:
	b GPIOEIntHandler				/* GPIO Port E */
UART0IntHandler:
	b UART0IntHandler			/* UART0 Rx and Tx */
UART1IntHandler:
	b UART1IntHandler			/* UART1 Rx and Tx */
SSI0IntHandler:
	b SSI0IntHandler				/* SSI0 Rx and Tx */
I2C0IntHandler:
	b I2C0IntHandler				/* I2C0 Master and Slave */
PWMFaultIntHandler:
	b PWMFaultIntHandler			/* PWM Fault */
PWMGen0IntHandler:
	b PWMGen0IntHandler			/* PWM Generator 0 */
PWMGen1IntHandler:
	b PWMGen1IntHandler			/* PWM Generator 1 */
PWMGen2IntHandler:
	b PWMGen2IntHandler			/* PWM Generator 2 */
QE0IntHandler:
	b QE0IntHandler				/* Quadrature Encoder 0 */
ADC0IntHandler:
	b ADC0IntHandler				/* ADC Sequence 0 */
ADC1IntHandler:
	b ADC1IntHandler				/* ADC Sequence 1 */
ADC2IntHandler:
	b ADC2IntHandler				/* ADC Sequence 2 */
ADC3IntHandler:
	b ADC3IntHandler				/* ADC Sequence 3 */
WatchdogIntHandler:
	b WatchdogIntHandler			/* Watchdog timer */
Timer0AIntHandler:
	b Timer0AIntHandler			/* Timer 0 subtimer A */
Timer0BIntHandler:
	b Timer0BIntHandler			/* Timer 0 subtimer B */
Timer1AIntHandler:
	b Timer1AIntHandler			/* Timer 1 subtimer A */
Timer1BIntHandler:
	b Timer1BIntHandler			/* Timer 1 subtimer B */
Timer2AIntHandler:
	b Timer2AIntHandler			/* Timer 2 subtimer A */
Timer2BIntHandler:
	b Timer2BIntHandler			/* Timer 2 subtimer B */
Comp0IntHandler:
	b Comp0IntHandler			/* Analog Comparator 0 */
Comp1IntHandler:
	b Comp1IntHandler			/* Analog Comparator 1 */
Comp2IntHandler:
	b Comp2IntHandler			/* Analog Comparator 2 */
SysCtrlIntHandler:
	b SysCtrlIntHandler			/* System Control (PLL, OSC, BO) */
FlashCtrlIntHandler:
	b FlashCtrlIntHandler			/* FLASH Control */
GPIOFIntHandler:
	b GPIOFIntHandler				/* GPIO Port F */
GPIOGIntHandler:
	b GPIOGIntHandler			/* GPIO Port G */
GPIOHIntHandler:
	b GPIOHIntHandler			/* GPIO Port H */
UART2IntHandler:
	b UART2IntHandler			/* UART2 Rx and Tx */
SSI1IntHandler:
	b SSI1IntHandler				/* SSI1 Rx and Tx */
Timer3AIntHandler:
	b Timer3AIntHandler			/* Timer 3 subtimer A */
Timer3BIntHandler:
	b Timer3BIntHandler			/* Timer 3 subtimer B */
I2C1IntHandler:
	b I2C1IntHandler				/* I2C1 Master and Slave */
QE1IntHandler:
	b QE1IntHandler				/* Quadrature Encoder 1 */
CAN0IntHandler:
	b CAN0IntHandler				/* CAN 0 */
CAN1IntHandler:
	b CAN1IntHandler				/* CAN 1 */
CAN2IntHandler:
	b CAN2IntHandler				/* CAN 2 */
EthernetIntHandler:
	b EthernetIntHandler			/* Ethernet */
HibernateIntHandler:
	b HibernateIntHandler			/* Hibernate */
