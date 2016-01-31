.equ STACK_TOP,0x20000800
.equ SYSCTL_RCGC2_R,		0x400FE108
.equ GPIO_PORTF_DIR,		0x40025400
.equ GPIO_PORTF_DEN,	0x4002551C
.equ GPIO_PORTF_DATA,	0x400253FC


.text
.global _start
.thumb					/*equal to ".code 16"*/
.syntax unified

_start:
.word STACK_TOP				/*栈顶*/
.word start+1				;
.word NmiHandler+1		;
.word FaultHandler+1		;
.word MPUFaultHandler+1	;
.word BusFaultHandler+1	;
.word UsageFaultHandler+1	;


.type start,function
start:
	movs r0,#0
	movs r1,#0
	movs r2,#0
	movs r3,#0
	movs r4,#0
	movs r5,#0
	ldr r6,=SYSCTL_RCGC2_R
	mov r0,0x20
	str r0,[r6]

	ldr r6,=GPIO_PORTF_DIR
	mov r0,0x1
	str r0,[r6]
	ldr r6,=GPIO_PORTF_DEN
	ldr r0,=0x1
	str r0,[r6]
	ldr r6,=GPIO_PORTF_DATA
	ldr r0,=0x1
	str r0,[r6]

loop:
	adds r1,r0
	subs r0,#1
	bne loop
deadloop:
	b deadloop


NmiHandler:
	b NmiHandler
FaultHandler:
	b NmiHandler
MPUFaultHandler:
	b NmiHandler
BusFaultHandler:
	b NmiHandler
UsageFaultHandler:
	b NmiHandler

.end

