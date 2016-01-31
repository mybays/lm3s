/*EK-LM3S8962 LED Blink example.*/
/********************************************************
http://pygmy.utoh.org/riscy/cortex/led-stm32.html
http://tech.groups.yahoo.com/group/lpc2000/message/52571
http://pramode.net/fosstronics/arm-assembly-programming.txt

********************************************************/

.include "def.h"

	.syntax unified
	.cpu cortex-m3
	.fpu softvfp
	.thumb			/*equal to ".code 16"*/

.section	.text
	.org 0
	.align 2
	.global _start

_start:
.include "vectors.h"

.type ResetHandler,function
/*在LM3S8962中,加上向量表,此处的地址应该是0x200000f0*/
ResetHandler:
	ldr sp,=STACK_TOP		/*由于在RAM中运行,所以需要手动设置栈顶*/

	push {r0}
	pop {r1}
	ldr r6,=SYSCTL_RCGC2_R
	mov r0,0x20
	str r0,[r6]

	mov r0,0x1
	ldr r6,=GPIO_PORTF_DIR
	str r0,[r6]

	ldr r6,=GPIO_PORTF_DEN
	str r0,[r6]

	ldr r6,=GPIO_PORTF_DATA
	str r0,[r6]

	mov r2,0
	mov r3,0x1
loops:
	str r2,[r6]
	ldr r1,=LEDDELAY
delay1:
	subs r1,1				/*影响Z标志位,当r1=0时,Z=1*/
	bne delay1			/*当Z=1时,执行下一条指令*/

	str r3,[r6]
	ldr r1,=LEDDELAY
delay2:
	subs r1,1
	bne delay2
	
	b loops


loop:
	adds r1,r0
	subs r0,#1
	bne loop
deadloop:
	b deadloop

	.include "vectors.s"

.end
