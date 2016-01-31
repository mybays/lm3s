.text
.global _start
.code 16
.syntax unified
.type _start,function
_start:
	movs r0,#0
	movs r1,#0
	movs r2,#0
	movs r3,#0
	movs r4,#0
	movs r5,#0
	ldr r0,=hello
	bl puts
	movs r0,#0x4
	bl putc
deadloop:
	b deadloop

hello:
	.ascii "Hello\n"
	.byte 0
	.align
puts:
	push {r0,r1,lr}
	mov r1,r0
putsloop:
	ldrb.w r0,[r1],#1
	cbz r0,putsloopexit
	bl putc
	b putsloop
putsloopexit:
	pop {r0,r1,pc}
	.equ UART0_DATA,0x4000C000
	.equ UART0_FLAG,0x4000C018
putc:
	push {r1,r2,r3,lr}
	ldr r1,=UART0_FLAG
putcwaitloop:
	ldr r2,[r1]
	tst.w r2,#0x20
	bne putcwaitloop
	ldr r1,=UART0_DATA
	str r0,[r1]
	pop {r1,r2,r3,pc}
	.end
