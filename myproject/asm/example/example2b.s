.text
.global _start
.global start
.global nmi_handler
.code 16
.syntax unified
.type start,function
.type nmi_handler,function
_start:
start:
	movs r0,#10
	movs r1,#0
loop:
	adds r1,r0
	subs r0,#1
	bne loop
deadloop:
	b deadloop
nmi_handler:
	bx lr
	.end

