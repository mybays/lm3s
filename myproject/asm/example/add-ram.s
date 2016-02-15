        .syntax unified
        .thumb
		.data
num1:   .word 0x10
num2:   .word 0x20
result: .word 0x0

		.text
sp:     .word 0x200
reset:  .word start+1
start:

		ldr r0, =sdata		@ Load the address of sdata
		ldr r1, =edata		@ Load the address of edata
		ldr r2, =etext		@ Load the address of etext
copy:		
		ldrb r3, [r2]		@ Load the value from Flash
		strb r3, [r0]		@ Store the value in RAM
		add  r2, r2, #1		@ Increment Flash pointer
		add  r0, r0, #1		@ Increment RAM pointer
		cmp  r0, r1			@ Check if end of data
		bne  copy			@ Branch if not end of data

		ldr r0, =num1		@ Load the address of num1
		ldr r1, [r0]		@ Load the value in num1

		ldr r0, =num2		@ Load the address of num2
		ldr r2, [r0]		@ Load the value in num2

		add r3, r1, r2		@ Add num1 and num2

		ldr r0, =result		@ Load the address of result
		str r3, [r0]		@ Store the value in result

stop:	b stop
