	.thumb
	.syntax unified
sp:	.word 0x200
reset:	.word start+1
	.text
start:				@ Label, not really required
	mov r0,#4		@ Load register r0 with the value 5
	mov r1,#5		@ Load register r1 with the value 4
	add r2,r1,r0	@ Add r0 and r1 and store in r2
stop:	b stop		@ Infinite loop to stop execution
