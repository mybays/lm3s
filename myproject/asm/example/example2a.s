.equ STACK_TOP,0x20000800
.global vectors_table
.global start
.global nmi_handler
.code 16
.syntax unified

vectors_table:
	.word STACK_TOP,start,nmi_handler,0x0000000
	.end

