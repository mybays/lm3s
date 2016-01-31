.equ STACK_TOP,0x20000800
.global vectors_table
.global _start
.code 16
.syntax unified
vectors_table:
	.word STACK_TOP,_start
	.end
