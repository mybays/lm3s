	.thumb
	.syntax unified
sp:	.word 0x200
reset:	.word start+1

start:
	mov r0,#4097
	mov r1,#5
	add r2,r1,r0
stop:	b stop
