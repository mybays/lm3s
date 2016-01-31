example1:main.s
	
	arm-stellaris-eabi-as -g -mcpu=cortex-m3 -mthumb $< -o main.o
	arm-stellaris-eabi-ld -Ttext 0x20000000 -o main.axf main.o
	arm-stellaris-eabi-objdump -S main.axf>main.list
	arm-stellaris-eabi-objcopy -Obinary main.axf main.bin
	arm-stellaris-eabi-size -Ax main.axf
clean:
	rm -fr *.o *.axf *.list *.bin

