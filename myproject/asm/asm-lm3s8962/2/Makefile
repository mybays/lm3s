example1:main.s
	
	arm-none-eabi-as -g -mcpu=cortex-m3 -mthumb $< -o main.o
	arm-none-eabi-ld -Ttext 0x20000000 -o main.axf main.o
	arm-none-eabi-objdump -S main.axf>main.list
	arm-none-eabi-objcopy -Obinary main.axf main.bin
clean:
	rm -fr *.o *.axf *.list *.bin

