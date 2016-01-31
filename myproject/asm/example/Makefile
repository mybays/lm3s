example1:example1.s
	
	arm-stellaris-eabi-as -mcpu=cortex-m3 -mthumb $< -o example1.o
	arm-stellaris-eabi-ld -Ttext 0x20000000 -o example1.axf example1.o
	arm-stellaris-eabi-objdump -S example1.axf>example1.list
	arm-stellaris-eabi-objcopy -Obinary example1.axf example1.bin

example2:example2a.s example2b.s
	arm-stellaris-eabi-as -mcpu=cortex-m3 -mthumb example2a.s -o example2a.o
	arm-stellaris-eabi-as -mcpu=cortex-m3 -mthumb example2b.s -o example2b.o
	arm-stellaris-eabi-ld -Ttext 0x20000000 -o example2.axf example2a.o example2b.o
	arm-stellaris-eabi-objcopy -Obinary example2.axf example2.bin
	arm-stellaris-eabi-objdump -S example2.axf>example2.list

example3:example3a.s example3b.s
	arm-stellaris-eabi-as -mcpu=cortex-m3 -mthumb example3a.s -o example3a.o
	arm-stellaris-eabi-as -mcpu=cortex-m3 -mthumb example3b.s -o example3b.o
	arm-stellaris-eabi-ld -Ttext 0x20000000 -o example3.axf example3a.o example3b.o
	arm-stellaris-eabi-objcopy -Obinary example3.axf example3.bin
	arm-stellaris-eabi-objdump -S example3.axf>example3.list

example4:example4.s
	arm-stellaris-eabi-as -mcpu=cortex-m3 -mthumb example4.s -o example4.o
	arm-stellaris-eabi-ld -Ttext 0x20000000 -Tdata 0x20000500 -o example4.axf example4.o
	arm-stellaris-eabi-objcopy -Obinary -R .data example4.axf example4.bin
	arm-stellaris-eabi-objdump -S example4.axf>example4.list

example5:example5.c example5.ld
	arm-stellaris-eabi-gcc -mcpu=cortex-m3 -mthumb example5.c -nostartfiles -T example5.ld -o example5.o
	arm-stellaris-eabi-ld -T example5.ld -o example5.axf example5.o
	arm-stellaris-eabi-objcopy -Obinary example5.axf example5.bin
	arm-stellaris-eabi-objdump -S example5.axf >example5.list

example6:example6.c
	arm-stellaris-eabi-gcc -mcpu=cortex-m3 -mthumb example6.c -L lib -T lm3s8962-rom.ld -o example6.axf
	arm-stellaris-eabi-objcopy -Obinary example6.axf example6.bin
	arm-stellaris-eabi-objdump -S example6.axf >example6.list

add:add.s
	arm-stellaris-eabi-as -mcpu=cortex-m3 -mthumb $< -o add.o
	arm-stellaris-eabi-ld -Ttext=0x0 -o add.elf add.o
	arm-stellaris-eabi-objcopy -O binary add.elf add.bin
	arm-stellaris-eabi-objdump -h -S -C -r add.elf >add.list

clean:
	rm -fr *.o *.axf *.list *.bin

