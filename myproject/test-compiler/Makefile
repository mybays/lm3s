all:
	arm-none-eabi-gcc -c -o c.o main.c
	arm-none-eabi-g++ -c -o cpp.o main.cpp
	arm-none-eabi-objdump -h -S -C -r c.o>c.lss
	arm-none-eabi-objdump -h -S -C -r cpp.o>cpp.lss
