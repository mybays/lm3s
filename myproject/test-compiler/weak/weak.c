#include<stdio.h>

extern void foo(void) __attribute__((weak));

void foo(void)
{
	printf("weak.\n");
}

int main(void)
{
	foo();
}


