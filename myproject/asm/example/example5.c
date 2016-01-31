#define STACK_TOP 0x20000800
#define NVIC_CCR ((volatile unsigned long *)(0xE000ED14))

void myputs(char *string1);
void myputc(char mychar);
int main(void);
void nmi_handler(void);
void hardfault_handler(void);

__attribute__ ((section("vectors"))) void (*const VectorArray[]) (void) =
{
	STACK_TOP,
	main,
	nmi_handler,
	hardfault_handler
};

int main(void)
{
 	const char *helloworld="Hello world\n";
	*NVIC_CCR=*NVIC_CCR|0x200;
	myputs(*helloworld);
	while(1);
	return (0);
}

void myputs(char *string1)
{
	char mychar;
	int j;
	j=0;
	do
	{
		mychar=string1[j];
		if(mychar!=0)
		{
			myputc(mychar);
			j++;
		}
	} while(mychar !=0);
	return;
}

void myputc(char mychar)
{
	#define UART0_DATA ((volatile unsigned long *)(0x4000C000))
	#define UART0_FLAG ((volatile unsigned long *)(0x4000C018))

	while((*UART0_FLAG &0x20) !=0);
	*UART0_DATA=mychar;
	return;
}

void nmi_handler(void)
{
	return;
}

void hardfault_handler(void)
{
	return;
}
