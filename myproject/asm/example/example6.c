void myputs(char *string1);
void myputc(char mychar);
int main(void);
void _nmi_isr(void);
void _fault_isr(void);

int main(void)
{
	const char *helloworld="Hello world\n";
	myputs(*helloworld);
	while(1);
	return(0);
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
	} while(mychar!=0);
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

void _nmi_isr(void)
{
	return;
}

void _fault_isr(void)
{
	return;
}
