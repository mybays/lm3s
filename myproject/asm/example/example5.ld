MEMORY
{
	rom(rx) : ORIGIN = 0x00000000, LENGTH = 0x00040000
	ram(rwx) : ORIGIN = 0x20000000,LENGTH = 0x00010000
}

SECTIONS
{
	. = 0x0;
	.text : {
				*(vectors)
				*(.text)
				*(.rodata)
			}
	. = 0x20000000;
	.data : {
				*(.data)
			}
	.bss :	{
				*(.bss)
			}
}
