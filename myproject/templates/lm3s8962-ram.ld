/*
Link script for LM3S8962 in ram.
Modified in Jul.17.2011
256K Flash,based on 0x00000000 to 0x00040000
64K Ram,based on 0x20000000 to 0x00010000
其中32K ram当flash使用,32K ram当ram使用
*/
MEMORY
{
    FLASH (rx) : ORIGIN = 0x20000000, LENGTH = 0x00008000
    SRAM (rwx) : ORIGIN = 0x20008000, LENGTH = 0x00008000
}


SECTIONS
{
    __STACKSIZE__ = 2048;
    __HEAPSIZE__  =  128;
    __SRAM_TOP__  = 0x20000000 + 0x00010000;

    .text 0x20000000 : AT(0x20000000)
    {
        _text = .;
        KEEP(*(.vectors))
        KEEP(*(.init))
        *(.text*)
        *(.rodata*)
    } > FLASH

    .ARM.extab :
    {
        *(.ARM.extab* .gnu.linkonce.armextab.*)
    } > FLASH

    .ARM.exidx :
    {
        *(.ARM.exidx* .gnu.linkonce.armexidx.*)
    } > FLASH
    _etext = _text + SIZEOF(.text) + SIZEOF(.ARM.extab) + SIZEOF(.ARM.exidx);


    .data : AT(ALIGN(_etext , 4))
    {
        _data = .;
        *(.vectors_ram)
        *(.data*)
    } > SRAM
    _edata = _data + SIZEOF(.data);

    .bss : AT(ALIGN(_edata , 4))
    {
        _bss = .;
        *(.bss*)
        *(COMMON)
    } > SRAM
    _ebss = _bss + SIZEOF(.bss);


    .heap : AT(ALIGN(_ebss , 4))
    {
      _heap = .;
      *(.heap*)
      . = ALIGN(MAX(_heap + __HEAPSIZE__ , .), 4);
    }
    _eheap = _heap + SIZEOF(.heap);

    .stack ALIGN(__SRAM_TOP__ - __STACKSIZE__, 4) : AT(ALIGN(__SRAM_TOP__ - __STACKSIZE__, 4))
    {
      _stack = .;
      *(.stack*)
      . = ALIGN(MAX(_stack + __STACKSIZE__ , .), 4);
    }
    _estack = _stack + SIZEOF(.stack);

    PROVIDE(__HEAP_START  = _heap);
}
