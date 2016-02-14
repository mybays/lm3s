target remote localhost:3333
#monitor soft_reset_halt

define ram
monitor reset
monitor soft_reset_halt
load main.axf
monitor reg pc 0x200000b9
monitor resume
end

define rr
monitor soft_reset_halt
symbol-file main.axf
thbreak main
continue
end

define regs
info reg
end

define flash
monitor mww 0x400fe0f0 1 1
monitor halt
monitor mww 0x400fe0f0 1 1
monitor stellaris mass_erase 0
#monitor flash erase_address 0 262144
monitor soft_reset_halt
monitor mww 0x400fe0f0 1 1
monitor flash write_image main.axf
monitor mww 0x400fe0f0 1 1
monitor verify_image main.axf
monitor reset
x/4 0
end

define f1
monitor flash erase_address 0 262144
end

define f2
monitor flash write_image main.axf
end

define mcm
make clean; make
end

define dis
disassemble $pc-16 $pc+16
end

set print pretty
set confirm off
