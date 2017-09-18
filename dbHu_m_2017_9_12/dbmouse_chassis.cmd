// TM4C1294NCPDT

MEMORY
{
    FLASH (RX) : origin = 0x00000000, length = 0x00100000
    SRAM (RWX) : origin = 0x20000000, length = 0x00040000
}

/* Section allocation in memory */

SECTIONS
{
    .text   :   > FLASH
    .const  :   > FLASH
    .cinit  :   > FLASH
    .pinit  :   > FLASH
    .init_array : > FLASH

    .data   :   > SRAM
    .bss    :   > SRAM
    .sysmem :   > SRAM
    .stack  :   > SRAM
}
