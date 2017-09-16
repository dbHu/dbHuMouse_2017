// TM4C123GH6PM

MEMORY
{
    FLASH (RX) : origin = 0x00000000, length = 0x00040000
    SRAM (RWX) : origin = 0x20000000, length = 0x00008000
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
