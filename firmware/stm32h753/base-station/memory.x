MEMORY
{
    FLASH      : ORIGIN = 0x08000000, LENGTH = 2048K

    AXI_HEAP   : ORIGIN = 0x24000000, LENGTH = 128K
    RAM        : ORIGIN = 0x24020000, LENGTH = 384K

    RAM_D3     : ORIGIN = 0x38000000, LENGTH = 64K
}

SECTIONS
{
    .ram_d3 :
    {
        *(.ram_d3)
    } > RAM_D3

    .axi_heap (NOLOAD) :
    {
        *(.axi_heap)
    } > AXI_HEAP
}
