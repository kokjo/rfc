MEMORY {
    FLASH               : ORIGIN = 0x08000000, LENGTH = 32K
    BOOTLOADER_STATE    : ORIGIN = 0x08008000, LENGTH = 4K
    DFU                 : ORIGIN = 0x08009000, LENGTH = 476K
    RAM           (rwx) : ORIGIN = 0x20000000, LENGTH = 128K /* SRAM1 + SRAM2 + CCMRAM_DCODE */
}

__bootloader_state_start = ORIGIN(BOOTLOADER_STATE) - ORIGIN(FLASH);
__bootloader_state_end = ORIGIN(BOOTLOADER_STATE) + LENGTH(BOOTLOADER_STATE) - ORIGIN(FLASH);

__bootloader_dfu_start = ORIGIN(DFU) - ORIGIN(FLASH);
__bootloader_dfu_end = ORIGIN(DFU) + LENGTH(DFU) - ORIGIN(FLASH);