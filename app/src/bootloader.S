/* Explicitly define bootloader section */
.section .bootloader_section
    .incbin "../bootloader/bootloader.bin" // Relative to linker script