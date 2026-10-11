MEMORY
{
  FLASH (rx) : ORIGIN = 0x00010000, LENGTH = 512K
  RAM (rwx) : ORIGIN = 0x20000000, LENGTH = 64K
}
SECTIONS
{
  .text : { *(.text*) *(.rodata*) } > FLASH
  .data : { *(.data*) } > RAM
  .bss : { *(.bss*) *(COMMON) } > RAM
  /DISCARD/ : { *(.ARM.exidx*) *(.ARM.extab*) }
}
