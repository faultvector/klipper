# Additional NXP MCX build rules

# Setup the toolchain
CROSS_PREFIX=arm-none-eabi-

dirs-y += src/mcx src/generic

CFLAGS += -mcpu=cortex-m33 -mthumb -Ilib/cmsis-core

CFLAGS_klipper.elf += -nostdlib -lgcc -lc_nano
CFLAGS_klipper.elf += -T $(OUT)src/generic/armcm_link.ld
$(OUT)klipper.elf: $(OUT)src/generic/armcm_link.ld

# Add more source files
src-y += mcx/main.c
src-y += generic/armcm_boot.c generic/armcm_irq.c generic/armcm_reset.c