# Additional NXP MCX build rules

# Setup the toolchain
CROSS_PREFIX=arm-none-eabi-

dirs-y += src/mcx src/generic

CFLAGS += -mcpu=cortex-m33 -mthumb -Ilib/cmsis-core
CFLAGS += -Ilib/cmsis-core
CFLAGS += -Ilib/mcxa366
CFLAGS += -Ilib/mcxa366/periph2
CFLAGS += -DCPU_MCXA366VLQ

CFLAGS_klipper.elf += -nostdlib -lgcc -lc_nano
CFLAGS_klipper.elf += -T $(OUT)src/generic/armcm_link.ld
$(OUT)klipper.elf: $(OUT)src/generic/armcm_link.ld

# Add more source files
src-y += mcx/main.c
src-y += generic/armcm_boot.c generic/armcm_irq.c generic/armcm_reset.c
src-y += generic/armcm_timer.c
src-y += generic/crc16_ccitt.c

src-$(CONFIG_SERIAL) += mcx/serial.c generic/serial_irq.c