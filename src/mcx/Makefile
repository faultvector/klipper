# Additional NXP MCX build rules

# Setup the toolchain
CROSS_PREFIX=arm-none-eabi-

dirs-y += src/mcx src/generic
dirs-y += lib/mcxa366
dirs-y += lib/fast-hash

CFLAGS += -mcpu=cortex-m33 -mthumb
CFLAGS += -Ilib/cmsis-core
CFLAGS += -Ilib/mcxa366
CFLAGS += -Ilib/mcxa366/periph2
CFLAGS += -DCPU_MCXA366VLQ
CFLAGS += -Ilib/fast-hash

CFLAGS_klipper.elf += -nostdlib -lgcc -lc_nano
CFLAGS_klipper.elf += -T $(OUT)src/generic/armcm_link.ld
$(OUT)klipper.elf: $(OUT)src/generic/armcm_link.ld

# Add Klipper source files
src-y += mcx/main.c
src-y += generic/armcm_boot.c generic/armcm_irq.c generic/armcm_reset.c
src-y += generic/armcm_timer.c
src-y += generic/crc16_ccitt.c
src-y += mcx/clock.c
src-y += mcx/gpio.c

src-$(CONFIG_SERIAL) += mcx/serial.c generic/serial_irq.c
src-$(CONFIG_WANT_ADC) += mcx/adc.c
src-$(CONFIG_WANT_SPI) += mcx/spi.c
src-$(CONFIG_CANSERIAL) += mcx/can.c
src-$(CONFIG_CANSERIAL) += generic/canbus.c generic/canserial.c
src-$(CONFIG_CANSERIAL) += ../lib/fast-hash/fasthash.c

# Add NXP device support
OBJS_klipper.elf += $(OUT)lib/mcxa366/system_MCXA366.o