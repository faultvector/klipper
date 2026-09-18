// I2C functions on MCXA366
//
// Copyright (C) 2026
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include "command.h"  // DECL_ENUMERATION, DECL_CONSTANT_STR
#include "gpio.h"     // i2c_setup
#include "i2ccmds.h"  // I2C_BUS_*
#include "internal.h" // LPI2C3, MRCC0, PORT3
#include "sched.h"    // shutdown


#define I2C LPI2C3

// TODO: These need to be set by users
#define I2C_SCL_PIN 27U
#define I2C_SDA_PIN 28U

#define I2C_PIN_PCR \
    (PORT_PCR_SRE(1) \
     | PORT_PCR_MUX(2) \
     | PORT_PCR_IBE(1))


DECL_ENUMERATION("i2c_bus", "i2c3", 0);
DECL_CONSTANT_STR("BUS_PINS_i2c3", "P3_27,P3_28");


static void
setup_i2c_clock(void)
{
    uint32_t clkunlock = SYSCON->CLKUNLOCK;

    SYSCON->CLKUNLOCK =
        clkunlock & ~SYSCON_CLKUNLOCK_UNLOCK_MASK;

    /*
     * Enable the LPI2C3 register-interface clock.
     */
    MRCC0->MRCC_GLB_CC1_SET =
        MRCC_MRCC_GLB_CC1_LPI2C3_MASK;

    /*
     * Feed LPI2C3 from FRO_LF_DIV.
     *
     * MUX 0 = FRO_LF_DIV = 12 MHz.
     */
    MRCC0->MRCC_LPI2C3_CLKSEL =
        MRCC_MRCC_LPI2C3_CLKSEL_MUX(0U);

    /*
     * Functional clock divider = /1.
     */
    MRCC0->MRCC_LPI2C3_CLKDIV =
        MRCC_MRCC_LPI2C3_CLKDIV_RESET_MASK
        | MRCC_MRCC_LPI2C3_CLKDIV_HALT_MASK;

    MRCC0->MRCC_LPI2C3_CLKDIV =
        MRCC_MRCC_LPI2C3_CLKDIV_HALT_MASK
        | MRCC_MRCC_LPI2C3_CLKDIV_DIV(0U);

    MRCC0->MRCC_LPI2C3_CLKDIV =
        MRCC_MRCC_LPI2C3_CLKDIV_DIV(0U);

    /*
     * Release LPI2C3 from reset.
     */
    MRCC0->MRCC_GLB_RST1_SET =
        MRCC_MRCC_GLB_RST1_LPI2C3_MASK;

    SYSCON->CLKUNLOCK = clkunlock;
}


static void
setup_i2c_pins(void)
{
    uint32_t clkunlock = SYSCON->CLKUNLOCK;

    SYSCON->CLKUNLOCK =
        clkunlock & ~SYSCON_CLKUNLOCK_UNLOCK_MASK;

    /*
     * Enable PORT3 and release it from reset.
     */
    MRCC0->MRCC_GLB_CC1_SET =
        MRCC_MRCC_GLB_CC1_PORT3_MASK;

    MRCC0->MRCC_GLB_RST1_SET =
        MRCC_MRCC_GLB_RST1_PORT3_MASK;

    SYSCON->CLKUNLOCK = clkunlock;

    /*
     * FRDM-MCXA366:
     *
     *     P3_27 = LPI2C3_SCL, ALT2
     *     P3_28 = LPI2C3_SDA, ALT2
     *
     * Match NXP's generated pin configuration:
     *
     *     fast slew
     *     internal pulls disabled
     *     input buffer enabled
     */
    PORT3->PCR[I2C_SCL_PIN] = I2C_PIN_PCR;
    PORT3->PCR[I2C_SDA_PIN] = I2C_PIN_PCR;
}


struct i2c_config
i2c_setup(uint32_t bus, uint32_t rate, uint8_t addr)
{
    if (bus != 0U)
        shutdown("Unsupported i2c bus");

    /*
     * First implementation supports standard-mode I2C only.
     */
    if (rate != 100000U)
        shutdown("Unsupported i2c rate");

    if (addr > 0x7fU)
        shutdown("Invalid i2c address");

    setup_i2c_clock();
    setup_i2c_pins();

    return (struct i2c_config) {
        .i2c = I2C,
        .addr = addr,
    };
}


int
i2c_write(struct i2c_config config, uint8_t write_len, uint8_t *write)
{
    /*
     * Transaction support comes next.
     */
    return I2C_BUS_TIMEOUT;
}


int
i2c_read(struct i2c_config config,
         uint8_t reg_len, uint8_t *reg,
         uint8_t read_len, uint8_t *read)
{
    /*
     * Transaction support comes next.
     */
    return I2C_BUS_TIMEOUT;
}