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
#include "board/misc.h" // timer_is_before

#define I2C LPI2C3

// FRDM-MCXA366 LPI2C3 pin pair
#define I2C_SCL_PIN 27U
#define I2C_SDA_PIN 28U

#define I2C_PIN_PCR \
    (PORT_PCR_SRE(1) \
     | PORT_PCR_MUX(2) \
     | PORT_PCR_IBE(1))

#define I2C_100K_CLKHI      55U
#define I2C_100K_CLKLO      61U
#define I2C_100K_SETHOLD    59U
#define I2C_100K_DATAVD     29U

#define I2C_CMD_TX_DATA 0U
#define I2C_CMD_STOP    2U
#define I2C_CMD_START   4U

#define I2C_CMD_RX_DATA 1U


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
setup_i2c_controller(void)
{
    /*
     * Start from a known peripheral state.
     */
    I2C->MCR = LPI2C_MCR_RST_MASK;
    I2C->MCR = 0U;

    /*
     * Default master configuration:
     *
     *     2-pin open-drain mode
     *     ACK checking enabled
     *     prescaler = /1
     *
     * PINCFG=0 selects 2-pin open-drain operation.
     */
    I2C->MCFGR1 =
        LPI2C_MCFGR1_PINCFG(0U)
        | LPI2C_MCFGR1_PRESCALE(0U);

    /*
     * No glitch filtering or bus/pin timeout yet.
     */
    I2C->MCFGR2 = 0U;
    I2C->MCFGR3 = 0U;

    /*
     * Default FIFO watermarks.
     */
    I2C->MFCR = 0U;

    /*
     * Standard-mode timing for a 12 MHz LPI2C functional clock.
     *
     * These values follow NXP's LPI2C_MasterSetBaudRate()
     * calculation for:
     *
     *     source clock = 12 MHz
     *     bus rate     = 100 kHz
     *     PRESCALE     = /1
     *     FILTSCL      = 0
     */
    I2C->MCCR0 =
        LPI2C_MCCR0_CLKHI(I2C_100K_CLKHI)
        | LPI2C_MCCR0_CLKLO(I2C_100K_CLKLO)
        | LPI2C_MCCR0_SETHOLD(I2C_100K_SETHOLD)
        | LPI2C_MCCR0_DATAVD(I2C_100K_DATAVD);

    /*
     * Timing is configured. Enable master operation.
     */
    I2C->MCR = LPI2C_MCR_MEN_MASK;
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
    setup_i2c_controller();

    return (struct i2c_config) {
        .i2c = I2C,
        .addr = addr,
    };
}

static int
i2c_check_error(LPI2C_Type *i2c, uint32_t status)
{
    uint32_t errors =
        status & (LPI2C_MSR_NDF_MASK
                  | LPI2C_MSR_ALF_MASK
                  | LPI2C_MSR_FEF_MASK
                  | LPI2C_MSR_PLTF_MASK);

    if (!errors)
        return I2C_BUS_SUCCESS;

    /*
     * LPI2C status error flags are write-one-to-clear.
     */
    i2c->MSR = errors;

    /*
     * An error may leave stale commands or data in the FIFOs.
     * Reset both before allowing another transaction.
     *
     * RRF and RTF clear automatically.
     */
    i2c->MCR |=
        LPI2C_MCR_RRF_MASK
        | LPI2C_MCR_RTF_MASK;

    if (errors & LPI2C_MSR_NDF_MASK)
        return I2C_BUS_NACK;

    return I2C_BUS_TIMEOUT;
}

static int
i2c_wait_tx_ready(LPI2C_Type *i2c, uint32_t timeout)
{
    for (;;) {
        uint32_t status = i2c->MSR;

        int ret = i2c_check_error(i2c, status);
        if (ret != I2C_BUS_SUCCESS)
            return ret;

        if (status & LPI2C_MSR_TDF_MASK)
            return I2C_BUS_SUCCESS;

        if (!timer_is_before(timer_read_time(), timeout))
            return I2C_BUS_TIMEOUT;
    }
}

static int
i2c_read_byte(LPI2C_Type *i2c, uint8_t *data, uint32_t timeout)
{
    for (;;) {
        uint32_t status = i2c->MSR;

        int ret = i2c_check_error(i2c, status);
        if (ret != I2C_BUS_SUCCESS)
            return ret;

        uint32_t value = i2c->MRDR;

        if (!(value & LPI2C_MRDR_RXEMPTY_MASK)) {
            *data = value & LPI2C_MRDR_DATA_MASK;
            return I2C_BUS_SUCCESS;
        }

        if (!timer_is_before(timer_read_time(), timeout))
            return I2C_BUS_TIMEOUT;
    }
}

static int
i2c_wait_stop(LPI2C_Type *i2c, uint32_t timeout)
{
    for (;;) {
        uint32_t status = i2c->MSR;

        int ret = i2c_check_error(i2c, status);
        if (ret != I2C_BUS_SUCCESS)
            return ret;

        if (status & LPI2C_MSR_SDF_MASK) {
            i2c->MSR = LPI2C_MSR_SDF_MASK;
            return I2C_BUS_SUCCESS;
        }

        if (!timer_is_before(timer_read_time(), timeout))
            return I2C_BUS_TIMEOUT;
    }
}

int
i2c_write(struct i2c_config config, uint8_t write_len, uint8_t *write)
{
    LPI2C_Type *i2c = config.i2c;
    uint32_t timeout =
        timer_read_time() + timer_from_us(5000);

    /*
     * Clear status left over from a previous transaction.
     */
    i2c->MSR =
        LPI2C_MSR_SDF_MASK
        | LPI2C_MSR_NDF_MASK
        | LPI2C_MSR_ALF_MASK
        | LPI2C_MSR_FEF_MASK
        | LPI2C_MSR_PLTF_MASK;

    /*
     * Generate START and transmit the 7-bit address with R/W = 0.
     */
    int ret = i2c_wait_tx_ready(i2c, timeout);
    if (ret != I2C_BUS_SUCCESS)
        return ret;

    i2c->MTDR =
        LPI2C_MTDR_CMD(I2C_CMD_START)
        | LPI2C_MTDR_DATA((uint32_t)config.addr << 1);

    /*
     * Wait until the transmit FIFO can accept another command.
     *
     * TDF indicates FIFO availability; it does not guarantee that
     * the address phase has completed on the bus.
     */
    ret = i2c_wait_tx_ready(i2c, timeout);
    if (ret != I2C_BUS_SUCCESS)
        return ret;

    /*
     * Queue the payload bytes.
     */
    while (write_len--) {
        i2c->MTDR =
            LPI2C_MTDR_CMD(I2C_CMD_TX_DATA)
            | LPI2C_MTDR_DATA(*write++);

        ret = i2c_wait_tx_ready(i2c, timeout);
        if (ret != I2C_BUS_SUCCESS)
            return ret;
    }

    /*
     * Generate STOP and wait until it has actually appeared on the bus.
     */
    i2c->MTDR = LPI2C_MTDR_CMD(I2C_CMD_STOP);

    return i2c_wait_stop(i2c, timeout);
}

int
i2c_read(struct i2c_config config,
         uint8_t reg_len, uint8_t *reg,
         uint8_t read_len, uint8_t *read)
{
    LPI2C_Type *i2c = config.i2c;
    uint32_t timeout =
        timer_read_time() + timer_from_us(5000);

    if (!read_len)
        return I2C_BUS_SUCCESS;

    /*
     * Clear status left over from a previous transaction.
     */
    i2c->MSR =
        LPI2C_MSR_SDF_MASK
        | LPI2C_MSR_NDF_MASK
        | LPI2C_MSR_ALF_MASK
        | LPI2C_MSR_FEF_MASK
        | LPI2C_MSR_PLTF_MASK;

    int ret;

    if (reg_len) {
        /*
         * START + slave address with R/W = 0.
         */
        ret = i2c_wait_tx_ready(i2c, timeout);
        if (ret != I2C_BUS_SUCCESS)
            return ret;

        i2c->MTDR =
            LPI2C_MTDR_CMD(I2C_CMD_START)
            | LPI2C_MTDR_DATA((uint32_t)config.addr << 1);

        /*
         * Wait until the transmit FIFO can accept another command.
         *
         * TDF indicates FIFO availability; it does not guarantee that
         * the address phase has completed on the bus.
         */
        ret = i2c_wait_tx_ready(i2c, timeout);
        if (ret != I2C_BUS_SUCCESS)
            return ret;

        /*
         * Send the register/subaddress bytes without issuing STOP.
         */
        while (reg_len--) {
            i2c->MTDR =
                LPI2C_MTDR_CMD(I2C_CMD_TX_DATA)
                | LPI2C_MTDR_DATA(*reg++);

            ret = i2c_wait_tx_ready(i2c, timeout);
            if (ret != I2C_BUS_SUCCESS)
                return ret;
        }
    }

    /*
     * Generate START or repeated START with R/W = 1.
     */
    ret = i2c_wait_tx_ready(i2c, timeout);
    if (ret != I2C_BUS_SUCCESS)
        return ret;

    i2c->MTDR =
        LPI2C_MTDR_CMD(I2C_CMD_START)
        | LPI2C_MTDR_DATA(((uint32_t)config.addr << 1) | 1U);

    /*
     * Wait until the transmit FIFO can accept the receive command.
     *
     * TDF is not an address-ACK indication, so any NACK observed here
     * is reported generically.
     */
    ret = i2c_wait_tx_ready(i2c, timeout);
    if (ret != I2C_BUS_SUCCESS)
        return ret;

    /*
     * Request read_len bytes. LPI2C encodes this as N - 1.
     */
    i2c->MTDR =
        LPI2C_MTDR_CMD(I2C_CMD_RX_DATA)
        | LPI2C_MTDR_DATA((uint32_t)read_len - 1U);

    while (read_len--) {
        ret = i2c_read_byte(i2c, read, timeout);
        if (ret != I2C_BUS_SUCCESS)
            return ret;
        read++;
    }

    /*
     * End the transaction.
     */
    ret = i2c_wait_tx_ready(i2c, timeout);
    if (ret != I2C_BUS_SUCCESS)
        return ret;

    i2c->MTDR = LPI2C_MTDR_CMD(I2C_CMD_STOP);

    return i2c_wait_stop(i2c, timeout);
}