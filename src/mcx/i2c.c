// I2C functions on MCXA366
//
// Copyright (C) 2026
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include "board/misc.h" // timer_is_before
#include "command.h"    // DECL_ENUMERATION, DECL_CONSTANT_STR
#include "gpio.h"       // i2c_setup
#include "i2ccmds.h"    // I2C_BUS_*
#include "internal.h"   // LPI2C3, MRCC0, PORT3
#include "sched.h"      // shutdown


#define I2C LPI2C3

/*
 * FRDM-MCXA366 LPI2C3 pin pair.
 */
#define I2C_SCL_PIN 27U
#define I2C_SDA_PIN 28U

/*
 * Match NXP's generated pin configuration:
 *
 *     fast slew
 *     internal pulls disabled
 *     input buffer enabled
 *     ALT2 = LPI2C3
 *
 * Open-drain behavior is provided by the LPI2C peripheral when
 * PINCFG=0. PORT open-drain is therefore intentionally not enabled.
 */
#define I2C_PIN_PCR \
    (PORT_PCR_SRE(1) \
     | PORT_PCR_MUX(2) \
     | PORT_PCR_IBE(1))


/****************************************************************
 * LPI2C timing
 ****************************************************************/

/*
 * Timing values for a 12 MHz LPI2C functional clock.
 *
 * These values are derived from NXP's LPI2C_MasterSetBaudRate()
 * calculations with:
 *
 *     source clock = 12 MHz
 *     PRESCALE     = /1
 *     FILTSCL      = 0
 *
 * Supported modes:
 *
 *     100 kHz  Standard-mode
 *     400 kHz  Fast-mode
 *     1 MHz    Fast-mode Plus
 */

#define I2C_100K_CLKHI      55U
#define I2C_100K_CLKLO      61U
#define I2C_100K_SETHOLD    59U
#define I2C_100K_DATAVD     29U

#define I2C_400K_CLKHI      12U
#define I2C_400K_CLKLO      14U
#define I2C_400K_SETHOLD    14U
#define I2C_400K_DATAVD      6U

#define I2C_1M_CLKHI         3U
#define I2C_1M_CLKLO         5U
#define I2C_1M_SETHOLD       5U
#define I2C_1M_DATAVD        2U


/****************************************************************
 * LPI2C commands and status
 ****************************************************************/

#define I2C_CMD_TX_DATA 0U
#define I2C_CMD_RX_DATA 1U
#define I2C_CMD_STOP    2U
#define I2C_CMD_START   4U

#define I2C_STATUS_ERRORS \
    (LPI2C_MSR_NDF_MASK \
     | LPI2C_MSR_ALF_MASK \
     | LPI2C_MSR_FEF_MASK \
     | LPI2C_MSR_PLTF_MASK)

#define I2C_STATUS_CLEAR \
    (LPI2C_MSR_SDF_MASK | I2C_STATUS_ERRORS)


/****************************************************************
 * Klipper bus enumeration
 ****************************************************************/

DECL_ENUMERATION("i2c_bus", "i2c3", 0);
DECL_CONSTANT_STR("BUS_PINS_i2c3", "P3_27,P3_28");


/****************************************************************
 * Clock setup
 ****************************************************************/

static void
setup_i2c_clock(void)
{
    uint32_t clkunlock = SYSCON->CLKUNLOCK;

    /*
     * Permit clock configuration while preserving the previous
     * CLKUNLOCK state.
     */
    SYSCON->CLKUNLOCK =
        clkunlock & ~SYSCON_CLKUNLOCK_UNLOCK_MASK;

    /*
     * Enable the LPI2C3 register-interface clock.
     */
    MRCC0->MRCC_GLB_CC1_SET =
        MRCC_MRCC_GLB_CC1_LPI2C3_MASK;

    /*
     * LPI2C3 functional clock:
     *
     *     MUX 0 = FRO_LF_DIV
     *     FRO_LF_DIV = 12 MHz
     */
    MRCC0->MRCC_LPI2C3_CLKSEL =
        MRCC_MRCC_LPI2C3_CLKSEL_MUX(0U);

    /*
     * Reset and halt the functional clock divider before configuring
     * it for /1.
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

    /*
     * Restore the original clock-unlock state.
     */
    SYSCON->CLKUNLOCK = clkunlock;
}


/****************************************************************
 * Pin setup
 ****************************************************************/

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
     */
    PORT3->PCR[I2C_SCL_PIN] = I2C_PIN_PCR;
    PORT3->PCR[I2C_SDA_PIN] = I2C_PIN_PCR;
}


/****************************************************************
 * Baud-rate configuration
 ****************************************************************/

static uint32_t
i2c_get_timing(uint32_t rate)
{
    switch (rate) {
    case 100000U:
        return LPI2C_MCCR0_CLKHI(I2C_100K_CLKHI)
            | LPI2C_MCCR0_CLKLO(I2C_100K_CLKLO)
            | LPI2C_MCCR0_SETHOLD(I2C_100K_SETHOLD)
            | LPI2C_MCCR0_DATAVD(I2C_100K_DATAVD);

    case 400000U:
        return LPI2C_MCCR0_CLKHI(I2C_400K_CLKHI)
            | LPI2C_MCCR0_CLKLO(I2C_400K_CLKLO)
            | LPI2C_MCCR0_SETHOLD(I2C_400K_SETHOLD)
            | LPI2C_MCCR0_DATAVD(I2C_400K_DATAVD);

    case 1000000U:
        return LPI2C_MCCR0_CLKHI(I2C_1M_CLKHI)
            | LPI2C_MCCR0_CLKLO(I2C_1M_CLKLO)
            | LPI2C_MCCR0_SETHOLD(I2C_1M_SETHOLD)
            | LPI2C_MCCR0_DATAVD(I2C_1M_DATAVD);

    default:
        shutdown("Unsupported i2c rate");
    }

    /*
     * shutdown() does not return.
     */
    return 0U;
}


/****************************************************************
 * Controller setup
 ****************************************************************/

static void
setup_i2c_controller(uint32_t rate)
{
    /*
     * Reset the master and start from a known peripheral state.
     */
    I2C->MCR = LPI2C_MCR_RST_MASK;
    I2C->MCR = 0U;

    /*
     * Normal two-pin I2C operation.
     *
     * PINCFG=0:
     *     SCL/SDA operate as standard two-pin I2C signals.
     *
     * PRESCALE=0:
     *     divide the 12 MHz functional clock by 1.
     */
    I2C->MCFGR1 =
        LPI2C_MCFGR1_PINCFG(0U)
        | LPI2C_MCFGR1_PRESCALE(0U);

    /*
     * No digital glitch filtering or pin/bus timeout.
     */
    I2C->MCFGR2 = 0U;
    I2C->MCFGR3 = 0U;

    /*
     * Use the default TX and RX FIFO watermarks.
     */
    I2C->MFCR = 0U;

    /*
     * Configure bus timing before enabling the master.
     */
    I2C->MCCR0 = i2c_get_timing(rate);

    /*
     * Clear any stale status that may have survived initialization.
     */
    I2C->MSR = I2C_STATUS_CLEAR;

    /*
     * Enable master operation.
     */
    I2C->MCR = LPI2C_MCR_MEN_MASK;
}


/****************************************************************
 * Klipper I2C setup
 ****************************************************************/

struct i2c_config
i2c_setup(uint32_t bus, uint32_t rate, uint8_t addr)
{
    if (bus != 0U)
        shutdown("Unsupported i2c bus");

    /*
     * Explicitly support the bus rates validated on MCXA366:
     *
     *     100 kHz  Standard-mode
     *     400 kHz  Fast-mode
     *     1 MHz    Fast-mode Plus
     */
    if (rate != 100000U
        && rate != 400000U
        && rate != 1000000U)
        shutdown("Unsupported i2c rate");

    if (addr > 0x7fU)
        shutdown("Invalid i2c address");

    setup_i2c_clock();
    setup_i2c_pins();
    setup_i2c_controller(rate);

    return (struct i2c_config) {
        .i2c = I2C,
        .addr = addr,
    };
}


/****************************************************************
 * Error recovery
 ****************************************************************/

static int
i2c_check_error(LPI2C_Type *i2c, uint32_t status)
{
    uint32_t errors = status & I2C_STATUS_ERRORS;

    if (!errors)
        return I2C_BUS_SUCCESS;

    /*
     * Discard any commands or receive data associated with the failed
     * transaction.
     *
     * This ordering is important:
     *
     *     1. Reset the FIFOs.
     *     2. Queue STOP.
     *
     * Resetting the transmit FIFO after queuing STOP can discard the
     * STOP command and leave SCL asserted by the controller.
     */
    i2c->MCR |=
        LPI2C_MCR_RRF_MASK
        | LPI2C_MCR_RTF_MASK;

    /*
     * If this controller still owns the bus, terminate the failed
     * transaction.
     */
    if (status & LPI2C_MSR_MBF_MASK)
        i2c->MTDR = LPI2C_MTDR_CMD(I2C_CMD_STOP);

    /*
     * Clear sticky error flags.
     */
    i2c->MSR = errors;

    if (errors & LPI2C_MSR_NDF_MASK)
        return I2C_BUS_NACK;

    return I2C_BUS_TIMEOUT;
}

static void
i2c_recover_timeout(LPI2C_Type *i2c)
{
    /*
     * Remove any pending commands and unread receive data.
     */
    i2c->MCR |=
        LPI2C_MCR_RRF_MASK
        | LPI2C_MCR_RTF_MASK;

    /*
     * Request STOP if this controller still owns the bus and STOP has
     * not already completed.
     */
    uint32_t status = i2c->MSR;

    if ((status & (LPI2C_MSR_SDF_MASK | LPI2C_MSR_MBF_MASK))
        == LPI2C_MSR_MBF_MASK)
        i2c->MTDR = LPI2C_MTDR_CMD(I2C_CMD_STOP);
}


/****************************************************************
 * Transfer helpers
 ****************************************************************/

static int
i2c_wait_tx_ready(LPI2C_Type *i2c, uint32_t timeout)
{
    for (;;) {
        uint32_t status = i2c->MSR;

        int ret = i2c_check_error(i2c, status);
        if (ret != I2C_BUS_SUCCESS)
            return ret;

        /*
         * TDF indicates that the transmit FIFO can accept another
         * command. It does not indicate that the previous address or
         * data byte has been ACKed on the wire.
         */
        if (status & LPI2C_MSR_TDF_MASK)
            return I2C_BUS_SUCCESS;

        if (!timer_is_before(timer_read_time(), timeout)) {
            i2c_recover_timeout(i2c);
            return I2C_BUS_TIMEOUT;
        }
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

        if (!timer_is_before(timer_read_time(), timeout)) {
            i2c_recover_timeout(i2c);
            return I2C_BUS_TIMEOUT;
        }
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
            /*
             * SDF is write-one-to-clear.
             */
            i2c->MSR = LPI2C_MSR_SDF_MASK;
            return I2C_BUS_SUCCESS;
        }

        if (!timer_is_before(timer_read_time(), timeout)) {
            i2c_recover_timeout(i2c);
            return I2C_BUS_TIMEOUT;
        }
    }
}


/****************************************************************
 * Write transfers
 ****************************************************************/

int
i2c_write(struct i2c_config config, uint8_t write_len, uint8_t *write)
{
    LPI2C_Type *i2c = config.i2c;
    uint32_t timeout =
        timer_read_time() + timer_from_us(5000);

    /*
     * Clear sticky state left by the previous transaction.
     */
    i2c->MSR = I2C_STATUS_CLEAR;

    /*
     * Wait for room in the transmit FIFO before generating START.
     */
    int ret = i2c_wait_tx_ready(i2c, timeout);
    if (ret != I2C_BUS_SUCCESS)
        return ret;

    /*
     * START + 7-bit slave address + write direction.
     */
    i2c->MTDR =
        LPI2C_MTDR_CMD(I2C_CMD_START)
        | LPI2C_MTDR_DATA((uint32_t)config.addr << 1);

    ret = i2c_wait_tx_ready(i2c, timeout);
    if (ret != I2C_BUS_SUCCESS)
        return ret;

    /*
     * Queue payload bytes.
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
     * Finish the transaction and wait until STOP has completed.
     */
    i2c->MTDR = LPI2C_MTDR_CMD(I2C_CMD_STOP);

    return i2c_wait_stop(i2c, timeout);
}


/****************************************************************
 * Read transfers
 ****************************************************************/

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
     * Clear sticky state left by the previous transaction.
     */
    i2c->MSR = I2C_STATUS_CLEAR;

    int ret;

    if (reg_len) {
        /*
         * START + slave address in write direction.
         */
        ret = i2c_wait_tx_ready(i2c, timeout);
        if (ret != I2C_BUS_SUCCESS)
            return ret;

        i2c->MTDR =
            LPI2C_MTDR_CMD(I2C_CMD_START)
            | LPI2C_MTDR_DATA((uint32_t)config.addr << 1);

        ret = i2c_wait_tx_ready(i2c, timeout);
        if (ret != I2C_BUS_SUCCESS)
            return ret;

        /*
         * Send register/subaddress bytes without issuing STOP.
         *
         * The following START therefore becomes a repeated START.
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
     * START or repeated START + slave address in read direction.
     */
    ret = i2c_wait_tx_ready(i2c, timeout);
    if (ret != I2C_BUS_SUCCESS)
        return ret;

    i2c->MTDR =
        LPI2C_MTDR_CMD(I2C_CMD_START)
        | LPI2C_MTDR_DATA(((uint32_t)config.addr << 1) | 1U);

    ret = i2c_wait_tx_ready(i2c, timeout);
    if (ret != I2C_BUS_SUCCESS)
        return ret;

    /*
     * Request read_len bytes.
     *
     * LPI2C encodes receive count as N - 1.
     */
    i2c->MTDR =
        LPI2C_MTDR_CMD(I2C_CMD_RX_DATA)
        | LPI2C_MTDR_DATA((uint32_t)read_len - 1U);

    /*
     * Drain received bytes from the hardware RX FIFO.
     */
    while (read_len--) {
        ret = i2c_read_byte(i2c, read++, timeout);
        if (ret != I2C_BUS_SUCCESS)
            return ret;
    }

    /*
     * Wait for room in the command FIFO, then terminate the transfer.
     */
    ret = i2c_wait_tx_ready(i2c, timeout);
    if (ret != I2C_BUS_SUCCESS)
        return ret;

    i2c->MTDR = LPI2C_MTDR_CMD(I2C_CMD_STOP);

    return i2c_wait_stop(i2c, timeout);
}