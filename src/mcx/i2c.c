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

#define I2C_400K_CLKHI      12U
#define I2C_400K_CLKLO      14U
#define I2C_400K_SETHOLD    14U
#define I2C_400K_DATAVD      6U

#define I2C_1M_CLKHI         3U
#define I2C_1M_CLKLO         5U
#define I2C_1M_SETHOLD       5U
#define I2C_1M_DATAVD        2U

#define I2C_CMD_TX_DATA 0U
#define I2C_CMD_STOP    2U
#define I2C_CMD_START   4U

#define I2C_CMD_RX_DATA 1U


DECL_ENUMERATION("i2c_bus", "i2c3", 0);
DECL_CONSTANT_STR("BUS_PINS_i2c3", "P3_27,P3_28");

volatile uint32_t mcx_i2c_dbg_mcr;
volatile uint32_t mcx_i2c_dbg_msr;
volatile uint32_t mcx_i2c_dbg_mcfgr1;
volatile uint32_t mcx_i2c_dbg_mccr0;
volatile uint32_t mcx_i2c_dbg_mfsr;
volatile uint32_t mcx_i2c_dbg_pcr27;
volatile uint32_t mcx_i2c_dbg_pcr28;

volatile uint32_t mcx_i2c_dbg_clksel;
volatile uint32_t mcx_i2c_dbg_clkdiv;
volatile uint32_t mcx_i2c_dbg_verid;
volatile uint32_t mcx_i2c_dbg_param;

volatile uint32_t mcx_i2c_dbg_sirccsr;
volatile uint32_t mcx_i2c_dbg_frolfdiv;

volatile uint32_t mcx_i2c_dbg_after_start_msr;
volatile uint32_t mcx_i2c_dbg_after_start_mfsr;
volatile uint32_t mcx_i2c_dbg_after_start_mcr;

volatile uint32_t mcx_i2c_dbg_error_status;
volatile uint32_t mcx_i2c_dbg_errors;

volatile uint32_t mcx_i2c_dbg_read_start1_msr;
volatile uint32_t mcx_i2c_dbg_read_start1_mfsr;
volatile uint32_t mcx_i2c_dbg_read_start1_mcr;

volatile uint32_t mcx_i2c_dbg_read_start2_msr;
volatile uint32_t mcx_i2c_dbg_read_start2_mfsr;
volatile uint32_t mcx_i2c_dbg_read_start2_mcr;

volatile uint32_t mcx_i2c_dbg_mcfgr2;
volatile uint32_t mcx_i2c_dbg_mcfgr3;
volatile uint32_t mcx_i2c_dbg_gpio3_pin;

volatile uint32_t mcx_i2c_dbg_after_tdf_msr;
volatile uint32_t mcx_i2c_dbg_after_tdf_mfsr;
volatile uint32_t mcx_i2c_dbg_after_tdf_mcr;


volatile uint32_t mcx_i2c_dbg_tx_loops_max;
volatile uint32_t mcx_i2c_dbg_rx_loops_max;
volatile uint32_t mcx_i2c_dbg_stop_loops_max;

volatile uint32_t mcx_i2c_dbg_tx_calls;
volatile uint32_t mcx_i2c_dbg_rx_calls;
volatile uint32_t mcx_i2c_dbg_stop_calls;

volatile uint64_t mcx_i2c_dbg_tx_loops_total;
volatile uint64_t mcx_i2c_dbg_rx_loops_total;
volatile uint64_t mcx_i2c_dbg_stop_loops_total;

volatile uint32_t mcx_i2c_dbg_read_transactions;

volatile uint32_t mcx_i2c_dbg_read_calls;
volatile uint32_t mcx_i2c_dbg_write_calls;

volatile uint64_t mcx_i2c_dbg_read_ticks_total;
volatile uint64_t mcx_i2c_dbg_write_ticks_total;

volatile uint32_t mcx_i2c_dbg_read_ticks_max;
volatile uint32_t mcx_i2c_dbg_write_ticks_max;

/*
 * Poll-loop diagnostics.
 *
 * These track the largest number of iterations spent in each wait
 * routine since reset. They are intentionally diagnostic only and
 * should be removed once the load regression is understood.
 */
volatile uint32_t mcx_i2c_dbg_tx_loops_max;
volatile uint32_t mcx_i2c_dbg_rx_loops_max;
volatile uint32_t mcx_i2c_dbg_stop_loops_max;

volatile uint32_t mcx_i2c_dbg_read_lt_100us;
volatile uint32_t mcx_i2c_dbg_read_100_500us;
volatile uint32_t mcx_i2c_dbg_read_500_1000us;
volatile uint32_t mcx_i2c_dbg_read_1_2ms;
volatile uint32_t mcx_i2c_dbg_read_gt_2ms;

static void
i2c_dbg_record_read_duration(uint32_t elapsed)
{
    if (elapsed < timer_from_us(100))
        mcx_i2c_dbg_read_lt_100us++;
    else if (elapsed < timer_from_us(500))
        mcx_i2c_dbg_read_100_500us++;
    else if (elapsed < timer_from_us(1000))
        mcx_i2c_dbg_read_500_1000us++;
    else if (elapsed < timer_from_us(2000))
        mcx_i2c_dbg_read_1_2ms++;
    else
        mcx_i2c_dbg_read_gt_2ms++;
}

static void
i2c_dbg_update_max(volatile uint32_t *maximum, uint32_t loops)
{
    if (loops > *maximum)
        *maximum = loops;
}

static void
i2c_capture_debug_state(void)
{
    mcx_i2c_dbg_mcr = I2C->MCR;
    mcx_i2c_dbg_msr = I2C->MSR;
    mcx_i2c_dbg_mcfgr1 = I2C->MCFGR1;
    mcx_i2c_dbg_mccr0 = I2C->MCCR0;
    mcx_i2c_dbg_mfsr = I2C->MFSR;
    mcx_i2c_dbg_pcr27 = PORT3->PCR[27];
    mcx_i2c_dbg_pcr28 = PORT3->PCR[28];

    mcx_i2c_dbg_clksel = MRCC0->MRCC_LPI2C3_CLKSEL;
    mcx_i2c_dbg_clkdiv = MRCC0->MRCC_LPI2C3_CLKDIV;
    mcx_i2c_dbg_verid = I2C->VERID;
    mcx_i2c_dbg_param = I2C->PARAM;

    mcx_i2c_dbg_sirccsr = SCG0->SIRCCSR;
    mcx_i2c_dbg_frolfdiv = SYSCON->FROLFDIV;

    mcx_i2c_dbg_mcfgr2 = I2C->MCFGR2;
    mcx_i2c_dbg_mcfgr3 = I2C->MCFGR3;
    mcx_i2c_dbg_gpio3_pin = GPIO3->PDIR;
}


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
setup_i2c_controller(uint32_t rate)
{
    /*
     * Start from a known peripheral state.
     */
    I2C->MCR = LPI2C_MCR_RST_MASK;
    I2C->MCR = 0U;

    /*
     * Normal two-pin I2C master mode.
     *
     * PRESCALE=0 gives a /1 divider from the 12 MHz functional clock.
     */
    I2C->MCFGR1 =
        LPI2C_MCFGR1_PINCFG(0U)
        | LPI2C_MCFGR1_PRESCALE(0U);

    /*
     * No digital glitch filtering or bus/pin timeout yet.
     */
    I2C->MCFGR2 = 0U;
    I2C->MCFGR3 = 0U;

    /*
     * Default FIFO watermarks.
     */
    I2C->MFCR = 0U;

    /*
     * Timing values calculated using NXP's
     * LPI2C_MasterSetBaudRate() algorithm for a 12 MHz
     * LPI2C functional clock and FILTSCL=0.
     */
    switch (rate) {
    case 100000U:
        I2C->MCCR0 =
            LPI2C_MCCR0_CLKHI(I2C_100K_CLKHI)
            | LPI2C_MCCR0_CLKLO(I2C_100K_CLKLO)
            | LPI2C_MCCR0_SETHOLD(I2C_100K_SETHOLD)
            | LPI2C_MCCR0_DATAVD(I2C_100K_DATAVD);
        break;

    case 400000U:
        I2C->MCCR0 =
            LPI2C_MCCR0_CLKHI(I2C_400K_CLKHI)
            | LPI2C_MCCR0_CLKLO(I2C_400K_CLKLO)
            | LPI2C_MCCR0_SETHOLD(I2C_400K_SETHOLD)
            | LPI2C_MCCR0_DATAVD(I2C_400K_DATAVD);
        break;

    case 1000000U:
        I2C->MCCR0 =
            LPI2C_MCCR0_CLKHI(I2C_1M_CLKHI)
            | LPI2C_MCCR0_CLKLO(I2C_1M_CLKLO)
            | LPI2C_MCCR0_SETHOLD(I2C_1M_SETHOLD)
            | LPI2C_MCCR0_DATAVD(I2C_1M_DATAVD);
        break;

    default:
        shutdown("Unsupported i2c rate");
    }

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
     * Supported modes:
     *
     *     100 kHz  - Standard-mode
     *     400 kHz  - Fast-mode
     *     1 MHz    - Fast-mode Plus
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
     * Capture the state that actually triggered the error.
     */
    mcx_i2c_dbg_error_status = status;
    mcx_i2c_dbg_errors = errors;

    /*
     * Discard any commands/data still queued from the failed transfer.
     *
     * This MUST happen before queuing STOP, otherwise RTF can discard
     * the STOP command itself.
     */
    i2c->MCR |=
        LPI2C_MCR_RRF_MASK
        | LPI2C_MCR_RTF_MASK;

    /*
     * If this controller still owns the bus, terminate the transaction.
     */
    if (status & LPI2C_MSR_MBF_MASK)
        i2c->MTDR = LPI2C_MTDR_CMD(I2C_CMD_STOP);

    /*
     * Clear sticky error flags.
     */
    i2c->MSR = errors;

    /*
     * Capture state after initiating recovery.
     */
    i2c_capture_debug_state();

    if (errors & LPI2C_MSR_NDF_MASK)
        return I2C_BUS_NACK;

    return I2C_BUS_TIMEOUT;
}

static void
i2c_recover_timeout(LPI2C_Type *i2c)
{
    /*
     * Discard any stale transmit commands or unread receive data.
     */
    i2c->MCR |=
        LPI2C_MCR_RRF_MASK
        | LPI2C_MCR_RTF_MASK;

    /*
     * If this controller still owns the bus and no STOP has been
     * generated, request one before returning to the caller.
     */
    uint32_t status = i2c->MSR;

    if ((status & (LPI2C_MSR_SDF_MASK | LPI2C_MSR_MBF_MASK))
        == LPI2C_MSR_MBF_MASK)
        i2c->MTDR = LPI2C_MTDR_CMD(I2C_CMD_STOP);
}

static int
i2c_wait_tx_ready(LPI2C_Type *i2c, uint32_t timeout)
{
    uint32_t loops = 0;

    for (;;) {
        loops++;

        uint32_t status = i2c->MSR;

        int ret = i2c_check_error(i2c, status);
        if (ret != I2C_BUS_SUCCESS) {
            i2c_dbg_update_max(&mcx_i2c_dbg_tx_loops_max, loops);
            mcx_i2c_dbg_tx_calls++;
            mcx_i2c_dbg_tx_loops_total += loops;
            return ret;
        }

        if (status & LPI2C_MSR_TDF_MASK) {
            i2c_dbg_update_max(&mcx_i2c_dbg_tx_loops_max, loops);
            mcx_i2c_dbg_tx_calls++;
            mcx_i2c_dbg_tx_loops_total += loops;
            return I2C_BUS_SUCCESS;
        }

        if (!timer_is_before(timer_read_time(), timeout)) {
            i2c_dbg_update_max(&mcx_i2c_dbg_tx_loops_max, loops);
            mcx_i2c_dbg_tx_calls++;
            mcx_i2c_dbg_tx_loops_total += loops;

            i2c_recover_timeout(i2c);
            return I2C_BUS_TIMEOUT;
        }
    }
}

static int
i2c_read_byte(LPI2C_Type *i2c, uint8_t *data, uint32_t timeout)
{
    uint32_t loops = 0;

    for (;;) {
        loops++;

        uint32_t status = i2c->MSR;

        int ret = i2c_check_error(i2c, status);
        if (ret != I2C_BUS_SUCCESS) {
            i2c_dbg_update_max(&mcx_i2c_dbg_rx_loops_max, loops);
            mcx_i2c_dbg_rx_calls++;
            mcx_i2c_dbg_rx_loops_total += loops;
            return ret;
        }

        uint32_t value = i2c->MRDR;

        if (!(value & LPI2C_MRDR_RXEMPTY_MASK)) {
            *data = value & LPI2C_MRDR_DATA_MASK;

            i2c_dbg_update_max(&mcx_i2c_dbg_rx_loops_max, loops);
            mcx_i2c_dbg_rx_calls++;
            mcx_i2c_dbg_rx_loops_total += loops;

            return I2C_BUS_SUCCESS;
        }

        if (!timer_is_before(timer_read_time(), timeout)) {
            i2c_dbg_update_max(&mcx_i2c_dbg_rx_loops_max, loops);
            mcx_i2c_dbg_rx_calls++;
            mcx_i2c_dbg_rx_loops_total += loops;

            i2c_recover_timeout(i2c);
            return I2C_BUS_TIMEOUT;
        }
    }
}

static int
i2c_wait_stop(LPI2C_Type *i2c, uint32_t timeout)
{
    uint32_t loops = 0;

    for (;;) {
        loops++;

        uint32_t status = i2c->MSR;

        int ret = i2c_check_error(i2c, status);
        if (ret != I2C_BUS_SUCCESS) {
            i2c_dbg_update_max(&mcx_i2c_dbg_stop_loops_max, loops);
            mcx_i2c_dbg_stop_calls++;
            mcx_i2c_dbg_stop_loops_total += loops;
            return ret;
        }

        if (status & LPI2C_MSR_SDF_MASK) {
            i2c->MSR = LPI2C_MSR_SDF_MASK;

            i2c_dbg_update_max(&mcx_i2c_dbg_stop_loops_max, loops);
            mcx_i2c_dbg_stop_calls++;
            mcx_i2c_dbg_stop_loops_total += loops;

            return I2C_BUS_SUCCESS;
        }

        if (!timer_is_before(timer_read_time(), timeout)) {
            i2c_dbg_update_max(&mcx_i2c_dbg_stop_loops_max, loops);
            mcx_i2c_dbg_stop_calls++;
            mcx_i2c_dbg_stop_loops_total += loops;

            i2c_recover_timeout(i2c);
            return I2C_BUS_TIMEOUT;
        }
    }
}

int
i2c_write(struct i2c_config config, uint8_t write_len, uint8_t *write)
{
    uint32_t start_time = timer_read_time();

    LPI2C_Type *i2c = config.i2c;
    uint32_t timeout =
        start_time + timer_from_us(5000);

    i2c->MSR =
        LPI2C_MSR_SDF_MASK
        | LPI2C_MSR_NDF_MASK
        | LPI2C_MSR_ALF_MASK
        | LPI2C_MSR_FEF_MASK
        | LPI2C_MSR_PLTF_MASK;

    int ret = i2c_wait_tx_ready(i2c, timeout);
    if (ret != I2C_BUS_SUCCESS)
        goto out;

    i2c->MTDR =
        LPI2C_MTDR_CMD(I2C_CMD_START)
        | LPI2C_MTDR_DATA((uint32_t)config.addr << 1);

    mcx_i2c_dbg_after_start_msr = i2c->MSR;
    mcx_i2c_dbg_after_start_mfsr = i2c->MFSR;
    mcx_i2c_dbg_after_start_mcr = i2c->MCR;

    ret = i2c_wait_tx_ready(i2c, timeout);
    if (ret != I2C_BUS_SUCCESS)
        goto out;

    while (write_len--) {
        i2c->MTDR =
            LPI2C_MTDR_CMD(I2C_CMD_TX_DATA)
            | LPI2C_MTDR_DATA(*write++);

        ret = i2c_wait_tx_ready(i2c, timeout);
        if (ret != I2C_BUS_SUCCESS)
            goto out;
    }

    i2c->MTDR = LPI2C_MTDR_CMD(I2C_CMD_STOP);

    ret = i2c_wait_stop(i2c, timeout);

out:
    {
        uint32_t elapsed = timer_read_time() - start_time;

        mcx_i2c_dbg_write_calls++;
        mcx_i2c_dbg_write_ticks_total += elapsed;

        if (elapsed > mcx_i2c_dbg_write_ticks_max)
            mcx_i2c_dbg_write_ticks_max = elapsed;
    }

    return ret;
}

int
i2c_read(struct i2c_config config,
         uint8_t reg_len, uint8_t *reg,
         uint8_t read_len, uint8_t *read)
{
    uint32_t start_time = timer_read_time();

    LPI2C_Type *i2c = config.i2c;
    uint32_t timeout =
        start_time + timer_from_us(5000);

    if (!read_len) {
        uint32_t elapsed = timer_read_time() - start_time;

        mcx_i2c_dbg_read_calls++;
        mcx_i2c_dbg_read_ticks_total += elapsed;

        if (elapsed > mcx_i2c_dbg_read_ticks_max)
            mcx_i2c_dbg_read_ticks_max = elapsed;

        i2c_dbg_record_read_duration(elapsed);

        return I2C_BUS_SUCCESS;
    }

    /*
     * Clear sticky status from the previous transaction.
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
         * START + address, write direction.
         */
        ret = i2c_wait_tx_ready(i2c, timeout);
        if (ret != I2C_BUS_SUCCESS)
            goto out;

        i2c->MTDR =
            LPI2C_MTDR_CMD(I2C_CMD_START)
            | LPI2C_MTDR_DATA((uint32_t)config.addr << 1);

        mcx_i2c_dbg_read_start1_msr = i2c->MSR;
        mcx_i2c_dbg_read_start1_mfsr = i2c->MFSR;
        mcx_i2c_dbg_read_start1_mcr = i2c->MCR;

        ret = i2c_wait_tx_ready(i2c, timeout);
        if (ret != I2C_BUS_SUCCESS)
            goto out;

        mcx_i2c_dbg_after_tdf_msr = i2c->MSR;
        mcx_i2c_dbg_after_tdf_mfsr = i2c->MFSR;
        mcx_i2c_dbg_after_tdf_mcr = i2c->MCR;

        /*
         * Send register/subaddress bytes.
         */
        while (reg_len--) {
            i2c->MTDR =
                LPI2C_MTDR_CMD(I2C_CMD_TX_DATA)
                | LPI2C_MTDR_DATA(*reg++);

            ret = i2c_wait_tx_ready(i2c, timeout);
            if (ret != I2C_BUS_SUCCESS)
                goto out;
        }
    }

    /*
     * START/repeated START + address, read direction.
     */
    ret = i2c_wait_tx_ready(i2c, timeout);
    if (ret != I2C_BUS_SUCCESS)
        goto out;

    i2c->MTDR =
        LPI2C_MTDR_CMD(I2C_CMD_START)
        | LPI2C_MTDR_DATA(((uint32_t)config.addr << 1) | 1U);

    mcx_i2c_dbg_read_start2_msr = i2c->MSR;
    mcx_i2c_dbg_read_start2_mfsr = i2c->MFSR;
    mcx_i2c_dbg_read_start2_mcr = i2c->MCR;

    ret = i2c_wait_tx_ready(i2c, timeout);
    if (ret != I2C_BUS_SUCCESS)
        goto out;

    /*
     * Request read_len bytes.
     */
    i2c->MTDR =
        LPI2C_MTDR_CMD(I2C_CMD_RX_DATA)
        | LPI2C_MTDR_DATA((uint32_t)read_len - 1U);

    while (read_len--) {
        ret = i2c_read_byte(i2c, read, timeout);
        if (ret != I2C_BUS_SUCCESS)
            goto out;

        read++;
    }

    /*
     * Queue STOP and wait for completion.
     */
    ret = i2c_wait_tx_ready(i2c, timeout);
    if (ret != I2C_BUS_SUCCESS)
        goto out;

    i2c->MTDR =
        LPI2C_MTDR_CMD(I2C_CMD_STOP);

    ret = i2c_wait_stop(i2c, timeout);

    if (ret == I2C_BUS_SUCCESS)
        mcx_i2c_dbg_read_transactions++;

out:
    {
        uint32_t elapsed =
            timer_read_time() - start_time;

        mcx_i2c_dbg_read_calls++;
        mcx_i2c_dbg_read_ticks_total += elapsed;

        if (elapsed > mcx_i2c_dbg_read_ticks_max)
            mcx_i2c_dbg_read_ticks_max = elapsed;

        i2c_dbg_record_read_duration(elapsed);
    }

    return ret;
}