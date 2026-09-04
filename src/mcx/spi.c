// SPI functions on MCXA366
//
// Copyright (C) 2026
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include "command.h" // DECL_ENUMERATION, shutdown
#include "gpio.h" // spi_config
#include "internal.h" // GPIO
#include "sched.h" // sched_shutdown

#define LPSPI1_CLOCK 12000000U


/*
 * FRDM-MCXA366 LPSPI1 pin mapping:
 *
 *     MISO / SDI   P3_9
 *     MOSI / SDO   P3_8
 *     SCK          P3_10
 *
 * Chip select is handled separately by Klipper as a normal GPIO.
 */
DECL_ENUMERATION("spi_bus", "spi1", 0);
DECL_CONSTANT_STR("BUS_PINS_spi1", "P3_9,P3_8,P3_10");


struct spi_info {
    LPSPI_Type *spi;
    uint32_t miso;
    uint32_t mosi;
    uint32_t sck;
};

static const struct spi_info spi_bus[] = {
    {
        .spi = LPSPI1,
        .miso = GPIO(3, 9),
        .mosi = GPIO(3, 8),
        .sck = GPIO(3, 10),
    },
};


static void
spi1_clock_setup(void)
{
    /*
     * LPSPI1 uses FRO_LF_DIV.
     *
     * mcx/clock.c configures:
     *
     *     FRO12M
     *        |
     *        +--> FRO_LF_DIV / 1
     *
     * so the LPSPI1 functional clock is 12 MHz.
     */

    /*
     * Clock selector:
     *
     *     0 = FRO_LF_DIV
     */
    SYSCON->CLKUNLOCK &= ~SYSCON_CLKUNLOCK_UNLOCK_MASK;

    MRCC0->MRCC_LPSPI1_CLKSEL = 0U;

    /*
     * Reset and halt the clock divider before changing it.
     */
    MRCC0->MRCC_LPSPI1_CLKDIV =
        MRCC_MRCC_LPSPI1_CLKDIV_RESET_MASK
        | MRCC_MRCC_LPSPI1_CLKDIV_HALT_MASK;

    /*
     * DIV field is encoded as divisor - 1.
     *
     * DIV=0 therefore means divide-by-1.
     */
    MRCC0->MRCC_LPSPI1_CLKDIV =
        MRCC_MRCC_LPSPI1_CLKDIV_HALT_MASK
        | MRCC_MRCC_LPSPI1_CLKDIV_DIV(0U);

    /*
     * Clear HALT to start the divider.
     */
    MRCC0->MRCC_LPSPI1_CLKDIV &=
        ~MRCC_MRCC_LPSPI1_CLKDIV_HALT_MASK;

    SYSCON->CLKUNLOCK |= SYSCON_CLKUNLOCK_UNLOCK_MASK;

    /*
     * LPSPI1 is bit 22 in CC0 and RST0.
     *
     * On MCXA366, writing the SET register releases the peripheral
     * clock/reset state.
     */
    MRCC0->MRCC_GLB_CC0_SET = 1U << 22;
    MRCC0->MRCC_GLB_RST0_SET = 1U << 22;
}

static void
spi1_pin_setup(void)
{
    /*
     * LPSPI1:
     *
     *     P3_8  = SDO / MOSI
     *     P3_9  = SDI / MISO
     *     P3_10 = SCK
     *
     * All three use ALT2.
     *
     * P3_11 can provide LPSPI1_PCS0, but we intentionally leave it
     * as GPIO because Klipper handles chip select outside the SPI
     * peripheral.
     */

    /*
     * PORT3 is bit 15 in CC1/RST1.
     */
    MRCC0->MRCC_GLB_CC1_SET = 1U << 15;
    MRCC0->MRCC_GLB_RST1_SET = 1U << 15;

    /*
     * Match the important parts of NXP's generated pin configuration:
     *
     *     ALT2
     *     pull disabled
     *     fast slew
     *     open drain disabled
     *     digital input buffer enabled
     */
    PORT3->PCR[8] =
        PORT_PCR_MUX(2U)
        | PORT_PCR_SRE_MASK
        | PORT_PCR_IBE_MASK;

    PORT3->PCR[9] =
        PORT_PCR_MUX(2U)
        | PORT_PCR_SRE_MASK
        | PORT_PCR_IBE_MASK;

    PORT3->PCR[10] =
        PORT_PCR_MUX(2U)
        | PORT_PCR_SRE_MASK
        | PORT_PCR_IBE_MASK;
}

static void
spi1_init(void)
{
    LPSPI_Type *spi = LPSPI1;

    spi1_clock_setup();
    spi1_pin_setup();

    /*
     * Reset both FIFOs.
     */
    spi->CR = LPSPI_CR_RRF_MASK | LPSPI_CR_RTF_MASK;

    /*
     * No interrupts. Initial implementation is entirely polling based.
     */
    spi->IER = 0U;

    /*
     * Leave the peripheral disabled while programming configuration.
     */
    spi->CR = 0U;

    /*
     * Configure:
     *
     *     master mode
     *     PINCFG=0: normal SDI input / SDO output
     *     peripheral stalling enabled
     */
    spi->CFGR1 =
        LPSPI_CFGR1_MASTER_MASK
        | LPSPI_CFGR1_PINCFG(0U);

    /*
     * TX and RX FIFO watermark = 0.
     */
    spi->FCR = 0U;
}

static void
spi_calc_rate(uint32_t rate, uint32_t *prescale, uint32_t *scaler)
{
    uint32_t best_prescale = 7U;
    uint32_t best_scaler = 255U;
    uint32_t best_diff = 0xffffffffU;

    if (!rate)
        shutdown("Invalid spi rate");

    /*
     * MCXA366 LPSPI master clock:
     *
     *                    source clock
     *     SCK = --------------------------------
     *           2^PRESCALE * (SCKDIV + 2)
     *
     * Match the NXP SDK behavior: find the closest frequency that
     * does not exceed the requested rate.
     */
    for (uint32_t p = 0U; p < 8U; p++) {
        for (uint32_t s = 0U; s < 256U; s++) {
            uint32_t actual =
                LPSPI1_CLOCK / ((1U << p) * (s + 2U));

            if (actual > rate)
                continue;

            uint32_t diff = rate - actual;
            if (diff < best_diff) {
                best_diff = diff;
                best_prescale = p;
                best_scaler = s;

                if (!diff)
                    goto found;
            }
        }
    }

found:
    *prescale = best_prescale;
    *scaler = best_scaler;
}

struct spi_config
spi_setup(uint32_t bus, uint8_t mode, uint32_t rate)
{
    if (bus >= ARRAY_SIZE(spi_bus))
        shutdown("Invalid spi bus");

    if (mode > 3U)
        shutdown("Invalid spi mode");

    /*
     * Only one hardware SPI bus is currently exposed.
     *
     * Reinitializing it here keeps first-light bring-up deterministic.
     * We can move this to one-time initialization later if necessary.
     */
    spi1_init();

    uint32_t prescale, scaler;
    spi_calc_rate(rate, &prescale, &scaler);

    /*
     * Klipper SPI modes map directly to CPOL/CPHA:
     *
     *     mode 0 = CPOL 0, CPHA 0
     *     mode 1 = CPOL 0, CPHA 1
     *     mode 2 = CPOL 1, CPHA 0
     *     mode 3 = CPOL 1, CPHA 1
     *
     * FRAMESZ stores bits-per-frame minus one, so 7 means 8-bit
     * transfers.
     */
    struct spi_config config = {
        .spi = spi_bus[bus].spi,
        .ccr = LPSPI_CCR_SCKDIV(scaler),
        .tcr = LPSPI_TCR_FRAMESZ(7U)
             | LPSPI_TCR_PRESCALE(prescale)
             | LPSPI_TCR_CPOL((mode & 2U) != 0U)
             | LPSPI_TCR_CPHA((mode & 1U) != 0U),
    };

    return config;
}

void
spi_prepare(struct spi_config config)
{
    LPSPI_Type *spi = config.spi;

    /*
     * Ensure the previous transaction has completely finished.
     */
    while (spi->SR & LPSPI_SR_MBF_MASK)
        ;

    /*
     * Configuration registers are changed while the peripheral is
     * disabled.
     */
    spi->CR &= ~LPSPI_CR_MEN_MASK;

    /*
     * Remove any stale TX or RX FIFO contents.
     */
    spi->CR |= LPSPI_CR_RRF_MASK | LPSPI_CR_RTF_MASK;

    spi->CCR = config.ccr;
    spi->TCR = config.tcr;

    /*
     * Enable LPSPI.
     */
    spi->CR |= LPSPI_CR_MEN_MASK;
}

void
spi_transfer(struct spi_config config, uint8_t receive_data,
             uint8_t len, uint8_t *data)
{
    LPSPI_Type *spi = config.spi;

    /*
     * First-light implementation deliberately transfers one byte at a
     * time instead of filling the hardware FIFO.
     *
     * That keeps the polling logic simple while validating the port.
     */
    while (len--) {
        /*
         * Wait for room in the transmit FIFO.
         */
        while (!(spi->SR & LPSPI_SR_TDF_MASK))
            ;

        spi->TDR = *data;

        /*
         * Every transmitted byte also clocks one received byte into
         * the receive FIFO.
         */
        while (!(spi->SR & LPSPI_SR_RDF_MASK))
            ;

        uint8_t value = spi->RDR;

        if (receive_data)
            *data = value;

        data++;
    }

    /*
     * Do not return until the final SPI frame has actually completed.
     *
     * Klipper's generic SPI layer may deassert the GPIO chip-select
     * immediately after this function returns.
     */
    while (spi->SR & LPSPI_SR_MBF_MASK)
        ;
}