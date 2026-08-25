// MCXA366 serial port
//
// Copyright (C) 2026
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include "autoconf.h"           // CONFIG_SERIAL_BAUD
#include "board/armcm_boot.h"   // armcm_enable_irq
#include "board/serial_irq.h"   // serial_rx_byte, serial_get_tx_byte
#include "command.h"            // DECL_CONSTANT_STR
#include "internal.h"
#include "sched.h"              // DECL_INIT


#define UART LPUART2

/*
 * LPUART2 uses FRO_LF_DIV.
 *
 * mcx_clock_init() configures the FRO12M peripheral output and sets
 * FRO_LF_DIV to divide-by-1, yielding a 12 MHz functional UART clock.
 */
#define UART_CLOCK_FREQ 12000000U


#define UART_CTRL_FLAGS \
    (LPUART_CTRL_RE_MASK \
     | LPUART_CTRL_TE_MASK \
     | LPUART_CTRL_RIE_MASK)


/*
 * FRDM-MCXA366:
 *
 *   P2_2 = LPUART2_TXD, ALT3
 *   P2_3 = LPUART2_RXD, ALT3
 *
 * This matches the NXP board pin configuration:
 *
 *   pull select       = up
 *   pull enable       = enabled
 *   slew rate         = normal
 *   mux               = ALT3
 *   input buffer      = enabled
 */
#define UART_PIN_PCR \
    (PORT_PCR_PS(1) \
     | PORT_PCR_PE(1) \
     | PORT_PCR_SRE(0) \
     | PORT_PCR_MUX(3) \
     | PORT_PCR_IBE(1))


DECL_CONSTANT_STR("RESERVE_PINS_serial", "P2_3,P2_2");

static void
enable_peripheral_clocks(void)
{
    /*
     * LPUART2:
     *   MRCC CC0 bit 25
     *
     * PORT2:
     *   MRCC CC1 bit 14
     *
     * CC1 SET is 0x10 bytes after CC0 SET.
     */
    SYSCON->CLKUNLOCK &= ~SYSCON_CLKUNLOCK_UNLOCK_MASK;

    MRCC0->MRCC_GLB_CC0_SET = 1U << 25;

    volatile uint32_t *cc1_set =
        (volatile uint32_t *)((uint32_t)&MRCC0->MRCC_GLB_CC0_SET + 0x10U);

    *cc1_set = 1U << 14;

    SYSCON->CLKUNLOCK |= SYSCON_CLKUNLOCK_UNLOCK_MASK;
}


static void
release_port_reset(void)
{
    /*
     * PORT2:
     *   MRCC reset register 1, bit 14
     *
     * SET releases the peripheral from reset.
     */
    SYSCON->CLKUNLOCK &= ~SYSCON_CLKUNLOCK_UNLOCK_MASK;

    MRCC0->MRCC_GLB_RST1_SET = 1U << 14;

    SYSCON->CLKUNLOCK |= SYSCON_CLKUNLOCK_UNLOCK_MASK;
}


static void
setup_uart_clock(void)
{
    /*
     * LPUART2 CLKSEL:
     *
     *   selector 0 = FRO_LF_DIV
     */
    SYSCON->CLKUNLOCK &= ~SYSCON_CLKUNLOCK_UNLOCK_MASK;

    MRCC0->MRCC_LPUART2_CLKSEL = 0U;

    /*
     * Configure the LPUART2 functional clock divider for divide-by-1.
     *
     * Match NXP CLOCK_SetClockDiv():
     *
     *   RESET=1, HALT=1
     *   RESET=0, HALT=1, DIV=0
     *   RESET=0, HALT=0, DIV=0
     *
     * DIV=0 encodes divide-by-1.
     */
    MRCC0->MRCC_LPUART2_CLKDIV =
        MRCC_MRCC_LPUART2_CLKDIV_RESET_MASK
        | MRCC_MRCC_LPUART2_CLKDIV_HALT_MASK;

    MRCC0->MRCC_LPUART2_CLKDIV =
        MRCC_MRCC_LPUART2_CLKDIV_HALT_MASK;

    MRCC0->MRCC_LPUART2_CLKDIV = 0U;

    SYSCON->CLKUNLOCK |= SYSCON_CLKUNLOCK_UNLOCK_MASK;
}


static void
reset_uart(void)
{
    /*
     * LPUART2:
     *   MRCC reset register 0, bit 25
     *
     * CLR asserts reset.
     * SET releases reset.
     *
     * This matches NXP RESET_PeripheralReset().
     */
    SYSCON->CLKUNLOCK &= ~SYSCON_CLKUNLOCK_UNLOCK_MASK;

    MRCC0->MRCC_GLB_RST0_CLR = 1U << 25;
    MRCC0->MRCC_GLB_RST0_SET = 1U << 25;

    SYSCON->CLKUNLOCK |= SYSCON_CLKUNLOCK_UNLOCK_MASK;
}


static void
software_reset_uart(void)
{
    /*
     * LPUART GLOBAL software reset.
     *
     * Match LPUART_SoftwareReset().
     */
    UART->GLOBAL |= LPUART_GLOBAL_RST_MASK;
    UART->GLOBAL &= ~LPUART_GLOBAL_RST_MASK;
}


static void
setup_uart_baud(void)
{
    uint32_t best_diff = CONFIG_SERIAL_BAUD;
    uint32_t best_osr = 0;
    uint32_t best_sbr = 0;

    /*
     * LPUART baud:
     *
     *   baud = UART_CLOCK_FREQ / (OSR * SBR)
     *
     * OSR may range from 4 through 32.
     */
    for (uint32_t osr = 4; osr <= 32; osr++) {
        uint64_t denom = (uint64_t)CONFIG_SERIAL_BAUD * osr;

        uint32_t sbr =
            (uint32_t)((((uint64_t)UART_CLOCK_FREQ * 2U) / denom + 1U)
                       / 2U);

        if (!sbr)
            sbr = 1;

        /*
         * SBR is the low-order field in BAUD. Clamp it to the largest
         * representable value.
         */
        if (sbr > LPUART_BAUD_SBR_MASK)
            sbr = LPUART_BAUD_SBR_MASK;

        uint32_t actual =
            UART_CLOCK_FREQ / (osr * sbr);

        uint32_t diff = actual > CONFIG_SERIAL_BAUD
            ? actual - CONFIG_SERIAL_BAUD
            : CONFIG_SERIAL_BAUD - actual;

        if (diff <= best_diff) {
            best_diff = diff;
            best_osr = osr;
            best_sbr = sbr;
        }
    }

    uint32_t baud = UART->BAUD;

    baud &= ~(LPUART_BAUD_OSR_MASK
              | LPUART_BAUD_SBR_MASK
              | LPUART_BAUD_BOTHEDGE_MASK);

    /*
     * NXP requires BOTHEDGE when OSR is in the low oversampling range.
     */
    if (best_osr < 8)
        baud |= LPUART_BAUD_BOTHEDGE_MASK;

    baud |= LPUART_BAUD_OSR(best_osr - 1U);
    baud |= LPUART_BAUD_SBR(best_sbr);

    UART->BAUD = baud;
}


static void
setup_uart_fifo(void)
{
    /*
     * Match the basic FIFO configuration used by the NXP driver:
     *
     *   - zero TX/RX watermark
     *   - enable TX and RX FIFO
     *   - flush both FIFOs
     */
    UART->WATER = 0U;

    UART->FIFO |=
        LPUART_FIFO_TXFE_MASK
        | LPUART_FIFO_RXFE_MASK;

    UART->FIFO |=
        LPUART_FIFO_TXFLUSH_MASK
        | LPUART_FIFO_RXFLUSH_MASK;
}


static void
clear_uart_status(void)
{
    /*
     * Clear the writable status/error flags before enabling the UART.
     */
    UART->STAT =
        LPUART_STAT_OR_MASK
        | LPUART_STAT_NF_MASK
        | LPUART_STAT_FE_MASK
        | LPUART_STAT_PF_MASK;
}


void __visible
LPUART2_IRQHandler(void)
{
    uint32_t stat = UART->STAT;

    if (stat & LPUART_STAT_RDRF_MASK)
        serial_rx_byte(UART->DATA);

    if ((stat & LPUART_STAT_TDRE_MASK)
        && (UART->CTRL & LPUART_CTRL_TIE_MASK)) {
        uint8_t data;
        int ret = serial_get_tx_byte(&data);

        if (ret)
            UART->CTRL = UART_CTRL_FLAGS;
        else
            UART->DATA = data;
    }
}


void
serial_enable_tx_irq(void)
{
    UART->CTRL = UART_CTRL_FLAGS | LPUART_CTRL_TIE_MASK;
}


void
serial_init(void)
{
    /*
     * Enable the register-interface clocks for LPUART2 and PORT2.
     */
    enable_peripheral_clocks();

    /*
     * PORT2 must be released before its PCR registers are configured.
     */
    release_port_reset();

    /*
     * P2_2 = LPUART2_TXD, ALT3
     * P2_3 = LPUART2_RXD, ALT3
     */
    *(volatile uint16_t *)&PORT2->PCR[2] = UART_PIN_PCR;
    *(volatile uint16_t *)&PORT2->PCR[3] = UART_PIN_PCR;

    /*
     * Feed LPUART2 from the 12 MHz FRO_LF_DIV source.
     */
    setup_uart_clock();

    /*
     * Start from a known peripheral state.
     */
    reset_uart();
    software_reset_uart();

    /*
     * Configure baud and FIFO state while TX/RX are still disabled.
     */
    setup_uart_baud();
    setup_uart_fifo();
    clear_uart_status();

    /*
     * Enable receiver, transmitter, and receive interrupts.
     *
     * TX-empty interrupts remain disabled until Klipper places data
     * into its transmit queue and calls serial_enable_tx_irq().
     */
    UART->CTRL = UART_CTRL_FLAGS;

    armcm_enable_irq(LPUART2_IRQHandler, LPUART2_IRQn, 0);
}
DECL_INIT(serial_init);