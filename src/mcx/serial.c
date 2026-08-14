#include "autoconf.h"           // CONFIG_SERIAL_BAUD
#include "board/armcm_boot.h"   // armcm_enable_irq
#include "board/serial_irq.h"   // serial_rx_byte, serial_get_tx_byte
#include "command.h"            // DECL_CONSTANT_STR
#include "internal.h"
#include "sched.h"              // DECL_INIT

#define UART LPUART2
#define UART_CLOCK_FREQ 12000000U

#define UART_CTRL_FLAGS \
    (LPUART_CTRL_RE_MASK | LPUART_CTRL_TE_MASK | LPUART_CTRL_RIE_MASK)

#define UART_PIN_PCR \
    (PORT_PCR_PS(1) \
     | PORT_PCR_PE(1) \
     | PORT_PCR_SRE(1) \
     | PORT_PCR_MUX(3) \
     | PORT_PCR_IBE(1))

DECL_CONSTANT_STR("RESERVE_PINS_serial", "P2_3,P2_2");


static void
enable_port2_clock(void)
{
    /*
     * kCLOCK_GatePORT2 = ((0x10 << 16) | 14)
     * 
     * CLOCK_EnableClock() therefore writes bit 14 to the CCx SET
     * register located 0x10 bytes after MRCC_GLB_CC0_SET.
     */
    SYSCON->CLKUNLOCK &= ~SYSCON_CLKUNLOCK_UNLOCK_MASK;

    volatile uint32_t *cc =
        (volatile uint32_t *)((uint32_t)&MRCC0->MRCC_GLB_CC0_SET + 0x10U);

    *cc = 1U << 14;

    SYSCON->CLKUNLOCK |= SYSCON_CLKUNLOCK_UNLOCK_MASK;
}


static void
release_resets(void)
{
    SYSCON->CLKUNLOCK &= ~SYSCON_CLKUNLOCK_UNLOCK_MASK;

    /*
     * LPUART2:
     *   reset register 0, bit 25
     * 
     * PORT2:
     *   reset register 1, bit 14
     * 
     * On this MCXA reset block, SET releases the peripheral from reset.
     */
    MRCC0->MRCC_GLB_RST0_SET = 1U << 25;
    MRCC0->MRCC_GLB_RST1_SET = 1U << 14;

    SYSCON->CLKUNLOCK |= SYSCON_CLKUNLOCK_UNLOCK_MASK;
}


static void
setup_uart_clock(void)
{
    /*
     * LPUART2 CLKSEL offset 0x110.
     * Selector 0 = FRO_LF_DIV.
     */
    SYSCON->CLKUNLOCK &= ~SYSCON_CLKUNLOCK_UNLOCK_MASK;

    MRCC0->MRCC_LPUART2_CLKSEL = 0U;

    /*
     * Divider = 1.
     *
     * Match CLOCK_SetClockDiv():
     *   assert RESET+HALT,
     *   program DIV-0 with HALT,
     *   then clear HALT. 
     */
    MRCC0->MRCC_LPUART2_CLKDIV =
        MRCC_MRCC_LPUART2_CLKDIV_RESET_MASK
        | MRCC_MRCC_LPUART2_CLKDIV_HALT_MASK;

    MRCC0->MRCC_LPUART2_CLKDIV =
        MRCC_MRCC_LPUART3_CLKDIV_HALT_MASK;

    MRCC0->MRCC_LPUART2_CLKDIV &=
        ~MRCC_MRCC_LPUART3_CLKDIV_HALT_MASK;

    SYSCON->CLKUNLOCK |= SYSCON_CLKUNLOCK_UNLOCK_MASK;
}


static void
setup_uart_baud(void)
{
    uint32_t best_diff = CONFIG_SERIAL_BAUD;
    uint32_t best_osr = 0;
    uint32_t best_sbr = 0;

    for (uint32_t osr = 4; osr <= 32; osr++) {
        uint64_t denom = (uint64_t)CONFIG_SERIAL_BAUD *osr;

        uint32_t sbr = 
            (uint32_t)((((uint64_t)UART_CLOCK_FREQ * 2U) / denom + 1U)
                        / 2U);
        
        if (!sbr)
            sbr = 1;
        if (sbr < LPUART_BAUD_SBR_MASK)
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

    if (best_osr < 8)
        baud |= LPUART_BAUD_BOTHEDGE_MASK;

    baud |= LPUART_BAUD_OSR(best_osr - 1);
    baud |= LPUART_BAUD_SBR(best_sbr);

    UART->BAUD = baud;
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
    enable_port2_clock();
    release_resets();

    /*
     * P2_2 = LPUART2_TXD, ALT3
     * P2_3 = LPUART2_RXD, ALT3
     */
    *(volatile uint16_t *)&PORT2->PCR[2] = UART_PIN_PCR;
    *(volatile uint16_t *)&PORT2->PCR[3] = UART_PIN_PCR;

    setup_uart_clock();
    setup_uart_baud();

    UART->CTRL = UART_CTRL_FLAGS;

    armcm_enable_irq(LPUART2_IRQHandler, LPUART2_IRQn, 0);
}
DECL_INIT(serial_init);