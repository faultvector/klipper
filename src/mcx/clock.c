// MCXA366 clock setup
//
// Copyright (C) 2026
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include "internal.h"


void
mcx_clock_init(void)
{
    /*
     * Enable the 12 MHz FRO output for peripheral clock consumers.
     *
     * This mirrors the relevant part of NXP's
     * CLOCK_SetupFRO12MClocking():
     *
     *   - unlock SIRCCSR
     *   - enable SIRC peripheral clock output
     *   - relock SIRCCSR
     *   - wait for SIRC to become valid
     *
     * The Cortex-M core clock is separate from this peripheral clock
     * path. Klipper currently measures the core clock at approximately
     * 45 MHz, while LPUART2 is clocked from this 12 MHz FRO_LF path.
     */
    SCG0->SIRCCSR &= ~SCG_SIRCCSR_LK_MASK;
    SCG0->SIRCCSR |= SCG_SIRCCSR_SIRC_CLK_PERIPH_EN_MASK;
    SCG0->SIRCCSR |= SCG_SIRCCSR_LK_MASK;

    while (!(SCG0->SIRCCSR & SCG_SIRCCSR_SIRCVLD_MASK))
        ;

    /*
     * Configure FRO_LF_DIV for divide-by-1 and release it from HALT.
     *
     * NXP encodes divide-by-1 as DIV=0.
     *
     * Match the CLOCK_SetClockDiv() sequence:
     *
     *   RESET=1, HALT=1
     *   RESET=0, HALT=1, DIV=0
     *   RESET=0, HALT=0, DIV=0
     *
     * FROLFDIV is protected by SYSCON->CLKUNLOCK.
     */
    SYSCON->CLKUNLOCK &= ~SYSCON_CLKUNLOCK_UNLOCK_MASK;

    SYSCON->FROLFDIV =
        SYSCON_FROLFDIV_RESET_MASK
        | SYSCON_FROLFDIV_HALT_MASK;

    SYSCON->FROLFDIV =
        SYSCON_FROLFDIV_HALT_MASK;

    SYSCON->FROLFDIV = 0U;

    SYSCON->CLKUNLOCK |= SYSCON_CLKUNLOCK_UNLOCK_MASK;
}