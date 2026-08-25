// MCXA366 clock setup
//
// Copyright (C) 2026
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include "internal.h"


#define IFR1_ADDR       0x01100000U
#define IFR1_240M_TRIM  (*(volatile uint32_t *)(IFR1_ADDR + 0x874U))


static void
set_main_clock(uint32_t source)
{
    SCG0->RCCR =
        (SCG0->RCCR & ~SCG_RCCR_SCS_MASK)
        | SCG_RCCR_SCS(source);

    while ((SCG0->CSR & SCG_CSR_SCS_MASK)
           != SCG_CSR_SCS(source))
        ;
}


static void
set_clock_divider(volatile uint32_t *reg, uint32_t value)
{
    /*
     * Match NXP CLOCK_SetClockDiv() for ordinary clock dividers.
     *
     * value is the actual divider:
     *
     *     value = 1 -> register DIV field = 0
     */
    SYSCON->CLKUNLOCK &= ~SYSCON_CLKUNLOCK_UNLOCK_MASK;

    /*
     * Assert RESET and HALT.
     */
    *reg = 0x3UL << 29U;

    if (!value) {
        /*
         * Leave divider halted.
         */
        *reg |= 1UL << 30U;
    } else {
        /*
         * Program divider while HALT remains asserted.
         */
        *reg = (value - 1U) | (1UL << 30U);

        /*
         * Release HALT.
         */
        *reg &= ~(1UL << 30U);
    }

    SYSCON->CLKUNLOCK |= SYSCON_CLKUNLOCK_UNLOCK_MASK;
}


static void
setup_power_240mhz(void)
{
    /*
     * NXP's 240 MHz configuration uses:
     *
     *     Core LDO voltage       = overdrive
     *     Core LDO drive         = normal
     *     Flash wait states      = 4
     *     SRAM operating voltage = 1.2 V
     *
     * Voltage changes require normal Core LDO drive strength.
     */
    while (SPC0->SC & SPC_SC_BUSY_MASK)
        ;

    SPC0->ACTIVE_CFG =
        (SPC0->ACTIVE_CFG
         & ~SPC_ACTIVE_CFG_CORELDO_VDD_DS_MASK)
        | SPC_ACTIVE_CFG_CORELDO_VDD_DS(1U);

    if ((SPC0->ACTIVE_CFG & SPC_ACTIVE_CFG_CORELDO_VDD_LVL_MASK)
        != SPC_ACTIVE_CFG_CORELDO_VDD_LVL(3U)) {
        SPC0->ACTIVE_CFG =
            (SPC0->ACTIVE_CFG
             & ~SPC_ACTIVE_CFG_CORELDO_VDD_LVL_MASK)
            | SPC_ACTIVE_CFG_CORELDO_VDD_LVL(3U);

        while (SPC0->SC & SPC_SC_BUSY_MASK)
            ;
    }

    /*
     * Keep the Core LDO in normal drive strength.
     */
    SPC0->ACTIVE_CFG =
        (SPC0->ACTIVE_CFG
         & ~SPC_ACTIVE_CFG_CORELDO_VDD_DS_MASK)
        | SPC_ACTIVE_CFG_CORELDO_VDD_DS(1U);

    /*
     * Four additional flash wait states are required above 90 MHz
     * in overdrive mode.
     */
    FMU0->FCTRL =
        (FMU0->FCTRL & ~FMU_FCTRL_RWSC_MASK)
        | FMU_FCTRL_RWSC(4U);

    /*
     * Configure SRAM timing for 1.2 V operation and request that
     * the hardware apply the new voltage setting.
     */
    SPC0->SRAMCTL = SPC_SRAMCTL_VSM(3U);
    SPC0->SRAMCTL |= SPC_SRAMCTL_REQ_MASK;

    while (!(SPC0->SRAMCTL & SPC_SRAMCTL_ACK_MASK))
        ;

    SPC0->SRAMCTL &= ~SPC_SRAMCTL_REQ_MASK;
}


static void
setup_ahb_divider(void)
{
    /*
     * AHBCLKDIV is special. Unlike most MCXA clock dividers it does
     * not implement RESET/HALT bits.
     *
     * Divide-by-1 is encoded as zero.
     */
    SYSCON->CLKUNLOCK &= ~SYSCON_CLKUNLOCK_UNLOCK_MASK;

    SYSCON->AHBCLKDIV = 0U;

    SYSCON->CLKUNLOCK |= SYSCON_CLKUNLOCK_UNLOCK_MASK;
}


static void
setup_fro12m(void)
{
    /*
     * Enable the 12 MHz SIRC/FRO12M peripheral clock.
     *
     * LPUART2 uses FRO_LF_DIV, and FRO12M also provides a safe
     * temporary MAIN_CLK source while FRO_HF is reconfigured.
     */
    SCG0->SIRCCSR &= ~SCG_SIRCCSR_LK_MASK;
    SCG0->SIRCCSR |= SCG_SIRCCSR_SIRC_CLK_PERIPH_EN_MASK;
    SCG0->SIRCCSR |= SCG_SIRCCSR_LK_MASK;

    while (!(SCG0->SIRCCSR & SCG_SIRCCSR_SIRCVLD_MASK))
        ;

    /*
     * FRO_LF_DIV = /1.
     */
    set_clock_divider(&SYSCON->FROLFDIV, 1U);
}


static void
setup_fro240m(void)
{
    uint32_t trim_value = IFR1_240M_TRIM;
    uint32_t current_source =
        (SCG0->CSR & SCG_CSR_SCS_MASK) >> SCG_CSR_SCS_SHIFT;

    /*
     * If the CPU is currently running from FRO_HF, move it to
     * FRO12M before modifying FRO_HF.
     *
     * MAIN_CLK selector:
     *
     *     2 = FRO12M
     *     3 = FRO_HF
     */
    if (current_source == 3U)
        set_main_clock(2U);

    /*
     * Load the factory 240 MHz FRO_HF trim from IFR1.
     */
    if (SCG0->FIRCTRIM != trim_value) {
        SCG0->TRIM_LOCK = 0x5A5A0001U;
        SCG0->FIRCTRIM = trim_value;
        SCG0->TRIM_LOCK = 0x5A5A0000U;
    }

    /*
     * FREQ_SEL=7 selects the full-frequency FRO_HF operating point.
     * With the MCXA366 240 MHz factory trim loaded, this is the
     * nominal 240 MHz FRO_HF configuration.
     */
    SCG0->FIRCCFG = SCG_FIRCCFG_FREQ_SEL(7U);

    /*
     * Enable both FRO_HF peripheral outputs and the FIRC itself.
     */
    SCG0->FIRCCSR &= ~SCG_FIRCCSR_LK_MASK;

    SCG0->FIRCCSR |=
        SCG_FIRCCSR_FIRC_SCLK_PERIPH_EN_MASK
        | SCG_FIRCCSR_FIRC_FCLK_PERIPH_EN_MASK
        | SCG_FIRCCSR_FIRCEN_MASK;

    SCG0->FIRCCSR |= SCG_FIRCCSR_LK_MASK;

    while (!(SCG0->FIRCCSR & SCG_FIRCCSR_FIRCACC_MASK))
        ;

    /*
     * FRO_HF_DIV = /1.
     */
    set_clock_divider(&SYSCON->FROHFDIV, 1U);
}


void
mcx_clock_init(void)
{
    /*
     * Configure the MCXA366 to run directly from the factory-trimmed
     * nominal 240 MHz FRO_HF.
     *
     * The order is important:
     *
     *     voltage/timing
     *     -> safe bus divider
     *     -> FRO12M fallback/peripheral clock
     *     -> factory-trimmed 240 MHz FRO_HF
     *     -> MAIN_CLK switch
     */
    setup_power_240mhz();

    setup_ahb_divider();

    setup_fro12m();

    setup_fro240m();

    /*
     * MAIN_CLK selector 3 = FRO_HF.
     */
    set_main_clock(3U);
}