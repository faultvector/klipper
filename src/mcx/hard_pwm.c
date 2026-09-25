// Hardware PWM support on MCXA366
//
// Copyright (C) 2026
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include "autoconf.h" // CONFIG_CLOCK_FREQ
#include "command.h" // DECL_CONSTANT, shutdown
#include "gpio.h" // gpio_pwm_setup
#include "internal.h" // GPIO
#include "sched.h" // shutdown


#define MAX_PWM (1U << 15)

DECL_CONSTANT("PWM_MAX", MAX_PWM);


/*
 * First-light CTIMER route:
 *
 *     P3_12
 *       -> ALT4
 *       -> CTIMER1 MAT2
 *
 * NXP's FRDM-MCXA366 simple PWM example uses:
 *
 *     CTIMER1
 *     MAT2 as PWM output
 *     MAT3 as the PWM period channel
 */
#define PWM_TEST_PIN          GPIO(3, 12)
#define PWM_TEST_MUX          4U

#define PWM_OUTPUT_CHANNEL    2U
#define PWM_PERIOD_CHANNEL    3U


/*
 * CTIMER1 is clocked from FRO_LF_DIV.
 *
 * clock.c configures FRO_LF_DIV as:
 *
 *     FRO12M / 1 = 12 MHz
 */
#define CTIMER_CLOCK_HZ       12000000U


static uint32_t ctimer1_period_ticks;


/****************************************************************
 * Clock and pin setup
 ****************************************************************/

static void
ctimer1_clock_setup(void)
{
    uint32_t clkunlock = SYSCON->CLKUNLOCK;

    SYSCON->CLKUNLOCK =
        clkunlock & ~SYSCON_CLKUNLOCK_UNLOCK_MASK;

    /*
     * Enable CTIMER1 register/interface clock.
     */
    MRCC0->MRCC_GLB_CC0_SET =
        MRCC_MRCC_GLB_CC0_CTIMER1_MASK;

    /*
     * CTIMER1 clock source:
     *
     *     MUX 0 = FRO_LF_DIV
     *
     * FRO_LF_DIV is configured as 12 MHz / 1.
     */
    MRCC0->MRCC_CTIMER1_CLKSEL =
        MRCC_MRCC_CTIMER1_CLKSEL_MUX(0U);

    /*
     * Functional clock divider = /1.
     */
    MRCC0->MRCC_CTIMER1_CLKDIV =
        MRCC_MRCC_CTIMER1_CLKDIV_RESET_MASK
        | MRCC_MRCC_CTIMER1_CLKDIV_HALT_MASK;

    MRCC0->MRCC_CTIMER1_CLKDIV =
        MRCC_MRCC_CTIMER1_CLKDIV_HALT_MASK
        | MRCC_MRCC_CTIMER1_CLKDIV_DIV(0U);

    MRCC0->MRCC_CTIMER1_CLKDIV =
        MRCC_MRCC_CTIMER1_CLKDIV_DIV(0U);

    /*
     * Release CTIMER1 from reset.
     */
    MRCC0->MRCC_GLB_RST0_SET =
        MRCC_MRCC_GLB_RST0_CTIMER1_MASK;

    SYSCON->CLKUNLOCK = clkunlock;
}


static void
pwm_pin_setup(void)
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
     * P3_12 = CT1_MAT2 on ALT4.
     */
    PORT3->PCR[12] =
        PORT_PCR_MUX(PWM_TEST_MUX)
        | PORT_PCR_SRE_MASK;
}


/****************************************************************
 * Timing conversion
 ****************************************************************/

static uint32_t
cycle_time_to_ctimer_ticks(uint32_t cycle_time)
{
    /*
     * Klipper expresses cycle_time in CONFIG_CLOCK_FREQ ticks:
     *
     *     requested_period = cycle_time / CONFIG_CLOCK_FREQ
     *
     * CTIMER1 runs at CTIMER_CLOCK_HZ, so:
     *
     *     ctimer_ticks =
     *         cycle_time * CTIMER_CLOCK_HZ / CONFIG_CLOCK_FREQ
     *
     * Use 64-bit arithmetic to avoid overflow before division.
     */
    uint32_t ticks =
        ((uint64_t)cycle_time * CTIMER_CLOCK_HZ
         + CONFIG_CLOCK_FREQ / 2U)
        / CONFIG_CLOCK_FREQ;

    /*
     * A one-tick period is not useful for PWM
     */
    if (ticks < 2U)
        ticks = 2U;

    return (uint32_t)ticks;
}


/****************************************************************
 * Duty-cycle update
 ****************************************************************/

void 
gpio_pwm_write(struct gpio_pwm g, uint32_t val)
{
    CTIMER_Type *timer = g.timer;

    if (val > MAX_PWM)
        val = MAX_PWM;

    /*
     * NXP CTIMER PWM semantics:
     *
     *     period MR = period_ticks - 1
     *
     *     pulse MR =
     *         period_ticks * (1 - duty)
     *
     * Examples:
     *
     *     0%   -> pulse = period_ticks
     *             no pulse match occurs before the period reset
     *
     *     50%  -> pulse = period_ticks / 2
     *
     *     100% -> pulse = 0
     */
    uint64_t pulse =
        ((uint32_t)g.hwpwm_ticks * (MAX_PWM - val)
         + MAX_PWM / 2U)
         / MAX_PWM;

    timer->MR[g.channel] = (uint32_t)pulse;
}


/****************************************************************
 * PWM setup
 ****************************************************************/

 struct gpio_pwm
 gpio_pwm_setup(uint8_t pin, uint32_t cycle_time, uint32_t val)
 {
    if (pin != PWM_TEST_PIN)
        shutdown("Not a valid PWM pin");

    uint32_t period_ticks =
        cycle_time_to_ctimer_ticks(cycle_time);

    ctimer1_clock_setup();
    pwm_pin_setup();

    /*
     * If CTIMER1 is already active, any additional output on this
     * timer must use the same PWM period.
     */
    if (CTIMER1->TCR & CTIMER_TCR_CEN_MASK) {
       if (ctimer1_period_ticks != period_ticks)
           shutdown("PWM timer already programmed at different speed");
    } else {
        /*
         * Start from a known timer state.
         */
        CTIMER1->TCR = CTIMER_TCR_CRST_MASK;

        CTIMER1->CTCR = 0U;
        CTIMER1->PR = 0U;
        CTIMER1->PC = 0U;
        CTIMER1->TC = 0U;

        /*
         * MAT3vdefines the PWM period.
         *
         * Reset the counter whenever TC matches MR3.
         */
         CTIMER1->MCR =
            CTIMER_MCR_MR3R_MASK;

        CTIMER1->MR[PWM_PERIOD_CHANNEL] =
            period_ticks - 1U;

        /*
         * No PWM channels enabled yet.
         */
        CTIMER1->PWMC = 0U;

        ctimer1_period_ticks = period_ticks;

        /*
         * Release the counter from reset, but leave it stopped until
         * the output channel has been fully configured.
         */
        CTIMER1->TCR = 0U;
    }

    /*
     * Enable PWM mode on MAT2.
     */
    CTIMER1->PWMC |=
        1U << PWM_OUTPUT_CHANNEL;

    struct gpio_pwm g = {
        .timer = CTIMER1,
        .hwpwm_ticks = period_ticks,
        .channel = PWM_OUTPUT_CHANNEL,
    };

    /*
     * Program the initial requested duty before starting the timer.
     */
    gpio_pwm_write(g, val);

    /*
     * Start CTIMER1.
     */
    CTIMER1->TCR |=
        CTIMER_TCR_CEN_MASK;

    return g;
} 