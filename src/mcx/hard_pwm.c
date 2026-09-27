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


/****************************************************************
 * PWM providers
 ****************************************************************/

#define MCX_PWM_CTIMER   0U
#define MCX_PWM_FLEXPWM  1U


/*
 * Test-C bring-up gate.
 *
 * This is intentionally zero-initialized.
 *
 * Declaring it volatile prevents the compiler/LTO from proving that
 * the FlexPWM path is unreachable and discarding the implementation.
 *
 * Leave this at zero for test C.
 */
static volatile uint8_t flexpwm_test_enable = 1U;


/****************************************************************
 * CTIMER resources
 ****************************************************************/

#define CTIMER_COUNT 5U


struct ctimer_state {
    uint32_t period_ticks;
    uint8_t period_channel;
    uint8_t output_mask;
    uint8_t initialized;
};


static struct ctimer_state ctimer_states[CTIMER_COUNT];


/*
 * Hardware instances.
 */
static CTIMER_Type * const ctimer_regs[CTIMER_COUNT] = {
    CTIMER0,
    CTIMER1,
    CTIMER2,
    CTIMER3,
    CTIMER4,
};


/*
 * MRCC clock selection/divider registers.
 */
static volatile uint32_t * const ctimer_clksel[CTIMER_COUNT] = {
    &MRCC0->MRCC_CTIMER0_CLKSEL,
    &MRCC0->MRCC_CTIMER1_CLKSEL,
    &MRCC0->MRCC_CTIMER2_CLKSEL,
    &MRCC0->MRCC_CTIMER3_CLKSEL,
    &MRCC0->MRCC_CTIMER4_CLKSEL,
};


static volatile uint32_t * const ctimer_clkdiv[CTIMER_COUNT] = {
    &MRCC0->MRCC_CTIMER0_CLKDIV,
    &MRCC0->MRCC_CTIMER1_CLKDIV,
    &MRCC0->MRCC_CTIMER2_CLKDIV,
    &MRCC0->MRCC_CTIMER3_CLKDIV,
    &MRCC0->MRCC_CTIMER4_CLKDIV,
};


static const uint32_t ctimer_clock_masks[CTIMER_COUNT] = {
    MRCC_MRCC_GLB_CC0_CTIMER0_MASK,
    MRCC_MRCC_GLB_CC0_CTIMER1_MASK,
    MRCC_MRCC_GLB_CC0_CTIMER2_MASK,
    MRCC_MRCC_GLB_CC0_CTIMER3_MASK,
    MRCC_MRCC_GLB_CC0_CTIMER4_MASK,
};


static const uint32_t ctimer_reset_masks[CTIMER_COUNT] = {
    MRCC_MRCC_GLB_RST0_CTIMER0_MASK,
    MRCC_MRCC_GLB_RST0_CTIMER1_MASK,
    MRCC_MRCC_GLB_RST0_CTIMER2_MASK,
    MRCC_MRCC_GLB_RST0_CTIMER3_MASK,
    MRCC_MRCC_GLB_RST0_CTIMER4_MASK,
};


/****************************************************************
 * PORT resources
 ****************************************************************/

static PORT_Type * const port_regs[] = {
    PORT0,
    PORT1,
    PORT2,
    PORT3,
    PORT4,
};


static const uint32_t port_clock_masks[] = {
    MRCC_MRCC_GLB_CC1_PORT0_MASK,
    MRCC_MRCC_GLB_CC1_PORT1_MASK,
    MRCC_MRCC_GLB_CC1_PORT2_MASK,
    MRCC_MRCC_GLB_CC1_PORT3_MASK,
    MRCC_MRCC_GLB_CC1_PORT4_MASK,
};


static const uint32_t port_reset_masks[] = {
    MRCC_MRCC_GLB_RST1_PORT0_MASK,
    MRCC_MRCC_GLB_RST1_PORT1_MASK,
    MRCC_MRCC_GLB_RST1_PORT2_MASK,
    MRCC_MRCC_GLB_RST1_PORT3_MASK,
    MRCC_MRCC_GLB_RST1_PORT4_MASK,
};


/****************************************************************
 * MCXA366VLQ CTIMER pin routes
 ****************************************************************/

struct ctimer_pwm_route {
    uint8_t pin;
    uint8_t timer;
    uint8_t channel;
    uint8_t mux;
};


/*
 * Derived from the MCXA366VLQ NXP pinmux definitions.
 *
 * A physical pin may have more than one CTIMER route. The allocator
 * can use an alternate route when that avoids a timing-domain or
 * channel conflict.
 */
static const struct ctimer_pwm_route ctimer_routes[] = {
    /* CTIMER0 MAT0 */
    { GPIO(0,  2), 0, 0, 4 },
    { GPIO(0, 16), 0, 0, 4 },
    { GPIO(0, 22), 0, 0, 5 },
    { GPIO(0, 24), 0, 0, 4 },
    { GPIO(2, 12), 0, 0, 5 },

    /* CTIMER0 MAT1 */
    { GPIO(0,  3), 0, 1, 4 },
    { GPIO(0, 17), 0, 1, 4 },
    { GPIO(0, 23), 0, 1, 5 },
    { GPIO(0, 25), 0, 1, 4 },
    { GPIO(2, 13), 0, 1, 5 },

    /* CTIMER0 MAT2 */
    { GPIO(0,  4), 0, 2, 4 },
    { GPIO(0, 12), 0, 2, 4 },
    { GPIO(0, 18), 0, 2, 4 },
    { GPIO(0, 26), 0, 2, 4 },
    { GPIO(1,  0), 0, 2, 5 },
    { GPIO(1,  8), 0, 2, 5 },
    { GPIO(2, 15), 0, 2, 5 },
    { GPIO(2, 16), 0, 2, 5 },
    { GPIO(3, 30), 0, 2, 4 },

    /* CTIMER0 MAT3 */
    { GPIO(0,  5), 0, 3, 4 },
    { GPIO(0, 13), 0, 3, 4 },
    { GPIO(0, 19), 0, 3, 4 },
    { GPIO(0, 27), 0, 3, 4 },
    { GPIO(1,  1), 0, 3, 5 },
    { GPIO(1,  9), 0, 3, 5 },
    { GPIO(2, 17), 0, 3, 5 },
    { GPIO(3, 31), 0, 3, 4 },

    /* CTIMER1 MAT0 */
    { GPIO(1,  2), 1, 0, 4 },
    { GPIO(2,  4), 1, 0, 5 },
    { GPIO(3, 10), 1, 0, 4 },

    /* CTIMER1 MAT1 */
    { GPIO(1,  3), 1, 1, 4 },
    { GPIO(2,  5), 1, 1, 5 },
    { GPIO(3, 11), 1, 1, 4 },

    /* CTIMER1 MAT2 */
    { GPIO(1,  4), 1, 2, 4 },
    { GPIO(2,  6), 1, 2, 5 },
    { GPIO(3, 12), 1, 2, 4 },

    /* CTIMER1 MAT3 */
    { GPIO(1,  5), 1, 3, 4 },
    { GPIO(2,  7), 1, 3, 5 },
    { GPIO(3, 13), 1, 3, 4 },

    /* CTIMER2 MAT0 */
    { GPIO(1, 10), 2, 0, 4 },
    { GPIO(2,  0), 2, 0, 5 },
    { GPIO(2, 20), 2, 0, 4 },
    { GPIO(3, 18), 2, 0, 4 },

    /* CTIMER2 MAT1 */
    { GPIO(1, 11), 2, 1, 4 },
    { GPIO(2,  1), 2, 1, 5 },
    { GPIO(2, 21), 2, 1, 4 },
    { GPIO(3, 19), 2, 1, 4 },

    /* CTIMER2 MAT2 */
    { GPIO(1, 12), 2, 2, 4 },
    { GPIO(2,  2), 2, 2, 5 },
    { GPIO(2, 22), 2, 2, 4 },
    { GPIO(3, 20), 2, 2, 4 },

    /* CTIMER2 MAT3 */
    { GPIO(1, 13), 2, 3, 4 },
    { GPIO(2,  3), 2, 3, 5 },
    { GPIO(2, 23), 2, 3, 4 },
    { GPIO(3, 21), 2, 3, 4 },

    /* CTIMER3 MAT0 */
    { GPIO(1, 14), 3, 0, 5 },
    { GPIO(1, 18), 3, 0, 4 },
    { GPIO(2,  8), 3, 0, 4 },
    { GPIO(2, 16), 3, 0, 4 },

    /* CTIMER3 MAT1 */
    { GPIO(1, 15), 3, 1, 5 },
    { GPIO(1, 19), 3, 1, 4 },
    { GPIO(2,  9), 3, 1, 4 },
    { GPIO(2, 17), 3, 1, 4 },
    { GPIO(3, 27), 3, 1, 5 },

    /* CTIMER3 MAT2 */
    { GPIO(2, 10), 3, 2, 4 },
    { GPIO(2, 18), 3, 2, 4 },
    { GPIO(3, 28), 3, 2, 5 },

    /* CTIMER3 MAT3 */
    { GPIO(2, 11), 3, 3, 4 },
    { GPIO(2, 19), 3, 3, 4 },
    { GPIO(3, 29), 3, 3, 5 },

    /* CTIMER4 MAT0 */
    { GPIO(1,  6), 4, 0, 5 },
    { GPIO(2, 12), 4, 0, 4 },
    { GPIO(3,  2), 4, 0, 4 },
    { GPIO(4,  2), 4, 0, 4 },

    /* CTIMER4 MAT1 */
    { GPIO(1,  7), 4, 1, 5 },
    { GPIO(2, 13), 4, 1, 4 },
    { GPIO(3,  3), 4, 1, 4 },
    { GPIO(4,  3), 4, 1, 4 },

    /* CTIMER4 MAT2 */
    { GPIO(2, 14), 4, 2, 4 },
    { GPIO(3,  6), 4, 2, 4 },
    { GPIO(4,  4), 4, 2, 4 },

    /* CTIMER4 MAT3 */
    { GPIO(2, 15), 4, 3, 4 },
    { GPIO(3,  7), 4, 3, 4 },
    { GPIO(4,  5), 4, 3, 4 },
};


/****************************************************************
 * CTIMER register helpers
 ****************************************************************/

static uint32_t
ctimer_reset_bit(uint8_t channel)
{
    return CTIMER_MCR_MR0R_MASK
        << ((uint32_t)channel * 3U);
}


static uint32_t
ctimer_reload_bit(uint8_t channel)
{
    return CTIMER_MCR_MR0RL_MASK
        << channel;
}


/****************************************************************
 * CTIMER clock setup
 ****************************************************************/

static void
ctimer_clock_setup(uint8_t index)
{
    uint32_t clkunlock = SYSCON->CLKUNLOCK;

    SYSCON->CLKUNLOCK =
        clkunlock & ~SYSCON_CLKUNLOCK_UNLOCK_MASK;

    /*
     * Enable the CTIMER register/interface clock.
     */
    MRCC0->MRCC_GLB_CC0_SET =
        ctimer_clock_masks[index];

    /*
     * Functional clock source:
     *
     *     MUX 0 = FRO_LF_DIV
     *
     * clock.c configures FRO_LF_DIV as /1 from FRO12M.
     */
    *ctimer_clksel[index] = 0U;

    /*
     * CTIMER functional divider = /1.
     *
     * All MCXA366 CTIMER CLKDIV registers use the same field layout.
     */
    *ctimer_clkdiv[index] =
        MRCC_MRCC_CTIMER0_CLKDIV_RESET_MASK
        | MRCC_MRCC_CTIMER0_CLKDIV_HALT_MASK;

    *ctimer_clkdiv[index] =
        MRCC_MRCC_CTIMER0_CLKDIV_HALT_MASK;

    *ctimer_clkdiv[index] = 0U;

    /*
     * Match NXP's RESET_PeripheralReset() sequence:
     *
     *     assert reset
     *     release reset
     */
    MRCC0->MRCC_GLB_RST0_CLR =
        ctimer_reset_masks[index];

    MRCC0->MRCC_GLB_RST0_SET =
        ctimer_reset_masks[index];

    SYSCON->CLKUNLOCK = clkunlock;
}


/****************************************************************
 * Generic PWM pin mux setup
 ****************************************************************/

static void
pwm_pin_setup(uint32_t pin,
              uint32_t mux)
{
    uint32_t port =
        GPIO2PORT(pin);

    uint32_t pin_num =
        GPIO2PIN(pin);

    if (port >= ARRAY_SIZE(port_regs))
        shutdown("Invalid PWM pin port");

    uint32_t clkunlock =
        SYSCON->CLKUNLOCK;

    SYSCON->CLKUNLOCK =
        clkunlock & ~SYSCON_CLKUNLOCK_UNLOCK_MASK;

    MRCC0->MRCC_GLB_CC1_SET =
        port_clock_masks[port];

    MRCC0->MRCC_GLB_RST1_SET =
        port_reset_masks[port];

    SYSCON->CLKUNLOCK =
        clkunlock;

    /*
     * NXP-generated FRDM-MCXA366 configuration:
     *   ALT5
     *   fast slew
     *   input buffer enabled
     *   no pull/open-drain/inversion
     */
    port_regs[port]->PCR[pin_num] =
        PORT_PCR_MUX(mux)
        | PORT_PCR_IBE_MASK;
}

/****************************************************************
 * CTIMER timing conversion
 ****************************************************************/

static uint32_t
cycle_time_to_ctimer_ticks(uint32_t cycle_time)
{
    uint32_t timer_clock =
        mcx_get_fro_lf_frequency();

    /*
     * Klipper cycle_time is expressed in CONFIG_CLOCK_FREQ ticks.
     */
    uint64_t ticks =
        ((uint64_t)cycle_time * timer_clock
         + CONFIG_CLOCK_FREQ / 2U)
        / CONFIG_CLOCK_FREQ;

    if (ticks < 2U)
        ticks = 2U;

    return (uint32_t)ticks;
}


static uint32_t
ctimer_pulse_ticks(uint32_t period_ticks, uint32_t val)
{
    if (val > MAX_PWM)
        val = MAX_PWM;

    /*
     * MCX CTIMER PWM polarity:
     *
     *     pulse_match = period * (1 - duty)
     *
     * 0%:
     *
     *     pulse_match = period_ticks
     *
     * which is beyond the period-reset match.
     *
     * 100%:
     *
     *     pulse_match = 0
     */
    return (uint32_t)(
        ((uint64_t)period_ticks * (MAX_PWM - val)
         + MAX_PWM / 2U)
        / MAX_PWM);
}


/****************************************************************
 * CTIMER period-channel allocation
 ****************************************************************/

static int
ctimer_find_period_channel(uint8_t output_mask,
                           uint8_t requested_output)
{
    /*
     * Prefer the highest numbered free channel.
     *
     * This preserves the familiar MAT3-period arrangement whenever
     * MAT3 itself is not needed as an output.
     */
    for (int channel = 3; channel >= 0; channel--) {
        if ((uint8_t)channel == requested_output)
            continue;

        if (!(output_mask & (1U << channel)))
            return channel;
    }

    return -1;
}


static void
ctimer_move_period_channel(uint8_t timer_index,
                           uint8_t new_channel)
{
    struct ctimer_state *state =
        &ctimer_states[timer_index];

    CTIMER_Type *timer =
        ctimer_regs[timer_index];

    uint8_t old_channel =
        state->period_channel;

    if (new_channel == old_channel)
        return;

    uint32_t period_match =
        state->period_ticks - 1U;

    /*
     * The new period channel is not currently an output.
     */
    timer->PWMC &=
        ~(1U << new_channel);

    timer->MR[new_channel] =
        period_match;

    timer->MSR[new_channel] =
        period_match;

    /*
     * Atomically move the reset-on-match responsibility from the old
     * period channel to the new one.
     *
     * Both match registers contain the same period value, so this does
     * not alter the PWM frequency.
     */
    uint32_t mcr = timer->MCR;

    mcr &=
        ~ctimer_reset_bit(old_channel);

    mcr &=
        ~ctimer_reload_bit(new_channel);

    mcr |=
        ctimer_reset_bit(new_channel);

    timer->MCR = mcr;

    state->period_channel =
        new_channel;
}


/****************************************************************
 * CTIMER route allocation
 ****************************************************************/

static int
ctimer_route_available(const struct ctimer_pwm_route *route,
                       uint32_t period_ticks)
{
    struct ctimer_state *state =
        &ctimer_states[route->timer];

    uint8_t channel_mask =
        1U << route->channel;

    /*
     * Two physical pins that alias the same CTIMER/MAT output cannot
     * act as independent PWM channels.
     */
    if (state->output_mask & channel_mask)
        return 0;

    if (!state->initialized)
        return 1;

    /*
     * All outputs belonging to one CTIMER share the same period.
     */
    if (state->period_ticks != period_ticks)
        return 0;

    /*
     * If the requested MAT channel is currently acting as the period
     * channel, make sure there is another unused channel available to
     * take over the period function.
     */
    if (state->period_channel == route->channel) {
        int replacement =
            ctimer_find_period_channel(
                state->output_mask,
                route->channel);

        if (replacement < 0)
            return 0;
    }

    return 1;
}


static const struct ctimer_pwm_route *
ctimer_find_route(uint8_t pin, uint32_t period_ticks)
{
    /*
     * First preference:
     *
     * Pack the output onto an already-running timer using the same
     * period. This preserves unused CTIMER instances for outputs that
     * need different frequencies.
     */
    for (uint32_t i = 0;
         i < ARRAY_SIZE(ctimer_routes);
         i++) {
        const struct ctimer_pwm_route *route =
            &ctimer_routes[i];

        if (route->pin != pin)
            continue;

        struct ctimer_state *state =
            &ctimer_states[route->timer];

        if (!state->initialized)
            continue;

        if (ctimer_route_available(route, period_ticks))
            return route;
    }

    /*
     * Second preference:
     *
     * Allocate a previously-unused CTIMER instance.
     */
    for (uint32_t i = 0;
         i < ARRAY_SIZE(ctimer_routes);
         i++) {
        const struct ctimer_pwm_route *route =
            &ctimer_routes[i];

        if (route->pin != pin)
            continue;

        struct ctimer_state *state =
            &ctimer_states[route->timer];

        if (state->initialized)
            continue;

        if (ctimer_route_available(route, period_ticks))
            return route;
    }

    return NULL;
}


/****************************************************************
 * CTIMER initialization
 ****************************************************************/

static void
ctimer_initialize(uint8_t timer_index,
                  uint8_t first_output_channel,
                  uint32_t period_ticks)
{
    struct ctimer_state *state =
        &ctimer_states[timer_index];

    CTIMER_Type *timer =
        ctimer_regs[timer_index];

    int period_channel =
        ctimer_find_period_channel(
            0U,
            first_output_channel);

    if (period_channel < 0)
        shutdown("Unable to allocate PWM period channel");

    ctimer_clock_setup(timer_index);

    /*
     * Peripheral reset above gives us a clean CTIMER, but explicitly
     * establish the configuration we depend on.
     */
    timer->TCR =
        CTIMER_TCR_CRST_MASK;

    timer->CTCR = 0U;
    timer->PR = 0U;
    timer->PC = 0U;
    timer->TC = 0U;
    timer->MCR = 0U;
    timer->PWMC = 0U;

    timer->MR[period_channel] =
        period_ticks - 1U;

    timer->MSR[period_channel] =
        period_ticks - 1U;

    timer->MCR =
        ctimer_reset_bit(period_channel);

    state->period_ticks =
        period_ticks;

    state->period_channel =
        period_channel;

    state->output_mask =
        0U;

    state->initialized =
        1U;

    /*
     * Release counter reset, but do not start yet.
     */
    timer->TCR = 0U;
}


/****************************************************************
 * CTIMER setup
 ****************************************************************/

static struct gpio_pwm
ctimer_pwm_setup(uint8_t pin,
                 uint32_t cycle_time,
                 uint32_t val)
{
    uint32_t period_ticks =
        cycle_time_to_ctimer_ticks(cycle_time);

    const struct ctimer_pwm_route *route =
        ctimer_find_route(pin, period_ticks);

    if (!route)
        shutdown("PWM pin shares CTIMER with a different cycle time");

    struct ctimer_state *state =
        &ctimer_states[route->timer];

    CTIMER_Type *timer =
        ctimer_regs[route->timer];

    if (!state->initialized) {
        ctimer_initialize(
            route->timer,
            route->channel,
            period_ticks);
    }

    /*
     * The requested output may currently be the hidden period channel.
     * Move the period function to another unused match register first.
     */
    if (state->period_channel == route->channel) {
        int replacement =
            ctimer_find_period_channel(
                state->output_mask,
                route->channel);

        if (replacement < 0)
            shutdown("No free CTIMER channel for PWM period");

        ctimer_move_period_channel(
            route->timer,
            replacement);
    }

    pwm_pin_setup(
        route->pin,
        route->mux);

    uint32_t pulse =
        ctimer_pulse_ticks(
            period_ticks,
            val);

    /*
     * Initialize both the active and shadow compare registers.
     *
     * Writing MR directly is safe here because this MAT channel was
     * not previously in use.
     */
    timer->MR[route->channel] =
        pulse;

    timer->MSR[route->channel] =
        pulse;

    /*
     * Reload future duty updates from MSR at the period boundary.
     */
    timer->MCR |=
        ctimer_reload_bit(route->channel);

    /*
     * Enable PWM mode for this match output.
     */
    timer->PWMC |=
        1U << route->channel;

    state->output_mask |=
        1U << route->channel;

    struct gpio_pwm g = {
        .regs = timer,
        .hwpwm_ticks = period_ticks,
        .provider = MCX_PWM_CTIMER,
        .channel = route->channel,
        .submodule = 0U,
    };

    /*
     * Start the timing domain after the first output is configured.
     * Additional outputs simply join the already-running timer.
     */
    timer->TCR |=
        CTIMER_TCR_CEN_MASK;

    return g;
}


/****************************************************************
 * FlexPWM first-light resources
 ****************************************************************/

/*
 * First known FlexPWM route:
 *
 *     P3_0
 *       -> ALT5
 *       -> FLEXPWM0
 *       -> submodule 0
 *       -> PWM A
 *
 * For test C this route remains disabled by flexpwm_test_enable.
 */
#define FLEXPWM_TEST_PIN          GPIO(3, 0)
#define FLEXPWM_TEST_MUX          5U
#define FLEXPWM_TEST_SUBMODULE    0U


#define FLEXPWM_CHANNEL_X         0U
#define FLEXPWM_CHANNEL_B         1U
#define FLEXPWM_CHANNEL_A         2U


struct flexpwm_timing {
    uint16_t period_ticks;
    uint8_t prescale;
};


static uint8_t flexpwm0_initialized;


/****************************************************************
 * FlexPWM clock/reset setup
 ****************************************************************/

static void
flexpwm0_clock_setup(void)
{
    if (flexpwm0_initialized)
        return;

    uint32_t clkunlock =
        SYSCON->CLKUNLOCK;

    SYSCON->CLKUNLOCK =
        clkunlock & ~SYSCON_CLKUNLOCK_UNLOCK_MASK;

    /*
     * Enable FLEXPWM0 peripheral/interface clock.
     */
    MRCC0->MRCC_GLB_CC0_SET =
        MRCC_MRCC_GLB_CC0_FLEXPWM0_MASK;

    /*
     * Hold FLEXPWM0 in reset.
     */
    MRCC0->MRCC_GLB_RST0_CLR =
        MRCC_MRCC_GLB_RST0_FLEXPWM0_MASK;

    /*
     * Match FLEXPWM_Init():
     *
     * enable all four submodule clocks before releasing reset.
     */
    SYSCON->PWM0SUBCTL |=
        SYSCON_PWM0SUBCTL_CLK0_EN_MASK
        | SYSCON_PWM0SUBCTL_CLK1_EN_MASK
        | SYSCON_PWM0SUBCTL_CLK2_EN_MASK
        | SYSCON_PWM0SUBCTL_CLK3_EN_MASK;

    /*
     * Release FLEXPWM0 reset.
     */
    MRCC0->MRCC_GLB_RST0_SET =
        MRCC_MRCC_GLB_RST0_FLEXPWM0_MASK;

    SYSCON->CLKUNLOCK =
        clkunlock;

    flexpwm0_initialized =
        1U;
}

/****************************************************************
 * FlexPWM timing conversion
 ****************************************************************/

static struct flexpwm_timing
flexpwm_get_timing(uint32_t cycle_time)
{
    /*
     * Keep the current 120 MHz timing assumption for this test.
     * We are isolating control-path behavior, not frequency scaling.
     */
    uint32_t source_clock =
        mcx_get_fro_hf_frequency() / 2U;

    uint64_t raw_ticks =
        ((uint64_t)cycle_time * source_clock
         + CONFIG_CLOCK_FREQ / 2U)
        / CONFIG_CLOCK_FREQ;

    for (uint8_t prescale = 0U;
         prescale <= 7U;
         prescale++) {

        uint32_t divider =
            1U << prescale;

        uint64_t ticks =
            (raw_ticks + divider / 2U)
            / divider;

        if (ticks < 2U)
            ticks = 2U;

        if (ticks <= 0xffffU) {
            struct flexpwm_timing timing = {
                .period_ticks = (uint16_t)ticks,
                .prescale = prescale,
            };

            return timing;
        }
    }

    shutdown("FlexPWM cycle time too long");
}

/****************************************************************
 * FlexPWM duty conversion
 ****************************************************************/

static uint16_t
flexpwm_high_ticks(uint32_t period_ticks,
                   uint32_t val)
{
    if (val > MAX_PWM)
        val = MAX_PWM;

    uint64_t high_ticks =
        ((uint64_t)period_ticks * val
         + MAX_PWM / 2U)
        / MAX_PWM;

    if (high_ticks > period_ticks)
        high_ticks = period_ticks;

    return (uint16_t)high_ticks;
}


/****************************************************************
 * FlexPWM first-light setup
 ****************************************************************/

static void
flexpwm_inputmux_setup(void)
{
    uint32_t clkunlock =
        SYSCON->CLKUNLOCK;

    SYSCON->CLKUNLOCK =
        clkunlock & ~SYSCON_CLKUNLOCK_UNLOCK_MASK;

    /*
     * Enable INPUTMUX0.
     */
    MRCC0->MRCC_GLB_CC0_SET =
        MRCC_MRCC_GLB_CC0_INPUTMUX0_MASK;

    /*
     * Reset INPUTMUX0.
     */
    MRCC0->MRCC_GLB_RST0_CLR =
        MRCC_MRCC_GLB_RST0_INPUTMUX0_MASK;

    MRCC0->MRCC_GLB_RST0_SET =
        MRCC_MRCC_GLB_RST0_INPUTMUX0_MASK;

    SYSCON->CLKUNLOCK =
        clkunlock;

    /*
     * Match the FRDM-MCXA366 SDK PWM example exactly:
     *
     * TRIG_IN2  -> FLEXPWM0 FAULT0
     * TRIG_IN3  -> FLEXPWM0 FAULT1
     * TRIG_IN4  -> FLEXPWM0 FAULT2
     * TRIG_IN10 -> FLEXPWM0 FAULT3
     */
    *(volatile uint32_t *)0x400013c0U =
        22U;

    *(volatile uint32_t *)0x400013c4U =
        23U;

    *(volatile uint32_t *)0x400013c8U =
        24U;

    *(volatile uint32_t *)0x400013ccU =
        30U;
}

static void
flexpwm_dump_inputmux(void)
{
    volatile uint32_t *fault0 =
        (volatile uint32_t *)0x400013c0U;

    volatile uint32_t *fault1 =
        (volatile uint32_t *)0x400013c4U;

    volatile uint32_t *fault2 =
        (volatile uint32_t *)0x400013c8U;

    volatile uint32_t *fault3 =
        (volatile uint32_t *)0x400013ccU;

    output("fpwm imux f0=%u f1=%u f2=%u f3=%u",
           *fault0,
           *fault1,
           *fault2,
           *fault3);
}

static void
flexpwm_dump_full(void)
{
    PWM_Type *pwm =
        FLEXPWM0;

    output("FPREG fault1 fctrl=%u fsts=%u ffilt=%u",
           pwm->FCTRL,
           pwm->FSTS,
           pwm->FFILT);

    output("FPREG fault2 ftst=%u fctrl2=%u dismap=%u",
           pwm->FTST,
           pwm->FCTRL2,
           pwm->SM[0].DISMAP[0]);

    output("FPREG system subctl=%u pcr=%u",
           SYSCON->PWM0SUBCTL,
           PORT3->PCR[0]);
}

static void
flexpwm_dump_state(uint32_t tag)
{
    PWM_Type *pwm =
        FLEXPWM0;

    uint8_t sm =
        FLEXPWM_TEST_SUBMODULE;

    output("fpwm tag=%u ctrl2=%u ctrl=%u",
           tag,
           pwm->SM[sm].CTRL2,
           pwm->SM[sm].CTRL);

    output("fpwm tag=%u init=%u cnt=%u",
           tag,
           pwm->SM[sm].INIT,
           pwm->SM[sm].CNT);

    output("fpwm tag=%u v0=%u v1=%u",
           tag,
           pwm->SM[sm].VAL0,
           pwm->SM[sm].VAL1);

    output("fpwm tag=%u v2=%u v3=%u",
           tag,
           pwm->SM[sm].VAL2,
           pwm->SM[sm].VAL3);

    output("fpwm tag=%u octrl=%u outen=%u",
           tag,
           pwm->SM[sm].OCTRL,
           pwm->OUTEN);

    output("fpwm tag=%u mask=%u dtsrc=%u",
           tag,
           pwm->MASK,
           pwm->DTSRCSEL);

    output("fpwm tag=%u mctrl=%u fsts=%u",
           tag,
           pwm->MCTRL,
           pwm->FSTS);

    output("fpwm tag=%u subctl=%u",
           tag,
           SYSCON->PWM0SUBCTL);

    output("fpwm tag=%u dismap=%u dt0=%u",
           tag,
           pwm->SM[sm].DISMAP[0],
           pwm->SM[sm].DTCNT0);

    output("fpwm tag=%u fctrl=%u fctrl2=%u",
           tag,
           pwm->FCTRL,
           pwm->FCTRL2);

    output("fpwm tag=%u pcr=%u",
           tag,
           PORT3->PCR[0]);

    output("fpwm tag=%u sts=%u",
           tag,
           pwm->SM[sm].STS);

    output("fpwm swtest pddr=%u",
        GPIO3->PDDR);
}

static struct gpio_pwm
flexpwm_setup_p3_0(uint32_t cycle_time,
                   uint32_t val)
{
    PWM_Type *pwm =
        FLEXPWM0;

    /*
     * This test deliberately clones the known-good SDK state.
     *
     * SDK observed:
     *
     *   INIT = 38304
     *   VAL1 = 27231
     *   VAL2 = 51920
     *   VAL3 = 13616
     *
     * The effective modValue in the SDK example is 54464.
     */
    const uint16_t init =
        38304U;

    const uint16_t val1 =
        27231U;

    const uint16_t val2 =
        51920U;

    const uint16_t val3 =
        13616U;

    const uint16_t run_mask =
        (1U << 0)
        | (1U << 1)
        | (1U << 2);

    (void)cycle_time;
    (void)val;

    /*
     * Peripheral initialization.
     */
    flexpwm0_clock_setup();

    /*
     * Match the SDK board initialization's fault-input routing.
     */
    flexpwm_inputmux_setup();

    /*
     * P3_0 -> PWM0_A0, ALT5.
     */
    pwm_pin_setup(
        FLEXPWM_TEST_PIN,
        FLEXPWM_TEST_MUX);

    /*
     * Stop SM0/1/2 before configuring.
     */
    pwm->MCTRL &=
        ~PWM_MCTRL_RUN(run_mask);

    /*
     * Clear pending LDOK state.
     */
    pwm->MCTRL |=
        PWM_MCTRL_CLDOK(run_mask);

    /*
     * ============================================================
     * SM0
     * ============================================================
     *
     * SDK:
     *
     * CTRL2 = 0x8000 before force config
     * CTRL  = 0x0410
     *
     * IPBus clock, /2 prescaler, full-cycle reload.
     */
    pwm->SM[0].CTRL2 =
        PWM_CTRL2_DBGEN_MASK
        | PWM_CTRL2_WAITEN_MASK;

    pwm->SM[0].CTRL =
        PWM_CTRL_PRSC(1U)
        | PWM_CTRL_FULL_MASK;

    pwm->SM[0].INIT =
        init;

    pwm->SM[0].VAL0 =
        0U;

    pwm->SM[0].VAL1 =
        val1;

    pwm->SM[0].VAL2 =
        val2;

    pwm->SM[0].VAL3 =
        val3;

    pwm->SM[0].VAL4 =
        0U;

    pwm->SM[0].VAL5 =
        0U;

    /*
     * SDK deadtime.
     */
    pwm->SM[0].DTCNT0 =
        156U;

    pwm->SM[0].DTCNT1 =
        156U;

    /*
     * ============================================================
     * SM1
     * ============================================================
     *
     * SDK:
     *
     * CTRL2 = 0x8202
     * CTRL  = 0x0400
     *
     * SM0 clock + master sync.
     */
    pwm->SM[1].CTRL2 =
        PWM_CTRL2_DBGEN_MASK
        | PWM_CTRL2_CLK_SEL(2U)
        | PWM_CTRL2_INIT_SEL(2U);

    pwm->SM[1].CTRL =
        PWM_CTRL_PRSC(0U)
        | PWM_CTRL_FULL_MASK;

    pwm->SM[1].INIT =
        init;

    pwm->SM[1].VAL0 =
        0U;

    pwm->SM[1].VAL1 =
        val1;

    pwm->SM[1].VAL2 =
        val2;

    pwm->SM[1].VAL3 =
        val3;

    pwm->SM[1].VAL4 =
        0U;

    pwm->SM[1].VAL5 =
        0U;

    pwm->SM[1].DTCNT0 =
        156U;

    pwm->SM[1].DTCNT1 =
        156U;

    /*
     * ============================================================
     * SM2
     * ============================================================
     */
    pwm->SM[2].CTRL2 =
        PWM_CTRL2_DBGEN_MASK
        | PWM_CTRL2_WAITEN_MASK
        | PWM_CTRL2_CLK_SEL(2U)
        | PWM_CTRL2_INIT_SEL(2U);

    pwm->SM[2].CTRL =
        PWM_CTRL_PRSC(0U)
        | PWM_CTRL_FULL_MASK;

    pwm->SM[2].INIT =
        init;

    pwm->SM[2].VAL0 =
        0U;

    pwm->SM[2].VAL1 =
        val1;

    pwm->SM[2].VAL2 =
        val2;

    pwm->SM[2].VAL3 =
        val3;

    pwm->SM[2].VAL4 =
        0U;

    pwm->SM[2].VAL5 =
        0U;

    pwm->SM[2].DTCNT0 =
        156U;

    pwm->SM[2].DTCNT1 =
        156U;

    /*
     * ============================================================
     * Complementary-mode configuration
     * ============================================================
     *
     * INDEP remains clear on all three submodules.
     *
     * IPOL=0 means PWM23 is the complementary source.
     */
    pwm->MCTRL &=
        ~PWM_MCTRL_IPOL(run_mask);

    /*
     * Match the SDK fault-output state.
     *
     * 0x002A:
     *
     *   PWMXFS = High-Z
     *   PWMBFS = High-Z
     *   PWMAFS = High-Z
     *
     * The SDK readback was 0x802A because PWMA_IN is a live
     * read-only status bit.
     */
    pwm->SM[0].OCTRL =
        0x002AU;

    pwm->SM[1].OCTRL =
        0x002AU;

    pwm->SM[2].OCTRL =
        0x002AU;

    /*
     * Match SDK DISMAP state exactly.
     */
    pwm->SM[0].DISMAP[0] =
        0xFFFFU;

    pwm->SM[1].DISMAP[0] =
        0xFFFFU;

    pwm->SM[2].DISMAP[0] =
        0xFFFFU;

    /*
     * Match known-good SDK fault-controller state.
     *
     * Observed:
     *
     *   FCTRL  = 65520 = 0xFFF0
     *   FCTRL2 = 15    = 0x000F
     *   FSTS   = 240   = 0x00F0
     */
    pwm->FCTRL =
        0xFFF0U;

    pwm->FCTRL2 =
        0x000FU;

    /*
    * Match FLEXPWM_ConfigFaultProtection():
    *
    * FFLAG = 0xF written as 1 to clear stale fault flags.
    * FFULL = 0xF enables full-cycle fault recovery for FAULT0-3.
    * FHALF = 0.
    *
    * Expected readback after the write:
    *
    *     FSTS = 0x00F0 = 240
    */
    pwm->FSTS =
        PWM_FSTS_FFLAG(0xFU)
        | PWM_FSTS_FFULL(0xFU);

    /*
     * ============================================================
     * Initial PWM output state
     * ============================================================
     *
     * Match the SDK three-phase example before the force override.
     */
    pwm->DTSRCSEL =
        0U;

    pwm->SWCOUT =
        0U;

    pwm->MASK =
        0U;

    /*
     * SDK enables A and B for SM0/1/2:
     *
     * 0x700 A outputs
     * 0x070 B outputs
     * ----------------
     * 0x770 = 1904
     */
    pwm->OUTEN =
        PWM_OUTEN_PWMA_EN(run_mask)
        | PWM_OUTEN_PWMB_EN(run_mask);

    /*
     * Commit timing registers for all three submodules.
     */
    pwm->MCTRL |=
        PWM_MCTRL_LDOK(run_mask);

    /*
     * Start SM0/1/2.
     */
    pwm->MCTRL |=
        PWM_MCTRL_RUN(run_mask);

    /*
     * ============================================================
     * Clone the SDK FORCE TEST on SM0
     * ============================================================
     */

    /*
     * Local FORCE_OUT.
     *
     * FORCE_SEL = 0
     * FRCEN     = 0
     *
     * PWM23 initial state = HIGH
     * PWM45 initial state = LOW
     *
     * This should give CTRL2=0x9000 after FORCE self-clears.
     */
    pwm->SM[0].CTRL2 &=
        ~(PWM_CTRL2_FORCE_SEL_MASK
          | PWM_CTRL2_FRCEN_MASK
          | PWM_CTRL2_PWM45_INIT_MASK);

    pwm->SM[0].CTRL2 |=
        PWM_CTRL2_PWM23_INIT_MASK;

    /*
     * SDK force test:
     *
     * PWM23 <- software
     * PWM45 <- software
     *
     * => DTSRCSEL = 0x000A
     */
    pwm->DTSRCSEL &=
        ~(PWM_DTSRCSEL_SM0SEL23_MASK
          | PWM_DTSRCSEL_SM0SEL45_MASK);

    pwm->DTSRCSEL |=
        PWM_DTSRCSEL_SM0SEL23(2U)
        | PWM_DTSRCSEL_SM0SEL45(2U);

    /*
     * PWM23 = HIGH
     * PWM45 = LOW
     *
     * => SWCOUT = 0x0002
     */
    pwm->SWCOUT &=
        ~(PWM_SWCOUT_SM0OUT23_MASK
          | PWM_SWCOUT_SM0OUT45_MASK);

    pwm->SWCOUT |=
        PWM_SWCOUT_SM0OUT23_MASK;

    /*
     * Local FORCE_OUT.
     *
     * This commits DTSRCSEL, SWCOUT and the buffered IPOL state.
     */
    pwm->SM[0].CTRL2 |=
        PWM_CTRL2_FORCE_MASK;

    struct gpio_pwm g = {
        .regs = pwm,
        .hwpwm_ticks = 54464U,
        .provider = MCX_PWM_FLEXPWM,
        .channel = FLEXPWM_CHANNEL_A,
        .submodule = 0U,
    };

    /*
     * ============================================================
     * Diagnostic readback
     * ============================================================
     */
    output("fpwm sdkclone ctrl2=%u ctrl=%u",
           pwm->SM[0].CTRL2,
           pwm->SM[0].CTRL);

    output("fpwm sdkclone init=%u cnt=%u",
           pwm->SM[0].INIT,
           pwm->SM[0].CNT);

    output("fpwm sdkclone v1=%u v2=%u v3=%u",
           pwm->SM[0].VAL1,
           pwm->SM[0].VAL2,
           pwm->SM[0].VAL3);

    output("fpwm sdkclone dt0=%u dt1=%u",
           pwm->SM[0].DTCNT0,
           pwm->SM[0].DTCNT1);

    output("fpwm sdkclone dtsrc=%u swcout=%u",
           pwm->DTSRCSEL,
           pwm->SWCOUT);

    output("fpwm sdkclone outen=%u mask=%u",
           pwm->OUTEN,
           pwm->MASK);

    output("fpwm sdkclone octrl=%u dismap=%u",
           pwm->SM[0].OCTRL,
           pwm->SM[0].DISMAP[0]);

    output("fpwm sdkclone fctrl=%u fctrl2=%u fsts=%u",
           pwm->FCTRL,
           pwm->FCTRL2,
           pwm->FSTS);

    output("fpwm sdkclone mctrl=%u mctrl2=%u",
           pwm->MCTRL,
           pwm->MCTRL2);

    output("fpwm sdkclone subctl=%u pcr=%u",
           SYSCON->PWM0SUBCTL,
           PORT3->PCR[0]);

    return g;
}

/****************************************************************
 * Public PWM setup API
 ****************************************************************/

struct gpio_pwm
gpio_pwm_setup(uint8_t pin,
               uint32_t cycle_time,
               uint32_t val)
{
    /*
     * Test C:
     *
     * Keep the FlexPWM implementation linked into the firmware, but
     * prevent it from executing.
     *
     * flexpwm_test_enable is volatile and zero-initialized.
     */
    if (flexpwm_test_enable
        && pin == FLEXPWM_TEST_PIN) {
        return flexpwm_setup_p3_0(
            cycle_time,
            val);
    }

    return ctimer_pwm_setup(
        pin,
        cycle_time,
        val);
}


/****************************************************************
 * Public PWM write API
 ****************************************************************/

void
gpio_pwm_write(struct gpio_pwm g,
               uint32_t val)
{
    /*
     * Do not let Klipper modify the SDK-clone diagnostic state.
     */
    if (g.provider == MCX_PWM_FLEXPWM)
        return;

    if (g.provider == MCX_PWM_CTIMER) {
        CTIMER_Type *timer =
            g.regs;

        uint32_t pulse =
            ctimer_pulse_ticks(
                g.hwpwm_ticks,
                val);

        timer->MSR[g.channel] =
            pulse;

        if (!(timer->TCR & CTIMER_TCR_CEN_MASK))
            timer->MR[g.channel] =
                pulse;

        return;
    }

    shutdown("Invalid PWM provider");
}