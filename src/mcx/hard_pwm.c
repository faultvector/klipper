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
    uint32_t clkunlock =
        SYSCON->CLKUNLOCK;

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
    *ctimer_clksel[index] =
        0U;

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

    *ctimer_clkdiv[index] =
        0U;

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

    SYSCON->CLKUNLOCK =
        clkunlock;
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

    if (pin_num >= 32U)
        shutdown("Invalid PWM pin number");

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
     * NXP-generated MCXA366 configuration:
     *
     *     fast slew
     *     input buffer enabled
     *     no pull
     *     no open drain
     *     no inversion
     */
    port_regs[port]->PCR[pin_num] =
        PORT_PCR_MUX(mux)
        | PORT_PCR_IBE_MASK;
}


/****************************************************************
 * CTIMER timing conversion
 ****************************************************************/

static int
cycle_time_to_ctimer_ticks(uint32_t cycle_time,
                           uint32_t *period_ticks)
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

    /*
     * CTIMER needs at least two ticks for a useful PWM period.
     */
    if (ticks < 2U)
        ticks = 2U;

    /*
     * Our state and match calculations represent the period using
     * uint32_t. Do not silently truncate a request that exceeds it.
     */
    if (ticks > UINT32_MAX)
        return 0;

    *period_ticks =
        (uint32_t)ticks;

    return 1;
}


static uint32_t
ctimer_pulse_ticks(uint32_t period_ticks,
                   uint32_t val)
{
    if (val > MAX_PWM)
        val = MAX_PWM;

    /*
     * The period channel is programmed with:
     *
     *     MR = period_ticks - 1
     *
     * Match-channel duty calculations therefore need to use that same
     * counter terminal value.
     */
    uint32_t period_match =
        period_ticks - 1U;

    /*
     * 0% duty:
     *
     * Put the pulse match one count beyond the period match so the
     * output transition never occurs.
     *
     * This matches the MCUX SDK CTIMER PWM implementation.
     */
    if (val == 0U)
        return period_ticks;

    /*
     * 100% duty:
     *
     * Match immediately at zero.
     */
    if (val == MAX_PWM)
        return 0U;

    /*
     * MCX CTIMER PWM polarity:
     *
     *     pulse_match = period_match * (1 - duty)
     */
    return (uint32_t)(
        ((uint64_t)period_match * (MAX_PWM - val)
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
    for (int channel = 3;
         channel >= 0;
         channel--) {
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
    uint32_t mcr =
        timer->MCR;

    mcr &=
        ~ctimer_reset_bit(old_channel);

    mcr &=
        ~ctimer_reload_bit(new_channel);

    mcr |=
        ctimer_reset_bit(new_channel);

    timer->MCR =
        mcr;

    state->period_channel =
        new_channel;
}


/****************************************************************
 * CTIMER route allocation
 ****************************************************************/


static void
ctimer_validate_route(const struct ctimer_pwm_route *route)
{
    if (route->timer >= CTIMER_COUNT)
        shutdown("Invalid CTIMER PWM timer");

    if (route->channel >= 4U)
        shutdown("Invalid CTIMER PWM channel");

    if (GPIO2PORT(route->pin) >= ARRAY_SIZE(port_regs))
        shutdown("Invalid CTIMER PWM pin");
}


static int
ctimer_pin_has_route(uint8_t pin)
{
    for (uint32_t i = 0U;
         i < ARRAY_SIZE(ctimer_routes);
         i++) {
        const struct ctimer_pwm_route *route =
            &ctimer_routes[i];

        ctimer_validate_route(route);

        if (route->pin == pin)
            return 1;
    }

    return 0;
}


static int
ctimer_route_available(const struct ctimer_pwm_route *route,
                       uint32_t period_ticks)
{
    ctimer_validate_route(route);

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
ctimer_find_route(uint8_t pin,
                  uint32_t period_ticks)
{
    /*
     * First preference:
     *
     * Pack the output onto an already-running timer using the same
     * period. This preserves unused CTIMER instances for outputs that
     * need different frequencies.
     */
    for (uint32_t i = 0U;
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
    for (uint32_t i = 0U;
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

    timer->CTCR =
        0U;

    timer->PR =
        0U;

    timer->PC =
        0U;

    timer->TC =
        0U;

    timer->MCR =
        0U;

    timer->PWMC =
        0U;

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
    timer->TCR =
        0U;
}


/****************************************************************
 * CTIMER setup
 ****************************************************************/

static struct gpio_pwm
ctimer_pwm_setup(const struct ctimer_pwm_route *route,
                 uint32_t period_ticks,
                 uint32_t val)
{
    /*
     * The allocator must have confirmed that this route is usable.
     */
    if (!ctimer_route_available(
            route,
            period_ticks))
        shutdown("Invalid CTIMER PWM allocation");

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
     * The requested MAT channel may currently be acting as the hidden
     * period channel. Move that function to another free match channel
     * before enabling this output.
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
     * Initialize both active and shadow compare registers.
     *
     * Direct MR writes are safe here because this MAT output has not
     * previously been allocated.
     */
    timer->MR[route->channel] =
        pulse;

    timer->MSR[route->channel] =
        pulse;

    /*
     * Reload future duty changes from MSR at period boundaries.
     */
    timer->MCR |=
        ctimer_reload_bit(route->channel);

    /*
     * Enable PWM mode for this MAT output.
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
 * FlexPWM resources
 ****************************************************************/

#define FLEXPWM_CHANNEL_X 0U
#define FLEXPWM_CHANNEL_B 1U
#define FLEXPWM_CHANNEL_A 2U

#define FLEXPWM_OUTPUT_X (1U << FLEXPWM_CHANNEL_X)
#define FLEXPWM_OUTPUT_B (1U << FLEXPWM_CHANNEL_B)
#define FLEXPWM_OUTPUT_A (1U << FLEXPWM_CHANNEL_A)


struct flexpwm_pwm_route {
    uint8_t pin;
    uint8_t submodule;
    uint8_t channel;
    uint8_t mux;
};


struct flexpwm_timing {
    uint16_t period_ticks;
    uint8_t prescale;
};

#define FLEXPWM_SUBMODULE_COUNT 4U

struct flexpwm_state {
    uint16_t period_ticks;
    uint8_t prescale;
    uint8_t output_mask;
    uint8_t initialized;
};

static struct flexpwm_state
    flexpwm_states[FLEXPWM_SUBMODULE_COUNT];


static uint8_t
flexpwm_output_bit(uint8_t channel)
{
    switch (channel) {
    case FLEXPWM_CHANNEL_X:
        return FLEXPWM_OUTPUT_X;

    case FLEXPWM_CHANNEL_B:
        return FLEXPWM_OUTPUT_B;

    case FLEXPWM_CHANNEL_A:
        return FLEXPWM_OUTPUT_A;

    default:
        shutdown("Invalid FlexPWM channel");
    }
}


static const struct flexpwm_pwm_route flexpwm_routes[] = {
    {
        .pin = GPIO(3, 0),
        .submodule = 0U,
        .channel = FLEXPWM_CHANNEL_A,
        .mux = 5U,
    },

    {
        .pin = GPIO(4, 7),
        .submodule = 0U,
        .channel = FLEXPWM_CHANNEL_B,
        .mux = 5U,
    },

    {
        .pin = GPIO(4, 4),
        .submodule = 1U,
        .channel = FLEXPWM_CHANNEL_A,
        .mux = 5U,
    },

    {
        .pin = GPIO(4, 5),
        .submodule = 1U,
        .channel = FLEXPWM_CHANNEL_B,
        .mux = 5U,
    },

    {
        .pin = GPIO(4, 2),
        .submodule = 2U,
        .channel = FLEXPWM_CHANNEL_A,
        .mux = 5U,
    },

    {
        .pin = GPIO(4, 3),
        .submodule = 2U,
        .channel = FLEXPWM_CHANNEL_B,
        .mux = 5U,
    },

    {
        .pin = GPIO(4, 0),
        .submodule = 3U,
        .channel = FLEXPWM_CHANNEL_A,
        .mux = 5U,
    },

    {
        .pin = GPIO(4, 1),
        .submodule = 3U,
        .channel = FLEXPWM_CHANNEL_B,
        .mux = 5U,
    },
};


static uint8_t flexpwm0_initialized;


static uint16_t
flexpwm_dtsrcsel_23_mask(uint8_t sm)
{
    switch (sm) {
    case 0U:
        return PWM_DTSRCSEL_SM0SEL23_MASK;

    case 1U:
        return PWM_DTSRCSEL_SM1SEL23_MASK;

    case 2U:
        return PWM_DTSRCSEL_SM2SEL23_MASK;

    case 3U:
        return PWM_DTSRCSEL_SM3SEL23_MASK;

    default:
        shutdown("Invalid FlexPWM submodule");
    }
}


static uint16_t
flexpwm_swcout_23_mask(uint8_t sm)
{
    switch (sm) {
    case 0U:
        return PWM_SWCOUT_SM0OUT23_MASK;

    case 1U:
        return PWM_SWCOUT_SM1OUT23_MASK;

    case 2U:
        return PWM_SWCOUT_SM2OUT23_MASK;

    case 3U:
        return PWM_SWCOUT_SM3OUT23_MASK;

    default:
        shutdown("Invalid FlexPWM submodule");
    }
}


/****************************************************************
 * FlexPWM route lookup
 ****************************************************************/

static void
flexpwm_validate_route(const struct flexpwm_pwm_route *route)
{
    if (route->submodule >= FLEXPWM_SUBMODULE_COUNT)
        shutdown("Invalid FlexPWM submodule");

    if (route->channel != FLEXPWM_CHANNEL_A
        && route->channel != FLEXPWM_CHANNEL_B)
        shutdown("Invalid FlexPWM channel");

    if (GPIO2PORT(route->pin) >= ARRAY_SIZE(port_regs))
        shutdown("Invalid FlexPWM pin");
}


static int
flexpwm_pin_has_route(uint8_t pin)
{
    for (uint32_t i = 0U;
         i < ARRAY_SIZE(flexpwm_routes);
         i++) {
        const struct flexpwm_pwm_route *route =
            &flexpwm_routes[i];

        flexpwm_validate_route(route);

        if (route->pin == pin)
            return 1;
    }

    return 0;
}


static int
flexpwm_route_available(const struct flexpwm_pwm_route *route,
                        struct flexpwm_timing timing)
{
    flexpwm_validate_route(route);

    struct flexpwm_state *state =
        &flexpwm_states[route->submodule];

    uint8_t output_bit =
        flexpwm_output_bit(route->channel);

    /*
     * This FlexPWM output is already allocated.
     */
    if (state->output_mask & output_bit)
        return 0;

    /*
     * An unused submodule can accept any representable timing.
     */
    if (!state->initialized)
        return 1;

    /*
     * A/B outputs on one submodule share the timing domain.
     */
    if (state->period_ticks != timing.period_ticks
        || state->prescale != timing.prescale)
        return 0;

    return 1;
}

static const struct flexpwm_pwm_route *
flexpwm_find_route(uint8_t pin)
{
    for (uint32_t i = 0U;
         i < ARRAY_SIZE(flexpwm_routes);
         i++) {
        if (flexpwm_routes[i].pin == pin)
            return &flexpwm_routes[i];
    }

    return NULL;
}


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
     * Enable all four FlexPWM submodule clocks before releasing
     * peripheral reset.
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

    /*
     * Klipper currently does not use the FlexPWM hardware fault
     * inputs. Establish that policy once for the entire module.
     */
    FLEXPWM0->FCTRL =
        0U;

    FLEXPWM0->FCTRL2 =
        0U;

    /*
     * Clear any fault flags left after reset/setup.
     */
    FLEXPWM0->FSTS =
        PWM_FSTS_FFLAG_MASK;

    flexpwm0_initialized =
        1U;
}

/****************************************************************
 * FlexPWM timing conversion
 ****************************************************************/

static int
flexpwm_get_timing(uint32_t cycle_time,
                   struct flexpwm_timing *timing)
{
    /*
     * FLEXPWM0 IPBus clock runs at the full 240 MHz system clock
     * on the MCXA366.
     */
    uint32_t source_clock =
        mcx_get_fro_hf_frequency();

    uint64_t raw_ticks =
        ((uint64_t)cycle_time * source_clock
         + CONFIG_CLOCK_FREQ / 2U)
        / CONFIG_CLOCK_FREQ;

    /*
     * We use a signed, center-aligned counter:
     *
     *     INIT = -half_period
     *     VAL1 =  half_period - 1
     *
     * Therefore the hardware period is always:
     *
     *     2 * half_period
     *
     * Compute the half-period directly so period_ticks exactly
     * describes the period programmed into hardware.
     */
    for (uint8_t prescale = 0U;
         prescale <= 7U;
         prescale++) {
        uint32_t divider =
            1U << prescale;

        /*
         * Round raw_ticks / (2 * divider) to the nearest integer.
         */
        uint64_t half_ticks =
            (raw_ticks + divider)
            / (2U * divider);

        if (half_ticks < 1U)
            half_ticks = 1U;

        /*
         * period_ticks is uint16_t and must be even.
         *
         * Largest representable even period:
         *
         *     2 * 32767 = 65534
         */
        if (half_ticks <= 0x7fffU) {
            timing->period_ticks =
                (uint16_t)(half_ticks * 2U);

            timing->prescale =
                prescale;

            return 1;
        }
    }

    /*
     * Not representable by FlexPWM.
     *
     * This is not necessarily fatal -- the pin may also have a
     * CTIMER route.
     */
    return 0;
}

/****************************************************************
 * FlexPWM duty conversion
 ****************************************************************/

static void
flexpwm_set_duty(PWM_Type *pwm,
                 uint8_t sm,
                 uint8_t channel,
                 uint16_t period_ticks,
                 uint32_t val)
{
    if (val > MAX_PWM)
        val = MAX_PWM;

    uint16_t half_period =
        period_ticks / 2U;

    uint16_t rising;
    uint16_t falling;

    if (val == 0U) {
        /*
         * Zero-width pulse.
         */
        rising =
            0U;

        falling =
            0U;
    } else if (val == MAX_PWM) {
        /*
         * Cover the entire signed center-aligned counter range.
         *
         * VAL1 is half_period - 1, so falling = half_period never
         * matches and the output remains asserted for the full cycle.
         */
        rising =
            (uint16_t)(0U - half_period);

        falling =
            half_period;
    } else {
        /*
         * Generated A/B pulses are symmetric around zero, so represent
         * duty directly in half-period units.
         */
        uint16_t half_high =
            (uint16_t)(
                ((uint64_t)half_period * val
                 + MAX_PWM / 2U)
                / MAX_PWM);

        rising =
            (uint16_t)(0U - half_high);

        falling =
            half_high;
    }

    switch (channel) {
    case FLEXPWM_CHANNEL_A:
        pwm->SM[sm].VAL2 =
            rising;

        pwm->SM[sm].VAL3 =
            falling;
        break;

    case FLEXPWM_CHANNEL_B:
        pwm->SM[sm].VAL4 =
            rising;

        pwm->SM[sm].VAL5 =
            falling;
        break;

    default:
        shutdown("Unsupported FlexPWM channel");
    }
}

/****************************************************************
 * FlexPWM setup
 ****************************************************************/

static struct gpio_pwm
flexpwm_pwm_setup(const struct flexpwm_pwm_route *route,
                  const struct flexpwm_timing *timing,
                  uint32_t val)
{
    PWM_Type *pwm =
        FLEXPWM0;

    uint8_t sm =
        route->submodule;

    struct flexpwm_state *state =
        &flexpwm_states[sm];

    uint16_t sm_mask =
        1U << sm;

    uint8_t output_bit =
        flexpwm_output_bit(route->channel);

    /*
     * The allocator must have validated this route before calling
     * setup.
     */
    if (!flexpwm_route_available(route, *timing))
        shutdown("Invalid FlexPWM allocation");

    flexpwm0_clock_setup();

    pwm_pin_setup(
        route->pin,
        route->mux);

    /*
     * Initialize the timing domain only once.
     */
    if (!state->initialized) {
        pwm->MCTRL &=
            ~PWM_MCTRL_RUN(sm_mask);

        pwm->MCTRL |=
            PWM_MCTRL_CLDOK(sm_mask);

        uint16_t half_period =
            timing->period_ticks / 2U;

        /*
         * Independent A/B outputs.
         *
         * WAITEN is required so FlexPWM continues operating while
         * Klipper idles the Cortex-M33 in wait mode.
         */
        pwm->SM[sm].CTRL2 =
            PWM_CTRL2_DBGEN_MASK
            | PWM_CTRL2_WAITEN_MASK
            | PWM_CTRL2_INDEP_MASK;

        pwm->SM[sm].CTRL =
            PWM_CTRL_PRSC(timing->prescale)
            | PWM_CTRL_FULL_MASK;

        pwm->SM[sm].INIT =
            (uint16_t)(0U - half_period);

        pwm->SM[sm].VAL0 =
            0U;

        pwm->SM[sm].VAL1 =
            half_period - 1U;

        /*
         * Start both independent outputs at zero duty.
         */
        pwm->SM[sm].VAL2 =
            0U;

        pwm->SM[sm].VAL3 =
            0U;

        pwm->SM[sm].VAL4 =
            0U;

        pwm->SM[sm].VAL5 =
            0U;

        pwm->SM[sm].DTCNT0 =
            0U;

        pwm->SM[sm].DTCNT1 =
            0U;

        /*
         * Active-high A and B.
         */
        pwm->SM[sm].OCTRL &=
            ~(PWM_OCTRL_POLA_MASK
              | PWM_OCTRL_POLB_MASK
              | PWM_OCTRL_PWMAFS_MASK
              | PWM_OCTRL_PWMBFS_MASK);

        /*
         * No hardware fault mapping.
         */
        pwm->SM[sm].DISMAP[0] &=
            ~(PWM_DISMAP_DIS0A_MASK
              | PWM_DISMAP_DIS0B_MASK
              | PWM_DISMAP_DIS0X_MASK);

        /*
         * Normal generated PWM23 source.
         */
        pwm->DTSRCSEL &=
            ~flexpwm_dtsrcsel_23_mask(sm);

        pwm->SWCOUT &=
            ~flexpwm_swcout_23_mask(sm);

        state->period_ticks =
            timing->period_ticks;

        state->prescale =
            timing->prescale;

        state->output_mask =
            0U;

        state->initialized =
            1U;
    }

    /*
     * Program this output's initial duty.
     */
    flexpwm_set_duty(
        pwm,
        sm,
        route->channel,
        state->period_ticks,
        val);

    if (route->channel == FLEXPWM_CHANNEL_A) {
        pwm->MASK &=
            ~PWM_MASK_MASKA(sm_mask);

        pwm->OUTEN |=
            PWM_OUTEN_PWMA_EN(sm_mask);
    } else if (route->channel == FLEXPWM_CHANNEL_B) {
        pwm->MASK &=
            ~PWM_MASK_MASKB(sm_mask);

        pwm->OUTEN |=
            PWM_OUTEN_PWMB_EN(sm_mask);
    } else {
        shutdown("Unsupported FlexPWM channel");
    }

    /*
     * Commit this output's buffered compare registers.
     */
    pwm->MCTRL |=
        PWM_MCTRL_LDOK(sm_mask);

    /*
     * Start or preserve the timing domain.
     */
    pwm->MCTRL |=
        PWM_MCTRL_RUN(sm_mask);

    state->output_mask |=
        output_bit;

    struct gpio_pwm g = {
        .regs = pwm,
        .hwpwm_ticks = state->period_ticks,
        .provider = MCX_PWM_FLEXPWM,
        .channel = route->channel,
        .submodule = sm,
    };

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
    int has_flexpwm_route =
        flexpwm_pin_has_route(pin);

    int has_ctimer_route =
        ctimer_pin_has_route(pin);

    /*
     * Reject pins that have no hardware PWM capability before doing
     * any timing or resource-allocation work.
     */
    if (!has_flexpwm_route
        && !has_ctimer_route)
        shutdown("Pin does not support hardware PWM");

    /*
     * Prefer FlexPWM when this pin has a valid route and the requested
     * timing fits the existing FlexPWM submodule timing domain.
     */
    if (has_flexpwm_route) {
        const struct flexpwm_pwm_route *flex_route =
            flexpwm_find_route(pin);

        struct flexpwm_timing timing;

        if (flexpwm_get_timing(
                cycle_time,
                &timing)
            && flexpwm_route_available(
                flex_route,
                timing)) {
            return flexpwm_pwm_setup(
                flex_route,
                &timing,
                val);
        }
    }

    /*
     * Otherwise try CTIMER.
     */
    if (has_ctimer_route) {
        uint32_t period_ticks;

        if (cycle_time_to_ctimer_ticks(
                cycle_time,
                &period_ticks)) {
            const struct ctimer_pwm_route *ctimer_route =
                ctimer_find_route(
                    pin,
                    period_ticks);

            if (ctimer_route) {
                return ctimer_pwm_setup(
                    ctimer_route,
                    period_ticks,
                    val);
            }
        }
    }

    /*
     * At least one hardware PWM route exists for this pin, but none
     * can satisfy the requested timing/resource combination.
     */
    shutdown("No compatible hardware PWM resource available");
}

/****************************************************************
 * Public PWM write API
 ****************************************************************/

void
gpio_pwm_write(struct gpio_pwm g,
               uint32_t val)
{
    if (val > MAX_PWM)
        val = MAX_PWM;

    if (g.provider == MCX_PWM_FLEXPWM) {
        if (g.regs != FLEXPWM0)
            shutdown("Invalid FlexPWM peripheral");

        if (g.submodule >= FLEXPWM_SUBMODULE_COUNT)
            shutdown("Invalid FlexPWM submodule");

        if (g.channel != FLEXPWM_CHANNEL_A
            && g.channel != FLEXPWM_CHANNEL_B)
            shutdown("Invalid FlexPWM channel");

        if (!g.hwpwm_ticks
            || (g.hwpwm_ticks & 1U))
            shutdown("Invalid FlexPWM period");

        PWM_Type *pwm =
            g.regs;

        uint8_t sm =
            g.submodule;

        flexpwm_set_duty(
            pwm,
            sm,
            g.channel,
            g.hwpwm_ticks,
            val);

        /*
         * Transfer the buffered compare values at the next reload
         * opportunity.
         */
        pwm->MCTRL |=
            PWM_MCTRL_LDOK(1U << sm);

        return;
    }

    if (g.provider == MCX_PWM_CTIMER) {
        if (g.hwpwm_ticks < 2U)
            shutdown("Invalid CTIMER PWM period");

        if (g.channel >= 4U)
            shutdown("Invalid CTIMER PWM channel");

        CTIMER_Type *timer =
            g.regs;

        /*
         * The gpio_pwm handle should only ever contain one of the
         * five MCXA366 CTIMER instances.
         */
        int valid_timer =
            0;

        for (uint32_t i = 0U;
             i < CTIMER_COUNT;
             i++) {
            if (timer == ctimer_regs[i]) {
                valid_timer =
                    1;
                break;
            }
        }

        if (!valid_timer)
            shutdown("Invalid CTIMER PWM peripheral");

        uint32_t pulse =
            ctimer_pulse_ticks(
                g.hwpwm_ticks,
                val);

        /*
         * MSR is the shadow register used for glitch-free PWM updates.
         */
        timer->MSR[g.channel] =
            pulse;

        /*
         * If the timer is stopped, no reload event will occur.
         * Keep MR synchronized so the requested level is present when
         * the timing domain starts again.
         */
        if (!(timer->TCR & CTIMER_TCR_CEN_MASK))
            timer->MR[g.channel] =
                pulse;

        return;
    }

    shutdown("Invalid PWM provider");
}

