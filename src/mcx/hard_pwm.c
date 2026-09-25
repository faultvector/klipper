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

#define CTIMER_COUNT 5U
#define CTIMER_CHANNEL_COUNT 4U
#define CTIMER_NO_CHANNEL 0xffU


/****************************************************************
 * CTIMER resources
 ****************************************************************/

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
 * A physical pin may have more than one CTIMER route.  The allocator
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
 * Register helpers
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
 * Clock setup
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
 * Pin mux setup
 ****************************************************************/

static void
ctimer_pin_setup(const struct ctimer_pwm_route *route)
{
    uint32_t port = GPIO2PORT(route->pin);
    uint32_t pin = GPIO2PIN(route->pin);

    if (port >= ARRAY_SIZE(port_regs))
        shutdown("Invalid PWM pin port");

    uint32_t clkunlock = SYSCON->CLKUNLOCK;

    SYSCON->CLKUNLOCK =
        clkunlock & ~SYSCON_CLKUNLOCK_UNLOCK_MASK;

    MRCC0->MRCC_GLB_CC1_SET =
        port_clock_masks[port];

    MRCC0->MRCC_GLB_RST1_SET =
        port_reset_masks[port];

    SYSCON->CLKUNLOCK = clkunlock;

    port_regs[port]->PCR[pin] =
        PORT_PCR_MUX(route->mux)
        | PORT_PCR_SRE_MASK;
}


/****************************************************************
 * Timing conversion
 ****************************************************************/

static uint32_t
cycle_time_to_ctimer_ticks(uint32_t cycle_time)
{
    uint32_t timer_clock =
        mcx_get_fro_lf_frequency();

    /*
     *Klipper cycle_time is expressed in CONFIG_CLOCK_FREQ ticks.
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
 * Period-channel allocation
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
 * Route allocation
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
     * pack the output onto an already-running timer using the same
     * period.  This preserves unused CTIMER instances for outputs that
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
     * allocate a previously-unused CTIMER instance.
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
 * Timer initialization
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
 * PWM API
 ****************************************************************/

struct gpio_pwm
gpio_pwm_setup(uint8_t pin,
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

    ctimer_pin_setup(route);

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
        .timer = timer,
        .hwpwm_ticks = period_ticks,
        .channel = route->channel,
    };

    /*
     * Start the timing domain after the first output is configured.
     * Additional outputs simply join the already-running timer.
     */
    timer->TCR |=
        CTIMER_TCR_CEN_MASK;

    return g;
}


void
gpio_pwm_write(struct gpio_pwm g, uint32_t val)
{
    CTIMER_Type *timer =
        g.timer;

    uint32_t pulse =
        ctimer_pulse_ticks(
            g.hwpwm_ticks,
            val);

    /*
     * MRxRL transfers the shadow value into MRx at the next timer
     * reset, giving glitch-free duty changes.
     */
    timer->MSR[g.channel] =
        pulse;

    /*
     * This mainly covers setup/shutdown cases where the timer is not
     * yet running.
     */
    if (!(timer->TCR & CTIMER_TCR_CEN_MASK))
        timer->MR[g.channel] =
            pulse;
}