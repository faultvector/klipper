// ADC functions on MCXA366
//
// Copyright (C) 2026
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include "board/misc.h" // timer_from_us
#include "command.h" // DECL_CONSTANT
#include "gpio.h" // gpio_adc_setup
#include "internal.h" // GPIO
#include "sched.h" // shutdown


DECL_CONSTANT("ADC_MAX", 4095);


enum {
    ADC_IDLE = 0xff,
};


struct adc_pin {
    uint32_t pin;
    ADC_Type *regs;
    uint8_t chan;
};


struct adc_status {
    uint8_t chan;
    uint8_t ready;
    uint16_t value;
};


static const struct adc_pin adc_pins[] = {
    { GPIO(1, 10), ADC1, 8 },
    { GPIO(2, 24), ADC2, 3 },
    { GPIO(2, 25), ADC3, 3 },
    { GPIO(2, 10), ADC2, 1 },
    { GPIO(2, 11), ADC3, 1 },
    { GPIO(4, 0), ADC2, 16 },
    { GPIO(4, 1), ADC2, 17 },
    { GPIO(4, 2), ADC2, 18 },
    { GPIO(4, 3), ADC2, 19 },
    { GPIO(4, 4), ADC2, 20 },
    { GPIO(4, 6), ADC2, 22 },
};


static ADC_Type * const adc_regs[] = {
    ADC0,
    ADC1,
    ADC2,
    ADC3,
};


static PORT_Type * const port_regs[] = {
    PORT0,
    PORT1,
    PORT2,
    PORT3,
    PORT4,
};


static struct adc_status adc_status[] = {
    { .chan = ADC_IDLE },
    { .chan = ADC_IDLE },
    { .chan = ADC_IDLE },
    { .chan = ADC_IDLE },
};


static uint8_t adc_initialized;


static uint32_t
adc_get_index(ADC_Type *regs)
{
    for (uint32_t i = 0; i < ARRAY_SIZE(adc_regs); i++) {
        if (adc_regs[i] == regs)
            return i;
    }

    shutdown("Not a valid ADC");

    return 0;
}


static void
configure_adc_clock(void)
{
    SYSCON->CLKUNLOCK &= ~SYSCON_CLKUNLOCK_UNLOCK_MASK;

    // FRO_HF -> ADC.
    MRCC0->MRCC_ADC_CLKSEL = 1U;

    // Match CLOCK_SetClockDiv(kCLOCK_DivADC, 3U).
    MRCC0->MRCC_ADC_CLKDIV =
        MRCC_MRCC_ADC_CLKDIV_RESET_MASK
        | MRCC_MRCC_ADC_CLKDIV_HALT_MASK;

    MRCC0->MRCC_ADC_CLKDIV =
        MRCC_MRCC_ADC_CLKDIV_HALT_MASK
        | MRCC_MRCC_ADC_CLKDIV_DIV(2);

    MRCC0->MRCC_ADC_CLKDIV &=
        ~MRCC_MRCC_ADC_CLKDIV_HALT_MASK;

    SYSCON->CLKUNLOCK |= SYSCON_CLKUNLOCK_UNLOCK_MASK;
}


static void
enable_adc_clock(uint32_t index)
{
    static const uint8_t gate_bits[] = {
        2, 3, 28, 29,
    };

    if (index >= ARRAY_SIZE(gate_bits))
        shutdown("Not a valid ADC");

    uint32_t bit = 1U << gate_bits[index];

    SYSCON->CLKUNLOCK &= ~SYSCON_CLKUNLOCK_UNLOCK_MASK;

    volatile uint32_t *cc1_set =
        (volatile uint32_t *)((uint32_t)&MRCC0->MRCC_GLB_CC0_SET + 0x10U);

    *cc1_set = bit;
    MRCC0->MRCC_GLB_RST1_SET = bit;

    SYSCON->CLKUNLOCK |= SYSCON_CLKUNLOCK_UNLOCK_MASK;
}


static void
adc_pin_setup(uint32_t pin)
{
    uint32_t port = GPIO2PORT(pin);
    uint32_t pnum = GPIO2PIN(pin);

    if (port >= ARRAY_SIZE(port_regs))
        shutdown("Not a valid ADC pin");

    // PORT0..PORT4 are in CC1/RST1 bits 12..16.
    uint32_t bit = 1U << (12U + port);

    SYSCON->CLKUNLOCK &= ~SYSCON_CLKUNLOCK_UNLOCK_MASK;

    volatile uint32_t *cc1_set =
        (volatile uint32_t *)((uint32_t)&MRCC0->MRCC_GLB_CC0_SET + 0x10U);

    *cc1_set = bit;
    MRCC0->MRCC_GLB_RST1_SET = bit;

    SYSCON->CLKUNLOCK |= SYSCON_CLKUNLOCK_UNLOCK_MASK;

    PORT_Type *regs = port_regs[port];

    // Analog inputs use ALT0, no internal pull, and no digital input
    // buffer.
    regs->PCR[pnum] &= ~(PORT_PCR_MUX_MASK
                         | PORT_PCR_PE_MASK
                         | PORT_PCR_IBE_MASK);
}


static void
adc_init(ADC_Type *regs)
{
    uint32_t index = adc_get_index(regs);

    configure_adc_clock();
    enable_adc_clock(index);

    // Disable ADC while configuring it.
    regs->CTRL &= ~ADC_CTRL_ADCEN_MASK;

    // Reset internal ADC logic and configuration.
    regs->CTRL |= ADC_CTRL_RST_MASK;
    regs->CTRL &= ~ADC_CTRL_RST_MASK;

    // Clear the result FIFO.
    regs->CTRL |= ADC_CTRL_RSTFIFO_MASK;

    // Match the FRDM-MCXA366 LPADC example:
    //   REFSEL=2 -> Alt3 -> VDDA
    //   PWRSEL=3 -> highest power setting
    //   PWREN=1  -> analog preliminary enabled
    regs->CFG =
        ADC_CFG_PUDLY(0x80)
        | ADC_CFG_REFSEL(2)
        | ADC_CFG_PWRSEL(3)
        | ADC_CFG_PWREN(1)
        | ADC_CFG_TPRICTRL(0);

    regs->PAUSE = 0U;
    regs->FCTRL = ADC_FCTRL_FWMARK(0);

    // Trigger 0 executes command 1.
    regs->TCTRL[0] = ADC_TCTRL_TCMD(1);

    regs->CTRL |= ADC_CTRL_ADCEN_MASK;

    // TODO: Port the MCXA366 offset and gain calibration sequence.
}


struct gpio_adc
gpio_adc_setup(uint32_t pin)
{
    const struct adc_pin *ap = NULL;

    for (uint32_t i = 0; i < ARRAY_SIZE(adc_pins); i++) {
        if (adc_pins[i].pin == pin) {
            ap = &adc_pins[i];
            break;
        }
    }

    if (!ap)
        shutdown("Not a valid ADC pin");

    adc_pin_setup(pin);

    uint32_t index = adc_get_index(ap->regs);

    if (!(adc_initialized & (1U << index))) {
        adc_init(ap->regs);
        adc_initialized |= 1U << index;
    }

    return (struct gpio_adc) {
        .regs = ap->regs,
        .chan = ap->chan,
    };
}


uint32_t
gpio_adc_sample(struct gpio_adc g)
{
    ADC_Type *regs = g.regs;
    uint32_t index = adc_get_index(regs);
    struct adc_status *status = &adc_status[index];

    if (status->chan == g.chan) {
        if (status->ready)
            return 0;

        uint32_t result = regs->RESFIFO;

        if (!(result & ADC_RESFIFO_VALID_MASK))
            return timer_from_us(10);

        // In standard 12-bit mode the valid conversion bits are
        // RESFIFO.D[14:3].
        status->value =
            (result & ADC_RESFIFO_D_MASK) >> 3;
        status->ready = 1;

        return 0;
    }

    if (status->chan != ADC_IDLE)
        return timer_from_us(10);

    // Command 1: single-ended side A, standard 12-bit conversion.
    regs->CMD[0].CMDL =
        ADC_CMDL_ADCH(g.chan)
        | ADC_CMDL_CTYPE(0)
        | ADC_CMDL_MODE(0);

    // One conversion, no averaging.
    // Use a longer acquisition time for higher-impedance ADC sources.
    // The minimum sample time can cause the sample capacitor to settle low.
    regs->CMD[0].CMDH =
        ADC_CMDH_NEXT(0)
        | ADC_CMDH_LOOP(0)
        | ADC_CMDH_AVGS(0)
        | ADC_CMDH_STS(4)
        | ADC_CMDH_CMPEN(0);

    status->chan = g.chan;
    status->ready = 0;

    // Software trigger 0.
    regs->SWTRIG = 1U;

    return timer_from_us(10);
}


uint16_t
gpio_adc_read(struct gpio_adc g)
{
    ADC_Type *regs = g.regs;
    uint32_t index = adc_get_index(regs);
    struct adc_status *status = &adc_status[index];

    uint16_t value = status->value;

    status->chan = ADC_IDLE;
    status->ready = 0;

    return value;
}


void
gpio_adc_cancel_sample(struct gpio_adc g)
{
    ADC_Type *regs = g.regs;
    uint32_t index = adc_get_index(regs);
    struct adc_status *status = &adc_status[index];

    if (status->chan != g.chan)
        return;

    status->chan = ADC_IDLE;
    status->ready = 0;
}