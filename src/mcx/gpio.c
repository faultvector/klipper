// GPIO functions on MCXA366
//
// Copyright (C) 2026
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include "board/irq.h" // irq_save
#include "command.h" // DECL_ENUMERATION_RANGE
#include "gpio.h" // gpio_out_setup
#include "internal.h" // GPIO
#include "sched.h" // shutdown


DECL_ENUMERATION_RANGE("pin", "P0_0", GPIO(0, 0), 32);
DECL_ENUMERATION_RANGE("pin", "P1_0", GPIO(1, 0), 32);
DECL_ENUMERATION_RANGE("pin", "P2_0", GPIO(2, 0), 32);
DECL_ENUMERATION_RANGE("pin", "P3_0", GPIO(3, 0), 32);
DECL_ENUMERATION_RANGE("pin", "P4_0", GPIO(4, 0), 32);


static GPIO_Type * const gpio_regs[] = {
    GPIO0,
    GPIO1,
    GPIO2,
    GPIO3,
    GPIO4,
};


static PORT_Type * const port_regs[] = {
    PORT0,
    PORT1,
    PORT2,
    PORT3,
    PORT4,
};


static void
enable_port(uint32_t port)
{
    if (port >= ARRAY_SIZE(port_regs))
        shutdown("Not a valid PORT");

    uint32_t bit = 1U << (12U + port);

    SYSCON->CLKUNLOCK &= ~SYSCON_CLKUNLOCK_UNLOCK_MASK;

    volatile uint32_t *cc1_set =
        (volatile uint32_t *)((uint32_t)&MRCC0->MRCC_GLB_CC0_SET + 0x10U);

    *cc1_set = bit;
    MRCC0->MRCC_GLB_RST1_SET = bit;

    SYSCON->CLKUNLOCK |= SYSCON_CLKUNLOCK_UNLOCK_MASK;
}


static void
enable_gpio(uint32_t port)
{
    if (port >= ARRAY_SIZE(gpio_regs))
        shutdown("Not a valid GPIO port");

    uint32_t bit = 1U << (4U + port);

    SYSCON->CLKUNLOCK &= ~SYSCON_CLKUNLOCK_UNLOCK_MASK;

    volatile uint32_t *cc2_set =
        (volatile uint32_t *)((uint32_t)&MRCC0->MRCC_GLB_CC0_SET + 0x20U);

    *cc2_set = bit;
    MRCC0->MRCC_GLB_RST2_SET = bit;

    SYSCON->CLKUNLOCK |= SYSCON_CLKUNLOCK_UNLOCK_MASK;
}


static GPIO_Type *
gpio_pin_to_regs(uint32_t pin)
{
    uint32_t port = GPIO2PORT(pin);

    if (port >= ARRAY_SIZE(gpio_regs))
        shutdown("Not a valid pin");

    return gpio_regs[port];
}


static PORT_Type *
gpio_pin_to_port(uint32_t pin)
{
    uint32_t port = GPIO2PORT(pin);

    if (port >= ARRAY_SIZE(port_regs))
        shutdown("Not a valid pin");

    return port_regs[port];
}


static void
gpio_configure_output(uint32_t pin, uint32_t val)
{
    uint32_t port = GPIO2PORT(pin);
    uint32_t pin_num = GPIO2PIN(pin);
    uint32_t bit = GPIO2BIT(pin);

    GPIO_Type *gpio = gpio_pin_to_regs(pin);
    PORT_Type *port_regs_base = gpio_pin_to_port(pin);

    enable_port(port);
    enable_gpio(port);

    port_regs_base->PCR[pin_num] = PORT_PCR_MUX(0);

    if (val)
        gpio->PSOR = bit;
    else
        gpio->PCOR = bit;

    gpio->PDDR |= bit;
}


struct gpio_out
gpio_out_setup(uint32_t pin, uint32_t val)
{
    GPIO_Type *regs = gpio_pin_to_regs(pin);

    struct gpio_out g = {
        .regs = regs,
        .bit = GPIO2BIT(pin),
    };

    gpio_configure_output(pin, val);

    return g;
}


void
gpio_out_write(struct gpio_out g, uint32_t val)
{
    GPIO_Type *regs = g.regs;

    if (val)
        regs->PSOR = g.bit;
    else
        regs->PCOR = g.bit;
}


void
gpio_out_toggle_noirq(struct gpio_out g)
{
    GPIO_Type *regs = g.regs;

    regs->PTOR = g.bit;
}


void
gpio_out_toggle(struct gpio_out g)
{
    irqstatus_t flag = irq_save();

    gpio_out_toggle_noirq(g);

    irq_restore(flag);
}