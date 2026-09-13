// Main starting point for NXP MCX boards
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include "board/armcm_boot.h"
#include "internal.h"
#include "sched.h"
#include "system_MCXA366.h"


extern void mcx_clock_init(void);


__attribute__((naked, used, externally_visible))
void
mcx_reset_entry(void)
{
    asm volatile(
        "ldr r0, =VectorTable\n"
        "ldr r1, [r0, #0]\n"
        "msr msp, r1\n"
        "ldr r1, [r0, #4]\n"
        "bx r1\n"
    );
}


void
armcm_preinit(void)
{
    SystemInit();

    /*
     * The MCXA366 boot/debug path may leave VTOR pointing at a
     * temporary SRAM vector table. Restore Klipper's flash vectors
     * before interrupts are enabled.
     */
    SCB->VTOR = (uint32_t)VectorTable;
    __DSB();
    __ISB();

    mcx_clock_init();
}


void
armcm_main(void)
{
    sched_main();
}