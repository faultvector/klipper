// Main starting point for NXP MCX boards
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include "board/armcm_boot.h" // armcm_main
#include "sched.h" // sched_main

void
armcm_main(void)
{
    sched_main();
}