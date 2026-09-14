// Support for extracting the hardware unique id on NXP MCXA366
//
// Copyright (C) 2026
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include <stdint.h>
#include "generic/canserial.h"
#include "sched.h"

#define CHIP_UID_LEN 16
#define MCXA_UUID_ADDR 0x01100800U

void
chipid_init(void)
{
    uint8_t uuid[CHIP_UID_LEN];
    volatile const uint8_t *src =
        (volatile const uint8_t *)MCXA_UUID_ADDR;

    for (uint32_t i = 0; i < CHIP_UID_LEN; i++)
        uuid[i] = src[i];

    canserial_set_uuid(uuid, CHIP_UID_LEN);
}
DECL_INIT(chipid_init);