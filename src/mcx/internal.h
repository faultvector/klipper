#ifndef __MCX_INTERNAL_H
#define __MCX_INTERNAL_H

#include "MCXA366.h"

#define GPIO(PORT, PIN) (((PORT) << 5) | (PIN))
#define GPIO2PORT(PIN)  ((PIN) >> 5)
#define GPIO2PIN(PIN)   ((PIN) & 0x1f)
#define GPIO2BIT(PIN)   (1U << GPIO2PIN(PIN))

#endif // internal.h