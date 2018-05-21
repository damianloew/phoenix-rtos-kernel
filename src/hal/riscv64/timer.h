/*
 * Phoenix-RTOS
 *
 * Operating system kernel
 *
 * System timer (HAL RISCV64)
 *
 * Copyright 2018 Phoenix Systems
 * Author: Pawel Pisarczyk
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#ifndef _HAL_TIMEDEV_H_
#define _HAL_TIMEDEV_H_

#ifndef __ASSEMBLY__

#include "cpu.h"

#define TIMER_US2CYC(x) (x)
#define TIMER_CYC2US(x) (x)


extern int timer_reschedule(unsigned int n, cpu_context_t *ctx, void *arg);


extern void _timer_init(u32 interval);


#endif

#endif
