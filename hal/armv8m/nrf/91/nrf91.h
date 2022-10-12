/*
 * Phoenix-RTOS
 *
 * Operating system kernel
 *
 * nRF91 basic peripherals control functions
 *
 * Copyright 2022 Phoenix Systems
 * Author: Damian Loewnau
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#ifndef _HAL_NRF91_H_
#define _HAL_NRF91_H_

#include <arch/types.h>
#include "../pmap.h"

#include "../../../include/arch/nrf9160.h"


extern void _nrf91_platformInit(void);


extern int hal_platformctl(void *);


extern void _nrf91_platformInit(void);


extern int _nrf91_timerInit(u32 interval);


extern void _nrf91_timerDone(void);


extern void _nrf91_timerClearEvent(void);


extern int _nrf91_systickInit(u32 interval);


extern int _nrf91_gpioConfig(u8 pin, u8 dir, u8 pull);


extern int _nrf91_gpioSet(u8 pin, u8 val);


extern void _nrf91_scbSetPriorityGrouping(u32 group);


extern u32 _nrf91_scbGetPriorityGrouping(void);


extern void _nrf91_scbSetPriority(s8 excpn, u32 priority);


extern u32 _nrf91_scbGetPriority(s8 excpn);


extern void _nrf91_nvicSetIRQ(s8 irqn, u8 state);


extern void _nrf91_nvicSetPriority(s8 irqn, u32 priority);


extern void _nrf91_nvicSystemReset(void);


extern unsigned int _nrf91_cpuid(void);


#endif
