/*
 * Phoenix-RTOS
 *
 * Operating system kernel
 *
 * System timer driver
 *
 * Copyright 2022 Phoenix Systems
 * Author: Damian Loewnau
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#include "config.h"
#include "../../armv8m.h"
#include "../../../timer.h"
#include "../../../interrupts.h"
#include "../../../spinlock.h"


// enum { lptim_isr = 0, lptim_icr, lptim_ier, lptim_cfgr, lptim_cr, lptim_cmp, lptim_arr, lptim_cnt, lptim_or };


static struct {
	intr_handler_t overflowh;
	spinlock_t sp;
	volatile time_t timeUs;
	volatile u32 *lptim;
	volatile time_t upper;
	volatile int wakeup;
	u32 interval;
} timer_common;


static int timer_irqHandler(unsigned int n, cpu_context_t *ctx, void *arg)
{
	(void)n;
	(void)ctx;
	(void)arg;
	int ret = 0;

	_nrf91_timerClearEvent();
	timer_common.timeUs += timer_common.interval;
	hal_cpuDataSyncBarrier();

	return ret;
}


void hal_timerSetWakeup(u32 when)
{
}


/* Interface functions */


time_t hal_timerGetUs(void)
{
	return timer_common.timeUs;
}


int hal_timerRegister(int (*f)(unsigned int, cpu_context_t *, void *), void *data, intr_handler_t *h)
{
	h->f = f;
	h->n = SYSTICK_IRQ;
	h->data = data;

	return hal_interruptsSetHandler(h);
}


void _hal_timerInit(u32 interval)
{
	timer_common.upper = 0;
	timer_common.wakeup = 0;
	timer_common.timeUs = 0;
	timer_common.interval = interval;

	_nrf91_timerInit(interval);

	hal_spinlockCreate(&timer_common.sp, "timer");

	timer_common.overflowh.f = timer_irqHandler;
	/* irq number always equals nrf peripheral id + 16 */
	timer_common.overflowh.n = timer0 + 16;
	timer_common.overflowh.got = NULL;
	timer_common.overflowh.data = NULL;
	hal_interruptsSetHandler(&timer_common.overflowh);
	_nrf91_systickInit(interval);
}
