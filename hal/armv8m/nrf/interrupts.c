/*
 * Phoenix-RTOS
 *
 * Operating system kernel
 *
 * Interrupt handling
 *
 * Copyright 2017, 2020, 2022 Phoenix Systems
 * Author: Pawel Pisarczyk, Hubert Buczynski, Damian Loewnau
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#include "../../interrupts.h"
#include "../../spinlock.h"
#include "../../cpu.h"
#include "config.h"

#include "../../../proc/userintr.h"


/* Value based on CPU_STM32L4X6, TODO: verify the corectness */
#ifdef CPU_NRF91
#define SIZE_INTERRUPTS     97
#endif

/* Value based on other target architectures */
#define SIZE_HANDLERS       4


#define _intr_add(list, t) \
	do { \
		if (t == NULL) \
			break; \
		if (*list == NULL) { \
			t->next = t; \
			t->prev = t; \
			(*list) = t; \
			break; \
		} \
		t->prev = (*list)->prev; \
		(*list)->prev->next = t; \
		t->next = (*list); \
		(*list)->prev = t; \
	} while (0)


#define _intr_remove(list, t) \
	do { \
		if (t == NULL) \
			break; \
		if ((t->next == t) && (t->prev == t)) \
			(*list) = NULL; \
		else { \
			t->prev->next = t->next; \
			t->next->prev = t->prev; \
			if (t == (*list)) \
				(*list) = t->next; \
		} \
		t->next = NULL; \
		t->prev = NULL; \
	} while (0)


struct {
	spinlock_t spinlock;
	intr_handler_t *handlers[SIZE_INTERRUPTS];
	unsigned int counters[SIZE_INTERRUPTS];
} interrupts;


extern int threads_schedule(unsigned int n, cpu_context_t *context, void *arg);


void interrupts_dispatch(unsigned int n, cpu_context_t *ctx)
{
	intr_handler_t *h;
	int reschedule = 0;
	spinlock_ctx_t sc;

	if (n >= SIZE_INTERRUPTS)
		return;

	hal_spinlockSet(&interrupts.spinlock, &sc);

	interrupts.counters[n]++;

	if ((h = interrupts.handlers[n]) != NULL) {
		do {
			hal_cpuSetGot(h->got);
			if (h->f(n, ctx, h->data))
				reschedule = 1;
		} while ((h = h->next) != interrupts.handlers[n]);
	}

	hal_spinlockClear(&interrupts.spinlock, &sc);

	if (reschedule)
		threads_schedule(n, ctx, NULL);
}


int hal_interruptsSetHandler(intr_handler_t *h)
{
	spinlock_ctx_t sc;

	if (h == NULL || h->f == NULL || h->n >= SIZE_INTERRUPTS)
		return -1;

	/* what is going on with this got?? */
	hal_spinlockSet(&interrupts.spinlock, &sc);
	h->got = hal_cpuGetGot();

	/* adding to interrupt handlers tree */
	_intr_add(&interrupts.handlers[h->n], h);

	if (h->n >= 0x10) {
		_nrf91_nvicSetPriority(h->n - 0x10, 1);
		_nrf91_nvicSetIRQ(h->n - 0x10, 1);
	}
	hal_spinlockClear(&interrupts.spinlock, &sc);

	return 0;
}


int hal_interruptsDeleteHandler(intr_handler_t *h)
{
	spinlock_ctx_t sc;

	if (h == NULL || h->f == NULL || h->n >= SIZE_INTERRUPTS)
		return -1;

	hal_spinlockSet(&interrupts.spinlock, &sc);
	_intr_remove(&interrupts.handlers[h->n], h);

	if (h->n >= 0x10 && interrupts.handlers[h->n] == NULL)
		_nrf91_nvicSetIRQ(h->n - 0x10, 0);

	hal_spinlockClear(&interrupts.spinlock, &sc);

	return 0;
}


char *hal_interruptsFeatures(char *features, unsigned int len)
{
	hal_strncpy(features, "Using NVIC interrupt controller", len);
	features[len - 1] = 0;

	return features;
}


__attribute__ ((section (".init"))) void _hal_interruptsInit(void)
{
	unsigned int n;

	for (n = 0; n < SIZE_INTERRUPTS; ++n) {
		interrupts.handlers[n] = NULL;
		interrupts.counters[n] = 0;
	}

	hal_spinlockCreate(&interrupts.spinlock, "interrupts.spinlock");

	_nrf91_scbSetPriority(SYSTICK_IRQ, 1);
	_nrf91_scbSetPriority(PENDSV_IRQ, 1);
	_nrf91_scbSetPriority(SVC_IRQ, 0);

	/* Set no subprorities in Interrupt Group Priority */
	_nrf91_scbSetPriorityGrouping(3);

	return;
}
