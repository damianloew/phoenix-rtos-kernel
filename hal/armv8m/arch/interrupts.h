/*
 * Phoenix-RTOS
 *
 * Operating system kernel
 *
 * Interrupt handling
 *
 * Copyright 2016, 2017, 2020, 2022 Phoenix Systems
 * Author: Pawel Pisarczyk, Artur Wodejko, Hubert Buczynski, Damian Loewnau
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */


#ifndef _HAL_ARMV8M_INTERRUPTS_H_
#define _HAL_ARMV8M_INTERRUPTS_H_

#include "cpu.h"

//here add required irqs here
#define SVC_IRQ     11
#define PENDSV_IRQ  14
//where is info about it in armv8 doc - those are excepction numbers ?? - yes it's the same as exception numbers
#define SYSTICK_IRQ 15

/* here is got, whih isn't present in cortex a handler */
/* it's like additional variable, I think it's ok */
typedef struct _intr_handler_t {
	struct _intr_handler_t *next;
	struct _intr_handler_t *prev;
	/* irq */
	unsigned int n;
	/* handler function */
	int (*f)(unsigned int, cpu_context_t *, void *);
	void *data;
	void *got;
} intr_handler_t;

#endif
