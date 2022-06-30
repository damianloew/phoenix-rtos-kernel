/*
 * Phoenix-RTOS
 *
 * Operating system kernel
 *
 * CPU related routines
 *
 * Copyright 2014, 2017, 2022 Phoenix Systems
 * Author: Jacek Popko, Pawel Pisarczyk, Aleksander Kaminski, Damian Loewnau
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#ifndef _HAL_ARMV8M_CPU_H_
#define _HAL_ARMV8M_CPU_H_


#if defined(CPU_NRF9160)
#define CPU_NRF91
#endif

#include "types.h"

/* based on armv7 doc I think it's size of memory block p76, for armv8m I think it should be verified by reading ERG register p1948*/
/* ctr erg for nrf9160 is b0000 so it's also 512 words */
/* endif to remind about checking it for new armv8 targets*/
#if defined CPU_NRF91
#define SIZE_PAGE 0x200
#endif

/*nrf9160 have similar amount of memory to stm so when it's ok for stm it will be also ok for nrf9160 */
#ifndef SIZE_USTACK
#define SIZE_USTACK (3 * SIZE_PAGE)
#endif

#ifndef SIZE_KSTACK
#define SIZE_KSTACK (4 * SIZE_PAGE)
#endif

/* p1518 in armv8?, p515 armv7 to TODO: ask for it! */
#define RET_HANDLER_MSP 0xfffffff1
#define RET_THREAD_MSP  0xfffffff9
#define RET_THREAD_PSP  0xfffffffd
#define HWCTXSIZE       8
#define USERCONTROL     0x3

#ifndef __ASSEMBLY__

/* assuming it's in us, coz ia32 interval min sleep is 10ms */
#define SYSTICK_INTERVAL 1000

/* these defines are the same on armv7a so I assume it's ok */
#define PUTONSTACK(kstack, t, v) \
	do { \
		(kstack) -= (sizeof(t) + 3) & ~0x3; \
		*((t *)kstack) = (v); \
	} while (0)


#define GETFROMSTACK(ustack, t, v, n) \
	do { \
		ustack = (void *)(((ptr_t)ustack + sizeof(t) - 1) & ~(sizeof(t) - 1)); \
		(v) = *(t *)ustack; \
		ustack += (sizeof(t) + 3) & ~0x3; \
	} while (0)

/* TODO: ask for it */
typedef struct _cpu_context_t {
	u32 savesp;
	u32 fpuctx;

	/* Saved by ISR */
	u32 psp;
	u32 r4;
	u32 r5;
	u32 r6;
	u32 r7;
	u32 r8;
	u32 r9;
	u32 r10;
	u32 r11;
	u32 irq_ret;

	/* Saved by hardware */
	u32 r0;
	u32 r1;
	u32 r2;
	u32 r3;
	u32 r12;
	u32 lr;
	u32 pc;
	u32 psr;

} cpu_context_t;


static inline void hal_cpuDisableInterrupts(void)
{
	__asm__ volatile ("cpsid if");
}


/* for cortex a it's aif, I assume that it's ok */
/* based on doc it's ~90% ok */
static inline void hal_cpuEnableInterrupts(void)
{
	__asm__ volatile ("cpsie if");
}

/* TODO: ask */
/* why was it only for non-imxrt?? - for imxrt isn't is used?*/
/* it's called very often - so don't know why is it emoty */
static inline void hal_cpuHalt(void)
{
#ifndef CPU_IMXRT117X
	__asm__ volatile ("\
		wfi; \
		nop; ");
#endif
}


/* bit operations */


/* same for cortex a, assuming it remains same */
static inline unsigned int hal_cpuGetLastBit(unsigned long v)
{
	int pos;

	__asm__ volatile ("clz %0, %1" : "=r" (pos) : "r" (v));

	return 31 - pos;
}


/* same as above */
static inline unsigned int hal_cpuGetFirstBit(unsigned long v)
{
	unsigned pos;

	__asm__ volatile ("\
		rbit %0, %1; \
		clz  %0, %0;" : "=r" (pos) : "r" (v));

	return pos;
}


/* context management */

/* not defined for cortex a - leaving like in armv7m, TODO: ask*/
static inline void hal_cpuSetCtxGot(cpu_context_t *ctx, void *got)
{
	ctx->r9 = (u32)got;
}


/* not set for cortex a, we can put here some value and get it later (?) */
static inline void hal_cpuSetGot(void *got)
{
	__asm__ volatile ("mov r9, %0" :: "r" (got));
}


static inline void *hal_cpuGetGot(void)
{
	void *got;

	__asm__ volatile ("mov %0, r9" : "=r" (got));

	return got;
}


/* save stack pointer? do we switch here to next context? */
static inline void hal_cpuRestore(cpu_context_t *curr, cpu_context_t *next)
{
	curr->savesp = (u32)next;
}


static inline void hal_cpuSetReturnValue(cpu_context_t *ctx, int retval)
{
	ctx->r0 = retval;
}


/* assuming it's ok */
static inline u32 hal_cpuGetPC(void)
{
	void *pc;

	__asm__ volatile ("mov %0, pc" : "=r" (pc));

	return (u32)pc;
}


/* not implemented on armv7m so leaving like this */
static inline void _hal_cpuSetKernelStack(void *kstack)
{
}


/* same for other architectures - leaving */
static inline void *hal_cpuGetSP(cpu_context_t *ctx)
{
	return (void *)ctx;
}


/*for cortex a it's sp, but here I think it's psp */
static inline void *hal_cpuGetUserSP(cpu_context_t *ctx)
{
	return (void *)ctx->psp;
}

/* leaving rest as for armv7m: */
static inline int hal_cpuSupervisorMode(cpu_context_t *ctx)
{
	return 0;
}


static inline int hal_cpuPushSignal(void *kstack, void (*handler)(void), int sig)
{
	return 0;
}


/* core management */


static inline unsigned int hal_cpuGetID(void)
{
	return 0;
}


static inline unsigned int hal_cpuGetCount(void)
{
	return 1;
}


static inline void cpu_sendIPI(unsigned int cpu, unsigned int intr)
{
}


#endif

#endif
