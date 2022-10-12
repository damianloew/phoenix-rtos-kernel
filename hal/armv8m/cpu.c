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

#include "../cpu.h"
#include "../interrupts.h"
#include "../spinlock.h"
#include "../string.h"

#include "config.h"

struct {
	int busy;
	spinlock_t busySp;
} cpu_common;


volatile cpu_context_t *_cpu_nctx;


/* performance */


/* TODO: add implementation */
void hal_cpuLowPower(time_t us)
{
}


void hal_cpuGetCycles(cycles_t *cb)
{
	/* Cycle counter is not available on armv8m
	assumption that 1 cycle is 1us, so we use hal_timerGetUs() with 1ms resolution
	both cycles_t and time_t have the same size on armv8m */
	*cb = hal_timerGetUs();
}


void hal_cpuSetDevBusy(int s)
{
	spinlock_ctx_t scp;

	hal_spinlockSet(&cpu_common.busySp, &scp);
	if (s == 1)
		++cpu_common.busy;
	else
		--cpu_common.busy;

	if (cpu_common.busy < 0)
		cpu_common.busy = 0;
	hal_spinlockClear(&cpu_common.busySp, &scp);
}


int hal_cpuCreateContext(cpu_context_t **nctx, void *start, void *kstack, size_t kstacksz, void *ustack, void *arg)
{
	cpu_context_t *ctx;

	*nctx = 0;
	if (kstack == NULL)
		return -1;

	if (kstacksz < sizeof(cpu_context_t))
		return -1;

	/* Align user stack to 8 bytes */
	ustack = (void *)((ptr_t)ustack & ~0x7);

	/* Prepare initial kernel stack */
	ctx = (cpu_context_t *)(kstack + kstacksz - sizeof(cpu_context_t));

	hal_memset(ctx, 0, sizeof(*ctx));

	ctx->savesp = (u32)ctx;
	ctx->psp = (ustack != NULL) ? (u32)ustack - (HWCTXSIZE * sizeof(int)) : NULL;
	ctx->r4 = 0x44444444;
	ctx->r5 = 0x55555555;
	ctx->r6 = 0x66666666;
	ctx->r7 = 0x77777777;
	ctx->r8 = 0x88888888;
	ctx->r9 = 0x99999999;
	ctx->r10 = 0xaaaaaaaa;
	ctx->r11 = 0xbbbbbbbb;

	if (ustack != NULL) {
		((u32 *)ctx->psp)[0] = (u32)arg;   /* r0 */
		((u32 *)ctx->psp)[1] = 0x11111111; /* r1 */
		((u32 *)ctx->psp)[2] = 0x22222222; /* r2 */
		((u32 *)ctx->psp)[3] = 0x33333333; /* r3 */
		((u32 *)ctx->psp)[4] = 0xcccccccc; /* r12 */
		((u32 *)ctx->psp)[5] = 0xeeeeeeee; /* lr */
		((u32 *)ctx->psp)[6] = (u32)start; /* pc */
		((u32 *)ctx->psp)[7] = 0x01000000; /* psr */
		ctx->irq_ret = RET_THREAD_PSP;
	}
	else {
		ctx->r0 = (u32)arg;
		ctx->r1 = 0x11111111;
		ctx->r2 = 0x22222222;
		ctx->r3 = 0x33333333;
		ctx->r12 = 0xcccccccc;
		ctx->lr = 0xeeeeeeee;
		ctx->pc = (u32)start;
		ctx->psr = 0x01000000;
		ctx->fpuctx = (u32)(&ctx->psr + 1);
		ctx->irq_ret = RET_THREAD_MSP;
	}

	*nctx = ctx;
	return 0;
}


void hal_longjmp(cpu_context_t *ctx)
{
	__asm__ volatile
	(" \
		cpsid if; \
		str %1, [%0]; \
		bl _hal_invokePendSV; \
		cpsie if; \
	1:	b 1b"
	:
	: "r" (&_cpu_nctx), "r" (ctx)
	: "memory");
}


/* core management */


char *hal_cpuInfo(char *info)
{
	int i;
	unsigned int cpuinfo;

#ifdef CPU_NRF91
	cpuinfo = _nrf91_cpuid();
#else
	hal_strcpy(info, "unknown");
	return info;
#endif

	hal_strcpy(info, HAL_NAME_PLATFORM);
	i = sizeof(HAL_NAME_PLATFORM) - 1;

	if (((cpuinfo >> 24) & 0xff) == 0x41) {
		hal_strcpy(info + i, "ARMv8 ");
		i += 6;
	}

	if (((cpuinfo >> 4) & 0xfff) == 0xc23) {
		hal_strcpy(info + i, "Cortex-M33 ");
		i += 10;
	}

	*(info + i++) = 'r';
	*(info + i++) = '0' + ((cpuinfo >> 20) & 0xf);
	*(info + i++) = ' ';

	*(info + i++) = 'p';
	*(info + i++) = '0' + (cpuinfo & 0xf);
	*(info + i++) = '\0';

	return info;
}


char *hal_cpuFeatures(char *features, unsigned int len)
{
	unsigned int n = 0;
#ifdef CPU_NRF91
	if ((len - n) > 8) {
		hal_strcpy(features + n, "softfp, ");
		n += 8;
	}
#endif
	/* TODO: get region numbers from MPU controller */
	if ((len - n) > 8) {
		hal_strcpy(features + n, "MPU, ");
		n += 5;
	}

	if ((len - n) > 7) {
		hal_strcpy(features + n, "Thumb, ");
		n += 7;
	}

	if (n > 0)
		features[n - 2] = '\0';
	else
		features[0] = '\0';

	return features;
}


/* TODO: add Watchdog implementation */
void hal_wdgReload(void)
{
}


void _hal_cpuInit(void)
{
	cpu_common.busy = 0;
	_cpu_nctx = NULL;

	hal_spinlockCreate(&cpu_common.busySp, "devBusy");

#ifdef CPU_NRF91
	_nrf91_platformInit();
#endif
}
