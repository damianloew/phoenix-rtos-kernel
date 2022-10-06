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

#include "nrf91.h"

#include "../../../cpu.h"
#include "../../armv8m.h"
#include "../../../../include/errno.h"


static struct {
	volatile u32 *scb;
	volatile u32 *nvic;
	volatile u32 *power;
	volatile u32 *clock;
	volatile u32 *gpio;
	volatile u32 *rtc[2];
	volatile u32 *timer[3];
	u32 cpuclk;
	spinlock_t pltctlSp;
} nrf91_common;


enum { power_tasks_constlat = 30, power_tasks_lowpwr, power_inten = 192, power_intenset, power_intenclr, power_status = 272};


enum { clock_tasks_hfclkstart, clock_inten = 192, clock_intenset, clock_intenclr, clock_hfclkrun = 258, clock_hfclkstat };


enum { gpio_out = 1, gpio_outset, gpio_outclr, gpio_in, gpio_dir, gpio_dirsetout, gpio_dirsetin, gpio_cnf = 128 };


enum { rtc_tasks_start = 0, rtc_tasks_stop, rtc_tasks_clear, rtc_events_tick = 64, 
	rtc_events_compare0 = 80, rtc_events_compare1, rtc_events_compare2, rtc_events_compare3, rtc_intenset = 193, rtc_intenclr,
	rtc_evten = 208, rtc_evtenset, rtc_evtenclr, rtc_prescaler = 322, rtc_cc0 = 336, rtc_cc1, rtc_cc2, rtc_cc3};


enum { timer_tasks_start = 0, timer_tasks_stop, timer_tasks_count, timer_tasks_clear, timer_tasks_shutdown,
	timer_tasks_capture0 = 16, timer_tasks_capture1, timer_tasks_capture2, timer_tasks_capture3, timer_tasks_capture4, timer_tasks_capture5,
	timer_events_compare0 = 80, timer_events_compare1, timer_events_compare2, timer_events_compare3, timer_events_compare4, timer_events_compare5,
	timer_intenset = 193, timer_intenclr, timer_mode = 321, timer_bitmode, timer_prescaler = 324,
	timer_cc0 = 336, timer_cc1, timer_cc2, timer_cc3, timer_cc4, timer_cc5 };


enum { scb_actlr = 2, scb_cpuid = 832, scb_icsr, scb_vtor, scb_aircr, scb_scr, scb_ccr, scb_shp1, scb_shp2,
	scb_shp3, scb_shcsr, scb_cfsr, scb_mmsr, scb_bfsr, scb_ufsr, scb_hfsr, scb_mmar, scb_bfar, scb_afsr };


enum { syst_csr = 4, syst_rvr, syst_cvr, syst_calib };


enum { fpu_cpacr = 34, fpu_fpccr = 141, fpu_fpcar, fpu_fpdscr };


/* platformctl syscall */


/* TODO: add platformctl implementation */
int hal_platformctl(void *ptr)
{
	return 0;
}


void _nrf91_platformInit(void)
{
	hal_spinlockCreate(&nrf91_common.pltctlSp, "pltctl");
}


//********************TO REMOVE**********************************
/* RTC */


/* TODO: remove it and alternatively place rtc driver in devices */
int _nrf91_rtcInit(u32 interval)
{
	/* 1 tick per 1.007 ms - in theory, in fact it's much different */
	*(nrf91_common.rtc[0] + rtc_prescaler) = 0u;
	// *(nrf91_common.rtc[0] + rtc_prescaler) = 32999u; //temp - tick every 1s
	*(nrf91_common.rtc[0] + rtc_cc0) = 33u;
	/* Enable triggering compare events */
	*(nrf91_common.rtc[0] + rtc_evtenset) = 0x10000;
	/* Enable interrupts from compare events */
	*(nrf91_common.rtc[0] + rtc_intenset) = 0x10000;

	/* Clear and start RTC */
	*(nrf91_common.rtc[0] + rtc_tasks_clear) = 1u;
	*(nrf91_common.rtc[0] + rtc_tasks_start) = 1u;
	return 0;
}


void _nrf91_rtcDone(void)
{
	/* Stop RTC */
	*(nrf91_common.rtc[0] + rtc_tasks_start) = 1u;
}


void _nrf91_rtcClearEvent(void)
{
	/* Clear compare event */
	*(nrf91_common.rtc[0] + rtc_events_compare0) = 0u;
	/* Clear counter */
	*(nrf91_common.rtc[0] + rtc_tasks_clear) = 1u;
}
//********************TO REMOVE**********************************


/* TIMER */


int _nrf91_timerInit(u32 interval)
{
	/* Set timer mode */
	*(nrf91_common.timer[0] + timer_mode) = 0u;
	/* Set 16-bit mode */
	*(nrf91_common.timer[0] + timer_bitmode) = 0u;
	/* 1 tick per 1 us */
	*(nrf91_common.timer[0] + timer_prescaler) = 4u;
	/* 1 compare event per interval * 1us */
	*(nrf91_common.timer[0] + timer_cc0) = interval;
	/* Enable interrupts from compare0 events */
	*(nrf91_common.timer[0] + rtc_intenset) = 0x10000;

	/* Clear and start timer0 */
	*(nrf91_common.timer[0] + timer_tasks_clear) = 1u;
	*(nrf91_common.timer[0] + timer_tasks_start) = 1u;
	return 0;
}


void _nrf91_timerDone(void)
{
	/* Stop timer */
	*(nrf91_common.timer[0] + timer_tasks_stop) = 1u;
}


void _nrf91_timerClearEvent(void)
{
	/* Clear compare event */
	*(nrf91_common.timer[0] + timer_events_compare0) = 0u;
	/* Clear counter */
	*(nrf91_common.timer[0] + timer_tasks_clear) = 1u;
}


/* SysTick */


int _nrf91_systickInit(u32 interval)
{
	u64 load = ((u64) interval * nrf91_common.cpuclk) / 1000000;
	if (load > 0x00ffffff)
		return -EINVAL;

	*(nrf91_common.scb + syst_rvr) = (u32)load;
	*(nrf91_common.scb + syst_cvr) = 0;

	/* Enable systick */
	*(nrf91_common.scb + syst_csr) |= 0x7;

	return EOK;
}


/* GPIO */


int _nrf91_gpioConfig(u8 pin, u8 dir, u8 pull)
{
	if (pin > 31)
		return -1;

	if (dir == output) {
		*(nrf91_common.gpio + gpio_dirsetout) = (1u << pin);
		while ( (*(nrf91_common.gpio + gpio_dir) & (1u << pin)) != (1u << pin) ) ;
	}
	else if (dir == input) {
		*(nrf91_common.gpio + gpio_dirsetin) = (1u << pin);
		while ( (*(nrf91_common.gpio + gpio_dir) | ~(1u << pin)) != ~(1u << pin) ) ;
		/* connect input buffer */
		*(nrf91_common.gpio + gpio_cnf + pin) &= ~0x2;
	}
	
	if (pull) {
		*(nrf91_common.gpio + gpio_cnf + pin) = (pull << 2);
	}

	return 0;
}


int _nrf91_gpioSet(u8 pin, u8 val)
{
	if (pin > 31)
		return -1;

	if (val == high) {
		*(nrf91_common.gpio + gpio_outset) = (1u << pin);
		while ( (*(nrf91_common.gpio + gpio_out) & (1u << pin)) != (1u << pin) ) ;
	}
	else if (val == low) {
		*(nrf91_common.gpio + gpio_outclr) = (1u << pin);
		while ( (*(nrf91_common.gpio + gpio_out) | ~(1u << pin)) != ~(1u << pin) ) ;
	}

	return 0;
}


/* SCB */


void _nrf91_scbSetPriorityGrouping(u32 group)
{
	u32 t;

	/* Get register value and clear bits to set */
	t = *(nrf91_common.scb + scb_aircr) & ~0xffff0700;

	/* Set AIRCR.PRIGROUP to 3: 16 priority groups and 16 subgroups
	/* The value is same as for armv7m4-stm32l4x6 target
	/* Setting various priorities is not supported on Phoenix-RTOS, so it's just default value */
	*(nrf91_common.scb + scb_aircr) = t | 0x5fa0000 | ((group & 7) << 8);
}


u32 _nrf91_scbGetPriorityGrouping(void)
{
	return (*(nrf91_common.scb + scb_aircr) & 0x700) >> 8;
}


void _nrf91_scbSetPriority(s8 excpn, u32 priority)
{
	volatile u8 *ptr;

	ptr = &((u8*)(nrf91_common.scb + scb_shp1))[excpn - 4];

	/* We set only group priority field */
	*ptr = (priority << 4) & 0xff;
}


u32 _nrf91_scbGetPriority(s8 excpn)
{
	volatile u8 *ptr;

	ptr = &((u8*)(nrf91_common.scb + scb_shp1))[excpn - 4];

	return *ptr >> 4;
}


/* NVIC */


void _nrf91_nvicSetIRQ(s8 irqn, u8 state)
{
	volatile u32 *ptr = nrf91_common.nvic + ((u8)irqn >> 5) + (state ? nvic_iser : nvic_icer);
	*ptr = 1 << (irqn & 0x1F);

	hal_cpuDataSyncBarrier();
	hal_cpuInstrBarrier();
}


void _nrf91_nvicSetPriority(s8 irqn, u32 priority)
{
	volatile u32 *ptr;

	ptr = ((u32 *)(nrf91_common.nvic + nvic_ip)) + (irqn / 4);

	/* We set only group priority field */
	*ptr = (priority << (8 * (irqn % 4) + 4));
}


void _nrf91_nvicSystemReset(void)
{
	*(nrf91_common.scb + scb_aircr) = ((0x5fa << 16) | (*(nrf91_common.scb + scb_aircr) & (0x700)) | (1 << 0x02));

	__asm__ volatile ("dsb");

	for(;;);
}


/* CPU info */


unsigned int _nrf91_cpuid(void)
{
	return *(nrf91_common.scb + scb_cpuid);
}


void _nrf91_init(void)
{
	nrf91_common.scb = (void *)0xe000e000;
	nrf91_common.nvic = (void *)0xe000e100;
	nrf91_common.power = (void *)0x50005000;
	nrf91_common.clock = (void *)0x50005000;
	nrf91_common.gpio = (void *)0x50842500;
	/* TODO: remove it and alternatively place rtc driver in devices */
	nrf91_common.rtc[0] = (void *)0x50014000;
	nrf91_common.rtc[1] = (void *)0x50015000;
	/* */
	nrf91_common.timer[0] = (void *)0x5000F000;
	nrf91_common.timer[1] = (void *)0x50010000;
	nrf91_common.timer[2] = (void *)0x50011000;

	/* Based on nRF9160 product specification there is fixed cpu frequency */
	nrf91_common.cpuclk = 64 * 1000 * 1000;

	/* Enable low power mode - TODO: test how Phoenix-RTOS works in this mode */
	// *(nrf91_common.power + power_tasks_lowpwr) = 1u;
	/* Enable constant latency mode to disable automatic power management */
	*(nrf91_common.power + power_tasks_constlat) = 1u;
	hal_cpuDataMemoryBarrier();

	/* Disable all power interrupts */
	*(nrf91_common.power + power_intenclr) = 0x64;

	/* Disable all clock interrupts */
	*(nrf91_common.power + power_intenclr) = 0x3;

	/* Wait until the interrupts above will be cleared */
	while ( *(nrf91_common.power + power_inten) | 0x98 != 0x98 )
		;

	*(nrf91_common.clock + clock_tasks_hfclkstart) = 1u;
	/* Wait until HXFO start and clear event flag */
	while ( *(nrf91_common.clock + clock_hfclkrun) != 1u )
		;
	*(nrf91_common.clock + clock_hfclkrun) = 0u;
	hal_cpuDataMemoryBarrier();

	/* Enable UsageFault, BusFault and MemManage exceptions */
	*(nrf91_common.scb + scb_shcsr) |= (1 << 16) | (1 << 17) | (1 << 18);

	/* Disable FPU */
	*(nrf91_common.scb + fpu_cpacr) = 0;
	*(nrf91_common.scb + fpu_fpccr) = 0;
}
