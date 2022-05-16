/*
 * Phoenix-RTOS
 *
 * Operating system kernel
 *
 * HAL console (STM32L4 USART)
 *
 * Copyright 2016-2017, 2019-2020 Phoenix Systems
 * Author: Pawel Pisarczyk, Artur Wodejko, Aleksander Kaminski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#include "nrf91.h"

#include "../../../console.h"
#include "../../../cpu.h"
#include "../../armv8m.h"


struct {
	volatile u32 *base;
	unsigned cpufreq;
} console_common;


enum { cr1 = 0, cr2, cr3, brr, gtpr, rtor, rqr, isr, icr, rdr, tdr };


static struct {
	volatile u32 *base;
	unsigned cpufreq;
} halconsole_common;


// enum { cr1 = 0, cr2, cr3, brr, gtpr, rtor, rqr, isr, icr, rdr, tdr };
enum { uarte_startrx = 0, uarte_stoprx, uarte_starttx, uarte_stoptx, 
uarte_events_cts = 64, uarte_events_txdrdy = 71, uarte_events_endtx, uarte_events_error, uarte_events_txstarted = 84, 
uarte_inten = 192, uarte_errorsrc = 288, uarte_intenset, uarte_intenclr, uarte_enable = 320, 
uarte_psel_rts = 322, uarte_psel_txd, uarte_psel_cts, uarte_psel_rxd, uarte_baudrate = 329, 
uarte_rxd_ptr = 333, uarte_rxd_maxcnt, uarte_rxd_amount, uarte_txd_ptr = 337, uarte_txd_maxcnt, uarte_txd_amount, 
uarte_config = 347 };

enum { baud_9600 = 0x00275000, baud_115200 = 0x01D60000 };


void hal_consolePrint(int attr, const char *s)
{
	char *ram0 = (char *)0x20000000;
	int i;

	for (i = 0; *s != '\0'; i++) {
		ram0[i] = *s++;
	}

	*(halconsole_common.base + uarte_txd_ptr) = (volatile u32 *)ram0;
	*(halconsole_common.base + uarte_txd_maxcnt) = (u32)i;
	*(halconsole_common.base + uarte_starttx) = 1u;
	while ( *(halconsole_common.base + uarte_events_txstarted) != 1u )
		;
	*(halconsole_common.base + uarte_events_txstarted) = 0u;

	while ( *(halconsole_common.base + uarte_events_endtx) != 1u )
		;
	*(halconsole_common.base + uarte_events_endtx) = 0u;
}


void console_init(void)
{
	struct {
		void *base;
	} uarts[] = {
		{ (void *)UART0_BASE },
		{ (void *)UART1_BASE },
		{ (void *)UART2_BASE },
		{ (void *)UART3_BASE }
	};

	/* default uart instance for nrf9160 dk, connected to VCOM0 */
	const u8 uart = 0, txpin = 29, rxpin = 28, rtspin = 27, ctspin = 26, led1pin = 2, led2pin = 3; 
	unsigned int reg = 6, errsrc = 0;

	halconsole_common.base = uarts[uart].base;

	/* Init pins according to nrf9160 product specification */
	_nrf91_gpioConfig(txpin, output, nopull);
	_nrf91_gpioConfig(rxpin, input, nopull);
	_nrf91_gpioConfig(rtspin, output, nopull);
	_nrf91_gpioConfig(ctspin, input, pulldown);

	_nrf91_gpioSet(txpin, high);
	_nrf91_gpioSet(rtspin, high);

	/* Select pins */
	*(halconsole_common.base + uarte_psel_txd) = txpin;
	*(halconsole_common.base + uarte_psel_rxd) = rxpin;
	*(halconsole_common.base + uarte_psel_rts) = rtspin;
	*(halconsole_common.base + uarte_psel_cts) = ctspin;

	/* Set baud rate to 9600, TODO: verify why uart with 115200 br doesn't work properly */
	*(halconsole_common.base + uarte_baudrate) = baud_9600;

	/* Default settings - hardware flow control disabled, exclude parity bit, one stop bit */
	*(halconsole_common.base + uarte_config) = 0u;

	/* Set default max number of bytes in specific buffers to 4095 */
	*(halconsole_common.base + uarte_txd_maxcnt) = 0xFFF;
	*(halconsole_common.base + uarte_rxd_maxcnt) = 0xFFF;

	/* Set default uart sources: ram0 and ram1 start addresses */
	*(halconsole_common.base + uarte_txd_ptr) = 0x20000000;
	*(halconsole_common.base + uarte_txd_ptr) = 0x20008000;

	/* disable all uart interrupts */
	*(halconsole_common.base + uarte_intenclr) = 0xFFFFFFFF;

	*(halconsole_common.base + uarte_enable) = 0x8;

	/* Wait for cts activation - assuming that it should be active all the time */
	while ( *(halconsole_common.base + uarte_events_cts) != 1u )
		;
	*(halconsole_common.base + uarte_events_cts) = 0u;
}



// void _hal_consolePrint(const char *s)
// {
// 	while (*s)
// 		hal_consolePutch(*(s++));

// 	while (~(*(console_common.base + isr)) & 0x80)
// 		;

// 	return;
// }


// void hal_consolePrint(int attr, const char *s)
// {
// 	if (attr == ATTR_BOLD)
// 		_hal_consolePrint(CONSOLE_BOLD);
// 	else if (attr != ATTR_USER)
// 		_hal_consolePrint(CONSOLE_CYAN);

// 	_hal_consolePrint(s);
// 	_hal_consolePrint(CONSOLE_NORMAL);
// }


void hal_consolePutch(char c)
{
	// while (~(*(console_common.base + isr)) & 0x80)
	// 	;

	// *(console_common.base + tdr) = c;
}


// void _hal_consoleInit(void)
// {
// 	struct {
// 		void *base;
// 		u8 uart;
// 	} uarts[] = {
// 		{ (void *)0x40013800, pctl_usart1 }, /* USART1 */
// 		{ (void *)0x40004400, pctl_usart2 }, /* USART2 */
// 		{ (void *)0x40004800, pctl_usart3 }, /* USART3 */
// 		{ (void *)0x40004c00, pctl_uart4 }, /* UART4 */
// 		{ (void *)0x40005000, pctl_uart5 }  /* UART5 */
// 	};

// 	const int uart = 1, port = pctl_gpiod, txpin = 5, rxpin = 6, af = 7;

// 	_stm32_rccSetDevClock(port, 1);

// 	console_common.base = uarts[uart].base;

// 	/* Init tx pin - output, push-pull, high speed, no pull-up */
// 	_stm32_gpioConfig(port, txpin, 2, af, 0, 2, 0);

// 	/* Init rxd pin - input, push-pull, high speed, no pull-up */
// 	_stm32_gpioConfig(port, rxpin, 2, af, 0, 2, 0);

// 	/* Enable uart clock */
// 	_stm32_rccSetDevClock(uarts[uart].uart, 1);

// 	console_common.cpufreq = _stm32_rccGetCPUClock();

// 	/* Set up UART to 9600,8,n,1 16-bit oversampling */
// 	*(console_common.base + cr1) &= ~1;   /* disable USART */
// 	hal_cpuDataMemoryBarrier();
// 	*(console_common.base + cr1) = 0xa;
// 	*(console_common.base + cr2) = 0;
// 	*(console_common.base + cr3) = 0;
// 	*(console_common.base + brr) = console_common.cpufreq / 115200; /* 115200 baud rate */
// 	hal_cpuDataMemoryBarrier();
// 	*(console_common.base + cr1) |= 1;
// 	hal_cpuDataMemoryBarrier();

// 	_hal_consolePrint("\033[2J");
// 	_hal_consolePrint("\033[f");

// 	return;
// }
