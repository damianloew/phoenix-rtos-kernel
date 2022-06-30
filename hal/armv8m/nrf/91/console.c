/*
 * Phoenix-RTOS
 *
 * Operating system kernel
 *
 * HAL console (nRF9160 UART)
 *
 * Copyright 2022 Phoenix Systems
 * Author: Damian Loewnau
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#include "nrf91.h"

#include "../../../console.h"
#include "../../../cpu.h"
#include "../../armv8m.h"


static struct {
	volatile u32 *base;
} console_common;


enum { uarte_startrx = 0, uarte_stoprx, uarte_starttx, uarte_stoptx, 
uarte_events_cts = 64, uarte_events_txdrdy = 71, uarte_events_endtx, uarte_events_error, uarte_events_txstarted = 84, 
uarte_inten = 192, uarte_errorsrc = 288, uarte_intenset, uarte_intenclr, uarte_enable = 320, 
uarte_psel_rts = 322, uarte_psel_txd, uarte_psel_cts, uarte_psel_rxd, uarte_baudrate = 329, 
uarte_rxd_ptr = 333, uarte_rxd_maxcnt, uarte_rxd_amount, uarte_txd_ptr = 337, uarte_txd_maxcnt, uarte_txd_amount, 
uarte_config = 347 };


enum { baud_9600 = 0x00275000, baud_115200 = 0x01D60000 };


/* Init pins according to nrf9160 product specification */
static void console_configPins(void)
{
	_nrf91_gpioConfig(UART0_TX, output, nopull);
	_nrf91_gpioConfig(UART0_RX, input, nopull);
	_nrf91_gpioConfig(UART0_RTS, output, nopull);
	_nrf91_gpioConfig(UART0_CTS, input, pulldown);

	_nrf91_gpioSet(UART0_TX, high);
	_nrf91_gpioSet(UART0_RTS, high);
}


/* Maximum number of characters that will be sent is specifed by UART_TX_DMA_SIZE */
void _hal_consolePrint(const char *s)
{
	volatile char *tx_dma_buff = (volatile char *)UART0_TX_DMA;
	int cnt = 0;

	do {
		tx_dma_buff[cnt] = s[cnt];
		cnt++;
	} while (s[cnt-1] != '\0');

	if (cnt > UART_TX_DMA_SIZE) {
		cnt = UART_TX_DMA_SIZE;
	}

	*(console_common.base + uarte_txd_ptr) = UART0_TX_DMA;
	*(console_common.base + uarte_txd_maxcnt) = cnt;
	*(console_common.base + uarte_starttx) = 1u;
	while ( *(console_common.base + uarte_events_txstarted) != 1u )
		;
	*(console_common.base + uarte_events_txstarted) = 0u;

	while ( *(console_common.base + uarte_events_endtx) != 1u )
		;
	*(console_common.base + uarte_events_endtx) = 0u;
}


void hal_consolePrint(int attr, const char *s)
{
	if (attr == ATTR_BOLD)
		_hal_consolePrint(CONSOLE_BOLD);
	else if (attr != ATTR_USER)
		_hal_consolePrint(CONSOLE_CYAN);

	_hal_consolePrint(s);
	_hal_consolePrint(CONSOLE_NORMAL);
}


void hal_consolePutch(char c)
{
	volatile char *tx_dma_buff = (volatile char *)UART0_TX_DMA;

	tx_dma_buff[0] = c;

	*(console_common.base + uarte_txd_ptr) = UART0_TX_DMA;
	*(console_common.base + uarte_txd_maxcnt) = 1u;
	*(console_common.base + uarte_starttx) = 1u;
	while ( *(console_common.base + uarte_events_txstarted) != 1u )
		;
	*(console_common.base + uarte_events_txstarted) = 0u;

	while ( *(console_common.base + uarte_events_endtx) != 1u )
		;
	*(console_common.base + uarte_events_endtx) = 0u;
}


/* UART0 supported for the hal console */
void _hal_consoleInit(void)
{
	console_common.base = UART0_BASE;
	console_configPins();

	/* disable uarte instance */
	*(console_common.base + uarte_enable) = 0u;
	hal_cpuDataMemoryBarrier();

	/* Select pins */
	*(console_common.base + uarte_psel_txd) = UART0_TX;
	*(console_common.base + uarte_psel_rxd) = UART0_RX;
	*(console_common.base + uarte_psel_rts) = UART0_RTS;
	*(console_common.base + uarte_psel_cts) = UART0_CTS;


	switch (UART_BAUDRATE) {
		case 9600:
			*(console_common.base + uarte_baudrate) = baud_9600;
			break;
		case 115200:
		default:
			*(console_common.base + uarte_baudrate) = baud_115200;
	}

	/* Default settings - hardware flow control disabled, exclude parity bit, one stop bit */
	*(console_common.base + uarte_config) = 0u;

	/* Set default max number of bytes in specific buffers */
	*(console_common.base + uarte_txd_maxcnt) = UART_TX_DMA_SIZE;
	*(console_common.base + uarte_rxd_maxcnt) = UART_RX_DMA_SIZE;

	/* Set default memory regions for uart dma */
	*(console_common.base + uarte_txd_ptr) = UART0_TX_DMA;
	*(console_common.base + uarte_rxd_ptr) = UART0_RX_DMA;

	/* disable all uart interrupts */
	*(console_common.base + uarte_intenclr) = 0xFFFFFFFF;
	hal_cpuDataMemoryBarrier();

	/* enable uarte instance */
	*(console_common.base + uarte_enable) = 0x8;
	hal_cpuDataMemoryBarrier();
}
