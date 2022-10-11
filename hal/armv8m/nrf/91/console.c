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
	u8 txPin;
	u8 rxPin;
	u8 rtsPin;
	u8 ctsPin;
	volatile u32 *txDma;
	volatile u32 *rxDma;
	u32 txDmaSize;
	u32 rxDmaSize;
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
	_nrf91_gpioConfig(console_common.txPin, output, nopull);
	_nrf91_gpioConfig(console_common.rxPin, input, nopull);
	_nrf91_gpioConfig(console_common.rtsPin, output, nopull);
	_nrf91_gpioConfig(console_common.ctsPin, input, pulldown);

	_nrf91_gpioSet(console_common.txPin, high);
	_nrf91_gpioSet(console_common.rtsPin, high);
}


/* Maximum number of characters that will be sent is specifed by console_common.txDmaSize */
void _hal_consolePrint(const char *s)
{
	volatile char *tx_dma_buff = (volatile char *)console_common.txDma;
	int cnt = 0;

	do {
		tx_dma_buff[cnt] = s[cnt];
		cnt++;
	} while ((s[cnt-1] != '\0') && (cnt < console_common.txDmaSize));

	*(console_common.base + uarte_txd_ptr) = console_common.txDma;
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
	volatile char *tx_dma_buff = (volatile char *)console_common.txDma;

	tx_dma_buff[0] = c;

	*(console_common.base + uarte_txd_ptr) = console_common.txDma;
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
	/* Supported configuartions - uart0/uart2 + uart1/uart3
	   sizes of uart dma memory regions are set to max value of txd_maxcnt/rxd_maxcnt register (8191)
	   uart0 pins - default uart instance for nrf9160 dk, connected to VCOM0 
	   uart1 pins -second uart interface on nrf9160 dk called nRF91_UART_2 on the board's schematic
	   uart0 dma - ram7: section 2 and 3
	   uart1 dma - ram7: section 0 and 1 */

	const struct {
		u8 uart;
		volatile u32 *base;
		u8 txPin;
		u8 rxPin;
		u8 rtsPin;
		u8 ctsPin;
		volatile u32 *txDma;
		volatile u32 *rxDma;
		u32 txDmaSize;
		u32 rxDmaSize;
	} uarts[] = {
		{0, 0x50008000, 29, 28, 27, 26, 0x2003C000, 0x2003E000, 8191},
		{1, 0x50009000, 1, 0, 14, 15, 0x20038000, 0x2003A000, 8191},
		{2, 0x5000A000, 29, 28, 27, 26, 0x2003C000, 0x2003E000, 8191},
		{3, 0x5000B000, 1, 0, 14, 15, 0x20038000, 0x2003A000, 8191}
	};

	const int uart = 0;
	console_common.base = uarts[uart].base;
	console_common.txPin = uarts[uart].txPin;
	console_common.rxPin = uarts[uart].rxPin;
	console_common.rtsPin = uarts[uart].rtsPin;
	console_common.ctsPin = uarts[uart].ctsPin;
	console_common.txDma = uarts[uart].txDma;
	console_common.rxDma = uarts[uart].rxDma;
	console_common.txDmaSize = uarts[uart].txDmaSize;
	console_common.rxDmaSize = uarts[uart].rxDmaSize;

	console_configPins();

	/* disable uarte instance */
	*(console_common.base + uarte_enable) = 0u;
	hal_cpuDataMemoryBarrier();

	/* Select pins */
	*(console_common.base + uarte_psel_txd) = console_common.txPin;
	*(console_common.base + uarte_psel_rxd) = console_common.rxPin;
	*(console_common.base + uarte_psel_rts) = console_common.rtsPin;
	*(console_common.base + uarte_psel_cts) = console_common.ctsPin;

	/* currently supported baud rates: 9600, 115200 */
	*(console_common.base + uarte_baudrate) = baud_115200;

	/* Default settings - hardware flow control disabled, exclude parity bit, one stop bit */
	*(console_common.base + uarte_config) = 0u;

	/* Set default max number of bytes in specific buffers */
	*(console_common.base + uarte_txd_maxcnt) = console_common.txDmaSize;
	*(console_common.base + uarte_rxd_maxcnt) = console_common.rxDmaSize;

	/* Set default memory regions for uart dma */
	*(console_common.base + uarte_txd_ptr) = console_common.txDma;
	*(console_common.base + uarte_rxd_ptr) = console_common.rxDma;

	/* disable all uart interrupts */
	*(console_common.base + uarte_intenclr) = 0xFFFFFFFF;
	hal_cpuDataMemoryBarrier();

	/* enable uarte instance */
	*(console_common.base + uarte_enable) = 0x8;
	hal_cpuDataMemoryBarrier();
}
