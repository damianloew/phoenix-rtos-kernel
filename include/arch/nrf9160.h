/*
 * Phoenix-RTOS
 *
 * Operating system kernel
 *
 * Macros and enums for NRF9160 related code
 *
 * Copyright 2022 Phoenix Systems
 * Author: Damian Loewnau
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#ifndef _PHOENIX_ARCH_NRF9160_H_
#define _PHOENIX_ARCH_NRF9160_H_

/* currently supported baud rates: 9600, 115200 */
#define UART_BAUDRATE 115200

/* sizes of uart dma memory regions - it's set to max value of txd_maxcnt/rxd_maxcnt register */
#define UART_TX_DMA_SIZE 8191
#define UART_RX_DMA_SIZE 8191

#define UART0_BASE ((void *)0x50008000)
#define UART1_BASE ((void *)0x50009000)
#define UART2_BASE ((void *)0x5000A000)
#define UART3_BASE ((void *)0x5000B000)

/* default uart instance for nrf9160 dk, connected to VCOM0 */
#define UART0_TX 29
#define UART0_RX 28
#define UART0_RTS 27
#define UART0_CTS 26

/* second uart interface on nrf9160 dk called nRF91_UART_2 on the board's schematic*/
#define UART1_TX 1
#define UART1_RX 0
#define UART1_RTS 14
#define UART1_CTS 15

#define UART2_TX UART0_TX
#define UART2_RX UART0_RX
#define UART2_RTS UART0_RTS
#define UART2_CTS UART0_CTS

#define UART3_TX UART1_TX
#define UART3_RX UART1_RX
#define UART3_RTS UART1_RTS
#define UART3_CTS UART1_CTS

/* ram7: section 2 and 3 */
#define UART0_TX_DMA 0x2003C000
#define UART0_RX_DMA 0x2003E000

/* ram7: section 0 and 1 */
#define UART1_TX_DMA 0x20038000
#define UART1_RX_DMA 0x2003A000

#define UART2_TX_DMA UART0_TX_DMA
#define UART2_RX_DMA UART0_RX_DMA

#define UART3_TX_DMA UART1_TX_DMA
#define UART3_RX_DMA UART1_RX_DMA

#define FLASH_PROGRAM_1_ADDR    0x00000000
#define FLASH_PROGRAM_BANK_SIZE (1024 * 1024)


enum { input = 0, output };
enum { low = 0, high };
enum { nopull = 0, pulldown, pullup = 3};

enum { nvic_iser = 0, nvic_icer = 32, nvic_ispr = 64, nvic_icpr = 96, nvic_iabr = 128,
	nvic_ip = 192 };

/* nRF9160 peripheral id's - same as irq numbers */
enum { spu = 3, regulators, clock = 5, power = 5, ctrlapperi, spi0 = 8, twi0 = 8, uarte0 = 8,
spi1 = 9, twi1 = 9, uarte1 = 9, spi2 = 10, twi2 = 10, uarte2 = 10, spi3 = 11, twi3 = 11, uarte3 = 11,
gpiote0 = 13, saadc, timer0, timer1, timer2, rtc0 = 20, rtc1, ddpic = 23, wdt, 
egu0 = 27, egu1, egu2, egu3, egu4, egu5, pwm0, pwm1, pwm2, pwm3, pdm = 38, 
i2s = 40, ipc = 42, fpu = 44, gpiote1 = 49, kmu = 57, nvmc = 57, vmc, cc_host_rgf = 64,
cryptocell = 64, gpio = 66 };


#endif
