/*
 * Phoenix-RTOS
 *
 * Operating system kernel
 *
 * NRF91 basic peripherals control functions
 *
 * Copyright 2020 Phoenix Systems
 * Author: Aleksander Kaminski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#ifndef _PHOENIX_ARCH_NRF91_H_
#define _PHOENIX_ARCH_NRF91_H_

/* TODO: update */
#define PCTL_REBOOT_MAGIC 0xaa55aa55UL

#define FWRSTF   (1 << 0)
#define OBLRSTF  (1 << 1)
#define PINRSTF  (1 << 2)
#define BORRSTF  (1 << 3)
#define SFTRSTF  (1 << 4)
#define IWDGRSTF (1 << 5)
#define WWDGRSTF (1 << 6)
#define LPWRRSTF (1 << 7)

/* NRF91 peripherals */
enum {
	/* AHB1 */
	pctl_dma1 = 0, pctl_dma2, pctl_flash = 8, pctl_crc = 12, pctl_tsc = 16, pctl_dma2d,

	/* AHB2 */
	pctl_gpioa = 32, pctl_gpiob, pctl_gpioc, pctl_gpiod, pctl_gpioe, pctl_gpiof, pctl_gpiog, pctl_gpioh, pctl_gpioi,
	pctl_otgfs = 32 + 12, pctl_adc, pctl_dcmi, pctl_aes = 32 + 16, pctl_hash, pctl_rng,

	/* AHB3 */
	pctl_fmc = 64, pctl_qspi = 64 + 8,

	/* APB1_1 */
	pctl_tim2 = 96, pctl_tim3, pctl_tim4, pctl_tim5, pctl_tim6, pctl_tim7, pctl_lcd = 96 + 9, pctl_rtcapb,
	pctl_wwdg, pctl_spi2 = 96 + 14, pctl_spi3, pctl_usart2 = 96 + 17, pctl_usart3, pctl_uart4, pctl_uart5, pctl_i2c1,
	pctl_i2c2, pctl_i2c3, pctl_crs, pctl_can1, pctl_can2, pctl_pwr = 96 + 28, pctl_dac1, pctl_opamp, pctl_lptim1,

	/* APB1_2 */
	pctl_lpuart1 = 128, pctl_i2c4, pctl_swpmi1, pctl_lptim2 = 128 + 5,

	/* APB2 */
	pctl_syscfg = 160, pctl_fw = 160 + 7, pctl_sdmmc1 = 160 + 10, pctl_tim1, pctl_spi1, pctl_tim8, pctl_usart1,
	pctl_tim15, pctl_tim16, pctl_tim17, pctl_sai1 = 160 + 21, pctl_sai2, pctl_dfsdm1 = 160 + 24,

	/* MISC */
	pctl_rtc = 192
};


/* nRF9160 peripheral id's - same as irq numbers */
enum { spu = 3, regulators, clock = 5, power = 5, ctrlapperi, spi0 = 8, twi0 = 8, uarte0 = 8,
spi1 = 9, twi1 = 9, uarte1 = 9, spi2 = 10, twi2 = 10, uarte2 = 10, spi3 = 11, twi3 = 11, uarte3 = 11,
gpiote0 = 13, saadc, timer0, timer1, timer2, rtc0 = 20, rtc1, ddpic = 23, wdt, 
egu0 = 27, egu1, egu2, egu3, egu4, egu5, pwm0, pwm1, pwm2, pwm3, pdm = 38, 
i2s = 40, ipc = 42, fpu = 44, gpiote1 = 49, kmu = 57, nvmc = 57, vmc, cc_host_rgf = 64,
cryptocell = 64, gpio = 66 };


typedef struct {
	enum { pctl_set = 0, pctl_get } action;
	enum { pctl_devclk = 0, pctl_cpuclk, pctl_reboot } type;

	union {
		struct {
			unsigned int dev;
			unsigned int state;
		} devclk;

		struct {
			unsigned int hz;
		} cpuclk;

		struct {
			unsigned int magic;
			unsigned int reason;
		} reboot;
	};
} __attribute__((packed)) platformctl_t;


#endif
