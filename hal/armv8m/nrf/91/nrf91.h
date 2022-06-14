/*
 * Phoenix-RTOS
 *
 * Operating system kernel
 *
 * STM32 basic peripherals control functions
 *
 * Copyright 2017, 2019-2020 Phoenix Systems
 * Author: Aleksander Kaminski, Pawel Pisarczyk
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#ifndef _HAL_NRF91_H_
#define _HAL_NRF91_H_

#include <arch/types.h>
#include "../pmap.h"

// #if defined(CPU_STM32L152XD) || defined(CPU_STM32L152XE)
// #include "../../../include/arch/stm32l1.h"
// #endif

// #ifdef CPU_STM32L4X6
// #include "../../../include/arch/stm32l4.h"
// #endif

/* currently supported baud rates: 9600, 115200 */
#define UART_BAUDRATE 115200

/* sizes of uart dma memory regions - 1 because of max value of maxcnt */
#define UART_TX_DMA_SIZE 8191
#define UART_RX_DMA_SIZE 8191

#define UART0_BASE ((void *)0x50008000)
#define UART1_BASE ((void *)0x50009000)
#define UART2_BASE ((void *)0x5000A000)
#define UART3_BASE ((void *)0x5000B000)

#define UART0_IRQ uarte0
#define UART1_IRQ uarte1
#define UART2_IRQ uarte2
#define UART3_IRQ uarte3

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

// /* STM32L4 peripherals */
// enum {
// 	/* AHB1 */
// 	pctl_dma1 = 0, pctl_dma2, pctl_flash = 8, pctl_crc = 12, pctl_tsc = 16, pctl_dma2d,

// 	/* AHB2 */
// 	pctl_gpioa = 32, pctl_gpiob, pctl_gpioc, pctl_gpiod, pctl_gpioe, pctl_gpiof, pctl_gpiog, pctl_gpioh, pctl_gpioi,
// 	pctl_otgfs = 32 + 12, pctl_adc, pctl_dcmi, pctl_aes = 32 + 16, pctl_hash, pctl_rng,

// 	/* AHB3 */
// 	pctl_fmc = 64, pctl_qspi = 64 + 8,

// 	/* APB1_1 */
// 	pctl_tim2 = 96, pctl_tim3, pctl_tim4, pctl_tim5, pctl_tim6, pctl_tim7, pctl_lcd = 96 + 9, pctl_rtcapb,
// 	pctl_wwdg, pctl_spi2 = 96 + 14, pctl_spi3, pctl_usart2 = 96 + 17, pctl_usart3, pctl_uart4, pctl_uart5, pctl_i2c1,
// 	pctl_i2c2, pctl_i2c3, pctl_crs, pctl_can1, pctl_can2, pctl_pwr = 96 + 28, pctl_dac1, pctl_opamp, pctl_lptim1,

// 	/* APB1_2 */
// 	pctl_lpuart1 = 128, pctl_i2c4, pctl_swpmi1, pctl_lptim2 = 128 + 5,

// 	/* APB2 */
// 	pctl_syscfg = 160, pctl_fw = 160 + 7, pctl_sdmmc1 = 160 + 10, pctl_tim1, pctl_spi1, pctl_tim8, pctl_usart1,
// 	pctl_tim15, pctl_tim16, pctl_tim17, pctl_sai1 = 160 + 21, pctl_sai2, pctl_dfsdm1 = 160 + 24,

// 	/* MISC */
// 	pctl_rtc = 192
// };


// /* STM32L4 Interrupt numbers */
// enum { wwdq_irq = 16, pvd_pvm_irq, rtc_tamper_stamp_irq, rtc_wkup_irq, flash_irq, rcc_irq,
// 	exti0_irq, exti1_irq, exti2_irq, exti3_irq, exti4_irq, dma1_ch1_irq, dma1_ch2_irq,
// 	dma1_ch3_irq, dma1_ch4_irq, dma1_ch5_irq, dma1_ch6_irq, dma1_ch7_irq, adc1_2_irq,
// 	can1_tx_irq, can1_rx0_irq, can1_rx1_irq, can1_sce_irq, exti9_5_irq, tim1_brk_irq,
// 	tim1_up_irq, tim1_trg_com_irq, tim1_cc_irq, tim2_irq, tim3_irq, tim4_irq, i2c1_ev_irq,
// 	i2c1_er_irq, i2c2_ev_irq, i2c2_er_irq, spi1_irq, spi2_irq, usart1_irq, usart2_irq,
// 	usart3_irq, exti15_10_irq, rtc_alarm_irq, dfsdm1_flt3_irq, tim8_brk_irq, tim8_up_irq,
// 	tim8_trg_com_irq, tim8_cc_irq, adc3_irq, fmc_irq, sdmmc1_irq, tim5_irq, spi3_irq,
// 	uart4_irq, uart5_irq, tim6_dacunder_irq, tim7_irq, dma2_ch1_irq, dma2_ch2_irq,
// 	dma2_ch3_irq, dma2_ch4_irq, dma2_ch5_irq, dfsdm1_flt0_irq, dfsdm1_flt1_irq, dfsdm1_flt2_irq,
// 	comp_irq, lptim1_irq, lptim2_irq, otg_fs_irq, dm2_ch6_irq, dma2_ch7_irq, lpuart1_irq,
// 	quadspi_irq, i2c3_ev_irq, i2c3_er_irq, sai1_irq, sai2_irq, swpmi1_irq, tsc_irq, lcd_irq,
// 	aes_irq, rng_irq, fpu_irq, hash_irq, i2c4_ev_irq, i2c4_er_irq, dcmi_irq, can2_tx_irq,
// 	can2_rx0_irq, can2_rx1_irq, can2_sce_irq, dma2d_irq };


// /* Sets peripheral clock */
// extern int _stm32_rccSetDevClock(unsigned int d, u32 hz);


// /* Sets CPU clock to the closest smaller MSI frequency */
// extern int _stm32_rccSetCPUClock(u32 hz);


// extern int _stm32_rccGetDevClock(unsigned int d, u32 *hz);


// extern u32 _stm32_rccGetCPUClock(void);


// extern void _stm32_rccClearResetFlags(void);
extern void _nrf91_nvicSystemReset(void);


extern void _nrf91_timerClearEvent(void);


extern int _nrf91_timerInit(u32 interval);


extern unsigned int _nrf91_cpuid(void);


extern int _nrf91_gpioConfig(u8 pin, u8 dir, u8 pull);


extern int _nrf91_gpioSet(u8 pin, u8 val);
// extern int _stm32_gpioSet(unsigned int d, u8 pin, u8 val);


// extern int _stm32_gpioSetPort(unsigned int d, u16 val);


// extern int _stm32_gpioGet(unsigned int d, u8 pin, u8 *val);


// extern int _stm32_gpioGetPort(unsigned int d, u16 *val);


// /* Range = 0 - forbidden, 1 - 1.8V, 2 - 1.5V, 3 - 1.2V */
// extern void _stm32_pwrSetCPUVolt(u8 range);


// extern void _stm32_pwrEnterLPRun(u32 state);


// extern time_t _stm32_pwrEnterLPStop(time_t us);


// extern void _stm32_rtcUnlockRegs(void);


// extern void _stm32_rtcLockRegs(void);


// extern u32 _stm32_rtcGetms(void);


extern int _nrf91_rtcInit(u32 interval);


extern void _nrf91_rtcDone(void);


extern void _nrf91_rtcClear(void);


extern void _nrf91_nvicSetIRQ(s8 irqn, u8 state);


extern void _nrf91_nvicSetPriority(s8 irqn, u32 priority);

// /* platformctl syscall */
extern int hal_platformctl(void *);


// extern void _stm32_platformInit(void);


// /* Sets peripheral clock */
// extern int _stm32_rccSetDevClock(unsigned int d, u32 hz);


// /* Sets CPU clock to the closest smaller MSI freqency */
// extern int _stm32_rccSetCPUClock(u32 hz);


// extern int _stm32_rccGetDevClock(unsigned int d, u32 *hz);


// extern u32 _stm32_rccGetCPUClock(void);


// extern void _stm32_rccClearResetFlags(void);


// extern int _stm32_gpioConfig(unsigned int d, u8 pin, u8 mode, u8 af, u8 otype, u8 ospeed, u8 pupd);


// extern int _stm32_gpioSet(unsigned int d, u8 pin, u8 val);


// extern int _stm32_gpioSetPort(unsigned int d, u16 val);


// extern int _stm32_gpioGet(unsigned int d, u8 pin, u8 *val);


// extern int _stm32_gpioGetPort(unsigned int d, u16 *val);


// /* Range = 0 - forbidden, 1 - 1.8V, 2 - 1.5V, 3 - 1.2V */
// extern void _stm32_pwrSetCPUVolt(u8 range);


// extern void _stm32_pwrEnterLPRun(u32 state);


// extern time_t _stm32_pwrEnterLPStop(time_t us);


// extern void _stm32_rtcUnlockRegs(void);


// extern void _stm32_rtcLockRegs(void);


// extern u32 _stm32_rtcGetms(void);


// extern void _stm32_scbSetPriorityGrouping(u32 group);


// extern u32 _stm32_scbGetPriorityGrouping(void);


// extern void _stm32_scbSetPriority(s8 excpn, u32 priority);


// extern u32 _stm32_scbGetPriority(s8 excpn);


// extern void _stm32_nvicSetIRQ(s8 irqn, u8 state);


// extern u32 _stm32_nvicGetPendingIRQ(s8 irqn);


// extern void _stm32_nvicSetPendingIRQ(s8 irqn, u8 state);


// extern u32 _stm32_nvicGetActive(s8 irqn);


// extern void _stm32_nvicSetPriority(s8 irqn, u32 priority);


// extern u8 _stm32_nvicGetPriority(s8 irqn);


// extern void _stm32_nvicSystemReset(void);


// extern int _stm32_extiMaskInterrupt(u32 line, u8 state);


// extern int _stm32_extiMaskEvent(u32 line, u8 state);


// extern int _stm32_extiSetTrigger(u32 line, u8 state, u8 edge);


// extern int _stm32_syscfgExtiLineConfig(u8 port, u8 pin);


// extern int _stm32_extiSoftInterrupt(u32 line);


// extern u32 _stm32_extiGetPending(void);


// extern int _stm32_extiClearPending(u32 line);


// extern int _stm32_systickInit(u32 interval);


// extern void _stm32_systickSet(u8 state);


// extern u32 _stm32_systickGet(void);


// extern void _stm32_mpuReadRegion(u8 region, mpur_t *reg);


// extern void _stm32_mpuEnableRegion(u8 region, u8 state);


// extern void _stm32_mpuUpdateRegion(mpur_t *reg);


// extern unsigned int _stm32_cpuid(void);


// extern void _stm32_wdgReload(void);


// extern int _stm32_systickInit(u32 interval);


// extern void _nrf91_init(void);


#endif
