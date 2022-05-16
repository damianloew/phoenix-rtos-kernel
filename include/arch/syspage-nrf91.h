
/*
 * Phoenix-RTOS
 *
 * Operating system kernel
 *
 * HAL syspage for NRF91 boards
 *
 * Copyright 2021 Phoenix Systems
 * Authors: Hubert Buczynski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#ifndef _PHOENIX_SYSPAGE_NRF91_H_
#define _PHOENIX_SYSPAGE_NRF91_H_


typedef struct {
	struct {
		unsigned int type;
		unsigned int allocCnt;
		struct {
			unsigned int rbar;
			unsigned int rasr;
		} table[16] __attribute__((aligned(8)));
		unsigned int map[16]; /* ((unsigned int)-1) = map is not assigned */
	} __attribute__((packed)) mpu;
} __attribute__((packed)) hal_syspage_t;

#endif
