#
# Makefile for Phoenix-RTOS kernel (ARMv7 HAL)
#
# Copyright 2016-2017, 2019-2020 Phoenix Systems
#


ifneq (, $(findstring nrf, $(TARGET_SUBFAMILY)))
    include hal/armv8m/nrf/Makefile
    CFLAGS += -Ihal/armv8m/nrf -Ihal/armv8m
endif

$(info _________________)
OBJS += $(addprefix $(PREFIX_O)hal/armv8m/, string.o spinlock.o cpu.o hal.o pmap.o exceptions.o)
