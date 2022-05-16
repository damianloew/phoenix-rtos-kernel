#
# Makefile for Phoenix-RTOS kernel (ARMv7 HAL)
#
# Copyright 2016-2017, 2019-2020 Phoenix Systems
#


ifneq (, $(findstring nrf91, $(TARGET_SUBFAMILY)))
    include hal/armv8m/nrf/91/Makefile
endif

# CFLAGS += -Ihal/armv8m/nrf

OBJS += $(addprefix $(PREFIX_O)hal/armv8m/nrf/, _init.o interrupts.o)
