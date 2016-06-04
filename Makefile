#******************************************************************************
#
# Makefile - Rules for building the moto example.
#
# Copyright (c) 2013-2015 Texas Instruments Incorporated.  All rights reserved.
# Software License Agreement
# 
# Texas Instruments (TI) is supplying this software for use solely and
# exclusively on TI's microcontroller products. The software is owned by
# TI and/or its suppliers, and is protected under applicable copyright
# laws. You may not combine this software with "viral" open-source
# software in order to form a larger program.
# 
# THIS SOFTWARE IS PROVIDED "AS IS" AND WITH ALL FAULTS.
# NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT
# NOT LIMITED TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
# A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. TI SHALL NOT, UNDER ANY
# CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR CONSEQUENTIAL
# DAMAGES, FOR ANY REASON WHATSOEVER.
# 
# This is part of revision 2.1.2.111 of the EK-TM4C1294XL Firmware Package.
#
#******************************************************************************

#
# Defines the part type that this project uses.
#
#PART=TM4C1294NCPDT
PART=TM4C123GH6PM
#
# The base directory for TivaWare.
#
ROOT=./

#
# Include the common make definitions.
#
include ${ROOT}/makedefs

#
# Where to find header files that do not live in the source directory.
#

IPATH=.
IPATH+=./inc
IPATH+=./inc/utils
VPATH=./src
VPATH+=./src/tm4c123
#VPATH+=./src/tm4c129
VPATH+=./src/lib
#
# The default rule, which causes the moto example to be built.
#
all: ${COMPILER}
all: ${COMPILER}/moto.axf

install:
	openocd -f openocd.cfg -c "flash_image" 
#
# The rule to clean out all the build products.
#
clean:
	@rm -rf ${COMPILER} ${wildcard *~}

#
# The rule to create the target directory.
#
${COMPILER}:
	@mkdir -p ${COMPILER}

#
# Rules for building the moto example.
#
${COMPILER}/moto.axf: ${COMPILER}/startup_${COMPILER}.o
#${COMPILER}/moto.axf: ${COMPILER}/softssi.o
#${COMPILER}/moto.axf: ${COMPILER}/uartstdio.o
#${COMPILER}/moto.axf: ${COMPILER}/soft_spi_master.o
#${COMPILER}/moto.axf: ${COMPILER}/spi_master.o
#${COMPILER}/moto.axf: ${COMPILER}/ti_master.o
${COMPILER}/moto.axf: ${COMPILER}/moto.o
${COMPILER}/moto.axf: ${COMPILER}/gpio.o
${COMPILER}/moto.axf: ${COMPILER}/ssi.o
${COMPILER}/moto.axf: ${COMPILER}/sysctl.o
${COMPILER}/moto.axf: ${COMPILER}/uart.o
${COMPILER}/moto.axf: ${COMPILER}/uartstdio.o
${COMPILER}/moto.axf: ${COMPILER}/pwm.o
${COMPILER}/moto.axf: ${COMPILER}/lib6480.o
${COMPILER}/moto.axf: ${COMPILER}/cmdline.o
${COMPILER}/moto.axf: ${COMPILER}/uart_commands.o
${COMPILER}/moto.axf: ${COMPILER}/ustdlib.o
${COMPILER}/moto.axf: ${COMPILER}/interrupt.o
${COMPILER}/moto.axf: ${COMPILER}/hal.o
${COMPILER}/moto.axf: ${COMPILER}/softssi.o
${COMPILER}/moto.axf: ${COMPILER}/systick.o
${COMPILER}/moto.axf: moto.ld
SCATTERgcc_moto=moto.ld
ENTRY_moto=ResetISR
#CFLAGSgcc=-DTARGET_IS_TM4C129_RA0
CFLAGSgcc=-DTARGET_IS_TM4C123_RA1 -DUART_BUFFERED
#
# Include the automatically generated dependency files.
#
ifneq (${MAKECMDGOALS},clean)
-include ${wildcard ${COMPILER}/*.d} __dummy__
endif
