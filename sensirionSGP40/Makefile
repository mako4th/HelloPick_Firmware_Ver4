#******************************************************************************#{{{
#
# Makefile - Rules for building the libraries, examples and docs.
#
# Copyright (c) 2024, Ambiq Micro, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice,
# this list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright
# notice, this list of conditions and the following disclaimer in the
# documentation and/or other materials provided with the distribution.
#
# 3. Neither the name of the copyright holder nor the names of its
# contributors may be used to endorse or promote products derived from this
# software without specific prior written permission.
#
# Third party software included in this distribution is subject to the
# additional license terms as defined in the /docs/licenses directory.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# This is part of revision release_sdk_3_2_0-dd5f40c14b of the AmbiqSuite Development Package.
#
#******************************************************************************#}}}
TARGET := sgp40_apollo3.a
COMPILERNAME := gcc
PROJECT := sgp40_apollo3
BINDIR := bin
SDKROOT = ../../AmbiqSuite_R3.2.0
SHELL:=/bin/bash

# Enable printing explicit commands with 'make VERBOSE=1'
ifneq ($(VERBOSE),1)
Q:=@
endif

#### Setup ####{{{

TOOLCHAIN ?= arm-none-eabi
PART = apollo3
CPU = cortex-m4
FPU = fpv4-sp-d16
# Default to FPU hardware calling convention.  However, some customers and/or
# applications may need the software calling convention.
#FABI = softfp
FABI = hard
#}}}
#### Required Executables ####{{{
CC = $(TOOLCHAIN)-gcc
AR = $(TOOLCHAIN)-ar
RM = $(shell which rm 2>/dev/null)
#}}}
#defines#{{{
DEFINES = -DPART_$(PART)
DEFINES+= -DAM_PACKAGE_BGA
DEFINES+= -DAM_PART_APOLLO3
DEFINES+= -Dgcc
#}}}
#includes 不要削除済み#{{{
INCLUDES+= -I$(SDKROOT)/CMSIS/ARM/Include
INCLUDES+= -I$(SDKROOT)/CMSIS/AmbiqMicro/Include
INCLUDES+= -I$(SDKROOT)/devices
INCLUDES+= -I$(SDKROOT)/mcu/apollo3
INCLUDES+= -I$(SDKROOT)/mcu/apollo3/hal
INCLUDES+= -I$(SDKROOT)/utils
INCLUDES+= -I$(SDKROOT)/third_party/cordio/wsf/include
INCLUDES+= -I$(SDKROOT)/boards/apollo3_evb/bsp
INCLUDES+= -I$(SDKROOT)/third_party/cordio/ble-host/include
INCLUDES+= -I$(SDKROOT)/third_party/cordio/ble-host/sources/hci/ambiq
INCLUDES+= -I$(SDKROOT)/third_party/cordio/ble-host/sources/hci/ambiq/apollo3
INCLUDES+= -I$(SDKROOT)/third_party/cordio/ble-host/sources/stack/cfg
INCLUDES+= -I$(SDKROOT)/third_party/cordio/wsf/sources
INCLUDES+= -I$(SDKROOT)/third_party/cordio/wsf/sources/port/freertos
INCLUDES+= -I$(SDKROOT)/third_party/cordio/wsf/sources/util
INCLUDES+= -I$(SDKROOT)/third_party/cordio/ble-host/include
INCLUDES+= -I./src
#}}}
#VPATH#{{{
VPATH = $(SDKROOT)/utils
VPATH+= $(SDKROOT)/devices
VPATH+= $(SDKROOT)/third_party/cordio/stack/hci
VPATH+= $(SDKROOT)/third_party/cordio/stack/dm
VPATH+= $(SDKROOT)/third_party/cordio/stack/dm
VPATH+= $(SDKROOT)/third_party/cordio/ble-host/sources/stack/dm
VPATH+= $(SDKROOT)/third_party/cordio/ble-host/sources/stack/hci
VPATH+= $(SDKROOT)/third_party/cordio/ble-host/sources/app
VPATH+= $(SDKROOT)/third_party/cordio/ble-host/sources/hci/ambiq/apollo3
VPATH+= $(SDKROOT)/third_party/cordio/wsf/sources/common
VPATH+= $(SDKROOT)/third_party/cordio/wsf/sources/port/freertos
VPATH+= $(SDKROOT)/third_party/cordio/ble-profiles/sources/apps/app/common
VPATH+= ./src
#}}}
#SRC #{{{
SRC = am_util_delay.c
SRC += am_util_stdio.c
SRC += hci_drv_apollo3.c
SRC += sgp40_apollo3.c
SRC += sensirion_gas_index_algorithm.c
SRC += sensirion_common.c
SRC += sensirion_i2c_hal.c
SRC += sensirion_i2c.c
SRC += sgp40_i2c.c
SRC += am_devices_led.c
SRC += dm_main.c
SRC += dm_adv.c
SRC += hci_main.c
#}}}

CSRC = $(filter %.c,$(SRC))
ASRC = $(filter %.s,$(SRC))

OBJS = $(CSRC:%.c=$(BINDIR)/%.o)
OBJS+= $(ASRC:%.s=$(BINDIR)/%.o)

DEPS = $(CSRC:%.c=$(BINDIR)/%.d)
DEPS+= $(ASRC:%.s=$(BINDIR)/%.d)

#CFLAGS #{{{
CFLAGS = -mthumb -mcpu=$(CPU) -mfpu=$(FPU) -mfloat-abi=$(FABI)
CFLAGS+= -ffunction-sections -fdata-sections -fomit-frame-pointer
CFLAGS+= -MMD -MP -std=c99 -Wall -g
CFLAGS+= -O3
CFLAGS+= $(DEFINES)
CFLAGS+= $(INCLUDES)
CFLAGS+= 
#}}}
#LFLAGS #{{{
LFLAGS = -mthumb -mcpu=$(CPU) -mfpu=$(FPU) -mfloat-abi=$(FABI)
LFLAGS+= -nostartfiles -static
LFLAGS+= -Wl,--gc-sections,--entry,Reset_Handler,-Map,$(BINDIR)/$(TARGET).map
LFLAGS+= -Wl,--start-group -lm -lc -lgcc $(LIBS) -Wl,--end-group
#}}}
# Additional user specified CFLAGS{{{
CPFLAGS = -Obinary
ODFLAGS = -S
#}}}
#### Rules ####{{{
all: directories $(BINDIR)/$(TARGET)

directories: $(BINDIR)

$(BINDIR):
	@mkdir -p $@

$(BINDIR)/%.o: %.c $(BINDIR)/%.d
	@echo " Compiling $(COMPILERNAME) $<"
	$(Q) $(CC) -c $(CFLAGS) $< -o $@

$(BINDIR)/$(TARGET): $(OBJS)
	@echo " Linking $(AR) $@"
	$(Q) $(AR) rcs $@ $^
clean:
	@echo "Cleaning..."
	$(Q) $(RM) -rf bin

$(BINDIR)/%.d: ;
#}}}
# Automatically include any generated dependencies
-include $(DEPS)

.PHONY: all clean directories
# vim: set foldmethod=marker commentstring=#%s :
