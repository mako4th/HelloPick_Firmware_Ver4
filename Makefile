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

TARGET := HelloPickFirmware_build
COMPILERNAME := gcc
PROJECT := HelloPick_Apollo3_ver4
SDKROOT := ./AmbiqSuite_R3.2.0
SRCDIR := src
BINDIR := bin
BUILDNUMHEADER := src/build_number.h
BUILDNUM := $(shell awk '{printf "%03d",$$3}' $(BUILDNUMHEADER))
SHELL := /bin/bash
#$(info $(TARGET))
#$(error STOP)

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

LINKER_FILE := src/hello_pick_apollo3.ld
STARTUP_FILE := ./startup_$(COMPILERNAME).c
#}}}
#### Required Executables ####{{{
CC = $(TOOLCHAIN)-gcc
GCC = $(TOOLCHAIN)-gcc
CPP = $(TOOLCHAIN)-cpp
LD = $(TOOLCHAIN)-ld
CP = $(TOOLCHAIN)-objcopy
OD = $(TOOLCHAIN)-objdump
RD = $(TOOLCHAIN)-readelf
AR = $(TOOLCHAIN)-ar
SIZE = $(TOOLCHAIN)-size
RM = $(shell which rm 2>/dev/null)

EXECUTABLES = CC LD CP OD AR RD SIZE GCC
K := $(foreach exec,$(EXECUTABLES),\
        $(if $(shell which $($(exec)) 2>/dev/null),,\
        $(info $(exec) not found on PATH ($($(exec))).)$(exec)))
$(if $(strip $(value K)),$(info Required Program(s) $(strip $(value K)) not found))
#}}}
ifneq ($(strip $(value K)),)
all clean:
	$(info Tools $(TOOLCHAIN)-$(COMPILERNAME) not installed.)
	$(RM) -rf bin
else
#DEFINES #{{{
DEFINES = -DPART_$(PART)
DEFINES+= -DAM_FREERTOS
DEFINES+= -D'AM_MULTIBOOT_CONFIG_FILE="amota_profile_config.h"'
DEFINES+= -DAM_PACKAGE_BGA
DEFINES+= -DAM_PART_APOLLO3
DEFINES+= -DSEC_ECC_CFG=SEC_ECC_CFG_HCI
DEFINES+= -Dgcc

ifeq ($(HP_RELEASE),)
DEFINES+= -DAM_DEBUG_PRINTF
DEFINES+= -DWSF_TRACE_ENABLED
DEFINES+= -DHCI_TRACE_ENABLED
#$(info defines = $(DEFINES))
#$(error STOP)
endif
#}}}
#includes {{{
INCLUDES = -Isrc
INCLUDES+= -IsensirionSGP40
INCLUDES+= -I$(SDKROOT)
INCLUDES+= -I$(SDKROOT)/CMSIS/ARM/Include
INCLUDES+= -I$(SDKROOT)/CMSIS/AmbiqMicro/Include
INCLUDES+= -I$(SDKROOT)/ambiq_ble/apps/amota
INCLUDES+= -I$(SDKROOT)/ambiq_ble/profiles/amota
INCLUDES+= -I$(SDKROOT)/ambiq_ble/services
INCLUDES+= -I$(SDKROOT)/devices
INCLUDES+= -I$(SDKROOT)/mcu/apollo3
INCLUDES+= -I$(SDKROOT)/third_party/FreeRTOSv10.5.1/Source/include
INCLUDES+= -I$(SDKROOT)/third_party/FreeRTOSv10.5.1/Source/portable/GCC/AMapollo2
INCLUDES+= -I$(SDKROOT)/third_party/cordio/ble-host/include
INCLUDES+= -I$(SDKROOT)/third_party/cordio/ble-host/sources/hci/ambiq
INCLUDES+= -I$(SDKROOT)/third_party/cordio/ble-host/sources/hci/ambiq/apollo3
INCLUDES+= -I$(SDKROOT)/third_party/cordio/ble-host/sources/sec/common
INCLUDES+= -I$(SDKROOT)/third_party/cordio/ble-host/sources/stack/att
INCLUDES+= -I$(SDKROOT)/third_party/cordio/ble-host/sources/stack/cfg
INCLUDES+= -I$(SDKROOT)/third_party/cordio/ble-host/sources/stack/dm
INCLUDES+= -I$(SDKROOT)/third_party/cordio/ble-host/sources/stack/hci
INCLUDES+= -I$(SDKROOT)/third_party/cordio/ble-host/sources/stack/l2c
INCLUDES+= -I$(SDKROOT)/third_party/cordio/ble-host/sources/stack/smp
INCLUDES+= -I$(SDKROOT)/third_party/cordio/ble-profiles/include/app
INCLUDES+= -I$(SDKROOT)/third_party/cordio/ble-profiles/sources/apps
INCLUDES+= -I$(SDKROOT)/third_party/cordio/ble-profiles/sources/apps/app
INCLUDES+= -I$(SDKROOT)/third_party/cordio/ble-profiles/sources/profiles
INCLUDES+= -I$(SDKROOT)/third_party/cordio/ble-profiles/sources/profiles/bas
INCLUDES+= -I$(SDKROOT)/third_party/cordio/ble-profiles/sources/profiles/gap
INCLUDES+= -I$(SDKROOT)/third_party/cordio/ble-profiles/sources/profiles/gatt
INCLUDES+= -I$(SDKROOT)/third_party/cordio/ble-profiles/sources/profiles/hid
INCLUDES+= -I$(SDKROOT)/third_party/cordio/ble-profiles/sources/profiles/hrps
INCLUDES+= -I$(SDKROOT)/third_party/cordio/ble-profiles/sources/profiles/include
INCLUDES+= -I$(SDKROOT)/third_party/cordio/ble-profiles/sources/profiles/rscp
INCLUDES+= -I$(SDKROOT)/third_party/cordio/ble-profiles/sources/profiles/uribeacon
INCLUDES+= -I$(SDKROOT)/third_party/cordio/ble-profiles/sources/services
INCLUDES+= -I$(SDKROOT)/third_party/cordio/wsf/include
INCLUDES+= -I$(SDKROOT)/third_party/cordio/wsf/sources
INCLUDES+= -I$(SDKROOT)/third_party/cordio/wsf/sources/port/freertos
INCLUDES+= -I$(SDKROOT)/third_party/cordio/wsf/sources/util
INCLUDES+= -I$(SDKROOT)/third_party/uecc
INCLUDES+= -I$(SDKROOT)/utils
INCLUDES+= -I$(SDKROOT)/boards/$(PART)_evb/bsp
#}}}
#VPATH#{{{
VPATH =:src
VPATH+= $(SDKROOT)/ambiq_ble/apps/amota
VPATH+=:$(SDKROOT)/ambiq_ble/profiles/amota
VPATH+=:$(SDKROOT)/ambiq_ble/services
VPATH+=:$(SDKROOT)/devices
VPATH+=:$(SDKROOT)/third_party/FreeRTOSv10.5.1/Source
VPATH+=:$(SDKROOT)/third_party/FreeRTOSv10.5.1/Source/portable/GCC/AMapollo2
VPATH+=:$(SDKROOT)/third_party/FreeRTOSv10.5.1/Source/portable/MemMang
VPATH+=:$(SDKROOT)/third_party/cordio/ble-host/sources/hci/ambiq
VPATH+=:$(SDKROOT)/third_party/cordio/ble-host/sources/hci/ambiq/apollo3
VPATH+=:$(SDKROOT)/third_party/cordio/ble-host/sources/sec/common
VPATH+=:$(SDKROOT)/third_party/cordio/ble-host/sources/sec/uecc
VPATH+=:$(SDKROOT)/third_party/cordio/ble-host/sources/stack/att
VPATH+=:$(SDKROOT)/third_party/cordio/ble-host/sources/stack/cfg
VPATH+=:$(SDKROOT)/third_party/cordio/ble-host/sources/stack/dm
VPATH+=:$(SDKROOT)/third_party/cordio/ble-host/sources/stack/hci
VPATH+=:$(SDKROOT)/third_party/cordio/ble-host/sources/stack/l2c
VPATH+=:$(SDKROOT)/third_party/cordio/ble-host/sources/stack/smp
VPATH+=:$(SDKROOT)/third_party/cordio/ble-profiles/sources/apps/app
VPATH+=:$(SDKROOT)/third_party/cordio/ble-profiles/sources/apps/app/common
VPATH+=:$(SDKROOT)/third_party/cordio/ble-profiles/sources/apps/hidapp
VPATH+=:$(SDKROOT)/third_party/cordio/ble-profiles/sources/profiles/bas
VPATH+=:$(SDKROOT)/third_party/cordio/ble-profiles/sources/profiles/gap
VPATH+=:$(SDKROOT)/third_party/cordio/ble-profiles/sources/profiles/gatt
VPATH+=:$(SDKROOT)/third_party/cordio/ble-profiles/sources/profiles/hid
VPATH+=:$(SDKROOT)/third_party/cordio/ble-profiles/sources/profiles/hrps
VPATH+=:$(SDKROOT)/third_party/cordio/ble-profiles/sources/profiles/rscp
VPATH+=:$(SDKROOT)/third_party/cordio/ble-profiles/sources/services
VPATH+=:$(SDKROOT)/third_party/cordio/wsf/sources/port/freertos
VPATH+=:$(SDKROOT)/third_party/cordio/wsf/sources/util
VPATH+=:$(SDKROOT)/third_party/uecc
VPATH+=:$(SDKROOT)/utils
#}}}
#SRC{{{
SRC = am_devices_button.c
SRC += adc_vbatt.c
SRC += am_devices_led.c
SRC += sensor_tasks.c
SRC += am_util_ble.c
SRC += am_util_bootloader.c
SRC += am_util_debug.c
SRC += am_util_delay.c
SRC += am_util_faultisr.c
SRC += am_util_id.c
SRC += am_util_multi_boot.c
SRC += am_util_stdio.c
SRC += amota_main.c
SRC += amotas_main.c
SRC += hello_pick_main.c
SRC += app_db.c
SRC += app_disc.c
SRC += app_hw.c
SRC += app_main.c
SRC += app_master.c
SRC += app_master_ae.c
SRC += app_master_leg.c
SRC += app_server.c
SRC += app_slave.c
SRC += app_slave_ae.c
SRC += app_slave_leg.c
SRC += app_terminal.c
SRC += app_ui.c
SRC += att_eatt.c
SRC += att_main.c
SRC += att_uuid.c
SRC += attc_disc.c
SRC += attc_eatt.c
SRC += attc_main.c
SRC += attc_proc.c
SRC += attc_read.c
SRC += attc_sign.c
SRC += attc_write.c
SRC += atts_ccc.c
SRC += atts_csf.c
SRC += atts_dyn.c
SRC += atts_eatt.c
SRC += atts_ind.c
SRC += atts_main.c
SRC += atts_proc.c
SRC += atts_read.c
SRC += atts_sign.c
SRC += atts_write.c
SRC += bas_main.c
SRC += bda.c
SRC += ble_freertos_amota.c
SRC += bstream.c
SRC += calc128.c
SRC += cfg_stack.c
SRC += crc32.c
SRC += dm_adv.c
SRC += dm_adv_ae.c
SRC += dm_adv_leg.c
SRC += dm_bis_master.c
SRC += dm_bis_slave.c
SRC += dm_cis.c
SRC += dm_cis_master.c
SRC += dm_cis_slave.c
SRC += dm_cis_sm.c
SRC += dm_conn.c
SRC += dm_conn_cte.c
SRC += dm_conn_master.c
SRC += dm_conn_master_ae.c
SRC += dm_conn_master_leg.c
SRC += dm_conn_slave.c
SRC += dm_conn_slave_ae.c
SRC += dm_conn_slave_leg.c
SRC += dm_conn_sm.c
SRC += dm_dev.c
SRC += dm_dev_priv.c
SRC += dm_iso.c
SRC += dm_main.c
SRC += dm_past.c
SRC += dm_phy.c
SRC += dm_priv.c
SRC += dm_scan.c
SRC += dm_scan_ae.c
SRC += dm_scan_leg.c
SRC += dm_sec.c
SRC += dm_sec_lesc.c
SRC += dm_sec_master.c
SRC += dm_sec_slave.c
SRC += dm_sync_ae.c
SRC += event_groups.c
SRC += gap_main.c
SRC += gatt_main.c
SRC += hci_cmd.c
SRC += hci_cmd_ae.c
SRC += hci_cmd_bis.c
SRC += hci_cmd_cis.c
SRC += hci_cmd_cte.c
SRC += hci_cmd_iso.c
SRC += hci_cmd_past.c
SRC += hci_cmd_phy.c
SRC += hci_core.c
SRC += hci_core_ps.c
SRC += hci_drv_apollo3.c
SRC += hci_evt.c
SRC += hci_main.c
SRC += hci_tr.c
SRC += hci_vs_ae.c
SRC += hci_vs_apollo3.c
SRC += heap_2.c
SRC += hid_main.c
SRC += hidapp_main.c
SRC += hrps_main.c
SRC += l2c_coc.c
SRC += l2c_main.c
SRC += l2c_master.c
SRC += l2c_slave.c
SRC += list.c
SRC += port.c
SRC += print.c
SRC += queue.c
SRC += radio_task.c
SRC += rscps_main.c
SRC += rtos.c
SRC += sec_aes.c
SRC += sec_aes_rev.c
SRC += sec_ccm_hci.c
SRC += sec_cmac_hci.c
SRC += sec_ecc.c
SRC += sec_ecc_debug.c
SRC += sec_ecc_hci.c
SRC += sec_main.c
SRC += smp_act.c
SRC += smp_db.c
SRC += smp_main.c
SRC += smp_non.c
SRC += smp_sc_act.c
SRC += smp_sc_main.c
SRC += smpi_act.c
SRC += smpi_sc_act.c
SRC += smpi_sc_sm.c
SRC += smpi_sm.c
SRC += smpr_act.c
SRC += smpr_sc_act.c
SRC += smpr_sc_sm.c
SRC += smpr_sm.c
SRC += svc_alert.c
SRC += svc_amdtp.c
SRC += svc_amotas.c
SRC += svc_amvole.c
SRC += svc_batt.c
SRC += svc_bps.c
SRC += svc_core.c
SRC += svc_cps.c
SRC += svc_cscs.c
SRC += svc_cte.c
SRC += svc_cust.c
SRC += svc_dis.c
SRC += svc_gls.c
SRC += svc_gyro.c
SRC += svc_hid.c
SRC += svc_hrs.c
SRC += svc_hts.c
SRC += svc_ipss.c
SRC += svc_plxs.c
SRC += svc_px.c
SRC += svc_rscs.c
SRC += svc_scpss.c
SRC += svc_temp.c
SRC += svc_throughput.c
SRC += svc_time.c
SRC += svc_uricfg.c
SRC += svc_wdxs.c
SRC += svc_wp.c
SRC += svc_wss.c
SRC += tasks.c
SRC += terminal.c
SRC += timers.c
SRC += uECC.c
SRC += uECC_ll.c
SRC += ui_console.c
SRC += ui_lcd.c
SRC += ui_main.c
SRC += ui_platform.c
SRC += ui_timer.c
SRC += wsf_assert.c
SRC += wsf_buf.c
SRC += wsf_efs.c
SRC += wsf_math.c
SRC += wsf_msg.c
SRC += wsf_os.c
SRC += wsf_queue.c
SRC += wsf_timer.c
SRC += wsf_trace.c
SRC += wstr.c
SRC += startup_gcc.c
#}}}

CSRC = $(filter %.c,$(SRC))
ASRC = $(filter %.s,$(SRC))

OBJS = $(CSRC:%.c=$(BINDIR)/%.o)
OBJS+= $(ASRC:%.s=$(BINDIR)/%.o)

DEPS = $(CSRC:%.c=$(BINDIR)/%.d)
DEPS+= $(ASRC:%.s=$(BINDIR)/%.d)

LIBS = $(SDKROOT)/mcu/apollo3/hal/gcc/bin/libam_hal.a
LIBS+= $(SDKROOT)/boards/apollo3_evb/bsp/gcc/bin/libam_bsp.a
LIBS+= sensirionSGP40/bin/sgp40_apollo3.a

#FLAGS#{{{
CFLAGS = -mthumb -mcpu=$(CPU) -mfpu=$(FPU) -mfloat-abi=$(FABI)
CFLAGS+= -ffunction-sections -fdata-sections -fomit-frame-pointer
CFLAGS+= -MMD -MP -std=c99 -Wall
ifeq ($(HP_RELEASE),)
CFLAGS+= -g -O0
else
CFLAGS+= -O3
endif
CFLAGS+= $(DEFINES)
CFLAGS+= $(INCLUDES)
CFLAGS+= 
#$(info defines = $(CFLAGS))
#$(error STOP)

LFLAGS = -mthumb -mcpu=$(CPU) -mfpu=$(FPU) -mfloat-abi=$(FABI)
LFLAGS+= -nostartfiles -static
LFLAGS+= -Wl,--gc-sections,--entry,Reset_Handler,-Map,$(BINDIR)/$(TARGET)$(BUILDNUM).map
LFLAGS+= -Wl,--start-group -lm -lc -lgcc $(LIBS) -Wl,--end-group
LFLAGS+= 

# Additional user specified CFLAGS
CFLAGS+=$(EXTRA_CFLAGS)

CPFLAGS = -Obinary

ODFLAGS = -S
#}}}
#### Build ####{{{
all: amota

helloPick: directories $(LIBS) $(BINDIR)/$(TARGET)$(BUILDNUM).bin

directories: $(BINDIR)
$(BINDIR):
	$(Q) mkdir -p $@

$(SDKROOT)/mcu/apollo3/hal/gcc/bin/libam_hal.a:
	$(MAKE) -C $(SDKROOT)/mcu/apollo3/hal/gcc

$(SDKROOT)/boards/apollo3_evb/bsp/gcc/bin/libam_bsp.a:
	$(MAKE) -C $(SDKROOT)/boards/apollo3_evb/bsp/gcc

sensirionSGP40/bin/sgp40_apollo3.a:
	$(MAKE) -C sensirionSGP40

$(BINDIR)/%.o: %.c
	$(Q) echo " Compiling $(COMPILERNAME) $<"
	$(Q) $(CC) -c $(CFLAGS) $< -o $@

$(BINDIR)/%.o: %.s
	$(Q) echo " Assembling $(COMPILERNAME) $<"
	$(Q) $(CC) -c $(CFLAGS) $< -o $@

$(BINDIR)/$(TARGET)$(BUILDNUM).axf: $(OBJS) $(LIBS)
	$(Q) echo " Linking $(COMPILERNAME) $@"
	$(Q) $(CC) -Wl,-T,$(LINKER_FILE) -o $@ $(OBJS) $(LFLAGS)
	$(Q) chmod 644 $@

$(BINDIR)/$(TARGET)$(BUILDNUM).bin: $(BINDIR)/$(TARGET)$(BUILDNUM).axf
	$(Q) echo " Copying $(COMPILERNAME) $@..."
	$(Q) $(CP) $(CPFLAGS) $< $@
	$(Q) $(OD) $(ODFLAGS) $< > $(BINDIR)/$(TARGET)$(BUILDNUM).lst
	$(Q) $(SIZE) $(OBJS) $(LIBS) $(BINDIR)/$(TARGET)$(BUILDNUM).axf >$(BINDIR)/$(TARGET)$(BUILDNUM).size
	$(Q) chmod 644 $@

#}}}
#### AMOTA ####{{{

PY := .venv/bin/python3
PIP := .venv/bin/pip
APOLLO3SCRIPT := $(SDKROOT)/tools/apollo3_scripts
APOLLO3AMOTASC := $(SDKROOT)/tools/apollo3_amota/scripts
AMOTADIR := amotaout
#$(info $(BUILDNUM))
#$(error STOP)


amota: helloPick checkmodule amotaout $(BINDIR)/$(TARGET)$(BUILDNUM).bin
	$(Q) cp $(APOLLO3SCRIPT)/keys_info0.py $(APOLLO3SCRIPT)/keys_info.py
	$(PY) $(APOLLO3SCRIPT)/create_cust_image_blob.py --bin $(BINDIR)/$(TARGET)$(BUILDNUM).bin --load-address 0xc000 --magic-num 0xcb -o temp_main_nosecure_ota --version 0x0
	$(PY) $(APOLLO3AMOTASC)/ota_binary_converter.py --appbin temp_main_nosecure_ota.bin -o $(AMOTADIR)/$(TARGET)$(BUILDNUM)
	$(Q) rm -rf temp_main_nosecure_ota.bin

amotaout:
	$(Q) mkdir -p $@
	

checkmodule: .venv
	$(PY) -c "import Crypto" || ($(PIP) install -U pip && $(PY) -m pip install --no-input pycryptodome )

.venv:
	$(Q) python3 -m venv .venv
#}}}
#### UTIL ####{{{

buildnumInc:
	$(Q) awk '{$$3=$$3+1; print}' $(BUILDNUMHEADER) >temp && mv temp $(BUILDNUMHEADER)

buildnumDec:
	$(Q) awk '{$$3=$$3-1; print}' $(BUILDNUMHEADER) >temp && mv temp $(BUILDNUMHEADER)

flash:
	$(Q) echo "$(BINDIR)/$(TARGET).axf"
	$(Q) echo "r" > .flash.jlink
	$(Q) echo "h" >> .flash.jlink
	$(Q) echo "LoadFile  $(BINDIR)/$(TARGET).axf 0x00000000" >> .flash.jlink
	$(Q) echo "VerifyBin $(BINDIR)/$(TARGET).axf 0x00000000" >> .flash.jlink
	$(Q) echo "r" >> .flash.jlink
	$(Q) echo "g" >> .flash.jlink
	$(Q) echo "SWOStart" >> .flash.jlink
	$(Q) echo "exit" >> .flash.jlink
	$(Q) JlinkExe -nogui 1 -Device AMA3B1KK-KQR -If SWD -Speed 4000 -Autoconnect 1 -CommandFile .flash.jlink

adbpush:
	adb push $(AMOTADIR)/$(TARGET)$(BUILDNUM).bin /storage/sdcard0/Download/

tags:
	$(Q) echo "Generating tags under $(SDKROOT) and HelloPick_Apollo3_ver4/"
	$(Q) find ./ \( -path "./archive" -o -path "**/FreeRTOS9" -o -path "**/apollo3p*" -o -path "**/SVD" -o -path "**/examples" -o -path "apollo3_evb_cygnus" -o -path "**/docs" \) -prune -o \
	-type f \( -name '*.c' -o -name '*.h' -o -name 'Makefile' -o -name '*.mk' \) \
	-exec ctags -f tags {} +

archive:
	cd ../ && \
	zip -r .//archive/$$(date +%Y%m%d)_HelloPickFirmware_src_build$(BUILDNUM).zip . -x '*.git*' '*.DS_Store' 'archive/*' '*/.venv/*' '*.swp' '*/tags' '*/.flash.jlink'

swoview:
	JLinkSWOViewerCL -swofreq 3000000 -cpufreq 48236000 -itmmask 0x1 -device AMA3B1KK-KQR
	

clean:
	$(Q) echo "Cleaning..."
	$(Q) $(RM) -rf $(BINDIR) 
	$(Q) $(RM) -rf $(AMOTADIR) 
	$(MAKE) -C sensirionSGP40 $@
#}}}

# Automatically include any generated dependencies
-include $(DEPS)
endif
.PHONY: all clean directories buildnumInc buildnumDec flash tags

#:make -j$(sysctl -n hw.ncpu) -l$(sysctl -n hw.ncpu)
#:terminal bash -c "JLinkSWOViewerCL -swofreq 3000000 -cpufreq 48236000 -itmmask 0x1 -device AMA3B1KK-KQR | tee swo.log"
# vim: set foldmethod=marker commentstring=#%s :
