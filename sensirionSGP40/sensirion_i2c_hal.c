/*
 * Copyright (c) 2018, Sensirion AG
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of Sensirion AG nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "sensirion_i2c_hal.h"
#include "am_bsp.h"
#include "am_bsp_pins.h"
#include "am_hal_iom.h"
#include "am_mcu_apollo.h"
#include "am_util_delay.h"
#include "am_util.h"
#include "sensirion_common.h"
#include "sensirion_config.h"
#include <string.h>
#include "sgp40_apollo3.h"
// 使用する I2C バス
#define I2C_MODULE 0 // Apollo3 の IOM0 を使用

/*
 * INSTRUCTIONS
 * ============
 *
 * Implement all functions where they are marked as IMPLEMENT.
 * Follow the function specification in the comments.
 */

/**
 * Select the current i2c bus by index.
 * All following i2c operations will be directed at that bus.
 *
 * THE IMPLEMENTATION IS OPTIONAL ON SINGLE-BUS SETUPS (all sensors on the
 * same bus)
 *
 * @param bus_idx   Bus index to select
 * @returns         0 on success, an error code otherwise
 */
int16_t sensirion_i2c_hal_select_bus(uint8_t bus_idx)
{
    if (bus_idx >= AM_REG_IOM_NUM_MODULES)
    {
        return -1;
    }

    if (g_IOMHandle != NULL)
    {
        return 0;
    }

    am_hal_iom_config_t i2c_config = {.eInterfaceMode = AM_HAL_IOM_I2C_MODE,
                                      .ui32ClockFreq = AM_HAL_IOM_400KHZ};

    am_hal_iom_initialize(bus_idx, &g_IOMHandle);
    am_hal_iom_power_ctrl(g_IOMHandle, AM_HAL_SYSCTRL_WAKE, false);
    am_hal_iom_configure(g_IOMHandle, &i2c_config);

    return 0;
}

/**
 * Initialize all hard- and software components that are needed for the I2C
 * communication.
 */
void sensirion_i2c_hal_init(void)
{
    am_hal_iom_config_t i2c_config = {
        .eInterfaceMode = AM_HAL_IOM_I2C_MODE,
        .ui32ClockFreq = AM_HAL_IOM_400KHZ // 400kHz I²C
    };

    am_hal_iom_initialize(I2C_MODULE, &g_IOMHandle);
    am_hal_iom_power_ctrl(g_IOMHandle, AM_HAL_SYSCTRL_WAKE, false);
    am_hal_iom_configure(g_IOMHandle, &i2c_config);

    // SCL/SDA ピンの設定
    am_hal_gpio_pinconfig(AM_BSP_GPIO_IOM0_SCL, g_AM_BSP_GPIO_IOM0_SCL);
    am_hal_gpio_pinconfig(AM_BSP_GPIO_IOM0_SDA, g_AM_BSP_GPIO_IOM0_SDA);
}

/**
 * Release all resources initialized by sensirion_i2c_hal_init().
 */
void sensirion_i2c_hal_free(void)
{
    if (g_IOMHandle)
    {
        am_hal_iom_disable(g_IOMHandle);
        am_hal_iom_uninitialize(g_IOMHandle);
        g_IOMHandle = NULL;
    }

    am_hal_gpio_pinconfig(AM_BSP_GPIO_IOM0_SCL, g_AM_HAL_GPIO_DISABLE);
    am_hal_gpio_pinconfig(AM_BSP_GPIO_IOM0_SDA, g_AM_HAL_GPIO_DISABLE);
}

/**
 * Execute one read transaction on the I2C bus, reading a given number of bytes.
 * If the device does not acknowledge the read command, an error shall be
 * returned.
 *
 * @param address 7-bit I2C address to read from
 * @param data    pointer to the buffer where the data is to be stored
 * @param count   number of bytes to read from I2C and store in the buffer
 * @returns 0 on success, error code otherwise
 */
int8_t sensirion_i2c_hal_read(uint8_t address, uint8_t *data, uint16_t count)
{
    am_hal_iom_transfer_t transaction = {.ui32InstrLen = 0,
                                         .ui32NumBytes = count,
                                         .pui32RxBuffer = (uint32_t *)data,
                                         .uPeerInfo.ui32I2CDevAddr = address,
                                         .eDirection = AM_HAL_IOM_RX};

    return am_hal_iom_blocking_transfer(g_IOMHandle, &transaction);
}

/**
 * Execute one write transaction on the I2C bus, sending a given number of
 * bytes. The bytes in the supplied buffer must be sent to the given address. If
 * the slave device does not acknowledge any of the bytes, an error shall be
 * returned.
 *
 * @param address 7-bit I2C address to write to
 * @param data    pointer to the buffer containing the data to write
 * @param count   number of bytes to read from the buffer and send over I2C
 * @returns 0 on success, error code otherwise
 */
int8_t sensirion_i2c_hal_write(uint8_t address, const uint8_t *data, uint16_t count)
{
    // Ensure the buffer is 32-bit aligned
    static uint32_t aligned_buffer[16] __attribute__((aligned(4)));
    if (count > sizeof(aligned_buffer))
    {
        am_util_stdio_printf("Error: Data size exceeds buffer capacity\n");
        return -1;
    }

    // Copy data to the aligned buffer
    memcpy(aligned_buffer, data, count);

    // Ensure g_sIOMHandle is valid
    if (g_IOMHandle == NULL)
    {
        am_util_stdio_printf("Error: g_sIOMHandle is NULL\n");
        return -1;
    }

    // Configure the transaction
    am_hal_iom_transfer_t transaction = {
        .uPeerInfo.ui32I2CDevAddr = address,
        .ui32InstrLen = 0,
        .ui32NumBytes = count,
        .eDirection = AM_HAL_IOM_TX,
        .pui32TxBuffer = aligned_buffer,
        .bContinue = false,
    };

    // Debugging: Print transaction details
    // am_util_stdio_printf("Transaction details: ");
    // am_util_stdio_printf("Address: 0x%02X ", transaction.uPeerInfo.ui32I2CDevAddr);
    // am_util_stdio_printf("NumBytes: %d ", transaction.ui32NumBytes);
    // am_util_stdio_printf("Direction: %d ", transaction.eDirection);

    // Perform the transfer
    int8_t res = am_hal_iom_blocking_transfer(g_IOMHandle, &transaction);
    if (res != AM_HAL_STATUS_SUCCESS)
    {
        am_util_stdio_printf("I2C write failed with status: 0x%08X\n", res);
    }
    return res;
}

/**
 * Sleep for a given number of microseconds. The function should delay the
 * execution for at least the given time, but may also sleep longer.
 *
 * Despite the unit, a <10 millisecond precision is sufficient.
 *
 * @param useconds the sleep time in microseconds
 */
void sensirion_i2c_hal_sleep_usec(uint32_t useconds)
{
    am_util_delay_us(useconds);
}
