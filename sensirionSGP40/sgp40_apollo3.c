#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "am_mcu_apollo.h"
#include "am_bsp.h"
#include "am_util.h"
#include "sgp40_i2c.h"
#include "sensirion_i2c_hal.h"
#include "sensirion_gas_index_algorithm.h"

#define I2C_MODULE 1  // IOM0 を使用
#define I2C_SCL_PIN 8 // SCL (PAD5)
#define I2C_SDA_PIN 9 // SDA (PAD6)
#define SPG40_I2C_ADDRESS 0x59

void *g_IOMHandle = NULL;
GasIndexAlgorithmParams voc_params;

void voc_led_display(uint16_t voc_index)
{
    uint8_t led_pattern = 0;

    if (voc_index > 200)
        led_pattern = 0x1F; // 11111
    else if (voc_index > 150)
        led_pattern = 0x0F; // 01111
    else if (voc_index > 100)
        led_pattern = 0x07; // 00111
    else if (voc_index > 50)
        led_pattern = 0x03; // 00011
    else if (voc_index > 0)
        led_pattern = 0x01; // 00001
    else
        led_pattern = 0x00; // 全OFF

    am_devices_led_array_out(am_bsp_psLEDs, AM_BSP_NUM_LEDS, led_pattern);
}

int sgp40PinEnable(void)
{
    am_hal_iom_config_t i2cConfig = {
        .eInterfaceMode = AM_HAL_IOM_I2C_MODE,
        .ui32ClockFreq = AM_HAL_IOM_100KHZ, // 100kHz
    };

    uint32_t status = 0;
    status = am_hal_iom_initialize(I2C_MODULE, &g_IOMHandle);
    am_util_stdio_printf("am_hal_iom_initialize 0x%08X\n", status);

    status = am_hal_iom_power_ctrl(g_IOMHandle, AM_HAL_SYSCTRL_WAKE, false);
    am_util_stdio_printf("am_hal_iom_power_ctrl 0x%08X\n", status);

    status = am_hal_iom_configure(g_IOMHandle, &i2cConfig);
    am_util_stdio_printf("am_hal_iom_configure 0x%08X\n", status);
    status = am_hal_iom_enable(g_IOMHandle);
    am_util_stdio_printf("am_hal_iom_enable 0x%08X\n", status);
    am_bsp_iom_pins_enable(I2C_MODULE, AM_HAL_IOM_I2C_MODE);

    status = am_hal_gpio_pinconfig(I2C_SCL_PIN, g_AM_BSP_GPIO_IOM1_SCL);
    am_util_stdio_printf("am_hal_gpio_pinconfig 0x%08X\n", status);
    status = am_hal_gpio_pinconfig(I2C_SDA_PIN, g_AM_BSP_GPIO_IOM1_SDA);
    am_util_stdio_printf("am_hal_gpio_pinconfig 0x%08X\n", status);
    am_util_delay_ms(30);

    return 0;
}
int sgp40PinDisable(void)
{
    sensirion_i2c_hal_free();
    am_util_delay_ms(30);

    return 0;
}
int sgp40setup(float sampling_interval)
{
    sgp40PinEnable();

    uint32_t txData = 0x00; // dummy

    am_hal_iom_transfer_t xfer = {
        .uPeerInfo.ui32I2CDevAddr = SPG40_I2C_ADDRESS,
        .ui32InstrLen = 0,
        .ui32NumBytes = 1,
        .pui32TxBuffer = &txData,
        .eDirection = AM_HAL_IOM_TX,
        .bContinue = false,
        .ui8Priority = 1,
        .ui32PauseCondition = 0,
        .ui32StatusSetClr = 0};

    uint32_t status = am_hal_iom_blocking_transfer(g_IOMHandle, &xfer);
    am_util_delay_ms(30);
    if (status == AM_HAL_STATUS_SUCCESS)
    {
        am_util_stdio_printf("Device found at: 0x%02X\n", SPG40_I2C_ADDRESS);
        am_util_delay_ms(30);
    }
    else
    {
        am_util_stdio_printf("No ACK from 0x%2X  status 0x%08X\n", SPG40_I2C_ADDRESS, status);
    }

    // initialize gas index parameters
    GasIndexAlgorithm_init_with_sampling_interval(&voc_params, GasIndexAlgorithm_ALGORITHM_TYPE_VOC, sampling_interval);

    // シリアルナンバーを取得して表示
    uint16_t serial_number[3];
    uint8_t serial_number_size = 3;
    uint16_t error = sgp40_get_serial_number(serial_number, serial_number_size);
    if (error)
    {
        am_util_stdio_printf("Error executing sgp40_get_serial_number(): %i\n", error);
    }
    else
    {
        am_util_stdio_printf("serial: 0x%04x%04x%04x\n", serial_number[0], serial_number[1], serial_number[2]);
        am_util_stdio_printf("\n");
    }
    return 0;
}

uint16_t sgp40_get_voc_sraw(uint16_t *raw, uint32_t *index)
{
    uint16_t default_rh = 0x8000;
    uint16_t default_t = 0x6666;
    uint16_t sraw_voc = 0;
    int32_t voc_index_value = 0;
    int16_t error = sgp40_measure_raw_signal(default_rh, default_t, &sraw_voc);
    if (error)
    {
        am_util_stdio_printf("Error executing sgp40_measure_raw_signal(): "
                             "%li\n",
                             error);
    }
    else
    {
        GasIndexAlgorithm_process(&voc_params, sraw_voc, &voc_index_value);
        am_util_delay_ms(30);
    }
    *raw = sraw_voc;
    *index = voc_index_value;
    return error;
}

uint16_t sgp40_get_voc_sraw_lowpower(uint16_t *raw, uint32_t *index)
{
    int16_t error = 0;
    uint16_t default_rh = 0x8000;
    uint16_t default_t = 0x6666;
    uint16_t sraw_voc = 0;
    int32_t voc_index_value = 0;
    sgp40PinEnable();
    error = sgp40_measure_raw_signal(default_rh, default_t, &sraw_voc);
    sensirion_i2c_hal_sleep_usec(170000);
    error = sgp40_measure_raw_signal(default_rh, default_t, &sraw_voc);
    sgp40_turn_heater_off();
    sgp40PinDisable();
    // GasIndexAlgorithm_process(&voc_params, sraw_voc, &voc_index_value);
    *raw = sraw_voc;
    *index = voc_index_value;
    return error;
}
