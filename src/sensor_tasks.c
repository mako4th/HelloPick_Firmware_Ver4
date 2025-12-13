#include "wsf_types.h"
#include "sensor_tasks.h"
#include "adc_vbatt.h"
#include "hello_pick_main.h"
#include "am_util.h"
#include "sgp40_apollo3.h"
#include "sgp40_i2c.h"
#include "rtos.h"
#include "wsf_msg.h"
#include "wsf_trace.h"
#include <string.h>

QueueHandle_t sensorQueue;

void workerTask(void *args)
{
    APP_TRACE_INFO0("== == == == ==workerTask Ready....== == == == ==");

    sensorMsg_t msg;
    while (1)
    {
        if (xQueueReceive(sensorQueue, &msg, portMAX_DELAY) == pdTRUE)
        {
            if (msg.cmd == SNS_MSG_GETMEASUREMENTS)
            {
                hello_adv.dev_info.battery = getBattLevel();
                APP_TRACE_INFO2("battery: %fv %d%%", get_batt(), hello_adv.dev_info.battery);
                uint16_t voc_raw = 0;
                uint32_t voc_index_value = 0;
                sgp40_get_voc_sraw_lowpower(&voc_raw, &voc_index_value);
                hello_adv.voc_raw[0] = (voc_raw >> 8) & 0xFF;
                hello_adv.voc_raw[1] = voc_raw & 0xFF;
                APP_TRACE_INFO2("------- voc raw     0x%x voc raw %d=======", voc_raw, voc_raw);
                APP_TRACE_INFO2("------- adv_raw0 %x raw1 %x", hello_adv.voc_raw[0], hello_adv.voc_raw[1]);
            }
        }
    }
}
