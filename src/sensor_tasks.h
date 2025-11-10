#ifndef HELLO_SENSOR_TASK
#define HELLO_SENSOR_TASK

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
typedef enum
{
    SNS_MSG_GETMEASUREMENTS
} sensorCMD;

// センサー用queueHandle
extern QueueHandle_t sensorQueue;

typedef struct
{
    uint8_t cmd;
    uint32_t value;
} sensorMsg_t;

void workerTask(void *args);

#endif