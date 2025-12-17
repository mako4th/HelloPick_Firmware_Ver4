// ****************************************************************************
//
//  amota_main.c
//! @file
//!
//! @brief Ambiq Micro's demonstration of AMOTA service.
//!
//! @{
//
// ****************************************************************************

//*****************************************************************************
//
// Copyright (c) 2024, Ambiq Micro, Inc.
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
// this list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright
// notice, this list of conditions and the following disclaimer in the
// documentation and/or other materials provided with the distribution.
//
// 3. Neither the name of the copyright holder nor the names of its
// contributors may be used to endorse or promote products derived from this
// software without specific prior written permission.
//
// Third party software included in this distribution is subject to the
// additional license terms as defined in the /docs/licenses directory.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
// This is part of revision release_sdk_3_2_0-dd5f40c14b of the AmbiqSuite Development Package.
//
//*****************************************************************************

#include <string.h>
#include "wsf_types.h"
#include "bstream.h"
#include "wsf_msg.h"
#include "wsf_trace.h"
#include "hci_api.h"
#include "dm_api.h"
#include "att_api.h"
#include "app_api.h"
#include "app_db.h"
#include "app_ui.h"
#include "app_hw.h"
#include "svc_ch.h"
#include "svc_core.h"
#include "svc_dis.h"
#include "amota_api.h"
#include "amotas_api.h"
#include "svc_amotas.h"
#include "gatt_api.h"
#include "atts_main.h"
#include "build_number.h"
#include "am_mcu_apollo.h"
#include "adc_vbatt.h"
#include "sgp40_apollo3.h"
#include "sgp40_i2c.h"
#include "hello_pick_main.h"
#include "sensor_tasks.h"
#include "am_bsp.h"
#include "am_util.h"
#include "FreeRTOS.h"
#include "task.h"

/**************************************************************************************************
  Macros
**************************************************************************************************/

/**************************************************************************************************
  Data Types
**************************************************************************************************/

/**************************************************************************************************
  Configurable Parameters
**************************************************************************************************/

/*! configurable parameters for advertising */
static const appAdvCfg_t helloPickAdvCfg =
    {
        {800, 800, 800}, /*! Advertising durations in ms */
        {480, 480, 480}  /*! Advertising intervals in 0.625 ms units 240=150ms 480=300ms　800=500ms*/
};

/**************************************************************************************************
  Advertising Data
**************************************************************************************************/
// アドバタイジングパケット
adv_payload_hellotag_t hello_adv = {
    .len1 = 0x02,
    .type1 = 0x01,
    .flags = 0x06,
    .len2 = 0x03,
    .type2 = 0x03,
    .suuid = {0x1A, 0x18}, // 0x181A Environmental Sensing Service
    .len3 = 0x14,
    .type3 = 0xFF,
    .cid = {0x9B, 0x0D}, // 0x0D9B リトルエンディアン（LSB, MSB）
    .tag_type = 0x01,
	.dev_info.voc_res = 0b11,//2bit
	.dev_info.battery = 50, //6bit
    .lot_no = {0x00, 0x01},
    .build_no = {0x00, 0x01},
    .serial_no = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    .voc_raw = {0x00, 0x00},
    .check_digit = 0x00};

scan_rsp_t scanRsp = {
    .len1 = 0x0A,
    .type1 = 0x09,
    .local_name = "HelloPick"};

/**************************************************************************************************
  Global Variables
**************************************************************************************************/
static wsfTimer_t AdvWaitTimer;
static wsfHandlerId_t AdvWaitTimerHandler;

static wsfTimer_t AdvStartTimer;
static wsfHandlerId_t AdvStartTimerHandler;
static wsfTimer_t AdvUpdateTimer;
static wsfHandlerId_t AdvUpdateTimerHandler;

static wsfTimer_t resetTimer;
static wsfHandlerId_t resetTimerHandler;

// 起動、再起動後の待機時間中はVocRawを0xFFFFFにして送信(周辺環境安定待ち)
static wsfTimerTicks_t advWaitTimeSec = 120; //2分間待機
//static wsfTimerTicks_t advDurationMs = 800;
// アドバタイズ間隔(送信->320ms->送信停止->5000ms->送信)
static wsfTimerTicks_t advRestartDelayMs = 5000;
// センサーqueueのアドバタイズ停止からの待ち時間
static wsfTimerTicks_t sensorQueueDelayMs = 2500;
// 定期リセット間隔
//static wsfTimerTicks_t resetIntervalSec = 3600 * 24; // 1h=3600sec
// アドレス変更用
uint8_t bd_addr[6];
//起動再起動時待機フラグ
static bool vocRawFixed = true;

/*! WSF handler ID */
wsfHandlerId_t HelloPickHandlerId;

uint8_t crc8(uint8_t *data, size_t len)
{
    uint8_t crc = 0x00;
    for (size_t i = 0; i < len; ++i)
    {
        crc ^= data[i];
        for (int j = 0; j < 8; ++j)
        {
            crc = (crc & 0x80) ? (crc << 1) ^ 0x07 : (crc << 1);
        }
    }
    return crc;
}
static void AdvWaitTimerCallback(wsfEventMask_t event, wsfMsgHdr_t *pMsg){
    APP_TRACE_INFO0("==========vocRaw rock 0xFFFF=========");
	vocRawFixed = false;
}
static void resetTimerCallback(wsfEventMask_t event, wsfMsgHdr_t *pMsg)
{
    APP_TRACE_INFO0("==========Reset=========");
    MCUCTRL->SCRATCH1 = 0;
    AppAdvStop();
    am_hal_reset_control(AM_HAL_RESET_CONTROL_TPIU_RESET, 0);
    am_hal_reset_control(AM_HAL_RESET_CONTROL_STATUSCLEAR, 0);
    am_hal_reset_control(AM_HAL_RESET_CONTROL_SWPOR, 0);
    // am_hal_reset_control(AM_HAL_RESET_CONTROL_SWPOI, 0);
}
static void sendSensorQueue(wsfEventMask_t event, wsfMsgHdr_t *pMsg)
{
    sensorMsg_t msg = {.cmd = SNS_MSG_GETMEASUREMENTS};
    xQueueSend(sensorQueue, &msg, 0);
    APP_TRACE_INFO0("sendSensorQueue---------");
}
static void make_nrpa()
{
    for (int i = 0; i < 6; i++)
    {
        bd_addr[i] = (uint8_t)(rand() & 0xFF);
    }
    bd_addr[5] &= 0x3F;
}
static void AppAdvStartTimerCallback(wsfEventMask_t event, wsfMsgHdr_t *pMsg)
{
    if (hello_adv.suuid[0] == 0x1A)
    {
        hello_adv.suuid[0] = 0x00;
        hello_adv.suuid[1] = 0x18;
    }
    else
    {
        hello_adv.suuid[0] = 0x1A;
        hello_adv.suuid[1] = 0x18;
    }
    APP_TRACE_INFO2("==========suuid = %x%x", hello_adv.suuid[1], hello_adv.suuid[0]);
    // SecRand(bd_addr, 6);
    // DM_RAND_ADDR_SET(bd_addr, DM_RAND_ADDR_NONRESOLV);
    make_nrpa();
    DmDevSetRandAddr(bd_addr);
    DmAdvSetAddrType(DM_ADDR_RANDOM);

    /* set advertising and scan response data for discoverable mode */
    if(vocRawFixed){
		hello_adv.voc_raw[0] = 0xFF;
		hello_adv.voc_raw[1] = 0xFF;
    }
	adv_payload_hellotag_t adv_snapshot;
	taskENTER_CRITICAL();
	adv_snapshot = hello_adv;
	taskEXIT_CRITICAL();
    adv_snapshot.check_digit = crc8((uint8_t *)&adv_snapshot, sizeof(adv_snapshot) - 1);
    AppAdvSetData(APP_ADV_DATA_DISCOVERABLE, sizeof(adv_snapshot), (uint8_t *)&adv_snapshot);
    AppAdvSetData(APP_SCAN_DATA_DISCOVERABLE, sizeof(scanRsp), (uint8_t *)&scanRsp);
    AppSetAdvType(DM_ADV_SCAN_UNDIRECT);

    /* start advertising; automatically set connectable/discoverable mode and bondable mode */
    AppAdvStart(APP_MODE_DISCOVERABLE);
}

/*************************************************************************************************/
/*!
 *  \fn     helloPickDmCback
 *
 *  \brief  Application DM callback.
 *
 *  \param  pDmEvt  DM callback event
 *
 *  \return None.
 */
/*************************************************************************************************/
static void helloPickDmCback(dmEvt_t *pDmEvt)
{
    dmEvt_t *pMsg;

    if ((pMsg = WsfMsgAlloc(sizeof(dmEvt_t))) != NULL)
    {
        memcpy(pMsg, pDmEvt, sizeof(dmEvt_t));
        WsfMsgSend(HelloPickHandlerId, pMsg);
    }
}

/*************************************************************************************************/
/*!
 *  \fn     HelloPickSetup
 *
 *  \brief  Set up advertising and other procedures that need to be performed after
 *          device reset.
 *
 *  \param  pMsg    Pointer to message.
 *
 *  \return None.
 */
/*************************************************************************************************/
static void HelloPickSetup()
{
    APP_TRACE_INFO0(">>>>>>>>>>HelloPickSetup()");
    // 電圧取得初期化
    vbatt_init();
    // SGP40初期化
    float interval = (float)advRestartDelayMs / 1000;
    APP_TRACE_INFO1("========Init sgp40 vocIndex interval %f", interval);
    sgp40setup(interval);

    // ビルドナンバー
    uint16_t buildnum = BUILD_NUMBER;
    hello_adv.build_no[0] = (buildnum >> 8) & 0xFF;
    hello_adv.build_no[1] = buildnum & 0xFF;
    APP_TRACE_INFO1("HelloPick Farmware build %d", buildnum);

    // デバイスIDをApollo3チップIDから生成
    am_hal_mcuctrl_device_t device;
    am_hal_mcuctrl_info_get(AM_HAL_MCUCTRL_INFO_DEVICEID, &device);

    uint32_t chipIDs[2] = {device.ui32ChipID0, device.ui32ChipID1};
    memcpy(hello_adv.serial_no, chipIDs, 8);

    // デバイスIDの確認用出力
    APP_TRACE_INFO0("\nHelloPick serial_no :");
    for (int i = 0; i < 8; i++)
    {
        APP_TRACE_INFO1("%x ", hello_adv.serial_no[i]);
    }

    APP_TRACE_INFO2("\nApollo3 deviceID %x %x", chipIDs[0], chipIDs[1]);

    // 起動時にVocRawを0xFFFFに固定するタイマー DM_RESET_CMPL_INDでタイマースタート
    AdvWaitTimerHandler = WsfOsSetNextHandler(AdvWaitTimerCallback);
    AdvWaitTimer.handlerId = AdvWaitTimerHandler;

/***********************************************************************/
// 通常動作時のタイマー DM_ADV_STOP_IND でタイマースタート
// アドバタイズ開始
    AdvStartTimerHandler = WsfOsSetNextHandler(AppAdvStartTimerCallback);
    AdvStartTimer.handlerId = AdvStartTimerHandler;
// アドバタイズデータ更新
    AdvUpdateTimerHandler = WsfOsSetNextHandler(sendSensorQueue);
    AdvUpdateTimer.handlerId = AdvUpdateTimerHandler;
/***********************************************************************/



    // 定期リセットタイマー
    //resetTimerHandler = WsfOsSetNextHandler(resetTimerCallback);
    //resetTimer.handlerId = resetTimerHandler;
    //WsfTimerStartSec(&resetTimer, resetIntervalSec);

    // センサー駆動タスクとメッセージキュー
    sensorQueue = xQueueCreate(8, sizeof(sensorMsg_t));
    xTaskCreate(workerTask, "pSensorTask", 1024, NULL, tskIDLE_PRIORITY + 1, NULL);
}

/*************************************************************************************************/
/*!
 *  \fn     helloPickProcMsg
 *
 *  \brief  Process messages from the event handler.
 *
 *  \param  pMsg    Pointer to message.
 *
 *  \return None.
 */
/*************************************************************************************************/
void firstAdvTask()
{
    // 初回のアドバタイズ開始、以降アドバタイズ停止時のコールバックを起点に処理
    wsfEventMask_t mask = 0xFF;
    wsfMsgHdr_t hdr = {0xFFFF,0xFF,0xFF};
    AppAdvStartTimerCallback(mask, &hdr);
    vTaskSuspend(NULL);
}
static void helloPickProcMsg(wsfMsgHdr_t *pMsg)
{
    switch (pMsg->event)
    {
    case DM_RESET_CMPL_IND:
        APP_TRACE_INFO0("DM_RESET_CMP_IND");
        APP_TRACE_INFO0("VocRaw=0xFFFF ===================");
    	WsfTimerStartSec(&AdvWaitTimer, advWaitTimeSec );
		xTaskCreate(firstAdvTask, "pcfirstADVTask", 512, NULL, 3, NULL);
        break;

    case DM_ADV_START_IND:
        break;

    case DM_ADV_STOP_IND:
        APP_TRACE_INFO0("DM_ADV_STOP_IND");
        WsfTimerStartMs(&AdvUpdateTimer, sensorQueueDelayMs);
        WsfTimerStartMs(&AdvStartTimer, advRestartDelayMs);
        break;

    default:
        break;
    }
}

/*************************************************************************************************/
/*!
 *  \fn     HelloPickHandlerInit
 *
 *  \brief  Application handler init function called during system initialization.
 *
 *  \param  handlerID  WSF handler ID.
 *
 *  \return None.
 */
/*************************************************************************************************/
void HelloPickHandlerInit(wsfHandlerId_t handlerId)
{
    /* store handler ID */
    HelloPickHandlerId = handlerId;

    /* Set configuration pointers */
    pAppAdvCfg = (appAdvCfg_t *)&helloPickAdvCfg;
}

/*************************************************************************************************/
/*!
 *  \fn     HelloPickHandler
 *
 *  \brief  WSF event handler for application.
 *
 *  \param  event   WSF event mask.
 *  \param  pMsg    WSF message.
 *
 *  \return None.
 */
/*************************************************************************************************/
void HelloPickHandler(wsfEventMask_t event, wsfMsgHdr_t *pMsg)
{
    if (pMsg != NULL)
    {
        /* perform profile and user interface-related operations */
        helloPickProcMsg(pMsg);
    }
}

/*************************************************************************************************/
/*!
 *  \fn     HelloPickStart
 *
 *  \brief  Start the application.
 *
 *  \return None.
 */
/*************************************************************************************************/
void HelloPickStart(void)
{
    HelloPickSetup();
    /* Register for stack callbacks */
    DmRegister(helloPickDmCback);
    /* Reset the device */
    APP_TRACE_INFO0(">>>>>>>>>>HelloPickDmDevReset");
    DmDevReset();
}
