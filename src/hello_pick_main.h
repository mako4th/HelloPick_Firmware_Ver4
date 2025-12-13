#ifndef HELLO_SENSOR_H
#define HELLO_SENSOR_H

#include "wsf_os.h"

/**************************************************************************************************
  Advertising Data
**************************************************************************************************/
// アドバタイジングパケット
typedef union{
	struct{
		uint8_t battery :6; 
		uint8_t voc_res :2;
	};
	uint8_t raw;
} __attribute__((packed)) devinfo_t;

typedef struct
{
  // --- Flags フィールド ---
  uint8_t len1;  // 0x02
  uint8_t type1; // 0x01
  uint8_t flags; // 0x06

  // Complete List of 16-bit Service UUIDs
  uint8_t len2;
  uint8_t type2;
  uint8_t suuid[2];

  // --- Manufacturer Specific Data フィールド ---
  uint8_t len3;     // 0x18（内容に応じて調整）
  uint8_t type3;    // 0xFF
  uint8_t cid[2];   // Company ID：LSB, MSB（リトルエンディアン）
  uint8_t tag_type; // タグ種別0x01 
  devinfo_t dev_info;  // .voc_res:上位2bit 判定結果 .battery:下位6bit 2%刻みで(0~50)*2%
  uint8_t lot_no[2];
  uint8_t build_no[2];
  uint8_t serial_no[8];
  uint8_t voc_raw[2];
  uint8_t check_digit; // チェックディジット
} __attribute__((packed)) adv_payload_hellotag_t;

// scanRspパケット
typedef struct
{
  uint8_t len1;
  uint8_t type1;
  char local_name[9]; // = "HelloPick"（ASCII 文字列、終端なし）
} __attribute__((packed)) scan_rsp_t;

extern adv_payload_hellotag_t hello_adv;
void HelloPickStart(void);
void HelloPickHandlerInit(wsfHandlerId_t handlerId);
void HelloPickHandler(wsfEventMask_t event, wsfMsgHdr_t *pMsg);

#endif
