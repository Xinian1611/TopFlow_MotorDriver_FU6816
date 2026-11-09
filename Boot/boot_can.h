#ifndef _BOOT_CAN_H_
#define _BOOT_CAN_H_

#include <FU68xx_6.h>
#include "can_compat.h"          /* 提供 CanRxMsg / CANTxStc / RxMessage */
#include "can_bootloader.h" 

void CAN_Init(void);
void CAN_Read(void);
void CAN_detect(void);
void CANSendMsgCheck(uint8_t index,struct CANTxStc *Data,uint16_t Length);

// struct CANData
// {
// 	uint8 CAN_IDE:1;
// 	uint8 CAN_RTR:1;
// 	uint8 CAN_DLC:4;
// 	uint32 EXTID; // ID号范围：0-0x1FFFFFFF
// 	uint32 STDID; // ID号范围：0-0x7FF
// 	uint8 Data0;
// 	uint8 Data1;
// 	uint8 Data2;
// 	uint8 Data3;
// 	uint8 Data4;
// 	uint8 Data5;
// 	uint8 Data6;
// 	uint8 Data7;
// };

// typedef struct
// {
// 	uint8 CAN_IDE :1;
// 	uint8 CAN_RTR :1;
// 	uint8 CAN_DLC :4;
// 	uint32 CANID; // ID号范围：0-0x1FFFFFFF
// 	uint8 Data[8];
// }CanRxMsg;


// extern uint8 Erase_Flage;
// void CAN_Init(void);
// void CAN_Read(void);
// void CAN_Send(void);

#endif	//_BOOT_CAN_H_