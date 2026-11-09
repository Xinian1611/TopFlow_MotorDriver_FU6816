#ifndef CAN_COMPAT_H
#define CAN_COMPAT_H

#include <FU68xx_6.h>

/* boot 工程不含 RingBuffer.h，这里补 stdint 别名；
   （APP 工程里同名 typedef 由 RingBuffer.h 提供，两个工程互不影响）*/
typedef unsigned char  uint8_t;
typedef unsigned short uint16_t;
typedef unsigned long  uint32_t;

/* 让领导代码里的 struct CANTxStc / MsgFlag.bitf / MsgConfig.bitf 原样能编 */
struct MSG_STATUS_FLAG_BITS {
	uint16_t SendEnableFlag:1; 
	uint16_t SendReadyFlag:1; 
	uint16_t SendFinishFlag:1;
	uint16_t ReciveEnableFlag:1; 
	uint16_t ReciveReadyFlag:1; 
	uint16_t ReciveFinishFlag:1;
	uint16_t rsvd:10;
};
union MSG_STATUS_FLAG { uint16_t  all; struct MSG_STATUS_FLAG_BITS bitf; };

union CAN_IDENTIFY_ID { uint32_t all; };

struct CAN_MSG_CONFIG_BITS {
	uint16_t FilterID:8; uint16_t IdentifyType:2; uint16_t DataType:2; uint16_t rsvd:4;
};
union CAN_MSG_CONFIG { uint16_t all; struct CAN_MSG_CONFIG_BITS bitf; };

struct CANTxStc {
	union CAN_IDENTIFY_ID IdentifyID;
	union CAN_MSG_CONFIG   MsgConfig;
	union MSG_STATUS_FLAG  MsgFlag;
	uint8_t MsgBoxNum;
	uint8_t Buff[8];                 /* 原来是 uint16_t[8]，这里用 8 位够用 */
};
struct CANRxStc { uint32_t IdentifyID; };

/* 收帧结构 */
typedef struct { uint32_t ExtId; uint8_t DLC; uint8_t Data[8]; uint8_t valid; } CanRxMsg;
extern CanRxMsg RxMessage[1];

#define CPDD_CANTxStatus_NoMailBox  0
#define CPDD_CANType_Extended       1
#define CPDD_CANDataType_FrameData  0
#define CAN_Enable                  1

/* 由 bsp_can.c 提供 */
extern void CANSendMsgCheck(uint8_t index,struct CANTxStc *Data,uint16_t Length);

#endif
