#ifndef CAN_H
#define CAN_H

#include <FU68xx_6.h>

/* IDE */
#define IDE_STD                 0
#define IDE_EXT                 0x80
/* RTR */
#define RTR_DAT                 0
#define RTR_RMT                 0x40

typedef struct
{
    uint8 CAN_IDE;
    uint8 CAN_RTR;
    uint8 CAN_DLC;
    uint32 EXTID;                                                           //ID号范围：0-0x1FFFFFFF
    uint32 STDID;                                                           //ID号范围：0-0x7FF
    uint8 Data0;
    uint8 Data1;
    uint8 Data2;
    uint8 Data3;
    uint8 Data4;
    uint8 Data5;
    uint8 Data6;
    uint8 Data7;
}CAN_Data;

extern CAN_Data xdata Can; 
extern void CAN_Read(void);
extern uint8 CAN_Send(void);
extern uint8 can_hw_send(uint32 can_id, const uint8 *dat, uint8 len);
extern void CAN_Init(void);
extern void CAN_MsgParse(void);

#endif