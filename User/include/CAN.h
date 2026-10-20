#ifndef CAN_H
#define CAN_H

#include <FU68xx_6.h>
#include <definition.h>
#include <Customer.h>              /* 取 DEBUG_UART_OR_CAN */

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
extern uint8 CAN_Send(void);                                            /*  void → uint8（带超时）*/
extern uint8 can_hw_send(uint32 can_id, const uint8 *dat, uint8 len);   /*  新增 */
extern void CAN_Init(void);
extern void CAN_MsgParse(void);                                         /*  新增 */

/* ==================== 【调试用】CAN 观测变量 ====================
 * 纪律1：只在 CAN 中断 / 1ms 任务里累加，printf 一律放主循环
 *        （printf 是阻塞的，放中断里会拉长 1ms 节拍甚至触发看门狗）
 * 纪律2：全部显式 xdata —— 当前 data=111.3/128、xdata=3706/3830，都很紧
 */
extern volatile uint16 xdata g_can_rxIntCnt;   /* RXIF 中断次数 = 硬件真收到几帧 */
extern volatile uint16 xdata g_can_popCnt;     /* CAN_MsgParse 出队帧数 */
extern volatile uint16 xdata g_can_txCnt;      /* CAN_Send 调用次数 */
extern volatile uint16 xdata g_can_txToCnt;    /* CAN_Send 等 TXING 超时次数 */
extern volatile uint16 xdata g_can_errCnt;     /* ERRIF 次数 */
extern volatile uint16 xdata g_can_ovifCnt;    /* OVIF 接收溢出次数 */
extern volatile uint16 xdata g_can_arbCnt;     /* ARBIF 仲裁丢失次数 */
extern volatile uint8  xdata g_can_cr0;        /* CAN_Init 末尾回读的 CAN_CR0 */
extern volatile uint8  xdata g_can_srAfterTx;  /* 每次发送后的 CAN_SR */
extern volatile uint8  xdata g_can_ifrLast;    /* CAN 中断入口的 CAN_IFR 快照 */
extern volatile uint16 xdata g_can_busLowCnt;  /* 1ms 任务采样 CANRX(P0.1) 显性次数：
                                                  只算本机自己的帧约 11/s；明显更快 = 主机在发 */
extern CAN_Data xdata CanMsg;                  /* 最近出队帧（打印用）*/
extern void CAN_DebugPrint(void);

/* ==================== 【调试用】CAN 上电自检 ====================
 * 在 main() 里 HardwareInit()+DebugSet() 之后调用一次（只在上电时跑一次）。
 * 它把两件事分开：
 *   ① 引脚实际电平：CAN 空闲时 CANTX(P0.0)/CANRX(P0.1) 都应是隐性=高。
 *      若 P0.1 读回低(0)，说明 RX 一直被拉成显性 —— 收发器没供电/没接到这两脚/总线短路，
 *      控制器会永远等不到总线空闲 => 不发送(TEC 不涨)、也收不到(rx=0)，与实测现象吻合。
 *   ② 自发自收(TXSELF)：不需要总线上有第二个节点。能收到自己发的帧
 *      => 控制器 + 波特率 + 我们的收发代码通路全部正常，问题在引脚/收发器/总线。
 *      顺便校验 ID 组装：收到的 id 应等于发出去的 0x12345678（可验证 TXID3<<3 是否正确）。
 *   自检里会把 RSTMOD 的两种极性各试一次（A=RSTMOD 0 / B=RSTMOD 1），跑完恢复原状。
 */
extern void CAN_SelfTest(void);

/* ==================== 【调试用】CAN 引脚/收发器连通性探针 ====================
 * 正常模式下发一帧，同时高频采样 CANTX/CANRX 上的"显性(低)"次数：
 *   txLow>0 且 rxLow>0  => 两脚+收发器都通，问题在总线/主机侧
 *   txLow>0 且 rxLow=0  => 收发器不在这一对脚上（或没供电）
 *   txLow=0             => CANTX 引脚压根没被驱动
 * 需要换脚验证时改 CAN.c 顶部的 CAN_DEBUG_PROBE_CH1 开关。
 */
extern void CAN_PinProbe(void);

/* ==================== 【调试】 ==================== */
/* 正常 CAN 通信固定打开：心跳(500ms)/状态(150ms)/日志(1100ms) + 指令/配置解析。
 *    SPEED_MODE == CANMODE 时电机就是靠它收指令起停的，原来写 0 等于电机永远起不来；*/
#define CAN_DEBUG_PERIOD_MS   10   /* 观测帧发送周期(ms)。必须 ≥ 页数×1ms（参数，不是开关）*/
#if (DEBUG_UART_OR_CAN == 2)
extern void can_dbg_task(void);    /* 1ms 任务里调：计时，到点开一轮 */
extern void can_dbg_poll(void);    /* 1ms 任务里调：发一帧，不阻塞 */
#endif

/* ==================== 【调试】CAN 文本通道（模式2 用） ==================== */
#define CAN_DEBUG_TEXT_ID     (0x1F8UL)   /* 标准帧，DLC=8，载荷为 8 字节 ASCII */
extern volatile uint16 xdata g_dbgTxDrop; /* 文本帧“忙则丢”次数，排查总线拥塞用 */


/* ==================== 【调试】上位机→板的命令帧（预留通道）====================
 * 扩展帧 ID = CAN_DBG_CMD_ID，Byte0 = 命令码（命令码自己定义）。
 * 约定：这里只置标志，真正的动作放主循环执行（不要在 1ms 任务里做阻塞动作）。
 * 注：原 "0xE1 = EEPROM 自检" 已随 EEPROM 代码一起删除，移植 EEPROM 后按需再加。*/
#define CAN_DBG_CMD_ID        0x0A0000FFUL

extern volatile uint8 xdata g_dbgCmdLast;      /* 最近收到的命令码（观测用）*/

#endif

