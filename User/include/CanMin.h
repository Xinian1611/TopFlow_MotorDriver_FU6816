/**
 * @file      CanMin.h
 * @brief     最小 CAN 功能（无中断、轮询）—— 对外接口与数据类型
 *
 * ============================================================================
 *  与 20261030 的对照
 * ----------------------------------------------------------------------------
 *  本文件里的 MotorRunState_t / g_run_state 与 20261030
 *  User\include\MotorUserControl.h 中的定义“字段名、字段顺序、含义”完全一致，
 *  这样两边的 CanMin_ApplyCmd() / TargetRef_Process() CANMODE 分支可以逐行对照。
 *
 *  区别只有一个：20261030 的电机参数（Pole_Pairs/RS/LD/LQ/Ke/MOTOR_SPEED_BASE）
 *  是 float xdata 运行时变量、由 EEPROM 配置载入；最小工程仍是 Customer.h 的宏，
 *  所以 CanMin_SetTarget() 里直接用 CAN_PCT100_RPM（默认 MOTOR_SPEED_BASE）。
 * ============================================================================
 *
 *  调用位置（只改 3 行，不动电机控制结构）
 * ----------------------------------------------------------------------------
 *    main.c   HardwareInit()        : CanMin_Init();
 *    main.c   1ms 任务块            : CanMin_Poll();
 *    AddFunction.c TickCycle_1ms()  : TargetRef_Process(); 之后 CanMin_ApplyCmd();
 * ============================================================================
 */

#ifndef __CANMIN_H__
#define __CANMIN_H__

#include <CanProtocol.h>
#include <CanConfig.h>

/* ==========================================================================
 *  一、电机运行状态（字段与 20261030 的 MotorRunState_t 完全一致）
 * ========================================================================== */
typedef struct
{
    uint8  work_mode;         /* 工作模式：EXT_WORK_MODE_SPEED(2) / EXT_WORK_MODE_CURRENT(1) */
    uint8  is_started;        /* 启停状态：0=停机，1=起机 */
    uint8  is_estop;          /* 急停状态：0=非急停，1=急停 */
    uint8  reversal_braking;  /* 换向刹车：0=未刹车，1=刹车中（先减速到 0 再反向） */
    float  ref;               /* 目标值绝对值：转速模式是 rpm，电流模式是 A */
    uint8  direction;         /* 旋转方向：0=CW 正转，1=CCW 反转 */
    uint8  fault_code;        /* 当前故障码（最小版只填 FAULT_NONE / 0x80） */
    uint32 run_time;          /* 单次运行时间(s)，最小版不使用 */
    uint32 total_time;        /* 总运行时间(s)，最小版不使用 */
} MotorRunState_t;

/* 全局运行状态：CAN 指令解析后写这里，CanMin_ApplyCmd() 读这里 */
extern MotorRunState_t xdata g_run_state;

/* ==========================================================================
 *  二、通信统计（Keil Watch 里展开 g_canmin_stat 即可看通信是否正常）
 * ========================================================================== */
typedef struct
{
    uint16 RxFrames;          /* 收到的扩展帧总数（不管是不是本机 ID） */
    uint16 RxCmdOk;           /* 有效指令帧数（ID 匹配 + DLC 足够） */
    uint16 RxOther;           /* 非本机 / 非指令帧数 */
    uint16 TxFrames;          /* 已成功写进发送寄存器并置 TXREQ 的帧数 */
    uint16 TxSkip;            /* 因上一帧未发完而跳过的次数 */
    uint16 RxOvif;            /* 接收 FIFO 溢出次数（BUFOV 置位次数） */
    uint16 BusOffCnt;         /* 进入 bus-off 的次数 */
    uint32 LastId;            /* 最近一帧的 29 位 ID（调试用） */
} CanMinStat_t;

extern CanMinStat_t xdata g_canmin_stat;

/* ==========================================================================
 *  三、对外函数
 * ========================================================================== */

/* 初始化：配置引脚/位时间/过滤器，关闭全部 CAN 中断，使能 CAN 模块 */
void CanMin_Init(void);

/* 1ms 轮询：收帧 -> 解析 -> 周期发状态帧。放在 main.c 的 1ms 任务块里 */
void CanMin_Poll(void);

/* 把 g_run_state 里的目标值落到电机：放在 TargetRef_Process() 之后 */
void CanMin_ApplyCmd(void);

/* ---- 以下 4 个函数与 20261030 的 MotorCtrl_SetXxx() 一一对应 ---- */

/* 急停：state=1 急停（给定清零），state=0 解除 */
void CanMin_SetEstop(uint8 state);

/* 启停：state=1 起机，state=0 停机（停机时撤给定） */
void CanMin_SetRun(uint8 state);

/* 工作模式：最小版只支持 EXT_WORK_MODE_SPEED */
void CanMin_SetMode(uint8 mode);

/* 目标值：current 单位 0.01A，speed 单位 0.01%，零点都是 32767 */
void CanMin_SetTarget(uint16 current, uint16 speed);

/* IAP 专用：ID 虽 <=0x7FF，但必须发 29 位扩展帧；返回 1=已提交发送 */
uint8 CanMin_IapSend(uint16 can_id, uint8 *dat, uint8 len);

#endif /* __CANMIN_H__ */
