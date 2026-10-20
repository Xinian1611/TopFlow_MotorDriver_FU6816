#ifndef POSITION_CTRL_H                       /* 防止头文件被重复包含 */
#define POSITION_CTRL_H                       /* 定义头文件包含标志 */

#include <FU68xx_6.h>                         /* 提供 uint8/uint16/int16/int32 等基础类型 */
#include <Customer.h>                         /* 提供 MOTOR_POSITION_CLOSE_LOOP、DEBUG_UART_OR_CAN 等工程配置 */

/* ==================== 位置环公共参数 ==================== */
#ifndef POS_CTRL_PERIOD_MS                    /* 允许工程在 Target Define 里覆盖位置环周期 */
#define POS_CTRL_PERIOD_MS          10        /* 位置环周期(ms)：10ms 一次 */
#endif                                        /* POS_CTRL_PERIOD_MS 结束 */

#define POS_OPENLOOP_TIMEOUT_MS     2000      /* CtrlMode=0 超过 2s 仍未进速度环 -> 故障 */

/* ==================== 按宏选择参数 ==================== */
#if (MOTOR_POSITION_CLOSE_LOOP != 2)         /* 0/1：角度参数；*/

/* ---------- 角度位置闭环：转向 ---------- */
#define POS_CMD_ZERO                0         /* 闭环指令零点：0=0° */
#define POS_CMD_RES                 0.01f     /* 闭环指令分辨率：0.01°/bit */

#define POS_TARGET_MIN              0.0f      /* 目标角度下限(°) */
#define POS_TARGET_MAX              360.0f    /* 目标角度上限(°) */

#define POS_FB_ZERO_DEG             1.0f      /* 现场机械0°时 MT6701 读数约1.4°，按实测改 */
#define POS_FB_DIR                  1.0f      /* 反馈方向：正装 +1，反装 -1 */
#define POS_FB_SCALE                1.0f      /* 反馈比例：360°机构 1.0，540/360 机构改 1.5 */
#define POS_FB_MV_MIN               200       /* 反馈电压下限(mV) */
#define POS_FB_MV_MAX               1800      /* 反馈电压上限(mV) */

#define POS_ANGLE_WRAP              0         /* 0=不折误差；1=误差折到 -180~+180 */
#define POS_DEADBAND                1.0f      /* (°) 到位死区 */
#define POS_REENGAGE_BAND           1.5f      /* (°) 到位后超过该误差重新起控 */
#define POS_TARGET_CHANGE_BAND      0.5f      /* (°) 小于该变化视为同一条目标 */

#define POS_KP                      20.0f      /* 位置环比例系数，rpm/° */
#define POS_KI                      0.3f      /* 位置环积分系数，rpm/(°·s) */
#define POS_I_LIMIT                 5.0f     /* 积分限幅(rpm) */
#define POS_I_ZONE                  5.0f     /* (°) 误差大于该值清积分 */

#define POS_SPEED_MAX_RPM           2200.0f    /* 位置环最大速度给定(rpm) */
#define POS_SPEED_MIN_RPM           150.0f    /* 最小起控速度(rpm)，必须 > MOTOR_LOOP_RPM */
#define POS_INPOS_SPEED_RPM         0.0f     /* 到位确认时允许的转速(rpm) */

#define POS_INPOS_MS                100       /* (ms) 到位持续时间 */
#define POS_FB_LPF_ALPHA            0.8f      /* 反馈一阶低通系数 */

#else                                        /* 2=百分比位置闭环（升降/折叠） */

/* ---------- 百分比位置闭环：升降/折叠 ---------- */
#define POS_CMD_ZERO                0         /* 闭环指令零点：0=0% */
#define POS_CMD_RES                 0.1f      /* 闭环指令分辨率：0.1%/bit */

#define POS_TARGET_MIN              0.0f      /* 目标百分比下限(%) */
#define POS_TARGET_MAX              100.0f    /* 目标百分比上限(%) */

#define POS_FB_ZERO_DEG             0.0f      /* 0% 对应的传感器角度(°) */
#define POS_FB_DIR                  1.0f      /* 反馈方向：正装 +1，反装 -1 */
#define POS_FB_SCALE                (100.0f / 360.0f) /* 反馈比例：0~360°对应0~100% */
#define POS_FB_OFFSET               0.0f      /* 反馈偏置(%) */
#define POS_FB_MV_MIN               150       /* 反馈电压下限(mV) */
#define POS_FB_MV_MAX               1850      /* 反馈电压上限(mV) */

#define POS_ANGLE_WRAP              0         /* 百分比是线性量，不做角度折返 */
#define POS_DEADBAND                0.3f      /* (%) 到位死区 */
#define POS_REENGAGE_BAND           1.0f      /* (%) 抱闸回差 */
#define POS_TARGET_CHANGE_BAND      0.2f      /* (%) 小于该变化视为同一条目标 */

#define POS_KP                      10.0f     /* 位置环比例系数，rpm/% */
#define POS_KI                      0.8f      /* 位置环积分系数，rpm/(%·s) */
#define POS_I_LIMIT                 100.0f    /* 积分限幅(rpm) */
#define POS_I_ZONE                  10.0f     /* (%) 误差大于该值清积分 */

#define POS_SPEED_MAX_RPM           600.0f    /* 位置环最大速度给定(rpm) */
#define POS_SPEED_MIN_RPM           120.0f    /* 最小起控速度(rpm)，必须 > MOTOR_LOOP_RPM */
#define POS_INPOS_SPEED_RPM         50.0f     /* 到位确认时允许的转速(rpm) */

#define POS_INPOS_MS                300       /* (ms) 到位持续时间 */
#define POS_START_GRACE_MS          800       /* (ms) 起控宽限，期间不判堵转 */
#define POS_FB_FAULT_MS             300       /* (ms) 反馈异常持续时间 */
#define POS_FB_LPF_ALPHA            0.3f      /* 反馈一阶低通系数 */
#define POS_AUTO_REENGAGE           1         /* 1=到位后误差超回差自动重新起控 */

#endif                                        /* MOTOR_POSITION_CLOSE_LOOP 结束 */

/**
 * @brief  位置环状态
 * @note   取值会同步到 g_pos_state，给 TargetRef_Process 判断到位/限位/故障。
 */
typedef enum                                  /* 位置环状态枚举 */
{                                             /* 枚举开始 */
    POS_IDLE = 0,                             /* 0：空闲，没有有效目标 */
    POS_MOVING,                               /* 1：正在做位置调节 */
    POS_INPOS,                                /* 2：已到位，速度给定 0，等待状态机停机/合闸 */
    POS_LIMIT,                                /* 3：机械限位，已停机，等反向目标 */
    POS_STALL,                                /* 4：堵转（电流判定），需停机复位 */
    POS_FAULT                                 /* 5：反馈异常/开环超时等故障，已停车 */
} PosState_t;                                 /* 位置环状态类型定义结束 */

/* CAN/应用层写这两个 */
extern volatile uint16 xdata g_pos_cmd_raw;   /* 目标 raw：转向0.01°/bit，升降0.1%/bit，0=零点；0xFFFF=无目标 */
extern volatile uint8  xdata g_pos_cmd_start; /* 启停命令：1=启动/继续，0=停止 */

/* 位置模块输出这个，给 TargetRef_Process 判断 */
extern volatile uint8  xdata g_pos_state;     /* 当前状态，取值 PosState_t */
extern volatile uint16 xdata g_pos_fb_raw;    /* 闭环反馈 raw，给状态报文字节18-19 */

void  PositionCtrl_Init(void);                /* 初始化位置环 */
uint8 PositionCtrl_Current(void);             /* 读取并换算当前位置 */
uint8 PositionCtrl_PID(void);                 /* 位置 PID 计算 */
void  PositionCtrl_Update(void);              /* 位置环周期任务 */

uint8 PositionCtrl_SpeedLimitCheck(void);

#endif                                        /* POSITION_CTRL_H 结束 */
