/**
 * @file    PositionCtrl.c
 * @brief   最简位置环：读 MT6701 位置 -> 读当前速度 -> PID -> 输出目标速度
 * @note    本文件只写 g_run_state.ref / direction / is_started，
 *          不直接操作 FOC 电流、占空比、MOE；
 *          本版先去掉限位、堵转、开环超时等逻辑，只保留位置闭环主干。
 */
#include <MyProject.h>
#include "PositionCtrl.h"
#include <string.h>

#define POS_DEBUG_EN    1   //0:POS_DEBUG_TARGET_POSITION不生效

#if POS_DEBUG_EN
#define POS_DEBUG_TARGET_POSITION    18000   /* raw：18000 × 0.01° = 180.00° */
#endif

/* ==================== 对外命令/状态 ==================== */
volatile uint16 xdata g_pos_cmd_raw   = 0xFFFF;  /* 目标 raw：转向0.01°/升降0.1%，0=零点；0xFFFF=无目标 */
volatile uint8  xdata g_pos_cmd_start = 0;       /* 启停命令：1=启动/继续，0=停止 */
volatile uint8  xdata g_pos_state     = POS_IDLE;/* 位置模块状态输出，给 TargetRef_Process 判断 */
volatile uint16 xdata g_pos_fb_raw    = 0xFFFF;  /* 闭环反馈 raw，给状态报文字节18-19；0xFFFF=无效 */
/* ==================== 内部状态 ==================== */
typedef struct                                   /* 位置环内部状态结构体 */
{
    float target;                                /* 目标值：角度(°) 或 百分比(%) */
    float fb;                                    /* 当前反馈值：角度(°) 或 百分比(%) */
    float err;                                   /* 当前误差：target - fb */
    float integ;                                 /* 位置积分项，单位 rpm */
    float speedFb;                               /* 当前转速，单位 rpm，来自 mcFocCtrl.SpeedFlt */
    float speedCmd;                              /* 位置环输出速度给定，rpm，带符号 */
    float fbFilt;                                /* 单圈反馈的一阶低通值 */
    uint16 tick;                                 /* 1ms 分频计数 */
    uint16 inposCnt;                             /* 到位持续时间计数 */
    uint16 lastRaw;                              /* 上一次目标 raw，用于判断目标是否变化 */
    uint8  fbValid;                              /* 1=反馈有效；0=反馈还没准备好 */
    uint8  state;                                /* 位置环状态：PosState_t */
} PosCtrl_t;
static PosCtrl_t xdata s_pos;                    /* 位置环唯一实例，放 xdata，避免占用 data 段 */
/* ==================== 宏 ==================== */
#define POS_ABS_F(x)        (((x) < 0.0f) ? -(x) : (x))                    /* 浮点绝对值 */
#define POS_DT              ((float)POS_CTRL_PERIOD_MS / 1000.0f)           /* 位置环周期，单位 s */
#define POS_INPOS_CNT       (POS_INPOS_MS / POS_CTRL_PERIOD_MS)             /* 到位确认需要的周期数 */
#define POS_SPEED_RPM()     ((float)mcFocCtrl.SpeedFlt / 32767.0f * MOTOR_SPEED_BASE) /* Q15 转速 -> rpm */
/* ==================== 函数1：初始化 ==================== */
void PositionCtrl_Init(void)                     /* 初始化位置环 */
{
    memset(&s_pos, 0, sizeof(s_pos));            /* 清空所有内部状态 */
    s_pos.state   = POS_IDLE;                    /* 初始状态：空闲 */
    s_pos.lastRaw = 0xFFFF;                      /* 还没有目标 raw */
    g_pos_cmd_raw   = 0xFFFF;                    /* 清目标命令 */
    g_pos_cmd_start = 0;                         /* 清启停命令 */
    g_pos_state     = POS_IDLE;                  /* 状态输出置空闲 */
    g_pos_fb_raw    = 0xFFFF;                    /* 反馈无效 */
}
/* ==================== 函数2：当前位置换算 ==================== */
/**
 * @brief  读取 MT6701，换算成角度/百分比，写入 s_pos.fb
 * @retval 1=反馈有效；0=反馈无效
 */
uint8 PositionCtrl_Current(void)                 /* 读 MT6701 当前位置 */
{
    float raw;                                   /* 原始单圈角度，单位 ° */
    float rawFb;                                 /* 反馈 raw 临时值*/
    //均值还没完成、电压高于上限：越界、电压低于下限：断线/越界
    if (((g_mt6701_anaSt & 0x01) == 0) || (g_mt6701_mv < POS_FB_MV_MIN) || (g_mt6701_mv > POS_FB_MV_MAX))
    {
        s_pos.fbValid = 0;                       /* 标记反馈无效 */
        g_pos_fb_raw  = 0xFFFF;                  /* 状态报文反馈无效 */
        return 0;                                /* 返回无效 */
    }
    raw = (float)MT6701_GetAngle10() / 10.0f;    /* 0.1° -> 0° */

    if (s_pos.fbValid == 0)                      /* 第一次有效反馈 */
    {
        s_pos.fbFilt  = raw;                     /* 用当前值填种，避免从 0 拉起 */
        s_pos.fbValid = 1;                       /* 标记反馈有效 */
    }
    else                                         /* 后续反馈 */
    {
        s_pos.fbFilt += POS_FB_LPF_ALPHA * (raw - s_pos.fbFilt); /* 一阶低通 */
    }

    s_pos.fb = POS_FB_DIR * (s_pos.fbFilt - POS_FB_ZERO_DEG) * POS_FB_SCALE;   //当前反馈值

    /* 换算状态报文 raw */
    rawFb = s_pos.fb / POS_CMD_RES + POS_CMD_ZERO; /* 角度×100 或 百分比×10 */
    if (rawFb < 0.0f)       { rawFb = 0.0f; }      /* 下限保护 */
    if (rawFb > 65535.0f)   { rawFb = 65535.0f; }  /* 上限保护 */
    g_pos_fb_raw = (uint16)rawFb;                  /* 给状态报文字节18-19 */

    return 1;                                    /* 反馈有效 */
}
/* ==================== 函数3：PID 计算 ==================== */
/**
 * @brief  位置 PI 计算，输出速度给定
 * @retval 1=已在死区内/已到位，本周期不输出；0=已输出速度给定
 * @note   只算位置环；限位、堵转、故障不在这里处理。
 */
uint8 PositionCtrl_PID(void)                     /* 位置 PI 计算 */
{
    float out;                                   /* PI 输出，单位 rpm */
    s_pos.err = s_pos.target - s_pos.fb;         /* 计算误差：目标 - 反馈 */

    if (POS_ABS_F(s_pos.err) <= POS_DEADBAND)    /* 误差进入到位死区 */
    {
        s_pos.integ     = 0.0f;                  /* 清积分 */
        s_pos.speedCmd  = 0.0f;                  /* 清输出 */
        s_pos.state     = POS_INPOS;             /* 直接锁存到位，不再等速度确认 */
        g_run_state.ref = 0.0f;                  /* 速度给定 0 */
        g_run_state.is_started = 0;              /* 请求 mcStop，由 MC_Stop 减速/抱闸 */
        return 1;                                /* 本周期不输出 */
    }

    s_pos.inposCnt = 0;                          /* 误差出了死区，清到位计数 */
    out = POS_KP * s_pos.err;                    /* 比例项 */
    if (POS_ABS_F(s_pos.err) <= POS_I_ZONE)      /* 误差在积分区 */
    {
        s_pos.integ += POS_KI * s_pos.err * POS_DT; /* 积分累加 */
        if (s_pos.integ > POS_I_LIMIT)           /* 积分上限 */
        {
            s_pos.integ = POS_I_LIMIT;           /* 限幅 */
        }
        else if (s_pos.integ < -POS_I_LIMIT)     /* 积分下限 */
        {
            s_pos.integ = -POS_I_LIMIT;          /* 限幅 */
        }
    }
    else                                         /* 误差大，不用积分 */
    {
        s_pos.integ = 0.0f;                      /* 清积分，防饱和 */
    }
    out += s_pos.integ;                          /* P + I */

    if (out > POS_SPEED_MAX_RPM)                 /* 正向超限 */
    {
        out = POS_SPEED_MAX_RPM;                 /* 正向限幅 */
    }
    else if (out < -POS_SPEED_MAX_RPM)           /* 反向超限 */
    {
        out = -POS_SPEED_MAX_RPM;                /* 反向限幅 */
    }

    if ((out > 0.0f) && (out < POS_SPEED_MIN_RPM)) /* 正向小于最小速度 */
    {
        out = POS_SPEED_MIN_RPM;                 /* 抬到最小速度 */
    }
    if ((out < 0.0f) && (out > -POS_SPEED_MIN_RPM)) /* 反向小于最小速度 */
    {
        out = -POS_SPEED_MIN_RPM;                /* 抬到最小速度 */
    }

    s_pos.speedCmd = out;                        /* 保存输出，供观测 */
    g_run_state.direction  = (out >= 0.0f) ? CW : CCW; /* 符号 -> 方向 */
    g_run_state.ref        = POS_ABS_F(out);     /* 幅值 -> 速度给定 */
    g_run_state.is_started = 1;                  /* 允许运行 */
    s_pos.state            = POS_MOVING;         /* 标记正在调节 */
    return 0;                                    /* 已输出速度给定 */
}

/* ==================== 函数4：位置模块更新 ==================== */
/**
 * @brief  位置环周期任务，1ms 调用一次，内部按 POS_CTRL_PERIOD_MS 分频
 * @note   只做：启停/故障判断、目标换算、读位置、读速度、调 PID。
 */
void PositionCtrl_Update(void)                   /* 位置环周期任务 */
{
    float t;                                     /* 解码后的目标值 */
    if (++s_pos.tick < POS_CTRL_PERIOD_MS)       /* 还没到位置环周期 */
    {
        return;                                  /* 直接返回 */
    }
    s_pos.tick = 0;                              /* 到周期，清分频计数 */

#if POS_DEBUG_EN
    g_pos_cmd_start = 1;                         /* 调试：强制起控 */
    g_pos_cmd_raw   = POS_DEBUG_TARGET_POSITION; /* 调试：强制目标，覆盖 CAN */
#endif

    /* 收到停止命令、没有目标 raw */
    if ((g_pos_cmd_start == 0) || (g_pos_cmd_raw == 0xFFFFU))
    {
        s_pos.state     = POS_IDLE;              /* 回空闲 */
        s_pos.integ     = 0.0f;                  /* 清积分 */
        s_pos.speedCmd  = 0.0f;                  /* 清输出 */
        g_pos_state     = POS_IDLE;              /* 状态输出空闲 */
        g_run_state.ref = 0.0f;                  /* 速度给定 0 */
        g_run_state.is_started = 0;              /* 启停位清 0 */
        return;                                  /* 本周期结束 */
    }
    /* 有故障源、状态机已进故障 */
    if ((mcFaultSource != FaultNoSource) || (mcState == mcFault))
    {
        s_pos.state     = POS_FAULT;             /* 位置环故障 */
        s_pos.integ     = 0.0f;                  /* 清积分 */
        s_pos.speedCmd  = 0.0f;                  /* 清输出 */
        g_pos_state     = POS_FAULT;             /* 状态输出故障 */
        g_run_state.ref = 0.0f;                  /* 速度给定 0 */
        g_run_state.is_started = 0;              /* 启停位清 0 */
        return;                                  /* 本周期结束 */
    }
    t = ((float)g_pos_cmd_raw - (float)POS_CMD_ZERO) * POS_CMD_RES; /* raw -> 角度/% */

    if ((t < POS_TARGET_MIN) || (t > POS_TARGET_MAX)) /* 目标超程 */
    {
        g_run_state.ref = 0.0f;                  /* 速度给定 0 */
        g_run_state.is_started = 0;              /* 不允许运行 */
        return;                                  /* 拒绝本次目标 */
    }
    s_pos.target = t;                            /* 保存新目标 */

    if (g_pos_cmd_raw != s_pos.lastRaw)          /* 目标 raw 变了 */
    {
        s_pos.lastRaw  = g_pos_cmd_raw;          /* 记录本次 raw */
        s_pos.integ    = 0.0f;                   /* 清积分，防旧目标积分拖累 */
        s_pos.inposCnt = 0;                      /* 清到位计数 */
        s_pos.state = POS_MOVING;                /* 新目标，清到位/限位锁存 */
    }

#if MOTOR_LIMIT_SWITCH
    if (s_pos.state == POS_LIMIT)                /* 已经限位：锁住，等新目标 */
    {
        g_pos_state     = POS_LIMIT;             /* 保持限位状态 */
        g_run_state.ref = 0.0f;                  /* 速度给定 0 */
        g_run_state.is_started = 0;              /* 保持停机 */
        return;                                  /* 本周期结束 */
    }
#endif

    if (PositionCtrl_Current() == 0)             /* 反馈无效 */
    {
        s_pos.state     = POS_FAULT;             /* 位置环故障 */
        s_pos.integ     = 0.0f;                  /* 清积分 */
        s_pos.speedCmd  = 0.0f;                  /* 清输出 */
        g_pos_state     = POS_FAULT;             /* 状态输出故障 */
        g_run_state.ref = 0.0f;                  /* 速度给定 0 */
        g_run_state.is_started = 0;              /* 启停位清 0 */
        return;                                  /* 本周期结束 */
    }
    s_pos.speedFb = POS_SPEED_RPM();             /* 读当前速度：Q15 -> rpm */
    s_pos.err     = s_pos.target - s_pos.fb;     /* 先算一次误差，决定是否允许启动 */

    if (s_pos.state == POS_INPOS)                /* 已经到位：锁住，不自动重起 */
    {
        if (POS_ABS_F(s_pos.err) <= POS_REENGAGE_BAND)
        {
            g_pos_state     = POS_INPOS;         /* 保持到位 */
            g_run_state.ref = 0.0f;              /* 速度给定 0 */
            g_run_state.is_started = 0;          /* 保持停机 */
            return;                              /* 本周期结束 */
        }
        s_pos.state = POS_MOVING;                /* 漂移超过回差，才重新调节 */
    }
    if (POS_ABS_F(s_pos.err) > POS_DEADBAND)     /* 误差在死区外，需要运动 */
    {
        s_pos.state     = POS_MOVING;            /* 进入位置调节 */
        g_pos_state     = POS_MOVING;            /* 通知 TargetRef 允许启动 */
        g_run_state.is_started = 1;              /* 请求状态机启动 */
    }
    else if (mcState != mcRun)                   /* 已在死区且本来就没运行 */
    {
        s_pos.state     = POS_INPOS;             /* 直接报到位 */
        g_pos_state     = POS_INPOS;             /* 状态输出到位 */
        g_run_state.ref = 0.0f;                  /* 速度给定 0 */
        g_run_state.is_started = 0;              /* 不为一个已在死区的目标启动 FOC */
        return;                                  /* 本周期结束 */
    }

        if (mcState != mcRun)                        /* 状态机还没进运行态 */
    {
        s_pos.integ    = 0.0f;                   /* 清积分 */
        s_pos.speedCmd = 0.0f;                   /* 清输出 */
        g_run_state.ref = 0.0f;                  /* 速度给定 0，先不控 */
        return;                                  /* 本周期结束 */
    }

#if MOTOR_LIMIT_SWITCH
    /* 限位：到软件 MAX/MIN 且目标还往外 → 立即停机，和速度无关 */
    if (PositionProtect_LimitCheck(s_pos.target, s_pos.fb, 1) == POS_PROTECT_LIMIT)
    {
        s_pos.state     = POS_LIMIT;             /* 锁存限位 */
        s_pos.integ     = 0.0f;
        s_pos.speedCmd  = 0.0f;

        g_pos_state     = POS_LIMIT;             /* 状态输出限位 */
        g_run_state.ref = 0.0f;                  /* 速度给定 0 */
        g_run_state.is_started = 0;              /* 立即 mcStop */
        return;
    }
#endif

    (void)PositionCtrl_PID();                    /* PID 计算并输出目标速度 */
    g_pos_state = s_pos.state;                   /* 同步状态：MOVING 或 INPOS */
}

static uint8 xdata s_speedLimitLatch = 0;        /* 速度模式限位锁存 */
/**
 * @brief  纯速度模式下的软件限位判断
 * @retval POS_PROTECT_LIMIT / POS_PROTECT_NONE
 * @note   位置模式由 PositionCtrl_Update() 里的 LimitCheck 处理；
 *         这里只处理 work_mode == SPEED 的情况。
 */
uint8 PositionCtrl_SpeedLimitCheck(void)
{
#if MOTOR_LIMIT_SWITCH
    if (g_run_state.work_mode != EXT_WORK_MODE_SPEED)
    {
        return POS_PROTECT_NONE;
    }

    /* 已经锁存：只有方向往回、或者速度给定为 0，才解除 */
    if (s_speedLimitLatch)
    {
        if ((g_run_state.ref <= 0.0f) ||
            ((s_pos.fb >= POS_LIMIT_MAX) && (g_run_state.direction == CCW)) ||
            ((s_pos.fb <= POS_LIMIT_MIN) && (g_run_state.direction == CW)))
        {
            s_speedLimitLatch = 0;
        }
        else
        {
            return POS_PROTECT_LIMIT;
        }
    }

    if (PositionCtrl_Current() == 0)             /* 读 MT6701，刷新 s_pos.fb */
    {
        return POS_PROTECT_NONE;                 /* 反馈无效，没法判 */
    }

    if ((s_pos.fb >= POS_LIMIT_MAX) &&
        (g_run_state.is_started != 0) &&
        (g_run_state.ref > 0.0f) &&
        (g_run_state.direction == CW))
    {
        s_speedLimitLatch = 1;
        return POS_PROTECT_LIMIT;
    }

    if ((s_pos.fb <= POS_LIMIT_MIN) &&
        (g_run_state.is_started != 0) &&
        (g_run_state.ref > 0.0f) &&
        (g_run_state.direction == CCW))
    {
        s_speedLimitLatch = 1;
        return POS_PROTECT_LIMIT;
    }
#endif

    return POS_PROTECT_NONE;
}
