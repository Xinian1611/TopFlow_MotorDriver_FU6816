/**
 * @file      CanMin.c
 * @brief     最小 CAN 功能验证模块（无中断、纯轮询）
 *  设计要点
 * ----------------------------------------------------------------------------
 *   1. 不使用 CAN 中断：
 *        20261030 是 SetBit(CAN_IER, ...RXIE|TXIE|...) + Interrupt.c 的
 *        CAN_INT(interrupt 8) 中断收帧；本模块把 CAN_IER 全部写 0，
 *        在 1ms 任务里轮询 CAN_SR.NOEMP 收帧。两条路径完全独立，
 *        原 CAN.c / CAN.h / Interrupt.c 一行未改。
 *
 *   2. 只有两类报文：
 *        指令 0x042CF081（8 字节，优先级 2，主机 -> 驱动板）
 *        状态 0x062CF081（8 字节，优先级 3，驱动板 -> 主机，150ms）
 *
 *   3. 过滤器放行全部扩展帧，地址在软件层比对（和 20261030 的 CAN_Init 一样）。
 *
 *   4. 发送非阻塞：
 *        20261030 的 CAN_Send() 是 while(!TXSUC) 死等（10000 次超时）；
 *        本模块上一帧没发完就跳过本周期，另加 CAN_TX_TIMEOUT_MS 超时重发，
 *        保证 1ms 任务绝不被 CAN 发送拖住。
 *
 *   5. 控制映射与 20261030 的 TargetRef_Process() CANMODE 分支一致：
 *        急停 -> Ref=0、isCtrlPowOn=0；
 *        起机/停机 -> isCtrlPowOn = g_run_state.is_started；
 *        换向 -> 先减速到 50rpm 以下再切 mcFocCtrl.FR；
 *        目标 -> mcFocCtrl.Ref = S_Value(g_run_state.ref)。
 * ============================================================================
 */

#include <MyProject.h>
#include <string.h>

/* ==========================================================================
 *  一、工程量换算（与 20261030 的 CanSlave.c 完全一致）
 * ========================================================================== */
/* 速度：Q15 -> rpm。
 * mcFocCtrl.SpeedFlt 是 Q15（32767 对应 MOTOR_SPEED_BASE = 4096rpm），
 * 除以 32767 再乘基准，就是实际转速。 */
#define CANMIN_S2F(x)   ((float)(x) / 32767.0f * MOTOR_SPEED_BASE)

/* 电流：Q15 -> A。
 * 电流采样链路的满量程 = HW_ADC_REF / (HW_RSHUNT * HW_AMPGAIN)，
 * 本板：4.5V / (0.001Ω * 16) = 281.25A（对应 32767）。 */
#define CANMIN_I2F(x)   ((float)(x) / 32767.0f * (float)HW_ADC_REF / ((float)HW_RSHUNT * (float)HW_AMPGAIN))

/* 母线电压：Q15 -> V。满量程 = HW_BOARD_VOLT_MAX（由分压电阻决定）。 */
#define CANMIN_U2F(x)   ((float)(x) / 32767.0f * (float)HW_BOARD_VOLT_MAX)

/* ==========================================================================
 *  二、全局变量
 * ========================================================================== */

/* 电机运行状态：CAN 指令解析后写这里，CanMin_ApplyCmd() 读这里。
 * 字段与 20261030 的 MotorUserControl.h 完全一致，可以直接对照。 */
MotorRunState_t xdata g_run_state;

/* 通信统计：Keil Watch 里展开 g_canmin_stat 就能看到通信是否正常。 */
CanMinStat_t xdata g_canmin_stat;

/* 当前接收帧 / 待发送帧。
 * 结构体 CAN_Data 来自 CAN.h，字段：
 *   CAN_IDE(0x80=扩展帧) / CAN_RTR / CAN_DLC / EXTID / STDID / Data0~Data7
 * 名字不能叫 Can/CanSend —— 原 CAN.c 里已经定义了，会重复定义。 */
CAN_Data xdata CanMin;          /* 接收帧 */
CAN_Data xdata CanMinSend;      /* 发送帧（对应 20261030 的 CanSend） */

/* ==========================================================================
 *  三、模块内部状态
 * ========================================================================== */

/* 状态帧周期计数：每 1ms +1，到 CAN_STATUS_PERIOD_MS 发一帧。
 * 对应 20261030 CanSlave.c 里的 g_ext_status_cnt。 */
static uint16 xdata g_canmin_status_cnt;

/* 发送就绪标志：1=可以写发送寄存器；0=上一帧还没发完。
 * 20261030 用 while(!TXSUC) 等，这里用状态机，避免阻塞。 */
static uint8  xdata g_canmin_tx_ready;

/* 发送等待计时（ms）：g_canmin_tx_ready=0 时累加，超过 CAN_TX_TIMEOUT_MS 强制重发 */
static uint16 xdata g_canmin_tx_wait;

/* bus-off 边沿检测：只在“进入”bus-off 时计一次数 */
static uint8  xdata g_canmin_busoff_seen;

/* bus-off 重试计时(ms)：恢复一次后若还没退出，间隔 10ms 再试 */
static uint16 xdata g_canmin_busoff_ms;

#if (CAN_CMD_TIMEOUT_MS > 0)
/* 指令超时计数：超过 CAN_CMD_TIMEOUT_MS 没收到有效指令就自动停机 */
static uint16 xdata g_canmin_cmd_timeout;
#endif

/* 前置声明：
 * CanMin_MsgParse() 在下面第 229 行左右会调用 CanMin_ProcessCmd()，
 * 而它的定义在后面。C51 看不到原型时会先把这次调用当成“隐式 int 函数”，
 * 等后面真正定义时报 C231 redefinition。这里先声明一次即可。 */
static void CanMin_ProcessCmd(void);

/* ==========================================================================
 *  四、接收：把当前 FIFO 里最老的一帧读进 CanMin
 * --------------------------------------------------------------------------
 *  这是 20261030 的 CAN_Read() 去掉环形缓冲后的版本，读法逐字节一致：
 *    - CAN_CR2[RDPTR1:RDPTR0] 指示当前该读哪个接收缓冲（RX0/RX1/RX2）
 *    - 每个缓冲有 控制寄存器(IDE/RTR/DLC) + 4 个 ID 字节 + 8 个数据字节
 *    - 扩展帧 ID 重组：ID0<<21 | ID1<<13 | ID2<<5 | ID3>>3
 *    - 标准帧 ID 重组：ID0<<3 | ID1>>5
 *  读完由调用者置 CAN_CR1.BUFRLS 释放该缓冲。
 * ========================================================================== */
static void CanMin_ReadFrame(void)
{
    uint8 rdptr;

    /* 读指针：0x00=RX0, 0x40=RX1, 0x80=RX2 */
    rdptr = CAN_CR2 & (RDPTR1 | RDPTR0);

    switch (rdptr)
    {
        /* ---------------------------- 接收缓冲 0 ---------------------------- */
        case (0 | 0):
            CanMin.CAN_IDE = (CAN_RX0CR & 0x80);      /* 0x80=扩展帧，0=标准帧 */
            CanMin.CAN_RTR = (CAN_RX0CR & 0x40);      /* 0x40=远程帧 */
            CanMin.CAN_DLC = (CAN_RX0CR & 0x0F);      /* 数据长度 0~8 */

            if (CanMin.CAN_IDE)
            {
                CanMin.EXTID  = 0;
                CanMin.EXTID |= ((uint32)CAN_RX0ID0 << 21);
                CanMin.EXTID |= ((uint32)CAN_RX0ID1 << 13);
                CanMin.EXTID |= ((uint32)CAN_RX0ID2 << 5);
                CanMin.EXTID |= ((uint32)CAN_RX0ID3 >> 3);
            }
            else
            {
                CanMin.STDID  = 0;
                CanMin.STDID |= ((uint32)CAN_RX0ID0 << 3);
                CanMin.STDID |= (uint32)(CAN_RX0ID1 >> 5);
            }

            CanMin.Data0 = CAN_RX0DR0;
            CanMin.Data1 = CAN_RX0DR1;
            CanMin.Data2 = CAN_RX0DR2;
            CanMin.Data3 = CAN_RX0DR3;
            CanMin.Data4 = CAN_RX0DR4;
            CanMin.Data5 = CAN_RX0DR5;
            CanMin.Data6 = CAN_RX0DR6;
            CanMin.Data7 = CAN_RX0DR7;
            break;

        /* ---------------------------- 接收缓冲 1 ---------------------------- */
        case RDPTR0:
            CanMin.CAN_IDE = (CAN_RX1CR & 0x80);
            CanMin.CAN_RTR = (CAN_RX1CR & 0x40);
            CanMin.CAN_DLC = (CAN_RX1CR & 0x0F);

            if (CanMin.CAN_IDE)
            {
                CanMin.EXTID  = 0;
                CanMin.EXTID |= ((uint32)CAN_RX1ID0 << 21);
                CanMin.EXTID |= ((uint32)CAN_RX1ID1 << 13);
                CanMin.EXTID |= ((uint32)CAN_RX1ID2 << 5);
                CanMin.EXTID |= ((uint32)CAN_RX1ID3 >> 3);
            }
            else
            {
                CanMin.STDID  = 0;
                CanMin.STDID |= ((uint32)CAN_RX1ID0 << 3);
                CanMin.STDID |= (uint32)(CAN_RX1ID1 >> 5);
            }

            CanMin.Data0 = CAN_RX1DR0;
            CanMin.Data1 = CAN_RX1DR1;
            CanMin.Data2 = CAN_RX1DR2;
            CanMin.Data3 = CAN_RX1DR3;
            CanMin.Data4 = CAN_RX1DR4;
            CanMin.Data5 = CAN_RX1DR5;
            CanMin.Data6 = CAN_RX1DR6;
            CanMin.Data7 = CAN_RX1DR7;
            break;

        /* ---------------------------- 接收缓冲 2 ---------------------------- */
        case RDPTR1:
            CanMin.CAN_IDE = (CAN_RX2CR & 0x80);
            CanMin.CAN_RTR = (CAN_RX2CR & 0x40);
            CanMin.CAN_DLC = (CAN_RX2CR & 0x0F);

            if (CanMin.CAN_IDE)
            {
                CanMin.EXTID  = 0;
                CanMin.EXTID |= ((uint32)CAN_RX2ID0 << 21);
                CanMin.EXTID |= ((uint32)CAN_RX2ID1 << 13);
                CanMin.EXTID |= ((uint32)CAN_RX2ID2 << 5);
                CanMin.EXTID |= ((uint32)CAN_RX2ID3 >> 3);
            }
            else
            {
                CanMin.STDID  = 0;
                CanMin.STDID |= ((uint32)CAN_RX2ID0 << 3);
                CanMin.STDID |= (uint32)(CAN_RX2ID1 >> 5);
            }

            CanMin.Data0 = CAN_RX2DR0;
            CanMin.Data1 = CAN_RX2DR1;
            CanMin.Data2 = CAN_RX2DR2;
            CanMin.Data3 = CAN_RX2DR3;
            CanMin.Data4 = CAN_RX2DR4;
            CanMin.Data5 = CAN_RX2DR5;
            CanMin.Data6 = CAN_RX2DR6;
            CanMin.Data7 = CAN_RX2DR7;
            break;

        default:
            break;
    }
}

/* ==========================================================================
 *  五、报文分发：只认本机指令 ID 和广播 ID
 * --------------------------------------------------------------------------
 *  对应 20261030 的 CAN_MsgParse() + ext_slave_process_rx() 里的地址过滤部分。
 *  这里不做多帧组装（配置报文不用），只处理 8 字节单帧指令。
 * ========================================================================== */
static void CanMin_MsgParse(void)
{
    /* 只处理扩展帧 */
    if (CanMin.CAN_IDE != IDE_EXT)
    {
        g_canmin_stat.RxOther++;
        return;
    }

    /* 本机指令帧 0x042CF081 或广播帧 0x042CF001 */
    if ((CanMin.EXTID == CAN_CMD_ID) || (CanMin.EXTID == CAN_CMD_ID_BCAST))
    {
        /* 指令报文固定 8 字节，长度不对直接丢（同 20261030 的 len != 8） */
        if (CanMin.CAN_DLC == CAN_FRAME_DATA_LEN)
        {
            CanMin_ProcessCmd();
        }
        else
        {
            g_canmin_stat.RxOther++;
        }
    }
    else
    {
        g_canmin_stat.RxOther++;
    }
}

/* ==========================================================================
 *  六、指令解析
 * --------------------------------------------------------------------------
 *  对应 20261030 CanSlave.c 的 ext_process_cmd_message()。
 *  处理顺序（和 20261030 完全一致，顺序不能乱）：
 *      ① 急停（最高优先级，急停帧不再往下处理）
 *      ② 转速模式 + 0%（raw==32767）-> 直接停机
 *      ③ 启停
 *      ④ 工作模式（只认转速模式）
 *      ⑤ 目标值
 * ========================================================================== */
static void CanMin_ProcessCmd(void)
{
    uint8  dat[CAN_FRAME_DATA_LEN];   /* 先拷成 8 字节数组，方便按协议结构体解析 */
    ExtCmdMsg_t *msg;
    uint8  ctrl;
    uint8  work_mode;
    uint8  startstop;
    uint8  estop;
    uint16 cur_raw;
    uint16 speed_raw;

    dat[0] = CanMin.Data0;
    dat[1] = CanMin.Data1;
    dat[2] = CanMin.Data2;
    dat[3] = CanMin.Data3;
    dat[4] = CanMin.Data4;
    dat[5] = CanMin.Data5;
    dat[6] = CanMin.Data6;
    dat[7] = CanMin.Data7;

    msg = (ExtCmdMsg_t *)dat;

    /* ---- Byte0: ctrl 位域 ---- */
    ctrl      = msg->ctrl.value;
    work_mode = EXT_GET_WORKMODE(ctrl);
    startstop = EXT_GET_STARTSTOP(ctrl);
    estop     = EXT_GET_ESTOP(ctrl);

    /* ---- Byte1-2 / Byte3-4: 目标转速 / 目标电流（大端，零点 32767） ----
     * 注意必须按无符号读：正数的 raw 一定 > 32767，用 int16 会变成大负数。 */
    speed_raw = EXT_GET_U16(msg->target_speed_hi,   msg->target_speed_lo);
    cur_raw   = EXT_GET_U16(msg->target_current_hi, msg->target_current_lo);

    g_canmin_stat.RxCmdOk++;

#if (CAN_CMD_TIMEOUT_MS > 0)
    g_canmin_cmd_timeout = 0;         /* 收到有效指令，清通信超时计数 */
#endif
    g_run_state.fault_code = FAULT_NONE;   /* 清上一次“通信异常停机”的故障码 */

    /* ---- ① 急停：1=急停 2=非急停；0=无操作、3=无效值 -> 保持原状态 ----
     * 急停帧不再往下处理启停/给定。 */
    if (estop == EXT_ESTOP_ACTIVE)
    {
        CanMin_SetEstop(1);
        return;
    }
    if (estop == EXT_ESTOP_RELEASE)
    {
        CanMin_SetEstop(0);
    }

    /* ---- ② 转速模式 + 0%(raw == 32767)：直接按停机处理 ----
     * 必须放在“起机”之前：否则每个 0% 帧都会先 SetRun(1) 再 SetRun(0)，
     * 状态机会抖一下（预充电/预定位跑一半又被拉回停止）。 */
    if ((g_run_state.work_mode == EXT_WORK_MODE_SPEED) && (speed_raw == 32767))
    {
        CanMin_SetRun(0);                       /* 撤给定 + 进 mcStop */
        CanMin_SetTarget(cur_raw, speed_raw);   /* 内部把 ref 置 0 */
        return;                                 /* 不再走起机流程 */
    }

    /* ---- ③ 启停：1=起机 2=停机；0=无操作、3=无效值 -> 保持原状态 ---- */
    if (startstop == EXT_STARTSTOP_START)
    {
        CanMin_SetRun(1);
    }
    else if (startstop == EXT_STARTSTOP_STOP)
    {
        CanMin_SetRun(0);
    }

    /* ---- ④ 工作模式：最小版只支持转速模式 ----
     * 电流模式需要运行时切换 MOTOR_CTRL_MODE，那会改动电机控制结构，
     * 所以这里只接受 EXT_WORK_MODE_SPEED，收到 1(电流) 时忽略。 */
    if (work_mode == EXT_WORK_MODE_SPEED)
    {
        CanMin_SetMode(EXT_WORK_MODE_SPEED);
    }

    /* ---- ⑤ 目标值 ---- */
    CanMin_SetTarget(cur_raw, speed_raw);
}

/* ==========================================================================
 *  七、控制层：对应 20261030 MotorUserControl.c 的 MotorCtrl_SetXxx()
 * ========================================================================== */

/**
 * @brief  设置急停状态
 * @param  state 0=非急停，1=急停
 * @note   急停时把给定清零；解除急停后需要重新发“起机+目标”。
 */
void CanMin_SetEstop(uint8 state)
{
    g_run_state.is_estop = state;
    if (state)
    {
        g_run_state.ref      = 0;
        g_run_state.run_time = 0;
    }
}

/**
 * @brief  设置启停状态
 * @param  state 0=停机，1=起机
 * @note   停机时撤给定（ref=0），再起机必须重新下发速度 —— 与 20261030 一致。
 */
void CanMin_SetRun(uint8 state)
{
    if (state == g_run_state.is_started)
    {
        return;
    }

    g_run_state.is_started = state;

    if (g_run_state.is_started == 0)
    {
        g_run_state.run_time         = 0;
        g_run_state.ref              = 0.0f;
        g_run_state.reversal_braking = 0;
    }
}

/**
 * @brief  设置工作模式
 * @param  mode EXT_WORK_MODE_SPEED(2) 或 EXT_WORK_MODE_CURRENT(1)
 * @note   最小版只支持转速模式；电流模式需要改 MOTOR_CTRL_MODE，
 *         会破坏“不改电机控制结构”的约束，所以这里直接忽略。
 */
void CanMin_SetMode(uint8 mode)
{
    if (mode != EXT_WORK_MODE_SPEED)
    {
        return;
    }

    if (mode == g_run_state.work_mode)
    {
        return;
    }

    g_run_state.work_mode = mode;
}

/**
 * @brief  设置目标值（正数正转，负数反转）
 * @param  current 目标电流：0.01A，32767 = 0A
 * @param  speed   目标转速：0.01%，32767 = 0%，±10000 = ±100%
 * @note   ① 必须按无符号收：正数 raw 一定 >32767，用 int16 会变成大负数；
 *         ② 0% 只表示“目标 0”，停机由启停字段负责（见 CanMin_ProcessCmd）；
 *         ③ 20261030 用 g_motor_cfg->max_speed / idle_speed 做标定，
 *            最小版没有 EEPROM，改用 CanConfig.h 的 CAN_PCT100_RPM / CAN_IDLE_RPM。
 */
void CanMin_SetTarget(uint16 current, uint16 speed)
{
    int32 v;
    float mag;

    if (g_run_state.is_estop)
    {
        return;                                 /* 急停中不接受目标 */
    }

    if (g_run_state.work_mode == EXT_WORK_MODE_CURRENT)
    {
        /* 电流模式：0.01A -> A，存绝对值，方向由符号决定 */
        v   = (int32)current - 32767;
        mag = (v >= 0) ? (float)v : (float)(-v);
        /* 正数目标 = FR_MODE 方向；负数 = 反方向。
         * 20261030 的 Customer.h 是 FR_MODE=CW，所以它写死 CW/CCW；
         * 本工程对齐 20260923 后 FR_MODE=CCW，这里必须跟着 FR_MODE 走。 */
        if (v >= 0)
        {
            g_run_state.direction = FR_MODE;
        }
        else
        {
            g_run_state.direction = (FR_MODE == CW) ? CCW : CW;
        }
        g_run_state.ref       = mag / 100.0f;
    }
    else
    {
        /* 转速模式：0.01% -> rpm */
        v   = (int32)speed - 32767;
        mag = (v >= 0) ? (float)v : (float)(-v);
        if (mag > 10000.0f)
        {
            mag = 10000.0f;                     /* 钳到 ±100% */
        }

        /* 正数目标 = FR_MODE 方向；负数 = 反方向。
         * 20261030 的 Customer.h 是 FR_MODE=CW，所以它写死 CW/CCW；
         * 本工程对齐 20260923 后 FR_MODE=CCW，这里必须跟着 FR_MODE 走。 */
        if (v >= 0)
        {
            g_run_state.direction = FR_MODE;
        }
        else
        {
            g_run_state.direction = (FR_MODE == CW) ? CCW : CW;
        }

        if (mag <= 0.0f)
        {
            g_run_state.ref = 0.0f;             /* 0% = 目标 0 */
        }
        else
        {
            /* 非零指令：0.01%~100% 线性映射到 [CAN_MIN_RPM, CAN_PCT100_RPM]。
             * CAN_MIN_RPM / CAN_PCT100_RPM 都来自 Customer.h 的
             * MOTOR_SPEED_MIN_RPM / MOTOR_SPEED_MAX_RPM，
             * 所以 0.01% 不会掉到电机稳不住的转速。 */
            g_run_state.ref = CAN_MIN_RPM + (CAN_PCT100_RPM - CAN_MIN_RPM) * (mag / 10000.0f);
        }
    }
}

/* ==========================================================================
 *  八、状态帧组帧
 * --------------------------------------------------------------------------
 *  对应 20261030 CanSlave.c 的 ext_build_status()，字节定义见 CanProtocol.h：
 *      Byte0    ctrl：工作模式 + 启停 + 急停 回显
 *      Byte1-2  实际转速（1rpm，32767=0，负=反转）
 *      Byte3-4  相电流（0.01A，32767=0）
 *      Byte5-6  功率（1W，32767=0）
 *      Byte7    故障码（0xFF=无故障）
 * ========================================================================== */
static void CanMin_BuildStatus(uint8 *buf)
{
    ExtStatusMsg_t *msg;
    int16 speed;
    int16 current;
    int16 power;
    uint8 ctrl;

    msg = (ExtStatusMsg_t *)buf;
    memset(msg, 0xFF, sizeof(ExtStatusMsg_t));

    /* ---- Byte0: ctrl 回显 ---- */
    ctrl = 0;
    EXT_SET_WORKMODE(ctrl, g_run_state.work_mode);
    EXT_SET_STARTSTOP(ctrl, (g_run_state.is_started != 0) ? EXT_STARTSTOP_START : EXT_STARTSTOP_STOP);
    EXT_SET_ESTOP(ctrl, (g_run_state.is_estop != 0) ? EXT_ESTOP_ACTIVE : EXT_ESTOP_RELEASE);
    msg->ctrl.value = ctrl;

    /* ---- Byte1-2: 实际转速 ---- */
    speed = (int16)CANMIN_S2F(mcFocCtrl.SpeedFlt);
    if (mcFocCtrl.FR == CCW)
    {
        speed = -speed;                         /* 协议：负值 = 反转 */
    }
    EXT_SET_I16(msg->speed_hi, msg->speed_lo, speed + 32767);

    /* ---- Byte3-4: 相电流（Q轴电流，0.01A） ---- */
    current = (int16)(CANMIN_I2F(FOC__IQ) * 100.0f);
    EXT_SET_I16(msg->current_hi, msg->current_lo, current + 32767);

    /* ---- Byte5-6: 功率（1W） ---- */
    power = mcFocCtrl.PowerFlt;
    EXT_SET_I16(msg->power_hi, msg->power_lo, power + 32767);

    /* ---- Byte7: 故障码 ----
     * 优先上报通信超时置的 g_run_state.fault_code（0x87 = 三级/通信异常停机）；
     * 其次是电机故障（mcFaultSource 非 0 时粗报 0x80）；
     * 都没有就是 0xFF。具体故障码可在 Keil 里看 mcFaultSource。 */
    if (g_run_state.fault_code != FAULT_NONE)
    {
        msg->fault_code = g_run_state.fault_code;
    }
    else if (mcFaultSource != FaultNoSource)
    {
        msg->fault_code = FAULT_BUILD(FAULT_LEVEL_3, 0);
    }
    else
    {
        msg->fault_code = FAULT_NONE;
    }
}

/* ==========================================================================
 *  九、发送
 * --------------------------------------------------------------------------
 *  对应 20261030 的 CAN_Send()，区别是“非阻塞”：
 *     - g_canmin_tx_ready=0 表示上一帧还没发完（TXSUC=0），本周期直接跳过；
 *     - 等待超过 CAN_TX_TIMEOUT_MS 强制认为空闲，避免一次失败后永远不发；
 *     - 写 TXCR/TXID/TXDR 的字节顺序与 20261030 完全一致。
 * ========================================================================== */
static uint8 CanMin_Send(void)
{
    /* ---- 上一帧是否已发完 ---- */
    if (!g_canmin_tx_ready)
    {
        if (ReadBit(CAN_SR, TXSUC))
        {
            g_canmin_tx_ready = 1;
            g_canmin_tx_wait  = 0;
        }
        else if (g_canmin_tx_wait >= CAN_TX_TIMEOUT_MS)
        {
            g_canmin_tx_ready = 1;              /* 超时强制重发 */
            g_canmin_tx_wait  = 0;
        }
        else
        {
            g_canmin_stat.TxSkip++;
            return 0;
        }
    }

    /* ---- 控制字：IDE | RTR | DLC ---- */
    CAN_TXCR  = CanMinSend.CAN_IDE | CanMinSend.CAN_RTR | CanMinSend.CAN_DLC;

    /* ---- 29 位扩展 ID（或 11 位标准 ID） ---- */
    if (CanMinSend.CAN_IDE)
    {
        CAN_TXID0 = (uint8)(CanMinSend.EXTID >> 21);
        CAN_TXID1 = (uint8)(CanMinSend.EXTID >> 13);
        CAN_TXID2 = (uint8)(CanMinSend.EXTID >> 5);
        CAN_TXID3 = (uint8)(CanMinSend.EXTID << 3);   /* 20261030 这里也是 <<3 */
    }
    else
    {
        CAN_TXID0 = (uint8)(CanMinSend.STDID >> 3);
        CAN_TXID1 = (uint8)(CanMinSend.STDID << 5);
        CAN_TXID2 = 0;
        CAN_TXID3 = 0;
    }

    /* ---- 8 字节数据 ---- */
    CAN_TXDR0 = CanMinSend.Data0;
    CAN_TXDR1 = CanMinSend.Data1;
    CAN_TXDR2 = CanMinSend.Data2;
    CAN_TXDR3 = CanMinSend.Data3;
    CAN_TXDR4 = CanMinSend.Data4;
    CAN_TXDR5 = CanMinSend.Data5;
    CAN_TXDR6 = CanMinSend.Data6;
    CAN_TXDR7 = CanMinSend.Data7;

    /* ---- 启动发送 ---- */
    ClrBit(CAN_IFR, TXIF);
    SetBit(CAN_CR1, TXREQ);

    g_canmin_tx_ready = 0;
    g_canmin_tx_wait  = 0;
    g_canmin_stat.TxFrames++;
    return 1;
}

/* ==========================================================================
 *  十、按 ID + 数据发送一帧（对应 20261030 的 can_hw_send()）
 * --------------------------------------------------------------------------
 *  @param can_id 29 位扩展 ID（>0x7FF 自动用扩展帧）
 *  @param dat    8 字节数据
 *  @param len    必须是 8
 *  @retval 1=已提交发送，0=上一帧没发完/参数错
 * ========================================================================== */
static uint8 CanMin_HwSend(uint32 can_id, uint8 *dat, uint8 len)
{
    if ((len != CAN_FRAME_DATA_LEN) || (dat == 0))
    {
        return 0;
    }

    if (can_id > 0x7FFUL)
    {
        CanMinSend.CAN_IDE = IDE_EXT;           /* 扩展帧 */
    }
    else
    {
        CanMinSend.CAN_IDE = IDE_STD;           /* 标准帧 */
    }

    CanMinSend.CAN_RTR = RTR_DAT;               /* 数据帧 */
    CanMinSend.CAN_DLC = len;
    CanMinSend.EXTID   = can_id;
    CanMinSend.STDID   = can_id;

    CanMinSend.Data0 = dat[0];
    CanMinSend.Data1 = dat[1];
    CanMinSend.Data2 = dat[2];
    CanMinSend.Data3 = dat[3];
    CanMinSend.Data4 = dat[4];
    CanMinSend.Data5 = dat[5];
    CanMinSend.Data6 = dat[6];
    CanMinSend.Data7 = dat[7];

    return CanMin_Send();
}

/* ==========================================================================
 *  十一、初始化
 * --------------------------------------------------------------------------
 *  与 20261030 的 CAN_Init() 逐寄存器对照，只有两点不同：
 *    ① 不使能任何 CAN 中断（CAN_IER=0），20261030 是全开的；
 *    ② 不碰 P0.7（本板没有这个脚），20261030 里那两行本来就是注释掉的。
 * ========================================================================== */
void CanMin_Init(void)
{
    uint16 brp;

    /* ---- 1. 运行状态清零（对应 MotorCtrl_Init()） ---- */
    g_run_state.work_mode        = EXT_WORK_MODE_SPEED;
    g_run_state.is_started       = 0;           /* 上电不自动起机，等 CAN 的“起机” */
    g_run_state.is_estop         = 0;
    g_run_state.reversal_braking = 0;
    g_run_state.ref              = 0.0f;
    g_run_state.direction        = FR_MODE;
    g_run_state.fault_code       = FAULT_NONE;
    g_run_state.run_time         = 0;
    g_run_state.total_time       = 0;

    g_canmin_status_cnt  = 0;
    g_canmin_tx_ready    = 1;                   /* 上电允许发第一帧 */
    g_canmin_tx_wait     = 0;
    g_canmin_busoff_seen = 0;

    /* ---- 2. 位时间：BRP / SEG1 / SEG2 ---- */
    CAN_BTR0 = 0;                               /* 先清 0，规避寄存器写入 bug */
    CAN_BTR1 = 0;
    CAN_BTR2 = 0;

    brp = CAN_BRP;
    CAN_BTR0 |= (uint8)(brp >> 8);              /* BRP 高 2 位 */
    CAN_BTR1  = (uint8)brp;                     /* BRP 低 8 位 */
    CAN_BTR2 |= (uint8)(((CAN_SEG2 & 0x07) << 4) | (CAN_SEG1 & 0x0F));

    /* ---- 3. 过滤器：单过滤器模式 + 放行全部扩展帧 ----
     * 掩码全 1 = 全部“不关心”，所以任何扩展帧都能进来；
     * 真正的地址过滤在 CanMin_MsgParse() 里做（与 20261030 相同）。 */
    SetBit(CAN_CR0, FILMOD);                    /* 1 = 单过滤器模式 */
    CAN_FIR0 = 0xFF;
    CAN_FIR1 = 0xFF;
    CAN_FIR2 = 0xFF;
    CAN_FIR3 = 0xFF;
    CAN_FMR0 = 0xFF;
    CAN_FMR1 = 0xFF;
    CAN_FMR2 = 0xFF;
    CAN_FMR3 = 0xFF;

    /* ---- 4. 工作模式 ---- */
    ClrBit(CAN_CR0, CANREV);                    /* 0 = CAN2.0A/B，支持扩展帧 */
    ClrBit(CAN_CR0, AWAKMOD);                   /* 不使能唤醒模式 */
    ClrBit(CAN_CR0, SELFTST);                   /* 不使能自测模式 */
    ClrBit(CAN_CR0, LISTEN);                    /* 不使能只听模式 */

#if (CAN_CHANNEL)
    SetBit(CAN_CR0, CAN_CH);                    /* 1 = TXD->P0.5, RXD->P0.6 */
#else
    ClrBit(CAN_CR0, CAN_CH);                    /* 0 = TXD->P0.0, RXD->P0.1（本板） */
#endif

    /* ---- 5. 关闭全部 CAN 中断（本模块的关键） ---- */
    CAN_IER = 0x00;                             /* 接收/发送/错误/溢出中断全关 */
    ClrBit(CAN_CR2, ERRIE);                     /* 错误总中断也关 */
    ClrBit(CAN_IFR, RXIF | TXIF | OVIF | ARBIF | ERRIF);   /* 清残留标志 */

    /* ---- 6. 退出复位模式 + 使能 CAN ---- */
    ClrBit(CAN_CR0, RSTMOD);                    /* 0 = 退出复位，开始工作 */
    SetBit(CAN_CR0, CANEN);                     /* 1 = CAN 模块使能 */
}

/* ==========================================================================
 *  bus-off 恢复（最小版）
 * --------------------------------------------------------------------------
 *  RSTMOD 置 1：CAN 内核进复位，清发送/接收错误计数、退出 bus-off；
 *  RSTMOD 清 0：回正常模式。
 *
 *  只翻 RSTMOD，不重写 BTR/过滤器（SJA1000 兼容内核在复位模式下不会清
 *  这些配置寄存器），也不动 g_run_state / 电机状态。
 * ========================================================================== */
static void CanMin_CanRecover(void)
{
    uint8 i;

    SetBit(CAN_CR0, RSTMOD);                    /* 进复位：清错误计数、退出 bus-off */
    for (i = 0; i < 10; i++)
    {
        _nop_();                                /* 等复位请求生效（约 0.4us @24MHz） */
    }
    ClrBit(CAN_CR0, RSTMOD);                    /* 回正常模式 */

    g_canmin_tx_ready = 1;                      /* 允许发下一帧 */
    g_canmin_tx_wait  = 0;
}


/* ==========================================================================
 *  十二、1ms 轮询
 * --------------------------------------------------------------------------
 *  对应 20261030 的 TickCycle_1ms() 里那三行：
 *      CAN_MsgParse();  ext_slave_periodic_send();  AlarmMonitor_Process();
 *  这里把它们合并成一个 CanMin_Poll()：
 *      ① 收帧（最多 CAN_RX_PER_TICK 帧，读一帧释放一帧）
 *      ② 统计溢出 / bus-off
 *      ③ 每 CAN_STATUS_PERIOD_MS 发一帧状态
 * ========================================================================== */
void CanMin_Poll(void)
{
    uint8 dat[CAN_FRAME_DATA_LEN];
    uint8 n;

    /* ---- ① 收帧：NOEMP=1 表示接收 FIFO 非空 ---- */
    for (n = 0; n < CAN_RX_PER_TICK; n++)
    {
        if (!ReadBit(CAN_SR, NOEMP))
        {
            break;
        }

        CanMin_ReadFrame();                     /* 读出最老的一帧到 CanMin */
        SetBit(CAN_CR1, BUFRLS);                /* 释放该接收缓冲 */
        ClrBit(CAN_IFR, RXIF);                  /* 清接收标志（无中断，清一下更干净） */

        g_canmin_stat.RxFrames++;
        g_canmin_stat.LastId = CanMin.EXTID;    /* 调试：最近一帧 ID */

        CanMin_MsgParse();                      /* 地址过滤 + 解析指令 */
    }

    /* ---- ② 统计 ---- */
    if (ReadBit(CAN_SR, BUFOV))
    {
        g_canmin_stat.RxOvif++;                 /* 接收 FIFO 溢出 */
    }

    /* ---- ② bus-off：边沿触发立即恢复，之后每 10ms 重试 ----
     * 检测每 1ms 做一次（只读一个 SFR，开销可忽略）；
     * 恢复不能每个 1ms 都做：翻 RSTMOD 会清接收 FIFO、打断控制器同步。 */
    if (ReadBit(CAN_ESR, BOFF))
    {
        if (!g_canmin_busoff_seen)
        {
            g_canmin_busoff_seen = 1;
            g_canmin_stat.BusOffCnt++;          /* 只在“进入”bus-off 时计数 */
            g_canmin_busoff_ms = 0;
            CanMin_CanRecover();                /* 一检测到就立刻恢复 */
        }
        else if (g_canmin_busoff_ms < 0xFFFF)
        {
            g_canmin_busoff_ms++;
            if (g_canmin_busoff_ms >= 10)       /* 10ms 还没退出来，再试一次 */
            {
                g_canmin_busoff_ms = 0;
                CanMin_CanRecover();
            }
        }
    }
    else
    {
        g_canmin_busoff_seen = 0;
        g_canmin_busoff_ms   = 0;
    }

    /* ---- ③ 发送等待计时（给 CanMin_Send 做超时重发用） ---- */
    if (!g_canmin_tx_ready)
    {
        if (g_canmin_tx_wait < 0xFFFF)
        {
            g_canmin_tx_wait++;
        }
    }

    /* ---- ④ 状态帧：每 150ms 一帧 ---- */
    g_canmin_status_cnt++;
    if (g_canmin_status_cnt >= CAN_STATUS_PERIOD_MS)
    {
        g_canmin_status_cnt = 0;

        CanMin_BuildStatus(dat);
        CanMin_HwSend(CAN_STATUS_ID, dat, CAN_FRAME_DATA_LEN);
    }

#if (CAN_CMD_TIMEOUT_MS > 0)
    /* ---- ⑤ 指令通信超时：起机后 CAN_CMD_TIMEOUT_MS 内没收到有效指令 -> 停机 ----
     *   已起机且未急停时才计时；超时后 SetRun(0) 并把故障码置为
     *   “三级/通信异常停机”(0x87)，由状态帧 Byte7 上报。 */
    if (g_run_state.is_started && (!g_run_state.is_estop))
    {
        if (g_canmin_cmd_timeout < CAN_CMD_TIMEOUT_MS)
        {
            g_canmin_cmd_timeout++;
        }

        if (g_canmin_cmd_timeout >= CAN_CMD_TIMEOUT_MS)
        {
            CanMin_SetRun(0);                                   /* 撤给定 + 停机 */
            g_run_state.fault_code = FAULT_BUILD(FAULT_LEVEL_3, FAULT3_COMM_STOP);
        }
    }
    else
    {
        g_canmin_cmd_timeout = 0;
    }
#endif
}

/* ==========================================================================
 *  十三、指令 -> 电机给定
 * --------------------------------------------------------------------------
 *  对应 20261030 AddFunction.c 里 TargetRef_Process() 的 CANMODE 分支。
 *  调用位置：TickCycle_1ms() 中 TargetRef_Process() 之后、Speed_response() 之前。
 *  这一步不能省：NONEMODE 分支每 1ms 会把 mcFocCtrl.Ref 写成 0，
 *  必须在这之后把 CAN 的目标值覆盖回去。
 * ========================================================================== */
void CanMin_ApplyCmd(void)
{
    uint8 target_fr;
    int16 speed_abs;

    /* 起机/停机：直接决定 isCtrlPowOn，MC_Control() 据此走 mcInit / mcStop */
    isCtrlPowOn = g_run_state.is_started;

    /* 急停：给定清零 + 进入停机，并清掉换向状态 */
    if (g_run_state.is_estop)
    {
        mcFocCtrl.Ref = 0;
        isCtrlPowOn   = false;
        g_run_state.is_started       = 0;
        g_run_state.reversal_braking = 0;
    }

    /* 目标方向：0=CW 1=CCW */
    target_fr = (g_run_state.direction == CW) ? CW : CCW;

    if (g_run_state.reversal_braking)
    {
        /* ---- 换向减速中 ----
         * 退出条件：方向已一致 / 已停机 / 目标为 0；或者实际转速已降到 50rpm 以下。 */
        if ((target_fr == mcFocCtrl.FR) ||
            (g_run_state.is_started == 0) ||
            (g_run_state.ref == 0))
        {
            g_run_state.reversal_braking = 0;
        }
        else
        {
            speed_abs = mcFocCtrl.SpeedFlt;
            if (speed_abs < 0)
            {
                speed_abs = -speed_abs;
            }

            if (speed_abs < S_Value(50.0))
            {
                /* 减速到位，切方向，下一周期开始按新方向加速 */
                g_run_state.reversal_braking = 0;
                mcFocCtrl.FR                 = g_run_state.direction;
            }
        }
    }
    else
    {
        /* ---- 正常模式：检测是否需要启动换向减速 ----
         * 条件：目标方向变了、正在运行、没急停、目标不为 0、当前速度 >= 50rpm。 */
        if ((target_fr != mcFocCtrl.FR) &&
            g_run_state.is_started &&
            (!g_run_state.is_estop) &&
            (g_run_state.ref > 0))
        {
            speed_abs = mcFocCtrl.SpeedFlt;
            if (speed_abs < 0)
            {
                speed_abs = -speed_abs;
            }

            if (speed_abs >= S_Value(50.0))
            {
                g_run_state.reversal_braking = 1;
                mcState                      = mcStop;   /* 与 20261030 的 CANMODE 分支一致 */
            }
        }

        /* ---- 应用目标值 ----
         * 转速模式：g_run_state.ref 是 rpm，转成 Q15 给速度环。
         * 电流模式：最小版不支持（见 CanMin_SetMode），这里不会进来。 */
        if (g_run_state.work_mode == EXT_WORK_MODE_SPEED)
        {
            mcFocCtrl.Ref = S_Value(g_run_state.ref);
        }
    }
}

