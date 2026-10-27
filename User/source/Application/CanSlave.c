/**
 * @file      CanSlave.c
 * @brief     CAN 扩展帧从机协议 —— 移植自 初版\Software\User\source\Application\CanSlave.c
 * @note      
 */

#include "CanSlave.h"
#include <MyProject.h>
#include "DevConfig.h"

/************************ 用户实现的外部函数 ************************/
extern uint8 can_hw_send(uint32 can_id, const uint8_t *dat, uint8_t len);

/************************ 扩展帧从机配置 ************************/
static ExtSlaveConfig_t xdata g_ext_config;

// 多帧组装状态
typedef struct
{
    uint8_t  buf[EXT_MSG_LEN_CONFIG]; // 组装缓冲区(88字节)
    uint8_t  total_frames;            // 总帧数
    uint16_t received;                // 已收帧bitmap (bit0=frame1, ...)
    uint8_t  priority;                // 当前组装的报文优先级
    uint16_t timeout_cnt;             // 组装超时计数(ms)
} ExtAssembly_t;

static ExtAssembly_t xdata g_ext_asm;

// 发送队列(每周期只发送1帧, 避免阻塞)
#define CAN_TX_QUEUE_SIZE  32          /* 配置 12 帧 + 状态 2 + 日志 4，16 不够 */

typedef struct {
    uint32_t can_id;
    uint8_t  len;
    uint8_t  dat[CAN_FRAME_DATA_LEN];
} CanTxItem_t;

static CanTxItem_t xdata g_tx_queue[CAN_TX_QUEUE_SIZE];
static uint8_t xdata g_tx_head;
static uint8_t xdata g_tx_tail;
static uint8_t xdata g_tx_count;

// 周期发送计数器
static uint16_t xdata g_ext_heartbeat_cnt;
static uint8_t  xdata g_ext_status_cnt;
static uint16_t xdata g_ext_log_cnt;
static uint16_t xdata g_ext_cmd_timeout;   // 指令超时计数(ms)
static uint16_t xdata g_ext_runtime_cnt;   // 运行时间计数(ms)
static uint16_t xdata g_ext_posdbg_cnt;    // 位置环调试帧计数(ms)

/* 【调试】协议层计数（全部 xdata，避免挤占已经很紧的 data 段）*/
static uint16_t xdata g_dbg_rxTotal    = 0;   /* 进入 ext_slave_process_rx 的帧数 */
static uint16_t xdata g_dbg_rxBad      = 0;   /* len!=8 或 dat==NULL */
static uint16_t xdata g_dbg_rxFiltered = 0;   /* 地址不匹配 或 msg_num==0 */
static uint16_t xdata g_dbg_rxCmd      = 0;   /* 判定为指令报文(pri=2) */

/* 【调试】新增三个探针：把"被丢掉的帧"和"真正被执行的指令"都抓出来 */
uint32_t xdata g_dbg_filtId   = 0;   /* 最近一个被地址/帧号过滤掉的帧 ID */
uint16_t xdata g_dbg_cmdMsgNo = 0;   /* CMD 帧里 msg_num != 1（被静默忽略）的个数 */
uint8_t  xdata g_dbg_lastCtrl = 0;   /* 最近一次真正进入指令处理的 ctrl 字节 */

/* 【新增】配置落盘请求：1ms 任务里只置位（几微秒），真正的 EEPROM 写
 * 由主循环 ext_slave_cfg_save_pending() 执行。
 *    0 = 无请求   1 = 主机下发的配置要落盘   2 = SAVE 命令：当前配置强制落盘 */
static volatile uint8_t xdata g_cfgSaveReq = 0;
static uint8_t xdata g_cfgPending[EXT_MSG_LEN_CONFIG - 8];   /* 88B 配置正文 */

/************************ 内部辅助函数 ************************/

static uint8_t tx_queue_push(uint32_t can_id, const uint8_t *dat, uint8_t len)
{
    if (g_tx_count >= CAN_TX_QUEUE_SIZE) return 0;
    g_tx_queue[g_tx_tail].can_id = can_id;
    g_tx_queue[g_tx_tail].len    = len;
    memcpy(g_tx_queue[g_tx_tail].dat, dat, CAN_FRAME_DATA_LEN);
    g_tx_tail = (g_tx_tail + 1) % CAN_TX_QUEUE_SIZE;
    g_tx_count++;
    return 1;
}

static uint8_t tx_queue_pop(uint32_t *can_id, uint8_t *dat, uint8_t *len)
{
    if (g_tx_count == 0) return 0;
    *can_id = g_tx_queue[g_tx_head].can_id;
    *len    = g_tx_queue[g_tx_head].len;
    memcpy(dat, g_tx_queue[g_tx_head].dat, CAN_FRAME_DATA_LEN);
    g_tx_head = (g_tx_head + 1) % CAN_TX_QUEUE_SIZE;
    g_tx_count--;
    return 1;
}

static uint8_t tx_queue_available(uint8_t needed)
{
    return (CAN_TX_QUEUE_SIZE - g_tx_count) >= needed;
}

// 多帧入队: 将载荷拆分为多个CAN帧入队
static void ext_send_frames(uint8_t pri, const uint8_t *buf, uint16_t total_len)
{
    uint8_t  frame_cnt = (uint8_t)((total_len + CAN_FRAME_DATA_LEN - 1) / CAN_FRAME_DATA_LEN);
    uint8_t  i, j, len;
    uint8_t  dat[CAN_FRAME_DATA_LEN];
    uint32_t ext_id;

    if (!tx_queue_available(frame_cnt)) return;

    for (i = 0; i < frame_cnt; i++)
    {
        uint16_t offset = (uint16_t)i * CAN_FRAME_DATA_LEN;
        len = (total_len - offset >= CAN_FRAME_DATA_LEN) ? CAN_FRAME_DATA_LEN
                                                         : (uint8_t)(total_len - offset);

        memset(dat, 0xFF, CAN_FRAME_DATA_LEN);
        for (j = 0; j < len; j++)
        {
            dat[j] = buf[offset + j];
        }

        ext_id = EXTID_BUILD(pri, g_ext_config.sys_num, g_ext_config.main_cat,
                             g_ext_config.sub_cat, g_ext_config.dev_num, i + 1);
        tx_queue_push(ext_id, dat, len);
    }
}

// 单帧发送辅助（指定 DLC）
static void ext_send_single_len(uint8_t pri, const uint8_t *dat, uint8_t len)
{
    uint32_t ext_id = EXTID_BUILD(pri, g_ext_config.sys_num, g_ext_config.main_cat,
                                  g_ext_config.sub_cat, g_ext_config.dev_num, 1);
    tx_queue_push(ext_id, dat, len);
}

/************************ 指令/状态/日志 ************************/

// 解析指令报文(8字节载荷, 单帧)
static void ext_process_cmd_message(const uint8_t *buf)
{
    ExtCmdMsg_t *msg = (ExtCmdMsg_t *)buf;
    uint8_t ctrl      = msg->ctrl.value;
    uint8_t work_mode = EXT_GET_WORKMODE(ctrl);
    uint8_t startstop = EXT_GET_STARTSTOP(ctrl);
    uint8_t estop     = EXT_GET_ESTOP(ctrl);
    g_dbg_lastCtrl = ctrl;                         /* 【调试】记下真正被执行的 ctrl */
    g_ext_cmd_timeout = 0;                         /* 新协议 Byte5-6 是闭环指令，不再支持旧调试命令 */

    // 急停处理(最高优先级)
    if (estop == EXT_ESTOP_ACTIVE)
    {
        MotorCtrl_SetEstop(1);
        return;
    }
    if (estop == EXT_ESTOP_RELEASE)
    {
        MotorCtrl_SetEstop(0);
    }
    /* estop == 0：无操作，不碰急停状态 */

    // 启停处理
    if (startstop >= EXT_STARTSTOP_START && startstop <= EXT_STARTSTOP_STOP)
    {
        MotorCtrl_SetRun(startstop == EXT_STARTSTOP_START);
    }

    // 工作模式
    MotorCtrl_SetMode(work_mode);

    // 设置目标值
    /* 按"无符号"取原始值（协议：段字节11-12 目标转速 / 13-14 目标电流，
     *   零点 32767，正数会 >32767（最高位为 1）。
     *   原来用 EXT_GET_I16 有符号读，会把每一个正转指令读成大负数 —— 电机因此不动。*/
    MotorCtrl_SetTarget(EXT_GET_U16(msg->target_current_hi, msg->target_current_lo),
                        EXT_GET_U16(msg->target_speed_hi, msg->target_speed_lo),
                        EXT_GET_U16(msg->target_position_hi, msg->target_position_lo));
}

#define S_Value2F(x)   ((float)(x) / 32767.0f * MOTOR_SPEED_BASE)
#define I_Value2F(x)   ((float)(x) / 32767.0f * (float)HW_ADC_REF / ((float)HW_RSHUNT * (float)HW_AMPGAIN))
#define UDC_Value2F(x) ((float)(x) / 32767.0f * (float)HW_BOARD_VOLT_MAX)

// 构建状态报文(10字节载荷，拆两帧)
static void ext_build_status(uint8_t *buf)
{
    ExtStatusMsg_t *msg = (ExtStatusMsg_t *)buf;
    int16_t speed, current, power;
    uint8_t ctrl = 0;

    memset(msg, 0xFF, sizeof(ExtStatusMsg_t));

    EXT_SET_WORKMODE(ctrl, g_run_state.work_mode);
    EXT_SET_STARTSTOP(ctrl, g_run_state.is_started ? EXT_STARTSTOP_START : EXT_STARTSTOP_STOP);
    EXT_SET_ESTOP(ctrl, g_run_state.is_estop ? EXT_ESTOP_ACTIVE : EXT_ESTOP_RELEASE);
    msg->ctrl.value = ctrl;

    speed = (int16_t)S_Value2F(mcFocCtrl.SpeedFlt);
    EXT_SET_I16(msg->speed_hi, msg->speed_lo, speed + 32767);

    current = (int16_t)(I_Value2F(FOC__IQ) * 100.0f);   /* 0.01A/bit */
    EXT_SET_I16(msg->current_hi, msg->current_lo, current + 32767);

    power = (int16_t)(mcFocCtrl.PowerFlt);
    EXT_SET_I16(msg->power_hi, msg->power_lo, power + 32767);

    msg->fault_code = g_run_state.fault_code;

    EXT_SET_U16(msg->closed_loop_hi, msg->closed_loop_lo, MotorCtrl_GetLoopFeedback());       /* 字节18-19：转向0.01°/升降0.1%/折叠0-4 */
    msg->stall = MotorCtrl_GetStall();              /* 字节20：0/1 */
}

// 构建日志报文(32字节载荷)
static void ext_build_log(uint8_t *buf)
{
    ExtLogMsg_t *msg = (ExtLogMsg_t *)buf;
    int16_t val;

    memset(msg, 0xFF, sizeof(ExtLogMsg_t));

    val = (int16_t)S_Value2F(mcFocCtrl.SpeedFlt);
    EXT_SET_I16(msg->speed_hi, msg->speed_lo, val + 32767);

    val = (int16_t)(UDC_Value2F(mcFocCtrl.mcDcbusFlt) * 100.f);
    EXT_SET_I16(msg->voltage_hi, msg->voltage_lo, val + 32767);

    val = (int16_t)(I_Value2F(mcFocCtrl.mcADCCurrentbus) * 100.0f);
    EXT_SET_I16(msg->current_hi, msg->current_lo, val + 32767);

    val = (int16_t)(mcFocCtrl.PowerFlt);
    EXT_SET_I16(msg->power_hi, msg->power_lo, val + 32767);

    EXT_SET_I16(msg->mot_temp_hi, msg->mot_temp_lo, 32767);   // 无电机温度传感器：0℃

    val = mcFocCtrl.NTCTempNum;                    // NTC 驱动温度，0.1℃
    if ((val <= -400) || (val >= 1500))            // 未采到/无效
    {
        val = (int16_t)mcFocCtrl.MCU_TEMP * 10;    // 芯片温度 1℃ -> 0.1℃
    }
    EXT_SET_I16(msg->drv_temp_hi, msg->drv_temp_lo, val + 32767);

    EXT_SET_I16(msg->torque_hi, msg->torque_lo, 32767);       // 无转矩估算：0Nm

    val = (int16_t)(I_Value2F(FOC__IA) * 100.0f);  // Q15 -> A -> 0.01A
    EXT_SET_I16(msg->ia_hi, msg->ia_lo, val + 32767);
    val = (int16_t)(I_Value2F(FOC__IB) * 100.0f);
    EXT_SET_I16(msg->ib_hi, msg->ib_lo, val + 32767);
    val = (int16_t)(I_Value2F(FOC__IC) * 100.0f);
    EXT_SET_I16(msg->ic_hi, msg->ic_lo, val + 32767);

    EXT_SET_U32(msg->run_time[0], msg->run_time[1], msg->run_time[2], msg->run_time[3],
                g_run_state.run_time);
    EXT_SET_U32(msg->total_time[0], msg->total_time[1], msg->total_time[2], msg->total_time[3],
                g_run_state.total_time);
}

/************************ 扩展帧公开接口 ************************/

/// @brief 初始化扩展帧从站
void ext_slave_init(const ExtSlaveConfig_t *config)
{
    memset(&g_ext_config, 0, sizeof(ExtSlaveConfig_t));
    if (config)
    {
        g_ext_config.sys_num  = config->sys_num;
        g_ext_config.main_cat = config->main_cat;
        g_ext_config.sub_cat  = config->sub_cat;
        g_ext_config.dev_num  = config->dev_num;
    }

    memset(&g_ext_asm, 0, sizeof(ExtAssembly_t));
    g_tx_head = 0;
    g_tx_tail = 0;
    g_tx_count = 0;
    g_ext_heartbeat_cnt = 0;
    g_ext_status_cnt    = 0;
    g_ext_log_cnt       = 0;
    g_ext_cmd_timeout   = 0;
    g_ext_runtime_cnt   = 0;
}

/// @brief 处理配置报文
void ext_process_cfg_message(void)
{
    static uint8_t xdata resp_buf[EXT_MSG_LEN_CONFIG];   /* 96B，别放栈上 */

    if (g_ext_asm.total_frames == 1)
    {
        ExtCfgMsg_t *msg = (ExtCfgMsg_t *)g_ext_asm.buf;
        if (msg->cfg_read == 1)
        {
            /* 读配置：整段 96B 返回，data[0]=0(无操作)，data[1]=1(应答) */
            memset(resp_buf, 0xFF, sizeof(resp_buf));
            resp_buf[0] = 0;
            resp_buf[1] = 1;
            memcpy(&resp_buf[8], g_motor_cfg, sizeof(g_cfgPending));
            ext_send_frames(EXT_PRI_CONFIG, resp_buf, EXT_MSG_LEN_CONFIG);
        }
    }
    else if (g_ext_asm.total_frames == EXT_FRAME_COUNT(EXT_MSG_LEN_CONFIG))
    {
        /* 写配置：data[8..95] 是 88B 正文；先拷到 pending，主循环再落 EEPROM */
        memcpy(g_cfgPending, &g_ext_asm.buf[8], sizeof(g_cfgPending));
        g_cfgSaveReq = 1;

        /* 写应答：同样整段 96B，data[1]=1 */
        memset(resp_buf, 0xFF, sizeof(resp_buf));
        resp_buf[0] = 0;
        resp_buf[1] = 1;
        memcpy(&resp_buf[8], g_cfgPending, sizeof(g_cfgPending));
        ext_send_frames(EXT_PRI_CONFIG, resp_buf, EXT_MSG_LEN_CONFIG);
    }
}

/// @brief 处理扩展帧接收数据
void ext_slave_process_rx(uint32_t ext_id, const uint8_t *dat, uint8_t len)
{
    uint8_t pri, main_cat, sub_cat, dev_num, msg_num;

    g_dbg_rxTotal++;                                    /* 【调试】 */

    if (len != CAN_FRAME_DATA_LEN || dat == NULL)
    {
        g_dbg_rxBad++;                                  /* 【调试】 */
        return;
    }

    pri      = EXTID_GET_PRIORITY(ext_id);
    main_cat = EXTID_GET_MAINCAT(ext_id);
    sub_cat  = EXTID_GET_SUBCAT(ext_id);
    dev_num  = EXTID_GET_DEVNUM(ext_id);
    msg_num  = EXTID_GET_MSGNUM(ext_id);

    // 过滤：只处理与本机匹配的帧
    if (main_cat != g_ext_config.main_cat ||
        sub_cat  != g_ext_config.sub_cat  ||
        dev_num  != g_ext_config.dev_num  ||
        EXTID_GET_SYSNUM(ext_id) != g_ext_config.sys_num)
    {
        g_dbg_rxFiltered++;                             /* 【调试】地址不匹配被丢 */
        g_dbg_filtId = ext_id;                          /* 【调试】记下被丢的 ID */
        return;
    }

    if (msg_num == 0)
    {
        g_dbg_rxFiltered++;                             /* 【调试】 */
        g_dbg_filtId = ext_id;                          /* 【调试】记下被丢的 ID */
        return;
    }

    // 根据优先级分发处理
    if (pri == EXT_PRI_CMD)
    {
        g_dbg_rxCmd++;                                  /* 【调试】 */
        // 指令报文: 单帧(8字节载荷), msg_num=1即完整消息
        if (msg_num == 1)
        {
            ext_process_cmd_message(dat);
        }
        else
        {
            g_dbg_cmdMsgNo++;                           /* 【调试】帧号≠1，会被静默忽略 */
        }
    }
    else if (pri == EXT_PRI_CONFIG)
    {
        // 配置报文: 11帧(88字节载荷), 需要多帧组装
                if (msg_num == 1)
        {
            uint8_t read_op = (uint8_t)(dat[0] & 0x03);

            memset(&g_ext_asm, 0, sizeof(ExtAssembly_t));
            g_ext_asm.priority = pri;

            if (read_op == 1)                  /* data[0]=1：读请求，1 帧 */
            {
                g_ext_asm.total_frames = 1;
            }
            else if (read_op == 0)             /* data[0]=0：写配置，12 帧 */
            {
                g_ext_asm.total_frames = EXT_FRAME_COUNT(EXT_MSG_LEN_CONFIG);
            }
            else
            {
                return;                        /* 3：无效值 */
            }

            memcpy(g_ext_asm.buf, dat, CAN_FRAME_DATA_LEN);
            g_ext_asm.received = 0x01;
            g_ext_asm.timeout_cnt = 0;
        }
        else if ((g_ext_asm.priority == pri) && (g_ext_asm.received != 0) &&
                 (msg_num <= EXT_FRAME_COUNT(EXT_MSG_LEN_CONFIG)))
        {
            uint16_t offset = (uint16_t)(msg_num - 1) * CAN_FRAME_DATA_LEN;
            if (offset + CAN_FRAME_DATA_LEN <= sizeof(g_ext_asm.buf))
            {
                memcpy(&g_ext_asm.buf[offset], dat, CAN_FRAME_DATA_LEN);
            }
            g_ext_asm.received |= ((uint16_t)1 << (msg_num - 1));
        }

        if (g_ext_asm.received == ((uint16_t)1 << g_ext_asm.total_frames) - 1)
        {
            ext_process_cfg_message();
            g_ext_asm.received = 0;
        }
    }

    // 重置通信超时
    // g_ext_cmd_timeout = 0;
}

/// @brief 配置落盘（只能由 main.c 大循环调用；会阻塞约 50ms）
/// @note  绝不能在 1ms 任务 / CAN 中断里调用
void ext_slave_cfg_save_pending(void)
{
    uint8_t req = g_cfgSaveReq;

    if (req == 0)
    {
        return;
    }
    g_cfgSaveReq = 0;

    if (req == 1)
    {
        DevConfig_WriteParam(g_cfgPending);
    }
    else
    {
        DevConfig_ForceSave();                 /* 2：SAVE 命令，把当前配置强制落盘 */
    }
}

#if (DEBUG_UART_OR_CAN == 2)
/**
 * @brief  位置环 CAN 调试帧（标准帧 0x1F4，100ms 一帧）
 * @note   [0..1] 目标 raw
 *         [2..3] 反馈 raw
 *         [4..5] 误差 raw（目标 - 反馈，有符号）
 *         [6]    位置状态：0 IDLE 1 MOVING 2 INPOS 3 LIMIT 4 STALL 5 FAULT
 *         [7]    速度给定/10，有符号
 */
static void ext_send_position_debug(void)
{
    uint8_t f[8];
    int16_t err;
    int16_t speed10;
    uint16_t target;
    uint16_t fb;

    if (ReadBit(CAN_SR, TXING))
    {
        return;                                     /* CAN 忙，跳过 */
    }

    target = g_pos_cmd_raw;                         /* 目标 raw */
    fb     = g_pos_fb_raw;                          /* 反馈 raw */
    err    = (int16_t)((int32)target - (int32)fb);  /* 误差 raw */

    speed10 = (int16_t)(g_run_state.ref / 10.0f);   /* 速度给定 /10 */
    if (g_run_state.direction == CCW)
    {
        speed10 = -speed10;                         /* 方向取负 */
    }

    f[0] = (uint8_t)(target >> 8);
    f[1] = (uint8_t)target;
    f[2] = (uint8_t)(fb >> 8);
    f[3] = (uint8_t)fb;
    f[4] = (uint8_t)(err >> 8);
    f[5] = (uint8_t)err;
    f[6] = g_pos_state;
    f[7] = (uint8_t)((int8)speed10);

    can_hw_send(0x1F4, f, 8);

    /*
     * 第二帧 0x1F5：看位置环为什么没输出
     * [0] mcState
     * [1] mcFocCtrl.CtrlMode
     * [2..3] SpeedFlt(rpm，有符号)
     * [4..5] Iq 电流(0.01A，有符号)
     * [6] g_run_state.work_mode
     * [7] g_pos_cmd_start
     */
    {
        uint8_t g[8];
        int16_t spd;
        int16_t iq;

        spd = (int16_t)S_Value2F(mcFocCtrl.SpeedFlt);
        iq  = (int16_t)(I_Value2F(FOC__IQ) * 100.0f);

        g[0] = (uint8_t)mcState;
        g[1] = mcFocCtrl.CtrlMode;
        g[2] = (uint8_t)(spd >> 8);
        g[3] = (uint8_t)spd;
        g[4] = (uint8_t)(iq >> 8);
        g[5] = (uint8_t)iq;
        g[6] = g_run_state.work_mode;
        g[7] = g_pos_cmd_start;

        tx_queue_push(0x1F5, g, CAN_FRAME_DATA_LEN);
    }
}
#endif

/// @brief 定期发送扩展帧（1ms 周期调用）
void ext_slave_periodic_send(void)
{
    uint8_t  buf[EXT_MSG_LEN_LOG];
    uint32_t can_id;
    uint8_t  tx_dat[CAN_FRAME_DATA_LEN];
    uint8_t  tx_len;

    if (g_tx_count > 0)
    {
        if (tx_queue_pop(&can_id, tx_dat, &tx_len))
        {
            can_hw_send(can_id, tx_dat, tx_len);
        }
    }

    // 多帧组装超时检测(500ms未完成则丢弃)
    if (g_ext_asm.received != 0)
    {
        g_ext_asm.timeout_cnt++;
        if (g_ext_asm.timeout_cnt >= 500)
        {
            memset(&g_ext_asm, 0, sizeof(ExtAssembly_t));
        }
    }

    /* 指令超时：EXT_CMD_TIMEOUT_MS 内没收到本机命令就停机 */
    if (g_run_state.is_started)
    {
        if (g_ext_cmd_timeout < EXT_CMD_TIMEOUT_MS)
        {
            g_ext_cmd_timeout++;
        }
        if (g_ext_cmd_timeout >= EXT_CMD_TIMEOUT_MS)
        {
            g_run_state.is_started = 0;     /* 速度/开环停 */
            g_run_state.ref        = 0.0f;
            g_pos_cmd_start        = 0;     /* 位置环也停 */
            g_pos_cmd_raw          = 0xFFFF;/* 清位置目标 */
            g_motor_stall          = 0;
        }
    }
    else
    {
        g_ext_cmd_timeout = 0;
    }

    // 运行时间统计(每1000ms累加1s)
    g_ext_runtime_cnt++;
    if (g_ext_runtime_cnt >= 1000)
    {
        g_ext_runtime_cnt = 0;
        if (g_run_state.is_started)
        {
            g_run_state.run_time++;
        }
        g_run_state.total_time++;      // 上电即计
    }

    // 心跳报文: 每500ms发送(0字节载荷, 仅CAN ID标识, 数据填0xFF)
    g_ext_heartbeat_cnt++;
    if (g_ext_heartbeat_cnt >= 500)
    {
        uint8_t dummy[CAN_FRAME_DATA_LEN] = {0};
        g_ext_heartbeat_cnt = 0;
        ext_send_single_len(EXT_PRI_HEARTBEAT, dummy, 0);   /* 0015 心跳数据长 0，DLC 0 */
    }

    // 状态报文: 每150ms发送(单帧)
    g_ext_status_cnt++;
    if (g_ext_status_cnt >= (100 + 50))
    {
        g_ext_status_cnt = 0;
        ext_build_status(buf);
        ext_send_frames(EXT_PRI_STATUS, buf, EXT_MSG_LEN_STATUS);  /* 11字节 -> 2帧(8+3) */
    }

#if (DEBUG_UART_OR_CAN == 2)
    /* 位置环调试帧：每100ms一帧 */
    g_ext_posdbg_cnt++;
    if (g_ext_posdbg_cnt >= 100)
    {
        g_ext_posdbg_cnt = 0;
        ext_send_position_debug();
    }
#endif

    // 日志报文: 每1100ms发送(4帧)
    g_ext_log_cnt++;
    if (g_ext_log_cnt >= (1000 + 100))
    {
        g_ext_log_cnt = 0;
        ext_build_log(buf);
        ext_send_frames(EXT_PRI_LOG, buf, EXT_MSG_LEN_LOG);
    }
}

/// @brief 设置扩展帧故障码
void ext_set_fault(FaultLevel_e level, uint8_t cod)
{
    g_run_state.fault_code = FAULT_BUILD(level, cod);
}

/// @brief 清除扩展帧故障码
void ext_clear_fault(void)
{
    g_run_state.fault_code = FAULT_NONE;
}

/* ==================== 【调试用】协议层状态打印 ====================
 * 由 main.c 大循环每 500ms 调一次。printf 阻塞，禁止放进中断 / 1ms 任务。
 * 硬件/寄存器层的计数见 CAN.c 的 CAN_DebugPrint()。
 *
 * 与 CAN_DebugPrint() 配合分诊（从头往后看，哪一步断了就是哪一层的问题）：
 *   [CAN] rx 涨  → 硬件收到了帧
 *   [CAN] pop 涨 → 1ms 任务把它从环形缓冲取出来了
 *   [CAN-S] in 涨      → 帧进了协议层
 *   [CAN-S] cmd 涨     → 被当成指令报文处理了（CAN 链路到此为止全程正常）
 *   cmd 涨但电机不动   → 问题在指令解析 / 控制层，跟 CAN 无关
 */
void ext_slave_debug_print(void)
{
    /* --- 接收 ------------------------------------------------------------------------
     * in   = g_dbg_rxTotal    : 进入 ext_slave_process_rx() 的帧数
     *                           （= CAN_MsgParse 出队、且 CAN_IDE 判定为扩展帧）
     *                           正常：跟 [CAN] pop 同步涨
     *                           若 [CAN] pop 在涨而 in 不涨 => CAN_MsgParse 里 IDE 判断没进
     *                           （收到的帧被当成标准帧丢了），去看 [CAN] 那行的 ide 字段
     * bad  = g_dbg_rxBad      : len!=8 或 dat==NULL 被丢弃的帧数
     *                           正常 0；不为 0 说明主机发的 DLC 不是 8
     * filt = g_dbg_rxFiltered : 被地址过滤丢掉的帧数，两种情况都会算进来：
     *                             ① 主类别/副类别与本机不符，或 设备号既不是本机也不是 0(广播)
     *                             ② msg_num == 0（非法帧号）
     *                           正常 0；涨而 cmd 不涨 => 主机的 ID 和本机地址对不上，
     *                           本机地址在 main.c 的 ext_cfg：sys=0 / 主类别=11 / 副类别=15 / 设备号=1
     *                           （设备号填 0 表示广播，也会被接收）
     * cmd  = g_dbg_rxCmd      : 判定为指令报文(pri=2)并进入 ext_process_cmd_message() 的帧数
     *                           正常：跟 in 同步涨
     *                           ★注意：这里只统计"进入了指令处理函数"，
     *                             并不保证没被里面的调试钩子(db g_cmd)吞掉
     * -------------------------------------------------------------------------------------- */
    printf("[CAN-S] in=%u bad=%u filt=%u cmd=%u\r\n",
           g_dbg_rxTotal, g_dbg_rxBad, g_dbg_rxFiltered, g_dbg_rxCmd);

    /* --- 发送队列 + 三个上报节拍（实参统一 (uint16) + %u，st 是 uint8_t 必须显式转）------------
     * txQ = g_tx_count          : 发送队列里还没发出去的帧数（每 1ms 弹 1 帧发）
     *                             正常 0~2；持续 >5 说明入队快于出队，或 CAN_Send 被卡住
     * hb  = g_ext_heartbeat_cnt : 心跳节拍计数，满 500 发一帧心跳。取值 0~499
     *                             ★打印本身就是"每 500 次 1ms 调一次"，
     *                               所以打印时刻它恒为 0 —— 这是正常的，不是故障
     * st  = g_ext_status_cnt    : 状态节拍计数(uint8)，满 150 发一帧状态。取值 0~149
     *                             打印时刻会呈 0 / 50 / 100 循环（每次 +500 对 150 取模）
     * lg  = g_ext_log_cnt       : 日志节拍计数，满 1100 发一帧日志(4 帧)。取值 0~1099
     *                             打印时刻每次 +500（对 1100 取模）
     * 三个节拍只要在持续变化 = 1ms 任务(TickCycle_1ms)在正常跑；
     * 若连续多行完全不动 => 1ms 任务停了（主循环卡住 / 看门狗复位）
     * -------------------------------------------------------------------------------------- */
    printf("[CAN-S] txQ=%u hb=%u st=%u lg=%u\r\n",
           (uint16)g_tx_count, g_ext_heartbeat_cnt, (uint16)g_ext_status_cnt, g_ext_log_cnt);

    /* --- 配置报文多帧组装状态 ------------------------------------------------------------------
     * asmR = g_ext_asm.received : 正在组装的配置报文的"已收帧位图"
     *                             （bit0=第1帧、bit1=第2帧…配置报文共 11 帧=bit0~bit10）
     *                             正常 0（没在组装）；非 0 = 主机正在下发 11 帧配置报文，
     *                             500ms 内没收齐会被整体丢弃（asmR 归 0）
     * -------------------------------------------------------------------------------------- */
    printf("[CAN-S] asmR=%u\r\n", (uint16)g_ext_asm.received);

    /* ★ 新增三个探针（用来定位"状态帧 Byte0 为什么会跳"）：
     *   filtId  : 最近一个被"地址不匹配 / 帧号=0"过滤掉的帧 ID
     *             （形如 xx2cf080 就是帧号=0；若是别的地址说明有别的设备号在发）
     *   cmdNo   : CMD 帧里 msg_num≠1 的个数 —— 这类帧会被【静默忽略】
     *             （比如 ID 打成 042CF082 / 042CF083 的指令，只涨 cmd 不生效）
     *   lastCtrl: 最近一次"真正进入指令处理"的 ctrl 字节（十进制）
     *             10=0x0A 转速+起机 / 18=0x12 转速+停机 / 34=0x22 转速+急停 / 74=0x4A 起机+非急停 */
    printf("[CAN-S] filtId=%lx cmdNo=%u lastCtrl=%u\r\n",
           g_dbg_filtId, (uint16)g_dbg_cmdMsgNo, (uint16)g_dbg_lastCtrl);
}
