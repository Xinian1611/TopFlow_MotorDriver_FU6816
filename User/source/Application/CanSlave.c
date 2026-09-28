#include "CanSlave.h"
#include <Myproject.h>
#include "DevConfig.h"
#include "AlarmMonitor.h"


/************************ 用户实现的外部函数 ************************/
extern uint8 can_hw_send(uint32_t can_id, const uint8_t *dat, uint8_t len);

/************************ 扩展帧协议实现 ************************/
static ExtSlaveConfig_t xdata g_ext_config;

// 多帧组装状态
typedef struct
{
    uint8_t buf[EXT_MSG_LEN_CONFIG]; // 组装缓冲区(最大96字节用于配置报文)
    uint8_t total_frames;            // 总帧数
    uint16_t received;               // 已收到的帧bitmap (bit0=frame1, bit1=frame2, ...)
    uint8_t priority;                // 当前组装的报文优先级
    uint16_t timeout_cnt;            // 组装超时计数(ms)
} ExtAssembly_t;

static ExtAssembly_t xdata g_ext_asm;

// 发送队列(每周期只发送1帧, 避免阻塞)
#define CAN_TX_QUEUE_SIZE  16

typedef struct {
    uint32_t can_id;
    uint8_t dat[CAN_FRAME_DATA_LEN];
} CanTxItem_t;

static CanTxItem_t xdata g_tx_queue[CAN_TX_QUEUE_SIZE];
static uint8_t xdata g_tx_head = 0;
static uint8_t xdata g_tx_tail = 0;
static uint8_t xdata g_tx_count = 0;

// 周期发送计数器
static uint16_t xdata g_ext_heartbeat_cnt = 0;
static uint8_t xdata g_ext_status_cnt = 0;
static uint16_t xdata g_ext_log_cnt = 0;
// static uint16_t xdata g_ext_perception_cnt = 0;
static uint16_t xdata g_ext_cmd_timeout = 0; // 指令超时计数(ms)
static uint16_t xdata g_ext_runtime_cnt = 0; // 运行时间计数(ms)

/************************ 内部辅助函数 ************************/

static uint8_t tx_queue_push(uint32_t can_id, const uint8_t *dat)
{
    if (g_tx_count >= CAN_TX_QUEUE_SIZE) return 0;
    g_tx_queue[g_tx_tail].can_id = can_id;
    memcpy(g_tx_queue[g_tx_tail].dat, dat, CAN_FRAME_DATA_LEN);
    g_tx_tail = (g_tx_tail + 1) % CAN_TX_QUEUE_SIZE;
    g_tx_count++;
    return 1;
}

static uint8_t tx_queue_pop(uint32_t *can_id, uint8_t *dat)
{
    if (g_tx_count == 0) return 0;
    *can_id = g_tx_queue[g_tx_head].can_id;
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
    uint8_t frame_cnt = (uint8_t)((total_len + CAN_FRAME_DATA_LEN - 1) / CAN_FRAME_DATA_LEN);
    uint8_t i, j, len;
    uint8_t dat[CAN_FRAME_DATA_LEN];
    uint32_t ext_id;

    if (!tx_queue_available(frame_cnt)) return;

    for (i = 0; i < frame_cnt; i++)
    {
        uint16_t offset = (uint16_t)i * CAN_FRAME_DATA_LEN;
        len = (total_len - offset >= CAN_FRAME_DATA_LEN) ? CAN_FRAME_DATA_LEN : (uint8_t)(total_len - offset);

        memset(dat, 0xFF, CAN_FRAME_DATA_LEN);
        for (j = 0; j < len; j++)
        {
            dat[j] = buf[offset + j];
        }

        ext_id = EXTID_BUILD(pri, g_ext_config.sys_num, g_ext_config.main_cat,
                             g_ext_config.sub_cat, g_ext_config.dev_num, i + 1);
        tx_queue_push(ext_id, dat);
    }
}

// 单帧发送辅助
static void ext_send_single(uint8_t pri, const uint8_t *dat)
{
    uint32_t ext_id = EXTID_BUILD(pri, g_ext_config.sys_num, g_ext_config.main_cat,
                                  g_ext_config.sub_cat, g_ext_config.dev_num, 1);
    tx_queue_push(ext_id, dat);
}

// 解析指令报文(8字节载荷, 单帧)
static void ext_process_cmd_message(const uint8_t *buf)
{
    ExtCmdMsg_t *msg = (ExtCmdMsg_t *)buf;
    uint8_t ctrl = msg->ctrl.value;
    uint8_t work_mode = EXT_GET_WORKMODE(ctrl);
    uint8_t startstop = EXT_GET_STARTSTOP(ctrl);
    uint8_t estop = EXT_GET_ESTOP(ctrl);
    uint8_t dbg_cmd = msg->reserved[0];
    int16_t dbg_param = EXT_GET_I16(msg->reserved[1], msg->reserved[2]);

    // 调试指令处理(最高优先级, 不依赖启停状态)
    //if (dbg_cmd != EXT_DBG_CMD_NONE)
    if (dbg_cmd >= EXT_DBG_CMD_SET_KP && dbg_cmd <= EXT_DBG_CMD_SAVE)
    {
        switch (dbg_cmd)
        {
        case EXT_DBG_CMD_SET_KP:
            if (g_run_state.work_mode == EXT_WORK_MODE_SPEED)
            {
                g_motor_cfg->speed_kp = (uint16_t)dbg_param;
                PI1_KP = g_motor_cfg->speed_kp;
            }
            else
            {
                FOC_QKP = dbg_param;
            }
            break;

        case EXT_DBG_CMD_SET_KI:
            if (g_run_state.work_mode == EXT_WORK_MODE_SPEED)
            {
                g_motor_cfg->speed_ki = (uint16_t)dbg_param;
                PI1_KI = g_motor_cfg->speed_ki;
            }
            else
            {
                FOC_QKI = dbg_param;
            }
            break;

        case EXT_DBG_CMD_SET_KD:
            g_motor_cfg->speed_kd = (uint16_t)dbg_param;
            break;

        case EXT_DBG_CMD_SET_ALL:
            g_motor_cfg->speed_kp = (uint16_t)((uint16_t)msg->reserved[1] << 8);
            g_motor_cfg->speed_ki = (uint16_t)((uint16_t)msg->reserved[2] << 8);
            PI1_KP = g_motor_cfg->speed_kp;
            PI1_KI = g_motor_cfg->speed_ki;
            break;

        case EXT_DBG_CMD_SAVE:
            DevConfig_WriteParam(g_motor_cfg);
            break;

        default:
            break;
        }
        return;
    }

    // 急停处理(最高优先级)
    MotorCtrl_SetEstop(estop == EXT_ESTOP_ACTIVE);
    if (estop == EXT_ESTOP_ACTIVE)
    {
        return;
    }

    // 启停处理
    if (startstop >= EXT_STARTSTOP_START && startstop <= EXT_STARTSTOP_STOP)
    {
        MotorCtrl_SetRun(startstop == EXT_STARTSTOP_START);
    }

    // 工作模式
    MotorCtrl_SetMode(work_mode);

    // 设置目标值
    MotorCtrl_SetTarget(EXT_GET_I16(msg->target_current_hi, msg->target_current_lo),
                        EXT_GET_I16(msg->target_speed_hi, msg->target_speed_lo));

}
#define S_Value2F(x) ((float)(x) / 32767.0f * MOTOR_SPEED_BASE)
#define I_Value2F(x) ((float)(x) / 32767.0f * (float)HW_ADC_REF / ((float)HW_RSHUNT * (float)HW_AMPGAIN))
#define UDC_Value2F(x) ((float)(x) / 32767.0f * (float)HW_BOARD_VOLT_MAX)
// 构建状态报文(8字节载荷)
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

    // speed = (int16_t)S_Value2F(mcFocCtrl.SpeedFlt);
    // EXT_SET_I16(msg->speed_hi, msg->speed_lo, speed + 32767);
    speed = (int16_t)S_Value2F(mcFocCtrl.SpeedFlt);
    if (mcFocCtrl.FR == CCW) speed = -speed;          /* 协议：负值=反转转速 */
    EXT_SET_I16(msg->speed_hi, msg->speed_lo, speed + 32767);
    // current = (int16_t)(I_Value2F(FOC__IQ) * 10.0f);
    current = (int16_t)(I_Value2F(FOC__IQ) * 100.0f);
    EXT_SET_I16(msg->current_hi, msg->current_lo, current + 32767);

    power = (int16_t)(mcFocCtrl.PowerFlt);
    EXT_SET_I16(msg->power_hi, msg->power_lo, power + 32767);

    // msg->fault_code = g_run_state.fault_code;
    msg->fault_code = AlarmMonitor_GetActiveFault();   /* 多故障轮询上报 */
}

// 构建日志报文(32字节载荷)
static void ext_build_log(uint8_t *buf)
{
    ExtLogMsg_t *msg = (ExtLogMsg_t *)buf;
    int16_t val;

    memset(msg, 0xFF, sizeof(ExtLogMsg_t));

    val = (int16_t)S_Value2F(mcFocCtrl.SpeedFlt);
    if (mcFocCtrl.FR == CCW) val = -val;              /* 协议：负值=反转转速 */
    EXT_SET_I16(msg->speed_hi, msg->speed_lo, val + 32767);

    val = (int16_t)(UDC_Value2F(mcFocCtrl.mcDcbusFlt) * 100.f);
    EXT_SET_I16(msg->voltage_hi, msg->voltage_lo, val + 32767);

    val = (int16_t)(I_Value2F(mcFocCtrl.mcADCCurrentbus) * 100.0f);
    EXT_SET_I16(msg->current_hi, msg->current_lo, val + 32767);

    val = (int16_t)(mcFocCtrl.PowerFlt);
    EXT_SET_I16(msg->power_hi, msg->power_lo, val + 32767);

    EXT_SET_I16(msg->mot_temp_hi, msg->mot_temp_lo, 32767);

    // val = (int16_t)(mcFocCtrl.MCU_TEMP);
    val = (int16_t)(mcFocCtrl.MCU_TEMP * 10);
    EXT_SET_I16(msg->drv_temp_hi, msg->drv_temp_lo, val + 32767);

    EXT_SET_I16(msg->torque_hi, msg->torque_lo, 32767);

    // val = mcFocCtrl.Max_ia;
    // EXT_SET_I16(msg->ia_hi, msg->ia_lo, val + 32767);
    // val = mcFocCtrl.Max_ib;
    // EXT_SET_I16(msg->ib_hi, msg->ib_lo, val + 32767);
    // val = mcFocCtrl.Max_ic;
    // EXT_SET_I16(msg->ic_hi, msg->ic_lo, val + 32767);
    val = (int16_t)(I_Value2F(FOC__IA) * 100.0f);
    EXT_SET_I16(msg->ia_hi, msg->ia_lo, val + 32767);
    val = (int16_t)(I_Value2F(FOC__IB) * 100.0f);
    EXT_SET_I16(msg->ib_hi, msg->ib_lo, val + 32767);
    val = (int16_t)(I_Value2F(FOC__IC) * 100.0f);
    EXT_SET_I16(msg->ic_hi, msg->ic_lo, val + 32767);

    EXT_SET_U32(msg->run_time[0], msg->run_time[1], msg->run_time[2], msg->run_time[3], g_run_state.run_time);
    EXT_SET_U32(msg->total_time[0], msg->total_time[1], msg->total_time[2], msg->total_time[3], g_run_state.total_time);
}

// static void ext_build_percep(uint8_t *buf)
// {
//     ExtPecerpMsg_t *msg = (ExtPecerpMsg_t *)buf;
//     msg->speedKp = PI1_KP;
//     msg->speedKi = PI1_KI;
//     msg->targetSpeed = (uint16_t)((g_run_state.direction == 1) * -1 * g_run_state.ref);
//     msg->realSpeed = (uint16_t)S_Value2F(mcFocCtrl.SpeedFlt);
//     msg->currentA = FOC__IA;
//     msg->currentB = FOC__IB;
//     msg->currentC = FOC__IC;
//     msg->currentBus = (uint16_t)(I_Value2F(mcFocCtrl.mcADCCurrentbus) * 100.f);
//     msg->speedPiOutput = FOC_IQREF;
//     msg->currentPiOutput = (uint16_t)(UDC_Value2F(FOC__UD) * 10.f);
//     msg->focQFlag = 0;
//     msg->focQSpeed = 0;
//     msg->focQCurrent = FOC__IQ;
// }

/************************ 扩展帧公开接口 ************************/

/// @brief 初始化扩展帧从站
/// @param config 扩展帧从站配置指针
/// @retval None
void ext_slave_init(const ExtSlaveConfig_t *config)
{
    memset(&g_ext_config, 0, sizeof(ExtSlaveConfig_t));
    if (config)
    {
        g_ext_config.sys_num = config->sys_num;
        g_ext_config.main_cat = config->main_cat;
        g_ext_config.sub_cat = config->sub_cat;
        g_ext_config.dev_num = config->dev_num;
    }

    memset(&g_run_state, 0, sizeof(MotorRunState_t));
    g_run_state.fault_code = FAULT_NONE;
    g_run_state.work_mode = EXT_WORK_MODE_SPEED;

    memset(&g_ext_asm, 0, sizeof(ExtAssembly_t));
    g_tx_head = 0;
    g_tx_tail = 0;
    g_tx_count = 0;
    g_ext_heartbeat_cnt = 0;
    g_ext_status_cnt = 0;
    g_ext_log_cnt = 0;
    g_ext_cmd_timeout = 0;
    g_ext_runtime_cnt = 0;
}

/// @brief 处理配置报文
/// @param None
/// @retval None
void ext_process_cfg_message(void)
{
    uint8_t resp_buf[EXT_MSG_LEN_CONFIG];

    if (g_ext_asm.total_frames == 1)
    {
        // 读取配置参数报文(单帧)
        ExtCfgMsg_t *msg = (ExtCfgMsg_t *)g_ext_asm.buf;
        if (msg->cfg_read == 1)
        {
            // 读取配置参数请求
            memset(resp_buf, 0xFF, sizeof(resp_buf));
            memcpy(resp_buf, g_motor_cfg, EXT_MSG_LEN_CONFIG);
            ext_send_frames(EXT_PRI_CONFIG, resp_buf, EXT_MSG_LEN_CONFIG);
        }
    }
    else
    {
        // 配置参数报文(多帧)
        if (g_ext_asm.total_frames == EXT_FRAME_COUNT(EXT_MSG_LEN_CONFIG))
        {
            // 保存参数
            DevConfig_WriteParam(g_ext_asm.buf);

            // 发送配置参数确认
            memset(resp_buf, 0xFF, sizeof(resp_buf));
            resp_buf[1] = 1;
            // 应答
            ext_send_frames(EXT_PRI_CONFIG, resp_buf, EXT_MSG_LEN_CFG_RSP);
        }
    }
}

/// @brief 处理扩展帧接收数据
/// @param ext_id 扩展帧仲裁ID
/// @param dat 扩展帧数据指针
/// @param len 扩展帧数据长度
void ext_slave_process_rx(uint32_t ext_id, const uint8_t *dat, uint8_t len)
{
    uint8_t pri, main_cat, sub_cat, dev_num, msg_num;

    if (len != CAN_FRAME_DATA_LEN || dat == NULL)
    {
        return;
    }

    pri = EXTID_GET_PRIORITY(ext_id);
    main_cat = EXTID_GET_MAINCAT(ext_id);
    sub_cat = EXTID_GET_SUBCAT(ext_id);
    dev_num = EXTID_GET_DEVNUM(ext_id);
    msg_num = EXTID_GET_MSGNUM(ext_id);

    // 过滤：只处理与本机匹配的帧
    if (main_cat != g_ext_config.main_cat ||
        sub_cat != g_ext_config.sub_cat ||
        (dev_num != g_ext_config.dev_num && dev_num != 0))
    {
        return;
    }

    if (msg_num == 0)
        return;

    // 根据优先级分发处理
    if (pri == EXT_PRI_CMD)
    {
        // 指令报文: 单帧(8字节载荷), msg_num=1即完整消息
        if (msg_num == 1)
        {
            ext_process_cmd_message(dat);
        }
    }
    else if (pri == EXT_PRI_CONFIG)
    {
        // 配置报文: 11帧(88字节载荷), 需要多帧组装\读取配置参数报文
        if (msg_num == 1)
        {
            memset(&g_ext_asm, 0, sizeof(ExtAssembly_t));
            g_ext_asm.priority = pri;
            if (dat[2] == 0xFF)
                g_ext_asm.total_frames = 1; // 读取配置参数报文, 1帧
            else
                g_ext_asm.total_frames = EXT_FRAME_COUNT(EXT_MSG_LEN_CONFIG); // 读取配置参数报文, 11帧

            memcpy(g_ext_asm.buf, dat, CAN_FRAME_DATA_LEN);
            g_ext_asm.received = 0x01;
            g_ext_asm.timeout_cnt = 0;
        }
        else if (g_ext_asm.priority == pri && g_ext_asm.received != 0)
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
    g_ext_cmd_timeout = 0;
}

/// @brief 定期发送扩展帧
/// @param None
/// @param None
void ext_slave_periodic_send(void)
{
    uint8_t buf[EXT_MSG_LEN_LOG];
    uint32_t can_id;
    uint8_t tx_dat[CAN_FRAME_DATA_LEN];

    // 发送队列处理: 每周期仅发送1帧, 避免阻塞
    if (g_tx_count > 0)
    {
        if (tx_queue_pop(&can_id, tx_dat))
        {
            can_hw_send(can_id, tx_dat, CAN_FRAME_DATA_LEN);
        }
    }

    // 多帧组装超时检测(50ms未完成则丢弃)
    if (g_ext_asm.received != 0)
    {
        g_ext_asm.timeout_cnt++;
        if (g_ext_asm.timeout_cnt >= 500)
        {
            memset(&g_ext_asm, 0, sizeof(ExtAssembly_t));
        }
    }

    // 指令报文通信超时检测
    // if (g_run_state.is_started)
    // {
    //     g_ext_cmd_timeout++;
    //     if (g_ext_cmd_timeout >= EXT_CMD_TIMEOUT_MS)
    //     {
    //         ext_set_fault(FAULT_LEVEL_3, FAULT3_COMM_STOP);
    //         g_run_state.is_started = 0;
    //         g_run_state.ref = 0;
    //     }
    // }

    // 运行时间统计(每1000ms累加1s)

    g_ext_runtime_cnt++;
    if (g_ext_runtime_cnt >= 1000)
    {
        g_ext_runtime_cnt = 0;
        if (g_run_state.is_started)
        {
            g_run_state.run_time++;
        }

        g_run_state.total_time++;
    }

    // 心跳报文: 每500ms发送(0字节载荷, 仅CAN ID标识, 数据填充0xFF)
    g_ext_heartbeat_cnt++;
    if (g_ext_heartbeat_cnt >= 500)
    {
        uint8_t dummy[CAN_FRAME_DATA_LEN] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
        g_ext_heartbeat_cnt = 0;
        ext_send_single(EXT_PRI_HEARTBEAT, dummy);
    }

    // 状态报文: 每100ms发送(单帧)
    g_ext_status_cnt++;
    if (g_ext_status_cnt >= (100 + 50))
    {
        g_ext_status_cnt = 0;
        ext_build_status(buf);
        ext_send_single(EXT_PRI_STATUS, buf);
    }

    // 日志报文: 每1000ms发送(4帧)
    g_ext_log_cnt++;
    if (g_ext_log_cnt >= (1000 + 100))
    {
        g_ext_log_cnt = 0;
        ext_build_log(buf);
        ext_send_frames(EXT_PRI_LOG, buf, EXT_MSG_LEN_LOG);
    }

    // // 感知报文: 每100ms发送
    // g_ext_perception_cnt++;
    // if (g_ext_perception_cnt >= (100 + 60))
    // {
    //     g_ext_perception_cnt = 0;
    //     ext_build_percep(buf);
    //     ext_send_frames(EXT_PRI_PERCEP, buf, EXT_MSG_LEN_LOG);
    // }
}

/// @brief 设置扩展帧故障码
/// @param level 故障等级
/// @param cod 故障代码
void ext_set_fault(FaultLevel_e level, uint8_t cod)
{
    g_run_state.fault_code = FAULT_BUILD(level, cod);
}

/// @brief 清除扩展帧故障码
/// @param None
/// @param None
void ext_clear_fault(void)
{
    g_run_state.fault_code = FAULT_NONE;
}