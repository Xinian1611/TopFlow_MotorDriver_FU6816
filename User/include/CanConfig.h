/**
 * @file      CanConfig.h
 * @brief     最小 CAN 功能验证模块 —— 全部可调参数集中在这里
 *
 * ============================================================================
 *  一句话说明
 * ----------------------------------------------------------------------------
 *  本模块只做两件事：
 *     ① 收“指令报文”0x042CF081（8 字节），解析成 g_run_state 里的电机目标；
 *     ② 发“状态报文”0x062CF081（8 字节），150ms 一帧。
 *  不使用 CAN 中断（CAN_IER 全 0），由 1ms 任务轮询收发。
 *  原 CAN.c / CAN.h / Interrupt.c 一行未改，CAN_INT 仍然存在但不会触发。
 * ============================================================================
 *
 *  改参数只改本文件，不要动 CanMin.c。
 */

#ifndef __CAN_CONFIG_H__
#define __CAN_CONFIG_H__

/* ==========================================================================
 *  一、模块总开关
 * ========================================================================== */
/* 1 = 启用本模块
 * 0 = CanMin.c 编译成三个空函数（main.c / AddFunction.c 里的调用不用删，
 *     电机完全按原 NONEMODE 逻辑跑，方便做“有 CAN / 无 CAN”的对比实验） */
#define CAN_ENABLE                  (1)

/* ==========================================================================
 *  二、引脚与位时间
 * ========================================================================== */
/*
 * FU6816L 新板（本板）：
 *     CAN0_TX = 引脚17 = P0.0
 *     CAN0_RX = 引脚18 = P0.1
 *     -> CAN_CR0.CAN_CH 必须为 0
 *
 * 旧板 FU6866Q：
 *     CAN0_TX = 引脚41 = P0.5
 *     CAN0_RX = 引脚42 = P0.6
 *     -> CAN_CH = 1
 *
 * ⚠ 本板 P0.5 / P0.6 是 EEPROM 的 I2C，绝对不能置 1，否则 CAN 和 EEPROM 抢引脚。
 */
#define CAN_CHANNEL                 (0)

/*
 * 位时间：24MHz 下时间单元 tq = (BRP+1)/24MHz，一帧 8 个 tq
 *     250kbps : BRP=11, SEG2=0, SEG1=5   (采样点 87.5%)  ← 本工程使用
 *     500kbps : BRP=5,  SEG2=0, SEG1=5
 *     1Mbps   : BRP=3,  SEG2=0, SEG1=3
 * ⚠ 改波特率必须和主机一致，否则一帧都收不到。
 */
#define CAN_BRP                     (11)
#define CAN_SEG1                    (5)
#define CAN_SEG2                    (0)

/* ==========================================================================
 *  三、本机地址（29 位扩展 ID 的字段）
 * ========================================================================== */
/* 本机：系统编号=0（主系统）、主类别=11（动力驱动引擎）、副类别=15（拖钓电机）、设备号=1 */
#define CAN_SYS_NUM                 (0)
#define CAN_MAIN_CAT                (11)
#define CAN_SUB_CAT                 (15)
#define CAN_DEV_NUM                 (1)

/* ==========================================================================
 *  四、报文 ID（用 CanProtocol.h 的 EXTID_BUILD 拼出来）
 * ========================================================================== */
/*
 * 指令帧：优先级2 | 系统0 | 主类别11 | 副类别15 | 设备1 | 报文号1
 *         = 0x042CF081
 * 广播帧：设备号填 0
 *         = 0x042CF001
 * 状态帧：优先级3 | ... 同上 ...
 *         = 0x062CF081
 */
#define CAN_CMD_ID          EXTID_BUILD(EXT_PRI_CMD,    CAN_SYS_NUM, CAN_MAIN_CAT, CAN_SUB_CAT, CAN_DEV_NUM, 1)
#define CAN_CMD_ID_BCAST    EXTID_BUILD(EXT_PRI_CMD,    CAN_SYS_NUM, CAN_MAIN_CAT, CAN_SUB_CAT, 0,           1)
#define CAN_STATUS_ID       EXTID_BUILD(EXT_PRI_STATUS, CAN_SYS_NUM, CAN_MAIN_CAT, CAN_SUB_CAT, CAN_DEV_NUM, 1)

/* ==========================================================================
 *  五、报文周期与保护
 * ========================================================================== */
/* 状态帧周期（ms）。协议规定 150ms，和 20261030 的 ext_slave_periodic_send 一致。
 * 想在示波器上看得更密，可以改成 100 或 50（改小会增加总线负载）。 */
#define CAN_STATUS_PERIOD_MS        (100)

/* 发送等待超时（ms）
 * 发送是非阻塞的：上一帧没发完（TXSUC=0）就跳过本周期。
 * 但为了避免“总线一次没 ACK → 之后永远不发”，超过这个时间强制重新认为空闲。
 * 250k 下 8 字节约 0.3ms，100ms 已经非常宽松。 */
#define CAN_TX_TIMEOUT_MS           (100)

/* 每 1ms 最多处理几帧接收，防止突发帧把 1ms 任务撑爆 */
#define CAN_RX_PER_TICK             (2)

/* 指令超时（ms）：0 = 关闭（台架调试推荐）
 * >0 = 超过该时间没收到有效指令，就自动停机（is_started=0） */
#define CAN_CMD_TIMEOUT_MS          (5000)

/* ==========================================================================
 *  六、指令目标值标定
 * ========================================================================== */
/* Customer.h 里的 MOTOR_SPEED_MIN_RPM / MOTOR_SPEED_MAX_RPM 是 S_Value() 的 Q15 值
 * （0~32767 对应 0~MOTOR_SPEED_BASE），而 g_run_state.ref 是 rpm，不能直接比较。
 * 这里做一次反换算，CAN 的上下限就完全跟随 Customer.h，不用再写死 1800/4096。 */
#define CAN_RPM_FROM_Q15(q)     ((float)(q) * MOTOR_SPEED_BASE / 32767.0f)

/* 100% 指令对应的转速 = Customer.h 的 MOTOR_SPEED_MAX_RPM */
#define CAN_PCT100_RPM          CAN_RPM_FROM_Q15(MOTOR_SPEED_MAX_RPM)

/* 非零指令的最小转速 = Customer.h 的 MOTOR_SPEED_MIN_RPM
 * 低于它电机稳不住（速度环输出被 SOUTMIN=1.5A 顶住，实际会跑到 150rpm 左右）。 */
#define CAN_MIN_RPM             CAN_RPM_FROM_Q15(MOTOR_SPEED_MIN_RPM)

#endif /* __CAN_CONFIG_H__ */
