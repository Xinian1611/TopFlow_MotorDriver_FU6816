/**
 * @file      CanProtocol.h
 * @brief     最小 CAN 工程的协议定义（从 20261030 的 CanProtocol.h 裁剪而来）
 *
 * ============================================================================
 *  重要说明
 * ----------------------------------------------------------------------------
 *  1. 本文件只保留“指令报文 + 状态报文”需要的定义，宏名、位域、字节序与
 *     D:\Code\FU6816\Software_36v_20261030\User\include\CanProtocol.h 完全一致，
 *     方便你两边对照阅读、也方便以后把 20261030 的协议栈搬过来。
 *  2. 报文统一为 29 位扩展帧、8 字节载荷（DLC=8）。
 *  3. 多字节字段一律“大端”（高字节在前）。
 *  4. 带符号量的零点为 32767：raw = 物理值 + 32767。
 *       例：转速 410rpm  -> raw = 33177 = 0x8199
 *           电流 5.00A   -> raw = 32767 + 500 = 33267 = 0x81F3
 *  5. 本文件不依赖 RingBuffer / CanSlave / DevConfig / EEPROM，
 *     可以在最小工程里单独使用。
 * ============================================================================
 */

#ifndef CAN_PROTOCOL_H
#define CAN_PROTOCOL_H

/* ==========================================================================
 *  一、帧长度
 * ========================================================================== */
/* 一帧 CAN 的标准数据长度（本工程的指令/状态报文都是 8 字节） */
#define CAN_FRAME_DATA_LEN          8

/* ==========================================================================
 *  二、29 位扩展 ID 的位域划分
 * --------------------------------------------------------------------------
 *  bit[28:25] 优先级      (4 位, 0~15)
 *  bit[24:23] 系统编号    (2 位, 0~3)
 *  bit[22:18] 设备主类别  (5 位, 0~31)
 *  bit[17:12] 设备副类别  (6 位, 0~63)
 *  bit[11: 7] 设备编号    (5 位, 0~31, 0 表示广播)
 *  bit[ 6: 0] 报文编号    (7 位, 单帧固定为 1)
 * ========================================================================== */
#define EXTID_PRIORITY_POS          25
#define EXTID_PRIORITY_MASK         (0x0FUL << EXTID_PRIORITY_POS)
#define EXTID_SYSNUM_POS            23
#define EXTID_SYSNUM_MASK           (0x03UL << EXTID_SYSNUM_POS)
#define EXTID_MAINCAT_POS           18
#define EXTID_MAINCAT_MASK          (0x1FUL << EXTID_MAINCAT_POS)
#define EXTID_SUBCAT_POS            12
#define EXTID_SUBCAT_MASK           (0x3FUL << EXTID_SUBCAT_POS)
#define EXTID_DEVNUM_POS            7
#define EXTID_DEVNUM_MASK           (0x1FUL << EXTID_DEVNUM_POS)
#define EXTID_MSGNUM_POS            0
#define EXTID_MSGNUM_MASK           (0x7FUL << EXTID_MSGNUM_POS)

/**
 * @brief  拼装 29 位扩展帧 ID
 * @param  pri  优先级
 * @param  sys  系统编号
 * @param  main 设备主类别
 * @param  sub  设备副类别
 * @param  dev  设备编号
 * @param  msg  报文编号
 * @retval 29 位扩展 ID
 * @note   例：EXTID_BUILD(2, 0, 11, 15, 1, 1) = 0x042CF081
 */
#define EXTID_BUILD(pri, sys, main, sub, dev, msg) \
    (((uint32)(pri)  << EXTID_PRIORITY_POS) | \
     ((uint32)(sys)  << EXTID_SYSNUM_POS)   | \
     ((uint32)(main) << EXTID_MAINCAT_POS)  | \
     ((uint32)(sub)  << EXTID_SUBCAT_POS)   | \
     ((uint32)(dev)  << EXTID_DEVNUM_POS)   | \
     ((uint32)(msg)  << EXTID_MSGNUM_POS))

/* 从 29 位 ID 中拆出各字段（调试用） */
#define EXTID_GET_PRIORITY(id)      (((id) & EXTID_PRIORITY_MASK) >> EXTID_PRIORITY_POS)
#define EXTID_GET_SYSNUM(id)        (((id) & EXTID_SYSNUM_MASK)   >> EXTID_SYSNUM_POS)
#define EXTID_GET_MAINCAT(id)       (((id) & EXTID_MAINCAT_MASK)  >> EXTID_MAINCAT_POS)
#define EXTID_GET_SUBCAT(id)        (((id) & EXTID_SUBCAT_MASK)   >> EXTID_SUBCAT_POS)
#define EXTID_GET_DEVNUM(id)        (((id) & EXTID_DEVNUM_MASK)   >> EXTID_DEVNUM_POS)
#define EXTID_GET_MSGNUM(id)        (((id) & EXTID_MSGNUM_MASK)   >> EXTID_MSGNUM_POS)

/* ==========================================================================
 *  三、优先级（与 20261030 的 ExtPriority_e 对应）
 * ========================================================================== */
#define EXT_PRI_CONFIG              1    /* 配置报文（本最小版不用） */
#define EXT_PRI_CMD                 2    /* 指令报文 */
#define EXT_PRI_STATUS              3    /* 状态报文 */
#define EXT_PRI_HEARTBEAT           4    /* 心跳报文（本最小版不用） */
#define EXT_PRI_LOG                 5    /* 日志报文（本最小版不用） */

/* ==========================================================================
 *  四、指令报文 ctrl 字节（Byte0）位域
 * --------------------------------------------------------------------------
 *  bit[2:0] 工作模式
 *  bit[4:3] 启停
 *  bit[6:5] 急停
 *  bit[7]   保留（0）
 * ========================================================================== */
#define EXT_CMD_WORK_MODE_MASK      0x07
#define EXT_CMD_WORK_MODE_POS       0
#define EXT_CMD_STARTSTOP_MASK      0x18
#define EXT_CMD_STARTSTOP_POS       3
#define EXT_CMD_ESTOP_MASK          0x60
#define EXT_CMD_ESTOP_POS           5

/* 工作模式取值 */
#define EXT_WORK_MODE_NONE          0    /* 无操作（保持原模式） */
#define EXT_WORK_MODE_CURRENT       1    /* 电流控制模式（本最小版不支持，见 CanMin_SetMode） */
#define EXT_WORK_MODE_SPEED         2    /* 转速控制模式 */
#define EXT_WORK_MODE_INVALID       7    /* 无效值 */

/* 启停取值 */
#define EXT_STARTSTOP_NONE          0    /* 无操作（保持原状态） */
#define EXT_STARTSTOP_START         1    /* 起机 */
#define EXT_STARTSTOP_STOP          2    /* 停机 */
#define EXT_STARTSTOP_INVALID       3    /* 无效值 */

/* 急停取值 */
#define EXT_ESTOP_NONE              0    /* 无操作（保持原状态） */
#define EXT_ESTOP_ACTIVE            1    /* 急停 */
#define EXT_ESTOP_RELEASE           2    /* 非急停（解除急停） */
#define EXT_ESTOP_INVALID           3    /* 无效值 */

/* ctrl 位域读取：从 ctrl 字节里取出三个字段 */
#define EXT_GET_WORKMODE(ctrl)      ((ctrl) & EXT_CMD_WORK_MODE_MASK)
#define EXT_GET_STARTSTOP(ctrl)     (((ctrl) & EXT_CMD_STARTSTOP_MASK) >> EXT_CMD_STARTSTOP_POS)
#define EXT_GET_ESTOP(ctrl)         (((ctrl) & EXT_CMD_ESTOP_MASK) >> EXT_CMD_ESTOP_POS)

/* ctrl 位域写入：状态帧里回显当前模式/启停/急停时用 */
#define EXT_SET_WORKMODE(ctrl, mode)  ((ctrl) = ((ctrl) & ~EXT_CMD_WORK_MODE_MASK) | ((mode) & EXT_CMD_WORK_MODE_MASK))
#define EXT_SET_STARTSTOP(ctrl, val)  ((ctrl) = ((ctrl) & ~EXT_CMD_STARTSTOP_MASK) | (((val) << EXT_CMD_STARTSTOP_POS) & EXT_CMD_STARTSTOP_MASK))
#define EXT_SET_ESTOP(ctrl, val)      ((ctrl) = ((ctrl) & ~EXT_CMD_ESTOP_MASK) | (((val) << EXT_CMD_ESTOP_POS) & EXT_CMD_ESTOP_MASK))

/* ==========================================================================
 *  五、故障码（本最小版只做“有/无故障”的粗映射）
 * --------------------------------------------------------------------------
 *  bit[7:6] 故障等级：0=一级(报警) 1=二级(报警) 2=三级(停机) 3=四级
 *  bit[5:0] 故障编码：1浸水 2过压 3过流 4驱动温 5电机温 6超速 7通信异常
 *  0xFF = 无故障
 * ========================================================================== */
#define FAULT_NONE                  0xFF
#define FAULT_LEVEL_MASK            0xC0
#define FAULT_LEVEL_POS             6
#define FAULT_CODE_MASK             0x3F
#define FAULT_LEVEL_3               2    /* 三级故障：停机 */
#define FAULT3_COMM_STOP            7    /* 三级故障编码：通信异常停机 */

#define FAULT_BUILD(level, cod)     (((uint8)(level) << FAULT_LEVEL_POS) | ((uint8)(cod) & FAULT_CODE_MASK))
#define FAULT_GET_LEVEL(cod)        (((cod) & FAULT_LEVEL_MASK) >> FAULT_LEVEL_POS)
#define FAULT_GET_CODE(cod)         ((cod) & FAULT_CODE_MASK)

/* ==========================================================================
 *  六、16 位字段的大端读写（高字节在前）
 * ========================================================================== */
/* 读：hi 是高字节，lo 是低字节 */
#define EXT_GET_U16(hi, lo)         ((uint16)(((uint16)(hi) << 8) | (uint16)(lo)))
#define EXT_GET_I16(hi, lo)         ((int16)(((uint16)(hi) << 8) | (uint16)(lo)))

/* 写：把 val 拆成高/低字节 */
#define EXT_SET_U16(hi, lo, val) \
    do { (hi) = (uint8)((val) >> 8); (lo) = (uint8)((val) & 0xFF); } while (0)

#define EXT_SET_I16(hi, lo, val) \
    do { (hi) = (uint8)((val) >> 8); (lo) = (uint8)((val) & 0xFF); } while (0)

/* ==========================================================================
 *  七、指令报文载荷（8 字节，单帧）
 * --------------------------------------------------------------------------
 *  Byte0     ctrl          控制字节（位域见第四节）
 *  Byte1-2   target_speed  目标转速，0.01%，32767 = 0%，±10000 = ±100%（大端）
 *  Byte3-4   target_current目标电流，0.01A，32767 = 0A（大端）
 *  Byte5-7   reserved      保留，协议规定填 0xFF
 * ========================================================================== */
typedef struct
{
    union
    {
        uint8 value;                 /* 整个 ctrl 字节 */
        struct
        {
            uint8 work_mode  : 3;    /* bit[2:0] 工作模式 */
            uint8 startstop  : 2;    /* bit[4:3] 启停 */
            uint8 estop      : 2;    /* bit[6:5] 急停 */
            uint8 reserved   : 1;    /* bit[7]   保留 */
        } reg;
    } ctrl;
    uint8 target_speed_hi;           /* Byte1: 目标转速高字节 */
    uint8 target_speed_lo;           /* Byte2: 目标转速低字节 */
    uint8 target_current_hi;         /* Byte3: 目标电流高字节 */
    uint8 target_current_lo;         /* Byte4: 目标电流低字节 */
    uint8 reserved[3];               /* Byte5-7: 保留 */
} ExtCmdMsg_t;

/* ==========================================================================
 *  八、状态报文载荷（8 字节，单帧）
 * --------------------------------------------------------------------------
 *  Byte0     ctrl        回显：工作模式 + 启停状态 + 急停状态
 *  Byte1-2   speed       实际转速，1rpm，32767 = 0，负值 = 反转（大端）
 *  Byte3-4   current     相电流，0.01A，32767 = 0A（大端）
 *  Byte5-6   power       功率，1W，32767 = 0W（大端）
 *  Byte7     fault_code  故障码，0xFF = 无故障
 * ========================================================================== */
typedef struct
{
    union
    {
        uint8 value;
        struct
        {
            uint8 work_mode  : 3;
            uint8 startstop  : 2;
            uint8 estop      : 2;
            uint8 reserved   : 1;
        } reg;
    } ctrl;
    uint8 speed_hi;                  /* Byte1: 转速高字节 */
    uint8 speed_lo;                  /* Byte2: 转速低字节 */
    uint8 current_hi;                /* Byte3: 电流高字节 */
    uint8 current_lo;                /* Byte4: 电流低字节 */
    uint8 power_hi;                  /* Byte5: 功率高字节 */
    uint8 power_lo;                  /* Byte6: 功率低字节 */
    uint8 fault_code;                /* Byte7: 故障码 */
} ExtStatusMsg_t;

#endif /* CAN_PROTOCOL_H */
