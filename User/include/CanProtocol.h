#ifndef CAN_PROTOCOL_H
#define CAN_PROTOCOL_H

#include <stdbool.h>
#include <string.h>
#include "RingBuffer.h"

/************************ 协议基础常量 ************************/

// 数据长度
#define CAN_FRAME_DATA_LEN      8       // 标准CAN帧数据长度

/************************ 扩展帧仲裁段定义 ************************/
// 29位扩展ID位域布局: [28:25]优先级 [24:23]系统编号 [22:18]设备主类别 [17:12]设备副类别 [11:7]设备编号 [6:0]报文编号
#define EXTID_PRIORITY_POS     25
#define EXTID_PRIORITY_MASK    (0x0FUL << EXTID_PRIORITY_POS)
#define EXTID_SYSNUM_POS       23
#define EXTID_SYSNUM_MASK      (0x03UL << EXTID_SYSNUM_POS)
#define EXTID_MAINCAT_POS      18
#define EXTID_MAINCAT_MASK     (0x1FUL << EXTID_MAINCAT_POS)
#define EXTID_SUBCAT_POS       12
#define EXTID_SUBCAT_MASK      (0x3FUL << EXTID_SUBCAT_POS)
#define EXTID_DEVNUM_POS       7
#define EXTID_DEVNUM_MASK      (0x1FUL << EXTID_DEVNUM_POS)
#define EXTID_MSGNUM_POS       0
#define EXTID_MSGNUM_MASK      (0x7FUL << EXTID_MSGNUM_POS)

// 扩展ID构建
#define EXTID_BUILD(pri, sys, main, sub, dev, msg) \
    (((uint32_t)(pri)  << EXTID_PRIORITY_POS) | \
     ((uint32_t)(sys)  << EXTID_SYSNUM_POS)   | \
     ((uint32_t)(main) << EXTID_MAINCAT_POS)  | \
     ((uint32_t)(sub)  << EXTID_SUBCAT_POS)   | \
     ((uint32_t)(dev)  << EXTID_DEVNUM_POS)   | \
     ((uint32_t)(msg)  << EXTID_MSGNUM_POS))

// 扩展ID字段提取
#define EXTID_GET_PRIORITY(id)   (((id) & EXTID_PRIORITY_MASK) >> EXTID_PRIORITY_POS)
#define EXTID_GET_SYSNUM(id)     (((id) & EXTID_SYSNUM_MASK)   >> EXTID_SYSNUM_POS)
#define EXTID_GET_MAINCAT(id)    (((id) & EXTID_MAINCAT_MASK)  >> EXTID_MAINCAT_POS)
#define EXTID_GET_SUBCAT(id)     (((id) & EXTID_SUBCAT_MASK)   >> EXTID_SUBCAT_POS)
#define EXTID_GET_DEVNUM(id)     (((id) & EXTID_DEVNUM_MASK)   >> EXTID_DEVNUM_POS)
#define EXTID_GET_MSGNUM(id)     (((id) & EXTID_MSGNUM_MASK)   >> EXTID_MSGNUM_POS)

/************************ 优先级定义 ************************/
typedef enum {
    EXT_PRI_RESERVED0 = 0,
    EXT_PRI_CONFIG    = 1,    // 配置报文
    EXT_PRI_CMD       = 2,    // 指令报文
    EXT_PRI_STATUS    = 3,    // 状态报文
    EXT_PRI_HEARTBEAT = 4,    // 心跳报文
    EXT_PRI_LOG       = 5,    // 日志报文
    EXT_PRI_TEST      = 6,    // 测试报文
    EXT_PRI_TASK      = 7,    // 任务报文
    EXT_PRI_PERCEP    = 8,    // 感知报文
} ExtPriority_e;

/************************ 系统编号定义 ************************/
typedef enum {
    EXT_SYS_MAIN  = 0,    // 主系统
    EXT_SYS_RES1  = 1,    // 备用1
    EXT_SYS_RES2  = 2,    // 备用2
    EXT_SYS_RES3  = 3,    // 备用3
} ExtSysNum_e;

/************************ 设备主类别定义 ************************/
typedef enum {
    EXT_CAT_BASIC_CTRL   = 1,    // 基本控制器
    EXT_CAT_TASK_ENGINE  = 6,    // 任务管理引擎
    EXT_CAT_COMM_SYS     = 7,    // 通信管理系统
    EXT_CAT_NAV_SYS      = 8,    // 导航测量系统
    EXT_CAT_PERCEP_ENGINE= 9,    // 感知引擎
    EXT_CAT_BEHAV_ENGINE = 10,   // 行为规划引擎
    EXT_CAT_POWER_ENGINE = 11,   // 动力驱动引擎
    EXT_CAT_CTRL_SYS     = 12,   // 操控系统
    EXT_CAT_AUX_SYS      = 13,   // 辅助管理系统
    EXT_CAT_DISPLAY_SYS  = 14,   // 显示系统
    EXT_CAT_PAYLOAD_SYS  = 15,   // 载荷接入系统
    EXT_CAT_POWER_SUPPLY = 16,   // 供配电系统
} ExtMainCategory_e;

/************************ 基本控制器副类别(主编号1) ************************/
typedef enum {
    EXT_SUB_BEHAV_CTRL   = 1,    // 行为控制器
    EXT_SUB_POWER_CTRL   = 2,    // 动力控制器
    EXT_SUB_PORTABLE_BS  = 3,    // 便携基站
    EXT_SUB_HANDHELD_BS  = 4,    // 手持基站
    EXT_SUB_COMM_MGR     = 5,    // 通信管理机
    EXT_SUB_PERCEP_CTRL  = 6,    // 感知控制器
    EXT_SUB_IMU_GATEWAY  = 7,    // 惯导网关
    EXT_SUB_PERCEP_GW    = 8,    // 感知网关
    EXT_SUB_AUX_GW       = 9,    // 辅助网关
    EXT_SUB_DISPLAY_GW   = 10,   // 显示网关
    EXT_SUB_PAYLOAD_GW   = 11,   // 载荷网关
    EXT_SUB_CTRL_GW      = 12,   // 操控网关
    EXT_SUB_DIN          = 13,   // 综合DIN
    EXT_SUB_DAC          = 14,   // 综合DAC
    EXT_SUB_ADC          = 15,   // 综合ADC
    EXT_SUB_MKO          = 16,   // 综合MKO
    EXT_SUB_IPM          = 17,   // 综合IPM
    EXT_SUB_PWM          = 18,   // 综合PWM
    EXT_SUB_COM          = 19,   // 综合COM
    EXT_SUB_E_STEERING   = 20,   // 电子方向盘
    EXT_SUB_THROTTLE     = 21,   // 油门推杆
    EXT_SUB_HULL_PANEL   = 22,   // 船体控制面板
    EXT_SUB_OPTO_PANEL   = 23,   // 光电控制面板
    EXT_SUB_PAYLOAD_PANEL= 24,   // 载荷控制面板
    EXT_SUB_START_PANEL  = 25,   // 启停控制面板
    EXT_SUB_BT_RECEIVER  = 26,   // 蓝牙接收器
    EXT_SUB_OPTO_CTRL    = 27,   // 光电控制器
    EXT_SUB_LCD_12       = 28,   // 12寸触摸屏
    EXT_SUB_LCD_21       = 29,   // 21.5寸触摸屏
    EXT_SUB_PLAT_SERVER  = 30,   // 平台服务器
    EXT_SUB_MECH_ACTUATOR= 31,   // 机械执行器
    EXT_SUB_HF_RADIO_GW  = 32,   // 短波电台网关
    EXT_SUB_PAYLOAD_SVR  = 33,   // 载荷服务器
    EXT_SUB_AUTH_MGR     = 34,   // 权限管理器
    EXT_SUB_AUTH_CTRL    = 35,   // 权限操控端
    EXT_SUB_E_DRIVER     = 36,   // 电子驱动器
    EXT_SUB_STEER_DRIVER = 37,   // 转向驱动器
} ExtBasicCtrlSub_e;

/************************ 动力驱动引擎副类别(主编号11) ************************/
typedef enum {
    EXT_POWER_ICE          = 1,    // 内燃主机
    EXT_POWER_ELECTRIC     = 2,    // 电动主机
    EXT_POWER_THRUSTER     = 4,    // 推进器
    EXT_POWER_WAVE_PLATE   = 8,    // 压浪板
    EXT_POWER_THROTTLE     = 10,   // 节流板
    EXT_POWER_BOW_THRUST   = 12,   // 船艏侧推
    EXT_POWER_GEARBOX      = 14,   // 齿轮箱
    EXT_POWER_TROLL_MOTOR  = 15,   // 拖钓电机
} ExtPowerSub_e;

/************************ 供配电系统副类别(主编号16) ************************/
typedef enum {
    EXT_SUPPLY_GENERATOR = 1,    // 发电机
    EXT_SUPPLY_BATTERY   = 2,    // 电池组
} ExtSupplySub_e;

/************************ 拖钓电机消息载荷长度定义(不含公共头部) ************************/
#define EXT_MSG_LEN_CMD        8      // 指令报文载荷长度
#define EXT_MSG_LEN_STATUS     8      // 状态报文载荷长度
#define EXT_MSG_LEN_HEARTBEAT  0      // 心跳报文载荷长度(协议仅公共头部,无额外字段)
#define EXT_MSG_LEN_LOG        32     // 日志报文载荷长度
#define EXT_MSG_LEN_CONFIG     88     // 配置报文载荷长度
#define EXT_MSG_LEN_CFG_RSP    8      // 配置应答报文载荷长度

// 计算多帧帧数
#define EXT_FRAME_COUNT(msg_len)  (((msg_len) + CAN_FRAME_DATA_LEN - 1) / CAN_FRAME_DATA_LEN)

// 根据优先级获取消息载荷长度
#define EXT_MSG_LEN_BY_PRI(pri) \
    ((pri) == EXT_PRI_CMD       ? EXT_MSG_LEN_CMD : \
     (pri) == EXT_PRI_STATUS    ? EXT_MSG_LEN_STATUS : \
     (pri) == EXT_PRI_HEARTBEAT ? EXT_MSG_LEN_HEARTBEAT : \
     (pri) == EXT_PRI_LOG       ? EXT_MSG_LEN_LOG : \
     (pri) == EXT_PRI_CONFIG    ? EXT_MSG_LEN_CONFIG : 0)

/************************ 拖钓电机指令报文 ctrl 位域定义 ************************/
// ctrl: [2:0]工作模式 [4:3]启停 [6:5]急停
#define EXT_CMD_WORK_MODE_MASK   0x07
#define EXT_CMD_WORK_MODE_POS    0
#define EXT_CMD_STARTSTOP_MASK   0x18
#define EXT_CMD_STARTSTOP_POS    3
#define EXT_CMD_ESTOP_MASK       0x60
#define EXT_CMD_ESTOP_POS        5

// 工作模式值
#define EXT_WORK_MODE_NONE       0    // 无操作
#define EXT_WORK_MODE_CURRENT    1    // 电流控制模式
#define EXT_WORK_MODE_SPEED      2    // 转速控制模式
#define EXT_WORK_MODE_INVALID    7    // 无效值

// 启停值
#define EXT_STARTSTOP_NONE       0    // 无操作
#define EXT_STARTSTOP_START      1    // 起机
#define EXT_STARTSTOP_STOP       2    // 停机
#define EXT_STARTSTOP_INVALID    3    // 无效值

// 急停值
#define EXT_ESTOP_NONE           0    // 无操作
#define EXT_ESTOP_ACTIVE         1    // 急停
#define EXT_ESTOP_RELEASE        2    // 非急停
#define EXT_ESTOP_INVALID        3    // 无效值

// ctrl位域存取宏
#define EXT_GET_WORKMODE(ctrl)   ((ctrl) & EXT_CMD_WORK_MODE_MASK)
#define EXT_GET_STARTSTOP(ctrl)  (((ctrl) & EXT_CMD_STARTSTOP_MASK) >> EXT_CMD_STARTSTOP_POS)
#define EXT_GET_ESTOP(ctrl)      (((ctrl) & EXT_CMD_ESTOP_MASK) >> EXT_CMD_ESTOP_POS)

#define EXT_SET_WORKMODE(ctrl, mode)   ((ctrl) = ((ctrl) & ~EXT_CMD_WORK_MODE_MASK) | ((mode) & EXT_CMD_WORK_MODE_MASK))
#define EXT_SET_STARTSTOP(ctrl, val)   ((ctrl) = ((ctrl) & ~EXT_CMD_STARTSTOP_MASK) | (((val) << EXT_CMD_STARTSTOP_POS) & EXT_CMD_STARTSTOP_MASK))
#define EXT_SET_ESTOP(ctrl, val)       ((ctrl) = ((ctrl) & ~EXT_CMD_ESTOP_MASK) | (((val) << EXT_CMD_ESTOP_POS) & EXT_CMD_ESTOP_MASK))

/************************ 拖钓电机调试指令定义 ************************/
// 调试指令子命令(Byte5: reserved[0])
#define EXT_DBG_CMD_NONE        0    // 无操作
#define EXT_DBG_CMD_SET_KP      1    // 设置速度环KP(参数: Byte6-7, int16_t, Q12格式)
#define EXT_DBG_CMD_SET_KI      2    // 设置速度环KI(参数: Byte6-7, int16_t, Q15格式)
#define EXT_DBG_CMD_SET_KD      3    // 设置速度环KD(参数: Byte6-7, int16_t, Q12格式)
#define EXT_DBG_CMD_SET_ALL     4    // 同时设置速度环KP+KI+KD(参数: Byte6=KP, Byte7=KI, 分别作为Q8格式)
#define EXT_DBG_CMD_SAVE        5    // 保存当前PI参数到EEPROM(参数: 无)

/************************ 故障码定义 ************************/
// 故障码格式: Bit[7:6]=故障等级, Bit[5:0]=故障编码
#define FAULT_LEVEL_MASK         0xC0
#define FAULT_LEVEL_POS          6
#define FAULT_CODE_MASK          0x3F

#define FAULT_BUILD(level, cod) (((uint8_t)(level) << FAULT_LEVEL_POS) | ((uint8_t)(cod) & FAULT_CODE_MASK))
#define FAULT_GET_LEVEL(cod)    (((cod) & FAULT_LEVEL_MASK) >> FAULT_LEVEL_POS)
#define FAULT_GET_CODE(cod)     ((cod) & FAULT_CODE_MASK)

// 故障等级
typedef enum {
    FAULT_LEVEL_1 = 0,    // 一级故障(报警)
    FAULT_LEVEL_2 = 1,    // 二级故障(报警)
    FAULT_LEVEL_3 = 2,    // 三级故障(停机)
    FAULT_LEVEL_4 = 3,    // 四级故障
} FaultLevel_e;

// 三级故障编码(停机)
typedef enum {
    FAULT3_WATER_STOP       = 1,    // 浸水停机
    FAULT3_OVERVOLT_STOP    = 2,    // 电压高停机
    FAULT3_OVERCURR_STOP    = 3,    // 电流高停机
    FAULT3_DRVTEMP_STOP     = 4,    // 驱动温度高停机
    FAULT3_MOTTEMP_STOP     = 5,    // 电机温度高停机
    FAULT3_OVERSPEED_STOP   = 6,    // 超速停机
    FAULT3_COMM_STOP        = 7,    // 通信异常停机
} Fault3Code_e;

// 二级故障编码(报警)
typedef enum {
    FAULT2_WATER_ALARM      = 1,    // 浸水报警
    FAULT2_OVERVOLT_ALARM   = 2,    // 电压高报警
    FAULT2_OVERCURR_ALARM   = 3,    // 电流高报警
    FAULT2_DRVTEMP_ALARM    = 4,    // 驱动温度高报警
    FAULT2_MOTTEMP_ALARM    = 5,    // 电机温度高报警
    FAULT2_OVERSPEED_ALARM  = 6,    // 超速报警
} Fault2Code_e;

// 一级故障编码(报警)
typedef enum {
    FAULT1_UNDERVOLT_ALARM  = 1,    // 电压低报警
    FAULT1_STALL_ALARM      = 1,    // 失速报警
} Fault1Code_e;

// 无故障
#define FAULT_NONE  0xFF

/************************ 报文载荷结构体定义(不含公共头部) ************************/

// 指令报文载荷(8字节, 单帧)
typedef struct {
    union
    {
        uint8_t value;
        struct
        {
            uint8_t work_mode : 3;          ///< 工作模式
            uint8_t startstop : 2;          ///< 启停
            uint8_t estop : 2;              ///< 急停
        } reg;
    }ctrl;                     // Byte0: 控制字节(工作模式+启停+急停)
    uint8_t target_speed_hi;   // Byte1: 目标转速高字节(0.01%, 32767=0%)
    uint8_t target_speed_lo;   // Byte2: 目标转速低字节
    uint8_t target_current_hi; // Byte3: 目标电流高字节(0.01A, 32767=0A)
    uint8_t target_current_lo; // Byte4: 目标电流低字节
    uint8_t reserved[3];       // Byte5-7: 备用
} ExtCmdMsg_t;

// 状态报文载荷(8字节, 单帧)
typedef struct {
    union
    {
        uint8_t value;
        struct
        {
            uint8_t work_mode : 3;          ///< 工作模式
            uint8_t startstop : 2;          ///< 启停
            uint8_t estop : 2;              ///< 急停
        } reg;
    }ctrl;                     // Byte0: 控制字节(工作模式+启停状态+急停状态)
    uint8_t speed_hi;          // Byte1: 转速高字节(1rpm, 32767=0rpm)
    uint8_t speed_lo;          // Byte2: 转速低字节
    uint8_t current_hi;        // Byte3: 电流高字节(0.01A, 32767=0A)
    uint8_t current_lo;        // Byte4: 电流低字节
    uint8_t power_hi;          // Byte5: 功率高字节(1W, 32767=0W)
    uint8_t power_lo;          // Byte6: 功率低字节
    uint8_t fault_code;        // Byte7: 故障代码
} ExtStatusMsg_t;

// 心跳报文载荷
typedef struct {
    uint8_t reserved[8];       // Byte0-7: 备用
} ExtHeartbeatMsg_t;

// 日志报文载荷(32字节, 4帧)
typedef struct {
    uint8_t speed_hi;          // Byte0-1: 转速(1rpm, 32767=0rpm)
    uint8_t speed_lo;
    uint8_t voltage_hi;        // Byte2-3: 电机电压(0.01V, 32767=0V)
    uint8_t voltage_lo;
    uint8_t current_hi;        // Byte4-5: 电机电流(0.01A, 32767=0A)
    uint8_t current_lo;
    uint8_t power_hi;          // Byte6-7: 电机功率(1W, 32767=0W)
    uint8_t power_lo;
    uint8_t mot_temp_hi;       // Byte8-9: 电机温度(0.1℃, 32767=0℃)
    uint8_t mot_temp_lo;
    uint8_t drv_temp_hi;       // Byte10-11: 驱动温度(0.1℃, 32767=0℃)
    uint8_t drv_temp_lo;
    uint8_t torque_hi;         // Byte12-13: 电机转矩(0.1NM, 32767=0NM)
    uint8_t torque_lo;
    uint8_t reserved1[2];      // Byte14-15: 备用
    uint8_t ia_hi;             // Byte16-17: U相电流(0.01A, 32767=0A)
    uint8_t ia_lo;
    uint8_t ib_hi;             // Byte18-19: V相电流(0.01A, 32767=0A)
    uint8_t ib_lo;
    uint8_t ic_hi;             // Byte20-21: W相电流(0.01A, 32767=0A)
    uint8_t ic_lo;
    uint8_t reserved2[2];      // Byte22-23: 备用
    uint8_t run_time[4];       // Byte24-27: 单次运行时间(1s)
    uint8_t total_time[4];     // Byte28-31: 总运行时间(1s)
} ExtLogMsg_t;

// 电机测试报文载荷
typedef struct {
    uint16_t speedKp;          // Byte0-1: 速度环KP系数(Q12格式)
    uint16_t speedKi;          // Byte2-3: 速度环KI系数(Q15格式)
    uint16_t targetSpeed;      // Byte4-5: 目标转速(1rpm, 32767=0rpm)
    uint16_t realSpeed;        // Byte6-7: 实际转速(1rpm, 32767=0rpm)
    uint16_t currentA;         // Byte8-9: A 相电流(0.01A, 32767=0A)
    uint16_t currentB;         // Byte10-11: B 相电流(0.01A, 32767=0A)
    uint16_t currentC;         // Byte12-13: C 相电流(0.01A, 32767=0A)
    uint16_t currentBus;       // Byte14-15: 母线电流(0.01A, 32767=0A)
    uint16_t speedPiOutput;    // Byte16-17: 速度环输出(0.01A, 32767=0A)
    uint16_t currentPiOutput;  // Byte18-19: 电流环输出(0.1V, 32767=0V)
    uint16_t focQFlag;         // Byte20-21: FOC Q轴方向标志位(0:正, 1:负)
    uint16_t focQSpeed;        // Byte22-23: FOC Q轴转速(1rpm, 32767=0rpm)
    uint16_t focQCurrent;      // Byte24-25: FOC Q轴电流(0.01A, 32767=0A)
    uint8_t reserved[6];        // Byte26-31: 备用
} ExtPecerpMsg_t;

// 配置读取请求\配置应答报文载荷(8字节, 单帧)
typedef struct {
    uint8_t cfg_read;          // Byte0: 配置读操作(0:无操作, 1:配置读, 3:无效值)
    uint8_t cfg_resp;          // Byte1: 配置报文应答(0:无操作, 1:应答, 2:未应答, 3:无效值)
    uint8_t reserved0[6];      // Byte2-7: 保留
} ExtCfgMsg_t;

// 配置报文载荷(88字节, 11帧)
typedef struct {
    uint8_t hw_ver[4];         // Byte0-3: 硬件版本号(aaa.bbb.ccc.ddd)
    uint8_t sw_ver[4];         // Byte4-7: 软件版本号(aaa.bbb.ccc.ddd)
    uint8_t proto_ver[4];      // Byte8-11: 规约版本号(aaa.bbb.ccc.ddd)
    uint8_t ctrl_type;         // Byte12: 设备所属操控端类型(0:无操作, 1:船端, 2:集控端, 3:移动端)
    uint8_t speed_kp_hi;       // Byte13-14: 速度环KP系数(Q12格式)
    uint8_t speed_kp_lo;
    uint8_t speed_ki_hi;       // Byte15-16: 速度环KI系数(Q15格式)
    uint8_t speed_ki_lo;
    uint8_t speed_kd_hi;       // Byte17-18: 速度环KD系数(Q12格式)
    uint8_t speed_kd_lo;
    uint8_t motor_type;        // Byte19: 电机类型(0:无操作, 1:12V-600W, 2:24V-1200W, 3:36V-2500W)
    uint8_t drv_water_alarm;   // Byte20: 驱动浸水报警(低6位:持续时间1-63s, 高2位:使能0-3)
    uint8_t drv_water_stop;    // Byte21: 驱动浸水停机(低6位:延迟时间1-63s, 高2位:使能0-3)
    uint8_t mot_water_alarm;   // Byte22: 电机浸水报警(低6位:持续时间1-63s, 高2位:使能0-3)
    uint8_t mot_water_stop;    // Byte23: 电机浸水停机(低6位:延迟时间1-63s, 高2位:使能0-3)
    uint8_t speed_precision;   // Byte24: 转速控制精度(0-255rpm)
    uint8_t max_speed_hi;      // Byte25-26: 最高转速(0-5000rpm)
    uint8_t max_speed_lo;
    uint8_t idle_speed_hi;     // Byte27-28: 怠速转速(0-5000rpm)
    uint8_t idle_speed_lo;
    uint8_t max_speed_inc;     // Byte29: 最大转速控制增量(0-2550rpm, 10rpm)
    uint8_t min_speed_inc;     // Byte30: 最小转速控制增量(0-2550rpm, 10rpm)
    uint8_t reserved3;         // Byte31: 保留
    uint8_t rated_voltage_hi;  // Byte32-33: 额定电压(0.01V)
    uint8_t rated_voltage_lo;
    uint8_t rated_current_hi;  // Byte34-35: 额定电流(0.01A)
    uint8_t rated_current_lo;
    uint8_t fwd_power_pct;     // Byte36: 正向允许输入功率百分比(0-100%)
    uint8_t rev_power_pct;     // Byte37: 反向允许输入功率百分比(0-100%)
    uint8_t reserved4[2];      // Byte38-39: 保留
    uint8_t max_voltage_hi;    // Byte40-41: 最大电压(0.01V)
    uint8_t max_voltage_lo;
    uint8_t min_voltage_hi;    // Byte42-43: 最小电压(0.01V)
    uint8_t min_voltage_lo;
    uint8_t max_current_hi;    // Byte44-45: 最大电流(0.01A)
    uint8_t max_current_lo;
    uint8_t reserved5[2];      // Byte46-47: 保留
    uint8_t volt_high_alarm_hi;  // Byte48-49: 电压高报警值(0.01V, 32767=0V)
    uint8_t volt_high_alarm_lo;
    uint8_t volt_low_alarm_hi;   // Byte50-51: 电压低报警值(0.01V, 32767=0V)
    uint8_t volt_low_alarm_lo;
    uint8_t volt_high_stop_hi;   // Byte52-53: 电压高停车值(0.01V, 32767=0V)
    uint8_t volt_high_stop_lo;
    uint8_t volt_alarm;           // Byte54: 电压报警(低6位:持续时间1-63s, 高2位:使能0-3)
    uint8_t volt_stop;            // Byte55: 电压停机(低6位:延迟时间1-63s, 高2位:使能0-3)
    uint8_t curr_high_alarm_hi;   // Byte56-57: 电流高报警值(0.01A, 32767=0A)
    uint8_t curr_high_alarm_lo;
    uint8_t curr_low_alarm_hi;    // Byte58-59: 电流低报警值(0.01A, 32767=0A)
    uint8_t curr_low_alarm_lo;
    uint8_t curr_high_stop_hi;    // Byte60-61: 电流高停车值(0.01A, 32767=0A)
    uint8_t curr_high_stop_lo;
    uint8_t curr_alarm;           // Byte62: 电流报警(低6位:持续时间1-63s, 高2位:使能0-3)
    uint8_t curr_stop;            // Byte63: 电流停机(低6位:延迟时间1-63s, 高2位:使能0-3)
    uint8_t drv_temp_high_alarm_hi; // Byte64-65: 驱动温度高报警值(0.1℃, 32767=0℃)
    uint8_t drv_temp_high_alarm_lo;
    uint8_t drv_temp_low_alarm_hi;  // Byte66-67: 驱动温度低报警值(0.1℃, 32767=0℃)
    uint8_t drv_temp_low_alarm_lo;
    uint8_t drv_temp_high_stop_hi;  // Byte68-69: 驱动温度高停车值(0.1℃, 32767=0℃)
    uint8_t drv_temp_high_stop_lo;
    uint8_t drv_temp_alarm;         // Byte70: 驱动温度报警(低6位:持续时间1-63s, 高2位:使能0-3)
    uint8_t drv_temp_stop;          // Byte71: 驱动温度停机(低6位:延迟时间1-63s, 高2位:使能0-3)
    uint8_t mot_temp_high_alarm_hi; // Byte72-73: 电机温度高报警值(0.1℃, 32767=0℃)
    uint8_t mot_temp_high_alarm_lo;
    uint8_t mot_temp_low_alarm_hi;  // Byte74-75: 电机温度低报警值(0.1℃, 32767=0℃)
    uint8_t mot_temp_low_alarm_lo;
    uint8_t mot_temp_high_stop_hi;  // Byte76-77: 电机温度高停车值(0.1℃, 32767=0℃)
    uint8_t mot_temp_high_stop_lo;
    uint8_t mot_temp_alarm;         // Byte78: 电机温度报警(低6位:持续时间1-63s, 高2位:使能0-3)
    uint8_t mot_temp_stop;          // Byte79: 电机温度停机(低6位:延迟时间1-63s, 高2位:使能0-3)
    uint8_t overspeed_alarm_hi;     // Byte80-81: 超速报警值(0-5000rpm)
    uint8_t overspeed_alarm_lo;
    uint8_t stall_alarm_hi;         // Byte82-83: 失速报警值(0-5000rpm)
    uint8_t stall_alarm_lo;
    uint8_t overspeed_stop_hi;      // Byte84-85: 超速停车值(0-5000rpm)
    uint8_t overspeed_stop_lo;
    uint8_t speed_alarm;            // Byte86: 转速报警(低6位:持续时间1-63s, 高2位:使能0-3)
    uint8_t speed_stop;             // Byte87: 转速停机(低6位:延迟时间1-63s, 高2位:使能0-3)
} ExtConfigMsg_t;

// 16位字段存取宏(大端序)
#define EXT_GET_U16(hi, lo)     ((uint16_t)((hi) << 8) | (lo))
#define EXT_GET_I16(hi, lo)     ((int16_t)((hi) << 8) | (lo))

#define EXT_SET_U16(hi, lo, val) \
    do { (hi) = (uint8_t)((val) >> 8); (lo) = (uint8_t)((val) & 0xFF); } while (0)

#define EXT_SET_I16(hi, lo, val) \
    do { (hi) = (uint8_t)((val) >> 8); (lo) = (uint8_t)((val) & 0xFF); } while (0)

// 32位字段存取宏(大端序)
#define EXT_GET_U32(b0, b1, b2, b3) \
    (((uint32_t)(b0) << 24) | ((uint32_t)(b1) << 16) | ((uint32_t)(b2) << 8) | (uint32_t)(b3))

#define EXT_SET_U32(b0, b1, b2, b3, val) \
    do { (b0) = (uint8_t)((val) >> 24); (b1) = (uint8_t)((val) >> 16); \
         (b2) = (uint8_t)((val) >> 8);  (b3) = (uint8_t)((val) & 0xFF); } while (0)


#endif // CAN_PROTOCOL_H