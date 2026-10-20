/************************************************************************************
 * @Author       : FengYu
 * @Date         : 2026-07-09
 * @Description  : 基于MotorDrvCfg_t的可配置告警监控模块
 * @Copyright (c) 2026 by FengYu, All Rights Reserved.
*************************************************************************************/

#ifndef ALARM_MONITOR_H
#define ALARM_MONITOR_H

#include "DevConfig.h"
#include "CanProtocol.h"

/************************ 电池电压/温度保护配置 ************************/

/* 电池电压/温度保护总开关：0 = 警报不生效；1 = 警报正常生效
 * 注意：只关本文件新增的 OVP 电压和 NTC 温度两路，
 *       不影响浸水/电流/转速等原有告警。 */
#define BAT_ALARM_EN                    (1)

/* 报警/停机确认时间(ms) */
#define BAT_ALARM_DELAY_MS              (3000)
#define BAT_STOP_DELAY_MS               (1000)

/* OVP 输入电源检测：P3.3 = AD6
 *   PWR+ = OVP × OVP_DIV_NUM / OVP_DIV_DEN
 *   当前按 1150/150 计算，VREF=4.5V 时满量程 = 34.50V
 *
 *    上电前必须先量一次 P3.3 分压：
 *     实际 24.0V 时 P3.3 = 3.60V → 1000/150 正确；
 *     实际 24.0V 时 P3.3 = 3.13V → 分压是 1000k+150k，应改成 1150/150。
 *   如果实际是 1150/150 却按 1000/150 算，24V 会显示成约 20.9V，
 *   铅酸 21V 低压报警会在 24V 正常供电时直接误报。 */
#define OVP_ADC_DR                     ADC6_DR
#define OVP_DIV_NUM                    (1150UL)
#define OVP_DIV_DEN                    (150UL)
#define OVP_FULL_SCALE_CV              ((uint16_t)(HW_ADC_REF * 100.0 * (float)OVP_DIV_NUM / (float)OVP_DIV_DEN))
#define OVP_LPF_K                      (50)

/* 电池电压阈值，单位 0.01V */
#define BAT_LEAD_ACID_LOW_ALARM_CV     (2100)   /* 21.00V */        //铅酸电池
#define BAT_LEAD_ACID_HIGH_ALARM_CV    (2650)   /* 26.50V */
#define BAT_LEAD_ACID_HIGH_STOP_CV     (2700)   /* 27.00V，停机默认不开 */
#define BAT_LIFEPO4_LOW_ALARM_CV       (2200)   /* 22.00V */        //磷酸铁锂电池
#define BAT_LIFEPO4_HIGH_ALARM_CV      (3000)   /* 30.00V */
#define BAT_LIFEPO4_HIGH_STOP_CV       (3000)   /* 30.00V，停机默认不开 */

/* NTC 驱动模块温度阈值，单位 0.1℃ */
#define BAT_NTC_HIGH_ALARM_01C         (800)    /* 80.0℃ */
#define BAT_NTC_HIGH_STOP_01C          (850)    /* 85.0℃，停机默认不开 */

/* 报警打印事件：1ms任务只置位，主循环负责 printf，避免阻塞1ms节拍 */
#define ALARM_EVT_NONE          0
#define ALARM_EVT_VOLT_HIGH     1
#define ALARM_EVT_VOLT_LOW      2
#define ALARM_EVT_TEMP_HIGH     3

extern volatile uint8_t  xdata g_alarm_evt;
extern volatile uint16_t xdata g_alarm_evt_value;
extern volatile uint16_t xdata g_alarm_evt_limit;

/************************ 告警项ID ************************/
typedef enum {
    ALARM_DRV_WATER,      // 驱动浸水
    ALARM_MOT_WATER,      // 电机浸水
    ALARM_VOLT_HIGH,      // 电压高
    ALARM_VOLT_LOW,       // 电压低
    ALARM_CURR_HIGH,      // 电流高
    ALARM_CURR_LOW,       // 电流低
    ALARM_DRV_TEMP_HIGH,  // 驱动温度高
    ALARM_DRV_TEMP_LOW,   // 驱动温度低
    ALARM_MOT_TEMP_HIGH,  // 电机温度高
    ALARM_MOT_TEMP_LOW,   // 电机温度低
    ALARM_OVERSPEED,      // 超速
    ALARM_STALL,          // 失速
    ALARM_COUNT
} AlarmId_e;

/************************ 告警项状态 ************************/
typedef struct {
    uint16_t alarm_cnt;     // 报警持续时间计数(ms)
    uint16_t stop_cnt;      // 停机延迟时间计数(ms)
    uint8_t  alarm_active;  // 报警激活标志
    uint8_t  stop_active;   // 停机激活标志
} AlarmItemState_t;

/************************ 告警队列配置 ************************/
#define ALARM_QUEUE_SIZE      16   // 最大同时记录的活跃故障数

/************************ 告警模块状态 ************************/
typedef struct {
    AlarmItemState_t items[ALARM_COUNT];
    uint8_t water_drv_state;  // 驱动浸水状态(外部设置, 0=正常 1=浸水)
    uint8_t water_mot_state;  // 电机浸水状态(外部设置, 0=正常 1=浸水)
    // 告警队列
    uint8_t  fault_queue[ALARM_QUEUE_SIZE];  // 活跃故障码队列(停机在前, 报警在后)
    uint8_t  fault_count;                     // 队列中活跃故障码数量
    uint8_t  rotate_index;                    // 当前轮转显示索引
} AlarmMonitor_t;

/************************ 公开接口 ************************/

/// @brief 初始化告警模块
void AlarmMonitor_Init(void);

/// @brief 告警检测处理(1ms周期调用)
void AlarmMonitor_Process(void);

/// @brief 设置浸水检测状态(由外部GPIO等调用)
/// @param drv_water 驱动浸水状态(0=正常 1=浸水)
/// @param mot_water 电机浸水状态(0=正常 1=浸水)
void AlarmMonitor_SetWaterState(uint8_t drv_water, uint8_t mot_water);

/// @brief 获取当前最高级别故障码
/// @retval 故障码(FAULT_NONE=无故障)
uint8_t AlarmMonitor_GetActiveFault(void);

/// @brief 获取告警队列中活跃故障码数量
/// @retval 活跃故障码数量
uint8_t AlarmMonitor_GetFaultCount(void);

/// @brief 获取告警队列中指定索引的故障码
/// @param index 索引(0 ~ fault_count-1)
/// @retval 故障码, 越界返回FAULT_NONE
uint8_t AlarmMonitor_GetFaultAt(uint8_t index);

/// @brief 切换到下一个故障码(每次读取故障时调用, 实现轮流上报)
void AlarmMonitor_RotateNext(void);

/// @brief 清除指定告警项
/// @param id 告警项ID
void AlarmMonitor_ClearAlarm(AlarmId_e id);

/// @brief 清除所有告警
void AlarmMonitor_ClearAll(void);

/// @brief 获取 OVP 输入电源电压(0.01V)，未采样时返回0
uint16_t AlarmMonitor_GetInputVoltageCv(void);

/// @brief 获取 NTC 驱动模块温度(0.1℃)，NTC_TEMP_ERR=9999 表示无效
int16_t AlarmMonitor_GetNtcTempX10(void);

/************************ 外部变量 ************************/
extern AlarmMonitor_t xdata g_alarm_monitor;

#endif // ALARM_MONITOR_H
