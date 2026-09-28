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

/************************ 外部变量 ************************/
extern AlarmMonitor_t xdata g_alarm_monitor;

#endif // ALARM_MONITOR_H
