/************************************************************************************
 * @Author       : FengYu
 * @Date         : 2026-07-09
 * @Description  : 基于MotorDrvCfg_t的可配置告警监控模块实现
 * @Copyright (c) 2026 by FengYu, All Rights Reserved.
*************************************************************************************/

#include "AlarmMonitor.h"
#include "CanSlave.h"
#include <MyProject.h>

/************************ 模块变量 ************************/
AlarmMonitor_t xdata g_alarm_monitor;

/************************ 内部辅助函数 ************************/

/// @brief 判断WorningInfo_t是否使能
/// @param info 报警信息
/// @retval 1=使能 0=未使能
static uint8_t alarm_is_enabled(WorningInfo_t info)
{
    return (info.reg.enable != 0) ? 1 : 0;
}

/// @brief 获取WorningInfo_t超时时间(ms)
/// @param info 报警信息
/// @retval 超时时间(ms), 最小1000ms
static uint16_t alarm_get_timeout_ms(WorningInfo_t info)
{
    uint16_t t = info.reg.time;
    return (t > 0) ? (t * 1000u) : 1000u;
}

/// @brief 更新单项告警计时器并判断是否触发
/// @param cnt 计时器指针(ms)
/// @param active 激活标志指针
/// @param condition 1=条件满足 0=条件不满足
/// @param timeout_ms 超时时间(ms)
/// @retval 1=刚触发或已激活 0=未激活
static uint8_t alarm_update_timer(uint16_t *cnt, uint8_t *active, uint8_t condition, uint16_t timeout_ms)
{
    if (condition)
    {
        if (*active)
        {
            return 1;
        }
        if (*cnt < timeout_ms)
        {
            (*cnt)++;
        }
        if (*cnt >= timeout_ms)
        {
            *active = 1;
            return 1;
        }
    }
    else
    {
        *cnt = 0;
        *active = 0;
    }
    return 0;
}

/************************ 各类别告警检测 ************************/

/// @brief 电压告警检测
static void alarm_check_voltage(void)
{
    int16_t volt_001V;
    uint8_t over_cond, under_cond;

    volt_001V = (int16_t)(mcFocCtrl.mcDcbusFlt / UDC_Value(0.01));

    // 电压高报警/停机
    over_cond = (volt_001V > g_motor_cfg->volt_high_alarm) ? 1 : 0;
    if (alarm_is_enabled(g_motor_cfg->volt_alarm))
    {
        alarm_update_timer(&g_alarm_monitor.items[ALARM_VOLT_HIGH].alarm_cnt,
                           &g_alarm_monitor.items[ALARM_VOLT_HIGH].alarm_active,
                           over_cond,
                           alarm_get_timeout_ms(g_motor_cfg->volt_alarm));
    }

    if (alarm_is_enabled(g_motor_cfg->volt_stop))
    {
        over_cond = (volt_001V > g_motor_cfg->volt_high_stop) ? 1 : 0;
        alarm_update_timer(&g_alarm_monitor.items[ALARM_VOLT_HIGH].stop_cnt,
                           &g_alarm_monitor.items[ALARM_VOLT_HIGH].stop_active,
                           over_cond,
                           alarm_get_timeout_ms(g_motor_cfg->volt_stop));
    }

    // 电压低报警(仅报警,无停机)
    under_cond = (volt_001V < g_motor_cfg->volt_low_alarm) ? 1 : 0;
    if (alarm_is_enabled(g_motor_cfg->volt_alarm))
    {
        alarm_update_timer(&g_alarm_monitor.items[ALARM_VOLT_LOW].alarm_cnt,
                           &g_alarm_monitor.items[ALARM_VOLT_LOW].alarm_active,
                           under_cond,
                           alarm_get_timeout_ms(g_motor_cfg->volt_alarm));
    }
}

/// @brief 电流告警检测
static void alarm_check_current(void)
{
    int16_t curr_001A;
    uint8_t over_cond, under_cond;

    curr_001A = (int16_t)(mcFocCtrl.mcADCCurrentbus / I_Value(0.1));

    // 电流高报警/停机
    over_cond = (curr_001A > g_motor_cfg->curr_high_alarm) ? 1 : 0;
    if (alarm_is_enabled(g_motor_cfg->curr_alarm))
    {
        alarm_update_timer(&g_alarm_monitor.items[ALARM_CURR_HIGH].alarm_cnt,
                           &g_alarm_monitor.items[ALARM_CURR_HIGH].alarm_active,
                           over_cond,
                           alarm_get_timeout_ms(g_motor_cfg->curr_alarm));
    }

    if (alarm_is_enabled(g_motor_cfg->curr_stop))
    {
        over_cond = (curr_001A > g_motor_cfg->curr_high_stop) ? 1 : 0;
        alarm_update_timer(&g_alarm_monitor.items[ALARM_CURR_HIGH].stop_cnt,
                           &g_alarm_monitor.items[ALARM_CURR_HIGH].stop_active,
                           over_cond,
                           alarm_get_timeout_ms(g_motor_cfg->curr_stop));
    }

    // 电流低报警(仅报警)
    under_cond = (curr_001A < g_motor_cfg->curr_low_alarm) ? 1 : 0;
    if (alarm_is_enabled(g_motor_cfg->curr_alarm))
    {
        alarm_update_timer(&g_alarm_monitor.items[ALARM_CURR_LOW].alarm_cnt,
                           &g_alarm_monitor.items[ALARM_CURR_LOW].alarm_active,
                           under_cond,
                           alarm_get_timeout_ms(g_motor_cfg->curr_alarm));
    }
}

/// @brief 驱动温度告警检测
static void alarm_check_drv_temp(void)
{
    int16_t temp_01C;
    uint8_t over_cond, under_cond;

    temp_01C = (int16_t)(mcFocCtrl.MCU_TEMP * 10);

    // 驱动温度高报警/停机
    over_cond = (temp_01C > g_motor_cfg->drv_temp_high_alarm) ? 1 : 0;
    if (alarm_is_enabled(g_motor_cfg->drv_temp_alarm))
    {
        alarm_update_timer(&g_alarm_monitor.items[ALARM_DRV_TEMP_HIGH].alarm_cnt,
                           &g_alarm_monitor.items[ALARM_DRV_TEMP_HIGH].alarm_active,
                           over_cond,
                           alarm_get_timeout_ms(g_motor_cfg->drv_temp_alarm));
    }

    if (alarm_is_enabled(g_motor_cfg->drv_temp_stop))
    {
        over_cond = (temp_01C > g_motor_cfg->drv_temp_high_stop) ? 1 : 0;
        alarm_update_timer(&g_alarm_monitor.items[ALARM_DRV_TEMP_HIGH].stop_cnt,
                           &g_alarm_monitor.items[ALARM_DRV_TEMP_HIGH].stop_active,
                           over_cond,
                           alarm_get_timeout_ms(g_motor_cfg->drv_temp_stop));
    }

    // 驱动温度低报警(仅报警)
    under_cond = (temp_01C < g_motor_cfg->drv_temp_low_alarm) ? 1 : 0;
    if (alarm_is_enabled(g_motor_cfg->drv_temp_alarm))
    {
        alarm_update_timer(&g_alarm_monitor.items[ALARM_DRV_TEMP_LOW].alarm_cnt,
                           &g_alarm_monitor.items[ALARM_DRV_TEMP_LOW].alarm_active,
                           under_cond,
                           alarm_get_timeout_ms(g_motor_cfg->drv_temp_alarm));
    }
}

/// @brief 电机温度告警检测
static void alarm_check_mot_temp(void)
{
    int16_t temp_01C;
    uint8_t over_cond, under_cond;

    // NTC温度转换为0.1℃单位
    temp_01C = (int16_t)(mcFocCtrl.NTCTempFlt * 10);

    // 电机温度高报警/停机
    over_cond = (temp_01C > g_motor_cfg->mot_temp_high_alarm) ? 1 : 0;
    if (alarm_is_enabled(g_motor_cfg->mot_temp_alarm))
    {
        alarm_update_timer(&g_alarm_monitor.items[ALARM_MOT_TEMP_HIGH].alarm_cnt,
                           &g_alarm_monitor.items[ALARM_MOT_TEMP_HIGH].alarm_active,
                           over_cond,
                           alarm_get_timeout_ms(g_motor_cfg->mot_temp_alarm));
    }

    if (alarm_is_enabled(g_motor_cfg->mot_temp_stop))
    {
        over_cond = (temp_01C > g_motor_cfg->mot_temp_high_stop) ? 1 : 0;
        alarm_update_timer(&g_alarm_monitor.items[ALARM_MOT_TEMP_HIGH].stop_cnt,
                           &g_alarm_monitor.items[ALARM_MOT_TEMP_HIGH].stop_active,
                           over_cond,
                           alarm_get_timeout_ms(g_motor_cfg->mot_temp_stop));
    }

    // 电机温度低报警(仅报警)
    under_cond = (temp_01C < g_motor_cfg->mot_temp_low_alarm) ? 1 : 0;
    if (alarm_is_enabled(g_motor_cfg->mot_temp_alarm))
    {
        alarm_update_timer(&g_alarm_monitor.items[ALARM_MOT_TEMP_LOW].alarm_cnt,
                           &g_alarm_monitor.items[ALARM_MOT_TEMP_LOW].alarm_active,
                           under_cond,
                           alarm_get_timeout_ms(g_motor_cfg->mot_temp_alarm));
    }
}

/// @brief 转速告警检测
static void alarm_check_speed(void)
{
    int16_t speed_rpm;
    uint8_t over_cond, stall_cond;

    speed_rpm = (int16_t)(mcFocCtrl.SpeedFlt / S_Value(1));

    // 超速报警/停机
    over_cond = (speed_rpm > (int16_t)g_motor_cfg->overspeed_alarm) ? 1 : 0;
    if (alarm_is_enabled(g_motor_cfg->speed_alarm))
    {
        alarm_update_timer(&g_alarm_monitor.items[ALARM_OVERSPEED].alarm_cnt,
                           &g_alarm_monitor.items[ALARM_OVERSPEED].alarm_active,
                           over_cond,
                           alarm_get_timeout_ms(g_motor_cfg->speed_alarm));
    }

    if (alarm_is_enabled(g_motor_cfg->speed_stop))
    {
        over_cond = (speed_rpm > (int16_t)g_motor_cfg->overspeed_stop) ? 1 : 0;
        alarm_update_timer(&g_alarm_monitor.items[ALARM_OVERSPEED].stop_cnt,
                           &g_alarm_monitor.items[ALARM_OVERSPEED].stop_active,
                           over_cond,
                           alarm_get_timeout_ms(g_motor_cfg->speed_stop));
    }

    // 失速报警(仅报警)
    stall_cond = (speed_rpm < (int16_t)g_motor_cfg->stall_alarm) ? 1 : 0;
    if (alarm_is_enabled(g_motor_cfg->speed_alarm))
    {
        alarm_update_timer(&g_alarm_monitor.items[ALARM_STALL].alarm_cnt,
                           &g_alarm_monitor.items[ALARM_STALL].alarm_active,
                           stall_cond,
                           alarm_get_timeout_ms(g_motor_cfg->speed_alarm));
    }
}

/// @brief 浸水告警检测
static void alarm_check_water(void)
{
    // 驱动浸水报警/停机
    if (alarm_is_enabled(g_motor_cfg->drv_water_alarm))
    {
        alarm_update_timer(&g_alarm_monitor.items[ALARM_DRV_WATER].alarm_cnt,
                           &g_alarm_monitor.items[ALARM_DRV_WATER].alarm_active,
                           g_alarm_monitor.water_drv_state,
                           alarm_get_timeout_ms(g_motor_cfg->drv_water_alarm));
    }

    if (alarm_is_enabled(g_motor_cfg->drv_water_stop))
    {
        alarm_update_timer(&g_alarm_monitor.items[ALARM_DRV_WATER].stop_cnt,
                           &g_alarm_monitor.items[ALARM_DRV_WATER].stop_active,
                           g_alarm_monitor.water_drv_state,
                           alarm_get_timeout_ms(g_motor_cfg->drv_water_stop));
    }

    // 电机浸水报警/停机
    if (alarm_is_enabled(g_motor_cfg->mot_water_alarm))
    {
        alarm_update_timer(&g_alarm_monitor.items[ALARM_MOT_WATER].alarm_cnt,
                           &g_alarm_monitor.items[ALARM_MOT_WATER].alarm_active,
                           g_alarm_monitor.water_mot_state,
                           alarm_get_timeout_ms(g_motor_cfg->mot_water_alarm));
    }

    if (alarm_is_enabled(g_motor_cfg->mot_water_stop))
    {
        alarm_update_timer(&g_alarm_monitor.items[ALARM_MOT_WATER].stop_cnt,
                           &g_alarm_monitor.items[ALARM_MOT_WATER].stop_active,
                           g_alarm_monitor.water_mot_state,
                           alarm_get_timeout_ms(g_motor_cfg->mot_water_stop));
    }
}

/************************ 故障码映射与更新 ************************/

/// @brief 停机故障码映射
/// @param id 告警项ID
/// @retval 故障码, 无映射返回FAULT_NONE
static uint8_t alarm_map_stop_fault(AlarmId_e id)
{
    switch (id)
    {
    case ALARM_DRV_WATER:
    case ALARM_MOT_WATER:
        return FAULT_BUILD(FAULT_LEVEL_3, FAULT3_WATER_STOP);
    case ALARM_VOLT_HIGH:
        return FAULT_BUILD(FAULT_LEVEL_3, FAULT3_OVERVOLT_STOP);
    case ALARM_CURR_HIGH:
        return FAULT_BUILD(FAULT_LEVEL_3, FAULT3_OVERCURR_STOP);
    case ALARM_DRV_TEMP_HIGH:
        return FAULT_BUILD(FAULT_LEVEL_3, FAULT3_DRVTEMP_STOP);
    case ALARM_MOT_TEMP_HIGH:
        return FAULT_BUILD(FAULT_LEVEL_3, FAULT3_MOTTEMP_STOP);
    case ALARM_OVERSPEED:
        return FAULT_BUILD(FAULT_LEVEL_3, FAULT3_OVERSPEED_STOP);
    default:
        return FAULT_NONE;
    }
}

/// @brief 报警故障码映射
/// @param id 告警项ID
/// @retval 故障码, 无映射返回FAULT_NONE
static uint8_t alarm_map_alarm_fault(AlarmId_e id)
{
    switch (id)
    {
    case ALARM_DRV_WATER:
    case ALARM_MOT_WATER:
        return FAULT_BUILD(FAULT_LEVEL_2, FAULT2_WATER_ALARM);
    case ALARM_VOLT_HIGH:
        return FAULT_BUILD(FAULT_LEVEL_2, FAULT2_OVERVOLT_ALARM);
    case ALARM_VOLT_LOW:
        return FAULT_BUILD(FAULT_LEVEL_1, FAULT1_UNDERVOLT_ALARM);
    case ALARM_CURR_HIGH:
        return FAULT_BUILD(FAULT_LEVEL_2, FAULT2_OVERCURR_ALARM);
    case ALARM_DRV_TEMP_HIGH:
        return FAULT_BUILD(FAULT_LEVEL_2, FAULT2_DRVTEMP_ALARM);
    case ALARM_MOT_TEMP_HIGH:
        return FAULT_BUILD(FAULT_LEVEL_2, FAULT2_MOTTEMP_ALARM);
    case ALARM_OVERSPEED:
        return FAULT_BUILD(FAULT_LEVEL_2, FAULT2_OVERSPEED_ALARM);
    case ALARM_STALL:
        return FAULT_BUILD(FAULT_LEVEL_1, FAULT1_STALL_ALARM);
    default:
        return FAULT_NONE;
    }
}

/// @brief 构建活跃故障码队列(停机在前, 报警在后)
static void alarm_build_queue(void)
{
    uint8_t i, fault;
    g_alarm_monitor.fault_count = 0;

    // 先收集停机故障(高优先级)
    for (i = 0; i < ALARM_COUNT; i++)
    {
        if (g_alarm_monitor.items[i].stop_active)
        {
            fault = alarm_map_stop_fault((AlarmId_e)i);
            if (fault != FAULT_NONE && g_alarm_monitor.fault_count < ALARM_QUEUE_SIZE)
            {
                g_alarm_monitor.fault_queue[g_alarm_monitor.fault_count++] = fault;
            }
        }
    }

    // 再收集报警故障
    for (i = 0; i < ALARM_COUNT; i++)
    {
        if (g_alarm_monitor.items[i].alarm_active)
        {
            fault = alarm_map_alarm_fault((AlarmId_e)i);
            if (fault != FAULT_NONE && g_alarm_monitor.fault_count < ALARM_QUEUE_SIZE)
            {
                g_alarm_monitor.fault_queue[g_alarm_monitor.fault_count++] = fault;
            }
        }
    }
}

/// @brief 根据告警队列更新故障码
static void alarm_update_fault(void)
{
    // 构建活跃故障码队列
    alarm_build_queue();

    // 无活跃故障
    if (g_alarm_monitor.fault_count == 0)
    {
        // 清除故障码(通信超时故障不由告警模块管理, 不清除)
        if (g_run_state.fault_code != FAULT_NONE &&
            !(FAULT_GET_LEVEL(g_run_state.fault_code) == FAULT_LEVEL_3 &&
              FAULT_GET_CODE(g_run_state.fault_code) == FAULT3_COMM_STOP))
        {
            ext_clear_fault();
        }
        g_alarm_monitor.rotate_index = 0;
        return;
    }

    // 修正轮转索引越界
    if (g_alarm_monitor.rotate_index >= g_alarm_monitor.fault_count)
    {
        g_alarm_monitor.rotate_index = 0;
    }

    // 设置当前轮转位置的故障码到运行状态
    g_run_state.fault_code = g_alarm_monitor.fault_queue[g_alarm_monitor.rotate_index];
}

/************************ 公开接口 ************************/

/// @brief 初始化告警模块
void AlarmMonitor_Init(void)
{
    memset(&g_alarm_monitor, 0, sizeof(AlarmMonitor_t));
}

/// @brief 告警检测处理(1ms周期调用)
void AlarmMonitor_Process(void)
{
    if (g_motor_cfg == NULL)
    {
        return;
    }

    alarm_check_voltage();
    alarm_check_current();
    alarm_check_drv_temp();
    alarm_check_mot_temp();
    alarm_check_speed();
    alarm_check_water();

    alarm_update_fault();
}

/// @brief 设置浸水检测状态
void AlarmMonitor_SetWaterState(uint8_t drv_water, uint8_t mot_water)
{
    g_alarm_monitor.water_drv_state = drv_water ? 1 : 0;
    g_alarm_monitor.water_mot_state = mot_water ? 1 : 0;
}

/// @brief 读取当前故障码并切换到下一个(轮流上报)
uint8_t AlarmMonitor_GetActiveFault(void)
{
    uint8_t fault = g_run_state.fault_code;
    AlarmMonitor_RotateNext();
    return fault;
}

/// @brief 获取告警队列中活跃故障码数量
uint8_t AlarmMonitor_GetFaultCount(void)
{
    return g_alarm_monitor.fault_count;
}

/// @brief 获取告警队列中指定索引的故障码
uint8_t AlarmMonitor_GetFaultAt(uint8_t index)
{
    if (index < g_alarm_monitor.fault_count)
    {
        return g_alarm_monitor.fault_queue[index];
    }
    return FAULT_NONE;
}

/// @brief 切换到下一个故障码(每次读取时调用)
void AlarmMonitor_RotateNext(void)
{
    if (g_alarm_monitor.fault_count == 0)
    {
        return;
    }
    g_alarm_monitor.rotate_index++;
    if (g_alarm_monitor.rotate_index >= g_alarm_monitor.fault_count)
    {
        g_alarm_monitor.rotate_index = 0;
    }
    g_run_state.fault_code = g_alarm_monitor.fault_queue[g_alarm_monitor.rotate_index];
}

/// @brief 清除指定告警项
void AlarmMonitor_ClearAlarm(AlarmId_e id)
{
    if (id < ALARM_COUNT)
    {
        g_alarm_monitor.items[id].alarm_cnt = 0;
        g_alarm_monitor.items[id].stop_cnt = 0;
        g_alarm_monitor.items[id].alarm_active = 0;
        g_alarm_monitor.items[id].stop_active = 0;
    }
}

/// @brief 清除所有告警
void AlarmMonitor_ClearAll(void)
{
    uint8_t i;
    for (i = 0; i < ALARM_COUNT; i++)
    {
        g_alarm_monitor.items[i].alarm_cnt = 0;
        g_alarm_monitor.items[i].stop_cnt = 0;
        g_alarm_monitor.items[i].alarm_active = 0;
        g_alarm_monitor.items[i].stop_active = 0;
    }
}
