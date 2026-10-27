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

volatile uint8_t  xdata g_alarm_evt       = ALARM_EVT_NONE;
volatile uint16_t xdata g_alarm_evt_value = 0;
volatile uint16_t xdata g_alarm_evt_limit = 0;

/************************ OVP/NTC 保护状态 ************************/
static int16_t xdata s_ovp_filtered = 0;   /* OVP ADC 滤波值(0~32767) */
static uint16_t xdata s_ovp_volt_cv  = 0;   /* OVP 换算后的输入电压(0.01V) */
static uint8_t xdata s_ovp_inited   = 0;   /* 首次采样直接装载，避免上电从0爬升 */

/************************ 内部辅助函数 ************************/

/// @brief 判断WorningInfo_t是否使能
/// @param info 报警信息
/// @retval 1=使能 0=未使能
static uint8_t alarm_is_enabled(WorningInfo_t info)
{
    /* 0=无操作，1=使能，2=禁止，3=无效 */
    return (info.reg.enable == 1) ? 1 : 0;
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

/// @brief 当前配置的电池类型(非法值按铅酸处理)
static uint8_t alarm_get_battery_type(void)
{
    /*   电池类型：0 = 铅酸，1 = 磷酸铁锂 */
    if (g_motor_cfg->reserved3 == BATTERY_TYPE_LIFEPO4)
    {
        return BATTERY_TYPE_LIFEPO4;
    }
    return BATTERY_TYPE_LEAD_ACID;
}

/// @brief 按电池类型取电压阈值(0.01V)
static void alarm_get_voltage_limits(uint16_t *low_alarm,
                                     uint16_t *high_alarm,
                                     uint16_t *high_stop)
{
    if (alarm_get_battery_type() == BATTERY_TYPE_LIFEPO4)
    {
        *low_alarm  = BAT_LIFEPO4_LOW_ALARM_CV;
        *high_alarm = BAT_LIFEPO4_HIGH_ALARM_CV;
        *high_stop  = BAT_LIFEPO4_HIGH_STOP_CV;
    }
    else
    {
        *low_alarm  = BAT_LEAD_ACID_LOW_ALARM_CV;
        *high_alarm = BAT_LEAD_ACID_HIGH_ALARM_CV;
        *high_stop  = BAT_LEAD_ACID_HIGH_STOP_CV;
    }
}

/// @brief 读取 OVP 输入电源电压
/// @retval 0.01V；换算：Vraw = raw × VREF × 1150/150 / 32768
static uint16_t alarm_read_ovp_cv(void)
{
    int16_t raw;

    raw = (int16_t)(OVP_ADC_DR & 0x7FFF);

    if (s_ovp_inited == 0)
    {
        s_ovp_filtered = raw;          /* 首次直接装载，防止上电从0爬升误报低压 */
        s_ovp_inited   = 1;
    }
    else
    {
        s_ovp_filtered = MDU_LPF0(raw, s_ovp_filtered, OVP_LPF_K);
    }

    /* raw(0~32767) × 满量程(0.01V) / 32768
     * 1150/150、VREF=4.5V 时满量程 = 3450（34.50V） */
    s_ovp_volt_cv = (uint16_t)(((uint32_t)(uint16_t)s_ovp_filtered
                                * (uint32_t)OVP_FULL_SCALE_CV) >> 15);
    return s_ovp_volt_cv;
}

/// @brief 电压告警检测(输入电源 PWR+，来源 P3.3/AD6 OVP)
static void alarm_check_voltage(void)
{
#if (BAT_ALARM_EN)
    uint16_t volt_cv;
    uint16_t low_alarm, high_alarm, high_stop;
    uint8_t  was_active;

    volt_cv = alarm_read_ovp_cv();
    low_alarm  = (uint16_t)TRAN2VALUE(g_motor_cfg->volt_low_alarm);
    high_alarm = (uint16_t)TRAN2VALUE(g_motor_cfg->volt_high_alarm);
    high_stop  = (uint16_t)TRAN2VALUE(g_motor_cfg->volt_high_stop);

    if (alarm_is_enabled(g_motor_cfg->volt_alarm))
    {
        was_active = g_alarm_monitor.items[ALARM_VOLT_HIGH].alarm_active;
        alarm_update_timer(&g_alarm_monitor.items[ALARM_VOLT_HIGH].alarm_cnt,
                        &g_alarm_monitor.items[ALARM_VOLT_HIGH].alarm_active,
                        (volt_cv >= high_alarm) ? 1 : 0,
                        alarm_get_timeout_ms(g_motor_cfg->volt_alarm));
        if ((was_active == 0) && (g_alarm_monitor.items[ALARM_VOLT_HIGH].alarm_active != 0))
        {
            g_alarm_evt_value = volt_cv;
            g_alarm_evt_limit = high_alarm;
            g_alarm_evt       = ALARM_EVT_VOLT_HIGH;
        }
    }

    /* 电压高停机(由配置 volt_stop 的使能位决定是否启用) */
    if (alarm_is_enabled(g_motor_cfg->volt_stop))
    {
        alarm_update_timer(&g_alarm_monitor.items[ALARM_VOLT_HIGH].stop_cnt,
                           &g_alarm_monitor.items[ALARM_VOLT_HIGH].stop_active,
                           (volt_cv >= high_stop) ? 1 : 0,
                           alarm_get_timeout_ms(g_motor_cfg->volt_stop));
    }

    if (alarm_is_enabled(g_motor_cfg->volt_alarm))
    {
        was_active = g_alarm_monitor.items[ALARM_VOLT_LOW].alarm_active;
        alarm_update_timer(&g_alarm_monitor.items[ALARM_VOLT_LOW].alarm_cnt,
                           &g_alarm_monitor.items[ALARM_VOLT_LOW].alarm_active,
                           (volt_cv <= low_alarm) ? 1 : 0,
                           alarm_get_timeout_ms(g_motor_cfg->volt_alarm));
        if ((was_active == 0) && (g_alarm_monitor.items[ALARM_VOLT_LOW].alarm_active != 0))
        {
            g_alarm_evt_value = volt_cv;
            g_alarm_evt_limit = low_alarm;
            g_alarm_evt       = ALARM_EVT_VOLT_LOW;
        }
    }
    else
    {
        g_alarm_monitor.items[ALARM_VOLT_LOW].alarm_cnt    = 0;
        g_alarm_monitor.items[ALARM_VOLT_LOW].alarm_active = 0;
    }
#endif
}

/// @brief 电流告警检测
static void alarm_check_current(void)
{
    int16_t curr_001A;
    uint8_t over_cond, under_cond;

    curr_001A = (int16_t)(mcFocCtrl.mcADCCurrentbus / I_Value(0.01));  /* 0.01A */
    // 电流高报警/停机
    over_cond = (curr_001A > (int16_t)TRAN2VALUE(g_motor_cfg->curr_high_alarm)) ? 1 : 0;

    if (alarm_is_enabled(g_motor_cfg->curr_alarm))
    {
        alarm_update_timer(&g_alarm_monitor.items[ALARM_CURR_HIGH].alarm_cnt,
                           &g_alarm_monitor.items[ALARM_CURR_HIGH].alarm_active,
                           over_cond,
                           alarm_get_timeout_ms(g_motor_cfg->curr_alarm));
    }

    if (alarm_is_enabled(g_motor_cfg->curr_stop))
    {
        over_cond = (curr_001A > (int16_t)TRAN2VALUE(g_motor_cfg->curr_high_stop)) ? 1 : 0;
        alarm_update_timer(&g_alarm_monitor.items[ALARM_CURR_HIGH].stop_cnt,
                           &g_alarm_monitor.items[ALARM_CURR_HIGH].stop_active,
                           over_cond,
                           alarm_get_timeout_ms(g_motor_cfg->curr_stop));
    }

    // 电流低报警(仅报警)
    under_cond = (curr_001A < (int16_t)TRAN2VALUE(g_motor_cfg->curr_low_alarm)) ? 1 : 0;
    if (alarm_is_enabled(g_motor_cfg->curr_alarm))
    {
        alarm_update_timer(&g_alarm_monitor.items[ALARM_CURR_LOW].alarm_cnt,
                           &g_alarm_monitor.items[ALARM_CURR_LOW].alarm_active,
                           under_cond,
                           alarm_get_timeout_ms(g_motor_cfg->curr_alarm));
    }
}

/// @brief 驱动模块温度告警检测(NTC，来源 P2.4/AD2)
static void alarm_check_drv_temp(void)
{
#if (BAT_ALARM_EN)
    int16_t temp_01C;
    uint8_t was_active;

    temp_01C = mcFocCtrl.NTCTempNum;

    if ((temp_01C == NTC_TEMP_ERR) ||
        (temp_01C < -400) ||
        (temp_01C > 1500))
    {
        return;
    }

    if (alarm_is_enabled(g_motor_cfg->drv_temp_alarm))
    {
        /* 驱动温度高报警：阈值/时间都来自配置 */
        was_active = g_alarm_monitor.items[ALARM_DRV_TEMP_HIGH].alarm_active;
        alarm_update_timer(&g_alarm_monitor.items[ALARM_DRV_TEMP_HIGH].alarm_cnt,
                            &g_alarm_monitor.items[ALARM_DRV_TEMP_HIGH].alarm_active,
                            (temp_01C >= (int16_t)TRAN2VALUE(g_motor_cfg->drv_temp_high_alarm)) ? 1 : 0,
                           alarm_get_timeout_ms(g_motor_cfg->drv_temp_alarm));
        if ((was_active == 0) && (g_alarm_monitor.items[ALARM_DRV_TEMP_HIGH].alarm_active != 0))
        {
            g_alarm_evt_value = (uint16_t)temp_01C;
            g_alarm_evt_limit = (uint16_t)TRAN2VALUE(g_motor_cfg->drv_temp_high_alarm);
            g_alarm_evt       = ALARM_EVT_TEMP_HIGH;
        }
    }
    else
    {
        g_alarm_monitor.items[ALARM_DRV_TEMP_HIGH].alarm_cnt    = 0;
        g_alarm_monitor.items[ALARM_DRV_TEMP_HIGH].alarm_active = 0;
    }

    /* 驱动温度高停机(默认不使能，由配置 drv_temp_stop 的使能位决定) */
    if (alarm_is_enabled(g_motor_cfg->drv_temp_stop))
    {
        alarm_update_timer(&g_alarm_monitor.items[ALARM_DRV_TEMP_HIGH].stop_cnt,
                           &g_alarm_monitor.items[ALARM_DRV_TEMP_HIGH].stop_active,
                           (temp_01C >= (int16_t)TRAN2VALUE(g_motor_cfg->drv_temp_high_stop)) ? 1 : 0,
                           alarm_get_timeout_ms(g_motor_cfg->drv_temp_stop));
    }
#endif
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

    s_ovp_filtered = 0;
    s_ovp_volt_cv  = 0;
    s_ovp_inited   = 0;

    g_alarm_evt       = ALARM_EVT_NONE;
    g_alarm_evt_value = 0;
    g_alarm_evt_limit = 0;
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

/// @brief 获取 OVP 输入电源电压(0.01V)，未采样时返回0
uint16_t AlarmMonitor_GetInputVoltageCv(void)
{
    return s_ovp_volt_cv;
}

/// @brief 获取 NTC 驱动模块温度(0.1℃)，NTC_TEMP_ERR=9999 表示无效
int16_t AlarmMonitor_GetNtcTempX10(void)
{
    return mcFocCtrl.NTCTempNum;
}
