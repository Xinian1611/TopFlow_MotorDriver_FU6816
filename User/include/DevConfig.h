/************************************************************************************
 * @Author       : FengYu
 * @Date         : 2026-07-07 14:08:44
 * @LastEditors  : zhangbao zhb@wujiang.com
 * @LastEditTime : 2026-07-08 13:39:20
 * @Description  :
 * @
 * @Copyright (c) 2026 by ${git_name_email}, All Rights Reserved.
*************************************************************************************/


#ifndef DEV_CONFIG_H
#define DEV_CONFIG_H

#include "CanProtocol.h"

#define DEV_CONFIG_MAGIC_NUMBER 0xA5A5A5A5 ///< 配置参数有效性标识

#define DEV_CTRL_TYPE            1 ///< 设备所属操控端类型(0:无操作, 1:船端, 2:集控端, 3:移动端)


/// @brief 报警信息结构体
typedef union {
    uint8_t value;
    struct
    {
        uint8_t time : 6;          ///< 持续时间/延迟时间(1-63s)
        uint8_t enable : 2;        ///< 使能(0:无操作, 1:使能, 2:使能, 3:使能)
    } reg;
} WorningInfo_t;

/// @brief 设备配置结构体
typedef struct {

    uint8_t hw_ver[4];              // Byte0-3: 硬件版本号(aaa.bbb.ccc.ddd)
    uint8_t sw_ver[4];              // 软件版本号(aaa.bbb.ccc.ddd)
    uint8_t proto_ver[4];           // 规约版本号(aaa.bbb.ccc.ddd)
    uint8_t ctrl_type;              // 设备所属操控端类型(0:无操作, 1:船端, 2:集控端, 3:移动端)
    uint16_t speed_kp;              // 速度环KP系数(Q12格式)
    uint16_t speed_ki;              // 速度环KI系数(Q15格式)
    uint16_t speed_kd;              // 速度环KD系数(Q12格式)
    uint8_t motor_type;             // 电机类型(0:无操作, 1:12V-600W, 2:24V-1200W, 3:36V-2500W)
    WorningInfo_t drv_water_alarm;  // 驱动浸水报警(低6位:持续时间1-63s, 高2位:使能0-3)
    WorningInfo_t drv_water_stop;   // 驱动浸水停机(低6位:延迟时间1-63s, 高2位:使能0-3)
    WorningInfo_t mot_water_alarm;  // 电机浸水报警(低6位:持续时间1-63s, 高2位:使能0-3)
    WorningInfo_t mot_water_stop;   // 电机浸水停机(低6位:延迟时间1-63s, 高2位:使能0-3)
    uint8_t speed_precision;        // 转速控制精度(0-255rpm)
    uint16_t max_speed;             // 最高转速(0-5000rpm)
    uint16_t idle_speed;            // 怠速转速(0-5000rpm)
    uint8_t max_speed_inc;          // 最大转速控制增量(0-2550rpm, 10rpm)
    uint8_t min_speed_inc;          // 最小转速控制增量(0-2550rpm, 10rpm)
    uint8_t reserved3;              // 保留
    uint16_t rated_voltage;         // 额定电压(0.01V)
    uint16_t rated_current;         // 额定电流(0.01A)
    uint8_t fwd_power_pct;          // 正向允许输入功率百分比(0-100%)
    uint8_t rev_power_pct;          // 反向允许输入功率百分比(0-100%)
    uint8_t reserved4[2];           // 保留
    uint16_t max_voltage;           // 最大电压(0.01V)
    uint16_t min_voltage;           // 最小电压(0.01V)
    uint16_t max_current;           // 最大电流(0.01A)
    uint8_t reserved5[2];           // 保留
    int16_t volt_high_alarm;        // 电压高报警值(0.01V, 32767=0V)
    int16_t volt_low_alarm;         // 电压低报警值(0.01V, 32767=0V)
    int16_t volt_high_stop;         // 电压高停车值(0.01V, 32767=0V)
    WorningInfo_t volt_alarm;       // 电压报警(低6位:持续时间1-63s, 高2位:使能0-3)
    WorningInfo_t volt_stop;        // 电压停机(低6位:延迟时间1-63s, 高2位:使能0-3)
    int16_t curr_high_alarm;        // 电流高报警值(0.01A, 32767=0A)
    int16_t curr_low_alarm;         // 电流低报警值(0.01A, 32767=0A)
    int16_t curr_high_stop;         // 电流高停车值(0.01A, 32767=0A)
    WorningInfo_t curr_alarm;       // 电流报警(低6位:持续时间1-63s, 高2位:使能0-3)
    WorningInfo_t curr_stop;        // 电流停机(低6位:延迟时间1-63s, 高2位:使能0-3)
    int16_t drv_temp_high_alarm;    // 驱动温度高报警值(0.1℃, 32767=0℃)
    int16_t drv_temp_low_alarm;     // 驱动温度低报警值(0.1℃, 32767=0℃)
    int16_t drv_temp_high_stop;     // 驱动温度高停车值(0.1℃, 32767=0℃)
    WorningInfo_t drv_temp_alarm;   // 驱动温度报警(低6位:持续时间1-63s, 高2位:使能0-3)
    WorningInfo_t drv_temp_stop;    // 驱动温度停机(低6位:延迟时间1-63s, 高2位:使能0-3)
    int16_t mot_temp_high_alarm;    // 电机温度高报警值(0.1℃, 32767=0℃)
    int16_t mot_temp_low_alarm;     // 电机温度低报警值(0.1℃, 32767=0℃)
    int16_t mot_temp_high_stop;     // 电机温度高停车值(0.1℃, 32767=0℃)
    WorningInfo_t mot_temp_alarm;   // 电机温度报警(低6位:持续时间1-63s, 高2位:使能0-3)
    WorningInfo_t mot_temp_stop;    // 电机温度停机(低6位:延迟时间1-63s, 高2位:使能0-3)
    uint16_t overspeed_alarm;       // 超速报警值(0-5000rpm)
    uint16_t stall_alarm;           // 失速报警值(0-5000rpm)
    uint16_t overspeed_stop;        // 超速停车值(0-5000rpm)
    WorningInfo_t speed_alarm;      // 转速报警(低6位:持续时间1-63s, 高2位:使能0-3)
    WorningInfo_t speed_stop;       // 转速停机(低6位:延迟时间1-63s, 高2位:使能0-3)
} MotorDrvCfg_t;

typedef struct {
    uint32_t magic;          // 配置数据魔数
    MotorDrvCfg_t motor_cfg; // 电机驱动配置参数
    uint32_t crc;            // 配置数据CRC32校验码
} DevConfig_t;


extern MotorDrvCfg_t xdata *g_motor_cfg;


void DevConfig_LoadParam(void);
void DevConfig_WriteParam(const void *cfg);
void *DevConfig_GetParam(void);
void DevConfig_ApplySpeedPI(void);


#endif
// DEV_CONFIG_H
