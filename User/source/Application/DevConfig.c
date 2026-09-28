/************************************************************************************
 * @Author       : FengYu
 * @Date         : 2026-07-07 14:18:18
 * @LastEditors  : zhangbao zhb@wujiang.com
 * @LastEditTime : 2026-07-08 13:31:17
 * @Description  : 设备配置模块
 * @
 * @Copyright (c) 2026 by ${git_name_email}, All Rights Reserved.
*************************************************************************************/

/******************************************************************************
 * 头文件
 ******************************************************************************/
#include "Myproject.h"
#include "DevConfig.h"
#include "CanProtocol.h"


/******************************************************************************
 * 宏定义
 ******************************************************************************/

#define AT24CXX_CFG_ADDR              0x0000
#define WARNING_INFO(enable, time)    (((enable & 0x01) << 6) | (time & 0x3F))
#define VALUE2TRAN(value)             ((int16_t)(value) - 32767)

#define MOTOR_DRV_CFG_DEFAULT() do { \
        s_dev_cfg.motor_cfg.hw_ver[0]           = 1; \
        s_dev_cfg.motor_cfg.sw_ver[0]           = 1; \
        s_dev_cfg.motor_cfg.proto_ver[0]        = 1; \  
        s_dev_cfg.motor_cfg.ctrl_type           = DEV_CTRL_TYPE; \
        s_dev_cfg.motor_cfg.speed_kp            = SKP; \
        s_dev_cfg.motor_cfg.speed_ki            = SKI; \
        s_dev_cfg.motor_cfg.speed_kd            = SKD; \
        s_dev_cfg.motor_cfg.motor_type          = MOTOR_SELECT; \
        s_dev_cfg.motor_cfg.speed_precision     = SPEED_PRECISION; \
        s_dev_cfg.motor_cfg.max_speed           = MAX_SPEED; \
        s_dev_cfg.motor_cfg.idle_speed          = IDLE_SPEED; \ 
        s_dev_cfg.motor_cfg.drv_water_alarm.value   = WARNING_INFO(0, 30); \
        s_dev_cfg.motor_cfg.drv_water_stop.value    = WARNING_INFO(0, 30); \
        s_dev_cfg.motor_cfg.mot_water_alarm.value   = WARNING_INFO(0, 30); \
        s_dev_cfg.motor_cfg.mot_water_stop.value    = WARNING_INFO(0, 30); \
        s_dev_cfg.motor_cfg.max_speed_inc       = 100; \
        s_dev_cfg.motor_cfg.min_speed_inc       = 10; \
        s_dev_cfg.motor_cfg.rated_voltage       = 3600; \
        s_dev_cfg.motor_cfg.rated_current       = 10000; \
        s_dev_cfg.motor_cfg.fwd_power_pct       = 100; \
        s_dev_cfg.motor_cfg.rev_power_pct       = 100; \  
        s_dev_cfg.motor_cfg.max_voltage         =  3600; \
        s_dev_cfg.motor_cfg.min_voltage         =  3000; \  
        s_dev_cfg.motor_cfg.max_current         = 10000; \
        s_dev_cfg.motor_cfg.volt_high_alarm     = VALUE2TRAN(3600); \
        s_dev_cfg.motor_cfg.volt_low_alarm      = VALUE2TRAN(3000); \
        s_dev_cfg.motor_cfg.volt_high_stop      = VALUE2TRAN(3600); \
        s_dev_cfg.motor_cfg.volt_alarm.value    = WARNING_INFO(0, 30); \
        s_dev_cfg.motor_cfg.volt_stop.value     = WARNING_INFO(0, 30); \
        s_dev_cfg.motor_cfg.curr_high_alarm     = VALUE2TRAN(10000); \
        s_dev_cfg.motor_cfg.curr_low_alarm      = VALUE2TRAN(0); \
        s_dev_cfg.motor_cfg.curr_high_stop      = VALUE2TRAN(0); \
        s_dev_cfg.motor_cfg.curr_alarm.value    = WARNING_INFO(0, 30); \
        s_dev_cfg.motor_cfg.curr_stop.value     = WARNING_INFO(0, 30); \
        s_dev_cfg.motor_cfg.drv_temp_high_alarm = VALUE2TRAN(100); \
        s_dev_cfg.motor_cfg.drv_temp_low_alarm  = VALUE2TRAN(0); \
        s_dev_cfg.motor_cfg.drv_temp_high_stop  = VALUE2TRAN(0); \  
        s_dev_cfg.motor_cfg.drv_temp_alarm.value= WARNING_INFO(0, 30); \
        s_dev_cfg.motor_cfg.drv_temp_stop.value = WARNING_INFO(0, 30); \    
        s_dev_cfg.motor_cfg.mot_temp_high_alarm = VALUE2TRAN(100); \
        s_dev_cfg.motor_cfg.mot_temp_low_alarm  = VALUE2TRAN(0); \
        s_dev_cfg.motor_cfg.mot_temp_high_stop  = VALUE2TRAN(0); \
        s_dev_cfg.motor_cfg.mot_temp_alarm.value= WARNING_INFO(0, 30); \
        s_dev_cfg.motor_cfg.mot_temp_stop.value = WARNING_INFO(0, 30); \
        s_dev_cfg.motor_cfg.overspeed_alarm     = VALUE2TRAN(100); \
        s_dev_cfg.motor_cfg.stall_alarm         = VALUE2TRAN(0); \
        s_dev_cfg.motor_cfg.overspeed_stop      = VALUE2TRAN(0); \
        s_dev_cfg.motor_cfg.speed_alarm.value   = WARNING_INFO(0, 30); \
        s_dev_cfg.motor_cfg.speed_stop.value    = WARNING_INFO(0, 30); \
    } while(0)

/******************************************************************************
 * 函数声明
 ******************************************************************************/

static uint32_t DevConfig_CalcCRC(const uint8_t *dat, uint16_t len);


/******************************************************************************
 * 内部变量
 ******************************************************************************/
static DevConfig_t xdata s_dev_cfg; ///< 设备配置参数结构体
MotorDrvCfg_t xdata *g_motor_cfg; ///< 电机驱动配置参数指针


/******************************************************************************
 * 参数读写接口
 ******************************************************************************/

/// @brief 载入本地参数
/// @param cfg 配置参数结构体指针
/// @retval None
void DevConfig_LoadParam(void)
{
    AT24Cxx_Init();
    
    AT24Cxx_ReadBytes(AT24CXX_CFG_ADDR, (uint8 *)&s_dev_cfg, sizeof(DevConfig_t));

    if (s_dev_cfg.magic != DEV_CONFIG_MAGIC_NUMBER ||
        s_dev_cfg.crc != DevConfig_CalcCRC((const uint8_t *)&s_dev_cfg.motor_cfg, sizeof(MotorDrvCfg_t)))
    {
        memset(&s_dev_cfg, 0, sizeof(DevConfig_t));
        s_dev_cfg.magic = DEV_CONFIG_MAGIC_NUMBER;
        MOTOR_DRV_CFG_DEFAULT();
        s_dev_cfg.crc = 0;
    }

    g_motor_cfg = &s_dev_cfg.motor_cfg;

}

/// @brief 写入设备配置参数
/// @param cfg 配置参数结构体指针
/// @retval None
void DevConfig_WriteParam(const void *cfg)
{
    if (cfg == NULL)
        return;

    if (memcmp(&s_dev_cfg.motor_cfg, cfg, sizeof(MotorDrvCfg_t)) == 0)
        return;

    s_dev_cfg.crc = DevConfig_CalcCRC((const uint8_t *)cfg, sizeof(MotorDrvCfg_t));

    memcpy(&s_dev_cfg.motor_cfg, cfg, sizeof(MotorDrvCfg_t));

    AT24Cxx_WriteBytes(AT24CXX_CFG_ADDR, (const uint8 *)&s_dev_cfg, sizeof(DevConfig_t));
}

/// @brief 获取设备配置参数指针
/// @param None
/// @return 配置参数结构体指针
void *DevConfig_GetParam(void)
{
    return g_motor_cfg;
}

/// @brief 将配置中的速度环PI参数写入硬件寄存器
/// @param None
/// @retval None
void DevConfig_ApplySpeedPI(void)
{
    PI1_KP = g_motor_cfg->speed_kp;
    PI1_KI = g_motor_cfg->speed_ki;
}


/******************************************************************************
 * 内存读写接口(已注释)
 ******************************************************************************/

/*
/// @brief 从内存读取配置数据
/// @param addr 内存地址
/// @param buf 数据缓冲区指针
/// @param len 数据长度
/// @retval 实际读取的字节数
uint16 DevConfig_MemRead(uint32_t addr, uint8_t *buf, uint16_t len)
{
    uint16_t i;
    if (buf == NULL || len == 0) {
        return 0;
    }

    for (i = 0; i < len; i++) {
        buf[i] = *((uint8_t *)(addr + i));
    }

    return len;
}

/// @brief 向内存写入配置数据
/// @param addr 内存地址
/// @param buf 数据缓冲区指针
/// @param len 数据长度
/// @retval 实际写入的字节数
uint16 DevConfig_MemWrite(uint32_t addr, const uint8_t *buf, uint16_t len)
{
    uint16_t i;
    if (buf == NULL || len == 0) {
        return 0;
    }

    for (i = 0; i < len; i++) {
        *((uint8_t *)(addr + i)) = buf[i];
    }

    return len;
}
*/

/******************************************************************************
 * CRC计算函数
 ******************************************************************************/

/// @brief 计算配置参数CRC32校验码
/// @param cfg 配置参数结构体指针
/// @retval CRC32校验码
static uint32_t DevConfig_CalcCRC(const uint8_t *dat, uint16_t len)
{
    uint32_t crc = 0xFFFFFFFF;
    uint16_t i, j;

    for (i = 0; i < len; i++)
    {
        crc ^= (uint32_t)dat[i];
        for (j = 0; j < 8; j++)
        {
            if (crc & 0x01)
            {
                crc = (crc >> 1) ^ 0xEDB88320;
            }
            else
            {
                crc >>= 1;
            }
        }
    }

    return crc ^ 0xFFFFFFFF;
}

