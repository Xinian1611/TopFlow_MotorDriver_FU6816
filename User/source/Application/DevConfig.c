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
#define WARNING_INFO(enable, time)    (((enable & 0x03) << 6) | (time & 0x3F))
#define VALUE2TRAN(value)             ((uint16_t)(32767 + (int32)(value)))

#define MOTOR_DRV_CFG_DEFAULT() do { \
        s_dev_cfg.motor_cfg.hw_ver[0]           = 1; \
        s_dev_cfg.motor_cfg.hw_ver[1]           = 0xFF; \
        s_dev_cfg.motor_cfg.hw_ver[2]           = 0xFF; \
        s_dev_cfg.motor_cfg.hw_ver[3]           = 0xFF; \
        s_dev_cfg.motor_cfg.sw_ver[0]           = 1; \
        s_dev_cfg.motor_cfg.sw_ver[1]           = 0xFF; \
        s_dev_cfg.motor_cfg.sw_ver[2]           = 0xFF; \
        s_dev_cfg.motor_cfg.sw_ver[3]           = 0xFF; \
        s_dev_cfg.motor_cfg.proto_ver[0]        = 1; \
        s_dev_cfg.motor_cfg.proto_ver[1]        = 0xFF; \
        s_dev_cfg.motor_cfg.proto_ver[2]        = 0xFF; \
        s_dev_cfg.motor_cfg.proto_ver[3]        = 0xFF; \
        s_dev_cfg.motor_cfg.ctrl_type           = DEV_CTRL_TYPE; \
        s_dev_cfg.motor_cfg.motor_type          = MOTOR_SELECT; \
        s_dev_cfg.motor_cfg.reserved3           = 0xFF; \
        memset(s_dev_cfg.motor_cfg.reserved1, 0xFF, sizeof(s_dev_cfg.motor_cfg.reserved1)); \
        memset(s_dev_cfg.motor_cfg.reserved2, 0xFF, sizeof(s_dev_cfg.motor_cfg.reserved2)); \
        memset(s_dev_cfg.motor_cfg.reserved4, 0xFF, sizeof(s_dev_cfg.motor_cfg.reserved4)); \
        memset(s_dev_cfg.motor_cfg.reserved5, 0xFF, sizeof(s_dev_cfg.motor_cfg.reserved5)); \
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
        s_dev_cfg.motor_cfg.overspeed_alarm     = 100; \
        s_dev_cfg.motor_cfg.stall_alarm         = 0; \
        s_dev_cfg.motor_cfg.overspeed_stop      = 0; \
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
    (void)EEPROM_Init();

    /* 96 字节：读可以跨页，一次读完 */
    if (EEPROM_OK != EEPROM_ReadBuffer(AT24CXX_CFG_ADDR, sizeof(DevConfig_t), (uint8_t *)&s_dev_cfg))
    {
        memset(&s_dev_cfg, 0, sizeof(DevConfig_t));     /* 读失败 → 走下面默认值分支 */
    }

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
        return;      /* 和当前配置一样 → 直接返回，不写盘 */

    s_dev_cfg.crc = DevConfig_CalcCRC((const uint8_t *)cfg, sizeof(MotorDrvCfg_t));
    memcpy(&s_dev_cfg.motor_cfg, cfg, sizeof(MotorDrvCfg_t));

    /* 96 字节跨页（0x0000~0x005F）：用自动分页的接口层，内部拆成 64 + 32 两次页写 */
    (void)EEPROM_WriteBuffer(AT24CXX_CFG_ADDR, sizeof(DevConfig_t), (const uint8_t *)&s_dev_cfg);
}

/// @brief 强制把当前 s_dev_cfg 落盘（不做"是否变化"比较）
/// @note  内部写 96 字节 EEPROM，约 50ms 阻塞；只能在主循环里调用
void DevConfig_ForceSave(void)
{
    s_dev_cfg.crc = DevConfig_CalcCRC((const uint8_t *)&s_dev_cfg.motor_cfg,
                                      sizeof(MotorDrvCfg_t));
    (void)EEPROM_WriteBuffer(AT24CXX_CFG_ADDR, sizeof(DevConfig_t), (const uint8_t *)&s_dev_cfg);
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
    /* 0015 配置里没有 PI 字段，PI 用编译期默认值；
     * 如果还要在线调 PI，另开 EEPROM 区/私有报文，不要占 CAN 保留字节。 */
    PI1_KP = SKP;
    PI1_KI = SKI;
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

