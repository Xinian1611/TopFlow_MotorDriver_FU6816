#ifndef CAN_SLAVE_H
#define CAN_SLAVE_H

#include "CanProtocol.h"

/************************ 传感器/参数ID定义 ************************/
typedef enum {
    SENSOR_ID_DCBUS_VOLTAGE   = 0,    // 母线电压  int16
    SENSOR_ID_DCBUS_CURRENT   = 1,    // 母线电流  uint16
    SENSOR_ID_ANGLE_COM       = 2,    // 补偿角度  int16
    SENSOR_ID_NTC_TEMP        = 3,    // NTC温度   int16
    SENSOR_ID_POWER           = 4,    // 当前功率  int16
    SENSOR_ID_POWER_FLT       = 5,    // 功率滤波  int16
    SENSOR_ID_SPEED_FLT       = 6,    // 速度滤波  int16
    SENSOR_ID_SPEED_FLT_PRE   = 7,    // 上周期速度 int16
    SENSOR_ID_UQ_FLT          = 8,    // Q轴电压   int16
    SENSOR_ID_UD_FLT          = 9,    // D轴电压   int16
    SENSOR_ID_ALIGN_REF       = 10,   // 对齐参考  int16
    SENSOR_ID_ALIGN           = 11,   // 对齐值    int16
    SENSOR_ID_ALIGN_FLAG      = 12,   // 对齐标志  int16
    SENSOR_ID_ALIGN_SAMPLE    = 13,   // 对齐采样  int16
    SENSOR_ID_ALIGN_SAMPLE_PRE= 14,   // 对齐采样前 int16
    SENSOR_ID_ALIGN_ERR       = 15,   // 对齐误差  int16
    SENSOR_ID_ALIGN_THETA_ACC = 16,   // 对齐角度累 int16
    SENSOR_ID_ALIGN_ANGLE     = 17,   // 预定位角度 int16
    SENSOR_ID_ALIGN_ANGLE_TMP = 18,   // 对齐角度暂 int16
    SENSOR_ID_ALIGN_COUNT     = 19,   // 对齐计数  int16
    SENSOR_ID_MAX_IA          = 20,   // 最大A相电流 int16
    SENSOR_ID_MAX_IB          = 21,   // 最大B相电流 int16
    SENSOR_ID_MAX_IC          = 22,   // 最大C相电流 int16
    SENSOR_ID_REF             = 23,   // 控制目标给定 int16
    SENSOR_ID_IQ_REF          = 24,   // Q轴给定电流 int16
    SENSOR_ID_ID_REF          = 25,   // D轴给定电流 int16
    SENSOR_ID_IQ_SPEED_REF    = 26,   // Q轴速度给定 int16
    SENSOR_ID_IQ_CUR_REF      = 27,   // Q轴母线电流给定 int16
    SENSOR_ID_WEAK_REF        = 28,   // 弱磁给定   int16
    SENSOR_ID_WEAK_ANGLE      = 29,   // 弱磁角度   int16
    SENSOR_ID_SQRTUDQ         = 30,   // sqrt(Udq)  uint16
    SENSOR_ID_ES_VALUE        = 31,   // FOC_ESQU滤波 uint16
    SENSOR_ID_LOOP_TIME       = 32,   // 外环控制周期 uint16
    SENSOR_ID_STATE_COUNT     = 33,   // 状态计数   uint16
    SENSOR_ID_POS_CHECK_ANGLE = 34,   // 位置检测角度 int16
    SENSOR_ID_MCU_TEMP        = 35,   // MCU温度    uint8
    SENSOR_ID_MDU_TEMP        = 36,   // MDU温度    int16
    SENSOR_ID_RUN_STATE_CNT   = 37,   // 运行状态计数 uint16
} SensorId_t;

typedef struct {
    uint8 isOn;
    float ref;
} CanSlaveSetParam_t;

/************************ 扩展帧从机配置 ************************/
typedef struct {
    uint8_t sys_num;     // 系统编号
    uint8_t main_cat;    // 设备主类别
    uint8_t sub_cat;     // 设备副类别
    uint8_t dev_num;     // 设备编号
} ExtSlaveConfig_t;

/************************ 通信超时定义 ************************/
#define EXT_CMD_TIMEOUT_MS      1000   // 指令报文通信超时(ms)

/************************ 扩展帧运行状态 ************************/

/**
 * @brief  扩展帧从机初始化
 * @param  config  本机扩展帧设备配置
 * @retval None
 */
void ext_slave_init(const ExtSlaveConfig_t *config);

/**
 * @brief  处理接收到的扩展帧CAN命令（扩展帧协议）
 * @param  ext_id  29位扩展帧ID
 * @param  dat     数据缓冲区
 * @param  len     数据长度
 * @retval None
 */
void ext_slave_process_rx(uint32_t ext_id, const uint8_t *dat, uint8_t len);

/**
 * @brief  扩展帧周期性状态发送及通信超时检测（1ms周期调用）
 * @retval None
 */
void ext_slave_periodic_send(void);

/**
 * @brief  设置扩展帧故障码
 * @param  level  故障等级
 * @param  cod   故障编码
 * @retval None
 */
void ext_set_fault(FaultLevel_e level, uint8_t cod);

/**
 * @brief  清除扩展帧故障码
 * @retval None
 */
void ext_clear_fault(void);


#endif // CAN_SLAVE_H
