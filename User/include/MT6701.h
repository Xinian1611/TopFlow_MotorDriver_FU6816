#ifndef MT6701_H
#define MT6701_H
#include <FU68xx_6.h>

/* ============================================================================
 * MT6701 磁编码器（FU6866 / TopFlow_UpDownDriverBoard）
 * ----------------------------------------------------------------------------
 * 分工（与参考工程 D:\NEW\TopFlow_PowerBoard 一致）：
 *   ① I2C  P0.5=SDA / P0.6=SCL（PH_SEL2[I2C_FS]=1 功能转移后）
 *          —— 只用来"配置/标定"芯片：InitSequence 写寄存器 + 写零点 + 烧 EEPROM
 *   ② ADC  P2.5=AD3 读 MT6701 模拟输出 VOUT —— 位置(角度)的唯一来源
 *          量程 0.2V~1.8V ↔ 0~360°（0.1° 分辨率报告）
 * ============================================================================ */

/* ---------------- 开关 ---------------- */
#define MT6701_USE_HW_I2C     1   /* 1=FU6866 硬件 I2C；0=软件 I2C（同一对脚，兜底）*/
#define MT6701_SPEED_400K     0   /* 硬件 I2C 速率：0=100kHz  1=400kHz */
#define MT6701_BURN_EEPROM    1   /* InitSequence 内是否烧 EEPROM（所以它只能在标定时调）*/
#define MT6701_ZERO           0   /* 0=不校准；1=上电把当前位置写为零点（标定用，标定后改回0）*/
#define MT6701_ANGLE_DIR      1   /* 写 0x29.bit1：0=顺时针增角度 1=逆时针增角度 */

/* ---------------- I2C 器件参数（同参考工程 mt6701.h）---------------- */
#define MT6701_ADDR           0x06   /* 7 位地址 → 写 0x0C / 读 0x0D */
#define MT6701_REG_ANGLE_H    0x03
#define MT6701_REG_ANGLE_L    0x04
#define MT6701_REG_29         0x29
#define MT6701_REG_ZERO_H     0x32
#define MT6701_REG_ZERO_L     0x33
#define MT6701_REG_34         0x34
#define MT6701_REG_38         0x38
#define MT6701_REG_3E         0x3E
#define MT6701_REG_3F         0x3F
#define MT6701_REG_40         0x40
#define MT6701_REG_28         0x28
#define MT6701_REG_EE_1       0x09   /* 写 0xB3 触发 EEPROM 烧录 */
#define MT6701_REG_EE_2       0x0A   /* 写 0x05 触发 EEPROM 烧录 */

/* ---------------- 模拟量换算（同参考工程 Inter_zxmotor.c 的宏）---------------- */
#define MT6701_ANA_OFFSET_MV  200U    /* 0.2V → 0° */
#define MT6701_ANA_SPAN_MV    1600U   /* 0.2V~1.8V 跨度 → 360° */
#define MT6701_ANA_VREF_MV    4500U   /* 本板 ADC 参考 4.5V（参考工程是 3.3V）*/
#define MT6701_ANA_POINTS     10U     /* 去极值均值点数（10ms 窗口，和 10ms 位置环对齐）*/
#define MT6701_ANA_DROP       2U      /* 每端丢弃点数（10 点丢两端各 2 点，取中间 6 点）*/
#define MT6701_ANA_LPF_EN     0       /* 1=打开角度一阶低通(MT6701_GetAngleF)，0=直接返回不滤波 */
#define MT6701_ANA_LPF_ALPHA  0.3f    /* 一阶低通 α（参考工程参数表默认 0.3f），仅 LPF_EN=1 时有效 */

/* ---------------- 数据（Watch 已取消，按需发 CAN 观测）---------------- */
extern uint8  xdata g_mt6701_probe;      /* I2C 自检返回码：0=从机应答 */
extern uint8  xdata g_mt6701_err;        /* 最近一次 I2C 操作返回码 */
extern uint16 xdata g_mt6701_errCnt;     /* I2C 失败累计 */
extern uint8  xdata g_mt6701_boot[12];   /* InitSequence 各寄存器回读值 */
extern uint16 xdata g_mt6701_zero;       /* 本次写入的零点（12 位）*/
extern uint16 xdata g_mt6701_angleI2C;   /* I2C 读到的 14 位角度（观测用）*/

extern uint16 xdata g_mt6701_mv;         /* 去极值均值电压 mV */
extern uint16 xdata g_mt6701_angle10;    /* 角度 0.1°（未滤波）*/
extern uint16 xdata g_mt6701_raw15;      /* 去极值均值后的 15 位码值 */
extern uint16 xdata g_mt6701_pp;         /* 本批峰峰值 mV（噪声观察）*/
extern uint8  xdata g_mt6701_anaSt;      /* bit0 满批 bit1 低于量程 bit2 高于量程 */

/* ---------------- I2C 侧（= 参考工程 mt6701.c）---------------- */
void  MT6701_Init(void);
uint8 MT6701_ReadReg(uint8 reg, uint8 *pDat);
uint8 MT6701_WriteReg(uint8 reg, uint8 dat);
uint8 MT6701_Probe(void);
void  MT6701_InitSequence(void);
void  MT6701_ReadAngleI2C(void);
void  MT6701_BootDump(void);

/* ---------------- 模拟量侧（= 参考工程 Inter_adc.c + Inter_zxmotor.c）---------------- */
void  MT6701_AnaInit(void);
uint8 MT6701_AnaPush(uint16 code15);
uint16 MT6701_GetMv(void);
uint16 MT6701_GetAngle10(void);
float MT6701_GetAngleF(void);
float MT6701_GetSpeedDps(uint16 dt_ms);
void  MT6701_SendObs(void);

#endif /* MT6701_H */
