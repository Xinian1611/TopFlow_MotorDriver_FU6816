/**
 * @file      Brake.h
 * @brief     升降电机断电抱闸驱动（FU6866 新板）
 * @note      新增文件，放在 User\include\ 下。
 *
 *  ---- 硬件 ----
 *    控制：P4.3（原理图网络名 MAG_DRV）
 *          拉高 = 通电开闸（电磁铁吸合）；拉低 / 断电 = 弹簧合闸锁死
 *    检测：P2.3 = AD1（BZV_DET），测抱闸线圈通电电压判断实际开/合状态
 *          ★ 本板实测：合闸 det ≈ 0 mV，开闸 det ≈ 1085 mV
 *
 *  ---- 安全原则 ----
 *    ① 断电自动抱闸：P4.3 上电先给低
 *    ② 只有"确认开闸"后才允许驱动电机（互锁在 MotorControl.c 的 mcReady）
 *    ③ 停机顺序：电气减速停稳 → 合闸并确认 → 最后才关 MOE
 *       （先合闸再断输出，升降机构不会掉一下）
 *    ④ 故障时调 Brake_RequestLock() 立即断电合闸
 *
 *  ---- 寄存器依据 ----
 *    P4_OE   SFR   0xE9      P4 输出使能（非位寻址，用 SetBit/ClrBit 整字节读写）
 *    P4_AN   xdata 0x4047    P4 模拟使能
 *    P4_PU   xdata 0x4057    P4 上拉使能
 *    GP43    sbit  P4^3      P4.3 输出电平（掩码 P43 = 0x08）
 *    ADC1_DR xdata 0x0FDA    AD1 = P2.3 转换结果（只有低 15 位有效）
 *    ADC_CR  xdata 0x4039     ADCBSY(0x40) 是 rw1：写 1 启动一次转换
 */

#ifndef __DIR_BRAKE_H__
#define __DIR_BRAKE_H__

#include <FU68xx_6.h>

/* ---------------- 全局开关（调试用） ----------------
 * 1 = 抱闸功能全部生效（P4.3 控制 + 状态机互锁）
 * 0 = 全部注掉：接口编译成空实现，P4.3 不初始化不输出，
 *     Brake_IsReleased() 恒返回 1（互锁视为常开），电机可直接驱动 */
#define BRAKE_ENABLE            (1)

/* ---------------- 调试开关 ----------------
 * 1 = 用 P2.3/AD1 电压检测确认状态（正式使用，带互锁）
 * 0 = 不检测，拉高/拉低后立即视为动作完成（检测电路未就绪时用，无状态确认） */
#define BRAKE_USE_DETECT        (1)

/* ---------------- 检测阈值（mV） ----------------
 * ★ 必须落在"合闸 0 mV ~ 开闸 1085 mV"之间。
 *   ⚠️ 上一版 GD32 板的 2000 / 1000 在本板是错的：1085 会落进滞回带里，
 *      开闸永远确认不了，每次都会超时进 FAULT。 */
#define BRAKE_ON_THRESH_MV      (800)       /* 高于此值判"已开闸" */
#define BRAKE_OFF_THRESH_MV     (300)       /* 低于此值判"已合闸"（滞回 500mV）*/

/* ---------------- 去抖与超时（单位：10ms 拍）---------------- */
#define BRAKE_STABLE_CNT        (2)         /* 连续 N 拍一致才采信（= 20ms）*/
#define BRAKE_RELEASE_TIMEOUT   (50)        /* 开闸确认超时（×10ms = 500ms）*/
#define BRAKE_LOCK_TIMEOUT      (50)        /* 合闸确认超时（×10ms = 500ms）*/

/* ---------------- 抱闸状态 ---------------- */
typedef enum {
    BRAKE_STATE_LOCKED = 0,     /* 合闸锁死（默认安全状态）*/
    BRAKE_STATE_RELEASING,      /* 开闸过程中，等检测确认 */
    BRAKE_STATE_RELEASED,       /* 已确认打开，允许驱动电机 */
    BRAKE_STATE_LOCKING,        /* 合闸过程中，等检测确认 */
    BRAKE_STATE_FAULT           /* 确认超时（抱闸卡滞 / 检测电路异常）*/
} Brake_State_t;

/* ---------------- 对外接口（6 个，与原版 GD32 版一一对应）---------------- */

/* 初始化：P4.3 推挽输出并拉低（上电即合闸，断电抱闸安全默认）*/
void          Brake_Init(void);

/* 每 1ms 调一次（内部 10ms 分频）：读 AD1 → 去抖 → 状态切换 / 超时判故障 */
void          Brake_Update(void);

/* 请求开闸（电机运动前调用，内部幂等）。要等 Brake_IsReleased()==1 才可驱动 */
void          Brake_RequestRelease(void);

/* 请求合闸（停机 / 故障时调用，内部幂等）
 * ★ 无参数、无局部变量 → 中断里也能安全调用（原版另开了一个 emergency 接口，这里合并掉）*/
void          Brake_RequestLock(void);

/* 当前是否已确认打开（1 = 是，电机驱动互锁条件）*/
uint8         Brake_IsReleased(void);

/* 当前状态。"是否已合闸""是否故障"都由上层拿它比：
 *     Brake_GetState() == BRAKE_STATE_LOCKED   → 已合闸，可以关 MOE
 *     Brake_GetState() == BRAKE_STATE_FAULT    → 确认超时，报故障
 * 为避免接口膨胀，原来的 IsLocked / IsFault / GetDetMv / GetCtrlLevel 都并到这里。*/
Brake_State_t Brake_GetState(void);

#endif /* __DIR_BRAKE_H__ */