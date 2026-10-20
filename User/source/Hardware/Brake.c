/**
 * @file      Brake.c
 * @brief     升降电机断电抱闸驱动实现（FU6866 新板）
 * @note      新增文件：放在 User\source\Hardware\ 下，并加入 Keil 工程文件列表。
 *            引脚 / 寄存器说明见 dir_brake.h 文件头。
 */

#include <MyProject.h>
#include <Brake.h>

#if (BRAKE_ENABLE != 0)

/* ============================== 内部常量 ============================== */

/* VREF 折算成 mV。HW_ADC_REF 来自 Parameter.h（本板 VREF4_5 → 4.5）→ 编译期常量 4500 */
#define BRAKE_VREF_MV       ((uint16)(HW_ADC_REF * 1000.0))

#define BRAKE_DET_NONE      (2)         /* 滞回区内，电平不定，维持上次判定 */
#define BRAKE_TICK_DIV      (10)        /* Brake_Update 每 10 次算一拍 = 10ms */

/* ============================== 内部状态 ============================== */

static uint8 data s_state = BRAKE_STATE_LOCKED;

#if (BRAKE_USE_DETECT != 0)
static uint8  data s_tick10  = 0;               /* 10ms 分频计数 */
static uint8  data s_detLvl  = BRAKE_DET_NONE;  /* 0=合闸电平 1=开闸电平 2=不定 */
static uint8  data s_detSame = 0;               /* 连续一致拍数 */
static uint16 data s_waitCnt = 0;               /* 确认等待拍数（10ms/拍）*/
#endif

/* ============================== 内部函数 ============================== */

#if (BRAKE_USE_DETECT != 0)
/**
 * @brief  检测电平判定（读 AD1 → 折算 mV → 带滞回判电平）
 * @return 1 = 开闸电压；0 = 合闸电压；2 = 滞回区内不定（维持上次判定）
 * @note   ★ FOC 没在跑的时候（MOE=0 / FOCEN=0），硬件不会自动扫通道，ADC1_DR 是旧值
 *           → 必须软件触发一次；
 *           FOC 在跑的时候【禁止】软件触发（会插进 FOC 的电流采样时序），只读寄存器。
 * @note   等待转换必须带超时：原工程那种 `while (ReadBit(ADC_CR, ADCBSY));` 无限等，
 *         一旦 ADC 配置错就会卡死 → 看门狗复位 → 表现成"周期性重启"，极难排查。
 */
static uint8 Brake_ReadDetLevel(void)
{
    uint16 mv;

    /*  改动：不再自己触发 ADC。
     *   上游 TickCycle_1ms() 现在每 1ms 无条件做一次顺序采样，ADC1_DR 会被一起刷新；
     *   而 main.c 的 1ms 块里顺序是 TickCycle_1ms() → Brake_Update()，
     *   所以这里读到的 ADC1_DR 最多 1ms 旧，够用（Brake_Update 内部 10ms 才跑一拍）。
     *   旧代码用 (MOE==0)||(FOCEN==0) 挡着：电机一转 ADC1_DR 就冻结在"开闸"那个值，
     *   mcBrake 里的"合闸确认"永远等不到 → 必超时 → BRAKE_STATE_FAULT → FaultBrake 锁死。*/

    /* ADCALIGN 已开：满量程 32768 对应 VREF。32767 × 4500 = 1.47e8 < 2^32，用 uint32 安全 */
    mv = (uint16)(((uint32)(ADC1_DR & 0x7FFF) * (uint32)BRAKE_VREF_MV) / 32768UL);

    if (mv >= BRAKE_ON_THRESH_MV)  { return 1; }
    if (mv <= BRAKE_OFF_THRESH_MV) { return 0; }
    return BRAKE_DET_NONE;
}
#endif /* BRAKE_USE_DETECT */

/* ============================== 对外接口 ============================== */

/**
 * @brief  初始化：P4.3 配成数字推挽输出并拉低（上电即合闸，安全默认）
 */
void Brake_Init(void)
{
    /*  P4_OE 是 SFR(0xE9)、GP43 是 sbit、P4_AN(0x4047) / P4_PU(0x4057) 是 xdata 寄存器，
     *   三个都要配，否则 P4.3 不是"数字推挽输出"而是"模拟输入 / 带上拉"。
     *   SetBit / ClrBit 是整字节读改写宏（FU68xx_6_Type.h:37-38），对 SFR 也有效。 */
    ClrBit(P4_AN , P43);        /* 关模拟功能 → 数字脚 */
    ClrBit(P4_PU , P43);        /* 不上拉 */
    SetBit(P4_OE , P43);        /* 输出使能 */

    GP43 = 0;                   /* 低电平 = 断电合闸（安全默认）*/

    s_state = BRAKE_STATE_LOCKED;
#if (BRAKE_USE_DETECT != 0)
    s_tick10  = 0;
    s_detLvl  = BRAKE_DET_NONE;
    s_detSame = 0;
    s_waitCnt = 0;
#endif
}

/**
 * @brief  请求开闸（幂等）
 */
void Brake_RequestRelease(void)
{
    if ((s_state == BRAKE_STATE_RELEASED) || (s_state == BRAKE_STATE_RELEASING))
    {
        return;                                 /* 已开闸 / 开闸中，不重复计时 */
    }

    GP43 = 1;                                   /* 通电开闸 */
#if (BRAKE_USE_DETECT != 0)
    s_state   = BRAKE_STATE_RELEASING;
    s_waitCnt = 0;
    s_detSame = 0;
#else
    s_state   = BRAKE_STATE_RELEASED;           /* 不检测：立即视为动作完成 */
#endif
}

/**
 * @brief  请求合闸（幂等）
 * @note   无参数、无局部变量 → 可在中断里安全调用（原版另开的 emergency 接口已合并到这里）
 */
void Brake_RequestLock(void)
{
    if ((s_state == BRAKE_STATE_LOCKED) || (s_state == BRAKE_STATE_LOCKING))
    {
        return;
    }

    GP43 = 0;                                   /* 断电合闸 */
#if (BRAKE_USE_DETECT != 0)
    s_state   = BRAKE_STATE_LOCKING;
    s_waitCnt = 0;
    s_detSame = 0;
#else
    s_state   = BRAKE_STATE_LOCKED;
#endif
}

/**
 * @brief  周期任务：每 1ms 调一次，内部按 10ms 分频
 */
void Brake_Update(void)
{
#if (BRAKE_USE_DETECT != 0)
    uint8 level;

    if (++s_tick10 < BRAKE_TICK_DIV) { return; }    /* 10ms 才跑一拍 */
    s_tick10 = 0;

    /* ---- 检测电平 + 去抖：连续 BRAKE_STABLE_CNT 拍一致才采信 ---- */
    level = Brake_ReadDetLevel();
    if (level != BRAKE_DET_NONE)
    {
        if (level == s_detLvl)
        {
            if (s_detSame < BRAKE_STABLE_CNT) { s_detSame++; }
        }
        else
        {
            s_detLvl  = level;
            s_detSame = 1;
        }
    }

    /* ---- 状态机 ---- */
    switch (s_state)
    {
        case BRAKE_STATE_RELEASING:
            if ((s_detLvl == 1) && (s_detSame >= BRAKE_STABLE_CNT))
            {
                s_state = BRAKE_STATE_RELEASED;     /* 确认打开，允许驱动电机 */
            }
            else if (s_waitCnt < BRAKE_RELEASE_TIMEOUT)
            {
                s_waitCnt++;
            }
            else
            {
                s_state = BRAKE_STATE_FAULT;        /* 开闸超时：禁止驱动电机 */
            }
            break;

        case BRAKE_STATE_LOCKING:
            if ((s_detLvl == 0) && (s_detSame >= BRAKE_STABLE_CNT))
            {
                s_state = BRAKE_STATE_LOCKED;       /* 确认合闸锁死 */
            }
            else if (s_waitCnt < BRAKE_LOCK_TIMEOUT)
            {
                s_waitCnt++;
            }
            else
            {
                s_state = BRAKE_STATE_FAULT;        /* 合闸超时：故障供诊断 */
            }
            break;

        case BRAKE_STATE_RELEASED:
            /* 运行中监测：检测电压意外跌到合闸电平 → 抱闸可能意外吸合/失效，
               回退到"等待确认"，由上层决定停机 */
            if ((s_detLvl == 0) && (s_detSame >= BRAKE_STABLE_CNT))
            {
                s_state   = BRAKE_STATE_RELEASING;
                s_waitCnt = 0;
            }
            break;

        case BRAKE_STATE_LOCKED:
        case BRAKE_STATE_FAULT:
        default:
            break;
    }
#else
    /* 不使用检测：状态由 Request 接口直接置位，此处无需处理 */
#endif /* BRAKE_USE_DETECT */
}

/* ============================== 状态查询 ============================== */

/**
 * @brief  当前是否已确认打开（电机驱动互锁条件）
 */
uint8 Brake_IsReleased(void)
{
    return (uint8)((s_state == BRAKE_STATE_RELEASED) ? 1 : 0);
}

/**
 * @brief  当前状态（BRAKE_STATE_xxx）
 */
Brake_State_t Brake_GetState(void)
{
    return s_state;
}

#else   /* ============ BRAKE_ENABLE == 0：整个模块编译成空实现（调试用）============ */
        /* P4.3 不初始化、不输出；互锁视为常开，电机可直接驱动 */

void          Brake_Init(void)           { }
void          Brake_Update(void)         { }
void          Brake_RequestRelease(void) { }
void          Brake_RequestLock(void)    { }

uint8         Brake_IsReleased(void)     { return 1; }
Brake_State_t Brake_GetState(void)       { return BRAKE_STATE_RELEASED; }

#endif /* BRAKE_ENABLE */