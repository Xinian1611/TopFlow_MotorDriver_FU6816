/**
 * @copyright (C) COPYRIGHT 2022 Fortiortech Shenzhen
 * @file      TSD.c
 * @author    Fortiortech  Appliction Team
 * @date      2022-07-13
 * @brief     This file contains TSD function used for Motor Control.
 */

#include <MyProject.h> 
#include <math.h> 

uint8 Micro_TJ[] = { 70, 71, 72, 74, 75, 76, 77, 79, 80, 81,
	                 82, 83, 84, 86, 87, 88, 89, 91, 92, 93,
	                 95, 96, 97, 99, 100,101,102,104,105,107,
	                 108,110,111,112,114,115,117,118,120,121,
	                 123,124,126,128,129,131,132,134,136,137,
	                 139,141,142,144,146,148,149,151,152,154,
	                 156,158,160
                   };

/**
 * @brief        TSD初始化函数
 * @param        None
 * @return       none
 * @date         2022-09-13
 * @version      1.0
 * @property     Public
*/
void TSD_Init(void)
{
    SetBit(TSD_CR, TSDEN);        //温度保护使能
    SetBit(TSD_CR, TSEN_EN);        //温度传感器使能
}

/**
 * @brief        TSD芯片内部温度
 * @param        None
 * @return       none
 * @date         2023-
}3-24
 * @version      1.0
 * @property     Public
*/
uint8 TSD_Gain(void)
{
    uint8 SetTJ_Value = 0;
	uint8 TJ_Value = TSEN_DR;
	
    SetTJ_Value = Micro_TJ[TJ_Value];
	
    return SetTJ_Value;
}

/* ==================== NTC 原值的 50 点截尾平均 ====================
 * 为什么存 8 位：ADC2_DR 是"左次高位对齐"（ADCALIGN=1 → 结果在 [14:3]），取高 8 位
 *   （>>7）存进 50 字节缓冲；30 点平均后分辨率约 1/256 满量程
 *   （4.5V 下 ≈17.6mV ≈ NTC 0.3℃），远优于测试文档 5% 的判据，
 *   同时省 XDATA（存 15 位要 100 字节，本工程只剩约 100 字节）。
 */
#define NTC_TRIM_POINTS     (50)
#define NTC_TRIM_DROP       (10)

static uint8  xdata s_ntcBuf[NTC_TRIM_POINTS];
static uint8  xdata s_ntcCnt  = 0;
static uint16 xdata s_ntcTrim = 0;

void NTC_TrimPush(uint8 v)
{
    uint8  i, j, t;
    uint16 sum;

    s_ntcBuf[s_ntcCnt++] = v;
    if (s_ntcCnt < NTC_TRIM_POINTS)
    {
        return;                                 /* 还没满 50 点 */
    }
    s_ntcCnt = 0;

    /* 升序插入排序（50 点最坏约 1225 次比较；每 50ms 才跑一次，开销可忽略）*/
    for (i = 1; i < NTC_TRIM_POINTS; i++)
    {
        t = s_ntcBuf[i];
        j = i;
        while ((j > 0) && (s_ntcBuf[j - 1] > t))
        {
            s_ntcBuf[j] = s_ntcBuf[j - 1];
            j--;
        }
        s_ntcBuf[j] = t;
    }

    /* 去掉最小的 10 个和最大的 10 个，取中间 30 个的平均 */
    sum = 0;
    for (i = NTC_TRIM_DROP; i < (NTC_TRIM_POINTS - NTC_TRIM_DROP); i++)
    {
        sum += s_ntcBuf[i];
    }
    s_ntcTrim = (uint16)(sum / (NTC_TRIM_POINTS - 2 * NTC_TRIM_DROP));   /* /30 */
}

uint16 NTC_TrimValue(void)
{
    return s_ntcTrim;
}

/* ==========================================================================
 *   新增：驱动模块温度换算（NTC 分压节点电压 mV → NTC 阻值 → B 公式温度 ℃）
 *  与旧 GD32 工程 Motor_MD() 逐行对应，函数体一个字符没改；
 *  只换掉两个和芯片相关的入口：
 *    ① 入参 ch（旧工程有 ZXWD_DET / ZDWD_DET 两路）→ 本板只有一路 NTC，接 P2.4 = AD2
 *    ② ADC_Filter_GetChannelValueF(ch)（旧工程 DMA 多点去极值均值）
 *       → FU6866 是硬件自动扫通道，用本工程已经在跑的滤波器 mcFocCtrl.NTCTempFlt
 *  注：分压网络 5V 供电而 ADC 参考 4.5V（旧工程 3.3V，本板 4.5V，同样有饱和限制）
 * ========================================================================== */

/**
 * @brief  驱动模块温度换算：NTC 分压节点电压(mV) → NTC 阻值 → B公式温度(°C)
 * @param  adc_filt_counts  NTC 通道的 ADC 滤波值（ADC counts）
 * @return 温度（℃）；读数不可信返回 MOTOR_MD_ERR
 * @note   旧工程这里收的是"已经换算好的 mV"，本板收的是 ADC counts，
 *         所以前面多一句码值→mV；系数直接用本工程的 Brake.c:60 同一套（VREF=4500mV / 32768）
 */
float Motor_MD(uint16 adc_filt_counts)
{
    float v_node_mv;
    float v_node;
    float r_ntc;
    float temperature;

    adc_filt_counts &= 0x7FFF;                      /* 15bit 有效，最高位是状态位 */

    /* ---- 码值 → 节点电压 mV（等价于旧工程的 ADC_Filter_GetChannelValueF(ch)）---- */
    v_node_mv = ((float)((uint32)adc_filt_counts * 4500UL)) / 32768.0f;

    /* ---- 饱和/开路判据，对应旧工程注释里那句"低温时节点电压饱和，读数失效" ----
     *  本板 VREF=4.5V，所以饱和点比旧板（3.3V）更靠上：
     *    低于 0.05V → 上拉断路/通道没扫到
     *    高于 4.30V → 节点已经顶到参考，r_ntc 反解出来是假的
     *  旧工程没写返回什么，这里统一返回 MOTOR_MD_ERR，由调用者忽略 */
    if (v_node_mv < 50.0f)   { return MOTOR_MD_ERR; }
    if (v_node_mv > 4300.0f) { return MOTOR_MD_ERR; }

    /* ↓↓↓ 以下 3 行与旧工程完全一致，只是 v_node 由上面算出来 ↓↓↓ */
    v_node = v_node_mv / 1000.0f;                                   /* mV→V */
    r_ntc  = 10000.0f * v_node / (5.0f - v_node);                   /* 分压反解 NTC 阻值，Ω */
    // temperature = 1.0f / (1.0f / 298.15f + logf(r_ntc / 10000.0f) / 3450.0f) - 273.15f;
    temperature = 1.0f / (1.0f / 298.15f + (float)(log((double)(r_ntc / 10000.0f)) / 3450.0)) - 273.15f;

    return temperature;
}

/**
 * @brief  20ms 节拍调用：读 NTC → Motor_MD() → 存进 mcFocCtrl.NTCTempNum（0.1℃）
 * @note   float / log() 不进 1ms 任务，只在 main 大循环里跑
 */
void Motor_MD_Sample(void)
{
    float temp;

    /* NTC 电压的低通已经在 AddFunction.c:852（TickCycle_1ms）里做过了，这里直接用 */
    temp = Motor_MD((uint16)mcFocCtrl.NTCTempFlt);

    if (temp != MOTOR_MD_ERR)
    {
        /* ℃ → 0.1℃，四舍五入 */
        mcFocCtrl.NTCTempNum = (int16)(temp * 10.0f + ((temp >= 0.0f) ? 0.5f : -0.5f));
    }
    /* 读数不可信时保持上一次的值不动，不刷新 NTCTempNum */
}
