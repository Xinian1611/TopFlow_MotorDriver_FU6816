/**
 * @file    MT6701.c
 * @brief   MT6701 磁编码器：I2C 负责配置/标定，位置由 AD3 读模拟输出 VOUT 得到
 * @note    对应关系（本工程 ↔ D:\NEW\TopFlow_PowerBoard）：
 *   MT6701_Init/ReadReg/WriteReg/InitSequence/ReadAngleI2C
 *        ↔ Interface\MT6701\mt6701.c 的同名函数（软件 I2C 换成 FU6866 硬件 I2C）
 *   MT6701_AnaPush（50 点去极值均值）
 *        ↔ Interface\ADC\Inter_adc.c: ADC_Filter_GetChannelValue()
 *   MT6701_GetMv/GetAngle10/GetAngleF/GetSpeedDps
 *        ↔ Interface\Motor\Inter_zxmotor.c: MT6701_GetAngleRaw/GetAngle/GetSpeed()
 *   MT6701_Probe/BootDump/SendObs
 *        ↔ 原工程用 Keil Watch 看 mt6701_read_log/write_log 与
 *          app_motor.c 的 g_dbg.steer.mt6701_voltage_mv，本工程改发 CAN 报文
 */

#include <MyProject.h>
#include "MT6701.h"

/* ==================== 全局数据 ==================== */
uint8  xdata g_mt6701_probe  = 0xFF;
uint8  xdata g_mt6701_err    = 0;
uint16 xdata g_mt6701_errCnt = 0;
uint8  xdata g_mt6701_boot[12];
uint16 xdata g_mt6701_zero     = 0;
uint16 xdata g_mt6701_angleI2C = 0;

uint16 xdata g_mt6701_mv      = 0;
uint16 xdata g_mt6701_angle10 = 0;
uint16 xdata g_mt6701_raw15   = 0;
uint16 xdata g_mt6701_pp      = 0;
uint8  xdata g_mt6701_anaSt   = 0;

static uint16 xdata s_anaBuf[MT6701_ANA_POINTS];
static uint8  xdata s_anaCnt = 0;

/* ============================================================================
 *  第一部分：I2C 传输层
 *  = 参考工程 Driver\soft_iic\soft_iic.c（本工程 FU6866 用硬件 I2C，宏可切软件）
 * ============================================================================ */
#if (MT6701_USE_HW_I2C)
/* ---------------- 硬件 I2C（FU6816/FU6866 手册第 29 章 78~84 页）----------------
 * ① I2C_CR[I2CEN] 使能后引脚自动变集电极开漏，上拉由端口上拉设置决定 → 不配 P0_OE
 * ② P0.5/P0.6 = I²C 功能转移后的 SDA/SCL（引脚表）→ PH_SEL2[I2C_FS]=1 必须置
 *    置 0 会挂到 P0.0/P0.1 = CAN 的 TX/RX，调试通道一起死
 * ③ 主机模式：I2C_ID=从机地址、DMOD=读写方向、I2CSTA=发 START/RESTART+地址、I2CSTP=发 STOP
 * ④ STR=动作完成且 SCL 被硬件拉低；清 STR 才释放 SCL 并发出下一个动作
 *    (I2CSTA,I2CSTP)：00=数据字节 01=STOP 10=START+地址字节
 * ⑤ NACK：写模式=读(0 收到 ACK / 1 收到 NACK)；读模式=配置(0 发 ACK / 1 发 NACK)
 */
#define MT_I2C_TO   3000

static uint8 xdata g_mt_srSnap = 0, g_mt_crSnap = 0;   /* 超时瞬间寄存器快照（排查用）*/

static void mt_dly_short(void)                    /* 总线恢复用的短延时，示波器调这个循环 */
{
    uint8 i;
    for (i = 0; i < 6; i++) { _nop_(); }
}

/* 等一个动作完成（不清 STR）；超时返回 3 并抓快照 */
static uint8 mt_wait_str(void)                    /* ↔ soft_iic.c 的时序等待 */
{
    uint16 t = MT_I2C_TO;
    while (!ReadBit(I2C_SR, STR))
    {
        if (--t == 0) { g_mt_srSnap = I2C_SR; g_mt_crSnap = I2C_CR; return 3; }
    }
    return 0;
}

/* 等总线空闲（手册：主机发 STOP 成功后硬件清 I2CBSY）*/
static uint8 mt_wait_idle(void)
{
    uint16 t = MT_I2C_TO;
    while (ReadBit(I2C_SR, I2CBSY)) { if (--t == 0) { return 3; } }
    return 0;
}

/* 每笔事务开头清掉上一笔残留的动作标志（(I2CSTA,I2CSTP) 必须回到 00）*/
static void mt_flags_clr(void)
{
    ClrBit(I2C_SR, I2CSTA);
    ClrBit(I2C_SR, I2CSTP);
    ClrBit(I2C_SR, NACK);
}

/* 收尾 STOP：STR 置位时置 I2CSTP，清 STR 后发停止信号，再等 I2CBSY 清零并把标志清回 0 */
static void mt_stop(void)
{
    ClrBit(I2C_SR, I2CSTA);
    SetBit(I2C_SR, I2CSTP);
    ClrBit(I2C_SR, STR);
    mt_wait_idle();
    ClrBit(I2C_SR, I2CSTP);
}

/* 总线卡死恢复：外设让出引脚 → 9 个 SCL 脉冲 → 补 STOP → 重新初始化外设 */
static void mt_bus_free(void)
{
    uint8 k;

    ClrBit(I2C_CR, I2CEN);
    SetBit(P0_PU, P05);
    SetBit(P0_PU, P06);
    ClrBit(P0_OE, P05);
    ClrBit(P0_OE, P06);
    for (k = 0; k < 9; k++)
    {
        SetBit(P0_OE, P06); GP06 = 0; mt_dly_short();
        ClrBit(P0_OE, P06);           mt_dly_short();
    }
    SetBit(P0_OE, P05); GP05 = 0; mt_dly_short();
    SetBit(P0_OE, P06);           mt_dly_short();
    ClrBit(P0_OE, P05);           mt_dly_short();

    I2C_CR = 0;
    SetBit(I2C_CR, I2CMS);
#if (MT6701_SPEED_400K)
    ClrBit(I2C_CR, I2CSPD1); SetBit(I2C_CR, I2CSPD0);
#else
    ClrBit(I2C_CR, I2CSPD1); ClrBit(I2C_CR, I2CSPD0);
#endif
    ClrBit(I2C_CR, I2CIE);
    I2C_ID = (uint8)(MT6701_ADDR << 1);
    SetBit(I2C_CR, I2CEN);
}

/* 主-写：buf[0]=寄存器地址 buf[1]=数据。0=OK 1=地址没应答 2=数据没应答 3=超时 */
static uint8 mt_i2c_write(uint8 dev, const uint8 *buf, uint8 n)
{
    uint8 i, r;

    if (mt_wait_idle() != 0) { return 3; }
    mt_flags_clr();

    I2C_ID = (uint8)(dev << 1);
    ClrBit(I2C_SR, DMOD);                     /* 写方向 */
    SetBit(I2C_SR, I2CSTA);                   /* START + 地址字节（硬件自动发）*/
    if ((r = mt_wait_str()) != 0) { mt_bus_free(); return r; }
    if (ReadBit(I2C_SR, NACK)) { mt_stop(); return 1; }

    for (i = 0; i < n; i++)
    {
        I2C_DR = buf[i];
        ClrBit(I2C_SR, STR);
        if ((r = mt_wait_str()) != 0) { mt_bus_free(); return r; }
        if (ReadBit(I2C_SR, NACK)) { mt_stop(); return 2; }
    }

    mt_stop();
    return 0;
}

/* 主-读：写寄存器地址 → RESTART + 地址(R) → 读 n 字节 */
/* 主-读：写寄存器地址 → RESTART + 地址(R) → 读 n 字节
 * 相位规则（手册）：一次"清 STR"执行一次已配置的动作；
 *   (I2CSTA,I2CSTP)：00=数据字节  01=STOP  10=START+地址字节
 *   STR=1 = 上一动作完成且 SCL 被硬件拉低，清 STR 才释放并执行下一动作 */
static uint8 mt_i2c_read(uint8 dev, uint8 reg, uint8 *buf, uint8 n)
{
    uint8 i, r;

    if (mt_wait_idle() != 0) { return 3; }
    mt_flags_clr();

    /* --- 相位1：START + 地址(W)（总线空闲时，写 I2CSTA 即发出）--- */
    I2C_ID = (uint8)(dev << 1);               /* 0x0C；读写位由 DMOD 决定 */
    ClrBit(I2C_SR, DMOD);                     /* 写方向 */
    SetBit(I2C_SR, I2CSTA);                   /* 配置：START + 地址字节 */
    if ((r = mt_wait_str()) != 0) { mt_bus_free(); return r; }
    if (ReadBit(I2C_SR, NACK)) { mt_stop(); return 1; }

    /* --- 相位2：寄存器地址 --- */
    I2C_DR = reg;                             /* 配置数据字节 */
    ClrBit(I2C_SR, STR);                      /* 执行 */
    if ((r = mt_wait_str()) != 0) { mt_bus_free(); return r; }
    if (ReadBit(I2C_SR, NACK)) { mt_stop(); return 2; }

    /* --- 相位3：RESTART + 地址(R) --- */
    SetBit(I2C_SR, DMOD);                     /* 配置：读方向 */
    SetBit(I2C_SR, I2CSTA);                   /* 配置：RESTART + 地址字节 */
    ClrBit(I2C_SR, STR);                      /*  补这一句：执行 RESTART（原来缺它）*/
    if ((r = mt_wait_str()) != 0) { mt_bus_free(); return r; }   /* 地址相位完成，SCL 被拉低 */

    /* --- 相位4：读 n 字节（每字节一次清 STR，收尾那一次带 NACK+STOP）--- */
    for (i = 0; i < n; i++)
    {
        if (i == 0) { ClrBit(I2C_SR, STR); }  /* 执行：开始接收第 1 字节 */
        if ((r = mt_wait_str()) != 0) { mt_bus_free(); return r; }
        buf[i] = I2C_DR;                      /* 现在读到的才是从机数据 */

        if (i == (uint8)(n - 1))
        {
            SetBit(I2C_SR, NACK);             /* 最后一字节：第9位回 NACK */
            SetBit(I2C_SR, I2CSTP);           /* 同一次清 STR 里带上 STOP（表：01=STOP）*/
        }
        else
        {
            ClrBit(I2C_SR, NACK);             /* 还要继续：第9位回 ACK */
        }
        ClrBit(I2C_SR, STR);                  /* 执行：发第9位；最后一字节时随后发 STOP */
    }

    mt_wait_idle();                           /* 等 STOP 发完（I2CBSY 清 0）*/
    ClrBit(I2C_SR, I2CSTP);                   /* 清掉 STOP 标志，别留给下一笔 */
    return 0;
}

#else
/* ---------------- 软件 I2C（兜底）：= 参考工程 soft_iic.c 的时序，引脚换成本板 ---------------- */
#define MT_SCL_H()  do { ClrBit(P0_OE, P06); } while(0)          /* 高阻+上拉=高 */
#define MT_SCL_L()  do { SetBit(P0_OE, P06); GP06 = 0; } while(0)
#define MT_SDA_H()  do { ClrBit(P0_OE, P05); } while(0)
#define MT_SDA_L()  do { SetBit(P0_OE, P05); GP05 = 0; } while(0)
#define MT_SDA_RD() ((P0 & P05) != 0)
#define MT_DLY()    do { uint8 _i; for (_i = 0; _i < 6; _i++) { _nop_(); } } while(0)

static void mt_start(void) { MT_SDA_H(); MT_SCL_H(); MT_DLY(); MT_SDA_L(); MT_DLY(); MT_SCL_L(); MT_DLY(); }
static void mt_stop (void) { MT_SDA_L(); MT_SCL_H(); MT_DLY(); MT_SDA_H(); MT_DLY(); }

static void mt_wbyte(uint8 b)
{
    uint8 i;
    for (i = 0; i < 8; i++)
    {
        MT_SCL_L();
        if (b & 0x80) { MT_SDA_H(); } else { MT_SDA_L(); }
        MT_DLY(); MT_SCL_H(); MT_DLY(); MT_SCL_L(); MT_DLY();
        b <<= 1;
    }
}

static uint8 mt_rbyte(void)
{
    uint8 i, b = 0;
    MT_SDA_H();
    for (i = 0; i < 8; i++)
    {
        MT_SCL_L(); MT_DLY(); MT_SCL_H(); MT_DLY();
        b <<= 1;
        if (MT_SDA_RD()) b |= 0x01;
        MT_SCL_L(); MT_DLY();
    }
    return b;
}

static uint8 mt_rack(void)      /* 0=从机应答 1=无应答 */
{
    uint8 a;
    MT_SDA_H(); MT_SCL_L(); MT_DLY(); MT_SCL_H(); MT_DLY();
    a = MT_SDA_RD() ? 1 : 0;
    MT_SCL_L(); MT_DLY();
    return a;
}

static void mt_ack (void) { MT_SDA_L(); MT_SCL_L(); MT_DLY(); MT_SCL_H(); MT_DLY(); MT_SCL_L(); MT_DLY(); MT_SDA_H(); }
static void mt_nack(void) { MT_SDA_H(); MT_SCL_L(); MT_DLY(); MT_SCL_H(); MT_DLY(); MT_SCL_L(); MT_DLY(); }

static uint8 mt_i2c_write(uint8 dev, const uint8 *buf, uint8 n)
{
    uint8 i;
    mt_start();
    mt_wbyte((uint8)((dev << 1) | 0));
    if (mt_rack()) { mt_stop(); return 1; }
    for (i = 0; i < n; i++)
    {
        mt_wbyte(buf[i]);
        if (mt_rack()) { mt_stop(); return 2; }
    }
    mt_stop();
    return 0;
}

static uint8 mt_i2c_read(uint8 dev, uint8 reg, uint8 *buf, uint8 n)
{
    uint8 i;
    mt_start();
    mt_wbyte((uint8)((dev << 1) | 0));
    if (mt_rack()) { mt_stop(); return 1; }
    mt_wbyte(reg);
    if (mt_rack()) { mt_stop(); return 2; }

    mt_start();                                  /* 重复起始 */
    mt_wbyte((uint8)((dev << 1) | 1));
    if (mt_rack()) { mt_stop(); return 1; }
    for (i = 0; i < n; i++)
    {
        buf[i] = mt_rbyte();
        if (i == (uint8)(n - 1)) { mt_nack(); } else { mt_ack(); }
    }
    mt_stop();
    return 0;
}
#endif

/* ============================================================================
 *  第二部分：I2C 器件层
 * ============================================================================ */

/**
 * @brief  I2C 引脚与外设初始化
 * @note   ↔ soft_iic.c: SoftI2C_Init() + mt6701.c: MT6701_Init()
 *         P0.5/P0.6 同时是 UART1/SPI调试/CAN转移脚，所以 UART1 必须关、I2C_FS 必须置
 */
void MT6701_Init(void)
{
#if (MT6701_USE_HW_I2C)
    ClrBit(PH_SEL , UART1EN);       /* P0.5/P0.6 上的 UART1 关掉（同脚、同中断13）*/
    SetBit(PH_SEL2, I2C_FS);        /*  I2C 功能转移到 P0.5(SDA)/P0.6(SCL)，恒置 1 */
    SetBit(P0_PU, P05);             /* 上拉由端口上拉设置决定（板上有 4.7k 更好）*/
    SetBit(P0_PU, P06);

    I2C_CR = 0;
    SetBit(I2C_CR, I2CMS);                                  /* 主机模式 */
#if (MT6701_SPEED_400K)
    ClrBit(I2C_CR, I2CSPD1); SetBit(I2C_CR, I2CSPD0);       /* 400kHz */
#else
    ClrBit(I2C_CR, I2CSPD1); ClrBit(I2C_CR, I2CSPD0);       /* 100kHz */
#endif
    ClrBit(I2C_CR, I2CIE);                                  /* 轮询，不用中断 */
    I2C_ID = (uint8)(MT6701_ADDR << 1);                     /* 从机地址 7位<<1 = 0x0C */
    SetBit(I2C_CR, I2CEN);
#else
    ClrBit(PH_SEL , UART1EN);
    ClrBit(PH_SEL2, I2C_FS);        /* 软件 I2C 不用硬件外设，保持 0 即可 */
    SetBit(P0_PU, P05);
    SetBit(P0_PU, P06);
    MT_SCL_H();
    MT_SDA_H();
#endif
}

/**
 * @brief  向指定寄存器写一个字节
 * @note   ↔ mt6701.c: MT6701_WriteReg()；返回 0=成功，非 0=失败码
 */
uint8 MT6701_WriteReg(uint8 reg, uint8 dat)
{
    uint8 b[2];
    b[0] = reg;
    b[1] = dat;
    g_mt6701_err = mt_i2c_write(MT6701_ADDR, b, 2);
    if (g_mt6701_err != 0) { g_mt6701_errCnt++; }
    return g_mt6701_err;
}

/**
 * @brief  读指定寄存器一个字节
 * @note   ↔ mt6701.c: MT6701_ReadReg()；返回 0=成功，非 0=失败码
 */
uint8 MT6701_ReadReg(uint8 reg, uint8 *pDat)
{
    g_mt6701_err = mt_i2c_read(MT6701_ADDR, reg, pDat, 1);
    if (g_mt6701_err != 0) { g_mt6701_errCnt++; }
    return g_mt6701_err;
}

/**
 * @brief  I2C 在线自检（连读 4 次 0x03）
 * @note   原工程靠 Watch 看 read/write_log 判断；本工程把结果打进 g_mt6701_probe：
 *         0=从机应答(在线)  1=地址没应答  2=数据没应答  3=超时
 */
uint8 MT6701_Probe(void)
{
    uint8 d = 0, r, i, pk = 0;

    for (i = 0; i < 4; i++)
    {
        r = MT6701_ReadReg(MT6701_REG_ANGLE_H, &d);
        pk = (uint8)((pk << 2) | (r & 0x03));
    }
    g_mt6701_probe = (uint8)(pk & 0x03);      /* 最后一次的返回码 */
    g_mt6701_boot[11] = pk;                   /* 4 次返回码打包，随 0x1F3 发出 */
    return g_mt6701_probe;
}

/**
 * @brief  I2C 读 14 位角度（仅观测；位置不用它）
 * @note   ↔ mt6701.c: MT6701_Reangle()
 */
void MT6701_ReadAngleI2C(void)
{
    uint8 h = 0, l = 0;

    if (MT6701_ReadReg(MT6701_REG_ANGLE_H, &h) != 0) { return; }
    if (MT6701_ReadReg(MT6701_REG_ANGLE_L, &l) != 0) { return; }
    g_mt6701_angleI2C = ((uint16)h << 6) | ((uint16)(l & 0xFC) >> 2);
}

/* 初始化序列里所有写操作统一记失败：任一次失败 → boot[0].bit7 */
#define MT_WR(reg, val) \
    do { if (MT6701_WriteReg((reg), (val)) != 0) { g_mt6701_boot[0] |= 0x80; } } while (0)

/**
 * @brief  MT6701 配置/标定序列（写寄存器 + 写零点 + 烧 EEPROM）
 * @note   ↔ mt6701.c: MT6701_InitSequence()（逐条一致，去掉 printf/Watch 日志）
 *         ★ 只在"标定"时调用！里面有 EEPROM 烧录，每次上电烧会缩短芯片 EEPROM 寿命
 *         boot[0] 状态位图：bit0=0x29读失败 bit1=0x32读失败 bit2=0x34读失败
 *                          bit3=角度不稳(零点未写) bit4=零点回读失败 bit5=0x38读失败
 *                          bit6=0x28读失败 bit7=任一写失败
 *         boot[1..9] = 0x29/0x32/0x33/0x34/0x38/0x3E/0x3F/0x40/0x28 回读值
 *         boot[10]=最后错误码  boot[11]=自检 4 次返回码打包
 */
void MT6701_InitSequence(void)
{
    uint8  tmp = 0, zl = 0, reg32 = 0;
    uint16 zero = 0;

    memset(g_mt6701_boot, 0xFF, sizeof(g_mt6701_boot));
    g_mt6701_boot[0] = 0x00;

    /* 2.1 读 0x29，bit1 按 MT6701_ANGLE_DIR 写回 */
    if (MT6701_ReadReg(MT6701_REG_29, &tmp) == 0)
    {
        g_mt6701_boot[1] = tmp;
        if (MT6701_ANGLE_DIR == 0) { tmp &= (uint8)~(1 << 1); }
        else                       { tmp |= (uint8)(1 << 1); }
        MT_WR(MT6701_REG_29, tmp);
    }
    else { g_mt6701_boot[1] = 0xFF; g_mt6701_boot[0] |= 0x01; }

    /* 2.2 读 0x32，bit7 清 0 写回 */
    if (MT6701_ReadReg(MT6701_REG_ZERO_H, &tmp) == 0)
    {
        g_mt6701_boot[2] = tmp;
        tmp &= (uint8)~(1 << 7);
        MT_WR(MT6701_REG_ZERO_H, tmp);
    }
    else { g_mt6701_boot[2] = 0xFF; g_mt6701_boot[0] |= 0x02; }

    /* 2.3 读 0x34，bit7 置 1、bit6 清 0 写回 */
    if (MT6701_ReadReg(MT6701_REG_34, &tmp) == 0)
    {
        g_mt6701_boot[4] = tmp;
        tmp = (uint8)((tmp & ~(1 << 6)) | (1 << 7));
        MT_WR(MT6701_REG_34, tmp);
    }
    else { g_mt6701_boot[4] = 0xFF; g_mt6701_boot[0] |= 0x04; }

    /* 2.4 双读校验当前角度 → 累加旧零点写 0x32/0x33（当前位置 = 0°）*/
    {
        uint16 angle1 = 0, angle2 = 0, diff;
        uint8  valid = 0, attempt;
        uint8  h1, l1, h2, l2;

        for (attempt = 0; attempt < 5 && (valid == 0); attempt++)
        {
            h1 = 0; l1 = 0; h2 = 0; l2 = 0;
            if ((MT6701_ReadReg(MT6701_REG_ANGLE_H, &h1) == 0) &&
                (MT6701_ReadReg(MT6701_REG_ANGLE_L, &l1) == 0) &&
                (MT6701_ReadReg(MT6701_REG_ANGLE_H, &h2) == 0) &&
                (MT6701_ReadReg(MT6701_REG_ANGLE_L, &l2) == 0))
            {
                angle1 = ((uint16)h1 << 6) | ((uint16)(l1 & 0xFC) >> 2);
                angle2 = ((uint16)h2 << 6) | ((uint16)(l2 & 0xFC) >> 2);
                diff = (angle1 > angle2) ? (uint16)(angle1 - angle2) : (uint16)(angle2 - angle1);
                if (diff > 8192U) { diff = (uint16)(16384U - diff); }
                if (diff <= 4U)   { valid = 1; }
            }
        }

        if (valid)
        {
            if ((MT6701_ReadReg(MT6701_REG_ZERO_H, &reg32) == 0) &&
                (MT6701_ReadReg(MT6701_REG_ZERO_L, &zl) == 0))
            {
                uint16 old_zero = ((uint16)(reg32 & 0x0F) << 8) | zl;

                zero = (uint16)((old_zero + (angle1 >> 2)) & 0x0FFF);   /* 累加旧零点，12 位模 */
                g_mt6701_zero  = zero;
                g_mt6701_boot[3] = zl;

                reg32 = (uint8)((reg32 & 0xF0) | ((zero >> 8) & 0x0F));
                MT_WR(MT6701_REG_ZERO_H, reg32);
                MT_WR(MT6701_REG_ZERO_L, (uint8)(zero & 0xFF));
            }
            else { g_mt6701_boot[0] |= 0x10; }
        }
        else { g_mt6701_boot[0] |= 0x08; }        /* 角度在跳 → 拒绝写零点 */
    }

    /* 2.5 读 0x38，bit5 清 0 写回 */
    if (MT6701_ReadReg(MT6701_REG_38, &tmp) == 0)
    {
        g_mt6701_boot[5] = tmp;
        tmp &= (uint8)~(1 << 5);
        MT_WR(MT6701_REG_38, tmp);
    }
    else { g_mt6701_boot[5] = 0xFF; g_mt6701_boot[0] |= 0x20; }

    /* 2.6/2.7/2.8 写 0x3E=0xF0、0x3F=0x00、0x40=0xFF 后回读 */
    MT_WR(MT6701_REG_3E, 0xF0);
    MT_WR(MT6701_REG_3F, 0x00);
    MT_WR(MT6701_REG_40, 0xFF);
    if (MT6701_ReadReg(MT6701_REG_3E, &tmp) == 0) { g_mt6701_boot[6] = tmp; }
    if (MT6701_ReadReg(MT6701_REG_3F, &tmp) == 0) { g_mt6701_boot[7] = tmp; }
    if (MT6701_ReadReg(MT6701_REG_40, &tmp) == 0) { g_mt6701_boot[8] = tmp; }

    /* 2.9 读 0x28，bit5 置 1 写回 */
    if (MT6701_ReadReg(MT6701_REG_28, &tmp) == 0)
    {
        g_mt6701_boot[9] = tmp;
        tmp |= (uint8)(1 << 5);
        MT_WR(MT6701_REG_28, tmp);
    }
    else { g_mt6701_boot[9] = 0xFF; g_mt6701_boot[0] |= 0x40; }

#if (MT6701_BURN_EEPROM)
    /* 只有"全序列无失败"才烧 EEPROM（否则会把错误零点烧进芯片，需重新标定覆盖）
     * ↔ 参考工程 mt6701.c 的 0x09=0xB3 / 0x0A=0x05 两句 */
    if (g_mt6701_boot[0] == 0x00)
    {
        MT_WR(MT6701_REG_EE_1, 0xB3);
        MT_WR(MT6701_REG_EE_2, 0x05);
    }
#endif

    MT6701_ReadAngleI2C();                       /* 顺带读一次 I2C 角度做观测 */
    g_mt6701_boot[10] = g_mt6701_err;
}

/**
 * @brief  把 I2C 自检/标定结果一次性发 CAN（替代原工程的 Keil Watch 观察）
 * @note   0x1F0: [0]状态位图 [1]自检码 [2..3]I2C角度 [4]0x29 [5]0x32 [6]0x33 [7]0x34
 *         0x1F1: [0]0x38 [1]0x3E [2]0x3F [3]0x40 [4]0x28 [5..6]零点 [7]最后错误码
 *         0x1F3: [0]自检4次返回码 [1]超时快照I2C_SR [2]I2C_CR [3]I2C_ID [4..5]失败计数
 */
void MT6701_BootDump(void)
{
    uint8 f[8];

    f[0] = g_mt6701_boot[0];
    f[1] = g_mt6701_probe;
    f[2] = (uint8)(g_mt6701_angleI2C >> 8);
    f[3] = (uint8)g_mt6701_angleI2C;
    f[4] = g_mt6701_boot[1];
    f[5] = g_mt6701_boot[2];
    f[6] = g_mt6701_boot[3];
    f[7] = g_mt6701_boot[4];
    can_hw_send(0x1F0, f, 8);

    f[0] = g_mt6701_boot[5];
    f[1] = g_mt6701_boot[6];
    f[2] = g_mt6701_boot[7];
    f[3] = g_mt6701_boot[8];
    f[4] = g_mt6701_boot[9];
    f[5] = (uint8)(g_mt6701_zero >> 8);
    f[6] = (uint8)g_mt6701_zero;
    f[7] = g_mt6701_boot[10];
    can_hw_send(0x1F1, f, 8);

    f[0] = g_mt6701_boot[11];
    f[1] = g_mt_srSnap;    /* 硬件 I2C 分支才有意义；软件分支恒 0 */
    f[2] = g_mt_crSnap;
    f[3] = I2C_ID;
    f[4] = (uint8)(g_mt6701_errCnt >> 8);
    f[5] = (uint8)g_mt6701_errCnt;
    f[6] = 0xFF;
    f[7] = 0xFF;
    can_hw_send(0x1F3, f, 8);
}

/* ============================================================================
 *  第三部分：模拟量（位置来源）
 *  P2.5 = AD3 读 MT6701 VOUT；ADCALIGN=1 → 15 位量纲在 ADC3_DR[14:0]
 * ============================================================================ */

/**
 * @brief  模拟量通道自检/使能（幂等）
 * @note   ↔ 参考工程 dir_adc.c 的通道配置；本板 ADC.c 里也已配（这里再断言一次防漏）
 */
void MT6701_AnaInit(void)
{
    SetBit(P2_AN, PIN5);                     /* P2.5 模拟输入（AD3）*/
    ClrBit(P2_OE, PIN5);
    SetBit(ADC_MASK, CH3EN);                 /* AD3 通道使能 */
}

/**
 * @brief  逐点送入一次 AD3 采样；返回 1 = 本项目的一批（MT6701_ANA_POINTS 点）算完了
 * @note   ↔ Inter_adc.c: ADC_Filter_GetChannelValue() 的去极值均值算法
 *         （50 点升序排序，丢最大 10 / 最小 10，取中间 30 平均）
 *         本板 1ms 一点 → 50 点 = 50ms 窗口（参考工程 DMA 50 轮 ≈ 1.9ms）
 *         嫌慢就改 MT6701_ANA_POINTS（如 20 = 20ms，DRPO 相应减到 4）
 */
uint8 MT6701_AnaPush(uint16 code15)
{
    uint8  i, j;
    uint16 t;
    uint32 sum;

    s_anaBuf[s_anaCnt++] = code15;
    if (s_anaCnt < MT6701_ANA_POINTS)
    {
        return 0;
    }
    s_anaCnt = 0;

    /* 升序插入排序（本工程 TSD.c 的 NTC 滤波器同写法）*/
    for (i = 1; i < MT6701_ANA_POINTS; i++)
    {
        t = s_anaBuf[i];
        j = i;
        while ((j > 0) && (s_anaBuf[j - 1] > t))
        {
            s_anaBuf[j] = s_anaBuf[j - 1];
            j--;
        }
        s_anaBuf[j] = t;
    }

    /* 峰值 → mV*/
    g_mt6701_pp = (uint16)(((uint32)(s_anaBuf[MT6701_ANA_POINTS - 1] - s_anaBuf[0]) * MT6701_ANA_VREF_MV) / 32768UL);

    /* 去极值均值（30 点求和）*/
    sum = 0;
    for (i = MT6701_ANA_DROP; i < (MT6701_ANA_POINTS - MT6701_ANA_DROP); i++)
    {
        sum += s_anaBuf[i];
    }
    g_mt6701_raw15 = (uint16)(sum / (MT6701_ANA_POINTS - 2U * MT6701_ANA_DROP));

    /* 码值 → mV：本板 15 位 / 4.5V（参考工程是 12 位 / 3.3V）*/
    g_mt6701_mv = (uint16)(((uint32)g_mt6701_raw15 * MT6701_ANA_VREF_MV) / 32768UL);

    /* mV → 角度（0.1°）：(mv-200)*3600/1600，两端限幅并置量程标志
     * ↔ Inter_zxmotor.c: MT6701_GetAngleRaw() 的 (mv-200)*360/1600 */
    g_mt6701_anaSt = 0x01;
    if (g_mt6701_mv < MT6701_ANA_OFFSET_MV)
    {
        g_mt6701_angle10 = 0;
        g_mt6701_anaSt |= 0x02;                        /* 低于量程 */
    }
    else if (g_mt6701_mv > (uint16)(MT6701_ANA_OFFSET_MV + MT6701_ANA_SPAN_MV))
    {
        g_mt6701_angle10 = 3599;
        g_mt6701_anaSt |= 0x04;                        /* 高于量程 */
    }
    else
    {
        g_mt6701_angle10 = (uint16)(((uint32)(g_mt6701_mv - MT6701_ANA_OFFSET_MV) * 3600UL) / MT6701_ANA_SPAN_MV);
        if (g_mt6701_angle10 > 3599U) { g_mt6701_angle10 = 3599U; }
    }
    return 1;
}

/**
 * @brief  取去极值均值后的电压（mV）
 */
uint16 MT6701_GetMv(void)
{
    return g_mt6701_mv;
}

/**
 * @brief  取角度（0.1° 单位，未滤波）
 * @note   ↔ Inter_zxmotor.c: MT6701_GetAngleRaw()
 */
uint16 MT6701_GetAngle10(void)
{
    return g_mt6701_angle10;
}

/**
 * @brief  取角度（float，度；带一阶低通，α = MT6701_ANA_LPF_ALPHA，0=不滤波）
 * @note   ↔ Inter_zxmotor.c: MT6701_GetAngle()（原工程 α 来自参数表 filter_alpha=0.3f）
 *         本期只上报，未接入控制；将来做位置环时用这个
 */
float MT6701_GetAngleF(void)
{
#if (MT6701_ANA_LPF_EN)
    static float y_prev = 0.0f;
    static uint8 inited = 0;
    float raw = (float)g_mt6701_angle10 / 10.0f;

    if (!inited) { y_prev = raw; inited = 1; return raw; }
    y_prev += MT6701_ANA_LPF_ALPHA * (raw - y_prev);
    return y_prev;
#else
    return (float)g_mt6701_angle10 / 10.0f;     /* 不滤波：直接用去极值均值算出来的角度 */
#endif
}

/**
 * @brief  差分测速（°/s）
 * @param  dt_ms 两次调用之间的间隔(ms)
 * @note   ↔ Inter_zxmotor.c: MT6701_GetSpeed()（原工程 dt=10ms、内部还有 0.2 低通）
 *         本版未使用；要用时按固定周期调用
 */
float MT6701_GetSpeedDps(uint16 dt_ms)
{
    static uint16 prev10 = 0;
    static uint8  first  = 1;
    static float  spd    = 0.0f;
    float raw;

    if (dt_ms == 0) { return 0.0f; }
    if (first) { prev10 = g_mt6701_angle10; first = 0; return 0.0f; }

    raw = ((float)g_mt6701_angle10 - (float)prev10) / 10.0f * 1000.0f / (float)dt_ms;
    prev10 = g_mt6701_angle10;
    spd += 0.2f * (raw - spd);                  /* 0.2 低通，同参考工程 */
    return spd;
}

/**
 * @brief  周期上报模拟量（每批一帧，DLC=8）
 * @note   0x1F2: [0..1]角度×10(0.1°,大端) [2..3]电压mV [4..5]15位码值
 *                [6]状态 bit0=满批 bit1=低于量程 bit2=高于量程  [7]峰峰值mV(截 255)
 *         ↔ app_motor.c 里 g_dbg.steer.mt6701_voltage_mv 那种 Watch 观察 → 改发 CAN
 *         TXING 忙就跳过，不阻塞 1ms 任务
 */
void MT6701_SendObs(void)
{
    uint8  f[8];
    uint16 pp;

    if (ReadBit(CAN_SR, TXING)) { return; }

    pp = g_mt6701_pp;
    if (pp > 255U) { pp = 255U; }

    f[0] = (uint8)(g_mt6701_angle10 >> 8);
    f[1] = (uint8)g_mt6701_angle10;
    f[2] = (uint8)(g_mt6701_mv >> 8);
    f[3] = (uint8)g_mt6701_mv;
    f[4] = (uint8)(g_mt6701_raw15 >> 8);
    f[5] = (uint8)g_mt6701_raw15;
    f[6] = g_mt6701_anaSt;
    f[7] = (uint8)pp;
    can_hw_send(0x1F2, f, 8);
}