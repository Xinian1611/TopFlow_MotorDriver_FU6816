/**
 * @file    eeprom.c
 * @brief   EEPROM 驱动层（BL24C256A，24AA256 兼容，软件 I2C）
 * @note    实现要点：
 *          - 16 位地址，先高字节后低字节；
 *          - 页大小 64 字节，页写不得跨页（跨页地址自动回卷覆盖页首）；
 *          - 每次写操作 STOP 后 ACK 轮询器件地址，等待内部写周期完成（最大 5ms）；
 *          - WP（PB12）写操作前自动拉低解除写保护，结束后恢复写保护。
 *          移植参考：GearBox EEPROM_24AA256.c、GD32F30x 例程 at24cxx.c。
 */
#include "EEPROM.h"
#include "I2C.H"

/* ACK 轮询超时次数（每次轮询约 250us，足够覆盖 5ms 写周期） */
#define EEPROM_WRITE_POLL_TIMEOUT   1000

static int16_t EEPROM_HwWaitStandby(void);
static int16_t EEPROM_HwWritePage(uint16_t addr, uint16_t len, const uint8_t *buf);
static int16_t EEPROM_HwReadCore(uint16_t addr, uint16_t len, uint8_t *buf);

/**
 * @brief  初始化 EEPROM 引脚与 I2C 总线
 * @return EEPROM_OK
 */
int16_t EEPROM_Init(void)
{
    /* WP = P0.7：推挽输出，默认拉低解除写保护
     * （板上 R47 10k 把 WP 上拉到 VCC5V，上电默认是"写保护"，所以这里要主动拉低）*/
    SetBit(P0_OE, P07);
    GP07 = 0;

    /* 软件 I2C：P4.4-SCL / P0.4-SDA*/
    SoftI2C_Init();

    return EEPROM_OK;
}

/**
 * @brief  写保护控制
 * @param  enable 1=置写保护（WP 高，只读）；0=解除（WP 低，可写）
 */
int16_t EEPROM_WriteProtect(uint8_t enable)
{
    if (enable) 
    { 
        GP07 = 1; /* 写保护（只读）*/
    }
    else
    { 
        GP07 = 0; /* 解除写保护（可写）*/
    }
    return EEPROM_OK;
}

/**
 * @brief  器件在位检查：发送起始 + 写器件地址，收到 ACK 即就绪
 * @return EEPROM_OK 在位；EEPROM_ERR_DEV_ACK 无应答
 */
int16_t EEPROM_Check(void)
{
    uint8_t ret;

    SoftI2C_Start();
    SoftI2C_SendByte(EEPROM_DEV_ADDR_W);
    ret = SoftI2C_ReceiveACK();    /* 0=收到 ACK，1=NACK */
    SoftI2C_Stop();

    return ret ? EEPROM_ERR_DEV_ACK : EEPROM_OK;
}

/**
 * @brief  ACK 轮询等待内部写周期结束（每次写操作后必须调用）
 * @return EEPROM_OK 芯片就绪；DIR_EEPROM_ERR_TIMEOUT 超时
 */
static int16_t  EEPROM_HwWaitStandby(void)
{
    uint32_t timeout = 0;

    do {
        SoftI2C_Start();
        SoftI2C_SendByte(EEPROM_DEV_ADDR_W);
        if (0 == SoftI2C_ReceiveACK()) {
            SoftI2C_Stop();
            return EEPROM_OK;
        }
        SoftI2C_Stop();
        timeout++;
    } while (timeout < EEPROM_WRITE_POLL_TIMEOUT);

    return EEPROM_ERR_TIMEOUT;
}

/**
 * @brief  页写（≤64 字节且不跨页）
 * @param  addr 目标地址（0x0000~0x7FFF；0x7000 以上为芯片只读区，写入无效）
 * @param  len  数据长度（1~64）
 * @param  buf  数据指针
 * @return 驱动层返回码
 */
static int16_t  EEPROM_HwWritePage(uint16_t addr, uint16_t len, const uint8_t *buf)
{
    int16_t  ret;
    uint16_t i;

    /* 参数校验：地址越界、长度非法、跨页（BL24C256A 页写跨页会回卷覆盖页首） */
    if ((addr > (EEPROM_TOTAL_SIZE - 1)) || (0 == len) || (len > EEPROM_PAGE_SIZE) ||
        ((addr & (EEPROM_PAGE_SIZE - 1)) + len > EEPROM_PAGE_SIZE) || (NULL == buf)) {
        return EEPROM_ERR_PARAM;
    }

    EEPROM_WriteProtect(0);               /* 写前自动解除写保护 */

    SoftI2C_Start();
    SoftI2C_SendByte(EEPROM_DEV_ADDR_W);
    if (SoftI2C_ReceiveACK()) { ret = EEPROM_ERR_DEV_ACK; goto out; }
    SoftI2C_SendByte((uint8_t)(addr >> 8));
    if (SoftI2C_ReceiveACK()) { ret = EEPROM_ERR_ADDRH; goto out; }
    SoftI2C_SendByte((uint8_t)(addr & 0xFF));
    if (SoftI2C_ReceiveACK()) { ret = EEPROM_ERR_ADDRL; goto out; }
    for (i = 0; i < len; i++) {
        SoftI2C_SendByte(buf[i]);
        if (SoftI2C_ReceiveACK()) { ret = EEPROM_ERR_DATA; goto out; }
    }
    SoftI2C_Stop();

    ret = EEPROM_HwWaitStandby();             /* 等待写周期完成 */

    /* 写周期尾保护：BL24C256A 实测 ACK 响应后数据输出仍需约 5ms 才完全就绪
     * （写后 1ms 内读回 0xFF，5ms 后正常），统一延时保证上层“写后立即读”可靠 */
    if (EEPROM_OK == ret) {
        delay_us(5000);
    }

out:
    if (EEPROM_OK != ret) {
        SoftI2C_Stop();             /* 失败路径确保总线释放 */
    }
    EEPROM_WriteProtect(1);               /* 恢复写保护 */
    return ret;
}

/**
 * @brief  连续读公共实现（伪写寻址 + STOP 后重新起始读）
 * @note   不用重复 START：BL24C256A 对“写事务后紧跟的重复 START”响应异常
 *         （实测写后立即读回 0xFF，延时 50ms 后正常）；改为伪写寻址以 STOP
 *         完整结束事务（从机回 standby）后重新 START 发起“当前地址读”，
 *         协议兼容性更好。
 */
static int16_t  EEPROM_HwReadCore(uint16_t addr, uint16_t len, uint8_t *buf)
{
    int16_t  ret;
    uint16_t i;

    SoftI2C_Start();
    SoftI2C_SendByte(EEPROM_DEV_ADDR_W);
    if (SoftI2C_ReceiveACK()) { ret = EEPROM_ERR_DEV_ACK; goto out; }
    SoftI2C_SendByte((uint8_t)(addr >> 8));
    if (SoftI2C_ReceiveACK()) { ret = EEPROM_ERR_ADDRH; goto out; }
    SoftI2C_SendByte((uint8_t)(addr & 0xFF));
    if (SoftI2C_ReceiveACK()) { ret = EEPROM_ERR_ADDRL; goto out; }

    /* 伪写寻址 STOP 结束（从机回 standby），延时后重新 START + 读器件地址 */
    SoftI2C_Stop();
    delay_us(500);

    SoftI2C_Start();
    SoftI2C_SendByte(EEPROM_DEV_ADDR_R);
    if (SoftI2C_ReceiveACK()) { ret = EEPROM_ERR_DEV_ACK; goto out; }

    /* 前 len-1 字节收 ACK，最后一字节收 NACK 后停止 */
    for (i = 0; i < len - 1; i++) {
        buf[i] = SoftI2C_ReceiveByte();
        SoftI2C_SendACK();
    }
    buf[len - 1] = SoftI2C_ReceiveByte();
    SoftI2C_SendNoACK();
    SoftI2C_Stop();
    return EEPROM_OK;

out:
    SoftI2C_Stop();
    return ret;
}

/* ---------------------------------------------------------------------
 * 任意长度连续写：按 64 字节页边界自动拆分为多次页写
 *   输入：addr 起始地址(0x0000~0x7FFF)，len 长度(1~32KB)，buf 数据指针
 *   返回：EEPROM_OK 成功；EEPROM_ERR_PARAM 参数错误；其余=驱动层硬件错误码
 * 说明：驱动层 EEPROM_HwWritePage 每次写事务后已含写周期 ACK 轮询与
 *       5ms 写周期尾延时，逐页写互不影响，写后立即读可靠。
 * ------------------------------------------------------------------- */
int16_t EEPROM_WriteBuffer(uint16_t addr, uint16_t len, const uint8_t *buf)
{
    uint16_t chunk;
    int16_t ret;

    if ((len == 0) || (buf == 0) || ((uint32_t)addr + len > EEPROM_TOTAL_SIZE)) {
        return EEPROM_ERR_PARAM;
    }

    while (len > 0) {
        /* 本页剩余容量 = 页大小 - 页内偏移，取与剩余长度的较小值 */
        chunk = EEPROM_PAGE_SIZE - (addr % EEPROM_PAGE_SIZE);
        if (chunk > len) {
            chunk = len;
        }
        ret = (int16_t)EEPROM_HwWritePage(addr, chunk, buf);
        if (EEPROM_OK != ret) {
            return ret;   /* 原样返回驱动层错误码（1~6，区分失败阶段） */
        }
        addr += chunk;
        buf  += chunk;
        len  -= chunk;
    }
    return EEPROM_OK;
}

/* ---------------------------------------------------------------------
 * 任意长度连续读（可跨页）
 * ------------------------------------------------------------------- */
int16_t EEPROM_ReadBuffer(uint16_t addr, uint16_t len, uint8_t *buf)
{
    if ((len == 0) || (buf == 0) || ((uint32_t)addr + len > EEPROM_TOTAL_SIZE)) {
        return EEPROM_ERR_PARAM;
    }
    return (int16_t)EEPROM_HwReadCore(addr, len, buf);
}

/* ---------------------------------------------------------------------
 * 单字节写 / 单字节读（便捷封装，带参数校验）
 * ------------------------------------------------------------------- */
int16_t EEPROM_WriteByte(uint16_t addr, uint8_t dat)
{
    if (addr >= EEPROM_TOTAL_SIZE) {
        return EEPROM_ERR_PARAM;
    }
    return EEPROM_HwWritePage(addr, 1, &dat);
}

int16_t EEPROM_ReadByte(uint16_t addr, uint8_t *pdat)
{
    if ((pdat == 0) || (addr >= EEPROM_TOTAL_SIZE)) {
        return EEPROM_ERR_PARAM;
    }
    return EEPROM_HwReadCore(addr, 1, pdat);
}

/* ---------------------------------------------------------------------
 * 自检：器件在位 + 读写回环
 *   1. 器件在位检查（发写器件地址等 ACK）
 *   2. 对自检测试地址（0x6FF8，避开 0x0000 一键部署区与 0x6E00 测试区）：
 *      读原值 -> 写约定字节 -> 读回比对 -> 恢复原值 -> 读回确认恢复
 *   返回：EEPROM_OK 通过；其余为失败阶段码（1~6 硬件、7 回环不一致、8 恢复失败）
 * 说明：BL24C256A 无 0x7FFA 出厂 ID 区，不能沿用 GearBox 的
 *       “读 0x7FFA 比对 0x29/0x48”自检，回环验证同样可覆盖总线故障。
 * ------------------------------------------------------------------- */
int16_t EEPROM_SelfCheck(void)
{
    uint8_t old;
    uint8_t rd;
    int16_t ret;

    /* 1. 器件在位 */
    if (EEPROM_OK != EEPROM_Check()) {
        return EEPROM_ERR_DEV_ACK;
    }

    /* 2. 回环：读原值 */
    ret = EEPROM_ReadByte(EEPROM_SELFCHECK_ADDR, &old);
    if (EEPROM_OK != ret) {
        return ret;
    }

    /* 3. 写入约定字节 */
    ret = EEPROM_WriteByte(EEPROM_SELFCHECK_ADDR, EEPROM_SELFCHECK_MAGIC);
    if (EEPROM_OK != ret) {
        return ret;
    }

    /* 4. 读回比对 */
    ret = EEPROM_ReadByte(EEPROM_SELFCHECK_ADDR, &rd);
    if (EEPROM_OK != ret) {
        return ret;
    }
    if (EEPROM_SELFCHECK_MAGIC != rd) {
        return EEPROM_ERR_LOOPBACK;
    }

    /* 5. 恢复原值并确认 */
    ret = EEPROM_WriteByte(EEPROM_SELFCHECK_ADDR, old);
    if (EEPROM_OK != ret) {
        return EEPROM_ERR_RESTORE;
    }
    ret = EEPROM_ReadByte(EEPROM_SELFCHECK_ADDR, &rd);
    if (EEPROM_OK != ret) {
        return EEPROM_ERR_RESTORE;
    }
    if (old != rd) {
        return EEPROM_ERR_RESTORE;
    }

    return EEPROM_OK;
}
