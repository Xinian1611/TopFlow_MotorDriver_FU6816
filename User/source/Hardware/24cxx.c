/**
 * @file     24cxx.c
 * @brief    AT24Cxx/BL24Cxx EEPROM驱动，使用IO模拟I2C方式驱动
 * @note     移植自 MCU软件平台\00-Common\Project_XXX\BSP\BSP\24cxx
 *           适配FU68xx系列MCU，改用GPIO bit-bang实现I2C协议
 */

#include <MyProject.h>
#include "24cxx.h"

/* I2C时序延迟（调整此值可改变I2C时钟频率）-----------------------------------*/
#define AT24CXX_I2C_DELAY_CYCLES      250

/* 写操作超时重试次数 --------------------------------------------------------*/
#define AT24CXX_WRITE_TIMEOUT         50000

/* 内部函数声明 -------------------------------------------------------------*/
static void AT24Cxx_I2C_Delay(void);
static void AT24Cxx_I2C_Start(void);
static void AT24Cxx_I2C_Stop(void);
static uint8 AT24Cxx_I2C_WriteByte(uint8 dat);
static uint8 AT24Cxx_I2C_ReadByte(uint8 ack);
static uint8 AT24Cxx_I2C_WriteAddr(uint16 addr);

/*----------------------------------------------------------------------------*/
/*  内部函数 - I2C位时序实现                                                   */
/*----------------------------------------------------------------------------*/

/**
 * @brief  I2C时序延迟函数
 * @note   通过空循环产生约5us延迟，配合SCL高/低电平切换实现100kHz I2C时钟
 */
static void AT24Cxx_I2C_Delay(void)
{
    uint8 i = AT24CXX_I2C_DELAY_CYCLES;
    while (i--);
}

/**
 * @brief  I2C起始条件：SCL高电平时，SDA由高变低
 */
static void AT24Cxx_I2C_Start(void)
{
    AT24CXX_SDA_H();
    AT24CXX_SCL_H();
    AT24Cxx_I2C_Delay();
    AT24CXX_SDA_L();
    AT24Cxx_I2C_Delay();
    AT24CXX_SCL_L();
}

/**
 * @brief  I2C停止条件：SCL高电平时，SDA由低变高
 */
static void AT24Cxx_I2C_Stop(void)
{
    AT24CXX_SDA_L();
    AT24CXX_SCL_H();
    AT24Cxx_I2C_Delay();
    AT24CXX_SDA_H();
    AT24Cxx_I2C_Delay();
}

/**
 * @brief  I2C发送一个字节（MSB优先）
 * @param  dat 待发送的数据字节
 * @retval 0=收到ACK，1=收到NACK
 */
static uint8 AT24Cxx_I2C_WriteByte(uint8 dat)
{
    uint8 i;

    for (i = 0; i < 8; i++)
    {
        if (dat & 0x80)
            AT24CXX_SDA_H();
        else
            AT24CXX_SDA_L();

        dat <<= 1;
        AT24Cxx_I2C_Delay();
        AT24CXX_SCL_H();
        AT24Cxx_I2C_Delay();
        AT24CXX_SCL_L();
    }

    /* 释放SDA，接收ACK */
    AT24CXX_SDA_H();
    AT24Cxx_I2C_Delay();
    AT24CXX_SCL_H();

    /* 读取ACK */
    if (AT24CXX_SDA_READ())
    {
        AT24CXX_SCL_L();
        return 1;   /* NACK */
    }
    AT24CXX_SCL_L();
    return 0;       /* ACK */
}

/**
 * @brief  I2C接收一个字节
 * @param  ack 1=发送ACK，0=发送NACK
 * @retval 接收到的数据字节
 */
static uint8 AT24Cxx_I2C_ReadByte(uint8 ack)
{
    uint8 i;
    uint8 dat = 0;

    /* SDA设为输入 */
    AT24CXX_SDA_H();

    for (i = 0; i < 8; i++)
    {
        dat <<= 1;
        AT24CXX_SCL_H();
        AT24Cxx_I2C_Delay();
        if (AT24CXX_SDA_READ())
            dat |= 0x01;
        AT24CXX_SCL_L();
        AT24Cxx_I2C_Delay();
    }

    /* 发送ACK/NACK */
    if (ack)
        AT24CXX_SDA_L();    /* ACK: SDA低 */
    else
        AT24CXX_SDA_H();    /* NACK: SDA高 */

    AT24Cxx_I2C_Delay();
    AT24CXX_SCL_H();
    AT24Cxx_I2C_Delay();
    AT24CXX_SCL_L();
    AT24CXX_SDA_H();        /* 释放SDA */

    return dat;
}

/**
 * @brief  向EEPROM发送16位地址（2字节，MSB先发）
 * @param  addr 16位地址
 * @retval 0=成功，1=失败
 */
static uint8 AT24Cxx_I2C_WriteAddr(uint16 addr)
{
    if (AT24Cxx_I2C_WriteByte((uint8)(addr >> 8)))
        return 1;
    if (AT24Cxx_I2C_WriteByte((uint8)(addr & 0xFF)))
        return 1;
    return 0;
}

/*----------------------------------------------------------------------------*/
/*  擦除方法（移植自MCU软件平台\00-Common\Project_XXX\BSP\BSP\24cxx）          */
/*----------------------------------------------------------------------------*/

/**
 * @brief  用固定值填充一页
 * @param  page_index 页索引（0起始）
 * @param  val        填充值（擦除时传0xFF）
 * @retval 0=成功，1=失败
 */
uint8 AT24Cxx_PageFill(uint16 page_index, uint8 val)
{
    uint8 xdata buf[AT24CXX_PAGE_SIZE];
    uint16 i;

    for (i = 0; i < AT24CXX_PAGE_SIZE; i++)
        buf[i] = val;

    return AT24Cxx_WritePage(page_index * AT24CXX_PAGE_SIZE, buf, AT24CXX_PAGE_SIZE);
}

/**
 * @brief  擦除EEPROM数据（写0xFF）
 * @param  page_start 起始页索引
 * @param  page_cnt   要擦除的页数
 * @param  mode       擦除模式：
 *                    - AT24CXX_ERASE_SECTOR：扇区擦除，从page_start擦除page_cnt页
 *                    - AT24CXX_ERASE_CHIP：整片擦除，忽略page_start/page_cnt
 * @note   EEPROM无独立擦除命令，擦除实质为写入0xFF
 */
void AT24Cxx_Erase(uint16 page_start, uint16 page_cnt, uint8 mode)
{
    uint16 cnt;
    uint16 max_pages;

    max_pages = (AT24CXX_MAX_ADDR + 1) / AT24CXX_PAGE_SIZE;

    if (mode == AT24CXX_ERASE_CHIP)
    {
        page_start = 0;
        cnt = max_pages;
    }
    else
    {
        cnt = page_cnt;
        if (page_start + cnt > max_pages)
            cnt = max_pages - page_start;
    }

    AT24Cxx_WriteEnable();

    while (cnt--)
    {
        AT24Cxx_PageFill(page_start, 0xFF);
        page_start++;
    }
}

/*----------------------------------------------------------------------------*/
/*  外部接口函数                                                               */
/*----------------------------------------------------------------------------*/

/**
 * @brief  初始化EEPROM驱动
 * @note   配置I2C引脚（SCL/SDA为开漏输出，WP为推挽输出），关闭硬件I2C外设
 */
void AT24Cxx_Init(void)
{
    /* 关闭硬件I2C外设，释放引脚控制权给GPIO */
    ClrBit(I2C_CR, I2CEN);

    /* 配置SCL(P06)为推挽输出 */
    SetBit(P0_OE, P06);
    GP06 = 1;

    /* 配置SDA(P05)：初始为输入（高阻）状态，靠上拉拉高 */
    ClrBit(P0_OE, P05);

    /* 使能内部上拉 */
    SetBit(P0_PU, P05);
    SetBit(P0_PU, P06);

    /* 配置WP(P04)为推挽输出，默认写保护（FU6816L：EEPROM_WP = 引脚21 = P0.4） */
    SetBit(P0_OE, P04);
    GP04 = 1;

    AT24Cxx_WriteEnable();
}

/**
 * @brief  使能写入（取消写保护）
 */
void AT24Cxx_WriteEnable(void)
{
    AT24CXX_WP_CLR();
}

/**
 * @brief  禁止写入（使能写保护）
 */
void AT24Cxx_WriteDisable(void)
{
    AT24CXX_WP_SET();
}

/**
 * @brief  从指定地址读取一个字节
 * @param  addr 16位地址
 * @retval 读取到的数据，失败返回0xFF
 */
uint8 AT24Cxx_ReadByte(uint16 addr)
{
    uint8 dat;

    AT24Cxx_I2C_Start();
    if (AT24Cxx_I2C_WriteByte(AT24CXX_DEV_ADDR_W))
    {
        AT24Cxx_I2C_Stop();
        return 0xFF;
    }
    if (AT24Cxx_I2C_WriteAddr(addr))
    {
        AT24Cxx_I2C_Stop();
        return 0xFF;
    }

    /* 重复起始条件，切换为读操作 */
    AT24Cxx_I2C_Start();
    AT24Cxx_I2C_WriteByte(AT24CXX_DEV_ADDR_R);

    dat = AT24Cxx_I2C_ReadByte(0);  /* NACK，单字节读 */
    AT24Cxx_I2C_Stop();

    return dat;
}

/**
 * @brief  从指定地址读取多个字节
 * @param  addr 起始地址
 * @param  buf  数据缓冲区指针
 * @param  len  读取字节数
 * @retval 实际读取的字节数（失败返回0）
 */
uint16 AT24Cxx_ReadBytes(uint16 addr, uint8 *buf, uint16 len)
{
    uint16 i;

    if (buf == NULL || len == 0)
        return 0;

    AT24Cxx_I2C_Start();
    if (AT24Cxx_I2C_WriteByte(AT24CXX_DEV_ADDR_W))
    {
        AT24Cxx_I2C_Stop();
        return 0;
    }
    if (AT24Cxx_I2C_WriteAddr(addr))
    {
        AT24Cxx_I2C_Stop();
        return 0;
    }

    AT24Cxx_I2C_Start();
    AT24Cxx_I2C_WriteByte(AT24CXX_DEV_ADDR_R);

    for (i = 0; i < len; i++)
    {
        buf[i] = AT24Cxx_I2C_ReadByte((i == len - 1) ? 0 : 1);
    }
    AT24Cxx_I2C_Stop();

    return len;
}

/**
 * @brief  向指定地址写入一个字节
 * @param  addr 地址
 * @param  dat  待写入数据
 * @retval 0=成功，1=失败
 */
uint8 AT24Cxx_WriteByte(uint16 addr, uint8 dat)
{
    AT24Cxx_WriteEnable();

    AT24Cxx_I2C_Start();
    if (AT24Cxx_I2C_WriteByte(AT24CXX_DEV_ADDR_W))
    {
        AT24Cxx_I2C_Stop();
        return 1;
    }
    if (AT24Cxx_I2C_WriteAddr(addr))
    {
        AT24Cxx_I2C_Stop();
        return 1;
    }
    if (AT24Cxx_I2C_WriteByte(dat))
    {
        AT24Cxx_I2C_Stop();
        return 1;
    }
    AT24Cxx_I2C_Stop();

    /* 等待EEPROM内部写入完成（轮询ACK） */
    {
        uint16 timeout = AT24CXX_WRITE_TIMEOUT;
        do
        {
            AT24Cxx_I2C_Start();
            if (AT24Cxx_I2C_WriteByte(AT24CXX_DEV_ADDR_W) == 0)
            {
                AT24Cxx_I2C_Stop();
                break;
            }
            AT24Cxx_I2C_Stop();
            if (--timeout == 0)
                return 1;
        } while (1);
    }

    return 0;
}

/**
 * @brief  向指定地址写入多个字节（自动跨页处理）
 * @param  addr 起始地址
 * @param  buf  数据缓冲区指针
 * @param  len  写入字节数
 * @retval 实际写入的字节数（失败返回0）
 */
uint8 AT24Cxx_WriteBytes(uint16 addr, const uint8 *buf, uint16 len)
{
    uint16 total = 0;
    uint16 cnt;
    uint16 offset;

    if (buf == NULL || len == 0)
        return 0;

    while (total < len)
    {
        offset = addr + total;
        /* 计算当前页剩余空间 */
        cnt = AT24CXX_PAGE_SIZE - (offset % AT24CXX_PAGE_SIZE);
        if (cnt > (len - total))
            cnt = len - total;
        
        AT24Cxx_Erase(offset / AT24CXX_PAGE_SIZE, 1, AT24CXX_ERASE_SECTOR);

        if (AT24Cxx_WritePage(offset, buf + total, cnt))
            return total;

        total += cnt;
    }
    return total;
}

/**
 * @brief  页写入（单页内写入，不做跨页处理）
 * @param  addr 起始地址（页内偏移）
 * @param  buf  数据缓冲区指针
 * @param  len  写入字节数（不超过AT24CXX_PAGE_SIZE）
 * @retval 0=成功，1=失败
 */
uint8 AT24Cxx_WritePage(uint16 addr, const uint8 *buf, uint16 len)
{
    uint16 i;
    uint16 timeout;

    if (buf == NULL || len == 0 || len > AT24CXX_PAGE_SIZE)
        return 1;

    AT24Cxx_WriteEnable();

    AT24Cxx_I2C_Start();
    if (AT24Cxx_I2C_WriteByte(AT24CXX_DEV_ADDR_W))
    {
        AT24Cxx_I2C_Stop();
        return 1;
    }
    if (AT24Cxx_I2C_WriteAddr(addr))
    {
        AT24Cxx_I2C_Stop();
        return 1;
    }
    for (i = 0; i < len; i++)
    {
        if (AT24Cxx_I2C_WriteByte(buf[i]))
        {
            AT24Cxx_I2C_Stop();
            return 1;
        }
    }
    AT24Cxx_I2C_Stop();

    /* 等待EEPROM内部写入完成 */
    timeout = 0;
    do
    {
        AT24Cxx_I2C_Start();
        if (AT24Cxx_I2C_WriteByte(AT24CXX_DEV_ADDR_W) == 0)
        {
            AT24Cxx_I2C_Stop();
            break;
        }
        AT24Cxx_I2C_Stop();
        if (++timeout > AT24CXX_WRITE_TIMEOUT)
            return 1;
    } while (1);

    return 0;
}
