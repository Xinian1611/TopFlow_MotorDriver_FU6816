/**
 * @copyright (C) COPYRIGHT 2022 Fortiortech Shenzhen
 * @file      xxx.c
 * @author    Fortiortech  Appliction Team
 * @since     Create:2022-08-05
 * @date      Last modify:2022-08-05
 * @note      Last modify author is Leo.li
 * @brief
 */

 //模拟I2C,用于EEPROM

#include <MyProject.h>

/**
 * 初始化 IIC 总线引脚（FU6866 无开漏/推挽模式，只有方向 Px_OE + 数据位）
 *   高 = 输出 + 数据位 1；低 = 输出 + 数据位 0
 */
void SoftI2C_Init(void)
{
    GP44 = 1;
    /* SCL = P4.4：输出模式*/
    SetBit(P4_OE, P44);

    /* SDA = P0.4：输出模式*/
    GP04 = 1;
    SetBit(P0_OE, P04);

    I2C_SCL_H();
    I2C_SDA_H();
}

/**
 * 起始信号
 */
void SoftI2C_Start(void) {
    I2C_SDA_H();
    I2C_SCL_H();
    I2C_DELAY();

    I2C_SDA_L();
    I2C_DELAY();
    I2C_SCL_L();   // 钳住总线
}

/**
 * 停止信号
 */
void SoftI2C_Stop(void) {
    I2C_SDA_L();
    I2C_SCL_H();
    I2C_DELAY();

    I2C_SDA_H();
    I2C_DELAY();
}

/**
 * 发送一个字节
 */
void SoftI2C_SendByte(uint8_t byte) {
    uint8_t i;
    for (i = 0; i < 8; i++) {
        I2C_SCL_L();
        if (byte & 0x80)
            I2C_SDA_H();
        else
            I2C_SDA_L();
        I2C_DELAY();

        I2C_SCL_H();
        I2C_DELAY();
        I2C_SCL_L();
        I2C_DELAY();

        byte <<= 1;
    }
}

/**
 * 接收一个字节（主机释放 SDA 后读取）
 */
uint8_t SoftI2C_ReceiveByte(void) {
    uint8_t byte = 0;
    uint8_t i;
    I2C_SDA_H();   // 释放 SDA
    for (i = 0; i < 8; i++) {
        I2C_SCL_L();
        I2C_DELAY();

        I2C_SCL_H();
        I2C_DELAY();

        byte <<= 1;
        if (I2C_SDA_READ())
            byte |= 0x01;

        I2C_SCL_L();
        I2C_DELAY();
    }
    return byte;
}

/**
 * 发送 ACK (主机拉低 SDA)
 */
void SoftI2C_SendACK(void) {
    I2C_SDA_L();
    I2C_SCL_L();
    I2C_DELAY();

    I2C_SCL_H();
    I2C_DELAY();
    I2C_SCL_L();
    I2C_DELAY();

    I2C_SDA_H();
}

/**
 * 发送 NACK (主机释放 SDA)
 */
void SoftI2C_SendNoACK(void) {
    I2C_SDA_H();
    I2C_SCL_L();
    I2C_DELAY();

    I2C_SCL_H();
    I2C_DELAY();
    I2C_SCL_L();
    I2C_DELAY();
}

/**
 * 接收 ACK (返回 0 表示应答，非 0 表示无应答)
 */
uint8_t SoftI2C_ReceiveACK(void) {
    uint8_t ack;
    I2C_SDA_H();   // 释放 SDA，等待从机拉低
    I2C_SCL_L();
    I2C_DELAY();

    I2C_SCL_H();
    I2C_DELAY();

    ack = I2C_SDA_READ();

    I2C_SCL_L();
    I2C_DELAY();

    return ack ? 1 : 0;
}

/* ---- 微秒延时：24MHz 下用空循环实现 ----*/
void delay_us(uint32_t us)
{
    while (us--)
    {
        uint8 i;
        for (i = 0; i < 8; i++) { _nop_(); }
    }
}

/*
void I2C_Init_Master(void)
{
    SetBit(P0_PU , P00);
    SetBit(P0_PU , P01);
   // I2C initial
    SetBit(I2C_CR , I2CMS);	    //主从模式选择	0-->Slave Mode  1-->Host Mode
    ClrBit(I2C_CR , I2CSPD1);	//00-->100kHz		01-->400kHz
    SetBit(I2C_CR , I2CSPD0);	//10-->1MHz			11-->RSV
    ClrBit(I2C_CR , I2CIE);	    //中断使能		   0-->Disable 1-->Enable
    I2C_ID = 0xD0;   
    SetBit(I2C_ID , GC);	//广播呼叫使能	0-->Disable 1-->Enable
    SetBit(I2C_CR , I2CEN);	//I2C使能		0-->Disable 1-->Enable
}

void I2C_ID_Start(bool rw)
{
    if(rw)
        SetBit(I2C_SR , DMOD);	
    else
        ClrBit(I2C_SR , DMOD);	
    SetBit(I2C_SR , I2CSTA);	//1-->发送START或RESTART和地址字节
    while(!ReadBit(I2C_SR , STR));
    ClrBit(I2C_SR , STR);    
}

void I2C_Addr_Write(unsigned char addr,unsigned char wdata)
{
    I2C_ID_Start(0);    // ID write

    I2C_DR = addr;      // Slave reg addr.
    while(!ReadBit(I2C_SR , STR));	
    ClrBit(I2C_SR , STR);

    I2C_DR = wdata;     // Slave reg data.
    while(!ReadBit(I2C_SR , STR));	
    ClrBit(I2C_SR , STR);
    SetBit(I2C_SR , I2CSTP);	//1-->发送STOP
}

unsigned char I2C_Addr_Read(char addr)
{
    unsigned char rd_data;
    //write process  
    I2C_ID_Start(0);  // write+ID
    I2C_DR = addr;   // write reg. addr
    while(!ReadBit(I2C_SR , STR));	
    ClrBit(I2C_SR , STR);
//    SetBit(I2C_SR , I2CSTP);	//1-->发送STOP

    //read process
    I2C_ID_Start(1);  // read+ID
    while(!ReadBit(I2C_SR , STR));	 
    ClrBit(I2C_SR , STR);
    rd_data = (unsigned char)(I2C_DR); // read data
    SetBit(I2C_SR , NACK);      //第9位发送NACK 
    SetBit(I2C_SR , I2CSTP);    //1-->发送STOP
    return(rd_data);
}*/
