#include "Wire.h"
#include "main.h"

/**
 * @brief IIC延时
 * @param  
 * @return 无
 */
void IIC_Delay()
{
    uint32_t i =  IIC_DELAY_TIME * 500; // 用户根据自己的MCU时钟进行设置倍数

    while (i--)
    {
        ;
    }
}

/**
 * @brief IIC起始信号
 * @param  无
 * @return 无
 */
void Soft_IIC_Start(void)
{
	IIC_SDA_H();
	IIC_SCL_H();
	IIC_Delay();
	IIC_SDA_L();
	IIC_Delay();
	IIC_SCL_L();
	IIC_Delay();
}

/**
 * @brief IIC停止信号
 * @param  无
 * @return 无
 */
void Soft_IIC_Stop(void)
{
    IIC_SDA_L();
    IIC_Delay();
    IIC_SCL_H();
    IIC_Delay();
    IIC_SDA_H();
}

/**
 * @brief IIC应答信号
 * @param  无
 * @return 无
 */
void  Soft_IIC_ACK(void)
{
    IIC_SCL_L();
    IIC_Delay();
    IIC_SDA_L();
    IIC_Delay();
    IIC_SCL_H();
    IIC_Delay();

    IIC_SCL_L();
    IIC_Delay();
    IIC_SDA_H();
    IIC_Delay();
}
/**
 * @brief IIC无应答信号
 * @param  无
 * @return 无
 */
void Soft_IIC_NACK(void)
{
    IIC_SCL_L();
    IIC_Delay();
    IIC_SDA_H();
    IIC_Delay();
    IIC_SCL_H();
    IIC_Delay();
}

/**
 * @brief IIC等待应答信号
 * @param  无
 * @return 0无应答  1有应答
 */
uint8_t Soft_IIC_Wait_ACK(void)
{
    IIC_SDA_H();
    IIC_Delay();
    IIC_SCL_H();
    IIC_Delay();
    uint8_t ack = 1;
    long counter = 0;
    while (HAL_GPIO_ReadPin(IIC_SDA_PORT, IIC_SDA_PIN))
    {
        counter++;
        if (counter >= 500)
        {
            ack = 0;
            break;
        }
    }
    IIC_SCL_L();
    IIC_Delay();
    return ack;
}

/**
 * @brief IIC写入单个数据
 * @param  无
 * @return 应答信号, 0无应答 1有应答
 */
uint8_t Soft_IIC_Write_Byte(uint8_t _ucByte)
{
    uint8_t i;
    for(i=0;i<8;i++)
    {
        if(_ucByte & 0x80)
        {
            IIC_SDA_H();
        }
        else
        {
            IIC_SDA_L();
        }
        IIC_Delay();
        IIC_SCL_H();
        IIC_Delay();
        IIC_SCL_L();
      _ucByte <<= 1;//左移一个bit
       IIC_Delay();
    }
		IIC_SDA_H();    //释放总线
    return (Soft_IIC_Wait_ACK());
}

/**
 * @brief IIC读一个数据
 * @param  ACK:应答 NACK:不应答
 * @return 返回读到的数据
 */
uint8_t Soft_IIC_Recv_Byte(ACK_STATUS ack_sta)
{
    uint8_t i;
    uint8_t value = 0;
    /*读取到第一个bit为数据的bit7*/
    for (i = 0; i < 8; i++)
    {
        value <<= 1;
        IIC_SCL_H();
        IIC_Delay();
        if (HAL_GPIO_ReadPin(IIC_SDA_PORT, IIC_SDA_PIN))
        {
            value++;
        }
        IIC_SCL_L();
        IIC_Delay();
    }
    if (!ack_sta)
        Soft_IIC_NACK();
    else
        Soft_IIC_ACK();
    return value;
}

/**
 * @brief IIC初始化
 * @param  无
 * @return 无
 */
void Wire_Init(void)
{
    Soft_IIC_Stop();
}

/**
 * @brief IIC写数据
 * @param  device_addr 设备地址
 * @param  memory_addr 内存地址
 * @param  data_size 数据大小
 * @param  data 数据指针
 * @return (BaseType_t)
 */
HAL_StatusTypeDef Wire_Write(uint8_t device_addr, uint8_t memory_addr, uint8_t data_size, const uint8_t *data)
{
    // 启动信号
    Soft_IIC_Start();
    // 写设备地址
    if (Soft_IIC_Write_Byte((device_addr << 1)) == 0)
    {
        // 如果设备应答失败，则终止传输
        Soft_IIC_Stop();
        return HAL_ERROR;
    }
    // 发送内存地址
    if (Soft_IIC_Write_Byte(memory_addr) == 0)
    {
        // 如果内存地址应答失败，则终止传输
        Soft_IIC_Stop();
        return HAL_ERROR;
    }
    // 写入数据
    while (data_size--)
    {
        if (Soft_IIC_Write_Byte(*data++) == 0)
        {
            // 如果数据应答失败，则终止传输
            Soft_IIC_Stop();
            return HAL_ERROR;
        }
    }
    // 停止信号
    Soft_IIC_Stop();
    return HAL_OK;
}

/**
 * @brief IIC读数据
 * @param  device_addr 设备地址
 * @param  memory_addr 内存地址
 * @param  data_size 数据大小
 * @param  data 数据指针
 * @return (BaseType_t)
 */
HAL_StatusTypeDef Wire_Read(uint8_t device_addr, uint8_t memory_addr, uint8_t data_size, uint8_t *data)
{
    uint8_t i;
    // 启动信号
    Soft_IIC_Start();
    // 写设备地址
    if (Soft_IIC_Write_Byte((device_addr << 1)) == 0)
    {
        // 如果设备应答失败，则终止传输
        Soft_IIC_Stop();
        return HAL_ERROR;
    }
    // 发送内存地址
    if (Soft_IIC_Write_Byte(memory_addr) == 0)
    {
        // 如果内存地址应答失败，则终止传输
        Soft_IIC_Stop();
        return HAL_ERROR;
    }
    // 重新启动信号
    Soft_IIC_Start();
    // 读设备地址（读模式）
    if (Soft_IIC_Write_Byte(((device_addr << 1) | 1)) == 0)
    {
        // 如果设备应答失败，则终止传输
        Soft_IIC_Stop();
        return HAL_ERROR;
    }
    // 读取数据
    for(i = 0; i < data_size - 1; i++)
    {
        *data++ = Soft_IIC_Recv_Byte(ACK); // 在最后一个字节之前发送ACK
    }
    // 最后一个字节发送NACK
    *data++ = Soft_IIC_Recv_Byte(NACK);
    // 停止信号
    Soft_IIC_Stop();
    return HAL_OK;
}
