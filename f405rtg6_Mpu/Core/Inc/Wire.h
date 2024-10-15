#ifndef _WIRE_H_
#define _WIRE_H_

#include "main.h"

typedef enum{
    NACK = 0,
    ACK  = 1
}ACK_STATUS;

#define IIC_DELAY_TIME 2

#define IIC_SCL_PIN GPIO_PIN_6
#define IIC_SCL_PORT GPIOB

#define IIC_SDA_PIN GPIO_PIN_7
#define IIC_SDA_PORT GPIOB

#define IIC_SCL_H()     HAL_GPIO_WritePin(IIC_SCL_PORT,IIC_SCL_PIN,GPIO_PIN_SET)
#define IIC_SCL_L()     HAL_GPIO_WritePin(IIC_SCL_PORT,IIC_SCL_PIN,GPIO_PIN_RESET)
#define IIC_SDA_H()     HAL_GPIO_WritePin(IIC_SDA_PORT,IIC_SDA_PIN,GPIO_PIN_SET)
#define IIC_SDA_L()     HAL_GPIO_WritePin(IIC_SDA_PORT,IIC_SDA_PIN,GPIO_PIN_RESET)

void IIC_Delay(void);
void Soft_IIC_Start(void);
void Soft_IIC_Stop(void);
void Soft_IIC_ACK(void);
void Soft_IIC_NACK(void);
uint8_t Soft_IIC_Wait_ACK(void);
uint8_t Soft_IIC_Write_Byte(uint8_t);
uint8_t Soft_IIC_Recv_Byte(ACK_STATUS);

void Wire_Init(void);
HAL_StatusTypeDef Wire_Write(uint8_t, uint8_t, uint8_t, const uint8_t *);
HAL_StatusTypeDef Wire_Read(uint8_t, uint8_t, uint8_t, uint8_t *);

#endif
