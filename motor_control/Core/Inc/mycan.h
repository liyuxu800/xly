#ifndef MYCAN_H
#define MYCAN_H

#include "stm32f4xx.h"

extern CAN_RxHeaderTypeDef rceStu_can1_fifo0;
extern uint8_t Data_can1_fifo0[8];
extern CAN_RxHeaderTypeDef rceStu_can2_fifo0;
extern uint8_t Data_can2_fifo0[8];

void FilterInit(void);
void CAN1_Transmit(uint32_t ID, uint8_t Length, uint8_t *Data);
void CAN1_Receive(CAN_RxHeaderTypeDef *rceStu, uint8_t *Data);
void CAN2_Transmit(uint32_t ID, uint8_t Length, uint8_t *Data);
void CAN2_Receive(CAN_RxHeaderTypeDef *rceStu, uint8_t *Data);
void HAL_CAN_RxFifo0GetDataCallback(CAN_HandleTypeDef *hcan,CAN_RxHeaderTypeDef rceStu,uint8_t *Data);
uint8_t Can2_Receive_Judgment(uint16_t tagetID, uint8_t *receivedata);

#endif
