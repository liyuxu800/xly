#ifndef MYCAN_H
#define MYCAN_H

#include "stm32f4xx.h"

void FilterInit(void);
void CAN1_Transmit(uint32_t ID, uint8_t Length, uint8_t *Data);
void CAN1_Receive(uint32_t *ID, uint8_t *Length, uint8_t *Data);

#endif
