#ifndef CAN_H
#define CAN_H

void FilterInit(void);
void CAN_Transmit(uint32_t ID, uint8_t Length, uint8_t *Data);
void CAN_Receive(uint32_t *ID, uint8_t *Length, uint8_t *Data);

#endif
