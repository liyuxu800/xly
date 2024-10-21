#ifndef MOTOR_DRIVE_H
#define MOTOR_DRIVE_H

// cpp
#include "string.h"
#include "stdio.h"

// 自己移植的函�????????
#include "mycan.h"
#include "PID.h"
#include "DR16_control.h"

// 外设
#include "usart.h"
#include "can.h"

// mpu
#include "MPU6050.h"
#include "inv_mpu.h"

// mpu
extern float mpu_pitch, mpu_roll, mpu_yaw;

//数据包
extern uint8_t FW_dial_TxData[];

// yaw
typedef struct
{
  //  yaw_pid
  CascadePID pid_two;
  // yaw_Tx
  uint8_t TxData[8];
  // yaw_pid
  int16_t Speed;
  float outerTarget;
  float outerFeedback;
  float innerFeedback;
  // yaw_Rx
  uint8_t RxData[8];
  // yaw_Measurement
  float Angle;
} YawStructDef;
extern YawStructDef YawStruct;

// FrictionWheel
typedef struct
{
  // FrictionWheel_pid_L
  PID pid_one_L;
  // FrictionWheel_Tx_L
  uint8_t TxData_L[8];
  // FrictionWheel_pid_L
  float feedbackValue_L;
  float targetValue_L;
  // FrictionWheel_Rx_L
  uint8_t RxData_L[8];
  // FrictionWheel_pid_R
  PID pid_one_R;
  // FrictionWheel_Tx_R
  uint8_t TxData_R[8];
  // FrictionWheel_pid_R
  float feedbackValue_R;
  float targetValue_R;
  // FrictionWheel_Rx_R
  uint8_t RxData_R[8];
} FWStructDef;
extern FWStructDef FWStruct;

// pitch
typedef struct
{
  // pitch_pid
  CascadePID pid_two;
  // pitch_Tx
  uint8_t TxData[8];
  // pitch_pid
  int16_t Speed;
  float outerTarget;
  float outerFeedback;
  float innerFeedback;

  // pitch_Rx
  uint8_t RxData[8];
  // pitch_Measurement
  float Angle;
} pitchStructDef;
extern pitchStructDef pitchStruct;

// dial
typedef struct
{
  // dial_pid
  CascadePID pid_two;
  // dial_Tx
  uint8_t TxData[8];
  // dial_pid
  int16_t Speed;
  float outerTarget;
  float outerFeedback;
  float innerFeedback;
  // dial_Rx
  uint8_t RxData[8];
  // dial_Measurement
  uint16_t encoder;
  float Angle;
} dialStructDef;
extern dialStructDef dialStruct;

// 函数申明
void FW_Motor_Control(FWStructDef *FWStruct);
void Dial_Motor_Control(dialStructDef *dialStruct);
void Yaw_Motor_Control(YawStructDef *YawStruct);
void Pitch_Motor_Control(pitchStructDef *pitchStruct);

#endif
