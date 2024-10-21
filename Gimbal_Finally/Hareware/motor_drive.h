#ifndef MOTOR_DRIVE_H
#define MOTOR_DRIVE_H

//cpp
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
float mpu_pitch, mpu_roll, mpu_yaw;
float mpu_gx, mpu_gy, mpu_gz;

// yaw
typedef struct
{
  //  yaw_pid
  CascadePID pid_two;
  // yaw_Tx
  uint32_t TxID;
  uint8_t TxLength;
  uint8_t TxData[8];
  // yaw_pid
  float feedbackValue;
  float feedbackValue1;
  float targetValue;
  int16_t Speed;
  float outerTarget;
  float outerFeedback;
  float innerFeedback;
  // yaw_Rx
  uint32_t RxID;
  uint8_t RxLength;
  uint8_t RxData[8];
  // yaw_Measurement
  uint16_t encoder;
  float Angle;
  // yaw_can_return_flag
  uint8_t flag;
} YawStructDef;
extern YawStructDef YawStruct;

// FrictionWheel
typedef struct
{
  // FrictionWheel_pid_L
  PID pid_one_L;
  // FrictionWheel_Tx_L
  uint32_t TxID_L;
  uint8_t TxLength_L;
  uint8_t TxData_L[8];
  // FrictionWheel_pid_L
  float feedbackValue_L;
  float feedbackValue1_L;
  float targetValue_L;
  int16_t Speed_L;
  // FrictionWheel_Rx_L
  uint32_t RxID_L; // 接受�????
  uint8_t RxLength_L;
  uint8_t RxData_L[8];
  // FrictionWheel_can_return_flag_L
  uint8_t flag_L;
  // FrictionWheel_pid_R
  PID pid_one_R;
  // FrictionWheel_Tx_R
  uint32_t TxID_R;
  uint8_t TxLength_R;
  uint8_t TxData_R[8];
  // FrictionWheel_pid_R
  float feedbackValue_R;
  float feedbackValue1_R;
  float targetValue_R;
  int16_t Speed_R;
  // FrictionWheel_Rx_R
  uint32_t RxID_R; // 接受�????
  uint8_t RxLength_R;
  uint8_t RxData_R[8];
  // FrictionWheel_can_return_flag_R
  uint8_t flag_R;
} FWStructDef;
extern FWStructDef FWStruct;

// pitch
typedef struct
{
  // pitch_pid
  CascadePID pid_two;
  // pitch_Tx
  uint32_t TxID;
  uint8_t TxLength;
  uint8_t TxData[8];
  // pitch_pid
  float feedbackValue;
  float feedbackValue1;
  float targetValue;
  int16_t Speed;
  float outerTarget;
  float outerFeedback;
  float innerFeedback;
  // pitch_Rx
  uint32_t RxID; // 接受�????
  uint8_t RxLength;
  uint8_t RxData[8];
  // pitch_Measurement
  uint16_t encoder;
  float Angle;
  // pitch_can_return_flag_L
  uint8_t flag;
} pitchStructDef;
extern pitchStructDef pitchStruct;

// dial
typedef struct
{
  // dial_pid
  CascadePID pid_two;
  // dial_Tx
  uint32_t TxID;
  uint8_t TxLength;
  uint8_t TxData[8];
  // dial_pid
  float feedbackValue;
  float feedbackValue1;
  float targetValue;
  int16_t Speed;
  float outerTarget;
  float outerFeedback;
  float innerFeedback;
  // dial_Rx
  uint32_t RxID; // 接受�????
  uint8_t RxLength;
  uint8_t RxData[8];
  // dial_Measurement
  uint16_t encoder;
  float Angle;
  // dial_can_return_flag_L
  uint8_t flag;
} dialStructDef;
extern dialStructDef dialStruct;

// 平滑因子
float alpha = 0.05;

//函数申明
void FW_Motor_Control(FWStructDef *FWStruct);
void Dial_Motor_Control(dialStructDef dialStruct);
void Yaw_Motor_Control(YawStructDef YawStruct);
void Pitch_Motor_Control(pitchStructDef pitchStruct);

#endif
