#ifndef PID_H
#define PID_H

#include "stm32f4xx_hal.h"

typedef struct
{
  float kp, ki, kd;            // 三个系数
  float error, lastError;      // 误差、上次误�?
  float integral, maxIntegral; // 积分、积分限�?
  float output, maxOutput;     // 输出、输出限�?
} PID;
// extern PID pid_one;

// 串级PID的结构体，包含两个单级PID
typedef struct
{
  PID inner;    // 内环
  PID outer;    // 外环
  float output; // 串级输出，等于inner.output
} CascadePID;
// extern CascadePID pid_two;

typedef struct // 定义一个结构体类型，实现720°旋转
{
  uint16_t encoder, last_encoder; // 当前刻度值和上一次的刻度值
  float finally_angle;            // 最终旋转的角度
} encoderStructDef;
extern encoderStructDef angle_update_dial;

typedef struct // 定义一个结构体类型，实现720°旋转
{
  uint16_t angle, last_angle; // 当前刻度值和上一次的刻度值

  float finally_angle; // 最终旋转的角度
} AngleStructDef;
extern AngleStructDef angle_update_yaw;

void PID_Init(PID *pid, float p, float i, float d, float maxI, float maxOut);
void PID_Calc(PID *pid, float reference, float feedback);
void PID_CascadeCalc(CascadePID *pid, float outerRef, float outerFdb, float innerFdb);
void updata_angle_dial(encoderStructDef *__angle, uint16_t new_encoder);
void updata_angle_yaw(AngleStructDef *__angle, uint16_t new_angle);
float emaFilter(float input, float *prev_ema, float alpha);
float PID_One_Calculation(PID *pid_one, float targetvalue, uint8_t *receivedata);
float PID_Two_Calculation(CascadePID *pid_two, float outtargetvalue, uint8_t *receivedata);

#endif
