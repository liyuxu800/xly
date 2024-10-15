#ifndef PID_SINGLE_LOOP_H
#define PID_SINGLE_LOOP_H

typedef struct
{
  float kp, ki, kd;            // 三个系数
  float error, lastError;      // 误差、上次误�?????
  float integral, maxIntegral; // 积分、积分限�?????
  float output, maxOutput;     // 输出、输出限�?????
} PID;
extern PID mypid;

void PID_Init(PID *pid, float p, float i, float d, float maxI, float maxOut);
void PID_Calc(PID *pid, float reference, float feedback);
float emaFilter(float input, float *prev_ema, float alpha);

#endif
