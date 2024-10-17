#include "stm32f4xx.h"
#include "PID_Single_Loop.h"

PID mypid = {0};

// 用于初始化pid参数的函�?????
void PID_Init(PID *pid, float p, float i, float d, float maxI, float maxOut)
{
  pid->kp = p;
  pid->ki = i;
  pid->kd = d;
  pid->maxIntegral = maxI;
  pid->maxOutput = maxOut;
}

// 进行�?????次pid计算
// 参数�?????(pid结构�?????,目标�?????,反馈�?????)，计算结果放在pid结构体的output成员成员�?????
void PID_Calc(PID *pid, float reference, float feedback)
{
  // 更新数据
  pid->lastError = pid->error;       // 将旧error存起�?????
  pid->error = reference - feedback; // 计算新error
  // 计算微分
  static float dout;
  dout = (pid->error - pid->lastError) * pid->kd;
  // 计算比例
  static float pout;
  pout = pid->error * pid->kp;
  // 计算积分
  pid->integral += pid->error;
  static float iout;
  iout = pid->integral * pid->ki;
  // 积分限幅
  if (pid->integral > pid->maxIntegral)
    pid->integral = pid->maxIntegral;
  else if (pid->integral < -pid->maxIntegral)
    pid->integral = -pid->maxIntegral;
  // 计算输出
  pid->output = pout + dout + iout;
  // 输出限幅
  if (pid->output > pid->maxOutput)
    pid->output = pid->maxOutput;
  else if (pid->output < -pid->maxOutput)
    pid->output = -pid->maxOutput;
}

float emaFilter(float input, float *prev_ema, float alpha)	//ema滤波
{
  // 计算新的 EMA �?
  *prev_ema = alpha * input + (1.0f - alpha) * (*prev_ema);
  return *prev_ema;
}
