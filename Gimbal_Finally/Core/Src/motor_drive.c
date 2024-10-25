#include "stm32f4xx.h" // Device header
#include "motor_drive.h"
#include "math.h"

float Pre_Target = 0.0f; // 前馈
float Pre_Now = 0.0f;
float f_out = 0.0f;
float K_F = 0;
double ttt = 0;
double mpu_roll_new = 0;

float Pre_Target_dial = 0.0f; // 前馈
float Pre_Now_dial = 0.0f;
float f_out_dial = 0.0f;
float K_F_dial = 0;

float a = -182.02;
float b = 2911.1303;

float aaa;

const double PI = 3.14159;

extern uint8_t output_protection;

// mpu
float mpu_pitch, mpu_roll, mpu_yaw;
float gx, gy, gz;

// 结构体
YawStructDef YawStruct = {0};
FWStructDef FWStruct = {0};
pitchStructDef pitchStruct = {0};
dialStructDef dialStruct = {0};

// 摩擦轮、拨盘发送数据包
uint8_t FW_dial_TxData[8] = {0};

void FW_Motor_Control(FWStructDef *FWStruct)
{
  // s1控制转速
  if (RC_CtrlData.rc.s1 == 1)
  {
    FWStruct->targetValue_L = -6000; // 左轮需要取反
    FWStruct->targetValue_R = 6000;
  }
  else
  {
    FWStruct->targetValue_L = 0;
    FWStruct->targetValue_R = 0;
  }

  // 左摩擦轮
  FWStruct->feedbackValue_L = PID_One_Calculation(&FWStruct->pid_one_L, FWStruct->targetValue_L, FWStruct->RxData_L);
  FWStruct->pid_one_L.output *= output_protection;
  FWStruct->TxData_L[4] = (((int16_t)FWStruct->pid_one_L.output) >> 8) & 0xff; // 右移八位是因�????16位数据只有后面八位可以存�????8位的数组
  FWStruct->TxData_L[5] = ((int16_t)FWStruct->pid_one_L.output) & 0xff;

  // 右摩擦轮
  FWStruct->feedbackValue_R = PID_One_Calculation(&FWStruct->pid_one_R, FWStruct->targetValue_R, FWStruct->RxData_R);
  FWStruct->pid_one_R.output *= output_protection;
  FWStruct->TxData_R[2] = (((int16_t)FWStruct->pid_one_R.output) >> 8) & 0xff; // 右移八位是因�????16位数据只有后面八位可以存�????8位的数组
  FWStruct->TxData_R[3] = ((int16_t)FWStruct->pid_one_R.output) & 0xff;
}

void Dial_Motor_Control(dialStructDef *dialStruct)
{

  dialStruct->encoder = (dialStruct->RxData[0] << 8) | dialStruct->RxData[1];
  dialStruct->Angle = dialStruct->encoder * 360.0f / 8192.0f;
  dialStruct->Speed = ((dialStruct->RxData[2] << 8) | dialStruct->RxData[3]);

  updata_angle_dial(&angle_update_dial, dialStruct->encoder);

  // 有减速比
  dialStruct->outerFeedback = ((dialStruct->Angle + (float)angle_update_dial.finally_angle) / 36.0f); // 获取外环反馈�?
  dialStruct->innerFeedback = ((float)dialStruct->Speed / 36.0f);                                     // 获取内环反馈�?

  PID_CascadeCalc(&dialStruct->pid_two, dialStruct->outerTarget, dialStruct->outerFeedback, dialStruct->innerFeedback); // 进行PID计算

  Pre_Target_dial = dialStruct->outerTarget;
  Pre_Now_dial = dialStruct->outerFeedback;

  dialStruct->pid_two.output += (K_F_dial * (Pre_Target_dial - Pre_Now_dial));

  dialStruct->pid_two.output *= output_protection;

  dialStruct->TxData[0] = (((int16_t)dialStruct->pid_two.output) >> 8) & 0xff; // 右移八位是因�???16位数据只有后面八位可以存�???8位的数组
  dialStruct->TxData[1] = ((int16_t)dialStruct->pid_two.output) & 0xff;
}

void Yaw_Motor_Control(YawStructDef *YawStruct)
{
  float fina_data = RC_CtrlData.rc.ch0;
  float finally_data = fina_data / 660.0f;

  YawStruct->Angle = mpu_yaw + 180.0f;
  YawStruct->Speed = (gz * 60.0f / 360.0f);

  updata_angle_yaw(&angle_update_yaw, YawStruct->Angle);

  YawStruct->outerFeedback = (YawStruct->Angle + (float)angle_update_yaw.finally_angle);
  YawStruct->innerFeedback = YawStruct->Speed;
  // YawStruct->outerTarget += finally_data;                                                                           // 获取内环反馈�???
  PID_CascadeCalc(&YawStruct->pid_two, YawStruct->outerTarget, YawStruct->outerFeedback, YawStruct->innerFeedback); // 进行PID计算

  YawStruct->pid_two.output *= output_protection;

  YawStruct->TxData[0] = (((int16_t)YawStruct->pid_two.output) >> 8) & 0xff; // 右移八位是因�???16位数据只有后面八位可以存�???8位的数组
  YawStruct->TxData[1] = ((int16_t)YawStruct->pid_two.output) & 0xff;
}

void Pitch_Motor_Control(pitchStructDef *pitchStruct)
{

  float fina_data = RC_CtrlData.rc.ch1;
  float finally_data = fina_data / 6600.0f;

  aaa = (pitchStruct->RxData[0] << 8) | pitchStruct->RxData[1];

  pitchStruct->Angle = mpu_roll;
  pitchStruct->Speed = gx;
  pitchStruct->outerFeedback = pitchStruct->Angle;
  pitchStruct->innerFeedback = pitchStruct->Speed;
  // pitchStruct->outerTarget += finally_data;

  if (pitchStruct->outerTarget >= 15)
  {
    pitchStruct->outerTarget = 15;
  }
  if (pitchStruct->outerTarget <= -15)
  {
    pitchStruct->outerTarget = -15;
  }
  // 获取内环反馈�???
  PID_CascadeCalc(&pitchStruct->pid_two, pitchStruct->outerTarget, pitchStruct->outerFeedback, pitchStruct->innerFeedback); // 进行PID计算

  Pre_Now = pitchStruct->Angle;
  Pre_Target = pitchStruct->outerTarget;

  f_out = (Pre_Target - Pre_Now) * K_F;
  f_out *= f_out;

  mpu_roll_new = mpu_roll * PI / 180.0f;

  //  ttt = (a * cos(mpu_roll_new) - b);

  ttt = a * mpu_roll - b;

  pitchStruct->pid_two.output += f_out;

  pitchStruct->pid_two.output += ttt;

  // 22300.76 * mpu_roll

  pitchStruct->pid_two.output *= output_protection;

  pitchStruct->TxData[2] = (((int16_t)pitchStruct->pid_two.output) >> 8) & 0xff; // 右移八位是因�???16位数据只有后面八位可以存�???8位的数组
  pitchStruct->TxData[3] = ((int16_t)pitchStruct->pid_two.output) & 0xff;
}
