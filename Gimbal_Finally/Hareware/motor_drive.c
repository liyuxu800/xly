#include "stm32f4xx.h"                  // Device header
#include "motor_drive.h"

YawStructDef YawStruct = {0};
FWStructDef FWStruct = {0};
pitchStructDef pitchStruct = {0};
dialStructDef dialStruct = {0};

void FW_Motor_Control(FWStructDef *FWStruct)
{
  //左摩擦轮
    if (Can1_Receive_Judgment(0x203, FWStruct.RxData_L))
    {
      FWStruct.feedbackValue_L = PID_One_Calculation(&FWStruct.pid_one_L, FWStruct.targetValue_L, FWStruct.RxData_L);

      FWStruct.TxData_L[4] = (((int16_t)FWStruct.pid_one_L.output) >> 8) & 0xff; // 右移八位是因�????16位数据只有后面八位可以存�????8位的数组
      FWStruct.TxData_L[5] = ((int16_t)FWStruct.pid_one_L.output) & 0xff;
    }
    // 右摩擦轮
    if (Can1_Receive_Judgment(0x202, FWStruct.RxData_R))
    {
      FWStruct.feedbackValue_R = PID_One_Calculation(&FWStruct.pid_one_R, FWStruct.targetValue_R, FWStruct.RxData_R);

      FWStruct.TxData_R[2] = (((int16_t)FWStruct.pid_one_R.output) >> 8) & 0xff; // 右移八位是因�????16位数据只有后面八位可以存�????8位的数组
      FWStruct.TxData_R[3] = ((int16_t)FWStruct.pid_one_R.output) & 0xff;
    }
}

void Dial_Motor_Control(dialStructDef dialStruct)
{
  if (Can1_Receive_Judgment(0x201, dialStruct.RxData))
    {
      dialStruct.encoder = (dialStruct.RxData[0] << 8) | dialStruct.RxData[1];
      dialStruct.Angle = dialStruct.encoder * 360.0f / 8192.0f;
      dialStruct.Speed = (dialStruct.RxData[2] << 8) | dialStruct.RxData[3];

      dialStruct.outerFeedback = dialStruct.Angle;
      dialStruct.innerFeedback = dialStruct.Speed;                                                                      // 获取内环反馈�???
      PID_CascadeCalc(&dialStruct.pid_two, dialStruct.outerTarget, dialStruct.outerFeedback, dialStruct.innerFeedback); // 进行PID计算

      dialStruct.TxData[0] = (((int16_t)dialStruct.pid_two.output) >> 8) & 0xff; // 右移八位是因�???16位数据只有后面八位可以存�???8位的数组
      dialStruct.TxData[1] = ((int16_t)dialStruct.pid_two.output) & 0xff;
    }
}

void Yaw_Motor_Control(YawStructDef YawStruct)
{
 if (Can2_Receive_Judgment(0x209, YawStruct.RxData))
    {
      // YawStruct.encoder = (YawStruct.RxData[0] << 8) | YawStruct.RxData[1];
      YawStruct.Angle = mpu_yaw;
      YawStruct.Speed = (YawStruct.RxData[2] << 8) | YawStruct.RxData[3];

      YawStruct.outerFeedback = YawStruct.Angle;
      YawStruct.innerFeedback = YawStruct.Speed;                                                                    // 获取内环反馈�???
      PID_CascadeCalc(&YawStruct.pid_two, YawStruct.outerTarget, YawStruct.outerFeedback, YawStruct.innerFeedback); // 进行PID计算

      YawStruct.TxData[0] = (((int16_t)YawStruct.pid_two.output) >> 8) & 0xff; // 右移八位是因�???16位数据只有后面八位可以存�???8位的数组
      YawStruct.TxData[1] = ((int16_t)YawStruct.pid_two.output) & 0xff;
    }
}

void Pitch_Motor_Control(pitchStructDef pitchStruct)
{
   if (Can1_Receive_Judgment(0x206, pitchStruct.RxData))
    {
      pitchStruct.encoder = (pitchStruct.RxData[0] << 8) | pitchStruct.RxData[1];
      pitchStruct.Angle = pitchStruct.encoder * 360.0f / 8192.0f;
      pitchStruct.Speed = (pitchStruct.RxData[2] << 8) | pitchStruct.RxData[3];

      pitchStruct.outerFeedback = pitchStruct.Angle;
      pitchStruct.innerFeedback = pitchStruct.Speed;                                                                        // 获取内环反馈�???
      PID_CascadeCalc(&pitchStruct.pid_two, pitchStruct.outerTarget, pitchStruct.outerFeedback, pitchStruct.innerFeedback); // 进行PID计算

      pitchStruct.TxData[2] = (((int16_t)pitchStruct.pid_two.output) >> 8) & 0xff; // 右移八位是因�???16位数据只有后面八位可以存�???8位的数组
      pitchStruct.TxData[3] = ((int16_t)pitchStruct.pid_two.output) & 0xff;
    }
}
