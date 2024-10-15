#include "stm32f4xx.h"
#include "PID.h"

PID Mypid = {0};
CascadePID mypid = {0}; //创建串级PID结构体变�?
AngleStructDef angle_update = {0};

//用于初始化pid参数的函�?
void PID_Init(PID *pid, float p, float i, float d, float maxI, float maxOut)
{
    pid->kp = p;
    pid->ki = i;
    pid->kd = d;
    pid->maxIntegral = maxI;
    pid->maxOutput = maxOut;
}
 
//进行�?次pid计算
//参数�?(pid结构�?,目标�?,反馈�?)，计算结果放在pid结构体的output成员成员�?
void PID_Calc(PID *pid, float reference, float feedback)
{
 	//更新数据
    pid->lastError = pid->error; //将旧error存起�?
    pid->error = reference - feedback; //计算新error
    //计算微分
    static float dout;
		dout = (pid->error - pid->lastError) * pid->kd;
    //计算比例
    static float pout;
		pout	= pid->error * pid->kp;
    //计算积分
    pid->integral += pid->error;
	  static float iout;
		iout = pid->integral* pid->ki;
    //积分限幅
    if(pid->integral > pid->maxIntegral) pid->integral = pid->maxIntegral;
    else if(pid->integral < -pid->maxIntegral) pid->integral = -pid->maxIntegral;
    //计算输出
    pid->output = pout + dout + iout;
    //输出限幅
    if(pid->output > pid->maxOutput) pid->output =   pid->maxOutput;
    else if(pid->output < -pid->maxOutput) pid->output = -pid->maxOutput;
}


//串级PID的结构体，包含两个单级PID

//串级PID的计算函�?
//参数(PID结构�?,外环目标�?,外环反馈�?,内环反馈�?)
void PID_CascadeCalc(CascadePID *pid, float outerRef, float outerFdb, float innerFdb)
{
    PID_Calc(&pid->outer, outerRef, outerFdb); //计算外环
    PID_Calc(&pid->inner, pid->outer.output, innerFdb); //计算内环
    pid->output = pid->inner.output; //内环输出就是串级PID的输�?
}

void updata_angle(AngleStructDef *__angle,uint16_t new_encoder)
{
  __angle->last_encoder = __angle->encoder;		//上一次刻度值等于当前刻度值
  __angle->encoder = new_encoder;
  float resulte =  __angle->encoder - __angle->last_encoder;	//做差
  if(resulte < -4096)
  {
    __angle -> round ++;		//如果小于-4096，则为正转，圈数加一
  }
  else if(resulte >4096)
  {
     __angle -> round --;
  }
  __angle->finally_angle = __angle->round *360;		//如果大于4096，则为反转，圈数减一
}
