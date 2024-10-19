#include "stm32f4xx.h" // Device header
#include "Calculation.h"
#include "DR16_control.h"

void Finally_Calculation(RC_Ctl_t RC_CtrlData)
{
	RC_CtrlData.rc.ch0 -= 1024;
	finally_rc_ch0 = RC_CtrlData.rc.ch0 / 132.0f;

	RC_CtrlData.rc.ch1 -= 1024;
	finally_rc_ch1 = RC_CtrlData.rc.ch1 / 132.0f;

	RC_CtrlData.rc.ch2 -= 1024;
	finally_rc_ch2 = RC_CtrlData.rc.ch2 / 132.0f;

	RC_CtrlData.rc.ch3 -= 1024;
	finally_rc_ch3 = RC_CtrlData.rc.ch3 / 132.0f;
}
