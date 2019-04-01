#ifndef __MOTION_CTRL_H
#define __MOTION_CTRL_H


#include "stdint.h"
#include "PID.h"


typedef struct
{
	float v1;
	float v2;
	float v3;

}wheel_speed;

/* Public  macro -------------------------------------------------------------*/
#define Reduction_Ratio		(3591.0/187.0)	//电机减速比
#define Max_RPM				469			//电机额定转速（单位：RPM）

#define PID_I_OUT_MAX     	1000		//角度环积分限幅
#define PID_ANG_OUT_MAX     5000     	//角度闭环的输出最大值_224°/s（单位：轮子转速，RPM；换算关系：ω = ((out/60）*2π*R_Wheel))/R_Chassis <rad/s>）

#define PID_CURRENT_I_MAX 	8000		//电流环积分限幅
#define PID_CURRENT_OUT_MAX 13384	    //电流闭环的输出最大值（电调范围是-16384~16384->-20A~20A）


/* Public  variables ----------------------*/
extern struct PID pidData_Current;					//电机电流——外环PID量						//底盘角度——内环PID量


/**		参量值变动		**/
void PIDInit(void);



#endif

/******************* (C) COPYRIGHT 2017 YANGYIFAN *****END OF FILE****/
