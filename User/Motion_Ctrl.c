#include "Motion_Ctrl.h"
#include "stm32f10x.h"
#include <stdint.h>
#include "M3508.h"
#include <Math.h>

/* Private  macro -------------------------------------------------------------*/
#define LimitOutput(in, low, high) ((in)<(low)?(low):((in)>(high)?(high):(in)))	//输出限幅
#define OutLessNum(in1,in2,out1,out2) ((in1)<(in2)?(out1):(out2))				//比较大小,选择输出
#define ANGLE_TO_RAD(x) (x)/180.0f*3.14159f
#define RAD_TO_ANGLE(x) (x)/3.14159f*180.0f

#define R_Chassis			407			//底盘外接圆半径（一个轮子到底盘中心的距离）,单位:mm
#define R_Wheel				76			//全向轮半径,单位：mm

#define Pai 				3.1415926535f

/* Public  variables ---------------------------------------------------------*/
struct PID pidData_Current;					//电机电流——外环PID量
const float MaxSpeed = 5 * (Max_RPM/60) * 2 * Pai * R_Wheel * 0.866f / 3.0;	//0.866是cos(30°)的值，这个式子由速度分解得来,计算结果的单位是mm/s
const float MaxAccelerate = MaxSpeed / 0.5;	//假定带负载情况下1s从零加速至全速


/* Private  variables ---------------------------------------------------------*/
wheel_speed speed_out;				//分解后各电机的转速值


/***			******			内部调用的函数			******					***/


/**
  * @brief  电流PID
  * @param  Speedx:给电机x的电流，x可以是1,2,3,4（按预设的顺序）
  * @retval None
  */
void CurrentLoop(int32_t Speed) 
{
	
	//获取目标值
	pidData_Current.set = Speed;
	
	//获得传感器数据（实时速度）
	pidData_Current.feedback = Real_V_Value[0]*187/3591;

	//更新PID数据
	PIDdataUpdate(&(pidData_Current));
	
	//计算PID
	GetPID(&(pidData_Current));

	M3508SetCurrent(0x200, (short)pidData_Current.out, 0,0,0);
}




/************************ (C) COPYRIGHT 2017 YANGYIFAN *****END OF FILE****/
