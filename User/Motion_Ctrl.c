#include "Motion_Ctrl.h"
#include "stm32f10x.h"
#include <stdint.h>
#include "M3508.h"
#include <Math.h>

/* Private  macro -------------------------------------------------------------*/
#define LimitOutput(in, low, high) ((in)<(low)?(low):((in)>(high)?(high):(in)))	//����޷�
#define OutLessNum(in1,in2,out1,out2) ((in1)<(in2)?(out1):(out2))				//�Ƚϴ�С,ѡ�����
#define ANGLE_TO_RAD(x) (x)/180.0f*3.14159f
#define RAD_TO_ANGLE(x) (x)/3.14159f*180.0f

#define R_Chassis			407			//�������Բ�뾶��һ�����ӵ��������ĵľ��룩,��λ:mm
#define R_Wheel				76			//ȫ���ְ뾶,��λ��mm

#define Pai 				3.1415926535f

/* Public  variables ---------------------------------------------------------*/
struct PID pidData_Current;					//������������⻷PID��
const float MaxSpeed = 5 * (Max_RPM/60) * 2 * Pai * R_Wheel * 0.866f / 3.0;	//0.866��cos(30��)��ֵ�����ʽ�����ٶȷֽ����,�������ĵ�λ��mm/s
const float MaxAccelerate = MaxSpeed / 0.5;	//�ٶ������������1s���������ȫ��


/* Private  variables ---------------------------------------------------------*/
wheel_speed speed_out;				//�ֽ��������ת��ֵ


/***			******			�ڲ����õĺ���			******					***/


/**
  * @brief  ����PID
  * @param  Speedx:�����x�ĵ�����x������1,2,3,4����Ԥ���˳��
  * @retval None
  */
void CurrentLoop(int32_t Speed) 
{
	
	//��ȡĿ��ֵ
	pidData_Current.set = Speed;
	
	//��ô��������ݣ�ʵʱ�ٶȣ�
	pidData_Current.feedback = Real_V_Value[0]*187/3591;

	//����PID����
	PIDdataUpdate(&(pidData_Current));
	
	//����PID
	GetPID(&(pidData_Current));

	M3508SetCurrent(0x200, (short)pidData_Current.out, 0,0,0);
}




/************************ (C) COPYRIGHT 2017 YANGYIFAN *****END OF FILE****/
