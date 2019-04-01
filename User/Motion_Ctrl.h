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
#define Reduction_Ratio		(3591.0/187.0)	//������ٱ�
#define Max_RPM				469			//����ת�٣���λ��RPM��

#define PID_I_OUT_MAX     	1000		//�ǶȻ������޷�
#define PID_ANG_OUT_MAX     5000     	//�Ƕȱջ���������ֵ_224��/s����λ������ת�٣�RPM�������ϵ���� = ((out/60��*2��*R_Wheel))/R_Chassis <rad/s>��

#define PID_CURRENT_I_MAX 	8000		//�����������޷�
#define PID_CURRENT_OUT_MAX 13384	    //�����ջ���������ֵ�������Χ��-16384~16384->-20A~20A��


/* Public  variables ----------------------*/
extern struct PID pidData_Current;					//������������⻷PID��						//���̽Ƕȡ����ڻ�PID��


/**		����ֵ�䶯		**/
void PIDInit(void);



#endif

/******************* (C) COPYRIGHT 2017 YANGYIFAN *****END OF FILE****/
