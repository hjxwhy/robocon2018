#ifndef __SHOOTING_CTRL_H
#define __SHOOTING_CTRL_H

#include "PID.h"
#include "math.h"

/*-----------����޷�---------------*/ 

#define LIMITPWM 25000
#define LIMITVEL 100
#define LIMIT_POSITION_SUM 100
#define LIMIT_VEL_SUM 50000
#define PID_CURRENT_I_MAX 	8000		//�����������޷�
#define PID_CURRENT_OUT_MAX 16384	    //�����ջ���������ֵ�������Χ��-16384~16384->-20A~20A��

extern struct PID CurrentPID;
extern struct PID VelPID;
extern struct PID PosPID;
extern int16_t origin_position;
extern uint8_t safe_flag;  //��1��ֹ�˶�
extern uint8_t position_flag;  //λ��ģʽ�ı�ʶ
extern uint8_t shoot_flag;  //λ��ģʽ�ı�ʶ
extern int16_t out_speed;
extern int16_t position;
extern uint8_t send_flag;
extern uint8_t shoot_flag_save; //ÿ�η���ǰ����ֲ���
extern long real_position;


void PID_Config(void);
void CurrentLoop(int32_t Speed);
void Position_Loop(void);
void ClearAllPidPara(void);
void motion_process2(void);
void motion_process3(void); 
void motion_process4(void);
void motion_protection(void);

#endif /*__SHOOTING_CTRL_H*/
