#ifndef __SHOOTING_CTRL_H
#define __SHOOTING_CTRL_H

#include "PID.h"
#include "math.h"

/*-----------输出限幅---------------*/ 

#define LIMITPWM 25000
#define LIMITVEL 100
#define LIMIT_POSITION_SUM 100
#define LIMIT_VEL_SUM 50000
#define PID_CURRENT_I_MAX 	8000		//电流环积分限幅
#define PID_CURRENT_OUT_MAX 16384	    //电流闭环的输出最大值（电调范围是-16384~16384->-20A~20A）

extern struct PID CurrentPID;
extern struct PID VelPID;
extern struct PID PosPID;
extern int16_t origin_position;
extern uint8_t safe_flag;  //置1禁止运动
extern uint8_t position_flag;  //位置模式的标识
extern uint8_t shoot_flag;  //位置模式的标识
extern int16_t out_speed;
extern int16_t position;
extern uint8_t send_flag;
extern uint8_t shoot_flag_save; //每次发射前清积分操作
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
