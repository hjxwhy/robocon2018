#include "stm32f10x.h"
#include "shooting_ctrl.h"
#include "can.h"
#include "M3508.h"

/*------------初始化参数宏-----------*/
#define Pp  		0.049
#define Pi  		0
#define Pd  		0
#define Vp  		190
#define Vi  		0.42
#define Vd  		0


#define K  			0.04
#define K2  			0.04
#define K3           0.0051    //柔性控制斜率
#define SHOOTING_ANGLE1  			3000  //恒速阶段的qc数
#define SPEEDCHANGE_ANGLE1  			2000   //加速阶段qc
#define SPEEDCHANGE_ANGLE2  			3000   //减速
#define SPEEDCHANGE_ANGLE3  			1593   //柔性控制变速阶段
#define MAX_SPEED  			200   //匀速阶段转速

/*-----------globe variable------------*/
struct PID VelPID;
struct PID PosPID;
struct PID pidData_Current;	

float   K4=MAX_SPEED/(SPEEDCHANGE_ANGLE2*SPEEDCHANGE_ANGLE2);
uint16_t timer;
int16_t rpm;
int16_t out_speed;
int16_t position;
int16_t origin_position;  //位置原点，为准备发射的位置
long real_position;


uint8_t stop_flag;	//置1开始停止过程
uint8_t safe_flag;  //置1禁止运动
uint8_t position_flag;  //位置模式的标识
uint8_t shoot_flag;  //位置模式的标识
uint8_t send_flag; //准许向串口发送标识
uint8_t shoot_flag_save; //每次发射前清积分操作

/**	 
  *@breif PID参数设置
  *@param  
  *@retval 无
  */
void PID_Config(void)
{
	PID_INIT(&pidData_Current, Vp, Vi, Vd, PID_CURRENT_I_MAX, PID_CURRENT_OUT_MAX);   //-----此处注意PWM限幅---outmax
	PID_INIT(&PosPID,Pp,Pi,Pd,LIMIT_POSITION_SUM,LIMITVEL);
}



/*--------------------PID控制-------------------------*/

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


///**	 
//  *@breif 速度闭环，输出电流--------待定，可能改为微分先行形式
//  *@param  无
//  *@retval 无
//  */
//void Velocity_Loop(void)
//{
//	VelPID.set=PosPID.out;
//	VelPID.feedback=Real_V_Value[0];
//	PIDdataUpdate(&VelPID);
//	GetPID(&VelPID);
//	CAN_RoboModule_DRV_Current_Mode(0,1,5000,VelPID.out);
//	
//}

/**	 
  *@breif 位置闭环，输出速度
  *@param  无
  *@retval 无
  */
void Position_Loop(void)
{
	shoot_flag=1;
	PosPID.set=position;
	PosPID.feedback=real_position;
	PIDdataUpdate(&PosPID);
	GetPID(&PosPID);
	out_speed=PosPID.out;
}
  

/**	 
  *@breif 清除一系列pid运算量
  *@param  无
  *@retval 无
  */
void ClearAllPidPara(void)
{

	PID_Clear(&PosPID);
	ClearI(&PosPID);
	
}


/**	 
  *@breif 梯形运动过程
  *@breif 后期改为五段s型曲线实现柔性控制
  *@param 无
  *@retval 无
  */
void motion_process2()  //--和process1互斥
{
	position_flag=1;   //再次确保和位置模式互斥
	if(real_position-origin_position<-100)
		out_speed=0;
	if(real_position-origin_position<=SPEEDCHANGE_ANGLE1&&
		real_position-origin_position>=-100)  //处于加速阶段
	{
		out_speed=K*(real_position-origin_position)+10;
	}
	
	if(real_position-origin_position<=(SHOOTING_ANGLE1+SPEEDCHANGE_ANGLE1)&&
		real_position-origin_position>=SPEEDCHANGE_ANGLE1)                          //处于匀速阶段
	{
		out_speed=MAX_SPEED+10;
	}
	
	if(real_position-origin_position>=(SHOOTING_ANGLE1+SPEEDCHANGE_ANGLE1)&&
		real_position-origin_position<(SPEEDCHANGE_ANGLE1+SPEEDCHANGE_ANGLE2+SHOOTING_ANGLE1)) //处于减速阶段
	{
		out_speed=MAX_SPEED-K2*(real_position
			-origin_position-SPEEDCHANGE_ANGLE1-SHOOTING_ANGLE1)+10;
	}
	
	if(real_position-origin_position>=(SPEEDCHANGE_ANGLE1+SPEEDCHANGE_ANGLE2+SHOOTING_ANGLE1)) //------后期此块改成终点闭环
	{
		//safe_flag=1;   //运动结束，安全位置一
		shoot_flag=1;
		//out_speed=0;
		position=origin_position+SPEEDCHANGE_ANGLE2+SPEEDCHANGE_ANGLE1+SHOOTING_ANGLE1+100;
		position_flag=0;
		//Position_Loop();
		//timer=0; //终点闭环计时开始
		send_flag=0;
	}
}

/**	 
  *@breif 五段S型柔性控制---有错误
  *@param 无
  *@retval 无
  */
void motion_process3()  //--和process1，2互斥
{
	position_flag=1;   //再次确保和位置模式互斥
	
	if(real_position-origin_position<-50)
		out_speed=0;
	
	if(real_position-origin_position<=0.5*SPEEDCHANGE_ANGLE3&&
		real_position-origin_position>=-50)  //处于加速1阶段
	{
		out_speed=K3*(real_position-origin_position)
			*(real_position-origin_position)+10;
	}
	
	if(real_position-origin_position<=SPEEDCHANGE_ANGLE3&&
		real_position-origin_position>=0.5*SPEEDCHANGE_ANGLE3)  //处于加速2阶段
	{
		out_speed=-K3*(real_position-origin_position-SPEEDCHANGE_ANGLE3)
			*(real_position-origin_position-SPEEDCHANGE_ANGLE3)+MAX_SPEED+10;	
	}
	
	if(real_position-origin_position<=(SHOOTING_ANGLE1+SPEEDCHANGE_ANGLE3)&&
		real_position-origin_position>=SPEEDCHANGE_ANGLE3)                          //处于匀速阶段
	{
		out_speed=MAX_SPEED+10;
	}
	
	if(real_position-origin_position>=(SHOOTING_ANGLE1+SPEEDCHANGE_ANGLE3)&&
		real_position-origin_position<(1.5*SPEEDCHANGE_ANGLE3+SHOOTING_ANGLE1)) //处于减速1阶段
	{
		out_speed=-K3*(real_position-origin_position-SPEEDCHANGE_ANGLE3-SHOOTING_ANGLE1)
			*(real_position-origin_position-SPEEDCHANGE_ANGLE3-SHOOTING_ANGLE1)+MAX_SPEED+10;
	}
	
	if(real_position-origin_position>=(SHOOTING_ANGLE1+1.5*SPEEDCHANGE_ANGLE3)&&
		real_position-origin_position<(2*SPEEDCHANGE_ANGLE3+SHOOTING_ANGLE1)) //处于减速2阶段
	{
		out_speed=K3*(real_position-origin_position-2*SPEEDCHANGE_ANGLE3-SHOOTING_ANGLE1)
			*(real_position-origin_position-2*SPEEDCHANGE_ANGLE3-SHOOTING_ANGLE1)+10;
	}
	
	if(real_position-origin_position>=(2*SPEEDCHANGE_ANGLE3+SHOOTING_ANGLE1)) //------后期此块改成终点闭环
	{
		//safe_flag=1;   //运动结束，安全位置一
		shoot_flag=1;
		//out_speed=0;
		position=origin_position+2*SPEEDCHANGE_ANGLE3+SHOOTING_ANGLE1+100;
		position_flag=0;
		send_flag=0;
	}
}



/**	 
  *@breif 三段半柔性控制 
  *@breif 后期改为五段s型曲线实现柔性控制
  *@param 无
  *@retval 无
  */
void motion_process4()  //--和process1互斥
{
	position_flag=1;   //再次确保和位置模式互斥
	if(shoot_flag!=shoot_flag_save)  //当上次不是在发射模式，这次在发射模式时，清积分
			ClearI(&pidData_Current);
	if(real_position-origin_position<-100)
		out_speed=0;
	if(real_position-origin_position<=SPEEDCHANGE_ANGLE1&&
		real_position-origin_position>=-100)  //处于加速阶段
	{
		//out_speed=-K4*(real_position-origin_position-SPEEDCHANGE_ANGLE1)*(real_position-origin_position-SPEEDCHANGE_ANGLE1)+MAX_SPEED+10;
		out_speed=MAX_SPEED+10;
	}
	
	if(real_position-origin_position<=(SHOOTING_ANGLE1+SPEEDCHANGE_ANGLE1)&&
		real_position-origin_position>=SPEEDCHANGE_ANGLE1)                          //处于匀速阶段
	{
		out_speed=MAX_SPEED+10;
	}
	
	if(real_position-origin_position>=(SHOOTING_ANGLE1+SPEEDCHANGE_ANGLE1)&&
		real_position-origin_position<(SPEEDCHANGE_ANGLE1+SPEEDCHANGE_ANGLE2+SHOOTING_ANGLE1)) //处于减速阶段
	{
		out_speed=-K4*(real_position-origin_position-SPEEDCHANGE_ANGLE1-SHOOTING_ANGLE1)*(real_position-origin_position-SPEEDCHANGE_ANGLE1-SHOOTING_ANGLE1)+MAX_SPEED+10;
	}
	
	if(real_position-origin_position>=(SPEEDCHANGE_ANGLE1+SPEEDCHANGE_ANGLE2+SHOOTING_ANGLE1)) //------后期此块改成终点闭环
	{
		//safe_flag=1;   //运动结束，安全位置一
		shoot_flag=1;
		position=origin_position+SPEEDCHANGE_ANGLE2+SPEEDCHANGE_ANGLE1+SHOOTING_ANGLE1+100;
		position_flag=0;
		send_flag=0;
		ClearI(&pidData_Current);

	}
}


/**	 
  *@breif 安全保护，防止出现机械臂出现在不想出现的角度,也可以加速度限幅
  *@param 无
  *@retval 无
  */

//void motion_protection()
//{
////	if(real_position-origin_position<=0)  //当机械臂实际位置在原点位置的负值处时以一个慢慢的速度恢复到正处
////		CAN_RoboModule_DRV_Velocity_Mode(0,1,5000,100);
//	if(real_position-origin_position<=-4000)  //防止后期打到底部
//		CAN_RoboModule_DRV_Velocity_Mode(0,1,5000,0);
//}
		
		
	

