#include "stm32f10x.h"
#include "shooting_ctrl.h"
#include "can.h"
#include "M3508.h"

/*------------��ʼ��������-----------*/
#define Pp  		0.049
#define Pi  		0
#define Pd  		0
#define Vp  		190
#define Vi  		0.42
#define Vd  		0


#define K  			0.04
#define K2  			0.04
#define K3           0.0051    //���Կ���б��
#define SHOOTING_ANGLE1  			3000  //���ٽ׶ε�qc��
#define SPEEDCHANGE_ANGLE1  			2000   //���ٽ׶�qc
#define SPEEDCHANGE_ANGLE2  			3000   //����
#define SPEEDCHANGE_ANGLE3  			1593   //���Կ��Ʊ��ٽ׶�
#define MAX_SPEED  			200   //���ٽ׶�ת��

/*-----------globe variable------------*/
struct PID VelPID;
struct PID PosPID;
struct PID pidData_Current;	

float   K4=MAX_SPEED/(SPEEDCHANGE_ANGLE2*SPEEDCHANGE_ANGLE2);
uint16_t timer;
int16_t rpm;
int16_t out_speed;
int16_t position;
int16_t origin_position;  //λ��ԭ�㣬Ϊ׼�������λ��
long real_position;


uint8_t stop_flag;	//��1��ʼֹͣ����
uint8_t safe_flag;  //��1��ֹ�˶�
uint8_t position_flag;  //λ��ģʽ�ı�ʶ
uint8_t shoot_flag;  //λ��ģʽ�ı�ʶ
uint8_t send_flag; //׼���򴮿ڷ��ͱ�ʶ
uint8_t shoot_flag_save; //ÿ�η���ǰ����ֲ���

/**	 
  *@breif PID��������
  *@param  
  *@retval ��
  */
void PID_Config(void)
{
	PID_INIT(&pidData_Current, Vp, Vi, Vd, PID_CURRENT_I_MAX, PID_CURRENT_OUT_MAX);   //-----�˴�ע��PWM�޷�---outmax
	PID_INIT(&PosPID,Pp,Pi,Pd,LIMIT_POSITION_SUM,LIMITVEL);
}



/*--------------------PID����-------------------------*/

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


///**	 
//  *@breif �ٶȱջ����������--------���������ܸ�Ϊ΢��������ʽ
//  *@param  ��
//  *@retval ��
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
  *@breif λ�ñջ�������ٶ�
  *@param  ��
  *@retval ��
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
  *@breif ���һϵ��pid������
  *@param  ��
  *@retval ��
  */
void ClearAllPidPara(void)
{

	PID_Clear(&PosPID);
	ClearI(&PosPID);
	
}


/**	 
  *@breif �����˶�����
  *@breif ���ڸ�Ϊ���s������ʵ�����Կ���
  *@param ��
  *@retval ��
  */
void motion_process2()  //--��process1����
{
	position_flag=1;   //�ٴ�ȷ����λ��ģʽ����
	if(real_position-origin_position<-100)
		out_speed=0;
	if(real_position-origin_position<=SPEEDCHANGE_ANGLE1&&
		real_position-origin_position>=-100)  //���ڼ��ٽ׶�
	{
		out_speed=K*(real_position-origin_position)+10;
	}
	
	if(real_position-origin_position<=(SHOOTING_ANGLE1+SPEEDCHANGE_ANGLE1)&&
		real_position-origin_position>=SPEEDCHANGE_ANGLE1)                          //�������ٽ׶�
	{
		out_speed=MAX_SPEED+10;
	}
	
	if(real_position-origin_position>=(SHOOTING_ANGLE1+SPEEDCHANGE_ANGLE1)&&
		real_position-origin_position<(SPEEDCHANGE_ANGLE1+SPEEDCHANGE_ANGLE2+SHOOTING_ANGLE1)) //���ڼ��ٽ׶�
	{
		out_speed=MAX_SPEED-K2*(real_position
			-origin_position-SPEEDCHANGE_ANGLE1-SHOOTING_ANGLE1)+10;
	}
	
	if(real_position-origin_position>=(SPEEDCHANGE_ANGLE1+SPEEDCHANGE_ANGLE2+SHOOTING_ANGLE1)) //------���ڴ˿�ĳ��յ�ջ�
	{
		//safe_flag=1;   //�˶���������ȫλ��һ
		shoot_flag=1;
		//out_speed=0;
		position=origin_position+SPEEDCHANGE_ANGLE2+SPEEDCHANGE_ANGLE1+SHOOTING_ANGLE1+100;
		position_flag=0;
		//Position_Loop();
		//timer=0; //�յ�ջ���ʱ��ʼ
		send_flag=0;
	}
}

/**	 
  *@breif ���S�����Կ���---�д���
  *@param ��
  *@retval ��
  */
void motion_process3()  //--��process1��2����
{
	position_flag=1;   //�ٴ�ȷ����λ��ģʽ����
	
	if(real_position-origin_position<-50)
		out_speed=0;
	
	if(real_position-origin_position<=0.5*SPEEDCHANGE_ANGLE3&&
		real_position-origin_position>=-50)  //���ڼ���1�׶�
	{
		out_speed=K3*(real_position-origin_position)
			*(real_position-origin_position)+10;
	}
	
	if(real_position-origin_position<=SPEEDCHANGE_ANGLE3&&
		real_position-origin_position>=0.5*SPEEDCHANGE_ANGLE3)  //���ڼ���2�׶�
	{
		out_speed=-K3*(real_position-origin_position-SPEEDCHANGE_ANGLE3)
			*(real_position-origin_position-SPEEDCHANGE_ANGLE3)+MAX_SPEED+10;	
	}
	
	if(real_position-origin_position<=(SHOOTING_ANGLE1+SPEEDCHANGE_ANGLE3)&&
		real_position-origin_position>=SPEEDCHANGE_ANGLE3)                          //�������ٽ׶�
	{
		out_speed=MAX_SPEED+10;
	}
	
	if(real_position-origin_position>=(SHOOTING_ANGLE1+SPEEDCHANGE_ANGLE3)&&
		real_position-origin_position<(1.5*SPEEDCHANGE_ANGLE3+SHOOTING_ANGLE1)) //���ڼ���1�׶�
	{
		out_speed=-K3*(real_position-origin_position-SPEEDCHANGE_ANGLE3-SHOOTING_ANGLE1)
			*(real_position-origin_position-SPEEDCHANGE_ANGLE3-SHOOTING_ANGLE1)+MAX_SPEED+10;
	}
	
	if(real_position-origin_position>=(SHOOTING_ANGLE1+1.5*SPEEDCHANGE_ANGLE3)&&
		real_position-origin_position<(2*SPEEDCHANGE_ANGLE3+SHOOTING_ANGLE1)) //���ڼ���2�׶�
	{
		out_speed=K3*(real_position-origin_position-2*SPEEDCHANGE_ANGLE3-SHOOTING_ANGLE1)
			*(real_position-origin_position-2*SPEEDCHANGE_ANGLE3-SHOOTING_ANGLE1)+10;
	}
	
	if(real_position-origin_position>=(2*SPEEDCHANGE_ANGLE3+SHOOTING_ANGLE1)) //------���ڴ˿�ĳ��յ�ջ�
	{
		//safe_flag=1;   //�˶���������ȫλ��һ
		shoot_flag=1;
		//out_speed=0;
		position=origin_position+2*SPEEDCHANGE_ANGLE3+SHOOTING_ANGLE1+100;
		position_flag=0;
		send_flag=0;
	}
}



/**	 
  *@breif ���ΰ����Կ��� 
  *@breif ���ڸ�Ϊ���s������ʵ�����Կ���
  *@param ��
  *@retval ��
  */
void motion_process4()  //--��process1����
{
	position_flag=1;   //�ٴ�ȷ����λ��ģʽ����
	if(shoot_flag!=shoot_flag_save)  //���ϴβ����ڷ���ģʽ������ڷ���ģʽʱ�������
			ClearI(&pidData_Current);
	if(real_position-origin_position<-100)
		out_speed=0;
	if(real_position-origin_position<=SPEEDCHANGE_ANGLE1&&
		real_position-origin_position>=-100)  //���ڼ��ٽ׶�
	{
		//out_speed=-K4*(real_position-origin_position-SPEEDCHANGE_ANGLE1)*(real_position-origin_position-SPEEDCHANGE_ANGLE1)+MAX_SPEED+10;
		out_speed=MAX_SPEED+10;
	}
	
	if(real_position-origin_position<=(SHOOTING_ANGLE1+SPEEDCHANGE_ANGLE1)&&
		real_position-origin_position>=SPEEDCHANGE_ANGLE1)                          //�������ٽ׶�
	{
		out_speed=MAX_SPEED+10;
	}
	
	if(real_position-origin_position>=(SHOOTING_ANGLE1+SPEEDCHANGE_ANGLE1)&&
		real_position-origin_position<(SPEEDCHANGE_ANGLE1+SPEEDCHANGE_ANGLE2+SHOOTING_ANGLE1)) //���ڼ��ٽ׶�
	{
		out_speed=-K4*(real_position-origin_position-SPEEDCHANGE_ANGLE1-SHOOTING_ANGLE1)*(real_position-origin_position-SPEEDCHANGE_ANGLE1-SHOOTING_ANGLE1)+MAX_SPEED+10;
	}
	
	if(real_position-origin_position>=(SPEEDCHANGE_ANGLE1+SPEEDCHANGE_ANGLE2+SHOOTING_ANGLE1)) //------���ڴ˿�ĳ��յ�ջ�
	{
		//safe_flag=1;   //�˶���������ȫλ��һ
		shoot_flag=1;
		position=origin_position+SPEEDCHANGE_ANGLE2+SPEEDCHANGE_ANGLE1+SHOOTING_ANGLE1+100;
		position_flag=0;
		send_flag=0;
		ClearI(&pidData_Current);

	}
}


/**	 
  *@breif ��ȫ��������ֹ���ֻ�е�۳����ڲ�����ֵĽǶ�,Ҳ���Լ��ٶ��޷�
  *@param ��
  *@retval ��
  */

//void motion_protection()
//{
////	if(real_position-origin_position<=0)  //����е��ʵ��λ����ԭ��λ�õĸ�ֵ��ʱ��һ���������ٶȻָ�������
////		CAN_RoboModule_DRV_Velocity_Mode(0,1,5000,100);
//	if(real_position-origin_position<=-4000)  //��ֹ���ڴ򵽵ײ�
//		CAN_RoboModule_DRV_Velocity_Mode(0,1,5000,0);
//}
		
		
	

