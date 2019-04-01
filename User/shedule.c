#include "shedule.h"
#include "shooting_ctrl.h"
#include "can.h"
#include "USART.h"
#include "M3508.h"
#include "commiunication.h"
uint8_t count;
uint16_t t1;
uint16_t t2;
uint16_t t3;



int16_t fuction[2][600];   //��¼qc��rpm�Ķ�Ӧ��ϵ
int8_t real_v;

void foolish_delay(void)
{
	for(t3=0;t3<9999;t3++);
}

void frequency_2ms(void)  //��Ƶ��1msһ��  
{
	if(shoot_flag==0)   //������뷢��ģʽ����ʼ��¼
	{
		
		fuction[0][t2]=real_position;
		fuction[1][t2]=Real_V_Value[0]*187/3591;
		//fuction[1][t2]=-out_speed;
		t2++;
		if(t2==599)
			t2=0;
	}
}

void frequency_5ms(void)  //��Ƶ�� 5msһ��   
{
	//out_speed=0;      
	if(safe_flag==0)
	{
		if(shoot_flag==0)
		{
			motion_process4();
		}
		if(position_flag==0)
		{
			Position_Loop();
		}
	}
	//if(out_speed>300) out_speed=300;
	//if(out_speed<-300) out_speed=-300;
	CurrentLoop(-out_speed);
	shoot_flag_save=shoot_flag;
	
}

void frequency_10ms(void)  
{
	if(send_flag==0)  //����ģʽ��������ʼ��д
	{
		if(t1>600)	
		printf("%d\n",fuction[1][t1-600]);	
		//printf("%d\n",out_speed);			
		if(t1<600)
		printf("%d\n",fuction[0][t1]);
		//printf("%ld\n",Real_A_Value[0]);
		if(t1==600)
		printf("\n\n\n\n");
		if(t1==1200)
		{
			send_flag=1;
			t1=0;
			position=origin_position;
			t2=0;
			
		}
		t1++;
	}
//	
//	if(shoot_flag==1&&send_flag==1)  //���Ե������ʹ�ã�����ɾ��
//	{	
//		real_v=(Real_V_Value[0]*187/3591);
//		oscilloscope(real_v);
//	}
//	
//	
//			
		
}

void frequency_20ms(void)  
{
		//Position_Loop();
}



void Shedulue(void)  //��Ƶ��Ҫ��
{	
	if(count%2==0)
		frequency_2ms();
	if(count%5==0)
		frequency_5ms();
	if(count%9==0)
		frequency_10ms();
	if(count%19==0)
		frequency_20ms();

	if(count==19) count=0;
	count++;
} 

