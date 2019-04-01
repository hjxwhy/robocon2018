#include "stm32f10x.h"   
#include "SysTick.h"
#include "can.h"
#include "TIM.h"
#include "shooting_ctrl.h"
#include "USART.h"
int main(void)
{
 	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	BASIC_TIM_Config();

	CAN_Config();   
	SysTick_Delay_Ms(500);	//CAN1初始化
	SysTick_Delay_Ms(500);
	USART_Config();
	//USART2_Config();
	PID_Config();
	//safe_flag=1;
	origin_position=-2600;  //暂时的
	position=origin_position;
	shoot_flag=1;
	position_flag=0;
	send_flag=1;
	TIM_Cmd(TIM6,ENABLE);
	while(1);
}


