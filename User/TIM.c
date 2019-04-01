#include "TIM.h"

void TIM_NVIC_Config(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = TIM6_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

void TIM6_Config(void)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	RCC_APB1PeriphClockCmd(TIM6_IRQn, ENABLE);
	TIM_TimeBaseStructure.TIM_Period=1000;
	TIM_TimeBaseStructure.TIM_Prescaler=71;
	TIM_TimeBaseInit(TIM6,&TIM_TimeBaseStructure);
	TIM_ClearFlag(TIM6,TIM_IT_Update);
	TIM_ITConfig(TIM6,TIM_IT_Update,ENABLE);
	TIM_Cmd(TIM6,DISABLE);
}

void BASIC_TIM_Config(void)
{
	TIM6_Config();
	TIM_NVIC_Config();
}
