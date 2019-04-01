/**
  ******************************************************************************
  * @file    Project/STM32F10x_StdPeriph_Template/stm32f10x_it.c 
  * @author  MCD Application Team
  * @version V3.5.0
  * @date    08-April-2011
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and 
  *          peripherals interrupt service routine.
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_it.h"
#include "can.h"
#include "shedule.h"
#include "string.h"
#include "M3508.h"
#include "shooting_ctrl.h"
unsigned char can_tx_success_flag=0;
int16_t last_position;
int8_t cnt;

/** @addtogroup STM32F10x_StdPeriph_Template
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M3 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief  This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
{
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
}

/**
  * @brief  This function handles PendSVC exception.
  * @param  None
  * @retval None
  */
void PendSV_Handler(void)
{
}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
void SysTick_Handler(void)
{
}





//本接收数据的函数，默认为4个驱动器，都挂在0组，编号为1、2、3、4
/*************************************************************************
                          USB_LP_CAN1_RX0_IRQHandler
描述：CAN1的接收中断函数
*************************************************************************/
void USB_LP_CAN1_RX0_IRQHandler(void) //CAN RX
{
    	Init_RxMes(&CAN_Rece_Data);									//??????????
	
	if (CAN_GetITStatus(CAN1,CAN_IT_FMP0)!= RESET)
	{
        CAN_ClearITPendingBit(CAN1, CAN_IT_FMP0);
        CAN_Receive(CAN1, CAN_FIFO0, &CAN_Rece_Data);
	
        if((CAN_Rece_Data.IDE == CAN_Id_Standard)&&(CAN_Rece_Data.IDE == CAN_RTR_Data)&&(CAN_Rece_Data.DLC == 8)) //?????????????8
        {
            if(CAN_Rece_Data.StdId == 0x201)        //??1?????
            {
                Real_A_Value[0] = (CAN_Rece_Data.Data[0]<<8)|(CAN_Rece_Data.Data[1]);  //???
                Real_V_Value[0] = (CAN_Rece_Data.Data[2]<<8)|(CAN_Rece_Data.Data[3]);  //???
                Real_C_Value[0] = ((CAN_Rece_Data.Data[4]<<8)|(CAN_Rece_Data.Data[5]));//???
            }
            else if(CAN_Rece_Data.StdId == 0x202)   //??2?????
            {
                Real_A_Value[1] = (CAN_Rece_Data.Data[0]<<8)|(CAN_Rece_Data.Data[1]);  //???
                Real_V_Value[1] = (CAN_Rece_Data.Data[2]<<8)|(CAN_Rece_Data.Data[3]);  //???
                Real_C_Value[1] = ((CAN_Rece_Data.Data[4]<<8)|(CAN_Rece_Data.Data[5]));//???
            }
            else if(CAN_Rece_Data.StdId == 0x203)   //??3?????
            {
                Real_A_Value[2] = (CAN_Rece_Data.Data[0]<<8)|(CAN_Rece_Data.Data[1]);  //???
                Real_V_Value[2] = (CAN_Rece_Data.Data[2]<<8)|(CAN_Rece_Data.Data[3]);  //???
                Real_C_Value[2] = ((CAN_Rece_Data.Data[4]<<8)|(CAN_Rece_Data.Data[5]));//???
            }
		}
		if(Real_A_Value[0]-last_position<=-4000) cnt+=1;
		if(Real_A_Value[0]-last_position>4000) cnt-=1;
		last_position=Real_A_Value[0];
		real_position=-(int)((Real_A_Value[0]+cnt*8191)/19.2032);
		
    }
}
void TIM6_IRQHandler(void)
{
	if (TIM_GetITStatus(TIM6,TIM_IT_Update) != RESET)
	{
		Shedulue();
	}
	//TIM_ClearFlag(TIM6,TIM_FLAG_Update);
	TIM_ClearITPendingBit(TIM6,TIM_FLAG_Update);
}


void USART1_IRQHandler(void)
{
	if(USART_GetITStatus(USART1,USART_IT_RXNE)!=RESET)
	{
	}
}


void USART2_IRQHandler(void)		   //串口2全局中断服务函数
{
   if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)  //接收中断有效,若接收数据寄存器满
     {
			 
	 }
}


/******************************************************************************/
/*                 STM32F10x Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f10x_xx.s).                                            */
/******************************************************************************/

/**
  * @brief  This function handles PPP interrupt request.
  * @param  None
  * @retval None
  */
/*void PPP_IRQHandler(void)
{
}*/

/**
  * @}
  */ 


/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
