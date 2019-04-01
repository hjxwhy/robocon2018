#ifndef _CAN_H_
#define _CAN_H_

#include "stm32f10x.h"


#define STATE_DATA_TEMP 0x02B								//接收到的状态数据范例（用于设置掩码）

#define CAN_TX_GPIO_PROT  		GPIOA
#define CAN_TX_GPIO_PIN   		GPIO_Pin_12

#define  CAN_RX_GPIO_PORT    	GPIOA
#define  CAN_RX_GPIO_PIN      	GPIO_Pin_11

#define CAN_GPIO_CLK			RCC_APB2Periph_GPIOA

#define CAN_CLK					RCC_APB1Periph_CAN1


/*信息输出*/
#define CAN_DEBUG_ON         1

#define CAN_INFO(fmt,arg...)           printf("<<-CAN-INFO->> "fmt"\n",##arg)
#define CAN_ERROR(fmt,arg...)          printf("<<-CAN-ERROR->> "fmt"\n",##arg)
#define CAN_DEBUG(fmt,arg...)          do{\
                                          if(CAN_DEBUG_ON)\
                                          printf("<<-CAN-DEBUG->> [%d]"fmt"\n",__LINE__, ##arg);

extern CanRxMsg  CAN_Rece_Data;  

void CAN_Config(void);
void Init_RxMes(CanRxMsg *RxMessage);

#ifdef DEBUG

extern CanTxMsg  CAN_Tran_Data;	
																					
void CAN_SetMsg(CanTxMsg *TxMessage);																					

#endif
																					
#endif

