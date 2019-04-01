#include "can.h"
CanRxMsg  CAN_Rece_Data;


#ifdef DEBUG
	
	uint8_t Testflag = 0;
	CanTxMsg  CAN_Tran_Data;
#endif


/**
  * @brief  CAN外设GPIO初始化
  * @param  None
  * @retval None
  */
static void CAN_GPIO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	
	RCC_APB2PeriphClockCmd(CAN_GPIO_CLK | RCC_APB2Periph_AFIO,ENABLE);
	
	//初始化CAN_TX
	GPIO_InitStruct.GPIO_Pin = CAN_TX_GPIO_PIN;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	
	GPIO_Init(CAN_TX_GPIO_PROT, &GPIO_InitStruct);
	
	//初始化CAN_RX
	GPIO_InitStruct.GPIO_Pin = CAN_RX_GPIO_PIN;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	
	GPIO_Init(CAN_RX_GPIO_PORT, &GPIO_InitStruct);
}

/**
  * @brief  CAN外设模式初始化
  * @param  None
  * @retval None
  */
static void CAN_Mode_Init(void)
{
	CAN_InitTypeDef CAN_InitStruct;
	
	/************************CAN通信参数设置**********************************/
	/* Enable CAN clock */
	RCC_APB1PeriphClockCmd(CAN_CLK, ENABLE);

	/*CAN寄存器初始化*/
	CAN_DeInit(CAN1);
	CAN_StructInit(&CAN_InitStruct);

	/*CAN单元初始化*/
	CAN_InitStruct.CAN_TTCM=DISABLE;			  	//MCR-TTCM  关闭时间触发通信模式使能
	CAN_InitStruct.CAN_ABOM=DISABLE;			   	//MCR-ABOM  关闭自动离线管理 
	CAN_InitStruct.CAN_AWUM=DISABLE;			   	//MCR-AWUM  不使用自动唤醒模式
	CAN_InitStruct.CAN_NART=DISABLE;			   	//MCR-NART  关闭报文自动重传	  ENABLE-使能自动重传
	CAN_InitStruct.CAN_RFLM=DISABLE;			   	//MCR-RFLM  接收FIFO 锁定模式  DISABLE-溢出时新报文会覆盖原有报文  
	CAN_InitStruct.CAN_TXFP=ENABLE;			   		//MCR-TXFP  发送FIFO优先级 		ENABLE-优先级取决于信息存入邮箱的顺序 DISABLE-优先级取决于报文标示符 

#ifdef DEBUG
	CAN_InitStruct.CAN_Mode = CAN_Mode_Silent_LoopBack; //回环静默模式
#else
	CAN_InitStruct.CAN_Mode = CAN_Mode_Normal;//正常工作模式
#endif
	
	CAN_InitStruct.CAN_SJW=CAN_SJW_1tq;		   	//BTR-SJW 重新同步跳跃宽度 1个时间单元
	 
	/* ss=1 bs1=5 bs2=3 位时间宽度为(1+5+3) 波特率即为时钟周期tq*(1+3+6)  */
	CAN_InitStruct.CAN_BS1=CAN_BS1_5tq;		   	//BTR-TS1 时间段1 占用了6个时间单元
	CAN_InitStruct.CAN_BS2=CAN_BS2_3tq;		   	//BTR-TS1 时间段2 占用了3个时间单元	
	
	/* CAN Baudrate = 1 MBps (1MBps已为stm32的CAN最高速率) (CAN 时钟频率为 APB 1 = 45 MHz) */
	CAN_InitStruct.CAN_Prescaler =4;		   		//BTR-BRP 波特率分频器  定义了时间单元的时间长度 36/(1+5+3)/4=1 Mbps
	CAN_Init(CAN1, &CAN_InitStruct);
	
}

/**
  * @brief  CAN外设筛选器初始化
  * @param  None
  * @retval None
  */
static void CAN_Filter_Init(void)
{
	CAN_FilterInitTypeDef  CAN_FilterInitStructure;

	/*CAN筛选器初始化*/
	CAN_FilterInitStructure.CAN_FilterNumber=0;								//筛选器组0
	CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdMask;			//工作在掩码模式
	CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_32bit;			//筛选器位宽为单个32位。
	
	/* 使能筛选器，按照标志的内容进行比对筛选，扩展ID不是如下的就抛弃掉，是的话，会存入FIFO0。 */
	
	//要筛选的ID高位 
	CAN_FilterInitStructure.CAN_FilterIdHigh= 0x0000;							//照单全收，不过滤	
	//要筛选的ID低位 
	CAN_FilterInitStructure.CAN_FilterIdLow= 0x0000;												 							
	CAN_FilterInitStructure.CAN_FilterMaskIdHigh = 0x0000;						//全部不用匹配
	CAN_FilterInitStructure.CAN_FilterMaskIdLow = 0x0000;						//全部不用匹配
	CAN_FilterInitStructure.CAN_FilterFIFOAssignment = CAN_Filter_FIFO0 ;		//筛选器被关联到FIFO0
	CAN_FilterInitStructure.CAN_FilterActivation = ENABLE;						//使能筛选器
	CAN_FilterInit(&CAN_FilterInitStructure);
	
	/*CAN通信中断使能*/
	CAN_ITConfig(CAN1, CAN_IT_FMP0, ENABLE);
	CAN_ITConfig(CAN1,CAN_IT_TME,DISABLE);
}

/**
  * @brief  CAN外设NVIC初始化(期望它产生接收中断而不是通过轮询的方式接收数据)
  * @param  None
  * @retval None
  */
static void CAN_NVIC_Init(void)
{
		NVIC_InitTypeDef NVIC_InitStructure;
		
	 /*中断设置*/
	NVIC_InitStructure.NVIC_IRQChannel = USB_LP_CAN1_RX0_IRQn;	   	//CAN1 RX0中断
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;		//抢占优先级0
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;			   	//响应优先级为0
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
		
	NVIC_InitStructure.NVIC_IRQChannel = USB_HP_CAN1_TX_IRQn;	   	//CAN1 TX中断
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;		 //抢占优先级0
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;			   	//响应优先级为0
    NVIC_InitStructure.NVIC_IRQChannelCmd = DISABLE;
    NVIC_Init(&NVIC_InitStructure);
}

/**
  * @brief  CAN外设初始化
  * @param  None
  * @retval None
  */
void CAN_Config(void)
{
	CAN_GPIO_Init();
	CAN_NVIC_Init();
	CAN_Mode_Init();
	CAN_Filter_Init();
	
}



/**
  * @brief  初始化 Rx Message数据结构体
					（目的是格式化数据接收结构体变量中的残余数据，防止产生干扰）
  * @param  RxMessage: 指向要初始化的数据结构体
  * @retval None
  */
void Init_RxMes(CanRxMsg *RxMessage)
{
  uint8_t ubCounter = 0;

	/*把接收结构体清零*/
  RxMessage->StdId = 0x00;
  RxMessage->ExtId = 0x00;
  RxMessage->IDE = CAN_ID_STD;
  RxMessage->DLC = 0;
  RxMessage->FMI = 0;
  for (ubCounter = 0; ubCounter < 8; ubCounter++)
  {
    RxMessage->Data[ubCounter] = 0x00;
  }
}

#ifdef DEBUG

/*
 * 函数名：CAN_SetMsg
 * 描述  ：CAN通信报文内容设置,设置一个数据内容为0-7的数据包(调试用)
 * 输入  ：发送报文结构体
 * 输出  : 无
 * 调用  ：外部调用
 */	 
void CAN_SetMsg(CanTxMsg *TxMessage)
{	  
	uint8_t ubCounter = 0;

  TxMessage->StdId=STATE_DATA_TEMP;			//使用的标准ID
  //TxMessage->ExtId=0x1314;				//使用的扩展ID
  TxMessage->IDE=CAN_ID_STD;				//标准模式
  TxMessage->RTR=CAN_RTR_DATA;				//发送的是数据
  TxMessage->DLC=8;							//数据长度为8字节
	
	/*设置要发送的数据0-7*/
	for (ubCounter = 1; ubCounter < 8; ubCounter++)
  {
    TxMessage->Data[ubCounter] = ubCounter;
  }
}

#endif


