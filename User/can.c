#include "can.h"
CanRxMsg  CAN_Rece_Data;


#ifdef DEBUG
	
	uint8_t Testflag = 0;
	CanTxMsg  CAN_Tran_Data;
#endif


/**
  * @brief  CAN����GPIO��ʼ��
  * @param  None
  * @retval None
  */
static void CAN_GPIO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	
	RCC_APB2PeriphClockCmd(CAN_GPIO_CLK | RCC_APB2Periph_AFIO,ENABLE);
	
	//��ʼ��CAN_TX
	GPIO_InitStruct.GPIO_Pin = CAN_TX_GPIO_PIN;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	
	GPIO_Init(CAN_TX_GPIO_PROT, &GPIO_InitStruct);
	
	//��ʼ��CAN_RX
	GPIO_InitStruct.GPIO_Pin = CAN_RX_GPIO_PIN;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	
	GPIO_Init(CAN_RX_GPIO_PORT, &GPIO_InitStruct);
}

/**
  * @brief  CAN����ģʽ��ʼ��
  * @param  None
  * @retval None
  */
static void CAN_Mode_Init(void)
{
	CAN_InitTypeDef CAN_InitStruct;
	
	/************************CANͨ�Ų�������**********************************/
	/* Enable CAN clock */
	RCC_APB1PeriphClockCmd(CAN_CLK, ENABLE);

	/*CAN�Ĵ�����ʼ��*/
	CAN_DeInit(CAN1);
	CAN_StructInit(&CAN_InitStruct);

	/*CAN��Ԫ��ʼ��*/
	CAN_InitStruct.CAN_TTCM=DISABLE;			  	//MCR-TTCM  �ر�ʱ�䴥��ͨ��ģʽʹ��
	CAN_InitStruct.CAN_ABOM=DISABLE;			   	//MCR-ABOM  �ر��Զ����߹��� 
	CAN_InitStruct.CAN_AWUM=DISABLE;			   	//MCR-AWUM  ��ʹ���Զ�����ģʽ
	CAN_InitStruct.CAN_NART=DISABLE;			   	//MCR-NART  �رձ����Զ��ش�	  ENABLE-ʹ���Զ��ش�
	CAN_InitStruct.CAN_RFLM=DISABLE;			   	//MCR-RFLM  ����FIFO ����ģʽ  DISABLE-���ʱ�±��ĻḲ��ԭ�б���  
	CAN_InitStruct.CAN_TXFP=ENABLE;			   		//MCR-TXFP  ����FIFO���ȼ� 		ENABLE-���ȼ�ȡ������Ϣ���������˳�� DISABLE-���ȼ�ȡ���ڱ��ı�ʾ�� 

#ifdef DEBUG
	CAN_InitStruct.CAN_Mode = CAN_Mode_Silent_LoopBack; //�ػ���Ĭģʽ
#else
	CAN_InitStruct.CAN_Mode = CAN_Mode_Normal;//��������ģʽ
#endif
	
	CAN_InitStruct.CAN_SJW=CAN_SJW_1tq;		   	//BTR-SJW ����ͬ����Ծ��� 1��ʱ�䵥Ԫ
	 
	/* ss=1 bs1=5 bs2=3 λʱ����Ϊ(1+5+3) �����ʼ�Ϊʱ������tq*(1+3+6)  */
	CAN_InitStruct.CAN_BS1=CAN_BS1_5tq;		   	//BTR-TS1 ʱ���1 ռ����6��ʱ�䵥Ԫ
	CAN_InitStruct.CAN_BS2=CAN_BS2_3tq;		   	//BTR-TS1 ʱ���2 ռ����3��ʱ�䵥Ԫ	
	
	/* CAN Baudrate = 1 MBps (1MBps��Ϊstm32��CAN�������) (CAN ʱ��Ƶ��Ϊ APB 1 = 45 MHz) */
	CAN_InitStruct.CAN_Prescaler =4;		   		//BTR-BRP �����ʷ�Ƶ��  ������ʱ�䵥Ԫ��ʱ�䳤�� 36/(1+5+3)/4=1 Mbps
	CAN_Init(CAN1, &CAN_InitStruct);
	
}

/**
  * @brief  CAN����ɸѡ����ʼ��
  * @param  None
  * @retval None
  */
static void CAN_Filter_Init(void)
{
	CAN_FilterInitTypeDef  CAN_FilterInitStructure;

	/*CANɸѡ����ʼ��*/
	CAN_FilterInitStructure.CAN_FilterNumber=0;								//ɸѡ����0
	CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdMask;			//����������ģʽ
	CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_32bit;			//ɸѡ��λ��Ϊ����32λ��
	
	/* ʹ��ɸѡ�������ձ�־�����ݽ��бȶ�ɸѡ����չID�������µľ����������ǵĻ��������FIFO0�� */
	
	//Ҫɸѡ��ID��λ 
	CAN_FilterInitStructure.CAN_FilterIdHigh= 0x0000;							//�յ�ȫ�գ�������	
	//Ҫɸѡ��ID��λ 
	CAN_FilterInitStructure.CAN_FilterIdLow= 0x0000;												 							
	CAN_FilterInitStructure.CAN_FilterMaskIdHigh = 0x0000;						//ȫ������ƥ��
	CAN_FilterInitStructure.CAN_FilterMaskIdLow = 0x0000;						//ȫ������ƥ��
	CAN_FilterInitStructure.CAN_FilterFIFOAssignment = CAN_Filter_FIFO0 ;		//ɸѡ����������FIFO0
	CAN_FilterInitStructure.CAN_FilterActivation = ENABLE;						//ʹ��ɸѡ��
	CAN_FilterInit(&CAN_FilterInitStructure);
	
	/*CANͨ���ж�ʹ��*/
	CAN_ITConfig(CAN1, CAN_IT_FMP0, ENABLE);
	CAN_ITConfig(CAN1,CAN_IT_TME,DISABLE);
}

/**
  * @brief  CAN����NVIC��ʼ��(���������������ж϶�����ͨ����ѯ�ķ�ʽ��������)
  * @param  None
  * @retval None
  */
static void CAN_NVIC_Init(void)
{
		NVIC_InitTypeDef NVIC_InitStructure;
		
	 /*�ж�����*/
	NVIC_InitStructure.NVIC_IRQChannel = USB_LP_CAN1_RX0_IRQn;	   	//CAN1 RX0�ж�
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;		//��ռ���ȼ�0
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;			   	//��Ӧ���ȼ�Ϊ0
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
		
	NVIC_InitStructure.NVIC_IRQChannel = USB_HP_CAN1_TX_IRQn;	   	//CAN1 TX�ж�
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;		 //��ռ���ȼ�0
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;			   	//��Ӧ���ȼ�Ϊ0
    NVIC_InitStructure.NVIC_IRQChannelCmd = DISABLE;
    NVIC_Init(&NVIC_InitStructure);
}

/**
  * @brief  CAN�����ʼ��
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
  * @brief  ��ʼ�� Rx Message���ݽṹ��
					��Ŀ���Ǹ�ʽ�����ݽ��սṹ������еĲ������ݣ���ֹ�������ţ�
  * @param  RxMessage: ָ��Ҫ��ʼ�������ݽṹ��
  * @retval None
  */
void Init_RxMes(CanRxMsg *RxMessage)
{
  uint8_t ubCounter = 0;

	/*�ѽ��սṹ������*/
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
 * ��������CAN_SetMsg
 * ����  ��CANͨ�ű�����������,����һ����������Ϊ0-7�����ݰ�(������)
 * ����  �����ͱ��Ľṹ��
 * ���  : ��
 * ����  ���ⲿ����
 */	 
void CAN_SetMsg(CanTxMsg *TxMessage)
{	  
	uint8_t ubCounter = 0;

  TxMessage->StdId=STATE_DATA_TEMP;			//ʹ�õı�׼ID
  //TxMessage->ExtId=0x1314;				//ʹ�õ���չID
  TxMessage->IDE=CAN_ID_STD;				//��׼ģʽ
  TxMessage->RTR=CAN_RTR_DATA;				//���͵�������
  TxMessage->DLC=8;							//���ݳ���Ϊ8�ֽ�
	
	/*����Ҫ���͵�����0-7*/
	for (ubCounter = 1; ubCounter < 8; ubCounter++)
  {
    TxMessage->Data[ubCounter] = ubCounter;
  }
}

#endif


