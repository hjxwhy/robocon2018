#include "stm32f10x.h"
#include "M3508.h"
#include "can.h"
#include "PID.h"

short Real_C_Value[3] = {0};      //�����ʵ����ֵ
short Real_V_Value[3] = {0};      //�����ʵ�ٶ�ֵ
long  Real_A_Value[3] = {0};       //�����ʵ�Ƕ�ֵ
short id=0x200;

//�����й���������ǰ��pid��ֵ	��PIDparamUpdate(&pidData_C[i],50,50,0,50000,100000);

//ʹ��ʾ����M35_driver(0x200,(short)pidData_C[0].out,(short)pidData_C[1].out,(short) pidData_C[2].out,500);
 /**
  * @brief  ������������ֵ(1ms����һ��Ϊ��)
  * @param  id:�����ID��һ��ID���Թ���4�������ID������0x200��0x1FF(�����õ���0x200)
  * @param  Currentx:�����x�ĵ�����x������1,2,3,4����Ԥ���˳��
  * @retval ��
  */
void M3508SetCurrent(uint16_t id, int16_t Current1, int16_t Current2, int16_t Current3, int16_t Current4)
{
	unsigned short can_id = id;
	CanTxMsg tx_message;
    
	tx_message.IDE = CAN_ID_STD;    //��׼֡
	tx_message.RTR = CAN_RTR_DATA;  //����֡
	tx_message.DLC = 0x08;          //֡����Ϊ8
	 
	 if((can_id==0x200)||(can_id==0x1ff))
	{
		tx_message.StdId = can_id;      //֡IDΪ���������CAN_ID
	}
	else
	{
        return;
	}
	 
	tx_message.Data[0] = Current1 >> 8;
	tx_message.Data[1] = Current1;
	tx_message.Data[2] = Current2 >> 8;
	tx_message.Data[3] = Current2;
	tx_message.Data[4] = Current3 >> 8;
	tx_message.Data[5] = Current3;
	tx_message.Data[6] = Current4 >> 8;
	tx_message.Data[7] = Current4;

   CAN_Transmit(CAN1,&tx_message);
}


