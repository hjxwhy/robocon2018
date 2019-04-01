#include "stm32f10x.h"
#include "M3508.h"
#include "can.h"
#include "PID.h"

short Real_C_Value[3] = {0};      //电机真实电流值
short Real_V_Value[3] = {0};      //电机真实速度值
long  Real_A_Value[3] = {0};       //电机真实角度值
short id=0x200;

//在运行工程主函数前给pid赋值	例PIDparamUpdate(&pidData_C[i],50,50,0,50000,100000);

//使用示例：M35_driver(0x200,(short)pidData_C[0].out,(short)pidData_C[1].out,(short) pidData_C[2].out,500);
 /**
  * @brief  发送驱动电流值(1ms调用一次为宜)
  * @param  id:电机组ID，一个ID可以管理4个电调，ID可以是0x200和0x1FF(现在用的是0x200)
  * @param  Currentx:给电机x的电流，x可以是1,2,3,4（按预设的顺序）
  * @retval 无
  */
void M3508SetCurrent(uint16_t id, int16_t Current1, int16_t Current2, int16_t Current3, int16_t Current4)
{
	unsigned short can_id = id;
	CanTxMsg tx_message;
    
	tx_message.IDE = CAN_ID_STD;    //标准帧
	tx_message.RTR = CAN_RTR_DATA;  //数据帧
	tx_message.DLC = 0x08;          //帧长度为8
	 
	 if((can_id==0x200)||(can_id==0x1ff))
	{
		tx_message.StdId = can_id;      //帧ID为传入参数的CAN_ID
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


