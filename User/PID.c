#include "stm32f10x.h"
#include "PID.h"

 /**
  * @brief  PID参数初始化
  * @param  struct PID* pidData：待初始化的PID结构体变量
	*					uint16_t kp：P参数
	*					uint16_t ki：I参数
	*					uint16_t kd：D参数
	*					int32_t imax：积分限幅
	*					int32_t outmax：输出限幅
  * @retval 无
  */
  
 void PID_INIT(struct PID *pidData,float kp,float ki,float kd,int32_t imax,int32_t outmax)
 {
	 pidData->Imax = imax;
	 pidData->outmax = outmax;
	 pidData->P = 0;
	 pidData->I = 0;
	 pidData->D = 0;
	 pidData->kP = kp;
	 pidData->kI = ki;
	 pidData->kD = kd;
	 pidData->error = 0;
	 pidData->last_error = 0;
	 pidData->derror = 0;
	 pidData->set = 0;
	 pidData->out = 0;
 }
 
/**	 
  *@breif  每个周期的参数更新
  *@param stuct PID *pidData : PID结构体变量
  *@retval 无
  */
 void PIDdataUpdate(struct PID *pidData)
 {
	 pidData->error = pidData->set - pidData->feedback;
	 pidData->derror = pidData->error-pidData->derror;
	 pidData->last_error = pidData->error;
	 
	 pidData->P = pidData->kP*pidData->error;
	 
	 pidData->I += pidData->kI*pidData->error;
	 if(pidData->I >= pidData->Imax )
	 {
		 pidData->I = pidData->Imax;
	 }
	 	 if(pidData->I <= -pidData->Imax )
	 {
		 pidData->I = -pidData->Imax;
	 }
	 
	 pidData->D = pidData->kD*pidData->derror;
	 
 }
 
 /**	 
  *@breif 清空积分
  *@param stuct PID *pidData : PID结构体变量
  *@retval 无
  */
void ClearI(struct PID *pidData)
{
	pidData->I = 0;
}

 /**	 
  *@breif 获得PID的计算结果
  *@param stuct PID *pidData : PID结构体变量
  *@retval 无
  */
void GetPID(struct PID *pidData)
{
	pidData->out = pidData->P + pidData->I + pidData->D;
	if(pidData->out>= pidData->outmax)
	{
		pidData->out=pidData->outmax;
	}
	if(pidData->out <= (-pidData->outmax))
	{
		pidData->out=-pidData->outmax;
	}
}

/**	 
  *@breif 获得PI的计算结果
  *@param stuct PID *pidData : PID结构体变量
  *@retval 无
  */
void GetPI(struct PID *pidData)
{
	pidData->out = pidData->P + pidData->I;
	if(pidData->out >= pidData->outmax)
	{
		pidData->out=pidData->outmax;
	}
	if(pidData->out <= (-pidData->outmax))
	{
		pidData->out=-pidData->outmax;
	}
}

/**	 
  *@breif 获得PD的计算结果
  *@param stuct PID *pidData : PID结构体变量
  *@retval 无
  */
void GetPD(struct PID *pidData)
{
	pidData->out = pidData->P + pidData->D;
	if(pidData->out>= pidData->outmax)
	{
		pidData->out=pidData->outmax;
	}
	if(pidData->out <= (-pidData->outmax))
	{
		pidData->out=-pidData->outmax;
	}
}


/**	 
  *@breif 清除PID中的变量
  *@param stuct PID *pidData : PID结构体变量
  *@retval 无
  */

void PID_Clear(struct PID *pidData)
{
	pidData->error = 0;
	pidData->derror = 0;
	pidData->last_error = 0;
}
