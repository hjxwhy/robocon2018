#include "stm32f10x.h"
#include "PID.h"

 /**
  * @brief  PID������ʼ��
  * @param  struct PID* pidData������ʼ����PID�ṹ�����
	*					uint16_t kp��P����
	*					uint16_t ki��I����
	*					uint16_t kd��D����
	*					int32_t imax�������޷�
	*					int32_t outmax������޷�
  * @retval ��
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
  *@breif  ÿ�����ڵĲ�������
  *@param stuct PID *pidData : PID�ṹ�����
  *@retval ��
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
  *@breif ��ջ���
  *@param stuct PID *pidData : PID�ṹ�����
  *@retval ��
  */
void ClearI(struct PID *pidData)
{
	pidData->I = 0;
}

 /**	 
  *@breif ���PID�ļ�����
  *@param stuct PID *pidData : PID�ṹ�����
  *@retval ��
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
  *@breif ���PI�ļ�����
  *@param stuct PID *pidData : PID�ṹ�����
  *@retval ��
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
  *@breif ���PD�ļ�����
  *@param stuct PID *pidData : PID�ṹ�����
  *@retval ��
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
  *@breif ���PID�еı���
  *@param stuct PID *pidData : PID�ṹ�����
  *@retval ��
  */

void PID_Clear(struct PID *pidData)
{
	pidData->error = 0;
	pidData->derror = 0;
	pidData->last_error = 0;
}
