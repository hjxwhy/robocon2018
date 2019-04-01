#ifndef __PID_H_
#define __PID_H_

#include "stm32f10x.h"


struct PID
{
	float kP;
	float kI;
	float kD;
	int32_t P;
	int32_t I;
	int32_t D;
	int32_t set;
	int32_t feedback;
	int32_t last_error;
	int32_t error;
	int32_t derror;
	int32_t Imax;
	int32_t outmax;
	int32_t out;
};

void PID_INIT(struct PID* pidData,float kp,float ki,float kd,int32_t imax,int32_t outmax);
void PIDdataUpdate(struct PID* pidData);
void ClearI(struct PID *pidData);
void GetPID(struct PID* pidData);
void GetPI(struct PID* pidData);
void GetPD(struct PID* pidData);
void PID_Clear(struct PID *pidData);
#endif

