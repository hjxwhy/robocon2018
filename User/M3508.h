#ifndef _M3508_H_
#define _M3508_H_

#include "stm32f10x.h"

extern short Real_C_Value[3];
extern short Real_V_Value[3];
extern long Real_A_Value[3];
extern short id;

void M3508SetCurrent(uint16_t id, int16_t Current1, int16_t Current2, int16_t Current3, int16_t Current4)
;

#endif 
