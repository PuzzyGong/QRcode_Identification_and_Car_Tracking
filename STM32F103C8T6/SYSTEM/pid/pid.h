#ifndef _PID_H_
#define _PID_H_

#include "stm32f10x.h"

typedef struct{
    float P;
	float I;
	float D;
	float max_output;
	
    float LastError;
	float PrevError;
	float Integral;
	
}PID;

void PID_init(PID* pid, float P, float I, float D, float max_output, char zero);
float PID_calc1(PID* pid, float NowPoint, float SetPoint);
float PID_calc2(PID* pid, float NowPoint, float SetPoint);

#endif
