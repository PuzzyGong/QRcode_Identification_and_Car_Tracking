#include "pid.h"

void PID_init(PID* pid, float P, float I, float D, float max_output, char zero)
{
    pid->P = P;
	pid->D = D;
	pid->I = I;
	pid->max_output = max_output;
	
	if(zero == 1)
	{
		pid->LastError = 0;
		pid->PrevError = 0;
		pid->Integral  = 0;	
	}
}

float PID_calc1(PID* pid, float NowPoint, float SetPoint)
{
    float iError, output; 
	iError = SetPoint - NowPoint;

	pid->Integral += iError;
	
	output = pid->P * ( iError ) + 
	         pid->I * ( pid->Integral ) + 
	         pid->D * ( iError - pid->LastError );
	
	pid->LastError = iError;
	
	if     (output >  pid->max_output) output =  pid->max_output;
	else if(output < -pid->max_output) output = -pid->max_output;
    return  output;
}

float PID_calc2(PID* pid, float NowPoint, float SetPoint)
{
    float iError, output;
	iError = SetPoint - NowPoint;
	
	output = pid->P * ( iError - pid->LastError ) + 
             pid->I * ( iError ) + 
             pid->D * ( iError - 2 * pid->LastError + pid->PrevError );
	
	pid->PrevError = pid->LastError;
	pid->LastError = iError;
	
	if     (output >  pid->max_output) output =  pid->max_output;
	else if(output < -pid->max_output) output = -pid->max_output;	
    return  output;
}
