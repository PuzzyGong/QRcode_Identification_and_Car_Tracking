#include "pwm.h"
#include "sys.h"

static float pwm1 = 0;
static float pwm2 = 0;

void straight_set(float f1, float f2)
{
    pwm1 = f1;
	pwm2 = f2;
	
	if      (pwm1 > 7000)  pwm1 = 7000;
    else if (pwm1 < -7000) pwm1 = -7000;
    if      (pwm2 > 7000)  pwm2 = 7000;
    else if (pwm2 < -7000) pwm2 = -7000;

	if(pwm1 < 0) PBout(12) = 0, PBout(13) = 1, TIM_SetCompare4(TIM1, (uint16_t)pwm1);
    else         PBout(12) = 1, PBout(13) = 0, TIM_SetCompare4(TIM1, (uint16_t)pwm1);
	if(pwm2 < 0) PBout(15) = 0, PBout(14) = 1, TIM_SetCompare1(TIM1, (uint16_t)pwm2);
    else         PBout(15) = 1, PBout(14) = 0, TIM_SetCompare1(TIM1, (uint16_t)pwm2);
}

void straight_add(float f1, float f2)
{
	pwm1 += f1;
    pwm2 += f2;

    if      (pwm1 > 7000)  pwm1 = 7000;
    else if (pwm1 < -7000) pwm1 = -7000;
    if      (pwm2 > 7000)  pwm2 = 7000;
    else if (pwm2 < -7000) pwm2 = -7000;

	if(pwm1 < 0) PBout(12) = 0, PBout(13) = 1, TIM_SetCompare4(TIM1, (uint16_t)pwm1);
    else         PBout(12) = 1, PBout(13) = 0, TIM_SetCompare4(TIM1, (uint16_t)pwm1);
	if(pwm2 < 0) PBout(15) = 0, PBout(14) = 1, TIM_SetCompare1(TIM1, (uint16_t)pwm2);
    else         PBout(15) = 1, PBout(14) = 0, TIM_SetCompare1(TIM1, (uint16_t)pwm2);
}
