
#include "stm32f10x.h"

//******************* Stdlib ************************//
#include "math.h"
#include <stdio.h>

//******************* SYSTEM ************************//
#include "delay.h"
#include "sys.h"
#include "maincycle.h"
#include "pid.h"

//******************* Middle ************************//
#include "ccd.h"
#include "mpu.h"
#include "pwm.h"
#include "trans.h"
#include "oled.h"

//******************* Button ************************//
#include "gpio.h"
#include "ic.h"
#include "uart.h"

//***************************************************//


/*
			MPU6050*4	oledspi
			
			|			|
			|			|			uart		oc			ic			gpio
			|			|
			|			|			|			|			|			|			
			|			|			|			|			|			|
															|			|
ccd			mpu			oled		trans		pwm			|			|
															|			|
|			|			|			|			|			|			|
|			|			|			|			|			|			|

*/