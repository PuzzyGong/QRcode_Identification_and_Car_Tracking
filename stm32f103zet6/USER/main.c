#include "include.h"

PID v1_pid;
PID v2_pid;
PID angle_pid;
PID s_pid;

int main(void)
{	
	//***********************************************************************************************************//
	//***********************************************************************************************************//
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable ,ENABLE);//禁用JTAG 启用 SWD
	delay_init();
	
	//***********************************************************************************************************//
	//***********************************************************************************************************//

	trans_bt_init(2, 115200);

	move_init();

	ic_init(1);
	ic_init(2);

	PID_init(&v1_pid,    0, 0, 0, 1);
	PID_init(&v2_pid,    0, 0, 0, 1);
	PID_init(&angle_pid, 0, 0, 0, 1);
	PID_init(&s_pid,     0, 0, 0, 1);

	maincycle_ms_init(10);

	while(1)
	{
		;		
	}
}


void Maincycle_Handler()
{
	static int cnt = 0;
	int i = 0, j = 0, k = 0;
	
	//***********************************************************************************************************//
	//***********************************************************************************************************//
	
	//***************************************************//
	//轮速度
	static float P_v = 0, I_v = 0, D_v = 0;
	static float a_v1 = 0, a_v2= 0;
	static float a_v_max = 0;
	
	static float v1 = 0,   v2 = 0;

	static float pwm1 = 0, pwm2 = 0;

	//***************************************************//
	//角偏（小车指向目标物体的方向为x轴正向）
	static float P_angle = 0, I_angle = 0, D_angle = 0;
	static float a_angle = 0;

	static float angle = 0;
	static float angle_min_control = 0;

	//右轮大于左轮的速度
	static float a_vd = 0;

	//***************************************************//
	//距离（小车离目标物体的距离）
	static float P_s = 0, I_s = 0, D_s = 0;
	static float a_s = 0;

	static float s = 0;
	static float s_min_control = 0;

	//两轮的共同速度
	static float a_vc = 0;

	//***************************************************//
	float a_v1_tmp = 0; float a_v2_tmp = 0;
	
	//***********************************************************************************************************//
	//***********************************************************************************************************//
	CPUoccupationRate_Calculatestart();
	TIM_ClearITPendingBit(TIM6, TIM_IT_Update);
	cnt = (cnt >= 99) ? 0 : cnt + 1;
	
	//***********************************************************************************************************//
	//***********************************************************************************************************//

    //----- R_data
	if(trans_bt_R(2, 0, 0, 0, &P_v,     &I_v,     &D_v,     &a_v_max,           0,
							  &P_angle, &I_angle, &D_angle, &angle_min_control, 0,
							  &P_s,     &I_s,     &D_s,     &s_min_control,     &a_s,
						      0, 0, 0, 0, 0,
							  0, 0, 0, 0, 0,
							  0, 0, 0, 0, 0 ))
	{
        ;
	}
    
	//----- sensors
    v1 = ic_getdata(1) + 1;
	v2 = ic_getdata(2) + 1;
	
	//----- pid
	
	//pid_1
	if(angle > angle_min_control || angle < - angle_min_control)
	    a_vd = PID_calc1(&angle_pid, angle, a_angle);
	
	if(s > s_min_control || s < -s_min_control)
	    a_vc = PID_calc1(&s_pid, s, a_s);
	
    //pid_1_to_2
	
	
	
	//pid_2
	pwm1 = PID_calc2(&v1_pid, v1, a_v1);
	pwm2 = PID_calc2(&v2_pid, v2, a_v2);
	
	//pid_end
	straight_add(pwm1, pwm2, 0, 0);
	spin_add(0, 2800);


	if(cnt % 2 == 0)
		trans_bt_T(2, 0, 0, 0, v1,      v2,       a_v1,     a_v2,       0,
	                           pwm1,    pwm2,     CPUoccupationRate_Calculatefinish(), 0, 0,
						       0, 0, 0, 0, 0,
						       0, 0, 0, 0, 0,
							   0, 0, 0, 0, 0,
							   0, 0, 0, 0, 0 );
	
}
