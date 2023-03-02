#include "include.h"

PID v1_pid;
PID v2_pid;
PID angle_pid;
PID s_pid;

int main(void)
{	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable ,ENABLE);//禁用JTAG 启用 SWD
	delay_init();
	
    //*********|*********|*********|*********|*********|*********|*********|*********|*********|*********|**********
    //*********|*********|*********|*********|*********|*********|*********|*********|*********|*********|**********
    //********\|********\|********\|********\|********\|********\|********\|********\|********\|********\|**********	
    	
	//****************************** UART ******************************//

	trans_bt_init(3, 115200);

	//****************************** SENSORS ******************************//
	
	ic_init(1);
	ic_init(2);
	
	//****************************** STATE_AND_PID ******************************//
	
	PID_init(&v1_pid,    0, 0, 0, 0, 1);
	PID_init(&v2_pid,    0, 0, 0, 0, 1);
	PID_init(&angle_pid, 0, 0, 0, 0, 1);
	PID_init(&s_pid,     0, 0, 0, 0, 1);
	pwm_init();

    //****************************** OTHERS ******************************//

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
	
    //*********|*********|*********|*********|*********|*********|*********|*********|*********|*********|**********
    //*********|*********|*********|*********|*********|*********|*********|*********|*********|*********|**********
    //********\|********\|********\|********\|********\|********\|********\|********\|********\|********\|**********
	
	//****************************** STATE ******************************//
	
	enum{
		TEST__V_PID_ONLY,
	    MOVE,
		STILL
	}state = TEST__V_PID_ONLY;
	
	static float MOVE_to_STILL_delta_angle = 0;
	static float MOVE_to_STILL_delta_s = 0;
	static float MOVE_to_STILL_cnt = 0;
	static float MOVE_to_STILL_cnt_max = 0;
	
	//****************************** PID ******************************//
	
	//----- 速度（小车速度）
	static float P_v = 0, I_v = 0, D_v = 0;
	static float a_v1 = 0,     a_v2= 0;
	
	static float v1 = 0,       v2 = 0;

	static float pwm1 = 0,     pwm2 = 0;
	static float pwm_max = 0;

	//----- 角偏（小车指向目标物体的方向为x轴正向，小车朝向为向量）
	static float P_angle = 0, I_angle = 0, D_angle = 0;
	static float a_angle = 0;

	static float angle = 0;

	static float a_vd = 0;         // v_differ 右轮为正，左轮为负
	static float a_vd_max = 0;

	//----- 距离（小车离目标物体的距离）
	static float P_s = 0, I_s = 0, D_s = 0;
	static float a_s = 0;

	static float s = 0;

	static float a_vc = 0;        // v_common
	static float a_vc_max = 0;

	//****************************** OTHERS ******************************//
	
	float a_v1_tmp = 0; float a_v2_tmp = 0;
	
    //*********|\********|\********|\********|\********|\********|\********|\********|\********|\********|\*********
    //*********|*********|*********|*********|*********|*********|*********|*********|*********|*********|**********
    //*********|*********|*********|*********|*********|*********|*********|*********|*********|*********|**********
	
	CPUoccupationRate_Calculatestart();
	TIM_ClearITPendingBit(TIM4, TIM_IT_Update);
	cnt = (cnt >= 999) ? 0 : cnt + 1;
	
    //*********|*********|*********|*********|*********|*********|*********|*********|*********|*********|**********
    //*********|*********|*********|*********|*********|*********|*********|*********|*********|*********|**********
    //********\|********\|********\|********\|********\|********\|********\|********\|********\|********\|**********
		
    //****************************** UART_R ******************************//
	
	if(trans_bt_R(3, 0, 0, 0, 
		                &P_v,              &I_v,              &D_v,              &pwm_max,          0,
					    &P_angle,          &I_angle,          &D_angle,          &a_vd_max,         0,
					    &P_s,              &I_s,              &D_s,              &a_vc_max,         &a_s,
					    &a_v1, 0, 0, 0, 0,
					    0, 0, 0, 0, 0,
					    0, 0, 0, 0, 0 ))
	{	
		;
	}
		               /*P_v     = 0       ,I_v     = 0       ,D_v     = 0       ,*/pwm_max  = 7000           ;
					   /*P_angle = 0       ,I_angle = 0       ,D_angle = 0       ,a_vd_max = 0              ;*/
					   /*P_s     = 0       ,I_s     = 0       ,D_s     = 0       ,a_vc_max = 0      ,a_s = 0;*/	
	
	PID_init(&v1_pid,    P_v,               I_v,               D_v,               pwm_max,    0);
	PID_init(&v2_pid,    P_v,               I_v,               D_v,               pwm_max,    0);
	PID_init(&angle_pid, P_angle,           I_angle,           D_angle,           a_vd_max,   0);
	PID_init(&s_pid,     P_s,               I_s,               D_s,               a_vc_max,   0);
    
	//****************************** SENSORS ******************************//
	
    v1 = -( ic_getdata(1) - 1);
	v2 =    ic_getdata(2) - 1;
	
	//****************************** STATE_AND_PID ******************************//
	
	if(cnt % 10 == 0)
	{
		if(state == TEST__V_PID_ONLY)
		{
			//STATE_CHANGE
			
			
			//PID_0->1
			a_v2 = a_v1;
			
			//PID_2
			pwm1 = PID_calc2(&v1_pid, v1, a_v1);
			pwm2 = PID_calc2(&v2_pid, v2, a_v2);
			
			//PID_END
			straight_add(pwm1, pwm2);
		}
		
		else if(state == MOVE)
		{
			//STATE_CHANGE
			if(angle - a_angle <  MOVE_to_STILL_delta_angle && 
			   angle - a_angle > -MOVE_to_STILL_delta_angle && 
				   s -     a_s <      MOVE_to_STILL_delta_s && 
				   s -     a_s >     -MOVE_to_STILL_delta_s &&
			   v1 == 0 && v2 == 0 )
				if(MOVE_to_STILL_cnt++ > MOVE_to_STILL_cnt_max)
				{
					MOVE_to_STILL_cnt = 0;
					
					PID_init(&v1_pid,    P_v,               I_v,               D_v,               pwm_max,    1);
					PID_init(&v2_pid,    P_v,               I_v,               D_v,               pwm_max,    1);
					PID_init(&angle_pid, P_angle,           I_angle,           D_angle,           a_vd_max,   1);
					PID_init(&s_pid,     P_s,               I_s,               D_s,               a_vc_max,   1);
					state = STILL;
				}
			
			//PID_1
			a_vd = PID_calc1(&angle_pid, angle, a_angle);
			a_vc = PID_calc1(&s_pid, s, a_s);

			//PID_1->2
			a_v1 = a_vc - a_vd, a_v2 = a_vc + a_vd;
		 
			//PID_2
			pwm1 = PID_calc2(&v1_pid, v1, a_v1);
			pwm2 = PID_calc2(&v2_pid, v2, a_v2);
			
			//PID_END
			straight_add(pwm1, pwm2);	
		}
		
		else if(state == STILL)
		{
			//STATE_CHANGE
			if(angle - a_angle >  MOVE_to_STILL_delta_angle || 
			   angle - a_angle < -MOVE_to_STILL_delta_angle || 
				   s -     a_s >      MOVE_to_STILL_delta_s || 
				   s -     a_s <     -MOVE_to_STILL_delta_s  )
				state = MOVE;

			//PID_END
			straight_set(0, 0); 
		}
    }
		
    //****************************** UART_T ******************************//

	if(cnt % 4 == 0)
		trans_bt_T(3, 0, 0, 0, v1,      v2,       a_v1,     a_v2,       cnt,
	                           pwm1,    pwm2,     CPUoccupationRate_Calculatefinish(), 0, 0,
						       0, 0, 0, 0, 0,
						       0, 0, 0, 0, 0,
							   0, 0, 0, 0, 0,
							   0, 0, 0, 0, 0 );
}
