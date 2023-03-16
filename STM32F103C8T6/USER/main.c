#include "include.h"

//stm32zet6 的模板trans有两个问题
//pid调控要尽量线性且必有始末
//v环的控制周期大于 其他
//暂时待机（静止）状态的加入，
//

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
	
	trans_others_init(2, 115200);
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

	static enum{
		STOP,
		TEST__V_PID_ONLY,
	    MOVE,
		STILL
	}state = MOVE, state_store = MOVE;

	static char stop_button = 1;
	static char TEST__V_PID_ONLY_button = 0;
	
	static float MOVE_to_STILL_delta_angle = 0;
	static float MOVE_to_STILL_delta_s = 0;
	static float STILL_to_MOVE_delta_s = 0;
	static float MOVE_between_STILL_cnt = 0;
	static float MOVE_between_STILL_cnt_max = 0;
	
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

	static int angle_int[10] = {0};
	static float average_filter_num = 0;
	static float angle = 0;

	static float a_vd = 0;         // v_differ 右轮为正，左轮为负
	static float a_vd_max = 0;

	//----- 距离（小车离目标物体的距离的反比）
	static float P_s = 0, I_s = 0, D_s = 0;
	static float a_s = 0;

	static int s_int[10] = {0};
//  static int average_filter_num = 0;
	static float s = 0;

	static float a_vc = 0;        // v_common
	static float a_vc_max = 0;
	
	static float min_enable_pid__s = 0;
	static float min_enable_pid__delta_s = 0;

	//****************************** OTHERS ******************************//
	static float a_v1_temp = 0, a_v2_temp = 0;
	static float a_v__peradd = 0;
	
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
	
	if(trans_bt_R(3,    &stop_button, 
		                &TEST__V_PID_ONLY_button, 0, 
		                &P_v,              &I_v,              &D_v,              &pwm_max,          0,
					    &P_angle,          &I_angle,          &D_angle,          &a_vd_max,         0,
					    &P_s,              &I_s,              &D_s,              &a_vc_max,         &a_s,
					    &a_v1,             
	                    &min_enable_pid__s,
	                    &STILL_to_MOVE_delta_s,
	                    &MOVE_to_STILL_delta_angle, 
	                    &MOVE_to_STILL_delta_s, 
	                    &MOVE_between_STILL_cnt_max,
					    &average_filter_num, 
	                    &a_v__peradd, 0, 0,
					    0, 0, 0, 0, 0 ))
	{
        ;
	}
		               /*P_v     = 0       ,I_v     = 0       ,D_v     = 0       ,*/pwm_max  = 7000           ;
					   /*P_angle = 0       ,I_angle = 0       ,D_angle = 0       ,a_vd_max = 0              ;*/
					   /*P_s     = 0       ,I_s     = 0       ,D_s     = 0       ,a_vc_max = 0      ,a_s = 0;*/	
//					    a_v1,             
//	                    min_enable_pid__s,
//	                    min_enable_pid__delta_s,
//	                    MOVE_to_STILL_delta_angle, 
//	                    MOVE_to_STILL_delta_s, 
//	                    MOVE_to_STILL_cnt_max,
	
	PID_init(&v1_pid,    P_v,               I_v,               D_v,               pwm_max,    0);
	PID_init(&v2_pid,    P_v,               I_v,               D_v,               pwm_max,    0);
	PID_init(&angle_pid, P_angle,           I_angle,           D_angle,           a_vd_max,   0);
	PID_init(&s_pid,     P_s,               I_s,               D_s,               a_vc_max,   0);
    
	//****************************** SENSORS ******************************//
	
	for(i = 10 - 1; i > 0; i--)
	{
	    angle_int[i] = angle_int[i - 1];
		    s_int[i] =     s_int[i - 1];
	}
	trans_others_R(2, &(angle_int[0]), &(s_int[0]), 0,0,0,0,0,0,0,0);
	angle = 0;
	    s = 0;
	for(i = 0; i < average_filter_num; i++)
	{
	    angle += (float)(angle_int[i] / average_filter_num);
		    s += (float)(    s_int[i] / average_filter_num);
	}
	
    v1 =    ic_getdata(2) - 1  ;
	v2 = -( ic_getdata(1) - 1 );
	
	//****************************** STATE_AND_PID ******************************//
	
	if(stop_button == 1)
	{
	    state = STOP;
		if(state != STOP && state != TEST__V_PID_ONLY)
			state_store = state;
	}
	else if(TEST__V_PID_ONLY_button == 1)
	{
	    state = TEST__V_PID_ONLY;
		if(state != STOP && state != TEST__V_PID_ONLY)
			state_store = state;
	}
	
	else if(state == STOP || state == TEST__V_PID_ONLY)
		state = state_store;
	
	
	if(cnt % 10 == 0)
	{
		     if(state == STOP)
		{
			
			straight_set(0, 0);	
		}
		
		else if(state == TEST__V_PID_ONLY)
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
				   s -     a_s >     -MOVE_to_STILL_delta_s )
				if(MOVE_between_STILL_cnt++ > MOVE_between_STILL_cnt_max)
				{
					MOVE_between_STILL_cnt = 0;
					
					PID_init(&v1_pid,    P_v,               I_v,               D_v,               pwm_max,    1);
					PID_init(&v2_pid,    P_v,               I_v,               D_v,               pwm_max,    1);
					PID_init(&angle_pid, P_angle,           I_angle,           D_angle,           a_vd_max,   1);
					PID_init(&s_pid,     P_s,               I_s,               D_s,               a_vc_max,   1);
					state = STILL;
				}
			
			//PID_1
			    		
			if( s > min_enable_pid__s )
			{
				a_vd = PID_calc1(&angle_pid, angle, a_angle);
			    a_vc = PID_calc1(&s_pid,     s,     a_s);
			} 

			//PID_1->2
			a_v1_temp = a_vc - a_vd, a_v2_temp = a_vc + a_vd;
			
			//速度消抖
			     if(a_v1 > a_v1_temp + a_v__peradd) a_v1 -= a_v__peradd;
			else if(a_v1 < a_v1_temp - a_v__peradd) a_v1 += a_v__peradd;
			else                                    a_v1  = a_v1_temp;
			
			     if(a_v2 > a_v2_temp + a_v__peradd) a_v2 -= a_v__peradd;
			else if(a_v2 < a_v2_temp - a_v__peradd) a_v2 += a_v__peradd;
			else                                    a_v2  = a_v2_temp;
			
			//PID_2
			pwm1 = PID_calc2(&v1_pid, v1, a_v1);
			pwm2 = PID_calc2(&v2_pid, v2, a_v2);
			
			//PID_END
			straight_add(pwm1, pwm2);	
		}
		
		else if(state == STILL)
		{
			//STATE_CHANGE
			if( s -     a_s >      STILL_to_MOVE_delta_s || 
				s -     a_s <     -STILL_to_MOVE_delta_s  )
				if(MOVE_between_STILL_cnt++ > MOVE_between_STILL_cnt_max)
				{
					MOVE_between_STILL_cnt = 0;
					
					PID_init(&v1_pid,    P_v,               I_v,               D_v,               pwm_max,    1);
					PID_init(&v2_pid,    P_v,               I_v,               D_v,               pwm_max,    1);
					PID_init(&angle_pid, P_angle,           I_angle,           D_angle,           a_vd_max,   1);
					PID_init(&s_pid,     P_s,               I_s,               D_s,               a_vc_max,   1);
					state = MOVE;
				}
				
			//PID_END
			straight_set(0, 0); 
		}
    }
		
    //****************************** UART_T ******************************//

	if(cnt % 10 == 0)
		trans_bt_T(3, 0, 0, 0, v1,      v2,       a_v1,     a_v2,       state,
	                           pwm1,    pwm2,     CPUoccupationRate_Calculatefinish(), angle, s,
						       0, 0, 0, 0, 0,
						       0, 0, 0, 0, 0,
							   0, 0, 0, 0, 0,
							   0, 0, 0, 0, 0 );
}
