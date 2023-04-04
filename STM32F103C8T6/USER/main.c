#include "include.h"

//stm32zet6 的模板trans有两个问题
//pid调控要尽量线性且必有始末
//v环的控制周期大于 其他
//暂时待机（静止）状态的加入，
//

PID v1_____pid;
PID v2_____pid;
PID angle__pid;
PID s______pid;

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
	
	PID_init(&v1_____pid, 0, 0, 0, 0, 1);
	PID_init(&v2_____pid, 0, 0, 0, 0, 1);
	PID_init(&angle__pid, 0, 0, 0, 0, 1);
	PID_init(&s______pid, 0, 0, 0, 0, 1);
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
	static int rint1 = 0;
	static int rint2 = 0;
	static int rint3 = 0;
	
	//****************************** STATE ******************************//

	static enum{
		STOP,
		TEST__V_PID_ONLY,
	    MOVE,
		PRESTILL,
		STILL
	}state = MOVE, state_store = MOVE;
	static int state_cnt1 = 0, state_cnt2 = 0;

	static char stop_button = 1;
	static char TEST__V_PID_ONLY_button = 0;
	static int move_button = 0;

	static float cntmax1 = 0;
	static float sThr________MOVE_PRESTILL_STILL = 0;

	//
#define          sMin________MOVE_to_PRESTILL         sThr________MOVE_PRESTILL_STILL
#define          cntmax______MOVE_to_PRESTILL         cntmax1
	
	static float angleDMax___MOVE_to_STILL = 0;
	static float sDMax_______MOVE_to_STILL = 0;
#define          cntmax______MOVE_to_STILL            cntmax1
	
	//
#define          sMax________PRESTILL_to_MOVE         sThr________MOVE_PRESTILL_STILL
#define          cntmax______PRESTILL_to_MOVE         cntmax1
	
	static float cntmax______PRESTILL_to_STILL = 0;
	
	//
	static float sDMin_______STILL_to_MOVE = 0;
#define          sMax________STILL_to_MOVE            sThr________MOVE_PRESTILL_STILL
#define          cntmax______STILL_to_MOVE            cntmax1
	
	//****************************** PID ******************************//
	
	//----- 速度（小车速度）
	static float v______P = 0,         v______I = 0,         v______D = 0;
	static float v1_____a = 0,         v2_____a = 0;
	
	static float v1 = 0,               v2 = 0;

	static float pwm1 = 0,             pwm2 = 0;
	static float pwm_max = 0;

	//----- 角偏（小车指向目标物体的方向为x轴正向，小车朝向为向量）
	static float angle__P = 0,         angle__I = 0,         angle__D = 0;
	static float angle__a = 0;

	static int   angleInt[10] = {0};
	static float average_filter_num = 0;
	static float angle = 0;

	static float vd_____a = 0;         // v_differ 右轮为正，左轮为负
	static float vd_____a_max = 0;

	//----- 距离（小车离目标物体的距离）
	static float s______P = 0,         s______I = 0,         s______D = 0;
	static int   s______a = 0;

	static int   sInt[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 20};
//  static int average_filter_num = 0;
	static float s = 0;

	static float vc_____a = 0;         // v_common
	static float vc_____a_max = 0;

	//****************************** OTHERS ******************************//
	static float vselfc_a = 0, vselfd_a = 0;
	
	static float v1_____a__temp = 0, v2_____a__temp = 0;
	static float v______a__peradd = 0;
	
	float pwmtotal1 = 0, pwmtotal2 = 0;
	
    //*********|\********|\********|\********|\********|\********|\********|\********|\********|\********|\*********
    //*********|*********|*********|*********|*********|*********|*********|*********|*********|*********|**********
    //*********|*********|*********|*********|*********|*********|*********|*********|*********|*********|**********
	
	static int first = 0;
	
	CPUoccupationRate_Calculatestart();
	TIM_ClearITPendingBit(TIM4, TIM_IT_Update);
	cnt = (cnt >= 999) ? 0 : cnt + 1;
	
    //*********|*********|*********|*********|*********|*********|*********|*********|*********|*********|**********
    //*********|*********|*********|*********|*********|*********|*********|*********|*********|*********|**********
    //********\|********\|********\|********\|********\|********\|********\|********\|********\|********\|**********
		
    //****************************** UART_R ******************************//
	
	if(trans_bt_R(3,      &stop_button, 
		                  &TEST__V_PID_ONLY_button, 0, 
		                  &v______P,         &v______I,         &v______D,         &pwm_max,          0,
					      &angle__P,         &angle__I,         &angle__D,         &vd_____a_max,     0,
					      &s______P,         &s______I,         &s______D,         &vc_____a_max,     0,//&s______a,
					      &vselfc_a,
	                      &vselfd_a,
	                      &sDMin_______STILL_to_MOVE,
	                      &angleDMax___MOVE_to_STILL, 
	                      &sDMax_______MOVE_to_STILL, 
	                      &cntmax1,
					      &average_filter_num, 
	                      &v______a__peradd, 
	                      &sThr________MOVE_PRESTILL_STILL, 
	                      &cntmax______PRESTILL_to_STILL,
					      0, 0, 0, 0, 0 ))
	{
        ;
	}
                      if(first == 0)
					  { 
	                      first = 1;
						  
						  stop_button = 0;
		                  TEST__V_PID_ONLY_button = 0;
		                  v______P = 10;     v______I = 3;      v______D = 0;      pwm_max = 7000;         
					      angle__P = -0.02;  angle__I = 0;      angle__D = 0;      vd_____a_max = 10;   
					      s______P = 1;      s______I = 0;      s______D = 0;      vc_____a_max = 40;  //s______a = 20;
					      vselfc_a = 0;
	                      vselfd_a = 0;
	                      sDMin_______STILL_to_MOVE = 10;
	                      angleDMax___MOVE_to_STILL = 1000;
	                      sDMax_______MOVE_to_STILL = 5;
	                      cntmax1 = 10;
					      average_filter_num = 1;
	                      v______a__peradd = 5;
	                      sThr________MOVE_PRESTILL_STILL = 80;
	                      cntmax______PRESTILL_to_STILL = 300;
				      }
					   

	
	PID_init(&v1_____pid, v______P,          v______I,          v______D,          pwm_max,        0);
	PID_init(&v2_____pid, v______P,          v______I,          v______D,          pwm_max,        0);
	PID_init(&angle__pid, angle__P,          angle__I,          angle__D,          vd_____a_max,   0);
	PID_init(&s______pid, s______P,          s______I,          s______D,          vc_____a_max,   0);
    
	//****************************** SENSORS ******************************//
	
	for(i = 10 - 1; i > 0; i--)
	{
	    angleInt[i] = angleInt[i - 1];
		    sInt[i] =     sInt[i - 1];
	}
	if(trans_others_R(2, &rint1, &rint2, &rint3, 0,0,0,0,0,0,0))
	{
		if     (rint1 == 1)
	    {
		    if(rint2 == 1) state = MOVE;
		}
		else if(rint1 == 2)
		{
			s______a = rint2;
		}
		else if(rint1 == 3)
		{
		    angleInt[0] = rint2;
			sInt[0] = rint3;
		}
	}
	angle = 0;
	    s = 0;
	for(i = 0; i < average_filter_num; i++)
	{
	    angle += (float)(angleInt[i] / average_filter_num);
		    s += (float)(    sInt[i] / average_filter_num);
	}
	
    v1 =    ic_getdata(2) - 1  ;
	v2 = -( ic_getdata(1) - 1 );
	
	//****************************** STATE_AND_PID ******************************//
	
	if     (            stop_button == 1)
	{
	        state  = STOP;
		 if(state != STOP && state != TEST__V_PID_ONLY)
			   state_store = state;
	}
	else if(TEST__V_PID_ONLY_button == 1)
	{
	                         state  = TEST__V_PID_ONLY;
		 if(state != STOP && state != TEST__V_PID_ONLY)
			   state_store = state;
	}
	else if(state == STOP || state == TEST__V_PID_ONLY)
	{
		PID_init(&v1_____pid, v______P,          v______I,          v______D,          pwm_max,        1);
		PID_init(&v2_____pid, v______P,          v______I,          v______D,          pwm_max,        1);
		PID_init(&angle__pid, angle__P,          angle__I,          angle__D,          vd_____a_max,   1);
		PID_init(&s______pid, s______P,          s______I,          s______D,          vc_____a_max,   1);
		//state = state_store;
		state = STILL;
	}
		
	
	if     (state == STOP)
	{
		//STATE_CHANGE
		
		
		straight_set(0, 0);	
	}
	
	else if(state == TEST__V_PID_ONLY)
	{
		//STATE_CHANGE
		

		//PID_1->2

		if     (vselfc_a  < 0 && vselfd_a  < 0)
		{
		    v1_____a = (vselfc_a - vselfd_a > 0) ? 0 : vselfc_a - vselfd_a;
			v2_____a =  vselfc_a;
		}
		else if(vselfc_a >= 0 && vselfd_a  < 0)
		{
		    v1_____a = (vselfc_a + vselfd_a < 0) ? 0 : vselfc_a + vselfd_a;
			v2_____a =  vselfc_a;  
		}
		else if(vselfc_a  < 0 && vselfd_a >= 0)
		{
		    v2_____a = (vselfc_a + vselfd_a > 0) ? 0 : vselfc_a + vselfd_a;
			v1_____a =  vselfc_a;		    
		}
		else if(vselfc_a >= 0 && vselfd_a >= 0)
		{
		    v2_____a = (vselfc_a - vselfd_a < 0) ? 0 : vselfc_a - vselfd_a;
			v1_____a =  vselfc_a;
		}
		
		//PID_2
		pwm1 = PID_calc2(&v1_____pid, v1, v1_____a);
		pwm2 = PID_calc2(&v2_____pid, v2, v2_____a);
		
		//PID_END
		straight_add(pwm1, pwm2);
	}
	
	else if(state == MOVE)
	{
		//STATE_CHANGE
		if     (angle - angle__a <  angleDMax___MOVE_to_STILL && 
		        angle - angle__a > -angleDMax___MOVE_to_STILL && 
		        s     - s______a <  sDMax_______MOVE_to_STILL && 
		        s     - s______a > -sDMax_______MOVE_to_STILL )
		{
			if(state_cnt1++ > cntmax1)
			{
				state_cnt1 = 0;
				
				PID_init(&v1_____pid, v______P,          v______I,          v______D,          pwm_max,        1);
				PID_init(&v2_____pid, v______P,          v______I,          v______D,          pwm_max,        1);
				PID_init(&angle__pid, angle__P,          angle__I,          angle__D,          vd_____a_max,   1);
				PID_init(&s______pid, s______P,          s______I,          s______D,          vc_____a_max,   1);
				state = STILL;
			}			
		}
		else if(s > sMin________MOVE_to_PRESTILL )
			if(state_cnt2++ > cntmax1)
			{
				state_cnt2 = 0;
				
				state = PRESTILL;
			}
		
		//PID_1
		if(cnt % 10 == 0)
		{
			vd_____a =  PID_calc1(&angle__pid, angle, angle__a);
			vc_____a = -PID_calc1(&s______pid, s,     s______a);
		}

		//PID_1->2
		v1_____a__temp = vc_____a - vd_____a, v2_____a__temp = vc_____a + vd_____a;
		
		//速度消抖
			 if(v1_____a > v1_____a__temp + v______a__peradd) v1_____a -= v______a__peradd;
		else if(v1_____a < v1_____a__temp - v______a__peradd) v1_____a += v______a__peradd;
		else                                                  v1_____a  = v1_____a__temp;
		
			 if(v2_____a > v2_____a__temp + v______a__peradd) v2_____a -= v______a__peradd;
		else if(v2_____a < v2_____a__temp - v______a__peradd) v2_____a += v______a__peradd;
		else                                                  v2_____a  = v2_____a__temp;
		
		//PID_2
		pwm1 = PID_calc2(&v1_____pid, v1, v1_____a);
		pwm2 = PID_calc2(&v2_____pid, v2, v2_____a);
		
		//PID_END
		straight_add(pwm1, pwm2);	
	}
	
	else if(state == PRESTILL)
	{
		if     (s < sMax________PRESTILL_to_MOVE )
		{
			if(state_cnt2++ > cntmax1)
			{
				state_cnt2 = 0;
				
				state = MOVE;
			}
		}
		else
		{
			if(state_cnt1++ > cntmax______PRESTILL_to_STILL)
			{
				state_cnt1 = 0;
				
				PID_init(&v1_____pid, v______P,          v______I,          v______D,          pwm_max,        1);
				PID_init(&v2_____pid, v______P,          v______I,          v______D,          pwm_max,        1);
				PID_init(&angle__pid, angle__P,          angle__I,          angle__D,          vd_____a_max,   1);
				PID_init(&s______pid, s______P,          s______I,          s______D,          vc_____a_max,   1);
				state = STILL;
			}		
		}
		
		//PID_2
		pwm1 = PID_calc2(&v1_____pid, v1, v1_____a);
		pwm2 = PID_calc2(&v2_____pid, v2, v2_____a);
		
		//PID_END
		straight_add(pwm1, pwm2);
	}
	
	else if(state == STILL)
	{ 
		//STATE_CHANGE
//		if( (s     - s______a >  sDMin_______STILL_to_MOVE || 
//			 s     - s______a < -sDMin_______STILL_to_MOVE ) && 
//		     s                <  sMax________PRESTILL_to_MOVE )
//			if(state_cnt1++ > cntmax1)
//			{
//				state_cnt1 = 0;
//				
//				PID_init(&v1_____pid, v______P,          v______I,          v______D,          pwm_max,        1);
//				PID_init(&v2_____pid, v______P,          v______I,          v______D,          pwm_max,        1);
//				PID_init(&angle__pid, angle__P,          angle__I,          angle__D,          vd_____a_max,   1);
//				PID_init(&s______pid, s______P,          s______I,          s______D,          vc_____a_max,   1);
//				state = MOVE;
//			}
			
		//PID_END
		straight_set(0, 0); 
	}

		
    //****************************** UART_T ******************************//

    straight_read(&pwmtotal1, &pwmtotal2);
	
	if(cnt % 10 == 0)
		trans_bt_T(3, 0, 0, 0, v1,      v2,       v1_____a, v2_____a,   state,
	                           pwm1,    pwm2,     CPUoccupationRate_Calculatefinish(), angle, s,
						       pwmtotal1, pwmtotal2, 0, 0, 0,
						       0, 0, 0, 0, 0,
							   0, 0, 0, 0, 0,
							   0, 0, 0, 0, 0 );
	
	if(cnt % 10 == 0)
		printf("state = %d\r\n", state );
		
}
