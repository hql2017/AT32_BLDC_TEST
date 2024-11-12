#include <stdio.h>
#include "limits.h"
#include "apex_gc_port.h"
#include "gpio_port.h"
#include "pwm_t.h"
#include "system_at32f413.h"
#include "at32f413_board.h"
#include "para_list.h"
//#include "at32f413.h"
//#include "at32f413_tmr.h"

static uint16_t apex_gc_frequence =10000;//>8kHz;//beep  highest vol  f=4000(4k~5K)(4k70db,5k 60db);middle vol  f=1000(60db);
static int32_t gc_num_list[31] = {0};//闁跨喐鏋婚幏鐑芥晸閺傘倖瀚归柨鐔告灮閹风兘鏁撻弬銈嗗闁跨喐鏋婚幏鐑芥晸閹恒儳顒查幏鐑芥晸閺傘倖瀚�

extern union Param_Union  sys_param_un; 
/**
  * @brief  APEXInit
  * @param  none
  * @retval none
  */
void APEXInit(void)
{
	uint16_t div_value = 0 ;//tmer clock div value
	uint16_t ch1_val = 0 ;//PWM value
	uint16_t tmr_count_val = 390 ;//timer count value
	tmr_output_config_type tmr_oc_init_structure;
  gpio_init_type gpio_init_structurt;//gpio structure	
	
	//insert check
	crm_periph_clock_enable(CRM_GPIOA_PERIPH_CLOCK, TRUE);
  gpio_default_para_init(&gpio_init_structurt);
  gpio_init_structurt.gpio_mode = GPIO_MODE_INPUT;
  gpio_init_structurt.gpio_pins = GPIO_PINS_9;////GPIO_PINS_9;//tx
  gpio_init_structurt.gpio_pull = GPIO_PULL_UP;
  gpio_init(GPIOA, &gpio_init_structurt);		

	crm_periph_clock_enable(APEX_RELEY_CLOCK, TRUE);
  gpio_default_para_init(&gpio_init_structurt);
  gpio_init_structurt.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
  gpio_init_structurt.gpio_out_type  = GPIO_OUTPUT_PUSH_PULL;
  gpio_init_structurt.gpio_mode = GPIO_MODE_OUTPUT;
  gpio_init_structurt.gpio_pins = APEX_RELEY_IO;
  gpio_init_structurt.gpio_pull = GPIO_PULL_UP;
  gpio_init(APEX_RELEY_PORT, &gpio_init_structurt);			
	
	crm_periph_clock_enable(APEX_PWM_CLOCK,TRUE);

  gpio_init_structurt.gpio_pins =APEX_PWM_IO;//PA0 timer2 cha1
  gpio_init_structurt.gpio_mode =GPIO_MODE_MUX ;
	gpio_init_structurt.gpio_drive_strength= GPIO_DRIVE_STRENGTH_STRONGER ;
  gpio_init_structurt.gpio_out_type =GPIO_OUTPUT_PUSH_PULL ;
	gpio_init_structurt.gpio_pull =GPIO_PULL_UP;
  gpio_init(APEX_PWM_PORT,&gpio_init_structurt);
	
	gpio_bits_reset(APEX_PWM_PORT, APEX_PWM_IO);//close
	
	 /* compute the div value */
  div_value = (uint16_t)((system_core_clock) / 1000000) - 1;// div
		
	tmr_count_val=((system_core_clock/(div_value+1))/apex_gc_frequence)-1;//count frequency 4K
	ch1_val=tmr_count_val/2;//duty ratio

  /* tmr2 time base configuration */
	crm_periph_clock_enable(CRM_TMR2_PERIPH_CLOCK,TRUE);
  tmr_base_init(TMR2, tmr_count_val, div_value);
	
  tmr_cnt_dir_set(TMR2, TMR_COUNT_UP);	
	
  tmr_clock_source_div_set(TMR2, TMR_CLOCK_DIV1);

  tmr_output_default_para_init(&tmr_oc_init_structure);
  tmr_oc_init_structure.oc_mode = TMR_OUTPUT_CONTROL_PWM_MODE_A;
  tmr_oc_init_structure.oc_idle_state = FALSE;
  tmr_oc_init_structure.oc_polarity = TMR_OUTPUT_ACTIVE_LOW;
  tmr_oc_init_structure.oc_output_state = TRUE;
  tmr_output_channel_config(TMR2, TMR_SELECT_CHANNEL_1, &tmr_oc_init_structure);
  tmr_channel_value_set(TMR2, TMR_SELECT_CHANNEL_1, ch1_val);
  tmr_output_channel_buffer_enable(TMR2, TMR_SELECT_CHANNEL_1, TRUE);
	
  tmr_period_buffer_enable(TMR2, TRUE);
	
}

	/**
  * @brief  gc_in_or_exit
  * @param  none
  * @retval none
  */
void gc_in_or_exit(unsigned char inOrExit)
{
	if(inOrExit==APEX_GC_IN)
	{
		gpio_bits_set(APEX_RELEY_PORT, APEX_RELEY_IO);
	}
	else 
	{
		gpio_bits_reset(APEX_RELEY_PORT, APEX_RELEY_IO);
	}		
}	
/**
  * @brief  frequency setting for APEX gc
  * @param  
  * @retval none
  */
 void apex_frequency_set(unsigned short int frequency)
{
	uint16_t div_value = 0 ;//tmer clock div value
	uint16_t tmr_count_val = 125 ;//timer count value	
	uint16_t ch1_val = 0 ;//PWM value
	tmr_counter_enable(TMR2, FALSE);	
	if(frequency!=0)
	{	
		if(frequency==8000)//8k
		{	
			/* compute the div value */
			div_value = (uint16_t)((system_core_clock) / 1000000) - 1;// 1mHz div 			
			tmr_count_val=((system_core_clock/(div_value+1))/frequency)-1;//count frequency 8K  tmr_count_val=125;	
			ch1_val=tmr_count_val/2;//duty ratio
		}
		else if(frequency==400)//8k//400
		{
			/* compute the div value */
			div_value = (uint16_t)((system_core_clock) / 1000000) - 1;// 1mHz div			 
			tmr_count_val=((system_core_clock/(div_value+1))/frequency)-1;//count frequency 400 tmr_count_val=2500;
			ch1_val=tmr_count_val/2;//duty ratio
		}	
		else 
		{
			/* compute the div value */
			div_value = (uint16_t)((system_core_clock) / 1000000) - 1;// div			
			tmr_count_val=((system_core_clock/(div_value+1))/frequency)-1;
			ch1_val=tmr_count_val/2;//duty ratio
		}	
		tmr_base_init(TMR2, tmr_count_val, div_value);	
		tmr_channel_value_set(TMR2, TMR_SELECT_CHANNEL_1, ch1_val);
		tmr_period_buffer_enable(TMR2, TRUE);
		tmr_counter_enable(TMR2, TRUE);	
	}	
}

 void gc_list_init(int32_t n0, int32_t n1)
{					
//double gc_rate_g = (double)(gc_val_l)*0.28*k/3.0;//(double)gc_val_l*0.28/3.0;		
//	double gc_rate_b= (double)(n0-n1)*1.0/30;//25.0;//(double)gc_val_l*0.28/3.0;
//	double gc_rate_g = (double)(n0-n1)*0.15/3.0;	//gc_rate_b~=	gc_rate_g*3;	
//	int32_t gc_val_h = (int32_t)((double)n1 + (gc_rate_g*1 ) + (gc_rate_b * 13));
//	double gc_rate_l = (double)(n0-gc_val_h-10)/3.0;	
	int32_t gc_val_l = n0-n1;//闁跨喐鏋婚幏鐑芥晸閺傘倖瀚归柨鐔告灮閹疯渹绠涢柨鐔告灮閹风兘鏁撶悰妤冾暜閹风兘鏁撴鐚存嫹0闁跨喐鏋婚幏閿嬬墡閸戝棝鏁撻弬銈嗗闁跨喐鏋婚幏宄扳偓锟�
	double gc_rate_g = (double)gc_val_l*0.076;
	double gc_rate_b = (double)gc_val_l*0.034;
	double temp=gc_rate_b*0.2;	
	int32_t gc_val_h = (int32_t)((double)n1 +  (gc_rate_g+temp)*3+ (gc_rate_b *18));
	double gc_rate_l = (double)(n0-gc_val_h-10)/3.0;

//if(gc_rate_g>15.6) gc_rate_g=15.2;
//NRF_LOG_INFO("gc_rate_b = "NRF_LOG_FLOAT_MARKER"",NRF_LOG_FLOAT(gc_rate_b));
//NRF_LOG_INFO("gc_rate_l = "NRF_LOG_FLOAT_MARKER"",NRF_LOG_FLOAT(gc_rate_l));	
	
	gc_num_list[0] = (int32_t)((double)n1 - ((gc_rate_g+temp) * 12));
	gc_num_list[1] = (int32_t)((double)n1 - ((gc_rate_g+temp)  * 10));
	gc_num_list[2] = (int32_t)((double)n1 - ((gc_rate_g+temp)  * 8));
	gc_num_list[3] = (int32_t)((double)n1 - ((gc_rate_g+temp)  * 6));
	gc_num_list[4] = (int32_t)((double)n1 - ((gc_rate_g+temp)  * 4));
	gc_num_list[5] = (int32_t)((double)n1 - ((gc_rate_g+temp)  * 2));
	gc_num_list[6] = n1;
	gc_num_list[7] = (int32_t)((double)n1 + ((gc_rate_g+temp)  * 1));
	gc_num_list[8] = (int32_t)((double)n1 + ((gc_rate_g+temp)  * 2));
	gc_num_list[9] = (int32_t)((double)n1 + ((gc_rate_g+temp)  * 3));
//gc_num_list[10] = (int32_t)((double)n1 + ((gc_rate_g+temp)  * 4));
//	gc_num_list[9] = (int32_t)((double)gc_num_list[8] +  (gc_rate_b*1 ));
//gc_num_list[9] = (int32_t)((double)n1 + ((gc_rate_g+temp)  * 3));
//	gc_num_list[10] = (int32_t)((double)gc_num_list[9] + (gc_rate_b*1 ));
//	gc_num_list[9] = (int32_t)((double)gc_num_list[8] + (gc_rate_b*1 ));
	gc_num_list[10] = (int32_t)((double)gc_num_list[9] + (gc_rate_b*1 ));
	gc_num_list[11] = (int32_t)((double)gc_num_list[10] + (gc_rate_b * 1));
	gc_num_list[12] = (int32_t)((double)gc_num_list[10] + (gc_rate_b * 2));
	gc_num_list[13] = (int32_t)((double)gc_num_list[10] + (gc_rate_b * 3));
	gc_num_list[14] = (int32_t)((double)gc_num_list[10] + (gc_rate_b * 4));
	gc_num_list[15] = (int32_t)((double)gc_num_list[10] + (gc_rate_b * 5));
	gc_num_list[16] = (int32_t)((double)gc_num_list[10] + (gc_rate_b * 6));
	gc_num_list[17] = (int32_t)((double)gc_num_list[10] + (gc_rate_b * 7));
	gc_num_list[18] = (int32_t)((double)gc_num_list[10] + (gc_rate_b * 8));
	gc_num_list[19] = (int32_t)((double)gc_num_list[10] + (gc_rate_b * 9));
	gc_num_list[20] = (int32_t)((double)gc_num_list[10] + (gc_rate_b * 10));
	gc_num_list[21] = (int32_t)((double)gc_num_list[10] + (gc_rate_b * 11));
	gc_num_list[22] = (int32_t)((double)gc_num_list[10] + (gc_rate_b * 12));
	gc_num_list[23] = (int32_t)((double)gc_num_list[10] + (gc_rate_b * 13));
	gc_num_list[24] = (int32_t)((double)gc_num_list[10] + (gc_rate_b * 14));
	gc_num_list[25] = (int32_t)((double)gc_num_list[10] + (gc_rate_b * 15));
	gc_num_list[26] = (int32_t)((double)gc_num_list[10] + (gc_rate_b *16));
	gc_num_list[27] = (int32_t)((double)gc_num_list[10] + (gc_rate_b * 17));
	gc_num_list[28] = (int32_t)((double)gc_num_list[27] + (gc_rate_l * 1));
	gc_num_list[29] = (int32_t)((double)gc_num_list[27] + (gc_rate_l * 2));
	gc_num_list[30] = (int32_t)((double)gc_num_list[27] + (gc_rate_l * 3));

}
//闁跨喐鏋婚幏鐑芥晸鐟欙絽浼愰柨鐔告灮閹风兘鏁撻弬銈嗗闁跨喖銈虹涵閿嬪闁跨喐鏋婚幏锟�
//rat_r 8k闁跨喐鏋婚幏鐑芥晸閺傘倖瀚归崐锟� rat_c 400闁跨喐鏋婚幏鐑芥晸閺傘倖瀚归崐锟�
static int32_t gc_calc_wl(int32_t rat_r, int32_t rat_c)
{
	volatile static int32_t rat;
		volatile static int32_t hitory_rat;
	volatile static int32_t result = 30;	
		volatile static int32_t history_result = 30;	
	rat = rat_r * 1000 / rat_c;	

	if((rat_r <= 200)&&(rat_c <= 200)) {
		return(-7);//閺夆晜鏌ㄥú锟�
	}	
//		rat=gc_calc_cccc(rat_r,rat_c);
	//400Hz閸樺鏁撻弬銈嗗闁跨喐鏋婚崚浼存晸閺傘倖瀚归崐濂告晸閺傘倖瀚归柨鐔兼應閿濆繑瀚归柨鐔告灮閹风兘鏁撻弬銈嗗闁跨喐鏋婚幏鐑芥晸閺傘倖瀚归柨鐔告灮閹风兘鏁撻敓锟�	
//	if(rat_c< sys_param_un.device_param.apex_tine_400Value) 
		if(rat_c+10< sys_param_un.device_param.apex_tine_400Value) //闁跨喐鏋婚幏鐑芥晸閺傘倖瀚�
	//400Hz閸樺鏁撻弬銈嗗闁跨喐鏋婚崚浼存晸閺傘倖瀚归崐濂告晸閺傘倖瀚归柨鐔兼應閿濆繑瀚归柨鐔告灮閹风兘鏁撻弬銈嗗闁跨喐鏋婚幏鐑芥晸閺傘倖瀚归柨鐔告灮閹风兘鏁撻敓锟�	
 {
		//闁跨喐鏋婚幏鐑芥晸閺傘倖瀚归柨鐔告灮閹风兘鏁撻弬銈嗗闁跨喐鏋婚幏锟�
		if(rat < gc_num_list[0]) {
						result = 0;
					}
					else if((gc_num_list[0] <= rat)&&(rat < gc_num_list[1])) {
						result = 0;
					}
					else if((gc_num_list[1] <= rat)&&(rat < gc_num_list[2])) {
						result = 0;
					}
					else if((gc_num_list[2] <= rat)&&(rat < gc_num_list[3])) {
						result = 1;
					}
					else if((gc_num_list[3] <= rat)&&(rat < gc_num_list[4])) {
						result = 2;
					}
					else if((gc_num_list[4] <= rat)&&(rat < gc_num_list[5])) {
						result = 3;
					}
					else if((gc_num_list[5] <= rat)&&(rat < gc_num_list[6])) {
						result = 4;
					}
					else if((gc_num_list[6] <= rat)&&(rat < gc_num_list[7])) {
						result = 5;
					}
					else if((gc_num_list[7] <= rat)&&(rat < gc_num_list[8])) {
						result = 6;
					}
					else if((gc_num_list[8] <= rat)&&(rat < gc_num_list[9])) {
						result = 7;
					}
					else if((gc_num_list[9] <= rat)&&(rat < gc_num_list[10])) {
						result = 8;
					}
					else if((gc_num_list[10] <= rat)&&(rat < gc_num_list[11])) {
						result = 9;
					}
					else if((gc_num_list[11] <= rat)&&(rat < gc_num_list[12])) {
						result = 10;
					}
					else if((gc_num_list[12] <= rat)&&(rat < gc_num_list[13])) {
						result = 11;
					}
					else if((gc_num_list[13] <= rat)&&(rat < gc_num_list[14])) {
						result = 12;
					}
					else if((gc_num_list[14] <= rat)&&(rat < gc_num_list[15])) {
						result = 13;
					}
					else if((gc_num_list[15] <= rat)&&(rat < gc_num_list[16])) {
						result = 14;
					}
					else if((gc_num_list[16] <= rat)&&(rat < gc_num_list[17])) {
						result = 15;
					}
					else if((gc_num_list[17] <= rat)&&(rat < gc_num_list[18])) {
						result = 16;
					}
					else if((gc_num_list[18] <= rat)&&(rat < gc_num_list[19])) {
						result = 17;
					}
					else if((gc_num_list[19] <= rat)&&(rat < gc_num_list[20])) {
						result = 18;
					}
					else if((gc_num_list[20] <= rat)&&(rat < gc_num_list[21])) {
						result = 19;
					}
					else if((gc_num_list[21] <= rat)&&(rat < gc_num_list[22])) {
						result = 20;
					}
					else if((gc_num_list[22] <= rat)&&(rat < gc_num_list[23])) {
						result = 21;
					}
					else if((gc_num_list[23] <= rat)&&(rat < gc_num_list[24])) {
						result = 22;
					}
					else if((gc_num_list[24] <= rat)&&(rat < gc_num_list[25])) {
						result = 23;
					}
					else if((gc_num_list[25] <= rat)&&(rat < gc_num_list[26])) {
						result = 24;
					}
					else if((gc_num_list[26] <= rat)&&(rat < gc_num_list[27])) {
						result = 25;
					}
					else if((gc_num_list[27] <= rat)&&(rat < gc_num_list[28])) {
						result = 26;
					}
					else if((gc_num_list[28] <= rat)&&(rat < gc_num_list[29])) {
						result = 27;
					}
					else if((gc_num_list[29] <= rat)&&(rat < gc_num_list[30])) {
						result = 28;
					}
					else if(gc_num_list[30] <= rat) {
						result = 30;
					}
					else {
						result = 30;
					}
	}
	else {
		result = 30;
	}

//	if(result<26)
//	{
//		if(result>1+history_result||result+1<history_result)	history_result=result;
//		else 
//		{ 
//			if(result>history_result) 
//			{
//				if(rat>gc_num_list[result]+gc_rate_filter_b*0.35) history_result=result;
//				else result=history_result;//闁跨喐鏋婚幏鐑芥晸閻偅鎷濋幏宄扳偓锟�
//			} 
//		 else if(result<history_result) 
//			{
//				if(rat<gc_num_list[result]+gc_rate_filter_b*0.45) history_result=result;
//				else result=history_result;//闁跨喐鏋婚幏鐑芥晸閻偅鎷濋幏宄扳偓锟�
//			}  			
//		}//闁跨喐鏋婚幏鐑芥晸閻偅鎷濋幏宄扳偓锟�
//	}
//	else history_result=result;
//	if (result>30) result=30;
	return result;
}

/**
  * @brief  apex_depth_calculate
  * @param  
  * @retval none
  */
 int apex_depth_calculate( unsigned short int  GC_8kValue,unsigned short int GC_400Value)
{	
	int rate;
	rate=gc_calc_wl(GC_8kValue,GC_400Value);	
	return rate;	
}
/**
  * @brief  GC_depth_vlaue
  * @param  
  * @retval none
  */
int GC_depth_vlaue(int depthValue, unsigned char readOrWrite)
{
	static int GC_depth=30;
	if(readOrWrite==0) GC_depth=depthValue;//write	
	if(GC_depth<0)   GC_depth=-7;
	if(GC_depth>30)  GC_depth=30;
	return GC_depth;
}	



