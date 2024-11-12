#include <stdio.h>
#include "limits.h"
#include "buzz_port.h"
#include "gpio_port.h"
#include "pwm_t.h"
#include "system_at32f413.h"
//#include "at32f413.h"
//#include "at32f413_tmr.h"


const unsigned short int music_tab_a[]={
	0,
	221,248,278,294,330,371,416,//µÕ“Ù1-7
	441,495,556,589,661,742,833,//÷–“Ù1-7
	882,990,1112,1178,1322,1484,1665//∏ﬂ“Ù1-7
};

crm_clocks_freq_type crm_clocks_freq_struct = { 0 };


static uint16_t beep_frequence =4000;// 4000;//beep  highest vol  f=4000(4k~5K)(4k70db,5k 60db);middle vol  f=1000(60db);

/**
  * @brief  pwm for buzzer
  * @param  none
  * @retval none
  */
void BuzzerInit(void)
{
	uint16_t div_value = 0 ;//tmer clock div value
	uint16_t tmr_count_val = 390 ;//timer count value
  uint16_t ch1_val = 250 ;//PWM output toggle value
	
  gpio_init_type gpio_init_structurt;//gpio structure
	tmr_output_config_type tmr_oc_init_structure;
	
	crm_periph_clock_enable(CRM_GPIOB_PERIPH_CLOCK,TRUE);

  gpio_init_structurt.gpio_pins =GPIO_PINS_8;//GPIOB8 buzz
  gpio_init_structurt.gpio_mode =GPIO_MODE_MUX ;
	gpio_init_structurt.gpio_drive_strength= GPIO_DRIVE_STRENGTH_STRONGER ;
  gpio_init_structurt.gpio_out_type =GPIO_OUTPUT_PUSH_PULL ;
	gpio_init_structurt.gpio_pull =GPIO_PULL_UP;
  gpio_init(GPIOB,&gpio_init_structurt);
	
	gpio_bits_reset(GPIOB, GPIO_PINS_8);//close
	
	 /* compute the div value */
  div_value = (uint16_t)((system_core_clock) / 1000000) - 1;// div
		
	tmr_count_val=((system_core_clock/(div_value+1))/beep_frequence)-1;//count frequency 4K
	ch1_val=tmr_count_val/2;//duty ratio

  /* tmr10 time base configuration */
	crm_periph_clock_enable(CRM_TMR10_PERIPH_CLOCK,TRUE);
  tmr_base_init(TMR10, tmr_count_val, div_value);
  tmr_cnt_dir_set(TMR10, TMR_COUNT_UP);
  tmr_clock_source_div_set(TMR10, TMR_CLOCK_DIV1);

  tmr_output_default_para_init(&tmr_oc_init_structure);
  tmr_oc_init_structure.oc_mode = TMR_OUTPUT_CONTROL_PWM_MODE_B;
  tmr_oc_init_structure.oc_idle_state = FALSE;
  tmr_oc_init_structure.oc_polarity = TMR_OUTPUT_ACTIVE_LOW;
  tmr_oc_init_structure.oc_output_state = TRUE;
  tmr_output_channel_config(TMR10, TMR_SELECT_CHANNEL_1, &tmr_oc_init_structure);
  tmr_channel_value_set(TMR10, TMR_SELECT_CHANNEL_1, ch1_val);
  tmr_output_channel_buffer_enable(TMR10, TMR_SELECT_CHANNEL_1, TRUE);

  tmr_period_buffer_enable(TMR10, TRUE);

  /* tmr enable counter */
//  tmr_counter_enable(TMR10, TRUE);
	BuzzrSimpleSwitch(0);//close buzzer
}
/**
  * @brief  frequency setting for music
  * @param  
  * @retval none
  */
 void buzz_frequency_set(unsigned short int frequency,unsigned char vol)
{
	uint16_t div_value = 0 ;//tmer clock div value
	uint16_t tmr_count_val = 390 ;//timer count value	
//	uint16_t ch1_val = 250 ;//PWM output toggle value
	 /* compute the div value */
  div_value = (uint16_t)((system_core_clock) / 1000000) - 1;// div
		
	tmr_count_val=((system_core_clock/(div_value+1))/frequency)-1;//count frequency 4K
	
	tmr_base_init(TMR10, tmr_count_val, div_value);	
	//duty ratio
	//	ch1_val=tmr_count_val/2;//duty ratio
	//	tmr_channel_value_set(TMR10, TMR_SELECT_CHANNEL_1, ch1_val);
}
/**
  * @brief  simple Switch for buzzer
  * @param  
  * @retval none
  */
 void BuzzrSimpleSwitch(unsigned char OnOff)
{
	if(OnOff==0)//close
	{
		tmr_counter_enable(TMR10, FALSE);		
		crm_periph_clock_enable(CRM_TMR10_PERIPH_CLOCK,FALSE);		
	}
	else 
	{		
		crm_periph_clock_enable(CRM_TMR10_PERIPH_CLOCK,TRUE);
		tmr_counter_enable(TMR10, TRUE);
	}
}





