/*
*@file        pwm_t.c
*@brief       applation
*@attention   none
*@version     V1.0
*@data        2022/5/24
*@author      xuwenxi
*/

#include "pwm_t.h"
#include "at32f413.h"
#include "at32f413_tmr.h"

//void PWM_Init(void)
//{
//  gpio_init_type gpio_init_structurt;//gpio配置结构体
//	tmr_output_config_type tmr_output_struct;//pwm输出结构体
//	tmr_brkdt_config_type tmr_brkdt_config_struct;
//	
//	
//	crm_periph_clock_enable(CRM_TMR1_PERIPH_CLOCK,TRUE);//使能定时器1的外设
//	crm_periph_clock_enable(CRM_GPIOA_PERIPH_CLOCK,TRUE);//使能GPIOA外设
//	crm_periph_clock_enable(CRM_GPIOB_PERIPH_CLOCK,TRUE);//使能GPIOB外设
// 
//  gpio_init_structurt.gpio_pins =GPIO_PINS_8|GPIO_PINS_9|GPIO_PINS_10;//GPIOA-8 9 10
//  gpio_init_structurt.gpio_mode =GPIO_MODE_MUX ;//复用推挽
//	gpio_init_structurt.gpio_drive_strength= GPIO_DRIVE_STRENGTH_STRONGER ;//输出能力
//  gpio_init_structurt.gpio_out_type =GPIO_OUTPUT_PUSH_PULL ;//推挽输出
//	gpio_init_structurt.gpio_pull =GPIO_PULL_NONE;
//  gpio_init(GPIOA,&gpio_init_structurt);
//	
//	gpio_init_structurt.gpio_pins =GPIO_PINS_13|GPIO_PINS_14|GPIO_PINS_15;//GPIOA-8 9 10
//  gpio_init_structurt.gpio_mode =GPIO_MODE_MUX ;//复用推挽
//	gpio_init_structurt.gpio_drive_strength= GPIO_DRIVE_STRENGTH_STRONGER ;//输出能力
//  gpio_init_structurt.gpio_out_type =GPIO_OUTPUT_PUSH_PULL ;//推挽输出
//	gpio_init_structurt.gpio_pull =GPIO_PULL_NONE;
//  gpio_init(GPIOB,&gpio_init_structurt);
//  
//	
//	
//  tmr_base_init(TMR1,2499,0);//配置PR和div寄存器//20khz
////	TMR1->pr = 10000;
////  TMR1->div = 0;//div=0+1
////  TMR1->swevt_bit.ovfswtr = TRUE;
//	
//	tmr_cnt_dir_set(TMR1,TMR_COUNT_TWO_WAY_2);//模式
//	
//	tmr_output_default_para_init(&tmr_output_struct);//复位
//	tmr_output_struct.oc_mode =TMR_OUTPUT_CONTROL_OFF;//TMR_OUTPUT_CONTROL_PWM_MODE_B;
//	tmr_output_struct.oc_output_state = TRUE;
//  tmr_output_struct.oc_polarity = TMR_OUTPUT_ACTIVE_HIGH;
//  tmr_output_struct.oc_idle_state = TRUE;
//  tmr_output_struct.occ_output_state = TRUE;
//  tmr_output_struct.occ_polarity = TMR_OUTPUT_ACTIVE_HIGH;
//  tmr_output_struct.occ_idle_state = TRUE;	
//	
//	tmr_output_channel_config(TMR1,TMR_SELECT_CHANNEL_1,&tmr_output_struct);	
//	tmr_channel_value_set(TMR1, TMR_SELECT_CHANNEL_1, 1250);

//	
//	tmr_output_channel_config(TMR1, TMR_SELECT_CHANNEL_2, &tmr_output_struct);
//	tmr_channel_value_set(TMR1, TMR_SELECT_CHANNEL_2, 1250);

//	
//  tmr_output_channel_config(TMR1, TMR_SELECT_CHANNEL_3, &tmr_output_struct);
//	tmr_channel_value_set(TMR1, TMR_SELECT_CHANNEL_3, 1250);

//  /*channle_4 配置*/
//	
//	tmr_output_struct.oc_mode =TMR_OUTPUT_CONTROL_OFF;
//	tmr_output_struct.oc_output_state = FALSE;
//  tmr_output_struct.oc_polarity = TMR_OUTPUT_ACTIVE_HIGH;
//  tmr_output_struct.oc_idle_state = TRUE;
//  tmr_output_struct.occ_output_state = FALSE  ;
//  tmr_output_struct.occ_polarity = TMR_OUTPUT_ACTIVE_HIGH;
//  tmr_output_struct.occ_idle_state = TRUE;
//	
//  tmr_output_channel_config(TMR1, TMR_SELECT_CHANNEL_4, &tmr_output_struct);
//	
//	tmr_channel_value_set(TMR1, TMR_SELECT_CHANNEL_4, 2498);
//  tmr_event_sw_trigger(TMR1, TMR_C4_SWTRIG);
////   tmr_interrupt_enable(TMR1, TMR_C4_INT, TRUE );//失能timer1中断 
//  
//  
//  tmr_brkdt_default_para_init(&tmr_brkdt_config_struct);
//  tmr_brkdt_config_struct.brk_enable = FALSE;
//  tmr_brkdt_config_struct.auto_output_enable = TRUE;
//  tmr_brkdt_config_struct.deadtime = 18;
//  tmr_brkdt_config_struct.fcsodis_state = TRUE;
//  tmr_brkdt_config_struct.fcsoen_state = TRUE;
//  tmr_brkdt_config_struct.brk_polarity = TMR_BRK_INPUT_ACTIVE_HIGH;
//  tmr_brkdt_config_struct.wp_level = TMR_WP_OFF;
//  tmr_brkdt_config(TMR1, &tmr_brkdt_config_struct);
////  tmr_channel_buffer_enable(TMR1, TRUE);
//  
//	  /* hall interrupt enable */
////  tmr_interrupt_enable(TMR1, TMR_HALL_INT, FALSE );//失能timer1中断

//  /* tmr1 hall interrupt nvic init */
////  nvic_priority_group_config(NVIC_PRIORITY_GROUP_4);
////  nvic_irq_enable(TMR1_TRG_HALL_TMR11_IRQn, 0, 0);
//  /* tmr1 output enable */
//  tmr_output_enable(TMR1, TRUE);

//  /* enable tmr1 */
//  tmr_counter_enable(TMR1, TRUE);
//	

//}


//void PWM_Start(void)
//{
//	tmr_output_channel_mode_select(TMR1, TMR_SELECT_CHANNEL_1, TMR_OUTPUT_CONTROL_PWM_MODE_A);
//	tmr_output_channel_mode_select(TMR1, TMR_SELECT_CHANNEL_2, TMR_OUTPUT_CONTROL_PWM_MODE_A);
//	tmr_output_channel_mode_select(TMR1, TMR_SELECT_CHANNEL_3, TMR_OUTPUT_CONTROL_PWM_MODE_A);
//	tmr_output_channel_mode_select(TMR1, TMR_SELECT_CHANNEL_4, TMR_OUTPUT_CONTROL_PWM_MODE_B);

//	tmr_channel_enable(TMR1, TMR_SELECT_CHANNEL_1, TRUE);
//  tmr_channel_enable(TMR1, TMR_SELECT_CHANNEL_1C,TRUE);
//	
//	tmr_channel_enable(TMR1, TMR_SELECT_CHANNEL_2, TRUE);
//  tmr_channel_enable(TMR1, TMR_SELECT_CHANNEL_2C,TRUE);

//	tmr_channel_enable(TMR1, TMR_SELECT_CHANNEL_3, TRUE);
//  tmr_channel_enable(TMR1, TMR_SELECT_CHANNEL_3C,TRUE);
//	
//	tmr_channel_enable(TMR1, TMR_SELECT_CHANNEL_4, TRUE);
//}


//void PWM_Stop(void)
//{
//		tmr_channel_enable(TMR1, TMR_SELECT_CHANNEL_1, FALSE);
//		tmr_channel_enable(TMR1, TMR_SELECT_CHANNEL_1C,FALSE);
//		tmr_channel_enable(TMR1, TMR_SELECT_CHANNEL_2, FALSE);
//		tmr_channel_enable(TMR1, TMR_SELECT_CHANNEL_2C,FALSE);
//		tmr_channel_enable(TMR1, TMR_SELECT_CHANNEL_3, FALSE);
//		tmr_channel_enable(TMR1, TMR_SELECT_CHANNEL_3C,FALSE);

//		tmr_channel_enable(TMR1, TMR_SELECT_CHANNEL_4, FALSE);
//}
//void PWM_SET_UVW( uint16_t UU, uint16_t VV, uint16_t WW)
//{
//	TMR1->c1dt = UU;
//	TMR1->c2dt = VV;
//	TMR1->c3dt = WW;
//}
//void PWM_set(uint8_t num,uint16_t p)
//{
//  switch(num)
//	{
//	  case 1:
//			TMR1->c1dt = p;break;
//		
//		case 2:
//			TMR1->c2dt = p;break;
//		
//		case 3:
//			TMR1->c3dt = p;break;
//		
//		default:break;
//	
//	}
//}
