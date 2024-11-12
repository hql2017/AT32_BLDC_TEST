/*
*@file        time_init.c
*@brief       applation
*@attention   none
*@version     V1.0
*@data        2022/5/24
*@author      xuwenxi
*/
#include "time_init.h"
#include "at32f413.h"

//TIMER
#define SYSTEM_COUNTER_TIMER   TMR4
#define MOTOR_TIMER            TMR3

/**
  * @brief  motor base timer init
  * @param  uint32_t periodicTimeUs,uint32_t SystemClockFrequency
  * @retval none
  */
void MotorTimerInit(unsigned int periodicTimeUs,unsigned int systemClockFrequency)
{
	unsigned int Run_mhz,count_value;
	unsigned char div_value;
	
	Run_mhz=1000000;//
	count_value=periodicTimeUs-1;//systemclock=200M
	div_value=((systemClockFrequency)/Run_mhz)-1;//1us
	
  crm_periph_clock_enable(CRM_TMR3_PERIPH_CLOCK, TRUE);

	
	tmr_base_init(TMR3,count_value, div_value);
  tmr_cnt_dir_set(TMR3, TMR_COUNT_UP);

  /* overflow interrupt enable */
	tmr_interrupt_enable(TMR3, TMR_OVF_INT, TRUE);	

  /* tmr1 overflow interrupt nvic init */
  nvic_irq_enable(TMR3_GLOBAL_IRQn, 4, 0);

  /* enable tmr1 */
  tmr_counter_enable(TMR3,FALSE);	
}
/**
  * @brief  NormalTimerInit
  * @param  uint32_t periodicTimeUs,uint32_t SystemClockFrequency
  * @retval none
  */
void NormalTimerInit(unsigned int periodicTimeMs,unsigned int systemClockFrequency)
{
	unsigned int Run_mhz,count_value;
	unsigned char div_value;
	
	Run_mhz=1000000;//1m
	count_value=periodicTimeMs*1000-1;//systemclock=200M
	div_value=((systemClockFrequency)/Run_mhz)-1;//1us
	
  crm_periph_clock_enable(CRM_TMR4_PERIPH_CLOCK, TRUE);
	
	tmr_base_init(TMR4,count_value, div_value);
  tmr_cnt_dir_set(TMR4, TMR_COUNT_UP);

  /* overflow interrupt enable */
	tmr_interrupt_enable(TMR4, TMR_OVF_INT, TRUE);	

  /* tmr1 overflow interrupt nvic init */
  nvic_irq_enable(TMR4_GLOBAL_IRQn, 4, 0);

  /* enable tmr1 */
  tmr_counter_enable(TMR4,FALSE);	
}
/**
  * @brief  config system timer
  * @param  uint32_t periodicTimeMS,uint32_t SystemClockFrequency
  * @retval none
  */
void ConfigSys_Timer(unsigned int seriodicTimeMs,unsigned int systemClockFrequency)
{
  unsigned int Run_mhz,count_value;
	unsigned char div_value;
	
	Run_mhz=1000000;//¼ÆÊýÆµÂÊ1M
	count_value=seriodicTimeMs*1000-1;
	div_value=((systemClockFrequency)/Run_mhz)-1;//1us
	
  crm_periph_clock_enable(CRM_TMR4_PERIPH_CLOCK, TRUE);
  
  tmr_base_init(TMR4,count_value,div_value);
  tmr_cnt_dir_set(TMR4, TMR_COUNT_UP);
  tmr_interrupt_enable(TMR4, TMR_OVF_INT, TRUE);	
  nvic_irq_enable(TMR4_GLOBAL_IRQn, 0, 0);
	tmr_counter_enable(TMR4, TRUE);
}

