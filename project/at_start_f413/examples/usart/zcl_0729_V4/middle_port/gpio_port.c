
#include "stdint.h"
#include "at32f413_board.h" 
#include "gpio_port.h"
#ifdef LED_INDICATE_ENABLE
#define LED_ON  1//unstill on
#define LED_BREATH 2
#define LED_NOT_ACTION 3 //未执行

#define MAX_LED_NUM   2
#define MAX_LED_TASK   2//多种运行模式

typedef struct {	
	uint8_t 	contextualModel;//情景模式
	uint16_t  keepTime;	//breath
}LED_STATUS;

typedef struct {
	LED_STATUS led_status[MAX_LED_TASK];
	uint8_t ledOnOff;//run  status
	uint16_t  subTime;//run time
	uint8_t  priority;	
	uint16_t stillTime;	//持续时间
}LED_Task_Que;
static LED_Task_Que led_que[MAX_LED_NUM]={0};

#endif
/**
  * @brief  GPIO Init
  * @param  none
  * @retval none
  */
void user_gpio_init(void)
{
  gpio_init_type gpio_init_struct;
//sys_on  
  crm_periph_clock_enable(SYS_ON_CLOCK, TRUE);
//gpio_pin_remap_config(SWJTAG_GMUX_010, FALSE);
  gpio_default_para_init(&gpio_init_struct);
  gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
  gpio_init_struct.gpio_out_type  = GPIO_OUTPUT_PUSH_PULL;
  gpio_init_struct.gpio_mode = GPIO_MODE_OUTPUT;
  gpio_init_struct.gpio_pins = SYS_ON_IO;
  gpio_init_struct.gpio_pull = GPIO_PULL_UP;
  gpio_init(SYS_ON_PORT, &gpio_init_struct);		
//insert
  crm_periph_clock_enable(INSERT_CLOCK, TRUE);
  gpio_default_para_init(&gpio_init_struct);
  gpio_init_struct.gpio_mode = GPIO_MODE_INPUT;
  gpio_init_struct.gpio_pins = INSERT_IO;
  gpio_init_struct.gpio_pull = GPIO_PULL_NONE;//GPIO_PULL_UP;
  gpio_init(INSERT_PORT, &gpio_init_struct);
//charge gpio
  crm_periph_clock_enable(CHR_CLOCK, TRUE);
  gpio_default_para_init(&gpio_init_struct);
  gpio_init_struct.gpio_mode = GPIO_MODE_INPUT;
  gpio_init_struct.gpio_pins = CHR_IO;
  gpio_init_struct.gpio_pull = GPIO_PULL_NONE;//GPIO_PULL_UP;
  gpio_init(CHR_PORT, &gpio_init_struct);  
#ifdef LED_INDICATE_ENABLE
//led
  crm_periph_clock_enable(LEDB_CLOCK, TRUE);
  gpio_default_para_init(&gpio_init_struct);
  gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
  gpio_init_struct.gpio_out_type  = GPIO_OUTPUT_PUSH_PULL;
  gpio_init_struct.gpio_mode = GPIO_MODE_OUTPUT;
  gpio_init_struct.gpio_pins = LEDB_IO;
  gpio_init_struct.gpio_pull = GPIO_PULL_UP;
  gpio_init(LEDB_PORT, &gpio_init_struct);
	//close
  gpio_bits_reset(LEDB_PORT,LEDB_IO);
	
	//led
  crm_periph_clock_enable(LEDG_CLOCK, TRUE);
  gpio_default_para_init(&gpio_init_struct);
  gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
  gpio_init_struct.gpio_out_type  = GPIO_OUTPUT_PUSH_PULL;
  gpio_init_struct.gpio_mode = GPIO_MODE_OUTPUT;
  gpio_init_struct.gpio_pins = LEDG_IO;
  gpio_init_struct.gpio_pull = GPIO_PULL_UP;
  gpio_init(LEDG_PORT, &gpio_init_struct);
	//close
  gpio_bits_reset(LEDG_PORT,LEDG_IO);
	
	
#endif

//...init... 
	device_power_off(); 	//power off
	
}

/**
  * @brief  device power on
  * @param  none
  * @retval none
  */
void device_power_on(void)
{
		gpio_bits_reset(SYS_ON_PORT, SYS_ON_IO);
}
/**
  * @brief  device power off
  * @param  none
  * @retval none
  */
void device_power_off(void)
{
		gpio_bits_set(SYS_ON_PORT, SYS_ON_IO);
}
#ifdef  APEX_FUNCTION_EBABLE
/**input
  * @brief  get gpio  status
  * @param  GPIO level
	* @retval  status: SET or RESET
  */
flag_status get_gc_state(void)
{
   flag_status re;
   re=gpio_input_data_bit_read(GPIOA,GPIO_PINS_9);//GPIO_PINS_9);
   return re;
}
#endif
/**input
  * @brief  get gpio  status
  * @param  GPIO level
	* @retval  status: SET or RESET
  */
flag_status get_run_button_state(void)
{
   flag_status re;
   re=gpio_input_data_bit_read(PWR_SW_PORT,PWR_SW_IO);
   return re;
}

flag_status get_add_button_state(void)
{
   flag_status re;
   re=gpio_input_data_bit_read(KEY_A_PORT,KEY_A_IO);
   return re;
}

flag_status get_sub_button_state(void)
{
   flag_status re;
   re=gpio_input_data_bit_read(KEY_D_PORT,KEY_D_IO);
   return re;
}

flag_status get_s_button_state(void)
{
   flag_status re;
   re=gpio_input_data_bit_read(KEY_S_PORT,KEY_S_IO);
   return re;
}

flag_status get_insert_state(void)
{
   flag_status re;
   re=gpio_input_data_bit_read(INSERT_PORT,INSERT_IO);
   return re;
}

flag_status get_charge_state(void)
{
   flag_status re;
   re=gpio_input_data_bit_read(CHR_PORT,CHR_IO);
   return re;
}

/**
  * @brief  toggle GPIO level
  * @param  goio, pin
  * @retval none
  */
void gpio_pins_toggle(gpio_type* gpio_x, unsigned short int gpio_pin)
{
  gpio_x->odt ^= gpio_pin;
}
#ifdef LED_INDICATE_ENABLE
//====================LED api=================================================================
/**
  * @brief  led action
  * @param  led number,commands
  * @retval none
  */
static void led_op(uint8_t num,uint8_t onoff)
{	
  if(num==0){     		
		if(onoff==LED_ON)    {  gpio_bits_set(LEDG_PORT,LEDG_IO); 	 }			
		else if(onoff==LED_OFF)   {  gpio_bits_reset(LEDG_PORT,LEDG_IO);   }
		else if(onoff==LED_BREATH)   {  gpio_pins_toggle(LEDG_PORT, LEDG_IO);  }
	}
	else if(num==1){
		if(onoff==LED_ON)   { gpio_bits_set(LEDB_PORT,LEDB_IO);	}			
		else if(onoff==LED_OFF)   { gpio_bits_reset(LEDB_PORT,LEDB_IO); }
		else if(onoff==LED_BREATH)     { gpio_pins_toggle(LEDB_PORT, LEDB_IO); }
	}
}
/*********************************************************************************************************//**
  * @brief  LedPeriodicProcessing
	* @param  perTimeMs,Periodic Time
  * @author 
  ***********************************************************************************************************/
void LedPeriodicProcessing( unsigned short int realTimeMs,unsigned short int perTimeMs)
{
	uint8_t i;	

	if(perTimeMs==0) return ;//forbid
	if(realTimeMs%perTimeMs==0)
	{
		for(i=0;i<MAX_LED_NUM;i++)
		{
			if(led_que[i].stillTime==0){
				led_que[i].priority=LED_T_IDLE;		//				
			}	
			else 
			{	
				if(led_que[i].stillTime<LED_KEEP_ON)//
				{
					if(led_que[i].stillTime>=perTimeMs)	{led_que[i].stillTime-=perTimeMs;}
					else
					{ 
						led_que[i].stillTime=0;  
						if(led_que[i].priority!=LED_T_IDLE)
						{
							led_que[i].ledOnOff=LED_OFF;
							led_que[i].priority=LED_T_IDLE; 
							led_op(i,LED_OFF); 						
						}					
					}//reset status		
				}												
			}	
			if(led_que[i].led_status[led_que[i].priority].contextualModel==LED_BREATH)  
			{	
				led_que[i].ledOnOff=LED_BREATH;
				led_que[i].subTime+=perTimeMs;  
				if(led_que[i].subTime>=led_que[i].led_status[led_que[i].priority].keepTime)	
				{
					led_que[i].subTime=0;
					led_op(i,LED_BREATH);					
				}
			}	
			else if(led_que[i].ledOnOff!=led_que[i].led_status[led_que[i].priority].contextualModel)
			{
				led_que[i].ledOnOff=led_que[i].led_status[led_que[i].priority].contextualModel;
				led_op(i,led_que[i].ledOnOff);	
			}
		}  
	}	
}
/*********************************************************************************************************//**
  * @brief  LedFunctionSet 
	* @param  num，led编号 keepTimeMs呼吸间隔，led_priority优先级（0为最低优先级，空闲执行） StillTimeMs持续时间
  * @author 
  ***********************************************************************************************************/
void LedFunctionSet(unsigned char num ,unsigned short int  keepTimeMs,unsigned char ledPriority,unsigned short int stillTimeMs)
{
	if(num>MAX_LED_NUM)  return ;		
	if(ledPriority>=led_que[num].priority||stillTimeMs==0) 
	{	
		led_que[num].priority  = ledPriority; 
		led_que[num].stillTime = stillTimeMs;		
	}	
	led_que[num].led_status[ledPriority].keepTime=keepTimeMs;
	if(keepTimeMs==LED_OFF) led_que[num].led_status[ledPriority].contextualModel=LED_OFF;
	else if(keepTimeMs==LED_KEEP_ON) 
	{
		led_que[num].led_status[ledPriority].contextualModel=LED_ON;
	}
	else 
	{
		led_que[num].led_status[ledPriority].contextualModel=LED_BREATH;
		led_que[num].subTime=0;
	}//breath
}
#endif
