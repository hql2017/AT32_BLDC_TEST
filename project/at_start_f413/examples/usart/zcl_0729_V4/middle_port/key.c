
#include "gpio_port.h"
#include "at32f413_board.h" 
#include "key.h"

#include "para_list.h"

//4 bit 0~7
#define   BUTTON_EVENT_NONE   							0
#define   BUTTON_EVENT_CLICK  							1
#define   BUTTON_EVENT_LONG_PRESS           2
#define   BUTTON_EVENT_PRESS   							3//(CLICK to long press status)
#define   BUTTON_EVENT_DOUBLE_CLICK  				4
#define   BUTTON_EVENT_LONG_PRESS_RELEASE  	5   
#define   BUTTON_EVENT_TWICE_LONG_PRESS     6//long press two times
#define   BUTTON_EVENT_THRICE_LONG_PRESS  	7//long press three times

 typedef enum{
	RUN_BOTTON=0,
	ADD_BOTTON,
	SUB_BOTTON, 
	S_BOTTON,
	MAX_BUTTON_NUM//<8
}key_head;
typedef struct{
		unsigned char buttonEnable;
		unsigned char buttonLongPressEnable;
}BUTTON_CONFIG;

typedef struct{
	unsigned char buttonHead;		
	unsigned char buttonEvent;	
	unsigned int timeCount;
	unsigned char longPressCount;
	unsigned char reserve1;
	BUTTON_CONFIG cfg;
}KEY_STU;

static KEY_STU  key_t[MAX_BUTTON_NUM]={0};

static  error_status FlexButtonRegister(unsigned char buttonId,unsigned char buttonEnable,unsigned char longPressEnable);


//=========================key api======================================================================	

	/**
  * @brief  ButtonGPIO_INIT,硬件相关
  * @param  
  * @retval none
  */
static void FlexButtonInit(unsigned char button_num)
{	
	gpio_init_type gpio_init_struct;
	if(button_num==RUN_BOTTON)
	{
		crm_periph_clock_enable(PWR_SW_CLOCK, TRUE);
		gpio_default_para_init(&gpio_init_struct);
		gpio_init_struct.gpio_mode = GPIO_MODE_INPUT;
		gpio_init_struct.gpio_pins = PWR_SW_IO;
		gpio_init_struct.gpio_pull = GPIO_PULL_UP;
		gpio_init(PWR_SW_PORT, &gpio_init_struct);	 
	}
	else if(button_num==ADD_BOTTON)
	{
		crm_periph_clock_enable(KEY_A_CLOCK, TRUE);
		gpio_default_para_init(&gpio_init_struct);
		gpio_init_struct.gpio_mode = GPIO_MODE_INPUT;
		gpio_init_struct.gpio_pins = KEY_A_IO;
		gpio_init_struct.gpio_pull = GPIO_PULL_UP;
		gpio_init(KEY_A_PORT, &gpio_init_struct);  
	}
	else if(button_num==SUB_BOTTON)
	{	
		crm_periph_clock_enable(KEY_D_CLOCK, TRUE);
		gpio_default_para_init(&gpio_init_struct);
		gpio_init_struct.gpio_mode = GPIO_MODE_INPUT;
		gpio_init_struct.gpio_pins = KEY_D_IO;
		gpio_init_struct.gpio_pull = GPIO_PULL_UP;
		gpio_init(KEY_D_PORT, &gpio_init_struct);		
	}
	else if(button_num==S_BOTTON)
	{
		crm_periph_clock_enable(KEY_S_CLOCK, TRUE);
		gpio_default_para_init(&gpio_init_struct);
		gpio_init_struct.gpio_mode = GPIO_MODE_INPUT;
		gpio_init_struct.gpio_pins = KEY_S_IO;
		gpio_init_struct.gpio_pull = GPIO_PULL_UP;
		gpio_init(KEY_S_PORT, &gpio_init_struct); 
	}	
}
	/**
  * @brief  ButtonGPIO_INIT,硬件相关
  * @param  
  * @retval none
  */
static  flag_status GetButtonLevel(unsigned char button_num)
{	
	flag_status ret;
	if(button_num==RUN_BOTTON)
	{
		ret=get_run_button_state();	  
	}
	else if(button_num==ADD_BOTTON)
	{
			ret=get_add_button_state();	 
	}
	else if(button_num==SUB_BOTTON)
	{
			ret=get_sub_button_state();	 
	}
	else if(button_num==S_BOTTON)
	{
			ret=get_s_button_state();	 
	}	
	return ret;
}
	/**
  * @brief  ButtonRegister
  * @param  
  * @retval none
  */
error_status FlexButtonRegister(unsigned char buttonId,unsigned char buttonEnable,unsigned char longPressEnable)
{	
	error_status err;
	if(buttonId>=MAX_BUTTON_NUM) err=ERROR;
	else
	{
		FlexButtonInit(buttonId);	
		key_t[buttonId].cfg.buttonEnable=buttonEnable;
		key_t[buttonId].cfg.buttonLongPressEnable=longPressEnable;
		err=SUCCESS;
	}
	return err;	
}

		/**
  * @brief  KeyHandle
  * @param  
  * @retval none
  */
static unsigned char KeyHandle(unsigned  int keyValue)
{
	unsigned char message=null_signal;	
	switch(keyValue)
	{
		case 0:	
			message=null_signal;
					//none			
			break;
		case 0x00000001:
				message=run_button_press_signal;
			//run press
			break;
		case 0x00000002:
//		message=run_button_long_press_signal;
//			message=power_off_signal;
		//power off signal		
			break;
		case 0x00000005:
			message=run_button_release_signal;
			//run release 
			break;	
		case 0x00000006:
			message=run_button_long_press_signal;//longpress two seconds//延长开关按键时间
			//			message=power_off_signal;
			break;
		case 0x00000010:
				message=add_button_press_signal;
			break;
		case 0x00000020:
				message=add_button_long_press_signal;
			break;
		case 0x00000100:
				message=sub_button_press_signal;			
			break;
		case 0x00000200:
				message=sub_button_long_press_signal;
			break;
		case 0x00001000:
				message=s_button_press_signal;
			break;
		case 0x00002000:
				message=s_button_long_press_signal;
			break;
		case 0x00000660://同时长按+and-
//			message=start_MP6570_AutoThetaBias_signal;
		break;		
		default:
		//组合键等
		//nothing
			break;
	}
	return message;
}
		/**
  * @brief button Init
  * @param  
  * @retval none
  */
void ButtonInit(void)
{
	FlexButtonRegister(RUN_BOTTON,1,1);
	FlexButtonRegister(ADD_BOTTON,1,1);
	FlexButtonRegister(SUB_BOTTON,1,1);
	FlexButtonRegister(S_BOTTON,1,1);
}	
		/**
  * @brief  getButtonEvent
  * @param  
  * @retval none
  */
unsigned char GetButtonEvent(unsigned char buttonId)
{
	unsigned char buttonEvent;
	if(key_t[buttonId].cfg.buttonEnable==0) 
	{		
		buttonEvent=BUTTON_EVENT_NONE;
	}
	else 
	{
		buttonEvent=key_t[buttonId].buttonEvent;		
	}
	key_t[buttonId].buttonEvent=BUTTON_EVENT_NONE;
	return buttonEvent;
}	
	/**
  * @brief  ButtonScan
  * @param  
  * @retval none
  */
 unsigned char ButtonScan(unsigned int timeMs)
{	
	uint8_t i,message=0;	
	static uint32_t recKeyValue=0;
	uint32_t keyValue=0,keyValueBuff[MAX_BUTTON_NUM]={0};
	for(i=0;i<MAX_BUTTON_NUM;i++)
	{
		if(key_t[i].cfg.buttonEnable)	
		{	
				if(GetButtonLevel(i)==RESET) 
				{
					key_t[i].timeCount+=timeMs;
					if(key_t[i].timeCount>1000)
					{
						key_t[i].timeCount=0;
						if(key_t[i].cfg.buttonLongPressEnable)
						{
							key_t[i].longPressCount++;
							if(key_t[i].longPressCount>250) key_t[i].longPressCount=250;							
							if(key_t[i].longPressCount>1) 
							{								
								key_t[i].buttonEvent=BUTTON_EVENT_TWICE_LONG_PRESS;
							}
							else  
							{
								key_t[i].buttonEvent=BUTTON_EVENT_LONG_PRESS;
							}
						}
					}
				}
				else 
				{	
					if(key_t[i].longPressCount>0) 
					{ 
						if(key_t[i].cfg.buttonLongPressEnable)
						{
							key_t[i].buttonEvent=BUTTON_EVENT_LONG_PRESS_RELEASE;
						}
					}	
					else
					{
						if(key_t[i].timeCount>100)
						{
							key_t[i].buttonEvent=BUTTON_EVENT_CLICK;
							//else if(key_t[i].timeCount>500&&key_t[i].timeCount<1000)
							//{
							//	key_t[i].buttonEvent=BUTTON_EVENT_PRESS;//one status and no handle
							//}
						}
						else
						{
							key_t[i].buttonEvent=BUTTON_EVENT_NONE;	
						}
					}
					key_t[i].timeCount=0;
					key_t[i].longPressCount=0;
				}				
				keyValueBuff[i]=((0x0000000F&key_t[i].buttonEvent)<<(4*i));	
				keyValue|=keyValueBuff[i];//组合键
			}	
	}		
	if(recKeyValue!=keyValue)//组合键
	{
		recKeyValue=keyValue;
		message=KeyHandle(recKeyValue);//同步处理				
	}		
	return message;		
}


