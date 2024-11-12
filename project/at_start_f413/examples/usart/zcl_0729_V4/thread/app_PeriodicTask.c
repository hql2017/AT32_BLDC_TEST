
#include "app_PeriodicTask.h"
#include "app_MenuTask.h"
#include "adc_port.h"
#include "gpio_port.h"
#include "oled_port.h"
#include "para_list.h"

#include "at32f413_board.h"


	#include "FreeRTOS.h"
	#include "task.h"
	#include "queue.h"
	#include "semphr.h"
#ifdef WDT_ENABLE
	#include "event_groups.h"	
	extern EventGroupHandle_t  WDTEventGroup;
#endif
	
extern SemaphoreHandle_t xSemaphoreDispRfresh;
extern QueueHandle_t   xQueueMenuValue;
extern QueueHandle_t  xQueueKeyMessage;//key
extern QueueHandle_t  xQueueBeepMode;//beep
extern QueueHandle_t  xQueueBatValue;//batValue
extern TaskHandle_t  beepTemporaryTask_Handle;


#define BAT_STATUS_CHARGING  			0x10//insert  charging
#define BAT_STATUS_CHARGING_FULL  0x11//insert charging full
#define BAT_STATUS_CHARGING_EXIT  0x00//exit or err

extern union Param_Union sys_param_un; 
/*********************************************************************************************************//**
  * @brief  BatteryCapacityManage
	* @param  realTimeMs systemTime; perTimeMs:Periodic Time ;
  * @author 
  ***********************************************************************************************************/
void BatteryCapacityManage( unsigned int realTimeMs,unsigned int perTimeMs)
{
	static unsigned short int bat_AdcValue;
	static unsigned char sendMessage;
	static unsigned int lowpowerCount;		
	if(perTimeMs!=0&&realTimeMs%perTimeMs==0) 	
	{
		bat_AdcValue=get_vbat_value();
		if(bat_AdcValue<3000)
		{
			lowpowerCount+=perTimeMs;
			if(lowpowerCount>100)//100ms100)//500)//2s
			{
				if(sendMessage!=low_power_signal)
				{
					sendMessage=low_power_signal;
					xQueueSend(xQueueKeyMessage, &sendMessage, 0);	
				}
				else 
				{							
					if(lowpowerCount>5000)//power off£¬5s
					{
						sendMessage=power_off_signal;
						xQueueSend(xQueueKeyMessage, &sendMessage, 0);	
					}
				}			
			}	
		}
		else 
		{
			if(lowpowerCount>perTimeMs)//·À¶¶
			{
				if(lowpowerCount>100)
				{
					lowpowerCount=100;
				}
				lowpowerCount-=perTimeMs;
			}
			else 
			{
				sendMessage=null_signal;
				lowpowerCount=0;
			}			
		//send bat value		
		}	
//		bat_AdcValue=(uint16_t)Filter_VCC((double)bat_AdcValue, 0.001, 6.0, 0.0);//
		if(realTimeMs%500==0) 
		{
			xQueueSend(xQueueBatValue,&bat_AdcValue,0);
		}		
	}	
}
/*********************************************************************************************************//**
  * @brief  InsertStatutsMonitor
	* @param  perTimeMs,Periodic Time
  * @author 
  ***********************************************************************************************************/
void InsertStatusMonitor(  unsigned int realTimeMs,unsigned int perTimeMs)
{	
	unsigned char buff,value,sendBuff;
	static unsigned char recValue;	
		if(perTimeMs!=0&&realTimeMs%perTimeMs==0)	return;
		if(get_insert_state()==RESET) 
		{
			buff=0x10;
			if(get_charge_state()==SET)
			{
				value=BAT_STATUS_CHARGING;
			}
			else
			{			
				value=BAT_STATUS_CHARGING_FULL;
			}
			value=(get_charge_state()==SET)?1:0;		
			value|=buff;
		}
		else 
		{
			buff=0;
			value=0x00;
		}	
		if(recValue!=value)	 
		{						
			if(value==BAT_STATUS_CHARGING)
			{				
				#ifdef LED_INDICATE_ENABLE
					LedFunctionSet(LED_Y ,500,LED_T_HIGH_PRIORITY,LED_KEEP_ON); 
					LedFunctionSet(LED_B ,LED_OFF,LED_T_HIGH_PRIORITY,LED_KEEP_ON); 
				#endif
			}
			else if(value==BAT_STATUS_CHARGING_FULL)
			{				
				#ifdef LED_INDICATE_ENABLE
					LedFunctionSet(LED_Y ,LED_KEEP_ON,LED_T_HIGH_PRIORITY,LED_KEEP_ON); 
					LedFunctionSet(LED_B ,LED_OFF,LED_T_HIGH_PRIORITY,LED_KEEP_ON); 
				#endif
			}
			else if(value==BAT_STATUS_CHARGING_EXIT)
			{				
				#ifdef LED_INDICATE_ENABLE				
					LedFunctionSet(LED_Y ,LED_OFF,LED_T_HIGH_PRIORITY,LED_OFF); 	//exit charge
					LedFunctionSet(LED_B ,LED_OFF,LED_T_HIGH_PRIORITY,LED_OFF);
				#endif			
			}		
			if((recValue&0x10)!=buff)
			{
				if(buff==0x10)
				{
					sendBuff=MENU_CHARGING_PAGE;
				}
				else
				{
					sendBuff=MENU_HOME_PAGE;
				}								
				xQueueSend(xQueueMenuValue, &sendBuff, 0);
				sendBuff=BUZZER_MODE_BEEP;
				xQueueSend(xQueueBeepMode, &sendBuff, 0);
				sendBuff=null_signal;								
			}		
			recValue=value;		//disp
			#ifdef DEBUG_RTT
				SEGGER_RTT_WriteString(0, "insert\r\n");	
			#endif
		}	
}


/*==================================periodic  task===================================
			* @brief:  periodic task.
			* @param:  None
			* @retval: None
==============================================================================*/
void vAppPeriodicTask( void * pvParameters )
{ 	
	void *pMalloc=NULL;
	static unsigned int countPeriodicTimeMs=5;//		
#ifdef LED_INDICATE_ENABLE		
	LedFunctionSet(LED_Y ,LED_OFF,0,LED_KEEP_ON);
	LedFunctionSet(LED_B ,LED_KEEP_ON,0,LED_KEEP_ON);		
#endif	
	for(;;)
	{	
		#ifdef WDT_ENABLE
		xEventGroupSetBits(WDTEventGroup,PERIODIC_TASK_EVENT_BIT);
		#endif			
		InsertStatusMonitor( countPeriodicTimeMs,200);//300ms
		BatteryCapacityManage( countPeriodicTimeMs, 10);
		#ifdef LED_INDICATE_ENABLE		
		LedPeriodicProcessing( countPeriodicTimeMs,100);//100ms
		#endif	
		if(xSemaphoreTake(xSemaphoreDispRfresh,0))
		{
			pMalloc=pvPortMalloc(256);
			if(pMalloc!=NULL)
			{
				OLED_Refresh(sys_param_un.device_param.use_hand);
				vPortFree(pMalloc);
			}										
			#ifdef DEBUG_RTT
//		SEGGER_RTT_printf(0, "fresh time Ms %d\r\n", countPeriodicTimeMs);			
//		SEGGER_RTT_printf(0, "vat= %d\r\n", get_vbat_value());
//		SEGGER_RTT_printf(0, "vbus= %d\r\n", get_v_motor_value());
			#endif			
		}
		countPeriodicTimeMs+=5;	
		if(countPeriodicTimeMs>2000)	countPeriodicTimeMs=5;	
		vTaskDelay(5);//	1ms
	}
}

