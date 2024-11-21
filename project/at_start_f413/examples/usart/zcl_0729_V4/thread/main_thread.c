
#include "math.h"
#include "string.h"
#include "stdlib.h"
#include "para_list.h"
#include "at32f413_board.h"
#include "oled_port.h"
#include "gpio_port.h"
#include "buzz_port.h"
#include "apex_gc_port.h"
#include "adc_port.h"

#include "app_MotorControlTask.h"
#include "customer_control.h"
#include "macros.h"
#include "delay.h"

#include "key.h"
	#include "FreeRTOS.h"
	#include "task.h"
	#include "queue.h"
	#include "semphr.h"
#ifdef WDT_ENABLE
	#include "event_groups.h"	
	EventGroupHandle_t  WDTEventGroup;
#endif

	#define KEY_SCAN_STACK_DEPTH  64
	#define KEY_SCAN_TASK_PRIORITY 	TASK_PRIORITY_HIGH
	TaskHandle_t  keyScanTask_Handle;
	
	#define PERIODIC_STACK_DEPTH 256
	#define PERIODIC_TASK_PRIORITY 	TASK_PRIORITY_MIDDLE
	TaskHandle_t  periodicTask_Handle; //周期任务	
	
	#define APEX_GC_STACK_DEPTH  128
	#define APEX_GC_TASK_PRIORITY 	TASK_PRIORITY_MIDDLE
	TaskHandle_t  ApexGC_Task_Handle; //
	
	#define MENU_MANAGE_STACK_DEPTH 256
	#define MENU_MANAGE_TASK_PRIORITY 	TASK_PRIORITY_MIDDLE
	TaskHandle_t  menuManageTask_Handle; //周期任务	

	#define MOTOR_CONTROL_STACK_DEPTH 128
	#define MOTOR_CONTROL_TASK_PRIORITY 	TASK_PRIORITY_MIDDLE
	TaskHandle_t  motorControlTask_Handle;
	
	#define SECONDARY_BEEP_STACK_DEPTH  32
	#define SECONDARY_BEEP_TASK_PRIORITY 	TASK_PRIORITY_MIDDLE//(TASK_PRIORITY_LOWEST-1)
	TaskHandle_t  beepTemporaryTask_Handle;	
	
	#define WDT_MANAGE_STACK_DEPTH  32
	#define WDT_MANAGE_TASK_PRIORITY 	(TASK_PRIORITY_MIDDLE)
	TaskHandle_t  WDT_ManageTask_Handle;
	
	QueueHandle_t  xQueueKeyMessage;//key
	QueueHandle_t  xQueueBeepMode;//beep
	QueueHandle_t  xQueueMenuValue;//menu	
	QueueHandle_t  xQueueBatValue;//batValue	

	
	SemaphoreHandle_t xSemaphoreDispRfresh;			
	SemaphoreHandle_t xSemaphorePowerOff;
	#ifdef  APEX_FUNCTION_EBABLE


	SemaphoreHandle_t xSemaphoreApexAutoCalibration;
	#endif
	
	extern void vAppPeriodicTask( void * pvParameters );	
	extern void vAppMotorControlTask( void * pvParameters );
	extern void vAppMenuManageTask( void * pvParameters );
	extern void MenuDevicePowerOff(unsigned char feedDogFlag);
	
#if configUSE_TIMERS
TimerHandle_t xTimer01;
#endif

#ifdef  APEX_FUNCTION_EBABLE
#define MIN_GC_PWM_VALID_TIME  40//Minimum switching time Ms(40Hz>30  8000>20)
#define GC_ADC_DELAY_TIME  3//采集延迟

uint16_t   apex_ValueBuff[8]={0};  //根尖参考值
/**
  * @brief 	GC_PWM_SwitchTimeManage
  * @param  systemTime
  * @retval none
  */
static error_status  GC_PWM_SwitchTimeManage(unsigned int systemTime,unsigned int targetTime)
{	
	error_status err;	
	static  unsigned int recTicks;
	err=ERROR;
	if(targetTime==0) 
	{
		recTicks=systemTime;		
	}
	else
	{
		
		if(recTicks>systemTime) 
		{
			recTicks=systemTime;
		}
		if(systemTime-recTicks>targetTime)
		{
			recTicks=systemTime;
			err=SUCCESS;
		}		
	}	
	return err;
}

#endif


/*==================================GC task===================================
			* @brief:  GC task.
			* @param:  None
			* @retval: None
==============================================================================*/
void vApexGC_Task( void * pvParameters )
{ 	
	uint32_t countTimeMs=0;
	unsigned  int timeOut;
	unsigned  char i,sendBuff;	
	#ifdef  APEX_FUNCTION_EBABLE
	static uint8_t apexGC_Flag=1;	
	static unsigned short int GC_Frequency=8000;
	unsigned  int GC_8k_AdcValue=0;
	unsigned  int GC_400_AdcValue=1;
	unsigned  short int        rec_gc_rate;  //当前rate
	static 	int  GC_depth=30;//初始值
	gc_in_or_exit(APEX_GC_IN);
	#endif		
	for(;;)
	{		
		countTimeMs++;	
		#ifdef DEBUG_RTT
			if(countTimeMs>200)//1s
			{
				countTimeMs=0;
			
					SEGGER_RTT_printf(0, "gc8k= %d gc400=%d GC_depth=%d\r\n",GC_8k_AdcValue, GC_400_AdcValue,GC_depth);
			}			
		#endif
		#ifdef  APEX_FUNCTION_EBABLE		
		if(xSemaphoreTake(xSemaphoreApexAutoCalibration,0))
		{	
			apexGC_Flag=2;
		}		
		if(apexGC_Flag==1)//开机校准
		{
			gc_in_or_exit(APEX_GC_IN);
			GC_Frequency=8000;
			apex_frequency_set(GC_Frequency);
			GC_PWM_SwitchTimeManage(xTaskGetTickCount(),0);//restart	
			vTaskDelay(4);
			for(i=0;i<8;i++)//i<8;i++)
			{
				vTaskDelay(MIN_GC_PWM_VALID_TIME);
				restart_adc_sampling();
				timeOut=0;						
				while(fresh_adc_value()==0)	//wait collect finished
				{
					timeOut++;
					if(timeOut%48000==0)  taskYIELD();//exit task
					if(timeOut>144000) break;//3ms	
				}
				fresh_adc_value();//get latest value			
				if(GC_Frequency==8000)
				{
					GC_8k_AdcValue=get_gc_8k_value();	
					apex_ValueBuff[i]		=	GC_8k_AdcValue;			
					GC_Frequency=400;
					apex_frequency_set(GC_Frequency);	
				}
				else if(GC_Frequency==400)
				{
					GC_400_AdcValue=get_gc_400_value();								
					apex_ValueBuff[i]		=	GC_400_AdcValue;							
					GC_Frequency=8000;
					apex_frequency_set(GC_Frequency);
				}
				
				#ifdef WDT_ENABLE
				xEventGroupSetBits(WDTEventGroup,APEX_TASK_EVENT_BIT);
				#endif				
			}
			GC_8k_AdcValue=0;
			GC_400_AdcValue=0;			
			for(i=0;i<4;i++)	
			{
				GC_8k_AdcValue+=apex_ValueBuff[2*i]	;
				GC_400_AdcValue+=apex_ValueBuff[2*i+1]	;
			}		
			GC_400_AdcValue=	GC_400_AdcValue>>2;
			GC_8k_AdcValue=(GC_8k_AdcValue>>2);	
      if(GC_400_AdcValue==0)	 GC_400_AdcValue=1;		
			rec_gc_rate=(GC_8k_AdcValue*1000/GC_400_AdcValue);
//		if(sys_param_un.device_param.empty_rate<(rec_gc_rate+180)) rec_gc_rate = sys_param_un.device_param.gc_ref_rate;//差距过小，使用保存参考值
//		else sys_param_un.device_param.gc_ref_rate=rec_gc_rate;//too low ,error value 不更新
			if((rec_gc_rate+5)<sys_param_un.device_param.gc_ref_rate||rec_gc_rate>5+sys_param_un.device_param.gc_ref_rate)  sys_param_un.device_param.gc_ref_rate=rec_gc_rate;//更新参考值	
			else 
			{				
				if(sys_param_un.device_param.empty_rate<sys_param_un.device_param.gc_ref_rate+180){//使用默认值				
					sys_param_un.device_param.empty_rate=720;//1000;
					sys_param_un.device_param.gc_ref_rate=480;//680;			
				}
			}
			gc_list_init((int)sys_param_un.device_param.empty_rate,(int)(sys_param_un.device_param.gc_ref_rate-10));	//半格		
			GC_PWM_SwitchTimeManage(xTaskGetTickCount(),0);//restart	
			restart_adc_sampling();
			gc_in_or_exit(APEX_GC_EXIT);	
			#ifdef DEBUG_RTT
				SEGGER_RTT_printf(0, "ref-rate%d\r\n", sys_param_un.device_param.gc_ref_rate);	
			#endif
			apexGC_Flag=0;
		}
		else if(apexGC_Flag==2)//空载校准
		{
			gc_in_or_exit(APEX_GC_EXIT);
			vTaskDelay(3);	
			GC_Frequency=8000;
			apex_frequency_set(GC_Frequency);
			GC_PWM_SwitchTimeManage(xTaskGetTickCount(),0);//restart	
			vTaskDelay(4);
			for(i=0;i<8;i++)//i<8;i++)
			{
				vTaskDelay(MIN_GC_PWM_VALID_TIME);
				restart_adc_sampling();
				timeOut=0;						
				while(fresh_adc_value()==0)	//wait collect finished
				{
					timeOut++;
					if(timeOut%48000==0)  taskYIELD();//exit task
					if(timeOut>144000) break;//3ms	
				}
				fresh_adc_value();//get latest value				
				if(GC_Frequency==8000)
				{
					GC_8k_AdcValue=get_gc_8k_value();	
					apex_ValueBuff[i]		=	GC_8k_AdcValue;			
					GC_Frequency=400;
					apex_frequency_set(GC_Frequency);	
				}
				else if(GC_Frequency==400)
				{
					GC_400_AdcValue=get_gc_400_value();								
					apex_ValueBuff[i]		=	GC_400_AdcValue;							
					GC_Frequency=8000;
					apex_frequency_set(GC_Frequency);
				}
				
				#ifdef WDT_ENABLE
				xEventGroupSetBits(WDTEventGroup,APEX_TASK_EVENT_BIT);
				#endif				
			}
			GC_8k_AdcValue=0;
			GC_400_AdcValue=0;			
			for(i=0;i<4;i++)	
			{
				GC_8k_AdcValue+=apex_ValueBuff[2*i]	;
				GC_400_AdcValue+=apex_ValueBuff[2*i+1]	;
			}		
			GC_400_AdcValue=	GC_400_AdcValue>>2;
			GC_8k_AdcValue=(GC_8k_AdcValue>>2);	
      if(GC_400_AdcValue==0)	 GC_400_AdcValue=1;	
			rec_gc_rate=(GC_8k_AdcValue*1000/GC_400_AdcValue);			
		  if(sys_param_un.device_param.apex_tine_400Value+5<GC_400_AdcValue||sys_param_un.device_param.apex_tine_400Value>5+GC_400_AdcValue)
			{				
				sys_param_un.device_param.apex_tine_400Value = GC_400_AdcValue;
				sys_param_un.device_param.empty_rate=rec_gc_rate;//保存参考值
				if(sys_param_un.device_param.empty_rate<sys_param_un.device_param.gc_ref_rate+150||sys_param_un.device_param.empty_rate>1000) sys_param_un.device_param.empty_rate=720 ;//1000;//使用默认值
			
				gc_list_init((int)sys_param_un.device_param.empty_rate,(int)(sys_param_un.device_param.gc_ref_rate-10));
			}				
		//too low ,error value 不更新
			GC_PWM_SwitchTimeManage(xTaskGetTickCount(),0);//restart	
			restart_adc_sampling();	
			#ifdef DEBUG_RTT
					SEGGER_RTT_printf(0, "400Va=%d\r\n", sys_param_un.device_param.apex_tine_400Value);	
			#endif
			apexGC_Flag=0;
		}
		else 
		{	
        if(get_gc_state()	==RESET)
				{
					if(sys_param_un.device_param.apexFunctionLoad==0)
					{//beep	            						
						sendBuff=BUZZER_MODE_BEEP;
						xQueueSend(xQueueBeepMode, &sendBuff, 0);	
						GC_Frequency=8000;	
						apex_frequency_set(GC_Frequency);
						GC_PWM_SwitchTimeManage(xTaskGetTickCount(),0);//restart
						restart_adc_sampling();
					}
					sys_param_un.device_param.apexFunctionLoad=1;	
					//*****************sampling manage****************************
					if(GC_PWM_SwitchTimeManage(xTaskGetTickCount(),MIN_GC_PWM_VALID_TIME))			
					{							
						timeOut=0;				
						while(fresh_adc_value()==0)	//wait collect finished
						{
							timeOut++;
							if(timeOut%48000)  taskYIELD();//exit task
							if(timeOut>144000) break;//3ms	
						}						
						if(GC_Frequency==8000)
						{
							GC_8k_AdcValue=get_gc_8k_value();						
							GC_Frequency=400;
							apex_frequency_set(GC_Frequency);	
						}
						else if(GC_Frequency==400)
						{
							GC_400_AdcValue=get_gc_400_value();					
							GC_Frequency=8000;
							apex_frequency_set(GC_Frequency);	
							//handle
							if(GC_400_AdcValue==0) GC_400_AdcValue=1;// 6k阻抗 GC_400_AdcValue=1100;
							GC_depth=apex_depth_calculate(GC_8k_AdcValue,GC_400_AdcValue);	
//						OLED_ShowNum(100,32,GC_8k_AdcValue,4,16,1);	
//						OLED_ShowNum(100,48,GC_depth,4,16,1);							
						}										
						GC_PWM_SwitchTimeManage(xTaskGetTickCount(),0);//restart									
					}
					else
					{										
//					fresh_adc_value();	
						timeOut=0;						
						while(fresh_adc_value()==0)	//wait collect finished
						{
							timeOut++;
							if(timeOut%48000)  taskYIELD();//exit task
							if(timeOut>144000) 
							{
								timeOut=0;
								break;//3ms	
							}
						}				
					}					 
				}	
				else
				{
					if(GC_Frequency!=0)
					{
						GC_Frequency=0;
						apex_frequency_set(0);//关闭
					}	
					if(sys_param_un.device_param.apexFunctionLoad==1)
					{//beep						
						sendBuff=BUZZER_MODE_BEEP;
						xQueueSend(xQueueBeepMode, &sendBuff, 0);
					}					
					sys_param_un.device_param.apexFunctionLoad=0;					
					GC_depth=30;				
//				fresh_adc_value();
					timeOut=0;						
					while(fresh_adc_value()==0)	//wait collect finished
					{
						timeOut++;
						if(timeOut%48000)  taskYIELD();//exit task
						if(timeOut>144000) 
						{
							timeOut=0;
							break;//3ms	
						}
					}				
				}
        //send ramp
		}	    		
		#else
		//usart   only motor
		fresh_adc_value();
		//AppGetUsartData(&usartCmd);
		if(usartCmd>=USART_RX_CMD)
		{	
			if(sys_param_un.device_param.apexFunctionLoad==0)
			{//beep						
				sendBuff=BUZZER_MODE_BEEP;
				xQueueSend(xQueueBeepMode, &sendBuff, 0);
			}
			sys_param_un.device_param.apexFunctionLoad=1;
		}	
		else 
		{
			if(sys_param_un.device_param.apexFunctionLoad==1)
				{//beep						
					sendBuff=BUZZER_MODE_BEEP;
					xQueueSend(xQueueBeepMode, &sendBuff, 0);
				}
				sys_param_un.device_param.apexFunctionLoad=0;
		}
		if(countTimeMs>15)
		{
			countTimeMs=0;
			usartSendBuff[0]=0xa2;
			usartSendBuff[1]=0x92;
			usartSendBuff[2]=((abs(speed_list[motor_param_un.system_motor_pattern[sys_param_un.device_param.use_num].motorSpeedNum]*0.1)>>8)&0xFF)+5;
			usartSendBuff[3]=(abs(speed_list[motor_param_un.system_motor_pattern[sys_param_un.device_param.use_num].motorSpeedNum]*0.1)&0xFF)+5; 
			usartSendBuff[4]=0x93;
			usartSendBuff[5]=(torque_list[motor_param_un.system_motor_pattern[sys_param_un.device_param.use_num].torqueThresholdNum]>>8) +5;
			usartSendBuff[6]=(torque_list[motor_param_un.system_motor_pattern[sys_param_un.device_param.use_num].torqueThresholdNum]&0xFF) +5;
			AppUsartTransmit(usartSendBuff,7);	
		}	
		GC_depth=30;
		#endif
		GC_depth_vlaue(GC_depth, 0);//wrtite
		
		#ifdef WDT_ENABLE
		xEventGroupSetBits(WDTEventGroup,APEX_TASK_EVENT_BIT);
		#endif
		if(countTimeMs>5000) countTimeMs=1;
		vTaskDelay(5);  //10ms
	}
}

/*==================================key task===================================
			* @brief:  key task.
			* @param:  None
			* @retval: None
==============================================================================*/
void vAppKeyTask( void * pvParameters )
{ 
	static uint8_t sendKeyMessage=null_signal,buttonLock=1;	
	uint8_t countTimeMs=10;//scan	
	for(;;)
	{		
		sendKeyMessage= ButtonScan(countTimeMs);//10ms,//get key value			
		if(buttonLock!=0)	//button lock	
		{	
			if(sendKeyMessage!=null_signal||get_insert_state()==SET||crm_flag_get(CRM_WDT_RESET_FLAG) != RESET)	
			{
				buttonLock=0;//unlock				
			}
			sendKeyMessage=null_signal;			
		}
		//handle  or send to key message queue		
		if(sendKeyMessage!=null_signal)
		{		
			xQueueSend(xQueueKeyMessage, &sendKeyMessage, 0);			
			if(sendKeyMessage!=power_off_signal)	
			{
				sendKeyMessage= BUZZER_MODE_BEEP;		
				xQueueSend(xQueueBeepMode, &sendKeyMessage, 0);
			}		
			sendKeyMessage=null_signal;
		}
		#ifdef WDT_ENABLE
			xEventGroupSetBits(WDTEventGroup,KEY_TASK_EVENT_BIT);
		#endif
		//handle  or send to key message queue
		vTaskDelay(countTimeMs);  //10ms
	}
}
/*==================================vAppBeepTemporaryTask  task===================================
			* @brief:  vAppBeepTemporaryTask task.
			* @param:  None
			* @retval: None
==============================================================================*/
void vAppBeepTemporaryTask( void * pvParameters )
{ 
	static unsigned char  buzzerOp=BUZZER_MODE_POWER_ON_HINT_VOL;//开机提示音
	unsigned short int freq=4000;//		
	for(;;)
	{		
		if(buzzerOp==BUZZER_MODE_POWER_ON_HINT_VOL)
		{
			buzzerOp=BUZZER_MODE_MUTE;	
		}
		else if(buzzerOp==BUZZER_MODE_MOTOR_REVERSE)
		{
			if(	freq!=4000)
			{	
				freq=4000;
				buzz_frequency_set(freq,0);                                              
			}
			BuzzrSimpleSwitch(1);//on
			vTaskDelay(50);//	80ms	
			#ifdef WDT_ENABLE
			xEventGroupSetBits(WDTEventGroup,BEEP_TASK_EVENT_BIT);
			#endif				
			BuzzrSimpleSwitch(0);//off							
			buzzerOp=BUZZER_MODE_MUTE;			
		}
		else if(buzzerOp==BUZZER_MODE_MOTOR_REVERSE_LONG)
		{
			if(	freq!=1800)
			{	
				freq=1800;
				buzz_frequency_set(freq,0);                                              
			}
			BuzzrSimpleSwitch(1);//on
			vTaskDelay(100);//	80ms							
			BuzzrSimpleSwitch(0);//off							
			buzzerOp=BUZZER_MODE_MUTE;			
		}
		else if(buzzerOp==BUZZER_MODE_GC_APEX)
		{
			if(	freq!=1800)
			{	
				freq=1800;
				buzz_frequency_set(freq,0);
			}
			BuzzrSimpleSwitch(1);//on
			vTaskDelay(150);//	80ms	
			#ifdef WDT_ENABLE
			xEventGroupSetBits(WDTEventGroup,BEEP_TASK_EVENT_BIT);
			#endif				
			BuzzrSimpleSwitch(0);//off
			vTaskDelay(50);		
			#ifdef WDT_ENABLE
			xEventGroupSetBits(WDTEventGroup,BEEP_TASK_EVENT_BIT);						
			vTaskDelay(100);					
			xEventGroupSetBits(WDTEventGroup,BEEP_TASK_EVENT_BIT);
			vTaskDelay(100);				
			xEventGroupSetBits(WDTEventGroup,BEEP_TASK_EVENT_BIT);
			#else
			vTaskDelay(200);			
			#endif	
			if(	freq!=4000)
			{	
				freq=4000;
				buzz_frequency_set(freq,0);
			}
			BuzzrSimpleSwitch(1);//on
			vTaskDelay(100);//	80ms	
			#ifdef WDT_ENABLE
			xEventGroupSetBits(WDTEventGroup,BEEP_TASK_EVENT_BIT);
			#endif				
			BuzzrSimpleSwitch(0);//off
			vTaskDelay(100);
			#ifdef WDT_ENABLE
			xEventGroupSetBits(WDTEventGroup,BEEP_TASK_EVENT_BIT);						
			vTaskDelay(100);					
			xEventGroupSetBits(WDTEventGroup,BEEP_TASK_EVENT_BIT);
			vTaskDelay(100);				
			xEventGroupSetBits(WDTEventGroup,BEEP_TASK_EVENT_BIT);
			#else
			vTaskDelay(200);			
			#endif	
			buzzerOp=BUZZER_MODE_MUTE;						
		}	
		else if(buzzerOp==BUZZER_MODE_GC_ZREO_APEX)
		{			
			if(	freq!=1800)
			{	
				freq=1800;
				buzz_frequency_set(freq,0);
			}
			BuzzrSimpleSwitch(1);//on
			vTaskDelay(75);//	80ms	
			#ifdef WDT_ENABLE
			xEventGroupSetBits(WDTEventGroup,BEEP_TASK_EVENT_BIT);
			#endif				
			BuzzrSimpleSwitch(0);//off
			vTaskDelay(25);		
			#ifdef WDT_ENABLE
			xEventGroupSetBits(WDTEventGroup,BEEP_TASK_EVENT_BIT);						
			vTaskDelay(50);					
			xEventGroupSetBits(WDTEventGroup,BEEP_TASK_EVENT_BIT);
			vTaskDelay(50);				
			xEventGroupSetBits(WDTEventGroup,BEEP_TASK_EVENT_BIT);
			#else
			vTaskDelay(200);			
			#endif	
			if(	freq!=4000)
			{	
				freq=4000;
				buzz_frequency_set(freq,0);
			}
			BuzzrSimpleSwitch(1);//on
			vTaskDelay(50);//	80ms	
			#ifdef WDT_ENABLE
			xEventGroupSetBits(WDTEventGroup,BEEP_TASK_EVENT_BIT);
			#endif				
			BuzzrSimpleSwitch(0);//off
			vTaskDelay(50);
			#ifdef WDT_ENABLE
			xEventGroupSetBits(WDTEventGroup,BEEP_TASK_EVENT_BIT);						
			vTaskDelay(50);					
			xEventGroupSetBits(WDTEventGroup,BEEP_TASK_EVENT_BIT);
			vTaskDelay(50);				
			xEventGroupSetBits(WDTEventGroup,BEEP_TASK_EVENT_BIT);
			#else
			vTaskDelay(200);			
			#endif	
			buzzerOp=BUZZER_MODE_MUTE;				
		}	
		else if(buzzerOp==BUZZER_MODE_GC_OVER)
		{			
			if(	freq!=4000)
			{	
				freq=4000;
				buzz_frequency_set(freq,0);
			}
			BuzzrSimpleSwitch(1);//on
			vTaskDelay(80);//	80ms        				
			BuzzrSimpleSwitch(0);//off				
			#ifdef WDT_ENABLE		
			xEventGroupSetBits(WDTEventGroup,BEEP_TASK_EVENT_BIT);
			vTaskDelay(40);				
			xEventGroupSetBits(WDTEventGroup,BEEP_TASK_EVENT_BIT);
			#else
			vTaskDelay(100);			
			#endif	
			buzzerOp=BUZZER_MODE_MUTE;				
		}	
		else if(buzzerOp==BUZZER_MODE_GC_CONNECT_TEST)
		{
			if(	freq!=4000)
			{	
				freq=4000;
				buzz_frequency_set(freq,0);
			}
			BuzzrSimpleSwitch(1);//on
			vTaskDelay(50);//	80ms	
			#ifdef WDT_ENABLE
			xEventGroupSetBits(WDTEventGroup,BEEP_TASK_EVENT_BIT);
			#endif				
			BuzzrSimpleSwitch(0);//off
			vTaskDelay(50);
			buzzerOp=BUZZER_MODE_MUTE;				
		}
		buzzerOp=BUZZER_MODE_MUTE;	//清除旧值			
		xQueueReceive(xQueueBeepMode, &buzzerOp, 0);//获取新值				
		if(buzzerOp==BUZZER_MODE_BEEP)
		{
			if(freq!=4000)
			{
				freq=4000;				
				buzz_frequency_set(freq,0);
			}	
			BuzzrSimpleSwitch(1);//on
			vTaskDelay(60);//	80ms		
			BuzzrSimpleSwitch(0);//off	
			buzzerOp=BUZZER_MODE_MUTE;
		}		
		#ifdef WDT_ENABLE
			xEventGroupSetBits(WDTEventGroup,BEEP_TASK_EVENT_BIT);
		#endif		
		vTaskDelay(5);
	}	
}
/*==================================vSysWDT_ManageTask  ===================================
			* @brief:  vSysWDT_ManageTask.
			* @param:  None
			* @retval: None
==============================================================================*/
#ifdef WDT_ENABLE
void vSysWDT_ManageTask( void * pvParameters )
{ 	
	EventBits_t uxBits;
	BaseType_t retFlag;	
	for(;;)
	{
		vTaskDelay(MAX_WDT_FEED_TIME_MS);		
		uxBits =xEventGroupWaitBits(WDTEventGroup,EVENT_BITS_ALL,pdFALSE,pdTRUE,50);
		if((uxBits&EVENT_BITS_ALL)==EVENT_BITS_ALL)
		{			
			wdt_counter_reload();//feed dog
			xEventGroupClearBits(WDTEventGroup,EVENT_BITS_ALL);
			uxBits=0;
		}
		else
		{							
			if((uxBits&KEY_TASK_EVENT_BIT)!=KEY_TASK_EVENT_BIT)//key task err
			{
				vTaskDelete( keyScanTask_Handle);	
				retFlag=xTaskCreate( vAppKeyTask, "key scan", KEY_SCAN_STACK_DEPTH, NULL, KEY_SCAN_TASK_PRIORITY, &keyScanTask_Handle );	
				if(retFlag!=pdPASS)
				{				
					MenuDevicePowerOff(0);	
				}					
			}
			else if((uxBits&MENU_TASK_EVENT_BIT)!=MENU_TASK_EVENT_BIT)
			{	
				vTaskDelete( menuManageTask_Handle);	
				retFlag=xTaskCreate( vAppMenuManageTask, "menu manage", MENU_MANAGE_STACK_DEPTH, NULL, MENU_MANAGE_TASK_PRIORITY, &menuManageTask_Handle );
				if(retFlag!=pdPASS)
				{
					App_MotorControl(MOTOR_MODE_STOP);	
				}	
			}
			else if((uxBits&MOTOR_CONTROL_TASK_EVENT_BIT)!=MOTOR_CONTROL_TASK_EVENT_BIT)
			{									
				vTaskDelete( motorControlTask_Handle);	
				retFlag=xTaskCreate( vAppMotorControlTask, "motor control", MOTOR_CONTROL_STACK_DEPTH, NULL, MOTOR_CONTROL_TASK_PRIORITY, &motorControlTask_Handle );
				if(retFlag!=pdPASS)
				{
					App_MotorControl(MOTOR_MODE_STOP);	
				}					
			}
			else if((uxBits&PERIODIC_TASK_EVENT_BIT)!=PERIODIC_TASK_EVENT_BIT)
			{								
				vTaskDelete( periodicTask_Handle);	
				retFlag=xTaskCreate( vAppPeriodicTask, "periodic", PERIODIC_STACK_DEPTH, NULL, PERIODIC_TASK_PRIORITY, &periodicTask_Handle );	
				if(retFlag!=pdPASS)
				{
					App_MotorControl(MOTOR_MODE_STOP);	
				}						
			}	
			else if((uxBits&BEEP_TASK_EVENT_BIT)!=BEEP_TASK_EVENT_BIT){					
				vTaskDelete( beepTemporaryTask_Handle);
				xTaskCreate( vAppBeepTemporaryTask, "beep", SECONDARY_BEEP_STACK_DEPTH, NULL, SECONDARY_BEEP_TASK_PRIORITY, &beepTemporaryTask_Handle );			
			}
			else if((uxBits&APEX_TASK_EVENT_BIT)!=APEX_TASK_EVENT_BIT){					
				vTaskDelete( ApexGC_Task_Handle);
				xTaskCreate( vApexGC_Task, "apex", APEX_GC_STACK_DEPTH, NULL, APEX_GC_TASK_PRIORITY, &ApexGC_Task_Handle );			
			}				
		}
	
	}		
}
#endif

/*==================================start task===================================
			* @brief:  start task.
			* @param:  None
			* @retval: None
====================================================================================*/
void vTaskStart( void * pvParameters )
{		
		taskENTER_CRITICAL();
		Resume_RTOS_stick();//start
	 //queue
		xQueueKeyMessage =xQueueCreate(3,sizeof(uint16_t));	
		xQueueBeepMode =	xQueueCreate(1,sizeof(uint8_t));
		xQueueBatValue =	xQueueCreate(1,sizeof(unsigned short int));	
		xQueueMenuValue = xQueueCreate(1,sizeof(uint8_t));
	//Semaphore		
		xSemaphorePowerOff=xSemaphoreCreateBinary() ; //power off		
		xSemaphoreDispRfresh=xSemaphoreCreateBinary();
	#ifdef  APEX_FUNCTION_EBABLE
		xSemaphoreApexAutoCalibration=xSemaphoreCreateBinary();
	#endif
	
	#ifdef WDT_ENABLE
		WDTEventGroup=xEventGroupCreate();
	#endif		
		
		xTaskCreate( vAppPeriodicTask, "periodic", PERIODIC_STACK_DEPTH, NULL, PERIODIC_TASK_PRIORITY, &periodicTask_Handle );						
		xTaskCreate( vAppMotorControlTask, "motor control", MOTOR_CONTROL_STACK_DEPTH, NULL, MOTOR_CONTROL_TASK_PRIORITY, &motorControlTask_Handle );
		xTaskCreate( vAppKeyTask, "key scan", KEY_SCAN_STACK_DEPTH, NULL, KEY_SCAN_TASK_PRIORITY, &keyScanTask_Handle );
		xTaskCreate( vAppBeepTemporaryTask, "beep", SECONDARY_BEEP_STACK_DEPTH, NULL, SECONDARY_BEEP_TASK_PRIORITY, &beepTemporaryTask_Handle );
		#if configUSE_TIMERS == 1
//	xTimer01=xTimerCreate("xTimer01",100,pdTRUE,&1,NULL);		
		#endif		
		xTaskCreate( vApexGC_Task, "APEX", APEX_GC_STACK_DEPTH, NULL, APEX_GC_TASK_PRIORITY, &ApexGC_Task_Handle );
		
		xTaskCreate( vAppMenuManageTask, "menu manage", MENU_MANAGE_STACK_DEPTH, NULL, MENU_MANAGE_TASK_PRIORITY, &menuManageTask_Handle );	
	#ifdef WDT_ENABLE	
		if(WDTEventGroup!=NULL)
		{
			xTaskCreate( vSysWDT_ManageTask, "WDT", WDT_MANAGE_STACK_DEPTH, NULL, WDT_MANAGE_TASK_PRIORITY, &WDT_ManageTask_Handle );
		}
	#endif
		vTaskDelete( NULL);
		taskEXIT_CRITICAL();  			
}







