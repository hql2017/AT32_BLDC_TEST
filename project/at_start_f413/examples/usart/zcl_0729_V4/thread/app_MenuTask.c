#include "string.h"
#include "stdlib.h"
#include "math.h"
#include "gpio_port.h"
#include "oled_port.h"
#include "usart_port.h"
#include "adc_port.h"
#include "at32f413_board.h"
#include "app_MenuTask.h"
#include "app_MotorControlTask.h"

#include "customer_control.h"

#include "para_list.h"
#include "app_user_storage.h"
#include "apex_gc_port.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"	
#ifdef WDT_ENABLE
	#include "wdt.h"
	#include "event_groups.h"	
	extern EventGroupHandle_t  WDTEventGroup;
	extern TaskHandle_t  WDT_ManageTask_Handle;
#endif
	
extern TaskHandle_t  beepTemporaryTask_Handle;
extern 	QueueHandle_t  xQueueMotorControl;
extern QueueHandle_t   xQueueKeyMessage;//key	
extern QueueHandle_t  xQueueBeepMode;//beep
extern QueueHandle_t   xQueueMenuValue;
extern QueueHandle_t  xQueueBatValue;//batValue
extern QueueHandle_t xQueueApexValue;
extern SemaphoreHandle_t xSemaphoreDispRfresh;

static void Disp_ApexPreForSet(u8 value);
static void ApexBarFlashForSet(unsigned int systemTimeMs,unsigned char startFlag, short int apexValue);
#ifdef  APEX_FUNCTION_EBABLE
extern SemaphoreHandle_t xSemaphoreApexAutoCalibration;
#endif
	
#define MINIMUM_CW_CCW_ANGLE_GAP          80//minimum routor angle in  " motor_settings.mode=EndoModePositionToggle "	
	
#ifdef ZHX
#define MAX_MOTOR_SPEED   2200//2000//闂傚倷娴囧▔鏇㈠窗鎼淬們浜归柕濞炬櫆閸嬫牠鎮楀☉娅亪顢旈崼鏇熲拺閻犳亽鍔岄弸鎴︽煛鐎ｎ亶鐓兼鐐茬箻閺佹捇鏁撻敓锟�(max 2450*6=14700)
#define MAX_TORQUE_UUPER_THRESHOLD   50//42//闂傚倷娴囧▔鏇㈠窗鎼淬們浜归柕濞炬櫆閸嬫牠鎮楀☉娅亪顢旈崼鏇熲拺閻犳亽鍔岄弸鎴︽煛鐎ｎ亶鐓兼鐐茬箻閺佹捇鏁撻敓锟�(46.281g.cm/A~~0.454N.cm/A)(18.41mN.m or 1.841N.cm  1g.cm=0.00981N.cm=0.0981mN.m=0.0000981N.m)(闂傚倷娴囧▔鏇㈠窗閹版澘鍑犲┑鐘宠壘缁狀垶鏌ｉ妶搴＄仭缂佷緡鍣ｅ铏规崉閵娿儲鐎鹃梺鍝勵儏椤兘鐛箛娑欏€婚柤鎭掑劜濞呫垽姊洪崫鍕偓鍫曞磹閺嶎偀鍋撳鐐4.0A闂傚倷娴囧▔鏇㈠窗閹版澘鍑犲┑鐘宠壘缁狀垶姊洪崹顕呭剰妞ゃ儱顦靛铏规崉閵娿儲鐎鹃梺鍝勵儏椤兘鐛箛娑欐櫢闁跨噦鎷�187.71 g.cm=1.841N.cm=18.41mN.m=0.1841N.m)
#define HALF_MAX_TORQUE_UUPER_THRESHOLD   MAX_TORQUE_UUPER_THRESHOLD/2
#define MINIMUM_ROTOR_ANGLE          10//minimum routor angle in  " motor_settings.mode=EndoModePositionToggle "

//unsigned short int torque_list[16]={5,8,10,12,15,18, 20,22,25,30,35,40,42,50,55,60};//unit mN.m
  unsigned short int torque_list[21]={6,8,10,12,14,16,18, 20,22,24,26,28,30,32,35,40,42,45,50,55,60};//unit mN.m
short int speed_list[20]={100,150,200,250,300,350,400,450,500,550,600,800,1000,1200,1500,1800,2000,2200,2500};	
	
//	short int speed_list[20]={100,150,200,250,300,350,400,450,500,550,600,800,1000,1200,1500,1800,MAX_MOTOR_SPEED,2200,2500};	
	//7V
//unsigned short int torque_limit[22]={torque70_Ncm ,torque70_Ncm ,torque70_Ncm ,torque70_Ncm ,torque70_Ncm ,torque70_Ncm ,torque70_Ncm ,torque65_Ncm ,\
//	torque65_Ncm ,torque65_Ncm ,torque60_Ncm ,torque50_Ncm ,torque45_Ncm  ,torque35_Ncm  ,torque20_Ncm  ,torque05_Ncm  ,torque05_Ncm  ,torque05_Ncm,torque05_Ncm,torque05_Ncm};//unit mN.m闂傚倷娴囧▔鏇㈠窗閹版澘鍑犲┑鐘宠壘缁狀垶鏌ｉ幋锝呅撻柡鍛倐閺岋繝宕掑Ο琛″亾閺嶎偀鍋撳顒傜Ш鐎规洘绮岄濂稿醇閻旀亽鍋栭梻鍌欐祰濞夋洟宕伴幇鏉垮嚑濠电姵鑹剧粻顖炴煟閹达絽袚闁哄懏鎮傞弻锟犲磼濡　鍋撻弽顐熷亾濮橆剛绉洪柡灞诲姂閹垽宕ㄦ繝鍕磿闂備礁缍婇ˉ鎾诲礂濮椻偓瀵偊骞樼紒妯烘畻濠殿喗菧閸庨亶鍩€椤戭剙娲ょ粻顖炴煟閹达絽袚闁哄懏鎮傞弻銊╂偆閸屾稑顏�
//8V
unsigned short int torque_limit[22]={torque70_Ncm ,torque70_Ncm ,torque70_Ncm ,torque70_Ncm ,torque70_Ncm ,torque70_Ncm ,torque70_Ncm ,torque65_Ncm ,\
	torque65_Ncm ,torque65_Ncm ,torque60_Ncm ,torque60_Ncm ,torque60_Ncm  ,torque50_Ncm  ,torque35_Ncm  ,torque20_Ncm  ,torque08_Ncm  ,torque06_Ncm,torque06_Ncm,torque06_Ncm};//unit mN.m闂傚倷娴囧▔鏇㈠窗閹版澘鍑犲┑鐘宠壘缁狀垶鏌ｉ幋锝呅撻柡鍛倐閺岋繝宕掑Ο琛″亾閺嶎偀鍋撳顒傜Ш鐎规洘绮岄濂稿醇閻旀亽鍋栭梻鍌欐祰濞夋洟宕伴幇鏉垮嚑濠电姵鑹剧粻顖炴煟閹达絽袚闁哄懏鎮傞弻锟犲磼濡　鍋撻弽顐熷亾濮橆剛绉洪柡灞诲姂閹垽宕ㄦ繝鍕磿闂備礁缍婇ˉ鎾诲礂濮椻偓瀵偊骞樼紒妯烘畻濠殿喗菧閸庨亶鍩€椤戭剙娲ょ粻顖炴煟閹达絽袚闁哄懏鎮傞弻銊╂偆閸屾稑顏�
	
	#else 
#define MAX_MOTOR_SPEED    2450//moons =3000
#define MAX_TORQUE_UUPER_THRESHOLD  42// 50//moons(max 5.0N.cm)
#define HALF_MAX_TORQUE_UUPER_THRESHOLD   MAX_TORQUE_UUPER_THRESHOLD/2
#define MINIMUM_ROTOR_ANGLE          10//minimum routor angle in  " motor_settings.mode=EndoModePositionToggle "

short int speed_list[20]={100,150,200,250,300,350,400,450,500,550,600,800,1000,1200,1500,1800,2000,2200,MAX_MOTOR_SPEED};
//float torgue_list[13]={0.5,0.8,1.0,1.2,1.5,1.8,2.0,2.2,2.5,3.0,3.5,4.0,MAX_TORQUE_UUPER_THRESHOLD};//unit N.cm
unsigned short int torque_list[13]={5,8,10,12,15,18,20,22,25,30,35,40,MAX_TORQUE_UUPER_THRESHOLD};//unit mN.m
#endif

enum {
		ADJUST_MOTOR_PARAM_PROGRAM_NUM = 0,
		ADJUST_MOTOR_PARAM_SPEED,          
		ADJUST_MOTOR_PARAM_TORQUE, 
		ADJUST_MOTOR_PARAM_DIRECTION,	
		ADJUST_MOTOR_PARAM_ANGLE_FORWARD , 
		ADJUST_MOTOR_PARAM_ANGLE_RESERVE , 
};

#define MENU_PARAM_ADD           0
#define MENU_PARAM_SUB          1

#define MINUTE_IDLE_TICKS           60000//1minute
#define MENU_IDLE_TICKS           MINUTE_IDLE_TICKS*3//5000//5s
#define HOME_PARAM_IDLE_TICKS            5000//5s
#define DEVICE_POWER_OFF_TICKS          MINUTE_IDLE_TICKS*3//3 minute

extern MotorStatus_TypeDef motor_status;
extern MotorSettings_TypeDef motor_settings;

//SYSTEM_MOTOR_PARAM sys_MotorParam[10]={0};
extern union Param_Union sys_param_un; 
extern union Motor_Para_Union motor_param_un; 

static void MenuPageTurns(unsigned char pageID);
/**
  * @brief 	TorqueRangeCheck
  * @param  value
  * @retval none
  */
static unsigned short TorqueRangeCheck (unsigned short value)
{
	unsigned short temp;
	temp=value;
	if(temp>MAX_TORQUE_UUPER_THRESHOLD) temp=MAX_TORQUE_UUPER_THRESHOLD;
	return temp;
}
/**
  * @brief 	APEXIdleTimeManage
  * @param  systemTime
  * @retval none
  */
static error_status  APEXIdleTimeManage(unsigned int systemTime,unsigned int targetTime)
{	
	error_status err;	
	static  unsigned int recTicks;
	err=ERROR;
	if(targetTime==0||recTicks>systemTime) 
	{
		recTicks=systemTime;		
	}	
	else
	{		
		if(systemTime>targetTime+recTicks)
		{
			//recTicks=systemTime;
			err=SUCCESS;
		}		
	}	
	return err;
}
/**
  * @brief 	MenuIdleTimeManage
  * @param  systemTime
  * @retval none
  */
static error_status  MenuIdleTimeManage(unsigned int systemTime,unsigned int targetTime)
{	
	error_status err;	
	static  unsigned int recTicks;
	err=ERROR;
	if(targetTime==0||recTicks>systemTime) 
	{
		recTicks=systemTime;		
	}
	else
	{		
		if(systemTime-recTicks>targetTime)
		{
			recTicks=systemTime;
			err=SUCCESS;
		}
	
	}	
	return err;
}
/**
  * @brief 	DEviceOffTimeManage
  * @param  systemTime
  * @retval none
  */
static error_status  DeviceOffTimeManage(unsigned int systemTime,unsigned int targetTime)
{	
	error_status err;	
	static  unsigned int recTicks;
	err=ERROR;
	if(targetTime==0||recTicks>systemTime) 
	{
		recTicks=systemTime;		
	}
	else
	{		
		if(systemTime-recTicks>targetTime)
		{
			recTicks=systemTime;
			err=SUCCESS;
		}		
	}	
	return err;
}

/**
  * @brief 	timerCountDown
  * @param  
  * @retval none
  */
static error_status timerCountDown(unsigned int systemTimeS,unsigned short int targetTimeS,unsigned char startFlag)
{
	error_status err;	
	static unsigned int recTimeS;
	static unsigned int realTimeS;
	err=ERROR;	
	if(startFlag==0)
	{
		recTimeS=systemTimeS;
		realTimeS=0;

		if(targetTimeS==0)
		{
			OLED_ShowString(114,40,"   ",16,1);		 
		}
		else
		{
			OLED_ShowNum(114,40,targetTimeS,2,16,1); 
			OLED_ShowChar(130,40,'S',16,1); 
		}
	}
	else 
	{		
		if((systemTimeS-recTimeS)!=realTimeS)
		{
			MenuIdleTimeManage(xTaskGetTickCount(),0);
			realTimeS=systemTimeS-recTimeS;
			if(realTimeS>targetTimeS)
			{					
				err=SUCCESS;
			}
			else 
			{	
				if(targetTimeS>10)
				{
					OLED_ShowNum(114,40,targetTimeS,2,16,1);
					OLED_ShowChar(130,40,'S',16,1); 		
				}
				else
				{
					if(targetTimeS==0)
					{						
						OLED_ShowString(114,40,"   ",16,1);						
					}
					else
					{
						OLED_ShowChar(114,40,' ',16,1); 						
						OLED_ShowNum(122,40,targetTimeS-realTimeS,1,16,1);	
						OLED_ShowChar(130,40,'S',16,1); 	
					}						
				}			
			}	
			xSemaphoreGive(xSemaphoreDispRfresh);				
		}			
	}
	return err;
}
/**
  * @brief 	MenuDevicePowerOff
  * @param  unsigned char feedDogFlag
  * @retval none
  */
void MenuDevicePowerOff(unsigned char feedDogFlag)
{
		if(feedDogFlag==2)//restore
		{
			default_para_write_buff();
		}
		App_MotorControl(MOTOR_MODE_STOP);
		#ifndef RTOS_UNUSED
//		taskENTER_CRITICAL();	
		__disable_irq();	
		write_para_judge();//save  date			
//		taskEXIT_CRITICAL();
			__enable_irq();
		vTaskDelay(2);//	1ms
		#else
		__disable_irq();	
		write_para_judge();//save  date			
			__enable_irq();
		delay_1ms(10);		
		#endif
		OLED_DisPlay_Off();
		device_power_off();//power off	
	#ifdef DEBUG_RTT
		SEGGER_RTT_WriteString(0,"power off\r\n");
	#endif	
		while(1) 
		{			
//			if(feedDogFlag!=0)
//			{//闂傚倷娴囧▔鏇㈠窗閺嶎厼围闁绘垶蓱缂嶅洭鏌ｉ幇闈涘妞ゅ繐鐖煎铏规崉閵娿儲鐎鹃梺绯曟櫅閹冲繒鈧數鍘ч埢搴ㄥ箛椤掑绱﹂梻鍌欐祰濞夋洟宕伴幇鏉垮嚑濠电姵鑹剧粻顖炴煟閹达絽袚闁哄懏鎮傞弻銊╂偆閸屾稑顏�
				#ifdef WDT_ENABLE
					xEventGroupSetBits(WDTEventGroup,MENU_TASK_EVENT_BIT);
				#endif
//			wdt_counter_reload();//wait  run_button_release_signal 
//			}
		}				 
}
/**
  * @brief 	MenuConfigMotorParam
  * @param  paramType
  * @retval none
  */

static error_status  MenuConfigMotorParam(unsigned char programNum,unsigned char selectNum,unsigned char addOrSub)
{		
	error_status err;
	err=ERROR;
	//if(programNum<10) //==10onlay apex mode
	if((programNum>5)&&programNum<10) //==10onlay apex mode//p0~p5,for bit 
	{		
		if(selectNum==ADJUST_MOTOR_PARAM_PROGRAM_NUM)
		{	//p0~p9
		}		
		else if(selectNum==ADJUST_MOTOR_PARAM_SPEED)//speed
		{	
			if(addOrSub==MENU_PARAM_ADD)
			{	
				if(motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].dir==EndoModePositionToggle)
				{
					motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].toggleSpeedNum++;						
					motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].toggleSpeedNum%=(spd600_Rpm_num+1);	
				}
				else 
				{
					motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].motorSpeedNum++;						
					motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].motorSpeedNum%=(MAX_spd_Rpm_num+1);						
				}
			}
			else if(addOrSub==MENU_PARAM_SUB) 
			{//mini=150rpm
				if(motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].dir==EndoModePositionToggle)
				{
					if(motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].toggleSpeedNum==spd100_Rpm_num)
					{
						motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].toggleSpeedNum=spd600_Rpm_num;
					}
					else
					{
						motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].toggleSpeedNum--;	
						motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].toggleSpeedNum%=(spd600_Rpm_num+1)	;	
					}
				}
				else
				{
					if(motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].motorSpeedNum==spd100_Rpm_num)
					{
						motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].motorSpeedNum=MAX_spd_Rpm_num;
					}
					else
					{
						motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].motorSpeedNum--;	
						motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].motorSpeedNum%=(MAX_spd_Rpm_num+1)	;	
					}
				}				
			}		
			if(motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].torqueThresholdNum>torque_limit[motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].motorSpeedNum])
			{
			 motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].torqueThresholdNum=torque_limit[motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].motorSpeedNum];
			}	
		}
		else if(selectNum==ADJUST_MOTOR_PARAM_TORQUE)//torque
		{			
			if(addOrSub==MENU_PARAM_ADD)
			{	
				if(motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].dir==EndoModePositionToggle)
				{
					motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].recTorqueThresholdNum++;	
					motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].recTorqueThresholdNum%=(MAX_torque_42_Ncm+1);
					if(motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].recTorqueThresholdNum<torque20_Ncm) motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].recTorqueThresholdNum=torque20_Ncm;
					if(motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].recTorqueThresholdNum>torque_limit[motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].motorSpeedNum])
					 {
						 motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].recTorqueThresholdNum=torque_limit[motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].motorSpeedNum];
					 }	
				}		
				else if(motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].dir==EndoModeTorqueATC)//max 2.0
				{
					motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].atcTorqueThresholdNum++;	
					motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].atcTorqueThresholdNum%=(torque20_Ncm+1);
					if(motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].atcTorqueThresholdNum>torque_limit[motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].motorSpeedNum])
					 {
						 motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].atcTorqueThresholdNum=torque_limit[motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].motorSpeedNum];
					 }							
				}
				else
				{
					motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].torqueThresholdNum++;	
					motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].torqueThresholdNum%=(MAX_torque_42_Ncm+1);	
					if(motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].torqueThresholdNum>torque_limit[motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].motorSpeedNum])
					{
						motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].torqueThresholdNum=torque06_Ncm;
					}	
				}	
						
			}
			else  if(addOrSub==MENU_PARAM_SUB)
			{	
				if(motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].dir==EndoModePositionToggle)
				{
					if(motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].recTorqueThresholdNum<=torque20_Ncm)
					{
						motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].recTorqueThresholdNum=MAX_torque_42_Ncm;
					} 
					else 
					{
						motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].recTorqueThresholdNum--;	
						motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].recTorqueThresholdNum%=(MAX_torque_42_Ncm+1);
					}					
					if(motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].recTorqueThresholdNum>torque_limit[motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].motorSpeedNum])
					{
						motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].recTorqueThresholdNum=torque_limit[motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].motorSpeedNum];
					}	
				}																				
				else if(motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].dir==EndoModeTorqueATC)//max 2.0
				{
					if(motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].atcTorqueThresholdNum<=torque06_Ncm)
					{
						motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].atcTorqueThresholdNum=torque20_Ncm;
					}
					else
					{
						motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].atcTorqueThresholdNum--;
						motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].atcTorqueThresholdNum%=(torque20_Ncm+1);							
					}							
					if(motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].atcTorqueThresholdNum>torque_limit[motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].motorSpeedNum])
					{
						 motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].atcTorqueThresholdNum=torque_limit[motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].motorSpeedNum];
					}	
				}
				else 
				{
					if(motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].torqueThresholdNum<=torque06_Ncm)
					{
						motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].torqueThresholdNum=MAX_torque_42_Ncm;
					}
					else
					{
						motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].torqueThresholdNum--;
						motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].torqueThresholdNum%=(MAX_torque_42_Ncm+1);							
					}							
					if(motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].torqueThresholdNum>torque_limit[motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].motorSpeedNum])
					{
						 motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].torqueThresholdNum=torque_limit[motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].motorSpeedNum];
					}	
				}					
			}			
		}
		else if(selectNum==ADJUST_MOTOR_PARAM_DIRECTION)//dir,0,1,2,3
		{
			if(addOrSub==MENU_PARAM_ADD)
			{
				motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].dir++;
				motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].dir%=Max_endoMode;						
			}
			else if(addOrSub==MENU_PARAM_SUB)
			{				
				if(motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].dir>EndoModePositionToggle)	
				{
					motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].dir--;
					motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].dir%=Max_endoMode;
				}
				else 
				{
					motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].dir=EndoModeTorqueATC;
				}				
			}
		}
		else if(selectNum==ADJUST_MOTOR_PARAM_ANGLE_FORWARD)//angle   forward
		{			
			if(addOrSub==MENU_PARAM_ADD)
			{				
				motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].forwardPosition+=MINIMUM_ROTOR_ANGLE;
				if(motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].forwardPosition>360) 
				{
					motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].forwardPosition=10;	
				}	  				
			}
			else if(addOrSub==MENU_PARAM_SUB)
			{
				motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].forwardPosition-=MINIMUM_ROTOR_ANGLE;
				if(motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].forwardPosition<10) 
				{
					motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].forwardPosition=360;
				}		
			}			
		}
		else if(selectNum==ADJUST_MOTOR_PARAM_ANGLE_RESERVE)//angle   reserve
		{			
			if(addOrSub==MENU_PARAM_ADD)
			{	
				motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].reversePosition-=MINIMUM_ROTOR_ANGLE;
				if(motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].reversePosition<-360) 
				{
					motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].reversePosition=-10;
				}
			}
			else if(addOrSub==MENU_PARAM_SUB)
			{
				motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].reversePosition+=MINIMUM_ROTOR_ANGLE;
				if(motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].reversePosition>-10) 
				{
					motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].reversePosition=-360;
				}
			}				
		}		
	}
	err=SUCCESS;
	return err;
}
/**
  * @brief 	MenuMotorParamUpdate
  * @param program, selectNum
  * @retval none
  */
static  error_status  MenuMotorParamUpdate(unsigned short int programNum)
{		
	
	error_status err;
	err=ERROR;
  if(programNum<10) //==10 only apex mode
	{
		if(motor_param_un.system_motor_pattern[programNum].pNum!=programNum) //first
		{
			motor_param_un.system_motor_pattern[programNum].pNum=programNum;
			motor_param_un.system_motor_pattern[programNum].recTorqueThresholdNum=torque40_Ncm;//				
			motor_param_un.system_motor_pattern[programNum].forwardPosition = 30; 
			motor_param_un.system_motor_pattern[programNum].reversePosition = -150; 		
			motor_param_un.system_motor_pattern[programNum].torqueThresholdNum = torque40_Ncm;// (unit mN.m)(4.0 unit N.cm) 
			motor_param_un.system_motor_pattern[programNum].atcTorqueThresholdNum =	torque20_Ncm;	
			motor_param_un.system_motor_pattern[programNum].toggleSpeedNum =spd300_Rpm_num; 
		}		
		//.....//save to FLASH when power off	
		// MOTOR_SETTING_UPDATE
		motor_settings.autorev_mode=AutoReverseMode1;	
		if(motor_param_un.system_motor_pattern[programNum].dir==EndoModeTorqueATC)
		{
			motor_settings.mode=EndoModeSpeedForward;				
		}
		else
		{			
			motor_settings.mode=motor_param_un.system_motor_pattern[programNum].dir;	
		}	
		motor_settings.forward_position=motor_param_un.system_motor_pattern[programNum].forwardPosition;
		motor_settings.reverse_position=motor_param_un.system_motor_pattern[programNum].reversePosition;
		if(motor_param_un.system_motor_pattern[programNum].dir==EndoModeTorqueATC)
		{		
			if(motor_param_un.system_motor_pattern[programNum].atcTorqueThresholdNum>torque_limit[motor_param_un.system_motor_pattern[programNum].motorSpeedNum])
			{
				motor_param_un.system_motor_pattern[programNum].atcTorqueThresholdNum=torque_limit[motor_param_un.system_motor_pattern[programNum].motorSpeedNum];
			}				
			motor_settings.upper_threshold=torque_list[torque40_Ncm]*0.10;//MAX_TORQUE_UUPER_THRESHOLD*0.10;			
			motor_settings.lower_threshold=torque_list[motor_param_un.system_motor_pattern[programNum].atcTorqueThresholdNum]*0.10;//torque_list[motor_param_un.system_motor_pattern[programNum].torqueThresholdNum]*0.10;//闂備浇顫夐鏍矗閸愵喖鏋侀柟鎹愵嚙濡﹢鏌曢崼婵囶棞妞ゅ繐鐖奸弻鈥愁吋娴ｆ彃浜鹃柛鎰级閻濐垶姊绘担鑺ョ《闁哥姵鍔欏鍛婄節濮橆剛顔嗛梺缁樺灱婵倝寮查幖浣圭厸闁稿本锚閳ь剚鐗滈埀顒佽壘缂嶅﹪寮婚妸鈺傚亜闁告稑锕︽导鍕⒑瑜版帩妫戦柛蹇旓耿瀵偊骞樼拠鍙夘棟闂侀潧鐗嗗Λ妤咁敂閼稿吀绻嗘い鏍ㄣ仜閸嬫挸鐣烽崶鈺婃敤濠电偠鎻徊钘夘焽閿熺姴鏋侀柟鎹愵嚙濡﹢鏌曢崼婵囶棞妞ゅ骏鎷� int  param=5闂傚倷娴囧▔鏇㈠窗閹版澘鍑犲┑鐘宠壘缁狀垶鏌ㄩ悤鍌涘5闂備胶枪椤戝懐鎹㈠鈧顐﹀箻鐠囧弶顥濋梺闈涚墕濡顢旈敓锟�5 闂傚倷娴囧▔鏇㈠窗閹版澘鍑犲┑鐘宠壘缁狀垶鏌ㄩ悤鍌涘-5 闂備胶枪椤戝懐鎹㈠鈧顐﹀箻鐠囧弶顥濋梺闈涚墕濡顢旈敓锟�5闂傚倷娴囧▔鏇㈠窗閹版澘鍑犲┑鐘宠壘缁狀垶鏌ㄩ悤鍌涘
			if(motor_param_un.system_motor_pattern[programNum].atcTorqueThresholdNum>torque20_Ncm)
			{
				motor_settings.lower_threshold=torque_list[torque20_Ncm]*0.10;
			}
		}
		else if(motor_param_un.system_motor_pattern[programNum].dir==EndoModePositionToggle)
		{
			motor_settings.upper_threshold=MAX_TORQUE_UUPER_THRESHOLD*0.10;//torque_list[motor_param_un.system_motor_pattern[programNum].recTorqueThresholdNum]*0.10;//MAX_TORQUE_UUPER_THRESHOLD*0.10;//motor_param_un.system_motor_pattern[programNum].torqueThreshold*0.16;
			motor_settings.lower_threshold=MAX_TORQUE_UUPER_THRESHOLD*0.06;//torque_list[motor_param_un.system_motor_pattern[programNum].recTorqueThresholdNum]*0.06;//MAX_TORQUE_UUPER_THRESHOLD*0.06;//闂備浇顫夐鏍矗閸愵喖鏋侀柟鎹愵嚙濡﹢鏌曢崼婵囶棞妞ゅ繐鐖奸弻鈥愁吋娴ｆ彃浜鹃柛鎰级閻濐垶姊绘担鑺ョ《闁哥姵鍔欏鍛婄節濮橆剛顔嗛梺缁樺灱婵倝寮查幖浣圭厸闁稿本锚閳ь剚鐗滈埀顒佽壘缂嶅﹪寮婚妸鈺傚亜闁告稑锕︽导鍕⒑瑜版帩妫戦柛蹇旓耿瀵偊骞樼拠鍙夘棟闂侀潧鐗嗗Λ妤咁敂閼稿吀绻嗘い鏍ㄣ仜閸嬫挸鐣烽崶鈺婃敤濠电偠鎻徊钘夘焽閿熺姴鏋侀柟鎹愵嚙濡﹢鏌曢崼婵囶棞妞ゅ骏鎷� int  par
		}
		else 
		{
			motor_settings.upper_threshold=torque_list[motor_param_un.system_motor_pattern[programNum].torqueThresholdNum]*0.10;
			motor_settings.lower_threshold=torque_list[motor_param_un.system_motor_pattern[programNum].torqueThresholdNum]*0.06;//60%		
		}				
		if(motor_param_un.system_motor_pattern[programNum].toggleSpeedNum>spd600_Rpm_num)
		{
			motor_param_un.system_motor_pattern[programNum].toggleSpeedNum=spd600_Rpm_num;
		}
		if(motor_param_un.system_motor_pattern[programNum].motorSpeedNum>MAX_spd_Rpm_num)
		{
			motor_param_un.system_motor_pattern[programNum].motorSpeedNum=MAX_spd_Rpm_num;
		}	
		motor_settings.forward_speed=speed_list[motor_param_un.system_motor_pattern[programNum].motorSpeedNum];
		motor_settings.reverse_speed=-speed_list[motor_param_un.system_motor_pattern[programNum].motorSpeedNum];	
		motor_settings.toggle_mode_speed=speed_list[motor_param_un.system_motor_pattern[programNum].toggleSpeedNum];//[motor_param_un.system_motor_pattern[programNum].motorSpeedNum];
	//to motor control	
		App_MotorControl(MOTOR_SETTING_UPDATE);	
		err=SUCCESS;
	}	
	return err;
}

//==================LOGO PAGE======================================
static void OLED_Display_LOGO(unsigned short int logoNum)
{		
	//OLED_ShowString(38,16,"ENDOART",24,1); 	
	//OLED_ShowString(0,48,"Ver-1.0.020241028",16,1); 	
	logo_disppaly();
}
//==================HOME  PAGE=====================================
static void OLED_Display_motorDirection(unsigned short int dir)
{	
//	OLED_DrawCircle(136,24,19);
    main_dir_fill(112,16, dir);	
}
/**
  * @brief 	motor param
  * @param   programNum ,uint8_t select
  * @retval none
  */
static void OLED_disp_motor_param(SYSTEM_MOTOR_PARAM* motorParam,unsigned char selectNum )
{
	uint32_t temp,temp2;	
	if(motorParam->dir==EndoModePositionToggle)
	{
		temp=speed_list[motorParam->toggleSpeedNum];
	}
	else temp=speed_list[motorParam->motorSpeedNum];	
	if(temp<1000)	
	{	
		OLED_ShowNum(40,24,temp,3,16,(selectNum!=1));	
		OLED_ShowChar(32,24,' ',16,(selectNum!=1));	
	}
	else
	{
		OLED_ShowNum(32,24,temp,4,16,(selectNum!=1));	
	}	
	temp=torque_list[motorParam->torqueThresholdNum];		
	if(motorParam->dir==EndoModePositionToggle)
	{
		temp=torque_list[motorParam->recTorqueThresholdNum];	
	}	
 	else if( motorParam->dir==EndoModeTorqueATC)
	{
		temp=torque_list[motorParam->atcTorqueThresholdNum];	
	}	
	temp2=(temp/10);
	OLED_ShowNum(40,48,temp2,1,16,(selectNum!=2));//int
	OLED_ShowChar(48,48,'.',16,(selectNum!=2));
	temp2=temp%10;
	OLED_ShowNum(52,48,temp2,1,16,(selectNum!=2));	
	if(motorParam->dir==EndoModePositionToggle)
	{
		OLED_ShowString(116,0,"REC",16,(selectNum!=3));
		//OLED_ShowString(48,44," REC ",16,(selectNum!=3));
		temp=abs(motorParam->reversePosition);		
		if(temp<100)
		{
			//OLED_ShowString(104,48,"  -",8,(selectNum!=4));
			OLED_ShowString(96,56,"   ",8,(selectNum!=4));
			OLED_ShowNum(114,56,temp,2,8,(selectNum!=4));
		}
		else
		{
			//OLED_ShowChar(110,48,'-',8,(selectNum!=4));
			OLED_ShowChar(102,56,' ',8,(selectNum!=4));
			OLED_ShowNum(108,56,temp,3,8,(selectNum!=4));
		}
		OLED_ShowChar(126,56,'/',8,1);
		temp=abs(motorParam->forwardPosition);	// angle
		if(temp<100)
		{			
			OLED_ShowNum(132,56,temp,2,8,(selectNum!=4));
			OLED_ShowChar(144,56,' ',8,(selectNum!=4));
		}
		else
		{
			OLED_ShowNum(132,56,temp,3,8,(selectNum!=5));
		}			
	}
	else if(motorParam->dir==EndoModeTorqueATC){//ATC		
		OLED_ShowString(116,0,"ATC",16,(selectNum!=3));		
		temp=abs(motorParam->reversePosition);	// angle
		if(temp<100)
		{
//			OLED_ShowString(104,48,"  -",8,(selectNum!=4));
			OLED_ShowString(96,56,"   ",8,(selectNum!=4));
			OLED_ShowNum(114,56,temp,2,8,(selectNum!=4));
		}
		else
		{
//		OLED_ShowChar(110,48,'-',8,(selectNum!=4));
			OLED_ShowChar(102,56,' ',8,(selectNum!=4));
			OLED_ShowNum(108,56,temp,3,8,(selectNum!=4));
		}
		OLED_ShowChar(126,56,'/',8,1);
		temp=abs(motorParam->forwardPosition);
		if(temp<100)
		{
			OLED_ShowChar(144,56,' ',8,(selectNum!=4));
			OLED_ShowNum(132,56,temp,2,8,(selectNum!=4));
		}
		else
		{
			OLED_ShowNum(132,56,temp,3,8,(selectNum!=5));
		}				
	}
	else if(motorParam->dir==EndoModeSpeedForward)
	 {//forward
		OLED_ShowString(116,0,"CW ",16,(selectNum!=3));	
		temp=360;	// angle	
		OLED_ShowNum(120,56,temp,3,8,1);		
		OLED_ShowString(96,56,"    ",8,1);
		OLED_ShowString(138,56,"  ",8,1);	
	}
	else if(motorParam->dir==EndoModeSpeedReverse){//reserve
		OLED_ShowString(116,0,"CCW",16,(selectNum!=3));		
		temp=360;
		OLED_ShowNum(120,56,temp,3,8,1);		
		OLED_ShowString(96,56,"   -",8,1);
		OLED_ShowString(138,56,"  ",8,1);			
	}		
		if((motorParam->dir)==EndoModeTorqueATC||(motorParam->dir)==EndoModePositionToggle)	
	{
		if(abs(motorParam->reversePosition)<abs(motorParam->forwardPosition))
		{
			OLED_Display_motorDirection(EndoModeTorqueATC);
		}
		else  OLED_Display_motorDirection(EndoModePositionToggle);
	}
	else OLED_Display_motorDirection(motorParam->dir);
}
/**
  * @brief 	home page
  * @param   programNum ,uint8_t select
  * @retval none
  */
static void OLED_Display_MAIN(unsigned short int programNum ,uint8_t select)
{	
	if(programNum<10) //programNum=10 only apex mode
	{
		 if(programNum==0)//P1-> GOLD 04
		{
			OLED_ShowString(32,0," GOLD 04 ",16,1);	
			OLED_ShowString(72,24,"rpm",16,1);		
		}	
		else if(programNum==1)//P2-> GOLD 06
		{
			OLED_ShowString(32,0," GOLD 06 ",16,1);
			OLED_ShowString(72,24,"rpm",16,1);
		}	
		else if(programNum==2)//P3-> BLUE 04
		{
			OLED_ShowString(32,0," BLUE 04 ",16,1);
			OLED_ShowString(72,24,"rpm",16,1);
		}	
		else if(programNum==3)//P4-> BLUE 06
		{
			OLED_ShowString(32,0," BLUE 06 ",16,1);
			OLED_ShowString(72,24,"rpm",16,1);
		}	
		else if(programNum==4)//P5-> PATH FILE
		{
			OLED_ShowString(32,0,"PATH FILE",16,1);
			OLED_ShowString(72,24,"rpm",16,1);
		}	
		else if(programNum==5)//P6-> EXPERT
		{
			OLED_ShowString(32,0,"  EXPERT ",16,1);
			OLED_ShowString(72,24,"rpm",16,1);
		}	
		else //if(programNum>5)//p7~P10
		{
			OLED_ShowString(32,0,"    ",16,1);//闂傚倷娴囧▔鏇㈠窗閹版澘鍑犲┑鐘宠壘缁狀垶鏌ｉ幋锝呅撻柡鍛倐閺岋繝宕掑Ο琛″亾閺嶎偀鍋撳顒傜Ш闁哄被鍔戦幃銏ゅ川婵犲嫪绱曢梻浣哥秺椤ユ捇宕楀鈧顐﹀箻缂佹ɑ娅㈤梺璺ㄥ櫐閹凤拷
			OLED_ShowChar(64,0,'P',16,1);	
			OLED_ShowNum(72,0,programNum-5,1,16,1);//闂備礁鎼€氼剚鏅舵禒瀣︽慨婵撴嫹1-P4
			//OLED_ShowChar(22,4,' ',16,1);
			OLED_ShowString(80,0,"    ",16,1);
			OLED_ShowString(72,24,"rpm",16,1);			
		}		
		OLED_ShowChar(68,48,'N',16,1);	OLED_ShowChar(76,43,'.',16,1);	OLED_ShowString(80,48,"cm",16,1);	
		OLED_disp_motor_param(&motor_param_un.system_motor_pattern[programNum],select);
	}
}
//==================SET MENU MOTOR or APEX=====================================
/**
  * @brief 	SubmenuEntranceMotorOrApex
  * @param  systemTime
  * @retval none
  */
static void SubmenuEntranceMotorOrApex(unsigned char motorOrApex)
{	
	OLED_ShowString(36,8,"SETTING FOR",16,1);			
	OLED_ShowString(32,40,"MOTOR",16,(motorOrApex!=(0+2)));
	OLED_ShowString(96,40,"APEX",16,(motorOrApex!=(1+2)));		
}
/**
  * @brief 	HomePageHandle
  * @param  rec_Signal,enable
  * @retval none
  */
static void HomePageHandle(task_notify_enum rec_Signal,unsigned char selectNum)
{
	error_status err;	
	if(selectNum==ADJUST_MOTOR_PARAM_PROGRAM_NUM) 
	{
		if(rec_Signal==motor_setting_updata_signal&&sys_param_un.device_param.use_p_num<10)
		{
			err = MenuMotorParamUpdate(sys_param_un.device_param.use_p_num);				
			if(err==SUCCESS)
			{
				OLED_Display_MAIN(sys_param_un.device_param.use_p_num ,ADJUST_MOTOR_PARAM_PROGRAM_NUM);//	
			}	
		}		
	}
	else 
	{		
		if(rec_Signal==add_button_press_signal)
		{	
			MenuConfigMotorParam(sys_param_un.device_param.use_p_num,selectNum,MENU_PARAM_ADD);									
		}	
		else if(rec_Signal==sub_button_press_signal)
		{	
			MenuConfigMotorParam(sys_param_un.device_param.use_p_num,selectNum,MENU_PARAM_SUB);	
		}			
		if(rec_Signal!=null_signal)
		{
			err = MenuMotorParamUpdate(sys_param_un.device_param.use_p_num);				
			if(err==SUCCESS)
			{
				OLED_Display_MAIN(sys_param_un.device_param.use_p_num ,selectNum);//	
			}						
		}
		
	}	
}
/**
  * @brief 	SubmenuMotorParam
  * @param angleValue
  * @retval none
  */
static void SubmenuMotorParam( unsigned char cwOrCcw , short int angleValue)
{
	unsigned short int Value;
	Value=abs(angleValue);
	if(cwOrCcw==CCW_ANGLE)
	{	
		OLED_ShowString(44,8,"CCW ANGLE",16,1);	
		OLED_ShowChar(64,40,'-',16,1);
		if(Value<100)
		{	
			OLED_ShowNum(72,40,Value,2,16,1);
			OLED_ShowChar(88,40,' ',16,1);			
		}
		else
		{
			OLED_ShowNum(72,40,Value,3,16,1);	
		}
	}
	else  if(cwOrCcw==CW_ANGLE)
	{		
		OLED_ShowString(44,8,"CW  ANGLE",16,1);		
		OLED_ShowChar(72,40,' ',16,1);
		if(Value<100)
		{		
			OLED_ShowNum(72,40,Value,2,16,1);
			OLED_ShowChar(88,40,' ',16,1);
		}
		else
		{
			OLED_ShowNum(72,40,Value,3,16,1);
		}			
	}
}

/**
  * @brief 	SubmenuApexControlMode
  * @param angleValue
  * @retval none
  */
static void SubmenuApexControlMode( unsigned char apexMode,unsigned char enableFlag)
{
	if(apexMode==AUTO_START)//auto start 
	{		
		OLED_ShowString(40,8,"AUTO START",16,1);	
		OLED_ShowString(48,40,"OFF",16,(enableFlag!=0));	
		OLED_ShowString(88,40,"ON",16,(enableFlag!=1));
	}
	else if(apexMode==AUTO_STOP)//auto stop 
	{
		OLED_ShowString(40,8,"AUTO  STOP",16,1);	
		OLED_ShowString(48,40,"OFF",16,(enableFlag!=0));	
		OLED_ShowString(88,40,"ON",16,(enableFlag!=1));
	}
	else if(apexMode==APICAL_ACTION)//auto rec or stop
	{
		OLED_ShowString(28,8,"APICAL ACTION",16,1);	
		OLED_ShowString(32,40,"OFF",16,(enableFlag!=0));	
		OLED_ShowString(64,40,"REV",16,(enableFlag!=1));//REV
		OLED_ShowString(96,40,"STOP",16,(enableFlag!=2));
	}	
}
/**
  * @brief 	SubmenuApexControlMode
  * @param angleValue
  * @retval none
  */
static void SubmenuApexOffeset(	unsigned short int offeset)
{
		Disp_ApexPreForSet(60);		
		ApexBarFlashForSet(0,0,0);
}
//==================handness=====================================
/**
  * @brief 	SubmenuHandness
  * @param  systemTime
  * @retval none
  */
static void SubmenuHandness(unsigned char handness)
{	
//	OLED_ShowString(32,0,"Handness",24,1);			
//	OLED_ShowString(20,36,"Left",24,(handness!=0));			
//	OLED_ShowString(80,36,"Right",24,(handness!=1));	
		OLED_ShowString(48,8,"HANDNESS",16,1);			
		OLED_ShowString(40,40,"LEFT",16,(handness!=0));			
		OLED_ShowString(80,40,"RIGHT",16,(handness!=1));
}
//==================calibration=====================================
/**
  * @brief 	SubmenuCalibration
  * @param  flag
  * @retval none
  */
static void SubmenuCalibration(unsigned char flag)
{	
	if(flag==0||flag==1)
	{
		OLED_ShowString(16,8,"AUTO CALIBRATION",16,1);
		OLED_ShowString(22,40,"NO",16,(flag!=0));	
		OLED_ShowString(68,40,"YES",16,(flag!=1));	
	}
	else if(flag==2) 
	{
		OLED_ShowString(16,8," CALIBRATING... ",16,1);		
		OLED_ShowString(0,40,"   KEEP NO-LOAD     ",16,1);
	}
	else if(flag==3)
	{
		OLED_ShowString(16,8,"AUTO CALIBRATION",16,1);
		OLED_ShowString(0,40,"      FINISHED      ",16,1);
	}
	else
	{
		OLED_ShowString(16,8,"AUTO CALIBRATION",16,1);
		OLED_ShowString(0,40,"        FAILED      ",16,1);
	}
}

//==================restore default=====================================
/**
  * @brief 	SubmenuRestoreParam
  * @param  systemTime
  * @retval none
  */
static void SubmenuRestoreParam(unsigned char flag)
{	
	if(flag<2)
	{
		OLED_ShowString(8,8," RESTORE SETTINGS ",16,1);
		OLED_ShowString(22,40,"NO",16,(flag!=0));
		OLED_ShowString(68,40,"YES",16,(flag!=1));
	}
	else
	{
//		OLED_ShowString(16,8,"Restore Settings",16,1);
		 OLED_ShowString(8,8,"RESTORE SUCCESSFUL",16,1);
		OLED_ShowString(0,40,"  POWER OFF DEVICE  ",16,1);	
	}		
}

/**
  * @brief 	SubmenuPageHandle
  * @param  systemTime
  * @retval none
  */
static  void SubmenuPageHandle(unsigned short int subPageID,unsigned short int signal,unsigned int systemTime,unsigned char firstFlag)
{
	error_status err;
	static unsigned char secendNum,retCaliFanish=0;		
//	static unsigned char motorOrApex=0;	
	static unsigned int recSystemTime;
	static unsigned char blinkFlag;
	if(firstFlag==0)	
	{		
		if(subPageID==HANDEDNESS)
		{
			secendNum=sys_param_un.device_param.use_hand;	
		}		
		else 
		{
			secendNum=0;
			retCaliFanish=0;	
			recSystemTime =	systemTime;	
			timerCountDown(systemTime/1000,0,0)	;//start 10 			
		}		
		if(signal==motor_setting_updata_signal)	
		{		
			if(subPageID==CW_ANGLE||subPageID==CCW_ANGLE)	
			{
				SubmenuMotorParam( CW_ANGLE , motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].forwardPosition);					
				SubmenuMotorParam( CCW_ANGLE , motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].reversePosition);	
			}			
			MenuMotorParamUpdate(sys_param_un.device_param.use_p_num);	
		}			
	}
	else
	{	
		switch(subPageID)
		{		
			#ifdef APEX_FUNCTION_EBABLE	
			case SETTING_FOR:	 
				if(systemTime-recSystemTime>500)	
				{
					recSystemTime=systemTime;
					blinkFlag=(blinkFlag==0)?2:0;
					SubmenuEntranceMotorOrApex((firstFlag-1)|blinkFlag);
				}	
				if(signal==add_button_press_signal||signal==sub_button_press_signal)
				{
					SubmenuEntranceMotorOrApex((firstFlag-1)|blinkFlag);
				}	
			break;			
			case AUTO_START:	
				 if(signal==add_button_press_signal||signal==sub_button_press_signal)
				{		
					sys_param_un.device_param.auto_start_flag++;							
					sys_param_un.device_param.auto_start_flag%=2;
					SubmenuApexControlMode( AUTO_START,sys_param_un.device_param.auto_start_flag);		
				}		
			break;	
			case AUTO_STOP:
				if(signal==add_button_press_signal||signal==sub_button_press_signal)
				{	
					sys_param_un.device_param.auto_stop_flag++;							
					sys_param_un.device_param.auto_stop_flag%=2;
					SubmenuApexControlMode( AUTO_STOP,sys_param_un.device_param.auto_stop_flag);		
				}	
			break;	
			case APICAL_ACTION:			
				if(signal==add_button_press_signal||signal==sub_button_press_signal)
				{
					sys_param_un.device_param.apical_action_flag++;
					sys_param_un.device_param.apical_action_flag%=3;			
					SubmenuApexControlMode( APICAL_ACTION,sys_param_un.device_param.apical_action_flag);		
				}		
			break;
			case 	APICAL_OFFESET:				
				if(signal==sub_button_press_signal)
				{
					sys_param_un.device_param.ref_tine++;
					sys_param_un.device_param.ref_tine%=13;			
					SubmenuApexOffeset( sys_param_un.device_param.ref_tine);	
				}
				else if(signal==add_button_press_signal)
				{
					if(sys_param_un.device_param.ref_tine==0)
					{
						sys_param_un.device_param.ref_tine=12;
					}
					else 
					{
						sys_param_un.device_param.ref_tine--;
						sys_param_un.device_param.ref_tine%=13;		
					}
					SubmenuApexOffeset( sys_param_un.device_param.ref_tine);	
				}
				else
				{						
					ApexBarFlashForSet(xTaskGetTickCount(),1, 30);
				}	
			break;
			#endif
			case CW_ANGLE:
				if(signal==add_button_press_signal)
				{						
					motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].forwardPosition+=MINIMUM_ROTOR_ANGLE;
					if(motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].forwardPosition>360) motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].forwardPosition=MINIMUM_ROTOR_ANGLE;
					SubmenuMotorParam( CW_ANGLE , motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].forwardPosition);
				}
				else if(signal==sub_button_press_signal)
				{
					if(motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].forwardPosition<MINIMUM_ROTOR_ANGLE+MINIMUM_ROTOR_ANGLE) motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].forwardPosition=360;
					else motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].forwardPosition-=MINIMUM_ROTOR_ANGLE;	
					SubmenuMotorParam( CW_ANGLE , motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].forwardPosition);						
				}	
			break;	
			case CCW_ANGLE:
				if(signal==add_button_press_signal)
				{
					motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].reversePosition-=MINIMUM_ROTOR_ANGLE;
					if(motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].reversePosition<-360) motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].reversePosition=-MINIMUM_ROTOR_ANGLE;
					SubmenuMotorParam( CCW_ANGLE , motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].reversePosition);	
				}
				else if(signal==sub_button_press_signal)
				{
					motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].reversePosition+=MINIMUM_ROTOR_ANGLE;	
					if(motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].reversePosition>-MINIMUM_ROTOR_ANGLE)	
					{	
					motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].reversePosition=-360;	
					}			
					SubmenuMotorParam( CCW_ANGLE , motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].reversePosition);	
				}									
			break;			
			case HANDEDNESS:	
				if(signal==add_button_press_signal||signal==sub_button_press_signal)
				{	
					sys_param_un.device_param.use_hand++;
					sys_param_un.device_param.use_hand%=2;	
					SubmenuHandness(sys_param_un.device_param.use_hand);
				}					
			break;
			case AUTO_CALIBRATION:
				if(signal==add_button_press_signal||signal==sub_button_press_signal)
				{	
					secendNum++;//on or off
					secendNum%=2;	
					SubmenuCalibration(secendNum);
				}
				if(secendNum==0)
				{
					timerCountDown(systemTime/1000,10,0);				
				}
				else if(secendNum==1)
				{						
					err=timerCountDown(systemTime/1000,10,1)	;//start 10 
					if(err==SUCCESS)	
					{
						secendNum=2;//start
						retCaliFanish=0;
						SubmenuCalibration(2);
//						xSemaphoreGive(xSemaphoreApexAutoCalibration);							
					}	
				}
				else if(secendNum==2)
				{
					vTaskDelay(35);//wait apex calibration
					secendNum=3;
				}
				else if(secendNum==3)
				{		//start								
							
					#ifdef WDT_ENABLE							
					vTaskSuspend(WDT_ManageTask_Handle);								
					Wdt_Reset(MAX_WDT_FEED_TIME_MS*32);	//MAX_WDT_FEED_TIME_MS*32=9600mS						
					#endif 						
					retCaliFanish=App_MotorControl(MOTOR_MODE_SEARCH_ANGLE);						
					#ifdef WDT_ENABLE
					Wdt_Init();												
					xEventGroupSetBits(WDTEventGroup,MENU_TASK_EVENT_BIT);		
					vTaskResume(WDT_ManageTask_Handle);																
					#endif				
					if(retCaliFanish==0)
					{//	disp indicate	
						SubmenuCalibration(4);		
					}
					else
					{	//successful								
						SubmenuCalibration(3);
					}	
					xSemaphoreGive(xSemaphoreDispRfresh);	
					MenuIdleTimeManage(systemTime,0);	
					DeviceOffTimeManage(systemTime,0);							
					#ifdef  APEX_FUNCTION_EBABLE							
					xSemaphoreGive(xSemaphoreApexAutoCalibration);
					#endif								
					#ifdef WDT_ENABLE																		
					xEventGroupSetBits(WDTEventGroup,MENU_TASK_EVENT_BIT);
					vTaskDelay(MAX_WDT_FEED_TIME_MS);//wait wdt dog reset							
					xEventGroupSetBits(WDTEventGroup,MENU_TASK_EVENT_BIT);
					vTaskDelay(MAX_WDT_FEED_TIME_MS);//wait wdt dog reset
					xEventGroupSetBits(WDTEventGroup,MENU_TASK_EVENT_BIT);	
					vTaskDelay(MAX_WDT_FEED_TIME_MS);//wait wdt dog reset
					xEventGroupSetBits(WDTEventGroup,MENU_TASK_EVENT_BIT);	
					#else
					vTaskDelay(900);//wait wdt dog reset			
					#endif															
					secendNum=4;//end								
				}		
				else
				{
				//wait idle tiem
					signal=MENU_HOME_PAGE;//exit
					xQueueSend(xQueueMenuValue, &signal, 0);
				}	
				break;
				case RESTORE_DEFAULT_SETTING:
					if(signal==add_button_press_signal||signal==sub_button_press_signal)
					{	
						secendNum++;//on or off
						secendNum%=2;	
						SubmenuRestoreParam(secendNum);						
					}
					if(secendNum==0)
					{
						timerCountDown(systemTime/1000,10,0);
					}
					else if(secendNum==1)
					{
						err=timerCountDown(systemTime/1000,10,1)	;//start 10 
						if(err==SUCCESS)	
						{	
							SubmenuRestoreParam(2);
							xSemaphoreGive(xSemaphoreDispRfresh);	
							secendNum=2;	
							MenuIdleTimeManage(xTaskGetTickCount(),0);	
							#ifdef WDT_ENABLE																		
							xEventGroupSetBits(WDTEventGroup,MENU_TASK_EVENT_BIT);
							vTaskDelay(MAX_WDT_FEED_TIME_MS);//wait wdt dog reset							
							xEventGroupSetBits(WDTEventGroup,MENU_TASK_EVENT_BIT);
							vTaskDelay(MAX_WDT_FEED_TIME_MS);//wait wdt dog reset
							xEventGroupSetBits(WDTEventGroup,MENU_TASK_EVENT_BIT);
							vTaskDelay(MAX_WDT_FEED_TIME_MS);//wait wdt dog reset
							xEventGroupSetBits(WDTEventGroup,MENU_TASK_EVENT_BIT);							
							#else
							vTaskDelay(900);//wait wdt dog reset			
							#endif
						}								
					}
					else
					{						
						MenuDevicePowerOff(2);															
					}						
					break;
			default:
			break;		 
	}	
	if(signal!=null_signal)
	{						
		if(subPageID==CW_ANGLE)
		{
			SubmenuMotorParam( CW_ANGLE , motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].forwardPosition);
			MenuMotorParamUpdate(sys_param_un.device_param.use_p_num);							
		}
		else  if(secendNum==CCW_ANGLE)
		{
			SubmenuMotorParam( CCW_ANGLE , motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].reversePosition);	
			MenuMotorParamUpdate(sys_param_un.device_param.use_p_num);
		}		
	}
}	
	
}
/**
  * @brief 	SubmenuPageTurns
  * @param  systemTime
  * @retval none
  */
static  void SubmenuPageTurns(unsigned short int subPageID)
{		
	OLED_Clear_no_fresh();
	SubmenuPageHandle(subPageID,null_signal,0,0);//init
	switch(subPageID)
	{	
		#ifdef APEX_FUNCTION_EBABLE	
		case SETTING_FOR:				
			if(sys_param_un.device_param.use_p_num==10&&sys_param_un.device_param.apexFunctionLoad!=0)	
			{
				SubmenuEntranceMotorOrApex(1);	
			}	
			else 
			{
				SubmenuEntranceMotorOrApex(0);	
			}	
//			SubmenuEntranceMotorOrApex(motorOrApexSetFlag);		
		break;			
		case AUTO_START:
			SubmenuApexControlMode( AUTO_START,sys_param_un.device_param.auto_start_flag);
		break;	
		case AUTO_STOP:
			SubmenuApexControlMode( AUTO_STOP,sys_param_un.device_param.auto_stop_flag);
		break;	
		case APICAL_ACTION:
			SubmenuApexControlMode( APICAL_ACTION,sys_param_un.device_param.apical_action_flag);
		break;
		case APICAL_OFFESET:
			SubmenuApexOffeset(	sys_param_un.device_param.ref_tine);
		break;
		#endif
		case CW_ANGLE:
			SubmenuMotorParam( CW_ANGLE , motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].forwardPosition);
		break;	
		case CCW_ANGLE:
			SubmenuMotorParam( CCW_ANGLE , motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].reversePosition);
		break;		
		case HANDEDNESS:	
			SubmenuHandness(sys_param_un.device_param.use_hand);			
		break;
		case AUTO_CALIBRATION:
			SubmenuCalibration(0);
		break;
		case RESTORE_DEFAULT_SETTING:
			SubmenuRestoreParam(0);
		break;	
		default:
		break;		 
	}	
}

//==================DISP in work mode  MOTOR AND APEX   picture=====================================
/**
  * @brief 	disp_TorqueOnly
  * @param   u8 x,u8 y,u8 torqueValue 
  * @retval none
  */
static void Disp_TorqueOnly(u8 x,u8 y ,unsigned short int torqueValue)
{
	unsigned short int temp;	
	temp=TorqueRangeCheck(torqueValue)/10;
	OLED_ShowNum(x,y,temp,1,24,1);
	OLED_ShowChar(x+12,y,'.',24,1);
	temp=TorqueRangeCheck(torqueValue)%10;
	OLED_ShowNum(x+20,y,temp,1,24,1);	
		
}
/**
  * @brief 	TorqueProgressBar
  * @param   u8 x,u8 y,start coord 
  * @retval none
  */
static void Disp_TorqueProgressBar(u8 x,u8 y ,u8 value)
{
	u8 i,j,x1,y1;
	//num
	x1=x;
	y1=y;
	for(i=0;i<5;i++)
	{
		j=5-i;	
		if(j>3)
		{			
			OLED_ShowNum(x1-3+i*25,y1+4,j,1,8,1);//>4 
			OLED_DrawPoint(x1+3+i*25, y1+9,1);
			OLED_DrawPoint(x1+4+i*25, y1+9,1);
			OLED_DrawPoint(x1+3+i*25, y1+10,1);
			OLED_DrawPoint(x1+4+i*25, y1+10,1);		
			OLED_ShowNum(x1+5+i*25,y1+4,0,1,8,1);			
		}		
		else
		{			
			OLED_ShowNum(x1+i*25,y1+4,j,1,8,1);			
			OLED_DrawPoint(x1+7+i*25, y1+9,1);
			OLED_DrawPoint(x1+8+i*25, y1+9,1);
			OLED_DrawPoint(x1+7+i*25, y1+10,1);
			OLED_DrawPoint(x1+8+i*25, y1+10,1);		
			OLED_ShowNum(x1+9+i*25,y1+4,0,1,8,1);
		}				
	}	
	x1=x;
	y1=y+13;
	for(i=0;i<2;i++)	//	  
	{
		for(j=0;j<5;j++)
		{
			if(j<2)
			{
				OLED_DrawPoint(x1+3+j*25, y1+i,1);
				OLED_DrawPoint(x1+4+j*25, y1+i,1);
			}
			else
			{
				OLED_DrawPoint(x1+7+j*25, y1+i,1);
				OLED_DrawPoint(x1+8+j*25, y1+i,1);		
			}	
		}
	  disp_DrawRow(x1,y1+i+1,120,1);		
	}	
}
/**
  * @brief 	TorqueProgressBarFlash
  * @param  systemTimeMs, startFlag 
  * @retval none
  */
static void TorqueProgressBarFlash(unsigned int systemTimeMs,unsigned char startFlag,unsigned short torqueValue)
{
	static unsigned int recTimeMs;
	static unsigned char blinkFlag;//
	static unsigned short torqueReference,maxTorqueNum;
	unsigned char i,j,x,y,m,torqueNum;	
	
	x=52;//40+16-8;//bar
	y=18;//16+2;		
	if(startFlag==0)//preparation work
	{
		recTimeMs=systemTimeMs;
		blinkFlag=0;	
		torqueValue=TorqueRangeCheck(torqueValue);
		if(motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].dir==EndoModePositionToggle)
		{
			torqueValue=TorqueRangeCheck(torque_list[MAX_torque_42_Ncm]);
		}
		else if(motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].dir==EndoModeTorqueATC)
		{
			if(torqueValue>torque_list[torque20_Ncm])
			{
				torqueValue=torque_list[torque20_Ncm];
			}			
		}	
		maxTorqueNum=(torque_list[MAX_torque_42_Ncm]-30)*2/5+13;	
   		if(maxTorqueNum>21)		 maxTorqueNum=21;	
		if(torqueValue>50)
		{
			torqueReference=maxTorqueNum;
		}
		else  if(torqueValue>30)
		{
			torqueReference=(torqueValue-30)*2/5+13;
		}
		else
		{
			if(torqueValue<5)
			{
				torqueReference=0;
			}
			else
			{
				torqueReference=((torqueValue-5)>>1);
			}
		}
		if(torqueReference>maxTorqueNum) torqueReference=maxTorqueNum;		
		i=torqueReference;		
		j=i*5;
		if(i<7)
		{
//			disp_DrawColumn(x+100-j,y,12+i,0);
			disp_DrawColumn(x+101-j,y+1,12+i,1);
			disp_DrawColumn(x+102-j,y,14+i,1);
			disp_DrawColumn(x+103-j,y+1,12+i,1);	
//			disp_DrawColumn(x+104-j,y,12+i,0);
		}
		else if(i<13)	
		{
//			disp_DrawColumn(x+100-j,y,i*3,0);
			disp_DrawColumn(x+101-j,y+1,i*3,1);
			disp_DrawColumn(x+102-j,y,i*3+2,1);
			disp_DrawColumn(x+103-j,y+1,i*3,1);	
//			disp_DrawColumn(x+104-j,y,i*3,0);
		}	
		else
		{
			j=i*6-11;
//		disp_DrawColumn(x+100-j,y,40,0);
			disp_DrawColumn(x+101-j,y+2,36,1);
			disp_DrawColumn(x+102-j,y+1,38,1);
			disp_DrawColumn(x+103-j,y,40,1);
			disp_DrawColumn(x+104-j,y+1,38,1);
			disp_DrawColumn(x+105-j,y+2,36,1);	
		}			
	}	
	else 
	{					
		if(systemTimeMs-recTimeMs>500)
		{			
			torqueValue = TorqueRangeCheck(torqueValue);					
			if(torqueValue>30)
			{
				torqueNum=(torqueValue-30)*2/5+13;
			}
			else
			{
				if(torqueValue<5)
				{
					torqueNum=0;
				}
				else
				{
					torqueNum=(torqueValue-5)>>1;//offset
				}
			}						
			if(motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].dir!=EndoModeTorqueATC)
			{
//				if(torqueNum>torqueReference)//闂傚倷娴囧▔鏇㈠窗閺囩喓绠鹃柛銉墮缁€宀勬煃閳轰礁鏆欐い蹇撶埣濮婅櫣鎹勯妸銉︾€鹃梺鍝勵儏椤兘鐛箛鏃戞Ч閹煎瓨绋愮划锟�
//				{
//					torqueNum=torqueReference;
//			}		
			}	
			if(torqueNum>maxTorqueNum) torqueNum=maxTorqueNum;						
			for(i=0;i<22;i++)
			{
				m=(i<torqueNum)?1:0;
				j=i*5;					
				if(i<7)
				{
//					disp_DrawColumn(x+100-j,y,12+i,0);
					disp_DrawColumn(x+101-j,y+1,12+i,m);
					disp_DrawColumn(x+102-j,y,14+i,m);
					disp_DrawColumn(x+103-j,y+1,12+i,m);
//					disp_DrawColumn(x+104-j,y,12+i,0);							
				}
				else if(i<13)
				{					
//					disp_DrawColumn(x+100-j,y,i*3,0);
					disp_DrawColumn(x+101-j,y+1,i*3,m);
					disp_DrawColumn(x+102-j,y,i*3+2,m);
					disp_DrawColumn(x+103-j,y+1,i*3,m);
//					disp_DrawColumn(x+104-j,y,i*3,0);			
				}
				else
				{
					j=6*i-11;
//					disp_DrawColumn(x+100-j,y,40,0);
					disp_DrawColumn(x+101-j,y+2,36,m);
					disp_DrawColumn(x+102-j,y+1,38,m);
					disp_DrawColumn(x+103-j,y,40,m);
					disp_DrawColumn(x+104-j,y+1,38,m);
					disp_DrawColumn(x+105-j,y+2,36,m);			
				}		
			}	
				//torque//test
//				Disp_TorqueOnly(124,40,torqueValue);
//				if(torqueValue>torque_list[motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].torqueThresholdNum]	)

//				{
//					Disp_TorqueOnly(124,40,torqueValue);
//				}
//				else
//				{
//					Disp_TorqueOnly(124,40,torque_list[motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].torqueThresholdNum]);
//				}								
			recTimeMs=systemTimeMs;	
			blinkFlag=(blinkFlag)?0:1;				
			i=torqueReference;
			j=i*5;
			if(i<7)
			{
//				disp_DrawColumn(x+100-j,y,12+i,0);
				disp_DrawColumn(x+101-j,y+1,12+i,blinkFlag);
				disp_DrawColumn(x+102-j,y,14+i,blinkFlag);
				disp_DrawColumn(x+103-j,y+1,12+i,blinkFlag);
				disp_DrawColumn(x+104-j,y,12+i,0);		
			}
			else if(i<13)
			{
//				disp_DrawColumn(x+100-j,y,i*3,0);
				disp_DrawColumn(x+101-j,y+1,i*3,blinkFlag);
				disp_DrawColumn(x+102-j,y,i*3+2,blinkFlag);
				disp_DrawColumn(x+103-j,y+1,i*3,blinkFlag);
			}		
			else
			{
				j=i*6-11;
//				disp_DrawColumn(x+100-j,y,38,0);
				disp_DrawColumn(x+101-j,y+2,36,blinkFlag);
				disp_DrawColumn(x+102-j,y+1,38,blinkFlag);
				disp_DrawColumn(x+103-j,y,40,blinkFlag);
				disp_DrawColumn(x+104-j,y+1,38,blinkFlag);
				disp_DrawColumn(x+105-j,y+2,36,blinkFlag);	
			}				
//			xTaskNotifyGive(beepTemporaryTask_Handle);					
				xSemaphoreGive(xSemaphoreDispRfresh);	
		}
	}
}

/**
  * @brief 	TorqueProgressBarFlash
  * @param  systemTimeMs, startFlag 
  * @retval none
  */
//startFlag0,reset;1
static void ApexProgressBarFlash(unsigned int systemTimeMs,unsigned char startFlag, short int apexValue)
{
	static unsigned int recTimeMs,blinkTimeMs;
	static unsigned short blink,recApexValue,temp,apex_flag=1;
	unsigned char i,j,x,y,m,sendBuff;	
	if(startFlag==0)//preparation work
	{
		recTimeMs=systemTimeMs;
//	recApexValue=33;//<0 ,闂傚倷娴囧▔鏇㈠窗閹版澘姹查柍褜鍓熼弻鐔煎箒閹烘垵濮风紓鍌氱Т濞差參鐛箛娑欐櫢闁跨噦鎷�<3闂傚倷娴囧▔鏇㈠窗閹版澘鍑犲┑鐘宠壘缁狀垶鏌ㄩ悤鍌涘23,4闂傚倷娴囧▔鏇㈠窗閹版澘鍑犲┑鐘宠壘缁狀垶鏌ｉ幋锝呅撻柡鍛倐閺岋繝宕掑Ο琛″亾閺嶎偀鍋撳顒傜Ш闁哄被鍔戦幃銏ゅ川婵犲嫪绱曢梻浣哥秺椤ユ捇宕楀鈧顐﹀箻缂佹ɑ娅㈤梺璺ㄥ櫐閹凤拷
		recApexValue=0;
		temp=0;
//	OLED_ShowString(110,40,"APEX",24,1);
	}		
	else
	{	
		if(systemTimeMs<recTimeMs)		 recTimeMs=systemTimeMs;
		if(systemTimeMs-recTimeMs>10)
		{	
			recTimeMs=systemTimeMs;			
			if(apexValue<1)
			{			
//				if(temp!=0)
				recApexValue=33;
				temp=0;
			}
			else temp=apexValue;					
			// over area (40~60)
			if(recApexValue!=temp)
			{
				if(temp>=30) recApexValue=30;//
				if(temp==0) recApexValue=temp;//闂備胶枪缁诲牓宕濋幋锕€鏋侀柟鎹愵嚙濡﹢鏌曢崼婵囶棞妞ゅ骏鎷�
				if(recApexValue>temp) 
				{
					if(recApexValue==1) recApexValue=0;					
					else recApexValue--;
				}
				else if(recApexValue<temp)
				{
					recApexValue++;	
				}
				else 	recApexValue=temp;//fresh			
				x=33;//35;
				y=16;	
				m=x;
				for(i=0;i<3;i++)
				{		
					x=m+i*5;		
					if(recApexValue==0)
					{
	//				disp_DrawColumn(x,y,40,0);
						disp_DrawColumn(x+1,y+1,38,1);
						disp_DrawColumn(x+2,y,40,1);
						disp_DrawColumn(x+3,y,40,1);
						disp_DrawColumn(x+4,y+1,38,1);
					}
					else if(recApexValue>25)
					{
						disp_DrawColumn(x,y,40,0);
						disp_DrawColumn(x+1,y+1,38,0);
						disp_DrawColumn(x+2,y,40,0);
						disp_DrawColumn(x+3,y,40,0);
						disp_DrawColumn(x+4,y+1,38,0);
					}
					else
					{	
						j=(i<recApexValue)?0:1;					
	//				disp_DrawColumn(x,y,40,0);
						disp_DrawColumn(x+1,y+1,38,j);
						disp_DrawColumn(x+2,y,40,j);
						disp_DrawColumn(x+3,y,40,j);
						disp_DrawColumn(x+4,y+1,38,j);
					}	
				}	
				// apex  area (60~100)
				x=50;//52;
				y=16;
				m=x;
				for(i=0;i<7;i++)
				{
					x=m+i*6;
					if(recApexValue==0)
					{
	//				disp_DrawColumn(x,y,40,0);
						disp_DrawColumn(x+1,y+2,36,1);
						disp_DrawColumn(x+2,y+1,38,1);
						disp_DrawColumn(x+3,y,40,1);
						disp_DrawColumn(x+4,y+1,38,1);
						disp_DrawColumn(x+5,y+2,36,1);
					}
					else if(recApexValue>25)
					{
	//				disp_DrawColumn(x,y,40,0);
						if(i==sys_param_un.device_param.ref_tine)
						{
							disp_DrawColumn(x+1,y+2,36,1);
							disp_DrawColumn(x+2,y+1,38,1);
							disp_DrawColumn(x+3,y,40,1);
							disp_DrawColumn(x+4,y+1,38,1);
							disp_DrawColumn(x+5,y+2,36,1);						
	//						disp_DrawRow(x+3,y+41,1,1);
	//						disp_DrawRow(x+2,y+42,3,1);
	//						disp_DrawRow(x+1,y+43,5,1);
						}
						else
						{						
							disp_DrawColumn(x+1,y+2,36,0);
							disp_DrawColumn(x+2,y+1,38,0);
							disp_DrawColumn(x+3,y,40,0);
							disp_DrawColumn(x+4,y+1,38,0);
							disp_DrawColumn(x+5,y+2,36,0);						
						}					
					}
					else 
					{
						j=((i+3)<recApexValue)?0:1;
	//					disp_DrawColumn(x,y,40,0);
						disp_DrawColumn(x+1,y+2,36,j);
						disp_DrawColumn(x+2,y+1,38,j);
						disp_DrawColumn(x+3,y,40,j);
						disp_DrawColumn(x+4,y+1,38,j);
						disp_DrawColumn(x+5,y+2,36,j);			
					}		
				}
				// oter area  闂傚倷娴囧▔鏇㈠窗閹版澘鍑犲┑鐘宠壘缁狀垶鏌ㄩ悤鍌涘100~160)
				x=94;//96;
				y=16;
				m=x;
				for(i=0;i<15;i++)
				{
					x=m+i*4;
					if(recApexValue==0)
					{
	//				disp_DrawColumn(x,y,20,0);				
						if(i<4)
						{
							disp_DrawColumn(x+1,y+1,18+20-i*6,1);
							disp_DrawColumn(x+2,y,20+20-i*6,1);
							disp_DrawColumn(x+3,y+1,18+20-i*6,1);
						}
						else
						{
							disp_DrawColumn(x+1,y+1,17,1);
							disp_DrawColumn(x+2,y,19,1);
							disp_DrawColumn(x+3,y+1,17,1);
						}					
					}
					else if(recApexValue>25)
					{
	//				disp_DrawColumn(x,y,20,0);
					if(i+7==sys_param_un.device_param.ref_tine)//闂傚倷娴囧▔鏇㈠窗閺囩喓绠鹃柛銉墮缁€宀勬煃閳轰礁鏆欐い蹇撶埣濮婅櫣鎹勯妸銉︾€鹃梺鍝勵儏椤兘鐛箛娑欐櫢闁跨噦鎷�
					{
						if(i<4)
						{
							disp_DrawColumn(x+1,y+1,18+20-i*6,1);
							disp_DrawColumn(x+2,y,20+20-i*6,1);
							disp_DrawColumn(x+3,y+1,18+20-i*6,1);
						}
						else
						{
							disp_DrawColumn(x+1,y+1,17,1);
							disp_DrawColumn(x+2,y,19,1);
							disp_DrawColumn(x+3,y+1,17,1);
						}	
					}
					else 
					{
						if(i<4)
						{
							disp_DrawColumn(x+1,y+1,18+20-i*6,0);
							disp_DrawColumn(x+2,y,20+20-i*6,0);
							disp_DrawColumn(x+3,y+1,18+20-i*6,0);
						}
						else
						{
							disp_DrawColumn(x+1,y+1,17,0);
							disp_DrawColumn(x+2,y,19,0);
							disp_DrawColumn(x+3,y+1,17,0);
						}									
					}						
				}
				else
				{		
					j=((i+11)<recApexValue)?0:1;					
					//disp_DrawColumn(x,y,20,0);
					if(i<4)
					{
						disp_DrawColumn(x+1,y+1,18+20-i*6,j);
						disp_DrawColumn(x+2,y,20+20-i*6,j);
						disp_DrawColumn(x+3,y+1,18+20-i*6,j);
					}
					else
					{
						disp_DrawColumn(x+1,y+1,17,j);
						disp_DrawColumn(x+2,y,19,j);
						disp_DrawColumn(x+3,y+1,17,j);
					}					
				}	
			}				
			if(apexValue<0)//闂傚倷娴囧▔鏇㈠窗閹版澘鍑犲┑鐘宠壘缁狀垶鏌ｉ妶搴＄仯闁绘柨鎳樺铏规崉閵娿儲鐎鹃梺鍝勵儏椤兘鐛箛娑欏€婚柤鎭掑劜濞呫垽姊虹捄銊ユ珢闁瑰嚖鎷�
			{					
//				OLED_ShowString(110,40," -- ",24,1);
			}
			else if(apexValue<3)
			{
				OLED_ShowString(106,40,"OVER",24,1);//110,40,"OVER",24,1);
			}
			else if(apexValue>25)
			{		
				if(apex_flag==1)
				{
					OLED_ShowString(106,40,"APEX",24,1);//(110,40,"APEX",24,1);
					apex_flag=0;
//					APEXIdleTimeManage(xTaskGetTickCount(),0);
				}								
			}
			else
			{			
				if(recApexValue<sys_param_un.device_param.ref_tine+3) m=0;
				else m=recApexValue-sys_param_un.device_param.ref_tine-3;
				OLED_ShowChar(106,40,' ',24,1);//(110,40,' ',24,1);
				if(m<10)
				{
					if(recApexValue<sys_param_un.device_param.ref_tine+3)
					{
						OLED_ShowChar(118,40,'-',24,1);//(122,40,'-',24,1);
						OLED_ShowChar(130,40,'-',24,1);//(134,40,'-',24,1);				
					}
					else 
					{
						OLED_ShowChar(118,40,' ',24,1);//(122,40,' ',24,1);
						OLED_ShowNum(130,40,m,1,24,1);//(134,40,m,1,24,1);
					}					
				}
				else
				{
					OLED_ShowNum(118,40,m,2,24,1);//(122,40,m,2,24,1);
				}				
				OLED_ShowChar(142,40,' ',24,1);//(146,40,' ',24,1);
			}
			xSemaphoreGive(xSemaphoreDispRfresh);			
		}			
	    blinkTimeMs++;		   
		if(blinkTimeMs>50)//500ms
		{
			blinkTimeMs=0;
			blink=(blink)?0:1;		
			if(apexValue<0)
			{
				if(blink==0)	OLED_ShowString(106,40,"    ",24,1);//(110,40,"    ",24,1);
				else OLED_ShowString(106,40,"APEX",24,1);//(110,40,"APEX",24,1);
			}
			else
			{
				if(recApexValue<26&&recApexValue>=0&&recApexValue!=sys_param_un.device_param.ref_tine+3)
				{
					if(sys_param_un.device_param.ref_tine<7)
					{
						x=50+sys_param_un.device_param.ref_tine*6;//52+sys_param_un.device_param.ref_tine*6;
						y=16;
						disp_DrawColumn(x+1,y+2,36,blink);
						disp_DrawColumn(x+2,y+1,38,blink);
						disp_DrawColumn(x+3,y,40,blink);
						disp_DrawColumn(x+4,y+1,38,blink);
						disp_DrawColumn(x+5,y+2,36,blink);
					}
					else if(sys_param_un.device_param.ref_tine<11)
					{
//						x=96+(sys_param_un.device_param.ref_tine-7)*4;
						x=68+sys_param_un.device_param.ref_tine*4-2;
						y=16;	
						disp_DrawColumn(x+1,y+1,80-sys_param_un.device_param.ref_tine*6,blink);
						disp_DrawColumn(x+2,y,82-sys_param_un.device_param.ref_tine*6,blink);
						disp_DrawColumn(x+3,y+1,80-sys_param_un.device_param.ref_tine*6,blink);
					}
					else 
					{							
						x=68+sys_param_un.device_param.ref_tine*4-2;
						y=16;		
						disp_DrawColumn(x+1,y+1,17,blink);
						disp_DrawColumn(x+2,y,19,blink);
						disp_DrawColumn(x+3,y+1,17,blink);
					}
				}
				
			}
		}
	}
	}
	
}
static void ApexBarFlashForSet(unsigned int systemTimeMs,unsigned char startFlag, short int apexValue)
{
	static unsigned int recTimeMs,blinkTimeMs;
	static unsigned short recApexValue,temp;
	unsigned char i,j,x,y,m,sendBuff;	
	if(startFlag==0)//preparation work
	{
		recTimeMs=systemTimeMs;
//	recApexValue=33;//<0 ,闂傚倷娴囧▔鏇㈠窗閹版澘姹查柍褜鍓熼弻鐔煎箒閹烘垵濮风紓鍌氱Т濞差參鐛箛娑欐櫢闁跨噦鎷�<3闂傚倷娴囧▔鏇㈠窗閹版澘鍑犲┑鐘宠壘缁狀垶鏌ㄩ悤鍌涘23,4闂傚倷娴囧▔鏇㈠窗閹版澘鍑犲┑鐘宠壘缁狀垶鏌ｉ幋锝呅撻柡鍛倐閺岋繝宕掑Ο琛″亾閺嶎偀鍋撳顒傜Ш闁哄被鍔戦幃銏ゅ川婵犲嫪绱曢梻浣哥秺椤ユ捇宕楀鈧顐﹀箻缂佹ɑ娅㈤梺璺ㄥ櫐閹凤拷
		recApexValue=0;
		temp=0;
	}		
	else
	{	
		if(systemTimeMs<recTimeMs)		 recTimeMs=systemTimeMs;
		if(systemTimeMs-recTimeMs>10)
		{	
			recTimeMs=systemTimeMs;			
			if(apexValue<1)
			{			
//				if(temp!=0)
					recApexValue=33;
					temp=0;
			}
			else temp=apexValue;					
			// over area (40~60)
			if(recApexValue!=temp)
			{
				if(temp>=30) recApexValue=30;//
				if(temp==0) recApexValue=temp;//闂備胶枪缁诲牓宕濋幋锕€鏋侀柟鎹愵嚙濡﹢鏌曢崼婵囶棞妞ゅ骏鎷�
				if(recApexValue>temp) 
				{
					if(recApexValue==1) recApexValue=0;					
					else recApexValue--;
				}
				else if(recApexValue<temp)
				{
					recApexValue++;	
				}
				else 	recApexValue=temp;//fresh			
				x=33-16;//35;
				y=16;	
				m=x;
				for(i=0;i<3;i++)
				{		
					x=m+i*5;	
					disp_DrawColumn(x,y,40,0);
					disp_DrawColumn(x+1,y+1,38,0);
					disp_DrawColumn(x+2,y,40,0);
					disp_DrawColumn(x+3,y,40,0);
					disp_DrawColumn(x+4,y+1,38,0);
				}	
				// apex  area (60~100)
				x=50-16;//52;
				y=16;
				m=x;
				for(i=0;i<7;i++)
				{
					x=m+i*6;
//				disp_DrawColumn(x,y,40,0);
					if(i==sys_param_un.device_param.ref_tine)//闂傚倷娴囧▔鏇㈠窗閺囩喓绠鹃柛銉墮缁€宀勬煃閳轰礁鏆欐い蹇撶埣濮婅櫣鎹勯妸銉︾€鹃梺鍝勵儏椤兘鐛箛娑欐櫢闁跨噦鎷�
					{
						disp_DrawColumn(x+1,y+2,36,1);
						disp_DrawColumn(x+2,y+1,38,1);
						disp_DrawColumn(x+3,y,40,1);
						disp_DrawColumn(x+4,y+1,38,1);
						disp_DrawColumn(x+5,y+2,36,1);						
//						disp_DrawRow(x+3,y+41,1,1);//闂傚倷娴囧▔鏇㈠窗閹版澘鍑犲┑鐘宠壘缁狀垶鏌ｉ幋锝呅撻柡鍛倐閹嘲鈻庨幇顒傜懆闂佹悶鍊栭悧鐘荤嵁韫囨稒鍊婚柤鎭掑劜濞呫垽姊洪崫鍕偓鍫曞磹閺嶎偀鍋撳鐐
//						disp_DrawRow(x+2,y+42,3,1);
//						disp_DrawRow(x+1,y+43,5,1);
					}
					else
					{						
						disp_DrawColumn(x+1,y+2,36,0);
						disp_DrawColumn(x+2,y+1,38,0);
						disp_DrawColumn(x+3,y,40,0);
						disp_DrawColumn(x+4,y+1,38,0);
						disp_DrawColumn(x+5,y+2,36,0);						
					}	
				}
				// oter area  闂傚倷娴囧▔鏇㈠窗閹版澘鍑犲┑鐘宠壘缁狀垶鏌ㄩ悤鍌涘100~160)
				x=94-16;//96;
				y=16;
				m=x;
				for(i=0;i<15;i++)
				{
					x=m+i*4;
	//				disp_DrawColumn(x,y,20,0);
					if(i+7==sys_param_un.device_param.ref_tine)//闂傚倷娴囧▔鏇㈠窗閺囩喓绠鹃柛銉墮缁€宀勬煃閳轰礁鏆欐い蹇撶埣濮婅櫣鎹勯妸銉︾€鹃梺鍝勵儏椤兘鐛箛娑欐櫢闁跨噦鎷�
					{
						if(i<4)
						{
							disp_DrawColumn(x+1,y+1,18+20-i*6,1);
							disp_DrawColumn(x+2,y,20+20-i*6,1);
							disp_DrawColumn(x+3,y+1,18+20-i*6,1);
						}
						else
						{
							disp_DrawColumn(x+1,y+1,17,1);
							disp_DrawColumn(x+2,y,19,1);
							disp_DrawColumn(x+3,y+1,17,1);
						}	
					}
					else 
					{
						if(i<4)
						{
							disp_DrawColumn(x+1,y+1,18+20-i*6,0);
							disp_DrawColumn(x+2,y,20+20-i*6,0);
							disp_DrawColumn(x+3,y+1,18+20-i*6,0);
						}
						else
						{
							disp_DrawColumn(x+1,y+1,17,0);
							disp_DrawColumn(x+2,y,19,0);
							disp_DrawColumn(x+3,y+1,17,0);
						}									
					}			
				}			
				OLED_ShowString(92,40,"APEX",24,1);//(110,40,"APEX",24,1);
//				blinkTimeMs++;
//				if(blinkTimeMs>50)//500
//				{
//					blinkTimeMs=0;
					xSemaphoreGive(xSemaphoreDispRfresh);
//				}
				
			}			
		}
	}
	
}
/**
  * @brief 	disp_ApexPrepare
  * @param   u8 x,u8 y,start coord 
  * @retval none
  */
static void Disp_ApexPreForSet(u8 value)
{
	u8 i,x,y;
//bat area  闂傚倷娴囧▔鏇㈠窗閹版澘鍑犲┑鐘宠壘缁狀垶鏌ㄩ悤鍌涘0~40闂傚倷娴囧▔鏇㈠窗閹版澘鍑犲┑鐘宠壘缁狀垶鏌ㄩ悤鍌涘
	//P0 ,bat
	
	// over area (40~60)
	x=34-16;//36;//40-4;
	y=8;	
	for(i=0;i<2;i++)
	{		
		disp_DrawRow(x,y+i+3,15,1);
	}		
	// apex  area (60~102)
		x=52-16;//54;//60-6;
		y=8;
		for(i=0;i<2;i++)
		{
			OLED_DrawPoint(x, y+i+1,1);
			OLED_DrawPoint(x+1, y+i+1,1);
			disp_DrawRow(x,y+i+3,40,1);
		}			
		OLED_ShowNum(x-2,0,0,1,8,1);
		//闂傚倷娴囧▔鏇㈠窗閹版澘鍑犲┑鐘宠壘缁狀垶鏌ｉ幋锝呅撻柡鍛倐閹嘲鈻庨幇顒傜懆闂佹悶鍊栭悧鐘荤嵁韫囨稒鍊婚柤鎭掑劜濞呫垽姊洪崫鍕偓鍫曞磹閺嶎偀鍋撳鐐
//		sys_param_un.device_param.ref_tine=(sys_param_un.device_param.ref_tine%7);//offset	max=3	
//		disp_DrawRow(x+1+sys_param_un.device_param.ref_tine*6,60,1,1);//闂傚倷娴囧▔鏇㈠窗閹版澘鍑犲┑鐘宠壘缁狀垶鏌ｉ幋锝呅撻柡鍛倐閹嘲鈻庨幇顒傜懆闂佹悶鍊栭悧鐘荤嵁韫囨稒鍊婚柤鎭掑劜濞呫垽姊洪崫鍕偓鍫曞磹閺嶎偀鍋撳鐐
//		disp_DrawRow(x+sys_param_un.device_param.ref_tine*6,61,3,1);
//		disp_DrawRow(x-1+sys_param_un.device_param.ref_tine*6,62,5,1);			
		sys_param_un.device_param.ref_tine=(sys_param_un.device_param.ref_tine%13);//offset	max=3	
//		
//		disp_DrawRow(x-1,60,59,0);//闂傚倷娴囧▔鏇㈠窗閹版澘鍑犲┑鐘宠壘缁狀垶鏌ｉ幋锝呅撻柡鍛倐閹嘲鈻庨幇顒傜懆闂佹悶鍊栭悧鐘荤嵁韫囨稒鍊婚柤鎭掑劜濞呫垽姊洪崫鍕偓鍫曞磹閺嶎偀鍋撳鐐
//		disp_DrawRow(x-1,61,59,0);
//		disp_DrawRow(x-1,62,59,0);	
		
		disp_DrawRow(x+1+3*6,60,1,1);//闂傚倷娴囧▔鏇㈠窗閹版澘鍑犲┑鐘宠壘缁狀垶鏌ｉ幋锝呅撻柡鍛倐閹嘲鈻庨幇顒傜懆闂佹悶鍊栭悧鐘荤嵁韫囨稒鍊婚柤鎭掑劜濞呫垽姊洪崫鍕偓鍫曞磹閺嶎偀鍋撳鐐
		disp_DrawRow(x+3*6,61,3,1);
		disp_DrawRow(x-1+3*6,62,5,1);		
	// other area 闂傚倷娴囧▔鏇㈠窗閹版澘鍑犲┑鐘宠壘缁狀垶鏌ㄩ悤鍌涘100~160闂傚倷娴囧▔鏇㈠窗閹版澘鍑犲┑鐘宠壘缁狀垶鏌ㄩ悤鍌涘
		x=95-16;//97;//100-3;
		y=8;
		for(i=0;i<2;i++)
		{
		  OLED_DrawPoint(x, y+i+1,1);//1
			OLED_DrawPoint(x+1, y+i+1,1);
			
			OLED_DrawPoint(x+25, y+i+1,1);
			OLED_DrawPoint(x+1+25, y+i+1,1);
			
			OLED_DrawPoint(x+53, y+i+1,1);
			OLED_DrawPoint(x+1+53, y+i+1,1);
			
			disp_DrawRow(x,y+i+3,60,1);
		}		
		//num
		OLED_ShowNum(x-2,0,1,1,8,1);
		OLED_ShowNum(x+28-5,0,2,1,8,1);
		OLED_ShowNum(x+56-5,0,3,1,8,1);
}
/**
  * @brief 	disp_ApexPrepare
  * @param   u8 x,u8 y,start coord 
  * @retval none
  */
static void Disp_ApexPrepare(u8 value)
{
	u8 i,x,y;
//bat area  闂傚倷娴囧▔鏇㈠窗閹版澘鍑犲┑鐘宠壘缁狀垶鏌ㄩ悤鍌涘0~40闂傚倷娴囧▔鏇㈠窗閹版澘鍑犲┑鐘宠壘缁狀垶鏌ㄩ悤鍌涘
	//P0 ,bat
	
	// over area (40~60)
	x=34;//36;//40-4;
	y=8;	
	for(i=0;i<2;i++)
	{		
		disp_DrawRow(x,y+i+3,15,1);
	}		
	// apex  area (60~102)
		x=52;//54;//60-6;
		y=8;
		for(i=0;i<2;i++)
		{
			OLED_DrawPoint(x, y+i+1,1);
			OLED_DrawPoint(x+1, y+i+1,1);
			disp_DrawRow(x,y+i+3,40,1);
		}			
		OLED_ShowNum(x-2,0,0,1,8,1);
		//闂傚倷娴囧▔鏇㈠窗閹版澘鍑犲┑鐘宠壘缁狀垶鏌ｉ幋锝呅撻柡鍛倐閹嘲鈻庨幇顒傜懆闂佹悶鍊栭悧鐘荤嵁韫囨稒鍊婚柤鎭掑劜濞呫垽姊洪崫鍕偓鍫曞磹閺嶎偀鍋撳鐐
//		sys_param_un.device_param.ref_tine=(sys_param_un.device_param.ref_tine%7);//offset	max=3	
//		disp_DrawRow(x+1+sys_param_un.device_param.ref_tine*6,60,1,1);//闂傚倷娴囧▔鏇㈠窗閹版澘鍑犲┑鐘宠壘缁狀垶鏌ｉ幋锝呅撻柡鍛倐閹嘲鈻庨幇顒傜懆闂佹悶鍊栭悧鐘荤嵁韫囨稒鍊婚柤鎭掑劜濞呫垽姊洪崫鍕偓鍫曞磹閺嶎偀鍋撳鐐
//		disp_DrawRow(x+sys_param_un.device_param.ref_tine*6,61,3,1);
//		disp_DrawRow(x-1+sys_param_un.device_param.ref_tine*6,62,5,1);			
		sys_param_un.device_param.ref_tine=(sys_param_un.device_param.ref_tine%13);//offset	max=3	
//		
//		disp_DrawRow(x-1,60,59,0);//闂傚倷娴囧▔鏇㈠窗閹版澘鍑犲┑鐘宠壘缁狀垶鏌ｉ幋锝呅撻柡鍛倐閹嘲鈻庨幇顒傜懆闂佹悶鍊栭悧鐘荤嵁韫囨稒鍊婚柤鎭掑劜濞呫垽姊洪崫鍕偓鍫曞磹閺嶎偀鍋撳鐐
//		disp_DrawRow(x-1,61,59,0);
//		disp_DrawRow(x-1,62,59,0);	
		
		disp_DrawRow(x+1+3*6,60,1,1);//闂傚倷娴囧▔鏇㈠窗閹版澘鍑犲┑鐘宠壘缁狀垶鏌ｉ幋锝呅撻柡鍛倐閹嘲鈻庨幇顒傜懆闂佹悶鍊栭悧鐘荤嵁韫囨稒鍊婚柤鎭掑劜濞呫垽姊洪崫鍕偓鍫曞磹閺嶎偀鍋撳鐐
		disp_DrawRow(x+3*6,61,3,1);
		disp_DrawRow(x-1+3*6,62,5,1);		
	// other area 闂傚倷娴囧▔鏇㈠窗閹版澘鍑犲┑鐘宠壘缁狀垶鏌ㄩ悤鍌涘100~160闂傚倷娴囧▔鏇㈠窗閹版澘鍑犲┑鐘宠壘缁狀垶鏌ㄩ悤鍌涘
		x=95;//97;//100-3;
		y=8;
		for(i=0;i<2;i++)
		{
		  OLED_DrawPoint(x, y+i+1,1);//1
			OLED_DrawPoint(x+1, y+i+1,1);
			
			OLED_DrawPoint(x+25, y+i+1,1);
			OLED_DrawPoint(x+1+25, y+i+1,1);
			
			OLED_DrawPoint(x+53, y+i+1,1);
			OLED_DrawPoint(x+1+53, y+i+1,1);
			
			disp_DrawRow(x,y+i+3,60,1);
		}		
		//num
		OLED_ShowNum(x-2,0,1,1,8,1);
		OLED_ShowNum(x+28-5,0,2,1,8,1);
		OLED_ShowNum(x+56-5,0,3,1,8,1);
}

/**
  * @brief 	FreshTorquePictureInApexAndMotorMode
  * @param   u8 x,u8 y,start coord 
  * @retval none
  */
static void FreshTorquePictureInApexAndMotorMode(u8 x,u8 y,unsigned short int torqueValue,unsigned  int systemTimeMs)
{
	static unsigned short int temp;
	static unsigned  int recTimeMs;
	static unsigned  char blinkFlag=1;
	u8 i;
	if(systemTimeMs<recTimeMs) recTimeMs=systemTimeMs;
	if(systemTimeMs-recTimeMs>500)
	{
		recTimeMs=systemTimeMs;	
		blinkFlag =(blinkFlag)?0:1;
		if(motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].dir==EndoModeTorqueATC)
		{
			temp=(torqueValue*6/torque_list[motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].atcTorqueThresholdNum]);
		}
		else
		{
			temp=(torqueValue*6/torque_list[motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].torqueThresholdNum]);
		}			
//		temp=(torqueValue*6/torque_list[motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].torqueThresholdNum]);
		if(temp>6) temp=6;
		for(i=0;i<26;i++)//闂傚倷娴囧▔鏇㈠窗閹版澘鍑犲┑鐘宠壘缁狀垶鏌ｉ幋锝呅撻柡鍛倐閺屻劑鎮ら崒娑橆伓
		{
			disp_DrawRow(x+3,y+3+i,26,0);
		}
		for(i=0;i<2+4*temp;i++)//闂傚倷娴囧▔鏇㈠窗閹版澘鍑犲┑鐘宠壘缁狀垶鏌ｉ幋锝呅撻柡鍛倐閺屻劑鎮ら崒娑橆伓
		{
			if(temp>5)
			{
				disp_DrawRow(x+15-2*temp,y+i+15-2*temp,2+4*temp,blinkFlag);
			}
			else
			{
				disp_DrawRow(x+15-2*temp,y+i+15-2*temp,2+4*temp,1);
			}
		}
		xSemaphoreGive(xSemaphoreDispRfresh);
	}	
}
/**
  * @brief 	Disp_TorquePictureInApexAndMotorMode
  * @param   u8 x,u8 y,torqueValue 
  * @retval none
  */
static void Disp_TorquePictureInApexAndMotorMode(u8 x,u8 y,unsigned short int torqueValue)
{
	unsigned short int temp;
	u8 i;		
	disp_DrawRow(x,y,32,1);
	disp_DrawRow(x,y+31,32,1);
	disp_DrawColumn(x,y,32,1);
	disp_DrawColumn(x+31,y,32,1);
	
	 //90% temp=1;//10%
//	if(motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].dir==EndoModeTorqueATC)
//	{
//		temp=(torqueValue*6/torque_list[motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].atcTorqueThresholdNum]);
//	}
//	else
//	{
//		temp=(torqueValue*6/torque_list[motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].torqueThresholdNum]);
//	}	
//	for(i=0;i<2*temp;i++)
//	{
//		disp_DrawRow(x+3,y+3+i,26,0);
//		disp_DrawRow(x+3,y+2*temp+i,26,0);
//		disp_DrawColumn(x+3+i,y+3,2+4*temp,0);
//		disp_DrawColumn(x+3+2*temp+i,y+3,2+4*temp,0);
//	}
//	for(i=0;i<2+4*temp;i++)
//	{
//		disp_DrawRow(x+15-2*temp,y+i+15-2*temp,2+4*temp,1);
//	}	
}

/**
  * @brief 	disp_MotorAndApexPrepare
  * @param   u8 x,u8 y,start coord 
  * @retval none
  */
static void Disp_MotorAndApexPrepare(unsigned short int value)
{
	uint32_t temp;
//	u8 x,y;
	Disp_TorquePictureInApexAndMotorMode(0,0,0);//torque_list[motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].torqueThresholdNum]);
	if(motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].dir==EndoModePositionToggle)
	{
		temp=speed_list[motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].toggleSpeedNum];
	}
	else
	{
		temp=speed_list[motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].motorSpeedNum];
	}	
	if((temp)<1000)	
	{
		OLED_ShowChar(0,32,' ',16,1);
		OLED_ShowNum(4,32,temp,3,16,1);
		OLED_ShowChar(28,32,' ',16,1);
	}
	else
	{
		OLED_ShowNum(0,32,temp,4,16,1);					
	}
	OLED_ShowString(4,48,"rpm",16,1);	
	Disp_ApexPrepare(5);
	ApexProgressBarFlash(0,0,0);		
}
/**
  * @brief 	POWER_OFF
  * @param  num
  * @retval none
  */
void OLED_Display_POWER_OFF(unsigned char num)// power off flash
{		
	if(num==0)//low power 
	{
		OLED_ShowString(26,24,"LOW POWER",24,1);	
	}
	else //  power
	{
		OLED_ShowString(26,24,"POWER OFF",24,1);	
	}
}
/**
  * @brief 	work menu
  * @param  workingMode
  * @retval none
  */
void OLED_Display_WORK(unsigned char workingMode,unsigned char startFlag)//pageID (MENU_MOTOR_WORK_PAGE )only motor  (MENU_ONLY_APEX_PAGE) only apex (MENU_APEX_AND_MOTOR_PAGE) motor and apex 
{		
	unsigned short int temp;	
	if(workingMode==MENU_MOTOR_WORK_PAGE)
	{
		OLED_ShowString(0,0,"TQ",24,1);		
		if(motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].dir==EndoModePositionToggle)
		{
			temp=speed_list[motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].toggleSpeedNum];
		}
		else temp=speed_list[motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].motorSpeedNum];
		if((temp)<1000)	
		{
			OLED_ShowChar(24,32,' ',16,1);
			OLED_ShowNum(0,32,temp,3,16,1);
		}
		else
		{
			OLED_ShowNum(0,32,temp,4,16,1);					
		}
		OLED_ShowString(0,48,"rpm",16,1);	
		//picture
		Disp_TorqueProgressBar(36,0,0);
		if(motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].dir==EndoModeTorqueATC)
		{
			TorqueProgressBarFlash(0,0,torque_list[motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].atcTorqueThresholdNum]);
		}
		else if(motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].dir==EndoModePositionToggle)
		{
			TorqueProgressBarFlash(0,0,torque_list[motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].recTorqueThresholdNum]);
		}
		else 	TorqueProgressBarFlash(0,0,torque_list[motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].torqueThresholdNum]);
		//target torque
		if(motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].dir==EndoModePositionToggle)
		{
			Disp_TorqueOnly(120,40,torque_list[MAX_torque_42_Ncm]);
		}
		else if(motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].dir==EndoModeTorqueATC)
		{
			Disp_TorqueOnly(120,40,torque_list[motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].atcTorqueThresholdNum]);
		}
		else
		{
			Disp_TorqueOnly(120,40,torque_list[motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].torqueThresholdNum]);
		}	
	}	
	else if(workingMode==MENU_ONLY_APEX_PAGE)
	{	
		OLED_ShowString(6,4,"P0",16,1);
		Disp_ApexPrepare(60);		
		ApexProgressBarFlash(0,0,0);	
	}	
	else if(workingMode==MENU_APEX_AND_MOTOR_PAGE)
	{				
		Disp_MotorAndApexPrepare(60);
	}		
}
/**
  * @brief 	GC_ControlMotor
  * @param  systemTime
  * @retval none
  */
static void  GC_ControlMotor(int depth,unsigned int systemTimeMs)
{	
	static 	unsigned char auto_flag_ap=null_signal;
	static  unsigned int recTimeMs;
	static unsigned char temp_mode_flag=0, motor_run_mode=0;
	unsigned char sendSignal=null_signal;
	if(recTimeMs>systemTimeMs) recTimeMs=systemTimeMs;
	if(sys_param_un.device_param.apexFunctionLoad==1)
	{
		if(sys_param_un.device_param.use_p_num<10)//
		{
			#ifdef  APEX_FUNCTION_EBABLE	// 
			if(depth<27)
		 	{			 
				if(depth<0)
				{	
					if(motor_status.status!=Status_STOP)//auto start ,in
					{	
						if(sys_param_un.device_param.apical_action_flag!=0)		
							App_MotorControl(MOTOR_MODE_STOP);
						}		
					}						
					auto_flag_ap = motor_apex_run_signal;//		
				}
				else if(depth<=3+sys_param_un.device_param.ref_tine)
				{	//
					if(sys_param_un.device_param.apical_action_flag==1)//
					{
						if(temp_mode_flag==0)
						{	
							temp_mode_flag=0x55;	
							motor_run_mode=	motor_settings.mode;	
							App_MotorControl(MOTOR_MODE_STOP);
							if(motor_run_mode==EndoModePositionToggle)
							{
								if(motor_settings.forward_position > - motor_settings.reverse_position)
								{
									motor_settings.mode=EndoModeSpeedReverse;
								}
								else if(motor_settings.forward_position < - motor_settings.reverse_position)
								{
									motor_settings.mode=EndoModeSpeedForward;
								}
							}
							else  if(motor_run_mode==EndoModeSpeedForward)
							{
								motor_settings.mode=EndoModeSpeedReverse;
							}
							else  if(motor_run_mode==EndoModeSpeedReverse)
							{
								motor_settings.mode=EndoModeSpeedForward;
							}
							App_MotorControl(MOTOR_SETTING_UPDATE);									
							if(motor_status.status==Status_STOP)
							{	
								if(sys_param_un.device_param.auto_start_flag!=0&&auto_flag_ap != motor_apex_run_signal)
								{
									auto_flag_ap= motor_apex_run_signal;	
								}									
								App_MotorControl(MOTOR_MODE_START);
							}		
						}	
					}
					else if(sys_param_un.device_param.apical_action_flag==2)
					{	
						if(temp_mode_flag==0)
						{
							temp_mode_flag=0x55;
							motor_run_mode =	motor_settings.mode;
							if(motor_status.status!=Status_STOP)
							{													
								App_MotorControl(MOTOR_MODE_STOP);
							}			
						}	            			 
					}
					if(sys_param_un.device_param.apical_action_flag!=0||sys_param_un.device_param.auto_start_flag!=0||sys_param_un.device_param.auto_stop_flag!=0)//闂傚倷娴囧▔鏇㈠窗鎼淬們浜归柕濞炬櫅缁€宀勬煃閳轰礁鏆欐い蹇撶埣濮婅櫣鎹勯妸銉︾亖闁诲孩纰嶉惄顖炲极閹邦垳鐤€闁哄倹顑欐导鎾绘⒒娴ｈ姤纭堕柛鐘冲姍瀵憡绻濆顒傤唵闂佺粯鍨兼慨銈夊疾閹间焦鐓涢柛灞久埀顒佺墱閳ь剚鐔幏锟�
					{
						if(motor_status.status!=Status_STOP)
						{
							if(motor_settings.mode==EndoModePositionToggle)
							{
								if(motor_settings.toggle_mode_speed>speed_list[spd200_Rpm_num])
								{
									motor_settings.toggle_mode_speed=speed_list[spd200_Rpm_num];
									App_MotorControl(MOTOR_SETTING_UPDATE);
								}						
							}
							else 
							{
								if(motor_settings.forward_speed>speed_list[spd200_Rpm_num])
								{							
									motor_settings.forward_speed=speed_list[spd200_Rpm_num];
									motor_settings.reverse_speed=-speed_list[spd200_Rpm_num];									 
									App_MotorControl(MOTOR_SETTING_UPDATE);							
								}
							}							
						}	
					}
			 }
				else if(depth<=(7+sys_param_un.device_param.ref_tine))//0.67//250rpm
				{	
					if(motor_status.status!=Status_STOP)
					{
						if(sys_param_un.device_param.apical_action_flag!=0||sys_param_un.device_param.auto_start_flag!=0||sys_param_un.device_param.auto_stop_flag!=0)
						{
							if(motor_settings.mode==EndoModePositionToggle)
							{
								if(motor_settings.toggle_mode_speed>speed_list[spd250_Rpm_num])
								{
									motor_settings.toggle_mode_speed=speed_list[spd250_Rpm_num];
									App_MotorControl(MOTOR_SETTING_UPDATE);
								}						
							}
							else 
							{
								if(motor_settings.forward_speed>speed_list[spd250_Rpm_num])
								{							
									motor_settings.forward_speed=speed_list[spd250_Rpm_num];
									motor_settings.reverse_speed=-speed_list[spd250_Rpm_num];									 
									App_MotorControl(MOTOR_SETTING_UPDATE);							
								}
							}
						}
					}	
				}
				else if(depth<=(8+sys_param_un.device_param.ref_tine))//300rpm
			 	{	
					if(motor_status.status!=Status_STOP)
					{
						if(sys_param_un.device_param.apical_action_flag!=0||sys_param_un.device_param.auto_start_flag!=0||sys_param_un.device_param.auto_stop_flag!=0)
						{
							if(motor_settings.mode==EndoModePositionToggle)
							{
								if(motor_settings.toggle_mode_speed>speed_list[spd300_Rpm_num])
								{
									motor_settings.toggle_mode_speed=speed_list[spd300_Rpm_num];
									App_MotorControl(MOTOR_SETTING_UPDATE);
								}						
							}
							else 
							{
								if(motor_settings.forward_speed>speed_list[spd300_Rpm_num])
								{							
									motor_settings.forward_speed=speed_list[spd300_Rpm_num];
									motor_settings.reverse_speed=-speed_list[spd300_Rpm_num];									 
									App_MotorControl(MOTOR_SETTING_UPDATE);							
								}
							}
						}
					}	
				}
			    else if(depth<=(9+sys_param_un.device_param.ref_tine))
			    {		
					if(temp_mode_flag==0x55) { motor_settings.mode=motor_run_mode;		temp_mode_flag=0; App_MotorControl(MOTOR_SETTING_UPDATE);	}		//闂傚倷娴囧▔鏇㈠窗閺団偓鈧礁饪伴崼鐕佹闂佸啿鎼崐鎼侇敂閿燂拷				 
					if(motor_status.status!=Status_STOP)
					{
						if(sys_param_un.device_param.apical_action_flag!=0||sys_param_un.device_param.auto_start_flag!=0||sys_param_un.device_param.auto_stop_flag!=0)
						{
							if(motor_settings.mode==EndoModePositionToggle)
							{
								if(motor_settings.toggle_mode_speed>speed_list[spd350_Rpm_num])
								{
									motor_settings.toggle_mode_speed=speed_list[spd350_Rpm_num];
									App_MotorControl(MOTOR_SETTING_UPDATE);
								}						
							}
							else 
							{
								if(motor_settings.forward_speed>speed_list[spd350_Rpm_num])
								{							
									motor_settings.forward_speed=speed_list[spd350_Rpm_num];
									motor_settings.reverse_speed=-speed_list[spd350_Rpm_num];									 
									App_MotorControl(MOTOR_SETTING_UPDATE);							
								}
							}
						}
					}	
				}
				else
				{
					if(temp_mode_flag==0x55)
					{
						motor_settings.mode=motor_run_mode ;	
						motor_settings.toggle_mode_speed=speed_list[motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].toggleSpeedNum];
						motor_settings.forward_speed=speed_list[motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].motorSpeedNum];
						motor_settings.reverse_speed=-motor_settings.forward_speed;
						temp_mode_flag=0; 
						App_MotorControl(MOTOR_SETTING_UPDATE);
					}				
					if(motor_status.status!=Status_STOP)
					{
						if(sys_param_un.device_param.apical_action_flag!=0||sys_param_un.device_param.auto_start_flag!=0||sys_param_un.device_param.auto_stop_flag!=0)
						{
							if(motor_settings.mode==EndoModePositionToggle)
							{
								if(motor_settings.toggle_mode_speed!=speed_list[motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].toggleSpeedNum])
								{
									motor_settings.toggle_mode_speed=speed_list[motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].toggleSpeedNum];
									App_MotorControl(MOTOR_SETTING_UPDATE);
								}	
							}
							else 
							{
								if(motor_settings.toggle_mode_speed!=speed_list[motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].motorSpeedNum])
								{							
									motor_settings.forward_speed=speed_list[motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].motorSpeedNum];
									motor_settings.reverse_speed=-speed_list[motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].motorSpeedNum];									 
									App_MotorControl(MOTOR_SETTING_UPDATE);							
								}
							}
						}
					}	
				}				
				if(sys_param_un.device_param.auto_start_flag!=0)//auto start ,in 
					if(auto_flag_ap!=motor_apex_run_signal)
					{
						auto_flag_ap= motor_apex_run_signal;										
						if(depth>=0)
						{
							if(motor_status.status==Status_STOP)
							{
								App_MotorControl(MOTOR_MODE_START);
							}											
						}
					}
					else
					{
						if(motor_status.status==Status_STOP)
						{	
							if(depth>(8+sys_param_un.device_param.ref_tine))
							{
//							if(motor_status.status==Status_STOP)
//							{
//								App_MotorControl(MOTOR_MODE_START);
//							}
							}	
						}		
					}
					
				}						
			}	
			else 
			{
				if(temp_mode_flag==0x55) 
				{
					motor_settings.mode=motor_run_mode;		
					motor_settings.forward_speed=speed_list[motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].motorSpeedNum];
					motor_settings.reverse_speed=-speed_list[motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].motorSpeedNum];
					motor_settings.toggle_mode_speed=speed_list[motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].toggleSpeedNum];
					temp_mode_flag=0;	
					App_MotorControl(MOTOR_SETTING_UPDATE);
				}	 //闂傚倷娴囧▔鏇㈠窗閺団偓鈧礁饪伴崼鐕佹闂佸啿鎼崐鎼侇敂閿燂拷
				if(motor_settings.forward_speed!=speed_list[motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].motorSpeedNum])
				{
					motor_settings.forward_speed=speed_list[motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].motorSpeedNum];
					motor_settings.reverse_speed=-speed_list[motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].motorSpeedNum];
					App_MotorControl(MOTOR_SETTING_UPDATE);	
				} //闂傚倷娴囧▔鏇㈠窗閹版澘鍑犲┑鐘宠壘缁狀垳鈧懓瀚伴崑濠傤焽濡ゅ懏鈷戦悹鎭掑妼閺嬫垿鏌＄€ｎ亶鐓兼鐐茬箻閹粓鎳為妷锔筋仧闂備礁鎼崐鍫曞磹閺嶎偀鍋撳顒傜Ш闁哄被鍔戦幃銏ゅ川婵犲嫪绱曢梻浣烘嚀閸ゆ牠骞忛敓锟�
				if(motor_settings.toggle_mode_speed!=speed_list[motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].toggleSpeedNum])
				{
					motor_settings.toggle_mode_speed=speed_list[motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].toggleSpeedNum];
					App_MotorControl(MOTOR_SETTING_UPDATE);	
				}
				if(auto_flag_ap!=motor_apex_stop_signal)
				{
					if(sys_param_un.device_param.auto_stop_flag!=0)//exit,
					{
						if(motor_status.status!=Status_STOP)
						{
							 App_MotorControl(MOTOR_MODE_STOP);
						}	
						sendSignal=motor_apex_stop_signal;
						xQueueSend(xQueueKeyMessage, &sendSignal, 0);//闂備礁鎲＄敮鐐哄礂濮椻偓瀵偊骞樼拠鍙夘棟闂侀潧鐗嗗Λ妤咁敂閿燂拷						
					}						
					auto_flag_ap= motor_apex_stop_signal;//闂傚倷娴囧▔鏇㈠窗閺囩喍绻嗘い鎾跺У鐎氭氨鎲告惔锝傚亾濮橆剛绉洪柡灞诲姂閹垽宕ㄦ繝鍕磿闂備礁缍婇ˉ鎾诲礂濮椻偓瀵偊骞樼拠鍙夘棟闂侀潧鐗嗗Λ妤咁敂閿燂拷						
				}
				if(motor_status.status==Status_STOP&&sys_param_un.device_param.apical_action_flag!=0) 
				{
					App_MotorControl(MOTOR_MODE_START);
				}//restart
			}
			if(sys_param_un.device_param.auto_start_flag==0&&sys_param_un.device_param.auto_stop_flag==0&&sys_param_un.device_param.apical_action_flag==0)//闂傚倷娴囧▔鏇㈠窗閹炬剚鐒藉Δ锝呭暞椤ュ牓鏌嶉崫鍕偓鎼侇敂閸洘鈷戦悹鎭掑妼閺嬫垿鏌＄€ｎ亶鐓兼鐐茬箻閹粓鎳為妷锔筋仧闂備礁鎼崐鍫曞磹閺嶎偀鍋撳顒傜Ш闁哄被鍔戦幃銈夊磼濞戞﹩浼�
			{	//闂備胶绮灙闁糕晜鐗犻崺鈧い鎴ｆ硶缁愭棃鏌℃担瑙勫磳鐎殿噮鍓熸俊鍫曞幢濡ゅ﹣绱﹂梻鍌欐祰濞夋洟宕伴幇鏉垮嚑濠电姵鑹剧粻顖炴煥閻曞倹瀚�
				if(motor_status.status!=Status_STOP&&auto_flag_ap!=motor_apex_run_signal)	auto_flag_ap=motor_apex_run_signal;
				if(motor_status.status==Status_STOP&&auto_flag_ap!=motor_apex_stop_signal)	auto_flag_ap=motor_apex_stop_signal;
			}				
			#else        
			//only motor from usart
			#endif		
		}
		else
		{
			if(motor_status.status!=Status_STOP&&sys_param_un.device_param.use_p_num==10)
			{													
				App_MotorControl(MOTOR_MODE_STOP);
			}	
		}	
    	if(depth<=0)  //
		{
			if(systemTimeMs-recTimeMs>100)
			{
				recTimeMs=systemTimeMs;
				sendSignal=BUZZER_MODE_GC_CONNECT_TEST;
				xQueueSend(xQueueBeepMode, &sendSignal, 0);
			}	
		}
		else if(depth<(3+sys_param_un.device_param.ref_tine))
		{
			if(systemTimeMs-recTimeMs>150)
			{
				recTimeMs=systemTimeMs;
				sendSignal=BUZZER_MODE_GC_OVER;
				xQueueSend(xQueueBeepMode, &sendSignal, 0);
			 }
		}		
		else if(depth==(3+sys_param_un.device_param.ref_tine))
		{
			if(systemTimeMs-recTimeMs>400)
			{
				recTimeMs=systemTimeMs;
				sendSignal=BUZZER_MODE_GC_ZREO_APEX;
				xQueueSend(xQueueBeepMode, &sendSignal, 0);
			 }
		}		
		else if(depth<(16+sys_param_un.device_param.ref_tine))
		{
			 if(systemTimeMs-recTimeMs>800)
			{
				recTimeMs=systemTimeMs;
				 sendSignal=BUZZER_MODE_GC_APEX;
				xQueueSend(xQueueBeepMode, &sendSignal, 0);
			 }
		}		
     	if(depth==30)
		{	
			if(auto_flag_ap!=motor_apex_stop_signal)
			{
				auto_flag_ap= motor_apex_stop_signal;//闂傚倷娴囧▔鏇㈠窗閺囩喍绻嗘い鎾跺У鐎氭氨鎲告惔锝傚亾濮橆剛绉洪柡灞诲姂閹垽宕ㄦ繝鍕磿闂備礁缍婇ˉ鎾诲礂濮椻偓瀵偊骞樼拠鍙夘棟闂侀潧鐗嗗Λ妤咁敂閿燂拷
				if(sys_param_un.device_param.auto_stop_flag!=0||sys_param_un.device_param.auto_start_flag!=0||sys_param_un.device_param.apical_action_flag!=0)//exit,闂傚倷娴囧▔鏇㈠窗鎼淬垻鐝舵繛鍡樺灩妞规娊鎮峰▎蹇擃仼妞ゅ繐鐖煎铏规崉閵娿儲鐎鹃梺鍝勵儏椤兘鐛箛娑欏€婚柤鎭掑劜濞呫垽姊洪崫鍕偓鍫曞磹閺嶎偀鍋撳顒傜Ш闁哄被鍔戦幃銏ゅ川婵犲嫪绱曢梻浣烘嚀閸㈡煡宕查崣澶岊浄闁绘ê妯婂浼存煥閻曞倹瀚�,闂傚倷娴囧▔鏇㈠窗閹版澘鍑犲┑鐘宠壘缁狀垶鏌ｉ幋锝呅撻柡鍛倐閺岋繝宕掑Ο琛″亾閺嶎偀鍋撳顒傜Ш闁哄被鍔戦幃銏ゅ川婵犲嫪绱曢梻浣哥秺椤ユ捇宕楀鈧顐﹀箻鐠囧弶顥濋梺闈涚墕濡顢旈崼鏇熲拺閻犳亽鍔岄弸鎴︽煛鐎ｎ亶鐓兼鐐茬箻閹粓鎳為妷锔筋仧闂備礁鎼崐鍫曞磹閺嶎偀鍋撳顒傜Ш闁哄被鍔戦幃銏ゅ川婵犲嫪绱曢梻浣哥秺椤ユ捇宕楀鈧顐﹀箻鐠囪尙鐓戝銈嗗姦閸撴氨鈧熬鎷�
				{
					xQueueSend(xQueueKeyMessage, &auto_flag_ap, 0);//濠碉紕鍋戦崐妤呭极婵犳艾鏋侀柟鎹愵嚙濡﹢鏌曢崼婵囶棞妞ゅ繐鐖煎铏规崉閵娿儲鐎婚悷婊嗗焽閸斿秶鈧數鍘ч埢搴ㄥ箛椤掑绱�
					App_MotorControl(MOTOR_MODE_STOP);
				}					
			}
		}				
	 }
	else//闂傚倷娴囧▔鏇㈠窗閺囩喓绠鹃柛銉㈡櫇妞规娊鎮峰▎蹇擃仼妞ゅ骏鎷�
	{		
		if(auto_flag_ap!=motor_apex_stop_signal)
		{
			auto_flag_ap= motor_apex_stop_signal;//闂備胶绮灙闁糕晜鐗犻崺鈧い鎴ｆ硶缁愭棃鏌℃担瑙勫磳鐎殿噮鍓熸俊鍫曞幢濡ゅ﹣绱﹂梻鍌欐祰濞夋洟宕伴幇鏉垮嚑濠电姵鑹剧粻顖炴煥閻曞倹瀚�				
		}	
	}	
}
/**
  * @brief 	MotorRunModeAndTorque
  * @param  
  * @retval none
  */
static void MotorRunModeAndTorque(unsigned short int realTorque, unsigned int systemTimeMs)
{
	unsigned short int sendBuff;
	static unsigned short int overLoadCount;
	static unsigned short int overdCount,beep_time_out;
	if(realTorque>MAX_TORQUE_UUPER_THRESHOLD*1.05)//闂傚倷娴囧▔鏇㈠窗閹版澘鍑犲┑鐘宠壘缁狀垶鏌ｉ幋锝呅撻柡鍛倐閺岋繝宕掑Ο琛″亾閺嶎偀鍋撳鐐10%(1.10==5.0N,1.08<=4.8)
	{
		overLoadCount++;
		if(overLoadCount>20)//10)//10*10=100ms
		{
			sendBuff=run_button_press_signal;
			App_MotorControl(MOTOR_MODE_STOP);
			MenuMotorParamUpdate(sys_param_un.device_param.use_p_num);//save data	
			xQueueSend(xQueueKeyMessage, &sendBuff, 0);
			sendBuff= BUZZER_MODE_MUTE;		
			xQueueSend(xQueueBeepMode, &sendBuff, 0);	
			beep_time_out=0;			
		}		
	}
	else
	{
		overLoadCount=0;	
		if(motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].dir==EndoModeTorqueATC)
		{
			if(sys_param_un.device_param.apical_action_flag!=0&& GC_depth_vlaue(0, 1)<=(9+sys_param_un.device_param.ref_tine)&&sys_param_un.device_param.apexFunctionLoad!=0)				
			{
				;								
			}	
			else
			{
				if(realTorque>torque_list[torque40_Ncm])
				{
					if(motor_settings.mode !=EndoModeKeepForward&&motor_settings.mode!=EndoModeKeepReverse)
					{
						overdCount++;
						if(overdCount>35)//10*30=300ms;0.3S,change for test
						{												
							overdCount=0;
							if(motor_settings.forward_position>-motor_settings.reverse_position)	
							{//
								motor_settings.mode=EndoModeKeepReverse;
							}
							else 
							{
								motor_settings.mode=EndoModeKeepForward;
							}
							motor_settings.forward_speed=speed_list[motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].toggleSpeedNum];
							motor_settings.reverse_speed=-speed_list[motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].toggleSpeedNum];
							App_MotorControl(MOTOR_MODE_RESTART);
						}
					}					
				}
				else if(realTorque>=torque_list[motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].atcTorqueThresholdNum])
				{
					if(motor_settings.mode !=EndoModePositionToggle)
					{	
						overdCount++;
						if(overdCount>40)//10*30=300ms;0.3S,change for test
						{												
							overdCount=0;
							motor_settings.mode=EndoModePositionToggle;
							motor_settings.forward_speed=speed_list[motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].motorSpeedNum];
							motor_settings.reverse_speed=-speed_list[motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].motorSpeedNum];
							motor_settings.toggle_mode_speed=speed_list[motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].toggleSpeedNum];								
							motor_settings.upper_threshold=torque_list[torque40_Ncm]*0.10;
							motor_settings.lower_threshold=motor_settings.upper_threshold*0.6;//torque_list[motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].atcTorqueThresholdNum]*0.10;
							motor_settings.forward_position=motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].forwardPosition;
							motor_settings.reverse_position=motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].reversePosition;
//							App_MotorControl(MOTOR_SETTING_UPDATE);
							App_MotorControl(MOTOR_MODE_RESTART);
						}							
					}	
				}	
				else //(realTorque<torque_list[motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].atcTorqueThresholdNum])
				{	
					if(motor_settings.mode !=EndoModeSpeedForward)
					{			
						overdCount++;	
						if(overdCount>40)	//10*60=600ms;0.6S,change for test
						{
							overdCount=0;								
							motor_settings.mode=EndoModeSpeedForward;	
							motor_settings.forward_speed=speed_list[motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].motorSpeedNum];
							motor_settings.reverse_speed=-speed_list[motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].motorSpeedNum];
							App_MotorControl(MOTOR_MODE_RESTART);
						}
					}						
				}	 //beep				
				if(realTorque>torque_list[torque40_Ncm]*0.80)//
				{
					if(realTorque>torque_list[torque40_Ncm])
					{
						if(beep_time_out>10)//0.2S
						{
							beep_time_out=0;
						}	
					}
					else if(realTorque>torque_list[torque40_Ncm]*0.90)
					{
						beep_time_out++;
						if(beep_time_out>10)//0.4S
						{
							beep_time_out=0;
							sendBuff= BUZZER_MODE_MOTOR_REVERSE;		
							xQueueSend(xQueueBeepMode, &sendBuff, 0);
						}	
					}			
					else 
					{		
						beep_time_out++;								
						if(beep_time_out>20)//0.5s
						{
							beep_time_out=0;
							sendBuff= BUZZER_MODE_MOTOR_REVERSE_LONG;		
							xQueueSend(xQueueBeepMode, &sendBuff, 0);
						}	
					}		
				}
			}	
		}				 
		else if(motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].dir==EndoModePositionToggle)
		{			
			if(sys_param_un.device_param.apical_action_flag!=0&& GC_depth_vlaue(0, 1)<=(9+sys_param_un.device_param.ref_tine)&&sys_param_un.device_param.apexFunctionLoad!=0)				
			{
				;								
			}		 							
			else
			{	
				if(realTorque>torque_list[motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].recTorqueThresholdNum])//MAX_TORQUE_UUPER_THRESHOLD*0.95)//闂傚倷娴囧▔鏇㈠窗閹版澘鍑犲┑鐘宠壘缁狀垳鈧懓瀚伴。锔捐姳閹惰姤鍊堕煫鍥ㄦ尵缁犲鏌℃担瑙勫鞍闁逛究鍔戦獮鍥礈娴ｄ警妲烽梻鍌欐祰濞夋洟宕伴幇鏉垮嚑濠电姵鑹剧粻顖炴煥閻曞倹瀚�4.0濠电偞鍨堕幐鎼佸箹椤愶箑闂柨鐕傛嫹
				{		
					if(motor_settings.mode ==EndoModePositionToggle)
					{		
						overdCount++;	
						if(overdCount>35)	//10*60=600ms;0.6S,change for test
						{
							overdCount=0;	
							if(motor_settings.forward_position>-motor_settings.reverse_position)	
							{//
								motor_settings.mode=EndoModeKeepReverse;
							}
							else 
							{
								motor_settings.mode=EndoModeKeepForward;
							}
							motor_settings.forward_speed=speed_list[motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].toggleSpeedNum];
							motor_settings.reverse_speed=-speed_list[motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].toggleSpeedNum];
							App_MotorControl(MOTOR_MODE_RESTART);
						}		
					}				
				}	
				else if(realTorque<torque_list[motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].recTorqueThresholdNum]*0.6)//MAX_TORQUE_UUPER_THRESHOLD*0.6)//torque_list[motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].atcTorqueThresholdNum])
				{						
					if(motor_settings.mode !=EndoModePositionToggle)
					{
						overdCount++;
						if(overdCount>40)//10*30=300ms;0.3S,change for test
						{												
							overdCount=0;							
							motor_settings.mode=EndoModePositionToggle;
							motor_settings.forward_speed=speed_list[motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].motorSpeedNum];
							motor_settings.reverse_speed=-speed_list[motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].motorSpeedNum];
							motor_settings.toggle_mode_speed=speed_list[motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].toggleSpeedNum];
							motor_settings.upper_threshold=torque_list[motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].recTorqueThresholdNum]*0.10;//MAX_TORQUE_UUPER_THRESHOLD*0.10;//torque_list[torque40_Ncm]*0.10;
							motor_settings.lower_threshold=motor_settings.upper_threshold*0.6;//torque_list[motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].atcTorqueThresholdNum]*0.10;
							motor_settings.forward_position=motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].forwardPosition;
							motor_settings.reverse_position=motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].reversePosition;
							App_MotorControl(MOTOR_MODE_RESTART);	
						}								
					}							 
				} //beep
				if(realTorque>torque_list[motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].recTorqueThresholdNum]*0.8)//MAX_TORQUE_UUPER_THRESHOLD*0.80)//濠殿喗甯楃粙鎺楁晝閵忋倕鏋侀柟鎹愵嚙濡﹢鏌曢崼婵囶棞妞ゅ繐鐖奸弻娑欑節韫囨柨顏�
				{
					if(realTorque>torque_list[motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].recTorqueThresholdNum])
					{
						if(beep_time_out>10)//0.2S
						{
							beep_time_out=0;
						}	
					}
					else if(realTorque>torque_list[motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].recTorqueThresholdNum]*0.90)
					{
						beep_time_out++;
						if(beep_time_out>10)//0.4S
						{
							beep_time_out=0;
							sendBuff= BUZZER_MODE_MOTOR_REVERSE;		
							xQueueSend(xQueueBeepMode, &sendBuff, 0);
						}	
					}			
					else 
					{		
						beep_time_out++;							
						if(beep_time_out>20)//0.5s
						{
							beep_time_out=0;
							sendBuff= BUZZER_MODE_MOTOR_REVERSE_LONG;		
							xQueueSend(xQueueBeepMode, &sendBuff, 0);
						}	
					}	
				}
			}		  
		}			 	
		else 
		{
		 	 if(realTorque>torque_list[motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].torqueThresholdNum]*0.80)
			{	
				if(realTorque>torque_list[motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].torqueThresholdNum])
				{
					if(beep_time_out>10)//0.2S
					{
						beep_time_out=0;
					}	
				}
				else if(realTorque>torque_list[motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].torqueThresholdNum]*0.90)
				{
					beep_time_out++;
					if(beep_time_out>10)//0.4S
					{
						beep_time_out=0;
						sendBuff= BUZZER_MODE_MOTOR_REVERSE;		
						xQueueSend(xQueueBeepMode, &sendBuff, 0);
					}	
				}
				else 
				{
					beep_time_out++;
					if(beep_time_out>20)//0.5s
					{
						beep_time_out=0;
						sendBuff= BUZZER_MODE_MOTOR_REVERSE_LONG;		
						xQueueSend(xQueueBeepMode, &sendBuff, 0);
					}	
				}					
			}
		}
	}			
}

/**
  * @brief 	WorkPageFlash
  * @param  
  * @retval none
  */
void WorkPageFlash(unsigned char workingMode, unsigned int systemTimeMs)//pageID (MENU_MOTOR_WORK_PAGE )only motor  (MENU_ONLY_APEX_PAGE) only apex (MENU_APEX_AND_MOTOR_PAGE) motor and apex  
{	
	unsigned short int realTorque;
	unsigned short int sendBuff;
	short int realApex;
	static unsigned int recTimeMs,timeout;
	realApex=GC_depth_vlaue(0, 1);//read	
	if(workingMode==MENU_MOTOR_WORK_PAGE)
	{		
		if(systemTimeMs-recTimeMs>10)
		{
			recTimeMs=systemTimeMs;			
			realTorque=GetRealTorque();	
			MotorRunModeAndTorque(realTorque, systemTimeMs);
			if(motor_status.status!=Status_STOP)	
			{	
				TorqueProgressBarFlash(systemTimeMs,1, realTorque);	
			}
		}
	}		
	else if(workingMode==MENU_ONLY_APEX_PAGE)
	{ 
		if(realApex==30)//no action
		{
			if(DeviceOffTimeManage(xTaskGetTickCount(),MINUTE_IDLE_TICKS*3))//8))//8  minute no fresh
			{
				sendBuff=run_button_long_press_signal;//
				xQueueSend(xQueueKeyMessage, &sendBuff, 0);		
			}
		}
		else
		{
			DeviceOffTimeManage(xTaskGetTickCount(),0);
		}
		GC_ControlMotor(realApex, systemTimeMs);
		timeout+=10;
		if(timeout>1000)//1s
		{
			timeout=0;
			#ifdef DEBUG_RTT
//			SEGGER_RTT_printf(0, "realApex=%d\r\n", realApex);	
			#endif
		}
		ApexProgressBarFlash(systemTimeMs,1, realApex);
	}			
	else if(workingMode==MENU_APEX_AND_MOTOR_PAGE)
	{
		if(systemTimeMs>10+recTimeMs)
		{				
			recTimeMs=systemTimeMs;
			realTorque=GetRealTorque();
			GC_ControlMotor(realApex, systemTimeMs);
			MotorRunModeAndTorque(realTorque, systemTimeMs);			
			FreshTorquePictureInApexAndMotorMode(0,0,realTorque,systemTimeMs);			
			if(realApex==30)//no action
			{
				timeout+=10;
				if(timeout>MINUTE_IDLE_TICKS)//*3) 	//3
				{
					timeout=0;
					sendBuff=run_button_press_signal;
					xQueueSend(xQueueKeyMessage, &sendBuff, 0);	
				}					
			}
			else
			{
				timeout=0;
			}	
			ApexProgressBarFlash(systemTimeMs,1, realApex);		
		}
	}	
} 
//=================CHARGING=====================================
/**
  * @brief 	Bat_ValueBlink
  * @param  systemTimeMs, startFlag 
  * @retval none
  */
static void Bat_ValueBlink(unsigned short int batValue,unsigned char blinkFlag)
{
	static unsigned short int history_batValue=0;	
	if(blinkFlag==0)
	{
		charg_bgm(40,2,1,batValue);
	}
	else 
	{	
		if(batValue<3000)
		{			
			 history_batValue=3000;						
		}
		else if(batValue<3450)
		{
			history_batValue=3000;		
		}
		else if(batValue<3650)
		{
			history_batValue=3400;			
		}
		else if(batValue<3950)
		{
			history_batValue=3600;	
		}
		else 
		{
//			if(batValue<4200) history_batValue=3600;
//			else history_batValue=batValue;	
			history_batValue=3900;			
		}		
		charg_bgm(40,2,1,history_batValue);
	}	
}
/**
  * @brief 	Display_CHARGING
  * @param  systemTimeMs, startFlag 
  * @retval none
  */
static void OLED_Display_CHARGING(unsigned short int batValue ,unsigned char startFlag)
{	
	static unsigned char blinkFlag;
	if(startFlag==0)
	{	
		charg_bgm(40,2,0,batValue);
	}
	else 
	{		
		if(get_charge_state()==RESET)		//闂備礁鎼悧婊勭椤忓牆鏋侀柟鎹愵嚙濡﹢鏌曢崼婵囶棞妞ゅ繐鐖煎铏规崉閵娿儲鐎鹃梺鍝勵儏椤兘鐛箛娑欐櫢闁跨噦鎷�
		{
			blinkFlag =(blinkFlag==0)?1:0;	
			Bat_ValueBlink(batValue,blinkFlag);				
		}
		else
		{
			blinkFlag =0;
			Bat_ValueBlink(batValue,0);				
		}	
	}
}
/**
  * @brief  Page  turns
  * @param  pageID
  * @retval none
  */
static void MenuPageTurns(unsigned char pageID)
{	
	if(pageID>MENU_MAX_PAGE_NUM) return ;
	OLED_Clear_no_fresh();
	DeviceOffTimeManage(xTaskGetTickCount(),0);
	if(pageID==MENU_LOGO_PAGE)
	{
		OLED_Display_LOGO(0);//disp LOGO		
	}
	else if(pageID==MENU_HOME_PAGE)
	{		
		HomePageHandle(motor_setting_updata_signal ,ADJUST_MOTOR_PARAM_PROGRAM_NUM);
		disp_batValue(2,32,get_vbat_value());//prepare				
	}
	else if(pageID==MENU_SYSTEM_SET_PAGE)
	{
		#ifdef APEX_FUNCTION_EBABLE	
		SubmenuPageTurns(SETTING_FOR);
		#else
		SubmenuPageTurns(CW_ANGLE);
		#endif			
	}
	else if(pageID==MENU_MOTOR_WORK_PAGE)
	{		
		OLED_Display_WORK(MENU_MOTOR_WORK_PAGE,0);
	}
	else if(pageID==MENU_CHARGING_PAGE)
	{	
		OLED_Display_CHARGING(3300,0);		
	}
	else if(pageID==MENU_ONLY_APEX_PAGE)
	{	
		OLED_Display_WORK(MENU_ONLY_APEX_PAGE,0);	
		disp_batValue(2,32,get_vbat_value());//prepare		
	}
	else if(pageID==MENU_APEX_AND_MOTOR_PAGE)
	{	
		OLED_Display_WORK(MENU_APEX_AND_MOTOR_PAGE,0);		
	}
	else if(pageID==MENU_MAX_PAGE_NUM)//闂傚倷娴囧▔鏇㈠窗閹版澘鏄ラ柛娑卞枛椤曡鲸绻涢崱妯哄妞ゅ繐鐖奸弻娑㈠箻瀹曞泦褔鏌℃担瑙勫磳鐎殿噮鍓熸俊鍫曞幢濡ゅ﹣绱︾紓鍌欒閸嬫捇鏌ㄩ悤鍌涘
	{
		OLED_Display_POWER_OFF(0);
	}
}
/*=======================Menu Manage Task===================================
			* @brief:  Menu Manage Task.
			* @param:  None
			* @retval: None
==============================================================================*/

void vAppMenuManageTask( void * pvParameters )
{ 
	uint32_t rec_Signal=0;//
	static uint8_t menuPage=MENU_LOGO_PAGE,motor_run_cmd=0,selectNum=0,submenuPageNum=0,motorOrApexSetFlag=0,exitApexFlag=1;
	confirm_state  apexMotorStartFlag =TRUE;//apex闂傚倷娴囧▔鏇㈠窗閹版澘鍑犲┑鐘宠壘缁狀垶鏌ｉ幋锝呅撻柡鍛倐閺岋繝宕掑Ο琛″亾閺嶎偀鍋撳顒傜Ш闁哄被鍔戦幃銏ゅ川婵犲嫪绱曢梻浣哥秺椤ユ捇宕楀鈧顐﹀箻鐠囧弶顥濋梺闈涚墕濡顢旈崼鏇熲拺閻犳亽鍔岄弸鎴︽煛鐎ｎ亶鐓兼鐐茬箰閻ｏ繝骞嶉缁㈠晙
	static unsigned 	short int  batDispValue;
	void *pMalloc=NULL;
	
	for(;;)
	{				
		if(xQueueReceive(xQueueKeyMessage, &rec_Signal, 0) == pdTRUE)
		{			
			MenuIdleTimeManage(xTaskGetTickCount(),0); //clear ticks
			DeviceOffTimeManage(xTaskGetTickCount(),0);//clear ticks	 			
			if(rec_Signal==low_power_signal)
			{			
			//beep
			//disp low power
			}
			else if(rec_Signal==run_button_long_press_signal)
			{	
				OLED_Clear_no_fresh();
				menuPage=MENU_MAX_PAGE_NUM;					
				OLED_Display_POWER_OFF(1);
				xSemaphoreGive(xSemaphoreDispRfresh);	
	 		}		
			else if(rec_Signal==power_off_signal)
			{
				OLED_Clear_no_fresh();
				menuPage=MENU_MAX_PAGE_NUM;
				OLED_Display_POWER_OFF(0);
				xSemaphoreGive(xSemaphoreDispRfresh);			
//				MenuDevicePowerOff(0);
			}	
		}						 
        switch(menuPage)
		{
		    case MENU_LOGO_PAGE:										 
				MenuPageTurns(menuPage);
				xSemaphoreGive(xSemaphoreDispRfresh);	
				#ifdef WDT_ENABLE
				//	1s
				xEventGroupSetBits(WDTEventGroup,MENU_TASK_EVENT_BIT);
				vTaskDelay(MAX_WDT_FEED_TIME_MS);			 
				xEventGroupSetBits(WDTEventGroup,MENU_TASK_EVENT_BIT);
				vTaskDelay(MAX_WDT_FEED_TIME_MS);						
				xEventGroupSetBits(WDTEventGroup,MENU_TASK_EVENT_BIT);
				vTaskDelay(MAX_WDT_FEED_TIME_MS);
				xEventGroupSetBits(WDTEventGroup,MENU_TASK_EVENT_BIT);				
				#else
					vTaskDelay(1000);//	1s
				#endif				
				batDispValue=	get_vbat_value();	
				if(sys_param_un.device_param.use_p_num==10)
				{							
					menuPage=MENU_ONLY_APEX_PAGE;								
				}
				else
				{
					menuPage=MENU_HOME_PAGE;								
				}
				MenuMotorParamUpdate(sys_param_un.device_param.use_p_num);//save data						
				MenuPageTurns(menuPage);
				xSemaphoreGive(xSemaphoreDispRfresh);
			break;
			case MENU_HOME_PAGE:								
				if(rec_Signal==run_button_press_signal)
				{		
					apexMotorStartFlag=FALSE;//闂傚倷娴囧▔鏇㈠窗閹版澘鍑犲┑鐘宠壘缁狀垶姊洪崹顕呭剾闁告捇绠栧铏规崉閵娿儲鐎鹃梺绯曟櫅閹虫ê螞閸愵喖鐓涢柛灞惧嚬娴兼捇姊绘担鑺ョ《闁哥姵鍔欏鍛婄節濮橆剛顔嗛梺缁樺灱婵倝寮查幖浣圭厸闁稿本锚閳ь剚鐗滈埀顒佺啲閹凤拷
					MenuMotorParamUpdate(sys_param_un.device_param.use_p_num);//save data							
					if(selectNum==ADJUST_MOTOR_PARAM_PROGRAM_NUM)
					{
						if(sys_param_un.device_param.auto_stop_flag==0&&sys_param_un.device_param.auto_start_flag==0&&sys_param_un.device_param.apical_action_flag==0)
						{	//闂傚倷娴囧▔鏇㈠窗閹版澘鍑犲┑鐘宠壘缁狀垶鏌ｉ幋锝呅撻柡鍛倐閺岋繝宕掑Ο琛″亾閺嶎偀鍋撳顒傜Ш闁哄被鍔戦幃銏ゅ川婵犲嫪绱曢梻浣烘嚀閸ゆ牠骞忛敓锟�
							menuPage = MENU_MOTOR_WORK_PAGE;
							if(motor_run_cmd==MOTOR_MODE_STOP) // motor control
							{
								motor_run_cmd=MOTOR_MODE_START;
							}									
							App_MotorControl(motor_run_cmd);	
						}
						else
						{
							if(sys_param_un.device_param.apexFunctionLoad!=0)//闂傚倷娴囧▔鏇㈠窗閹版澘鍑犲┑鐘宠壘缁狀垶鏌ｉ幋锝呅撻柡鍛倐閺岋繝宕掑Ο琛″亾閺嶎偀鍋撳顒傜Ш闁哄被鍔戦幃銏ゅ川婵犲嫪绱曢梻浣哥秺椤ユ捇宕楀鈧顐﹀箻鐠囧弶顥濋梺闈涚墕濡顢旈崼鏇熲拺閻犳亽鍔屽▍鎰版煙閸戙倖瀚�																						
							{		//闂傚倷娴囧▔鏇㈠窗閹版澘鍑犲┑鐘宠壘缁狀垶鏌ｉ幋锝呅撻柡鍛倐閺岋繝宕掑Ο琛″亾閺嶎偀鍋撳鎰佹綈缂佸顦甸崺鈧柨鐕傛嫹							
								menuPage = MENU_APEX_AND_MOTOR_PAGE;
//								if(sys_param_un.device_param.auto_stop_flag==0)
//								{
									if(motor_run_cmd==MOTOR_MODE_STOP) // motor control
									{
										motor_run_cmd=MOTOR_MODE_START;
									}									
									App_MotorControl(motor_run_cmd);
//								}	
//								else 
//								{
//									motor_run_cmd=MOTOR_MODE_STOP;//闂傚倷娴囧▔鏇㈠窗閹版澘鍑犲┑鐘宠壘缁狀垶鏌ｉ幋锝呅撻柡鍛倐閺岋綁顢樿娴滄粓鎮樿箛鎾村殗妤犵偛绻橀幃婊堟嚍閵夛附顏熼梻浣告惈閸婂爼宕愰弽顐熷亾濮樼偓瀚�
//								}											
							}	
							else 
							{															
								menuPage = MENU_MOTOR_WORK_PAGE;
								if(motor_run_cmd==MOTOR_MODE_STOP) // motor control
								{
									motor_run_cmd=MOTOR_MODE_START;
								}									
								App_MotorControl(motor_run_cmd);	
							}	
						}				
						MenuPageTurns(menuPage);	
					}
					else
					{//save motor param								
						selectNum=ADJUST_MOTOR_PARAM_PROGRAM_NUM;	
						HomePageHandle(motor_setting_updata_signal,ADJUST_MOTOR_PARAM_PROGRAM_NUM);									
					}										
				}				
				else if(rec_Signal==s_button_long_press_signal)
				{						
					menuPage = MENU_SYSTEM_SET_PAGE;//to set 
					submenuPageNum=0;			
//					motorOrApexSetFlag%=2;	
					motorOrApexSetFlag=0;	//濠殿喗甯楃粙鎺楁晝閵忋倕鏋侀柟鎹愵嚙濡﹢鏌曢崼婵囶棞妞ゅ繐鐖奸弻娑欑節閸曨厹鈧湦TOR						
					MenuPageTurns(menuPage);
				}							
				else 
				{		
					if(rec_Signal==s_button_press_signal)
					{
						if(sys_param_un.device_param.use_p_num>5)
						{
							selectNum++;	
							selectNum%=(ADJUST_MOTOR_PARAM_DIRECTION+1);	
//							if(selectNum==ADJUST_MOTOR_PARAM_PROGRAM_NUM) selectNum=ADJUST_MOTOR_PARAM_SPEED;
							//if(selectNum==ADJUST_MOTOR_PARAM_TORQUE&&motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].dir==EndoModePositionToggle) 
							//{
							//	selectNum=ADJUST_MOTOR_PARAM_DIRECTION;
							//}
							HomePageHandle(motor_setting_updata_signal,selectNum);									
						}	
					}
					else
					{							
						if(selectNum==ADJUST_MOTOR_PARAM_PROGRAM_NUM)
						{									
							if(rec_Signal==sub_button_press_signal)
							{
								#ifdef APEX_FUNCTION_EBABLE
									sys_param_un.device_param.use_p_num++;		
									sys_param_un.device_param.use_p_num%=11;//apex
								#else 
									sys_param_un.device_param.use_p_num++;		
									sys_param_un.device_param.use_p_num%=10;//no apex											
								#endif
								HomePageHandle(motor_setting_updata_signal,ADJUST_MOTOR_PARAM_PROGRAM_NUM);
							}
							else if(rec_Signal==add_button_press_signal)
							{										
								#ifdef APEX_FUNCTION_EBABLE
								if(sys_param_un.device_param.use_p_num==0)		
								{
									sys_param_un.device_param.use_p_num=10;//apex	
								}
								else
								{
									sys_param_un.device_param.use_p_num--;		
									sys_param_un.device_param.use_p_num%=11;//apex
									HomePageHandle(motor_setting_updata_signal,ADJUST_MOTOR_PARAM_PROGRAM_NUM);//闂傚倷娴囧▔鏇㈠窗鎼淬們浜归柕濠忕畱椤曡鲸绻涢崱妯哄妞ゅ繐鐖煎铏规崉閵娿儲鐎鹃梺鍝勵儏椤兘鐛箛娑欏€婚柤鎭掑劜濞呫垽姊洪崫鍕偓鍫曞磹閺嶎偀鍋撳顒傜Ш闁哄被鍔戦幃銏ゅ川婵犲嫪绱曢梻浣哥秺椤ユ捇宕楀鈧顐﹀箻缂佹ɑ娅㈤梺璺ㄥ櫐閹凤拷
								}	
								#else 
									sys_param_un.device_param.use_p_num--;		
									sys_param_un.device_param.use_p_num%=10;//no apex
								#endif							
							}											
							if(sys_param_un.device_param.use_p_num==10)			
							{
								menuPage = MENU_ONLY_APEX_PAGE;//to apex
								MenuPageTurns(menuPage);
							}																																
						}	
						else 
						{		
							if(sys_param_un.device_param.use_p_num<10)							
							{ 										
								HomePageHandle(rec_Signal,selectNum);
							}									
						}	
						if(rec_Signal==null_signal)//APEX闂傚倷娴囧▔鏇㈠窗閹版澘鍑犲┑鐘宠壘缁狀垶鏌ｉ幋锝呅撻柡鍛倐閺岋繝宕掑Ο琛″亾閺嶎偀鍋撳顒傜Ш闁哄被鍔戦幃銏ゅ川婵犲嫪绱曢梻浣哥秺椤ユ捇寮绘径鎰┾偓鍛存晝閸屾稓鍙嗛梺缁樻煥閹诧繝鎮橀鈧弻鐔割槹鎼粹€冲箣闂佽桨鐒﹂幑鍥蓟瑜斿鍫曞垂椤旂偓顔愰梻浣哥秺椤ユ捇宕楀鈧顐﹀箻鐠囧弶顥濋梺闈涚墕濡顢旈敓锟�
						{	
							if(MenuIdleTimeManage(xTaskGetTickCount(),HOME_PARAM_IDLE_TICKS))
							{	
								#ifdef APEX_FUNCTION_EBABLE				
									submenuPageNum=SETTING_FOR;
								#else 
									submenuPageNum=CW_ANGLE;
								#endif								
								selectNum=ADJUST_MOTOR_PARAM_PROGRAM_NUM;								
								HomePageHandle(motor_setting_updata_signal,selectNum);
							}										
							if(sys_param_un.device_param.apexFunctionLoad!=0)
							{								
								if(sys_param_un.device_param.auto_start_flag!=0)
								{
									if(GC_depth_vlaue(0, 1)<27)
									{	
										if(apexMotorStartFlag==TRUE)	//闂傚倷娴囧▔鏇㈠窗閹版澘鍑犲┑鐘宠壘缁狀垶鏌ｉ幋锝呅撻柡鍛倐閺岋繝宕掑Ο琛″亾閺嶎偀鍋撳顒傜Ш闁哄被鍔戦幃銏ゅ川婵犲嫪绱曢梻浣哥秺椤ユ捇宕楀鈧顐﹀箻鐠囧弶顥濋梺闈涚墕濡顢旈崼鏇熲拺閻犳亽鍔岄弸鎴︽煛鐎ｎ亶鐓兼鐐茬箻閹粓鎳為妷锔筋仧闂備礁鎼崐鍫曞磹閺嶎偀鍋撳顒傜Ш闁哄被鍔戦幃銏ゅ川婵犲嫪绱曢梻浣哥秺椤ユ捇宕楀鈧顐﹀箻鐠囧弶顥濋梺闈涚墕濡顢旂捄琛℃闁圭虎鍨版禍楣冩⒒娴ｈ姤纭堕柛鐘冲姍瀵憡绻濆顒傤唵闂佽法鍣﹂幏锟�
										{	
//										apexMotorStartFlag=FALSE;//闂傚倷娴囧▔鏇㈠窗閹版澘鍑犲┑鐘宠壘缁狀垶姊洪崹顕呭剾闁告捇绠栧铏规崉閵娿儲鐎鹃梺绯曟櫅閹虫ê螞閸愵喖鐓涢柛灞惧嚬娴兼捇姊绘担鑺ョ《闁哥姵鍔欏鍛婄節濮橆剛顔嗛梺缁樺灱婵倝寮查幖浣圭厸闁稿本锚閳ь剚鐗滈埀顒佺啲閹凤拷
											MenuMotorParamUpdate(sys_param_un.device_param.use_p_num);//save data	
											if(GC_depth_vlaue(0, 1)>=0&&sys_param_un.device_param.apical_action_flag!=2)
											{
												if(motor_run_cmd==MOTOR_MODE_STOP) // motor control
												{
														motor_run_cmd=MOTOR_MODE_START;
												}	
												App_MotorControl(motor_run_cmd);			//闂傚倷娴囧▔鏇㈠窗閹版澘鍑犲┑鐘宠壘缁狀垶鏌ｉ幋锝呅撻柡鍛倐閹嘲鈻庡▎鎴濆煂婵犳鍠掗弲鐘诲蓟閵娾晜鍋勯柛娑橈功娴煎嫰姊洪悷鏉挎闁瑰嚖鎷�
											}														
											menuPage = MENU_APEX_AND_MOTOR_PAGE;//闂傚倷娴囧▔鏇㈠窗閹版澘鍑犲┑鐘宠壘缁狀垶鏌ｉ幋锝呅撻柡鍛倐閺岋繝宕掑Ο琛″亾閺嶎偀鍋撳顒傜Ш闁哄被鍔戦幃銏ゅ川婵犲嫪绱曢梻浣烘嚀閸ゆ牠骞忛敓锟�
											MenuPageTurns(menuPage);
										}
									}	
                					else 
									{
										apexMotorStartFlag=TRUE;//闂傚倷娴囧▔鏇㈠窗閹版澘鍑犲┑鐘宠壘缁狀垶鏌ｉ幋锝呅撻柡鍛倐閺岋繝宕掑Ο琛″亾閺嶎偀鍋撳顒傜Ш闁哄被鍔戦幃銏ゅ川婵犲嫪绱曢梻浣哥秺椤ユ捇宕楀鈧顐﹀箻鐠囧弶顥濋梺闈涚墕濡顢旈崼鏇熲拺閻犳亽鍔岄弸鎴︽煛鐎ｎ亶鐓兼鐐茬箻閺佹捇鏁撻敓锟�
									}												
								}						
							}									
							else
							{
								apexMotorStartFlag=TRUE;//闂傚倷娴囧▔鏇㈠窗閹版澘鍑犲┑鐘宠壘缁狀垶鏌ｉ幋锝呅撻柡鍛倐閺岋繝宕掑Ο琛″亾閺嶎偀鍋撳顒傜Ш闁哄被鍔戦幃銏ゅ川婵犲嫪绱曢梻浣哥秺椤ユ捇宕楀鈧顐﹀箻鐠囧弶顥濋梺闈涚墕濡顢旈崼鏇熲拺閻犳亽鍔岄弸鎴︽煛鐎ｎ亶鐓兼鐐茬箻閺佹捇鏁撻敓锟�
							}
						}																	
					}								
					if(DeviceOffTimeManage(xTaskGetTickCount(),DEVICE_POWER_OFF_TICKS))//3 minute
					{
						OLED_Clear_no_fresh();					
						OLED_Display_POWER_OFF(1);
						xSemaphoreGive(xSemaphoreDispRfresh);	
						#ifdef WDT_ENABLE//	1s								
						xEventGroupSetBits(WDTEventGroup,MENU_TASK_EVENT_BIT);
						vTaskDelay(MAX_WDT_FEED_TIME_MS);
						xEventGroupSetBits(WDTEventGroup,MENU_TASK_EVENT_BIT);
						vTaskDelay(MAX_WDT_FEED_TIME_MS);						
						#else
						vTaskDelay(600);//	1s
						#endif
						MenuDevicePowerOff(0);
					}		
				}	
			break;
			case MENU_SYSTEM_SET_PAGE:					
				if(rec_Signal==s_button_long_press_signal||rec_Signal==run_button_press_signal)
				{													
					SubmenuPageHandle(submenuPageNum,motor_setting_updata_signal,0,0);
					menuPage = MENU_HOME_PAGE;//exit						
					MenuPageTurns(menuPage);
				}
				else 
				{												
					if(rec_Signal==s_button_press_signal)
					{
						#ifdef APEX_FUNCTION_EBABLE	
						if(motorOrApexSetFlag==0)//motor
						{
							if(submenuPageNum==SETTING_FOR)
							{								
								submenuPageNum=CW_ANGLE;//
							}
							else
							{
								submenuPageNum++;
								submenuPageNum%=MAX_SUBMENU;
							}								
						}
						else//闂傚倷娴囧▔鏇㈠窗閹版澘鍑犲┑鐘宠壘缁狀垶鏌ｉ幋锝呅撻柡鍛倐閺岋繝宕掑Ο琛″亾閺嶎偀鍋撳鐐
						{
							if(submenuPageNum==SETTING_FOR)
							{								
								submenuPageNum=AUTO_START;//
							}							
							else
							{ 
								submenuPageNum++;
								submenuPageNum%=(APICAL_OFFESET+1);
							}								
						}	
						#else
						submenuPageNum++;
						submenuPageNum%=MAX_SUBMENU;
						if(submenuPageNum<CW_ANGLE) submenuPageNum=CW_ANGLE;
						#endif															
						SubmenuPageTurns(submenuPageNum);
					}		
					else
					{		
            			if(rec_Signal==add_button_press_signal||rec_Signal==sub_button_press_signal)	
						{
							if(submenuPageNum==SETTING_FOR)
							{
								motorOrApexSetFlag++;
								motorOrApexSetFlag%=2;
							}									
						}								
						SubmenuPageHandle(submenuPageNum,rec_Signal,xTaskGetTickCount(),1+motorOrApexSetFlag);	
						if(rec_Signal==null_signal)
						{
							if(MenuIdleTimeManage(xTaskGetTickCount(),MENU_IDLE_TICKS))
							{
								#ifdef APEX_FUNCTION_EBABLE	
								submenuPageNum=SETTING_FOR;
								#else 
								submenuPageNum=CW_ANGLE;
								#endif			
								SubmenuPageHandle(submenuPageNum,motor_setting_updata_signal,0,0);
								menuPage = MENU_HOME_PAGE;//exit						
								MenuPageTurns(menuPage);
							}
						}											
					}					
				}						
			break;
			case MENU_MOTOR_WORK_PAGE:
				if(rec_Signal==run_button_press_signal||rec_Signal==low_power_signal)
				{							
					if(motor_run_cmd==MOTOR_MODE_START) //motor control
					{						
						motor_run_cmd=MOTOR_MODE_STOP;
					}	
					App_MotorControl(motor_run_cmd);						
					menuPage=MENU_HOME_PAGE;		
					MenuPageTurns(menuPage);
				}							
				WorkPageFlash(MENU_MOTOR_WORK_PAGE,xTaskGetTickCount());	
				if(sys_param_un.device_param.apexFunctionLoad!=0)//闂傚倷娴囧▔鏇㈠窗閹版澘鍑犲┑鐘宠壘缁狀垶鏌ｉ幋锝呅撻柡鍛倐閺岋繝宕掑Ο琛″亾閺嶎偀鍋撳顒傜Ш闁哄被鍔戦幃銏ゅ川婵犲嫪绱曢梻浣哥秺椤ユ捇宕楀鈧顐﹀箻鐠囧弶顥濋梺闈涚墕濡顢旈崼鏇熲拺閻犳亽鍔屽▍鎰版煙閸戙倖瀚�
				{					
					menuPage=MENU_APEX_AND_MOTOR_PAGE;	//闂傚倷娴囧▔鏇㈠窗鎼淬們浜归柕濠忕畱椤曡鲸绻涢崱妯哄妞ゅ繐鐖煎铏规崉閵娿儲鐎鹃梺鍝勵儏椤兘鐛箛娑欏€婚柤鎭掑劜濞呫垽姊洪崫鍕偓鍫曞磹閺嶎偀鍋撳顒傜Ш闁哄被鍔戦幃銏ゅ川婵犲嫪绱曢梻浣哥秺椤ユ捇宕楀鈧顐﹀箻鐠囧弶顥濋梺闈涚墕濡顢旈崼鏇熲拺閻犳亽鍔岄弸鎴︽煛鐎ｎ亶鐓兼鐐茬箻閹粓鎳為妷锔筋仧闂備礁鎼崐鍫曞磹閺嶎偀鍋撳顒傜Ш闁哄被鍔戦幃銈夊磼濞戞﹩浼�	
					MenuPageTurns(menuPage);
				}													
				 break;
			case MENU_ONLY_APEX_PAGE:	
				if(rec_Signal==sub_button_press_signal)
				{	
					sys_param_un.device_param.use_p_num++;
					sys_param_un.device_param.use_p_num%=11;
					menuPage=MENU_HOME_PAGE;		
					MenuPageTurns(menuPage);
				}
				else if(rec_Signal==add_button_press_signal)
				{												
					sys_param_un.device_param.use_p_num--;		
					sys_param_un.device_param.use_p_num%=11;//apex	
					menuPage=MENU_HOME_PAGE;		
					MenuPageTurns(menuPage);
				}							
				else if(rec_Signal==s_button_long_press_signal)
				{								
					menuPage = MENU_SYSTEM_SET_PAGE;//to set
					submenuPageNum=0;					
					MenuPageTurns(menuPage);							
				}												
				else
				{	 						
					WorkPageFlash(MENU_ONLY_APEX_PAGE,xTaskGetTickCount());	
					if(sys_param_un.device_param.apexFunctionLoad!=0&&exitApexFlag!=0)
					{
						exitApexFlag=0;//闂傚倷娴囧▔鏇㈠窗閹版澘鍑犲┑鐘宠壘缁狀垶鏌ｉ幋锝呅撻柡鍛倐閺岋繝宕掑Ο琛″亾閺嶎偀鍋撳顒傜Ш闁诡喒鏅犻幊婊呭枈濡桨澹曢梺璺ㄥ櫐閹凤拷
					}
					if(sys_param_un.device_param.apexFunctionLoad==0&&exitApexFlag==0)//闂傚倷娴囧▔鏇㈠窗閺囩喓绠鹃柛銉㈡櫇妞规娊鎮峰▎蹇擃仼妞ゅ繐鐖煎铏规崉閵娿儲鐎鹃梺鍝勵儏椤兘鐛箛娑欏€婚柤鎭掑劜濞呫垽姊洪崫鍕偓鍫曞磹閺嶎偀鍋撳顒傜Ш闁哄被鍔戦幃銏ゅ川婵犲嫪绱曢梻浣烘嚀閸ゆ牠骞忛敓锟�,闂傚倷娴囧▔鏇㈠窗鎼淬們浜归柕濠忕畱椤曡鲸绻涢崱妯哄妞ゅ繐鐖煎铏规崉閵娿儲鐎鹃梺鍝勵儏椤兘鐛箛娑欏€婚柤鎭掑劜濞呫垽姊洪崫鍕偓鍫曞磹閺嶎偀鍋撳顒傜Ш闁哄被鍔戦幃銏ゅ川婵犲嫪绱曢梻浣哥秺椤ユ捇宕楀鈧顐﹀箻鐠囧弶顥濋梺闈涚墕濡顢旈崼鏇熲拺閻犳亽鍔岄弸鎴︽煛鐎ｎ亶鐓兼鐐茬箰閻ｏ繝骞忛弮鈧惄顖炪€侀弽銊︾秶妞ゆ劧绲藉☉褔姊鸿ぐ鎺濇闁稿繑锕㈠顐﹀箻鐠囪尙顔屽銈嗘尰缁诲嫭绋夐弻銉︾厵濡炲楠搁崢鎾煛娴ｈ宕岄柡浣规崌閺佹捇鏁撻敓锟�
					{
						exitApexFlag=1;
						//sys_param_un.device_param.use_p_num=0;
						//menuPage=MENU_HOME_PAGE;		
						//MenuPageTurns(menuPage);
					}	
				}													
			break;
			case MENU_APEX_AND_MOTOR_PAGE:
				if(rec_Signal==run_button_press_signal||rec_Signal==low_power_signal)
				{	
					apexMotorStartFlag=FALSE;//闂傚倷娴囧▔鏇㈠窗閹版澘鍑犲┑鐘宠壘缁狀垶姊洪崹顕呭剾闁告捇绠栧铏规崉閵娿儲鐎鹃梺绯曟櫅閹虫ê螞閸愵喖鐓涢柛灞惧嚬娴兼捇姊绘担鑺ョ《闁哥姵鍔欏鍛婄節濮橆剛顔嗛梺缁樺灱婵倝寮查幖浣圭厸闁稿本锚閳ь剚鐗滈埀顒佺啲閹凤拷
					if(motor_run_cmd == MOTOR_MODE_START) //motor control
					{						
						motor_run_cmd=MOTOR_MODE_STOP;
					}		
					menuPage=MENU_HOME_PAGE;//page change					
					MenuPageTurns(menuPage);
					App_MotorControl(MOTOR_MODE_STOP);						
				 	//if(rec_Signal==run_button_press_signal&&GC_depth_vlaue(0, 1)<=(3+sys_param_un.device_param.ref_tine)&&sys_param_un.device_param.apical_action_flag==2)//闂傚倷娴囧▔鏇㈠窗閹版澘鍑犲┑鐘宠壘缁狀垶鏌ｉ幋锝呅撻柡鍛倐閺岋繝宕掑Ο琛″亾閺嶎偀鍋撳顒傜Ш闁诡喒鏅犻幊婊呭枈濡桨澹曢梺璺ㄥ櫐閹凤拷
				//	{//闂傚倷娴囧▔鏇㈠窗閹版澘鍑犲┑鐘宠壘缁狀垶鏌ｉ幋锝呅撻柡鍛倐閺岋繝宕掑Ο琛″亾閺嶎偀鍋撳顒傜Ш鐎规洘鍨甸オ浼村礋闂堟稑顥濋梻鍌欐祰濞夋洟宕伴幇鏉垮嚑濠电姵鑹剧粻顖炴煟閹达絽袚闁哄懏鎮傞弻锟犲磼濡　鍋撻弽顐熷亾濮橆剚璐￠柟鑼缁楃喖鍩€椤掑嫬鏋侀柟鎹愵嚙濡﹢鏌曢崼婵囶棞妞ゅ繐鐖煎铏规崉閵娿儲鐎鹃梺鍝勵儏椤兘鐛箛娑欏€婚柤鎭掑劜濞呫垽姊洪崫鍕偓鍫曞磹閺嶎偀鍋撳顒傜Ш闁哄被鍔戦幃銏ゅ川婵犲嫪绱曢梻浣哥秺椤ユ捇宕楀鈧顐﹀箻鐠囧弶顥濋梺闈涚墕濡顢旈崼鏇熲拺閻犳亽鍔岄弸鎴︽煛鐎ｎ亶鐓兼鐐茬箻閹粓鎳為妷锔筋仧闂備礁鎲￠幑浣糕枖閺囥埄鏁嗛柡灞诲劚缁狀垶鏌ㄩ悤鍌涘,闂傚倷娴囧▔鏇㈠窗閹版澘鍑犲┑鐘宠壘缁狀垶姊洪崹顕呭剱妞は佸洦鈷戦悹鎭掑妼閺嬫垿鏌＄€ｎ亶鐓兼鐐茬箻閹粓鎳為妷锔筋仧闂備礁鎼崐鍫曞磹閺嶎偀鍋撳顒傜Ш闁哄被鍔戦幃銏ゅ川婵犲嫪绱曢梻浣哥秺椤ユ捇宕楀鈧顐﹀箻鐠囧弶顥濋梺闈涚墕濡顢旈敓锟�
//					if(motor_run_cmd==MOTOR_MODE_STOP)  //motor control
//					{		
//							motor_run_cmd=MOTOR_MODE_START;
//					}	
//						App_MotorControl(motor_run_cmd);	
				//	}
				}	
				else if(rec_Signal==motor_apex_stop_signal) //闂傚倷娴囧▔鏇㈠窗閹版澘鍑犲┑鐘宠壘缁狀垶鏌ｉ幋锝呅撻柡鍛倐閺屾盯骞樼憴鍕€婂銈嗗笚閻擄繝鐛箛娑欏€婚柤鎭掑劜濞呫垽姊洪崫鍕偓鍫曞磹閺嶎偀鍋撳顒傜Ш闁哄被鍔戦幃銏ゅ川婵犲嫪绱曢梻浣烘嚀閸ゆ牠骞忛敓锟�
				{
					apexMotorStartFlag=TRUE;//闂傚倷娴囧▔鏇㈠窗閹版澘鍑犲┑鐘宠壘缁狀垶鏌ｉ幋锝呅撻柡鍛倐閺岋繝宕掑Ο琛″亾閺嶎偀鍋撳顒傜Ш闁哄被鍔戦幃銏ゆ倻濡吋娈鹃梺鑽ゅС濞村洭锝炴径灞稿亾濮橆剛绉洪柡灞诲姂閹垽宕ㄦ繝鍕磿闂備礁缍婇ˉ鎾诲礂濮椻偓瀵偊骞樼拠鍙夘棟闂侀潧鐗嗗Λ妤咁敂閸洘鈷戦悹鎭掑妼閺嬫垿鏌＄€ｎ亶鐓兼鐐茬箻閹粓鎳為妷锔筋仧闂備礁鎼崐鍫曞磹閺嶎偀鍋撳鐐	
					if(motor_run_cmd==MOTOR_MODE_START) //motor control
					{						
						motor_run_cmd=MOTOR_MODE_STOP;
					}		
					menuPage=MENU_HOME_PAGE;		//page change					
					MenuPageTurns(menuPage);
					App_MotorControl(MOTOR_MODE_STOP);
				}					
				else
				{							
					WorkPageFlash(MENU_APEX_AND_MOTOR_PAGE,xTaskGetTickCount());
					if(sys_param_un.device_param.apexFunctionLoad==0)//闂傚倷娴囧▔鏇㈠窗閺囩喓绠鹃柛銉㈡櫇妞规娊鎮峰▎蹇擃仼妞ゅ繐鐖煎铏规崉閵娿儲鐎鹃梺鍝勵儏椤兘鐛箛娑欏€婚柤鎭掑劜濞呫垽姊洪崫鍕偓鍫曞磹閺嶎偀鍋撳顒傜Ш闁哄被鍔戦幃銏＄瑹椤栨瑧妫梻浣瑰缁诲倻鎹㈤幋鐘亾濮橆剛绉洪柡灞诲姂閹垹鐣￠柇锔斤紒缂傚倷鐒︾粙鏍綖婢跺备鍋撳顒傜Ш闁哄被鍔戦幃銏ゅ川婵犲嫪绱曢梻浣哥秺椤ユ捇宕楀鈧顐﹀箻鐠囧弶顥濋梺闈涚墕濡顢旈崼鏇熺厽婵☆垰鐏濋悡鎰版煃瑜滈崜銊╁箯閿燂拷
					{	
						apexMotorStartFlag=TRUE;//闂傚倷娴囧▔鏇㈠窗閹版澘鍑犲┑鐘宠壘缁狀垶鏌ｉ幋锝呅撻柡鍛倐閺岋繝宕掑Ο琛″亾閺嶎偀鍋撳顒傜Ш闁哄被鍔戦幃銏ゆ倻濡吋娈鹃梺鑽ゅС濞村洭锝炴径灞稿亾濮橆剛绉洪柡灞诲姂閹垽宕ㄦ繝鍕磿闂備礁缍婇ˉ鎾诲礂濮椻偓瀵偊骞樼拠鍙夘棟闂侀潧鐗嗗Λ妤咁敂閸洘鈷戦悹鎭掑妼閺嬫垿鏌＄€ｎ亶鐓兼鐐茬箻閹粓鎳為妷锔筋仧闂備礁鎼崐鍫曞磹閺嶎偀鍋撳鐐							
						motor_run_cmd =	MOTOR_MODE_STOP;						
						App_MotorControl(motor_run_cmd);							
						menuPage=MENU_HOME_PAGE;							
						MenuPageTurns(menuPage);
					}							
				}    			
			break;	
			case 	MENU_CHARGING_PAGE:				
				if(motor_run_cmd==MOTOR_MODE_START) //motor control
				{						
					motor_run_cmd=MOTOR_MODE_STOP;
					App_MotorControl(motor_run_cmd);						
				}					
			break;	
			case 	MENU_MAX_PAGE_NUM:				
				if(rec_Signal==run_button_long_press_signal)
				{									
					#ifdef WDT_ENABLE//	1s								
					xEventGroupSetBits(WDTEventGroup,MENU_TASK_EVENT_BIT);
					vTaskDelay(MAX_WDT_FEED_TIME_MS);
					xEventGroupSetBits(WDTEventGroup,MENU_TASK_EVENT_BIT);
					vTaskDelay(MAX_WDT_FEED_TIME_MS);						
					#else
					vTaskDelay(600);//	1s
					#endif							
					MenuDevicePowerOff(1);													
				}		
				else //if(rec_Signal==power_off_signal)
				{
					#ifdef WDT_ENABLE//	1s								
					xEventGroupSetBits(WDTEventGroup,MENU_TASK_EVENT_BIT);
					vTaskDelay(MAX_WDT_FEED_TIME_MS);
					xEventGroupSetBits(WDTEventGroup,MENU_TASK_EVENT_BIT);
					vTaskDelay(MAX_WDT_FEED_TIME_MS);						
					#else
					vTaskDelay(600);//	1s
					#endif	
					MenuDevicePowerOff(0);
				}		
			break;
			default:
				if(motor_run_cmd==MOTOR_MODE_START) //motor control
				{						
					motor_run_cmd=MOTOR_MODE_STOP;								
				}
				App_MotorControl(motor_run_cmd);
				menuPage=MENU_HOME_PAGE;							
				MenuPageTurns(menuPage);
			break;			
		}		 
		if(rec_Signal!=null_signal)
		{	
			#ifdef DEBUG_RTT
				SEGGER_RTT_printf(0, "key run %d\r\n", rec_Signal);
			#endif
			rec_Signal= null_signal;	
			if(menuPage==MENU_SYSTEM_SET_PAGE||menuPage	==MENU_HOME_PAGE)
			{
				xSemaphoreGive(xSemaphoreDispRfresh); //闂傚倷娴囧▔鏇㈠窗閹版澘鍑犲┑鐘宠壘缁狀垶鏌ｉ幋锝呅撻柡鍛倐閺岋繝宕掑Ο琛″亾閺嶎偀鍋撳顒傜Ш闁哄被鍔戦幃銏ゅ川婵犲嫪绱曢梻浣哥秺椤ユ捇宕楀鈧顐﹀箻缂佹鍙勯柣銏╁灱閸犳氨绮旈崼鏇熺厵濡炲楠搁崢鎾煃瑜滈崜姘潩閵娾晜鍋傞柨鐔哄Т缁€鍡涙煟閹达絽袚闁哄懏鎮傞弻锟犲磼濡　鍋撻弽顐熷亾濮樼偓瀚�
			}	
		}
		else
		{			
			if(xQueueReceive(xQueueMenuValue,&menuPage,0) == pdTRUE)
			{			
				MenuPageTurns(menuPage);				
			}			
			if(xQueueReceive(xQueueBatValue, &batDispValue, 0) == pdTRUE)
			{	
				if(batDispValue!=0)
				{
					if(menuPage==MENU_HOME_PAGE||menuPage==MENU_ONLY_APEX_PAGE)
					{
						disp_batValue(2,32,batDispValue);						
					}
					else if(menuPage==MENU_CHARGING_PAGE)
					{
						pMalloc=pvPortMalloc(256);
						if(pMalloc!=NULL)
						{
							OLED_Display_CHARGING(batDispValue,1);		
							vPortFree(pMalloc);
						}						
					}
				}					
				xSemaphoreGive(xSemaphoreDispRfresh);
			}
		}				
		#ifdef WDT_ENABLE
		xEventGroupSetBits(WDTEventGroup,MENU_TASK_EVENT_BIT);
		#endif
		 vTaskDelay(2);//1ms
	}
}
