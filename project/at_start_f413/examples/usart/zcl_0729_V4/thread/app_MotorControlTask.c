#include "string.h"
#include "stdlib.h"

#include "at32f413_conf.h"
#include "delay.h"

#include "gpio_port.h"
#include "at32f413_board.h"
#include "app_MotorControlTask.h"
#include "control.h"
#include "mp6570.h"
#include "customer_control.h"
#include "para_list.h"

#include "usart_port.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"	
#ifdef WDT_ENABLE
#include "event_groups.h"	
extern EventGroupHandle_t  WDTEventGroup;
#endif
//QueueHandle_t  xQueueMotorControl;	

static uint8_t foc_flag;
static unsigned int motorErrHeatBeat;
 
extern MotorStatus_TypeDef motor_status;
extern MotorSettings_TypeDef motor_settings;

extern union Param_Union  sys_param_un; 
extern union Motor_Para_Union  motor_param_un;

#define USART_RX_CMD            0x05//gc=30,检测到线
#define USART_RX_1CMD           0x06//5~27 gc=18 //进入根管
#define USART_RX_2CMD           0x07//1~4 gc=3 //减速,靠近根尖
#define USART_RX_3CMD           0x08//<=1  gc=1 //到达根尖
#define USART_RX_4CMD           0x09//<=0  gc=0 //穿过根尖

typedef struct   
{
	unsigned char       cmdSource;             //命令来源（按键、串口等）
	unsigned char       motorCmd_Type;         //命令类型
	unsigned char       motorDeviceId;        //电机编号
	unsigned char       controlCode;                //控制指令ID	
	unsigned int        Data;                              //备用数据
}MotorCtl_TypeDef;

/**
  * @brief MOTOR_150US_Callback
  * @param  none
  * @retval none
  */

void MOTOR_150US_Callback(void)//100us目前有点问题，改为150us
{	
	control_loop();	
	motorErrHeatBeat=0;		
}

/**
  * @brief motor default param
  * @param  none
  * @retval none
  */
void MotorParamInit(void)
{	
		unsigned short int useNum=0;
		stop();	
//	motor_settings.mode =EndoModeSpeedForward;// 1; //设置成需要的模式
//	motor_settings.autorev_mode =AutoReverseMode1;// 1; //设置转速模式下自动反转模式
//	motor_settings.forward_position = 30; //设置正转角度
//	motor_settings.reverse_position = -150; // 设置反转角度
//	motor_settings.forward_speed = 50;//500; // 设置速度模式下正转速度
//	motor_settings.reverse_speed = -50;//500;// 设置速度模式下反转速度（用负数）
//	motor_settings.upper_threshold = 4.0;//4.0; // 设置转矩保护点(n.cm)
//	motor_settings.lower_threshold =2.4;// 2.4; // 设置转矩取消保护点60%最大值
//	motor_settings.toggle_mode_speed = 500; //设置往复模式速度
//		MenuMotorParamUpdate(sys_param_un.device_param.use_num,0);
//  MOTOR_SETTING_UPDATE
		if(sys_param_un.device_param.use_p_num<10)
		{
			useNum=sys_param_un.device_param.use_p_num;
		}
		else
		{
			useNum=0;//默认P1
		}
		motor_settings.autorev_mode=AutoReverseMode1;//AutoReverseMode1;
		if(motor_param_un.system_motor_pattern[useNum].dir==EndoModeTorqueATC)
		{
			motor_settings.mode=EndoModeSpeedForward;
		}
		else
		{
			motor_settings.mode=(eEndoMode)motor_param_un.system_motor_pattern[useNum].dir;
		}	
		if(motor_param_un.system_motor_pattern[useNum].motorSpeedNum	>MAX_spd_Rpm_num) motor_param_un.system_motor_pattern[useNum].motorSpeedNum=MAX_spd_Rpm_num;	
		if(motor_param_un.system_motor_pattern[useNum].toggleSpeedNum	>spd600_Rpm_num) motor_param_un.system_motor_pattern[useNum].toggleSpeedNum=spd600_Rpm_num;			
		motor_settings.forward_speed=speed_list[motor_param_un.system_motor_pattern[useNum].motorSpeedNum];
		motor_settings.reverse_speed=-motor_settings.forward_speed;			
		motor_settings.toggle_mode_speed=speed_list[motor_param_un.system_motor_pattern[useNum].toggleSpeedNum];//
		motor_settings.forward_position=motor_param_un.system_motor_pattern[useNum].forwardPosition;
		motor_settings.reverse_position=motor_param_un.system_motor_pattern[useNum].reversePosition;			
		if(motor_param_un.system_motor_pattern[useNum].dir==EndoModeTorqueATC)
		{			
			motor_settings.upper_threshold=torque_list[torque40_Ncm]*0.1f;//torque_list[MAX_torque_42_Ncm]*0.1;
			if(motor_param_un.system_motor_pattern[useNum].atcTorqueThresholdNum>torque20_Ncm)
			{
				motor_settings.lower_threshold=torque_list[torque20_Ncm]*0.1f;
			}
			else
			{			
				motor_settings.lower_threshold=torque_list[motor_param_un.system_motor_pattern[useNum].atcTorqueThresholdNum]*0.10f;
			}			
		}
		else if(motor_param_un.system_motor_pattern[useNum].dir==EndoModePositionToggle)
		{		
		 	motor_settings.upper_threshold=torque_list[motor_param_un.system_motor_pattern[useNum].recTorqueThresholdNum]*0.10f;
			motor_settings.lower_threshold=motor_settings.upper_threshold*0.6f;
		}
		else 
		{					
			motor_settings.upper_threshold=torque_list[motor_param_un.system_motor_pattern[useNum].torqueThresholdNum]*0.10f;			
			motor_settings.lower_threshold=motor_settings.upper_threshold*0.6f;//60%		
		}			
		
		update_settings(&motor_settings);	
}
/**
  * @brief  motor run status monitor
  * @param   
  * @retval none
  */
static void MotorStatusMonitor(unsigned short int perTimeMs)
{	 
	if(perTimeMs==0)
	{
		motorErrHeatBeat=0;
	}	
	if(motorErrHeatBeat>100)//0.5s
	{
		motorErrHeatBeat=0;
		App_MotorControl(MOTOR_SETTING_ERR);//err
//		gpio_bits_reset(GPIOB,GPIO_PINS_14);//...mp6570_disable()	
	}	
  if(status_mp6570 !=04&&foc_flag	!=0)//malfunction
	{		
		motorErrHeatBeat++;
//		App_MotorControl(MOTOR_SETTING_ERR);//意外停止，重启
	}	
}

/**
  * @brief  motor  run in certain mode,指令下发
  * @param  uint8_t run_flag,run mode 
  * @retval none
  */
unsigned char App_MotorControl(unsigned char cmd)
{
	unsigned char err;
	err=0;
	if(cmd>MOTOR_SETTING_ERR) return err;				
	//cmd 0,stop,1 update,2 start	
	switch(cmd)
	{	
		case MOTOR_MODE_STOP:					
			stop();					
			if(foc_flag!=0)
			{
				foc_flag=0;
				MotorStatusMonitor(0);	// status monitor 20ms periodic	
				vTaskDelay(5);//20ms
				tmr_counter_enable(TMR3, FALSE);				
			}									
		#ifdef LED_INDICATE_ENABLE		
			LedFunctionSet( LED_B ,500,LED_T_HIGH_PRIORITY,LED_OFF);
		#endif			
			break;		
		case MOTOR_MODE_START:		
			if(foc_flag==0)		
			{
				MotorStatusMonitor(0);	// status monitor 20ms periodic
				tmr_counter_enable(TMR3, TRUE);//enable loop timer
				vTaskDelay(5);//							
				foc_flag=1;							
			}							
			start();	
			#ifdef LED_INDICATE_ENABLE		
			LedFunctionSet(LED_B ,500,LED_T_HIGH_PRIORITY,LED_KEEP_ON);		
			#endif		
			break;
		case MOTOR_SETTING_UPDATE:		
			update_settings(&motor_settings);	
			break;	
		case MOTOR_MODE_SEARCH_ANGLE:					
			taskENTER_CRITICAL();
//			err=MP6570_AutoThetaBias(0x00, 500,2000); //找初始角度
			err=MP6570_AutoThetaBias(0x00, 300,2000); //找初始角度
			#ifdef DEBUG_RTT
			SEGGER_RTT_printf(0, "err%d\r\n", err);	
			#endif		
			taskEXIT_CRITICAL(); 
			stop();	
			foc_flag=0;
			MotorStatusMonitor(0);	//status monitor 20ms periodic	
			vTaskDelay(5);//20ms
			tmr_counter_enable(TMR3, FALSE);	
			#ifdef LED_INDICATE_ENABLE
			LedFunctionSet( LED_B ,500,LED_T_HIGH_PRIORITY,LED_OFF);
			#endif	
		break;
		case MOTOR_MODE_RESTART:
			stop();					
			if(foc_flag!=0)
			{
				foc_flag=0;
				MotorStatusMonitor(0);	// status monitor 20ms periodic	
				vTaskDelay(5);//20ms
				tmr_counter_enable(TMR3, FALSE);				
			}		
			update_settings(&motor_settings); 			
			MotorStatusMonitor(0);	// status monitor 20ms periodic
			tmr_counter_enable(TMR3, TRUE);//enable loop timer
			vTaskDelay(5);//							
			foc_flag=1;				
			start();
		break;			
		case MOTOR_SETTING_ERR:		
			stop();//err handle	
			foc_flag=0;
			MotorStatusMonitor(0);	// status monitor 20ms periodic	
			vTaskDelay(5);//20ms
//			tmr_counter_enable(TMR3, FALSE);	
			MotorDeviceReset();
			MotorParamInit();	
		#ifdef DEBUG_RTT
			SEGGER_RTT_printf(0,"motor control err ID%d\r\n", motorErrHeatBeat);	
		#endif		
			break;		
		default:
			stop();
			foc_flag=0;
		break;
	}
	return err;	
}
/*=======================motor control task===================================
	* @brief:  motor control task.
	* @param:  None
	* @retval: None
==============================================================================*/
void vAppMotorControlTask( void * pvParameters )
{	
	unsigned int count;
	for(;;)	
	{	
		count++;
		MotorStatusMonitor(5);	// status monitor 20ms periodic	
//		update_settings(&motor_settings);			
		#ifdef WDT_ENABLE
		xEventGroupSetBits(WDTEventGroup,MOTOR_CONTROL_TASK_EVENT_BIT);
		#endif
		vTaskDelay(5);//5ms
	}
}
