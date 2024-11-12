#ifndef __PARA_LIST
#define __PARA_LIST
#include "limits.h"
#include "stdint.h"
#include "at32f413.h"

#define left_hand  0
#define right_hand 1

extern short int speed_list[20];
extern unsigned short int torque_list[21];

typedef struct Param
{
	uint16_t       W_flag;                    //写入版本标志位 年月日  
	uint16_t       use_p_num;                   //当前使用的记忆模式编号 num 0-9 （1-10）
	uint16_t       use_hand;                  //左右手切换 0 left hand 1 right hand	
	uint16_t       motor_max_current;         //电机允许的最大电流 单位mA
	uint16_t       highLevelConfigurationEnable;//高端配置用于后台升级高级功能	
	uint16_t       apical_action_flag;        //到达根尖电机行动 0 不管 1 反转 2停止
	uint16_t       auto_start_flag;           //根测控制电机自动启动 0 OFF 1 0N
	uint16_t       auto_stop_flag;            //根测控制电机自动停止 0 OFF 1 0N
	uint16_t        empty_rate;              
	uint16_t       ref_tine;                  //预设根尖点位置 ,根尖偏移量，工程调试 	
	uint16_t       apexFunctionLoad;          //根测线插入，加载根测功能	
	uint16_t        gc_ref_rate;  //根尖参考比例值
	uint16_t        apex_tine_400Value; //根尖参考值
}Device_Param_Def;  

union  Param_Union//所有参数均按照16位定义
{
  Device_Param_Def device_param;
  uint16_t para_buff[13];
};

typedef struct {
	unsigned short int pNum;	//apexEnable;//没用,后期再增加
	unsigned short int dir;
	unsigned short int motorSpeedNum; 
	unsigned short int torqueThresholdNum;		//torgue_list num ,iq upper threshold  ,((unit mN.m )=10*(unit N.cm) ) use mN.m
	unsigned short int atcTorqueThresholdNum;
	short int	forwardPosition;
	short int reversePosition;
	unsigned short int recTorqueThresholdNum;
	unsigned short int toggleSpeedNum;//rec spd	
	
}SYSTEM_MOTOR_PARAM;

union  Motor_Para_Union//所有参数均按照16位定义 10个记忆模式
{
	SYSTEM_MOTOR_PARAM  system_motor_pattern[11];//10个记忆模式,第十一组作为缓存
  uint16_t pattern_buff[99];
};
typedef enum { 
			spd100_Rpm_num=0,
			spd150_Rpm_num,//1
			spd200_Rpm_num,
			spd250_Rpm_num,
			spd300_Rpm_num,
			spd350_Rpm_num,
			spd400_Rpm_num,
			spd450_Rpm_num,
			spd500_Rpm_num,
			spd550_Rpm_num,
			spd600_Rpm_num,
			spd800_Rpm_num,
//			MAX_spd_Rpm_num,//max=1000rpm;
			spd1000_Rpm_num,			
			spd1200_Rpm_num,	
			spd1500_Rpm_num,	
			spd1800_Rpm_num,
			spd2000_Rpm_num,
			MAX_spd_Rpm_num,//max=2200rpm;				
			spd2200_Rpm_num,
			spd2500_Rpm_num,					
	}speed_list_num_enum; 

typedef enum { 
			torque06_Ncm=0,//6mN.m=0.6N.cm;
			torque08_Ncm,//1
			torque10_Ncm,
			torque12_Ncm,
			torque14_Ncm,
			torque16_Ncm,
			torque18_Ncm,
			torque20_Ncm,
			torque22_Ncm,
			torque24_Ncm,
			torque26_Ncm,
			torque28_Ncm,
			torque30_Ncm,
			torque32_Ncm,
			torque35_Ncm,
			torque40_Ncm,
			//	MAX_torque_42_Ncm,//13	
			torque42_Ncm,
			torque45_Ncm,
		    MAX_torque_42_Ncm,//13
			torque50_Ncm,
			torque55_Ncm,
			torque60_Ncm,
			torque65_Ncm,
			torque70_Ncm,
	}torque_list_num_enum; 

typedef enum { 
			null_signal=0,
			pwr_switch_irq_signal=1,//1
			s_irq_signal,
			add_irq_signal,
			sub_irq_signal,

			run_button_press_signal,//5
			run_button_long_press_signal,		

			add_button_press_signal,//7
			add_button_long_press_signal,

			sub_button_press_signal,//9
			sub_button_long_press_signal,

			s_button_press_signal,//11
			s_button_long_press_signal,

			charger_insert,//13
			charger_pull_out_signal,
			stop_chargering_signal,
								 
			low_power_signal,//16
			power_off_signal, //17
			
			motor_setting_updata_signal,//电机配置更新信号		
			run_button_release_signal,
			motor_apex_run_signal,//根测启动电机
			motor_apex_stop_signal///根测停止电机
//		start_MP6570_AutoThetaBias_signal///nothing
	}task_notify_enum; 
 
typedef enum { 		
		SETTING_FOR=0,//1
		AUTO_START,// 2 
		AUTO_STOP,// 3 	
		APICAL_ACTION,//4 
		APICAL_OFFESET,//5
		CW_ANGLE,// 6
		CCW_ANGLE, //7
	//	BLOOTEETH,
		HANDEDNESS,//8
		AUTO_CALIBRATION,//9
		RESTORE_DEFAULT_SETTING,//10
		MAX_SUBMENU,//10

 }SCREEN_SETTING_ENUM;
 typedef enum { 		
		MENU_LOGO_PAGE=0,
		MENU_HOME_PAGE,//1
		MENU_MOTOR_WORK_PAGE,//2
		MENU_SYSTEM_SET_PAGE,//3
		MENU_ONLY_APEX_PAGE,//4
		MENU_APEX_AND_MOTOR_PAGE,//5
		MENU_CHARGING_PAGE,//6
//		MENU_ERROR_MANAGE_PAGE,//7
//		MENU_POWER_OFF_PAGE,//8
		MENU_MAX_PAGE_NUM,
 }MENU_ENUM;
 
	extern union Param_Union  sys_param_un; 
	extern union Motor_Para_Union  motor_param_un;
 
#endif
