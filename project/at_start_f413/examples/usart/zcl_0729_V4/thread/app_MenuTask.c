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
#define MAX_MOTOR_SPEED   2200//2000//闂傚倸鍊搁崐宄懊归崶褉鏋栭柡鍥ュ灩缁愭骞栧ǎ顒€鈧垶绂嶈ぐ鎺撶厪濠电偟鍋撳▍鍡涙煕鐎ｎ偆澧甸柟顔筋殔閳藉鈻嶉褌閭い銏℃瀹曞ジ寮撮悢鍙夊闂佽崵濮垫禍浠嬪礉瀹€鍕嚑闁硅揪闄勯悡娑㈡倵閿濆簼鎲鹃柣鎾冲悑椤ㄣ儵鎮欓懠顒傤啋闂佽桨鐒﹂幑鍥极閹剧粯鏅搁柨鐕傛嫹(max 2450*6=14700)
#define MAX_TORQUE_UUPER_THRESHOLD   50//42//闂傚倸鍊搁崐宄懊归崶褉鏋栭柡鍥ュ灩缁愭骞栧ǎ顒€鈧垶绂嶈ぐ鎺撶厪濠电偟鍋撳▍鍡涙煕鐎ｎ偆澧甸柟顔筋殔閳藉鈻嶉褌閭い銏℃瀹曞ジ寮撮悢鍙夊闂佽崵濮垫禍浠嬪礉瀹€鍕嚑闁硅揪闄勯悡娑㈡倵閿濆簼鎲鹃柣鎾冲悑椤ㄣ儵鎮欓懠顒傤啋闂佽桨鐒﹂幑鍥极閹剧粯鏅搁柨鐕傛嫹(46.281g.cm/A~~0.454N.cm/A)(18.41mN.m or 1.841N.cm  1g.cm=0.00981N.cm=0.0981mN.m=0.0000981N.m)(闂傚倸鍊搁崐宄懊归崶褉鏋栭柡鍥ュ灩缁愭鏌熼悧鍫熺凡闁告垹濮撮埞鎴︽偐鐎圭姴顥濈紓浣哄閸ㄥ爼寮婚敐澶娢ч幖杈剧磿娴狀厾绱撴担椋庤窗闁革綇绲介～蹇涙惞鐟欏嫬纾梺闈浨归崕鏌ユ倵妤ｅ啯鈷戦柛婵嗗閸庡繑銇勯鐘插幋闁绘侗鍠涚粻娑樷槈濞嗗繆鍋撴繝姘厾闁诡厽甯掗崝婊勭箾閸涱偄鐏叉慨濠冩そ瀹曨偊宕熼鐔蜂壕闁割偅娲栫壕褰掓煛瀹ュ骸浜愰柛瀣尭椤繈鎮欓鈧锟�4.0A闂傚倸鍊搁崐宄懊归崶褉鏋栭柡鍥ュ灩缁愭鏌熼悧鍫熺凡闁告垹濮撮埞鎴︽偐鐎圭姴顥濈紓浣哄閸ㄨ泛顫忓ú顏勫瀭妞ゆ洖鎳庨崜鏉款渻閵堝啫鍔氭い锕傛涧椤繘鎼圭憴鍕／闂侀潧枪閸庢煡鎮楁ィ鍐┾拺闁告繂瀚烽崕蹇斻亜椤撶姴鍘撮柣娑卞枦缁犳稑鈽夊▎鎰仧闂備浇娉曢崳锕傚箯閿燂拷187.71 g.cm=1.841N.cm=18.41mN.m=0.1841N.m)
#define HALF_MAX_TORQUE_UUPER_THRESHOLD   MAX_TORQUE_UUPER_THRESHOLD/2
#define MINIMUM_ROTOR_ANGLE          10//minimum routor angle in  " motor_settings.mode=EndoModePositionToggle "

//unsigned short int torque_list[16]={5,8,10,12,15,18, 20,22,25,30,35,40,42,50,55,60};//unit mN.m
  unsigned short int torque_list[21]={6,8,10,12,14,16,18, 20,22,24,26,28,30,32,35,40,42,45,50,55,60};//unit mN.m
short int speed_list[20]={100,150,200,250,300,350,400,450,500,550,600,800,1000,1200,1500,1800,2000,2200,2500};	
	
//	short int speed_list[20]={100,150,200,250,300,350,400,450,500,550,600,800,1000,1200,1500,1800,MAX_MOTOR_SPEED,2200,2500};	
	//7V
//unsigned short int torque_limit[22]={torque70_Ncm ,torque70_Ncm ,torque70_Ncm ,torque70_Ncm ,torque70_Ncm ,torque70_Ncm ,torque70_Ncm ,torque65_Ncm ,\
//	torque65_Ncm ,torque65_Ncm ,torque60_Ncm ,torque50_Ncm ,torque45_Ncm  ,torque35_Ncm  ,torque20_Ncm  ,torque05_Ncm  ,torque05_Ncm  ,torque05_Ncm,torque05_Ncm,torque05_Ncm};//unit mN.m闂傚倸鍊搁崐宄懊归崶褉鏋栭柡鍥ュ灩缁愭鏌熼悧鍫熺凡闁告垹濮撮埞鎴︽偐鐎圭姴顥濈紓浣哄閸ㄥ爼寮婚敐澶婄闁挎繂鎲涢幘缁樼厸闁告侗鍠楅崐鎰版煛瀹€瀣瘈鐎规洘甯掕灒閻炴稈鈧厖澹曢梺鍝勭▉閸嬧偓闁稿鎸搁～婵嬵敆閸屾簽銊╂倵鐟欏嫭绀冪紒顔肩焸椤㈡ɑ绺界粙鍧楀敹闂佺粯姊规禍浠嬪磻閺嶎厽鈷掗柛灞剧懄缁佺増绻涙径瀣鐎规洑鍗抽獮鍥级閸喖娈ゅ┑鐘垫暩婵敻鎳濋崜褏涓嶆い鏍仦閻撶喖鏌熸潏鍓у埌鐞氭岸姊洪崫鍕櫤闁诡喖鍊垮濠氭晸閻樿尙锛滃┑鈽嗗灥閵嗏偓闁稿鎹囧浠嬵敃閻旇渹澹曞┑顔斤供閸撴稓绮斿ú顏呯厸閻忕偠顕ф慨鍌炴煙椤斿搫鐏茬€规洏鍔嶇换婵嬪礋椤忓棛锛熼梻鍌氬€风粈浣虹礊婵犲泚澶愬箻鐠囪尙顦у┑顔姐仜閸嬫挾鈧鍣崑濠囩嵁濡偐纾兼俊顖滃劋閻ｇ粯绻濆▓鍨灍閼垦囨煕鎼淬劋鎲鹃柛鈹惧亾濡炪倖鍩堥崜娆徝洪妶鍥╀笉妞ゆ牜鍋為悡鐔兼煙鏉堝墽鍒扮悮姘舵⒑閸濆嫭鍣洪柟顔煎€垮濠氬Ω閳哄倸浜為梺绋挎湰缁嬫垿顢旈敓锟�
//8V
unsigned short int torque_limit[22]={torque70_Ncm ,torque70_Ncm ,torque70_Ncm ,torque70_Ncm ,torque70_Ncm ,torque70_Ncm ,torque70_Ncm ,torque65_Ncm ,\
	torque65_Ncm ,torque65_Ncm ,torque60_Ncm ,torque60_Ncm ,torque60_Ncm  ,torque50_Ncm  ,torque35_Ncm  ,torque20_Ncm  ,torque08_Ncm  ,torque06_Ncm,torque06_Ncm,torque06_Ncm};//unit mN.m闂傚倸鍊搁崐宄懊归崶褉鏋栭柡鍥ュ灩缁愭鏌熼悧鍫熺凡闁告垹濮撮埞鎴︽偐鐎圭姴顥濈紓浣哄閸ㄥ爼寮婚敐澶婄闁挎繂鎲涢幘缁樼厸闁告侗鍠楅崐鎰版煛瀹€瀣瘈鐎规洘甯掕灒閻炴稈鈧厖澹曢梺鍝勭▉閸嬧偓闁稿鎸搁～婵嬵敆閸屾簽銊╂倵鐟欏嫭绀冪紒顔肩焸椤㈡ɑ绺界粙鍧楀敹闂佺粯姊规禍浠嬪磻閺嶎厽鈷掗柛灞剧懄缁佺増绻涙径瀣鐎规洑鍗抽獮鍥级閸喖娈ゅ┑鐘垫暩婵敻鎳濋崜褏涓嶆い鏍仦閻撶喖鏌熸潏鍓у埌鐞氭岸姊洪崫鍕櫤闁诡喖鍊垮濠氭晸閻樿尙锛滃┑鈽嗗灥閵嗏偓闁稿鎹囧浠嬵敃閻旇渹澹曞┑顔斤供閸撴稓绮斿ú顏呯厸閻忕偠顕ф慨鍌炴煙椤斿搫鐏茬€规洏鍔嶇换婵嬪礋椤忓棛锛熼梻鍌氬€风粈浣虹礊婵犲泚澶愬箻鐠囪尙顦у┑顔姐仜閸嬫挾鈧鍣崑濠囩嵁濡偐纾兼俊顖滃劋閻ｇ粯绻濆▓鍨灍閼垦囨煕鎼淬劋鎲鹃柛鈹惧亾濡炪倖鍩堥崜娆徝洪妶鍥╀笉妞ゆ牜鍋為悡鐔兼煙鏉堝墽鍒扮悮姘舵⒑閸濆嫭鍣洪柟顔煎€垮濠氬Ω閳哄倸浜為梺绋挎湰缁嬫垿顢旈敓锟�
	
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
//			{//闂傚倸鍊搁崐宄懊归崶褉鏋栭柡鍥ュ灩缁愭鏌″搴″箹閸ユ挳姊虹紒妯虹仸閽冭京绱撳鍛枠闁哄矉缍侀獮鍥濞戞﹩娼绘俊鐐€曠换鎰版偉閻撳寒娼栭柧蹇氼潐瀹曞鏌曟繛鍨姕闁诲酣绠栧铏瑰寲閺囩喐鐝旈梺鐟板暱缁绘帡鍩€椤掑倹鏆╅柛妯犲洤鐓濋幖娣妼缁犳稒銇勯幒鎴Х缂佹唻绠撳濠氬磼濞嗘劗銈板┑鐐差槹濞茬喎鐣锋导鏉戠疀闁哄鐏濋崵鎴炵節閻㈤潧校闁肩懓澧界划濠氼敍閻愬鍘介梺纭呮彧缁插€燁暱闂備礁鎼幊蹇涘箖閸岀偛钃熼柕濞炬櫆閸嬪棝鏌涚仦鍓р槈妞ゅ骏鎷�
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
			motor_settings.mode =(enum EndoMode) motor_param_un.system_motor_pattern[programNum].dir;	
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
			motor_settings.lower_threshold=torque_list[motor_param_un.system_motor_pattern[programNum].atcTorqueThresholdNum]*0.10;//torque_list[motor_param_un.system_motor_pattern[programNum].torqueThresholdNum]*0.10;//闂傚倸鍊峰ù鍥綖婢舵劦鏁婇柡宥庡幖閻鏌涢幇闈涙灈闁哄绶氶弻鐔煎箲閹伴潧娈┑鈽嗗亝閿曘垽寮婚弴銏犻唶婵犻潧娴傚Λ鐐差渻閵堝懐绠伴柣鏍с偢瀵鍨鹃幇浣告倯婵炶揪绲捐ぐ鍐╃妤ｅ啯鐓曢柟閭﹀墮缁狙囨煟濠垫劕鐏︽慨濠勭帛閹峰懘鎳為妷褋鈧﹪姊洪崫銉バｉ柛鏃€鐟ラ锝夊川婵犲嫮鐦堝┑顔斤供閸撴盯顢欓崱娑欌拺缂備焦锚閻忓崬鈹戦鍝勨偓婵嗩嚕閺屻儱绠瑰ù锝呮贡閸樻悂姊虹粙鎸庢拱闁挎岸鏌嶈閸撴岸鎮у⿰鍫濈劦妞ゆ帊娴囨竟妯肩磽瀹ュ拑韬€殿喖顭锋俊鎼佸煛閸屾矮绨婚梻浣告啞缁嬫垿鏁冮敂钘夘嚤闁告洦鍨遍埛鎴犳喐閻楀牆淇俊顐ｅ灴閺屾稖绠涢弮鎾光偓璺ㄢ偓娈垮櫘閸嬪﹪鐛Ο鍏煎珰闁告瑥顦藉Λ鐔兼⒒娓氣偓濞佳囨偋閸℃あ娑樜旈崪浣规櫆闂佽偐枪閸氣偓缂佽妫欓妵鍕冀閵婏絼绮堕梺绋款儐閹告悂鎮鹃悜钘夌倞闁冲搫锕ラ弫銈嗙節閻㈤潧浠╅柟娲讳簽瀵板﹪鎸婃径妯煎姺闂佽法鍠撴慨鎾几娓氣偓閺岀喖骞戦幇闈涙濠碘槅鍋呴敃銏ゅ蓟閺囥垹閱囨繝闈涙祩濡偛顪冮妶鍛寸崪闁瑰嚖鎷� int  param=5闂傚倸鍊搁崐宄懊归崶褉鏋栭柡鍥ュ灩缁愭鏌熼悧鍫熺凡闁告垹濮撮埞鎴︽偐鐎圭姴顥濈紓浣哄閸ㄥ爼寮婚妸鈺傚亞闁稿本绋戦锟�5闂傚倸鍊烽懗鑸电仚濡炪倖鍨甸幊鎰板箲閵忕媭娼ㄩ柍褜鍓欓锝夘敃閿曗偓缁犲鎮归崶褍绾фい銉︾箞濮婃椽妫冨☉姘暫濠碘槅鍋呴〃濠囥€侀弮鍫熸櫢闁跨噦鎷�5 闂傚倸鍊搁崐宄懊归崶褉鏋栭柡鍥ュ灩缁愭鏌熼悧鍫熺凡闁告垹濮撮埞鎴︽偐鐎圭姴顥濈紓浣哄閸ㄥ爼寮婚妸鈺傚亞闁稿本绋戦锟�-5 闂傚倸鍊烽懗鑸电仚濡炪倖鍨甸幊鎰板箲閵忕媭娼ㄩ柍褜鍓欓锝夘敃閿曗偓缁犲鎮归崶褍绾фい銉︾箞濮婃椽妫冨☉姘暫濠碘槅鍋呴〃濠囥€侀弮鍫熸櫢闁跨噦鎷�5闂傚倸鍊搁崐宄懊归崶褉鏋栭柡鍥ュ灩缁愭鏌熼悧鍫熺凡闁告垹濮撮埞鎴︽偐鐎圭姴顥濈紓浣哄閸ㄥ爼寮婚妸鈺傚亞闁稿本绋戦锟�
			if(motor_param_un.system_motor_pattern[programNum].atcTorqueThresholdNum>torque20_Ncm)
			{
				motor_settings.lower_threshold=torque_list[torque20_Ncm]*0.10;
			}
		}
		else if(motor_param_un.system_motor_pattern[programNum].dir==EndoModePositionToggle)
		{
			motor_settings.upper_threshold=MAX_TORQUE_UUPER_THRESHOLD/10;//torque_list[motor_param_un.system_motor_pattern[programNum].recTorqueThresholdNum]*0.10;//MAX_TORQUE_UUPER_THRESHOLD*0.10;//motor_param_un.system_motor_pattern[programNum].torqueThreshold*0.16;
			motor_settings.lower_threshold=MAX_TORQUE_UUPER_THRESHOLD*3/5;//torque_list[motor_param_un.system_motor_pattern[programNum].recTorqueThresholdNum]*0.06;//MAX_TORQUE_UUPER_THRESHOLD*0.06;//闂傚倸鍊峰ù鍥綖婢舵劦鏁婇柡宥庡幖閻鏌涢幇闈涙灈闁哄绶氶弻鐔煎箲閹伴潧娈┑鈽嗗亝閿曘垽寮婚弴銏犻唶婵犻潧娴傚Λ鐐差渻閵堝懐绠伴柣鏍с偢瀵鍨鹃幇浣告倯婵炶揪绲捐ぐ鍐╃妤ｅ啯鐓曢柟閭﹀墮缁狙囨煟濠垫劕鐏︽慨濠勭帛閹峰懘鎳為妷褋鈧﹪姊洪崫銉バｉ柛鏃€鐟ラ锝夊川婵犲嫮鐦堝┑顔斤供閸撴盯顢欓崱娑欌拺缂備焦锚閻忓崬鈹戦鍝勨偓婵嗩嚕閺屻儱绠瑰ù锝呮贡閸樻悂姊虹粙鎸庢拱闁挎岸鏌嶈閸撴岸鎮у⿰鍫濈劦妞ゆ帊娴囨竟妯肩磽瀹ュ拑韬€殿喖顭锋俊鎼佸煛閸屾矮绨婚梻浣告啞缁嬫垿鏁冮敂钘夘嚤闁告洦鍨遍埛鎴犳喐閻楀牆淇俊顐ｅ灴閺屾稖绠涢弮鎾光偓璺ㄢ偓娈垮櫘閸嬪﹪鐛Ο鍏煎珰闁告瑥顦藉Λ鐔兼⒒娓氣偓濞佳囨偋閸℃あ娑樜旈崪浣规櫆闂佽偐枪閸氣偓缂佽妫欓妵鍕冀閵婏絼绮堕梺绋款儐閹告悂鎮鹃悜钘夌倞闁冲搫锕ラ弫銈嗙節閻㈤潧浠╅柟娲讳簽瀵板﹪鎸婃径妯煎姺闂佽法鍠撴慨鎾几娓氣偓閺岀喖骞戦幇闈涙濠碘槅鍋呴敃銏ゅ蓟閺囥垹閱囨繝闈涙祩濡偛顪冮妶鍛寸崪闁瑰嚖鎷� int  par
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
			OLED_ShowString(32,0,"    ",16,1);//闂傚倸鍊搁崐宄懊归崶褉鏋栭柡鍥ュ灩缁愭鏌熼悧鍫熺凡闁告垹濮撮埞鎴︽偐鐎圭姴顥濈紓浣哄閸ㄥ爼寮婚敐澶婄闁挎繂鎲涢幘缁樼厸闁告侗鍠楅崐鎰版煛瀹€瀣瘈鐎规洘甯掕灒閻炴稈鈧厖澹曢梺鍝勭▉閸嬧偓闁稿鎸搁～婵嬵敆閸屾簽銊╂⒑閸濆嫯顫﹂柛鏃€鍨块獮鍐閵堝懎绐涙繝鐢靛Т鐎氼亞妲愰弴銏♀拻濞达絽鎽滅粔鐑樸亜閵夛附宕岀€规洘顨呴～婊堝焵椤掆偓椤曪綁顢曢敃鈧粻鑽ょ磽娴ｈ偂鎴濃枍閵忋倖鈷戦悹鎭掑妼濞呮劙鏌熼崙銈嗗
			OLED_ShowChar(64,0,'P',16,1);	
			OLED_ShowNum(72,0,programNum-5,1,16,1);//闂傚倸鍊风粈渚€骞栭銈傚亾濮樼厧澧柡鍛板煐缁傛帞鈧綆鈧叏闄勯幈銊モ攽閹炬潙顏�1-P4
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
//				if(torqueNum>torqueReference)//闂傚倸鍊搁崐宄懊归崶褉鏋栭柡鍥ュ灩缁愭鏌￠崶鈺佹灁缂佺娀绠栭弻娑㈠Ψ椤旂厧顫╃紓浣插亾鐎光偓閸曨剛鍘撻梺瀹犳〃缁€渚€寮冲▎鎰╀簻闊洦鎸鹃崺锝嗘叏婵犲懏顏犻柟鐟板婵℃悂濡烽敂閿亾妤ｅ啯鈷戦柛婵嗗閸庡繑銇勯鐘插幋闁绘侗鍠涚粻娑㈠籍閹寸倫褔鏌熼悡搴ｆ憼缂佸鍔楅崚鎺楁晸閿燂拷
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
	unsigned char i,j,x,y,m;	
	if(startFlag==0)//preparation work
	{
		recTimeMs=systemTimeMs;
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
				recApexValue=33;
				temp=0;
			}
			else temp=apexValue;					
			// over area (40~60)
			if(recApexValue!=temp)
			{
				if(temp>=30) recApexValue=30;
				if(temp==0) recApexValue=temp;
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
				// oter area  闂傚倸鍊搁崐宄懊归崶褉鏋栭柡鍥ュ灩缁愭鏌熼悧鍫熺凡闁告垹濮撮埞鎴︽偐鐎圭姴顥濈紓浣哄閸ㄥ爼寮婚妸鈺傚亞闁稿本绋戦锟�100~160)
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
					if(i+7==sys_param_un.device_param.ref_tine)//闂傚倸鍊搁崐宄懊归崶褉鏋栭柡鍥ュ灩缁愭鏌￠崶鈺佹灁缂佺娀绠栭弻娑㈠Ψ椤旂厧顫╃紓浣插亾鐎光偓閸曨剛鍘撻梺瀹犳〃缁€渚€寮冲▎鎰╀簻闊洦鎸鹃崺锝嗘叏婵犲懏顏犻柟鐟板婵℃悂濡烽敂閿亾妤ｅ啯鈷戦柛婵嗗閸庡繑銇勯鐘插幋闁绘侗鍠涚粻娑樷槈濞嗘劖顏熼梻浣芥硶閸ｏ箓骞忛敓锟�
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
			if(apexValue<0)//闂傚倸鍊搁崐宄懊归崶褉鏋栭柡鍥ュ灩缁愭鏌熼悧鍫熺凡闁告垹濮撮埞鎴︽偐鐎圭姴顥濈紓浣哄閸ㄥ爼寮婚敐澶娢ч幖杈剧磿娴狀垶姊虹紒妯荤叆闁硅櫕锚椤繘鎼圭憴鍕／闂侀潧枪閸庢煡鎮楁ィ鍐┾拺闁告繂瀚烽崕蹇斻亜椤撶姴鍘撮柣娑卞枦缁犳稑鈽夊▎蹇婂亾婵犳碍鐓犻柟顓熷笒閸旀粍绻涢崨顐㈢伈婵﹨娅ｉ幑鍕Ω閵夛妇褰氶梻浣烘嚀閸ゆ牠骞忛敓锟�
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
				if(recApexValue<26&&recApexValue!=sys_param_un.device_param.ref_tine+3)
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
	static unsigned int recTimeMs;
	static unsigned short recApexValue,temp;
	unsigned char i,x,y,m;	
	if(startFlag==0)//preparation work
	{
		recTimeMs=systemTimeMs;
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
				if(temp==0) recApexValue=temp;//闂傚倸鍊烽懗鑸电仚缂備浇顕ч悧鎾崇暦濠靛绠ｉ柨鏇楀亾闁哄绶氶弻鐔煎箲閹伴潧娈┑鈽嗗亝閿曘垽寮婚弴銏犻唶婵犻潧娴傚Λ鐐差渻閵堝懘鐛滈柟鍑ゆ嫹
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
					if(i==sys_param_un.device_param.ref_tine)//闂傚倸鍊搁崐宄懊归崶褉鏋栭柡鍥ュ灩缁愭鏌￠崶鈺佹灁缂佺娀绠栭弻娑㈠Ψ椤旂厧顫╃紓浣插亾鐎光偓閸曨剛鍘撻梺瀹犳〃缁€渚€寮冲▎鎰╀簻闊洦鎸鹃崺锝嗘叏婵犲懏顏犻柟鐟板婵℃悂濡烽敂閿亾妤ｅ啯鈷戦柛婵嗗閸庡繑銇勯鐘插幋闁绘侗鍠涚粻娑樷槈濞嗘劖顏熼梻浣芥硶閸ｏ箓骞忛敓锟�
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
				// oter area  
				x=94-16;//96;
				y=16;
				m=x;
				for(i=0;i<15;i++)
				{
					x=m+i*4;
	//				disp_DrawColumn(x,y,20,0);
					if(i+7==sys_param_un.device_param.ref_tine)
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
				xSemaphoreGive(xSemaphoreDispRfresh);
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
//bat area  
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
		//闂傚倸鍊搁崐宄懊归崶褉鏋栭柡鍥ュ灩缁愭鏌熼悧鍫熺凡闁告垹濮撮埞鎴︽偐鐎圭姴顥濈紓浣哄閸ㄥ爼寮婚敐澶婄闁挎繂鎲涢幘缁樼厸闁告侗鍠楅崐鎰版煙椤旂厧妲婚柍璇查叄楠炲洭顢楅崒婊勫櫙闂傚倷鐒﹂幃鍫曞磹閺嶎厽鍋嬮柣妯垮吹瀹撲線鐓崶銊р姇闁稿﹤顭烽弻銈夊箒閹烘垵濮曞┑鐐叉噷閸ㄨ棄顫忓ú顏勪紶闁告洦鍋€閸嬫捇宕奸弴鐐碉紮闂佸搫绋侀崑鈧柛瀣尭椤繈鎮欓鈧锟�
//		sys_param_un.device_param.ref_tine=(sys_param_un.device_param.ref_tine%7);//offset	max=3	
//		disp_DrawRow(x+1+sys_param_un.device_param.ref_tine*6,60,1,1);//闂傚倸鍊搁崐宄懊归崶褉鏋栭柡鍥ュ灩缁愭鏌熼悧鍫熺凡闁告垹濮撮埞鎴︽偐鐎圭姴顥濈紓浣哄閸ㄥ爼寮婚敐澶婄闁挎繂鎲涢幘缁樼厸闁告侗鍠楅崐鎰版煙椤旂厧妲婚柍璇查叄楠炲洭顢楅崒婊勫櫙闂傚倷鐒﹂幃鍫曞磹閺嶎厽鍋嬮柣妯垮吹瀹撲線鐓崶銊р姇闁稿﹤顭烽弻銈夊箒閹烘垵濮曞┑鐐叉噷閸ㄨ棄顫忓ú顏勪紶闁告洦鍋€閸嬫捇宕奸弴鐐碉紮闂佸搫绋侀崑鈧柛瀣尭椤繈鎮欓鈧锟�
//		disp_DrawRow(x+sys_param_un.device_param.ref_tine*6,61,3,1);
//		disp_DrawRow(x-1+sys_param_un.device_param.ref_tine*6,62,5,1);			
		sys_param_un.device_param.ref_tine=(sys_param_un.device_param.ref_tine%13);//offset	max=3	
//		
//		disp_DrawRow(x-1,60,59,0);//闂傚倸鍊搁崐宄懊归崶褉鏋栭柡鍥ュ灩缁愭鏌熼悧鍫熺凡闁告垹濮撮埞鎴︽偐鐎圭姴顥濈紓浣哄閸ㄥ爼寮婚敐澶婄闁挎繂鎲涢幘缁樼厸闁告侗鍠楅崐鎰版煙椤旂厧妲婚柍璇查叄楠炲洭顢楅崒婊勫櫙闂傚倷鐒﹂幃鍫曞磹閺嶎厽鍋嬮柣妯垮吹瀹撲線鐓崶銊р姇闁稿﹤顭烽弻銈夊箒閹烘垵濮曞┑鐐叉噷閸ㄨ棄顫忓ú顏勪紶闁告洦鍋€閸嬫捇宕奸弴鐐碉紮闂佸搫绋侀崑鈧柛瀣尭椤繈鎮欓鈧锟�
//		disp_DrawRow(x-1,61,59,0);
//		disp_DrawRow(x-1,62,59,0);	
		
		disp_DrawRow(x+1+3*6,60,1,1);//闂傚倸鍊搁崐宄懊归崶褉鏋栭柡鍥ュ灩缁愭鏌熼悧鍫熺凡闁告垹濮撮埞鎴︽偐鐎圭姴顥濈紓浣哄閸ㄥ爼寮婚敐澶婄闁挎繂鎲涢幘缁樼厸闁告侗鍠楅崐鎰版煙椤旂厧妲婚柍璇查叄楠炲洭顢楅崒婊勫櫙闂傚倷鐒﹂幃鍫曞磹閺嶎厽鍋嬮柣妯垮吹瀹撲線鐓崶銊р姇闁稿﹤顭烽弻銈夊箒閹烘垵濮曞┑鐐叉噷閸ㄨ棄顫忓ú顏勪紶闁告洦鍋€閸嬫捇宕奸弴鐐碉紮闂佸搫绋侀崑鈧柛瀣尭椤繈鎮欓鈧锟�
		disp_DrawRow(x+3*6,61,3,1);
		disp_DrawRow(x-1+3*6,62,5,1);		
	// other area 闂傚倸鍊搁崐宄懊归崶褉鏋栭柡鍥ュ灩缁愭鏌熼悧鍫熺凡闁告垹濮撮埞鎴︽偐鐎圭姴顥濈紓浣哄閸ㄥ爼寮婚妸鈺傚亞闁稿本绋戦锟�100~160闂傚倸鍊搁崐宄懊归崶褉鏋栭柡鍥ュ灩缁愭鏌熼悧鍫熺凡闁告垹濮撮埞鎴︽偐鐎圭姴顥濈紓浣哄閸ㄥ爼寮婚妸鈺傚亞闁稿本绋戦锟�
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
//bat area  闂傚倸鍊搁崐宄懊归崶褉鏋栭柡鍥ュ灩缁愭鏌熼悧鍫熺凡闁告垹濮撮埞鎴︽偐鐎圭姴顥濈紓浣哄閸ㄥ爼寮婚妸鈺傚亞闁稿本绋戦锟�0~40闂傚倸鍊搁崐宄懊归崶褉鏋栭柡鍥ュ灩缁愭鏌熼悧鍫熺凡闁告垹濮撮埞鎴︽偐鐎圭姴顥濈紓浣哄閸ㄥ爼寮婚妸鈺傚亞闁稿本绋戦锟�
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
		//闂傚倸鍊搁崐宄懊归崶褉鏋栭柡鍥ュ灩缁愭鏌熼悧鍫熺凡闁告垹濮撮埞鎴︽偐鐎圭姴顥濈紓浣哄閸ㄥ爼寮婚敐澶婄闁挎繂鎲涢幘缁樼厸闁告侗鍠楅崐鎰版煙椤旂厧妲婚柍璇查叄楠炲洭顢楅崒婊勫櫙闂傚倷鐒﹂幃鍫曞磹閺嶎厽鍋嬮柣妯垮吹瀹撲線鐓崶銊р姇闁稿﹤顭烽弻銈夊箒閹烘垵濮曞┑鐐叉噷閸ㄨ棄顫忓ú顏勪紶闁告洦鍋€閸嬫捇宕奸弴鐐碉紮闂佸搫绋侀崑鈧柛瀣尭椤繈鎮欓鈧锟�
//		sys_param_un.device_param.ref_tine=(sys_param_un.device_param.ref_tine%7);//offset	max=3	
//		disp_DrawRow(x+1+sys_param_un.device_param.ref_tine*6,60,1,1);//闂傚倸鍊搁崐宄懊归崶褉鏋栭柡鍥ュ灩缁愭鏌熼悧鍫熺凡闁告垹濮撮埞鎴︽偐鐎圭姴顥濈紓浣哄閸ㄥ爼寮婚敐澶婄闁挎繂鎲涢幘缁樼厸闁告侗鍠楅崐鎰版煙椤旂厧妲婚柍璇查叄楠炲洭顢楅崒婊勫櫙闂傚倷鐒﹂幃鍫曞磹閺嶎厽鍋嬮柣妯垮吹瀹撲線鐓崶銊р姇闁稿﹤顭烽弻銈夊箒閹烘垵濮曞┑鐐叉噷閸ㄨ棄顫忓ú顏勪紶闁告洦鍋€閸嬫捇宕奸弴鐐碉紮闂佸搫绋侀崑鈧柛瀣尭椤繈鎮欓鈧锟�
//		disp_DrawRow(x+sys_param_un.device_param.ref_tine*6,61,3,1);
//		disp_DrawRow(x-1+sys_param_un.device_param.ref_tine*6,62,5,1);			
		sys_param_un.device_param.ref_tine=(sys_param_un.device_param.ref_tine%13);//offset	max=3	
//		
//		disp_DrawRow(x-1,60,59,0);//闂傚倸鍊搁崐宄懊归崶褉鏋栭柡鍥ュ灩缁愭鏌熼悧鍫熺凡闁告垹濮撮埞鎴︽偐鐎圭姴顥濈紓浣哄閸ㄥ爼寮婚敐澶婄闁挎繂鎲涢幘缁樼厸闁告侗鍠楅崐鎰版煙椤旂厧妲婚柍璇查叄楠炲洭顢楅崒婊勫櫙闂傚倷鐒﹂幃鍫曞磹閺嶎厽鍋嬮柣妯垮吹瀹撲線鐓崶銊р姇闁稿﹤顭烽弻銈夊箒閹烘垵濮曞┑鐐叉噷閸ㄨ棄顫忓ú顏勪紶闁告洦鍋€閸嬫捇宕奸弴鐐碉紮闂佸搫绋侀崑鈧柛瀣尭椤繈鎮欓鈧锟�
//		disp_DrawRow(x-1,61,59,0);
//		disp_DrawRow(x-1,62,59,0);	
		
		disp_DrawRow(x+1+3*6,60,1,1);//闂傚倸鍊搁崐宄懊归崶褉鏋栭柡鍥ュ灩缁愭鏌熼悧鍫熺凡闁告垹濮撮埞鎴︽偐鐎圭姴顥濈紓浣哄閸ㄥ爼寮婚敐澶婄闁挎繂鎲涢幘缁樼厸闁告侗鍠楅崐鎰版煙椤旂厧妲婚柍璇查叄楠炲洭顢楅崒婊勫櫙闂傚倷鐒﹂幃鍫曞磹閺嶎厽鍋嬮柣妯垮吹瀹撲線鐓崶銊р姇闁稿﹤顭烽弻銈夊箒閹烘垵濮曞┑鐐叉噷閸ㄨ棄顫忓ú顏勪紶闁告洦鍋€閸嬫捇宕奸弴鐐碉紮闂佸搫绋侀崑鈧柛瀣尭椤繈鎮欓鈧锟�
		disp_DrawRow(x+3*6,61,3,1);
		disp_DrawRow(x-1+3*6,62,5,1);		
	// other area 闂傚倸鍊搁崐宄懊归崶褉鏋栭柡鍥ュ灩缁愭鏌熼悧鍫熺凡闁告垹濮撮埞鎴︽偐鐎圭姴顥濈紓浣哄閸ㄥ爼寮婚妸鈺傚亞闁稿本绋戦锟�100~160闂傚倸鍊搁崐宄懊归崶褉鏋栭柡鍥ュ灩缁愭鏌熼悧鍫熺凡闁告垹濮撮埞鎴︽偐鐎圭姴顥濈紓浣哄閸ㄥ爼寮婚妸鈺傚亞闁稿本绋戦锟�
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
		for(i=0;i<26;i++)//闂傚倸鍊搁崐宄懊归崶褉鏋栭柡鍥ュ灩缁愭鏌熼悧鍫熺凡闁告垹濮撮埞鎴︽偐鐎圭姴顥濈紓浣哄閸ㄥ爼寮婚敐澶婄闁挎繂鎲涢幘缁樼厸闁告侗鍠楅崐鎰版煛鐏炶濮傞柟顔哄€濆畷鎺戔槈濮楀棔绱�
		{
			disp_DrawRow(x+3,y+3+i,26,0);
		}
		for(i=0;i<2+4*temp;i++)//闂傚倸鍊搁崐宄懊归崶褉鏋栭柡鍥ュ灩缁愭鏌熼悧鍫熺凡闁告垹濮撮埞鎴︽偐鐎圭姴顥濈紓浣哄閸ㄥ爼寮婚敐澶婄闁挎繂鎲涢幘缁樼厸闁告侗鍠楅崐鎰版煛鐏炶濮傞柟顔哄€濆畷鎺戔槈濮楀棔绱�
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
	static enum EndoMode  motor_run_mode=Max_endoMode;
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
						{		
							App_MotorControl(MOTOR_MODE_STOP);
						}		
					}						
					auto_flag_ap = motor_apex_run_signal;//		
				}
				else if(depth<=3+sys_param_un.device_param.ref_tine)
				{	//
					if(sys_param_un.device_param.apical_action_flag==1)//
					{
						if(motor_run_mode==Max_endoMode)
						{	
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
						if(motor_run_mode==Max_endoMode)
						{
							motor_run_mode =	motor_settings.mode;
							if(motor_status.status!=Status_STOP)
							{													
								App_MotorControl(MOTOR_MODE_STOP);
							}			
						}	            			 
					}
					if(sys_param_un.device_param.apical_action_flag!=0||sys_param_un.device_param.auto_start_flag!=0||sys_param_un.device_param.auto_stop_flag!=0)//闂傚倸鍊搁崐宄懊归崶褉鏋栭柡鍥ュ灩缁愭骞栧ǎ顒€鈧垶绂嶈ぐ鎺撶厪濠电偟鍋撳▍鍛磼閳ь剙鐣濋崟顒傚帗闂佸疇妗ㄧ粈渚€寮冲▎鎰╀簻闊洦鎸鹃崺锝嗘叏婵犲懏顏犻柟鐟板婵℃悂濡烽敂鍙ョ按闂備浇顕х€涒晝鍠婂澶嬪剮妞ゆ牜鍋涢弸渚€鏌熼柇锕€鐏ｉ柣銈傚亾闂備礁鎼崐褰掝敄濞嗘劕顕遍柟鍓х帛閳锋帒霉閿濆牆袚缁绢厼鐖奸弻娑㈡偐閸愭彃顫掗悗娈垮枛閹诧紕鎹㈠┑鍡╂僵妞ゆ帒鍋嗛崬鐢告⒒娴ｈ櫣甯涢柛銊ュ悑閹便劑濡舵径濠勬煣闂佸綊妫块悞锕傛偂濞戙垺鐓曢悘鐐扮畽椤忓牆鐒垫い鎺嶈兌婢ч亶鏌嶈閸撴岸鎮㈤鍕闁跨噦鎷�
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
					if(motor_run_mode!=Max_endoMode) 
					{ 
						motor_settings.mode=motor_run_mode;		
						motor_run_mode=Max_endoMode; 
						App_MotorControl(MOTOR_SETTING_UPDATE);	
					}						 
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
					if(motor_run_mode!=Max_endoMode)
					{
						motor_settings.mode=motor_run_mode ;	
						motor_run_mode=Max_endoMode;
						motor_settings.toggle_mode_speed=speed_list[motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].toggleSpeedNum];
						motor_settings.forward_speed=speed_list[motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].motorSpeedNum];
						motor_settings.reverse_speed=-motor_settings.forward_speed;						
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
				{
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
				}						
			}	
			else 
			{
				if(motor_run_mode!=Max_endoMode) 
				{
					motor_settings.mode=motor_run_mode;	
					motor_run_mode=Max_endoMode;	
					motor_settings.forward_speed=speed_list[motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].motorSpeedNum];
					motor_settings.reverse_speed=-speed_list[motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].motorSpeedNum];
					motor_settings.toggle_mode_speed=speed_list[motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].toggleSpeedNum];
					App_MotorControl(MOTOR_SETTING_UPDATE);
				}	 
				if(motor_settings.forward_speed!=speed_list[motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].motorSpeedNum])
				{
					motor_settings.forward_speed=speed_list[motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].motorSpeedNum];
					motor_settings.reverse_speed=-speed_list[motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].motorSpeedNum];
					App_MotorControl(MOTOR_SETTING_UPDATE);	
				} 
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
						xQueueSend(xQueueKeyMessage, &sendSignal, 0);					
					}						
					auto_flag_ap= motor_apex_stop_signal;						
				}
				if(motor_status.status==Status_STOP&&sys_param_un.device_param.apical_action_flag!=0) 
				{
					App_MotorControl(MOTOR_MODE_START);
				}//restart
			}
			if(sys_param_un.device_param.auto_start_flag==0&&sys_param_un.device_param.auto_stop_flag==0&&sys_param_un.device_param.apical_action_flag==0)//闂傚倸鍊搁崐宄懊归崶褉鏋栭柡鍥ュ灩缁愭鏌熼悙顒€澧柣鎺曟铻栭柨婵嗘噹閺嗙偞銇勯妷銉уⅵ闁哄苯绉瑰畷顐﹀礋椤愮喎浜鹃柟闂寸贰閺佸倿鏌涢锝嗙闁抽攱鍨块幃褰掑箒閹烘垵顬堥梺鍝勵儐閸ㄥ潡寮婚敍鍕ㄥ亾閿濆簼鎲鹃柣鎾冲悑椤ㄣ儵鎮欓懠顒傤啋闂佽鍠楃划鎾诲箠閻愬搫唯闁挎梻鐡旀禒褔姊婚崒娆戭槮闁圭⒈鍋婂畷鎰板醇閺囩偟锛欓梺鍝勭▉閸嬧偓闁稿鎸搁～婵嬵敆閸屾簽銊╂⒑閸濆嫯顫﹂柛鏃€鍨块獮鍐Χ婢跺﹦锛滃┑鐐村灦閿曗晜瀵奸敓锟�
			{	
				if(motor_status.status!=Status_STOP&&auto_flag_ap!=motor_apex_run_signal)	auto_flag_ap=motor_apex_run_signal;
				if(motor_status.status==Status_STOP&&auto_flag_ap!=motor_apex_stop_signal)	auto_flag_ap=motor_apex_stop_signal;
			}	
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
				auto_flag_ap= motor_apex_stop_signal;//闂傚倸鍊搁崐宄懊归崶褉鏋栭柡鍥ュ灩缁愭鏌￠崶鈺佹瀺缂佽妫欓妵鍕箻鐠虹洅锝夋倵濮橆厽鐨戦柟鎻掓啞閹棃鏁愰崒姘濠殿喗锕╅崜娑氱矓濞差亝鐓涢悘鐐额嚙婵倿鏌熼鍝勭伈鐎规洏鍔嶇换婵嬪礋椤忓棛锛熼梻鍌氬€风粈浣虹礊婵犲泚澶愬箻鐠囪尙顦у┑顔姐仜閸嬫挾鈧鍣崑濠囩嵁濡吋瀚氶柛娆忣樈濡喖姊绘笟鈧褔鎮ч崱妞㈡稑螖閸滀焦鏅滈梺璺ㄥ櫐閹凤拷
				if(sys_param_un.device_param.auto_stop_flag!=0||sys_param_un.device_param.auto_start_flag!=0||sys_param_un.device_param.apical_action_flag!=0)//exit,闂傚倸鍊搁崐宄懊归崶褉鏋栭柡鍥ュ灩缁愭骞栧ǎ顒€鐏柣婵婂煐缁绘盯宕卞Ο铏逛患婵＄偠顫夋繛濠囧箖瀹勬壋鏋庨煫鍥ㄦ惄娴犵厧顪冮妶鍛闁绘牜鍘ч～蹇涙惞鐟欏嫬纾梺闈浨归崕鏌ユ倵妤ｅ啯鈷戦柛婵嗗閸庡繑銇勯鐘插幋闁绘侗鍠涚粻娑樷槈濞嗗繆鍋撴繝姘厾闁诡厽甯掗崝婊勭箾閸涱偄鐏叉慨濠冩そ瀹曨偊宕熼鐔蜂壕闁割偅娲栫壕褰掓煛瀹ュ骸浜愰柛瀣尭椤繈顢楅崒婧炪劑姊洪崫鍕潶闁告梹鍨块獮鍐閵堝懎绐涙繝鐢靛Т鐎氼亞妲愰弴銏♀拻濞达絿鍎ら崵鈧梺鎼炲灪閻撯€崇暦閺屻儱鐭楀璺虹焾濞村嫰姊虹紒妯忣亜螣婵犲偆鐒藉ù鐓庣摠閻撱儵鏌ｉ弴鐐测偓鍦偓姘炬嫹,闂傚倸鍊搁崐宄懊归崶褉鏋栭柡鍥ュ灩缁愭鏌熼悧鍫熺凡闁告垹濮撮埞鎴︽偐鐎圭姴顥濈紓浣哄閸ㄥ爼寮婚敐澶婄闁挎繂鎲涢幘缁樼厸闁告侗鍠楅崐鎰版煛瀹€瀣瘈鐎规洘甯掕灒閻炴稈鈧厖澹曢梺鍝勭▉閸嬧偓闁稿鎸搁～婵嬵敆閸屾簽銊╂⒑閸濆嫯顫﹂柛鏃€鍨块獮鍐閵堝懎绐涙繝鐢靛Т鐎氼亞妲愰弴銏♀拻濞达絽鎽滅粔鐑樸亜閵夛附宕岀€规洘顨呴～婊堝焵椤掆偓椤曪綁顢曢敃鈧粻濠氭偣閸パ冪骇妞ゃ儲绻堝娲濞戞艾顣哄┑鈽嗗亝椤ㄥ﹪銆侀弮鍫濋唶闁哄洨鍟块幏娲煟閻樺厖鑸柛鏂跨焸瀵悂骞嬮敂鐣屽幐闁诲函缍嗘禍鍫曟偂閸忕⒈娈介柣鎰皺缁犲鏌熼鐣岀煉闁瑰磭鍋ゆ俊鐑芥晜缁涘鎴烽梻鍌氬€风粈渚€骞栭锕€纾归柛顐ｆ礀绾惧綊鏌″搴′簮闁稿鎸搁～婵嬵敆閸屾簽銊╂⒑閸濆嫯顫﹂柛鏃€鍨块獮鍐閵堝懎绐涙繝鐢靛Т鐎氼亞妲愰弴銏♀拻濞达絽鎽滅粔鐑樸亜閵夛附宕岀€规洘顨呴～婊堝焵椤掆偓椤曪綁顢曢敃鈧粻濠氭偣閸ヮ亜鐨洪柣鎾村灥椤啴濡堕崱妤€袝闂佸憡鎸诲銊╁焵椤掍胶鍟查柟鍑ゆ嫹
				{
					xQueueSend(xQueueKeyMessage, &auto_flag_ap, 0);//濠电姷顣槐鏇㈠磻閹达箑纾规俊銈呮噹閺嬩礁鈹戦悩瀹犲闁哄绶氶弻鐔煎箲閹伴潧娈┑鈽嗗亝閿曘垽寮婚弴銏犻唶婵犻潧娴傚Λ鐐差渻閵堝懐绠伴柣鏍帶椤繘鎼圭憴鍕／闂侀潧枪閸庢煡鎮楁繝姘仭婵犲﹤妫楅悞浠嬫煕閺傝法肖闁逞屽墰閺佹悂宕㈣閸┿垺鎯旈妸銉х杸濡炪倖甯掗ˇ顔炬閿燂拷
					App_MotorControl(MOTOR_MODE_STOP);
				}					
			}
		}				
	 }
	else//闂傚倸鍊搁崐宄懊归崶褉鏋栭柡鍥ュ灩缁愭鏌￠崶鈺佹灁缂佺娀绠栭弻娑㈠Ψ閵忊剝鐝栨俊鐐额潐婵炲﹪骞冨畡鎵虫瀻闊洦鎼╂禒鐓庮渻閵堝懘鐛滈柟鍑ゆ嫹
	{		
		if(auto_flag_ap!=motor_apex_stop_signal)
		{
			auto_flag_ap= motor_apex_stop_signal;//闂傚倸鍊烽懗鍓佸垝椤栨稓浠氶梻浣虹《閺呮粓鎮ч悩璇叉槬闁逞屽墯閵囧嫰骞嬮敐鍡欍€婄紓浣瑰姈濡啴寮婚埄鍐╁閻熸瑥瀚壕鎶芥倵濞堝灝娅橀柛鎾跺枑娣囧﹪宕奸弴鐐茶€垮┑掳鍊曢敃锝囨閿曞倹鈷掗柛灞剧懄缁佺増绻涙径瀣鐎规洑鍗抽獮鍥级閸喖娈ゅ┑鐘垫暩婵敻鎳濋崜褏涓嶆い鏍仦閻撱儵鏌ｉ弴鐐测偓鍦偓姘炬嫹				
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
	if(realTorque>MAX_TORQUE_UUPER_THRESHOLD*1.05)//闂傚倸鍊搁崐宄懊归崶褉鏋栭柡鍥ュ灩缁愭鏌熼悧鍫熺凡闁告垹濮撮埞鎴︽偐鐎圭姴顥濈紓浣哄閸ㄥ爼寮婚敐澶婄闁挎繂鎲涢幘缁樼厸闁告侗鍠楅崐鎰版煛瀹€瀣瘈鐎规洘甯掕灒閻炴稈鈧厖澹曢梺鍝勭▉閸嬧偓闁稿鎸搁～婵嬫倷椤掆偓椤忥拷10%(1.10==5.0N,1.08<=4.8)
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
							motor_settings.lower_threshold= motor_settings.upper_threshold*3/5;//torque_list[motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].atcTorqueThresholdNum]*0.10;
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
				if(realTorque>torque_list[motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].recTorqueThresholdNum])//MAX_TORQUE_UUPER_THRESHOLD*0.95)//闂傚倸鍊搁崐宄懊归崶褉鏋栭柡鍥ュ灩缁愭鏌熼悧鍫熺凡闁告垹濮撮埞鎴︽偐鐎圭姴顥濈紓浣哄閸ㄦ娊鍩€椤掑倹鍤€閻庢矮鍗抽妴鍌炴晜閹规劕小闂佽鍎兼慨銈夊磹閸洜鍙撻柛銉ｅ妽鐏忕數绱掗悩宸吋闁哄备鍓濋幏鍛喆閸曨偊鐎洪梻渚€鈧稓鈹掗柛鏃€鍨块悰顕€宕堕鈧粈鍫澝归敐鍕劅婵¤尙鍏樺濠氬磼濞嗘劗銈板┑鐐差槹濞茬喎鐣锋导鏉戠疀闁哄鐏濋崵鎴炵節閻㈤潧校闁肩懓澧界划濠氼敍閻愬鍙嗛梺缁樻礀閸婂湱鈧熬鎷�4.0濠电姷鏁搁崑鐐哄垂閸洖绠伴柟闂寸缁犺銇勯幇鍓佺暠闂傚偆鍨堕弻銊╂偆閸屾稑顏�
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
							motor_settings.lower_threshold=motor_settings.upper_threshold*3/5;//torque_list[motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].atcTorqueThresholdNum]*0.10;
							motor_settings.forward_position=motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].forwardPosition;
							motor_settings.reverse_position=motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].reversePosition;
							App_MotorControl(MOTOR_MODE_RESTART);	
						}								
					}							 
				} //beep
				if(realTorque>torque_list[motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].recTorqueThresholdNum]*0.8)//MAX_TORQUE_UUPER_THRESHOLD*0.80)//濠电姵顔栭崰妤冩暜濡ゅ啰鐭欓柟鐑橆殕閺呮繈鏌曡箛瀣偓鏇㈠几娓氣偓閺岀喖骞戦幇闈涙濠碘槅鍋呴敃銏ゅ蓟閺囥垹閱囨繝闈涙祩濡偛顪冮妶鍛闁绘牕銈稿璇测槈濞嗘垹鐦堥棅顐㈡处閺屻劑顢旈敓锟�
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
		if(10+recTimeMs<systemTimeMs)
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
		if(get_charge_state()==RESET)		//闂傚倸鍊风粈渚€骞栭锔藉亱婵犲﹤瀚々鍙夈亜韫囨挾澧曢柡瀣╃窔閺岀喖骞戦幇闈涙濠碘槅鍋呴敃銏ゅ蓟閺囥垹閱囨繝闈涙祩濡偛顪冮妶鍛闁绘牜鍘ч～蹇涙惞鐟欏嫬纾梺闈浨归崕鏌ユ倵妤ｅ啯鈷戦柛婵嗗閸庡繑銇勯鐘插幋闁绘侗鍠涚粻娑樷槈濞嗘劖顏熼梻浣芥硶閸ｏ箓骞忛敓锟�
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
	else if(pageID==MENU_MAX_PAGE_NUM)//闂傚倸鍊搁崐宄懊归崶褉鏋栭柡鍥ュ灩缁愭鏌熼悧鍫熺凡闁哄嫨鍎甸弻娑樷槈閸楃偞鐏堝銈嗘礉妞村摜鎹㈠☉銏犲耿婵☆垰鎼～灞筋渻閵堝懐绠伴柣鏍с偢瀵鈽夐姀鐘殿啋閻庤娲栧▔锕侇樄闁哄备鍓濋幏鍛喆閸曨偆锛撻柣搴㈩問閸ｎ噣宕滈悢闀愮箚闁割偅娲栭獮銏′繆閵堝拑姊楃紒鎲嬪缁辨捇宕掑▎鎺濆敼闂佺ǹ顑嗛幑鍥蓟閵娾晜鍋嗛柛灞剧☉椤忥拷
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
	task_notify_enum rec_Signal=null_signal;//
	SCREEN_SETTING_ENUM selectNum=SETTING_FOR,submenuPageNum=SETTING_FOR;
	static uint8_t menuPage=MENU_LOGO_PAGE,motor_run_cmd=0,motorOrApexSetFlag=0,exitApexFlag=1;
	confirm_state  apexMotorStartFlag =TRUE;//apex
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
					apexMotorStartFlag=FALSE;//闂傚倸鍊搁崐宄懊归崶褉鏋栭柡鍥ュ灩缁愭鏌熼悧鍫熺凡闁告垹濮撮埞鎴︽偐鐎圭姴顥濈紓浣哄閸ㄨ泛顫忓ú顏勫瀭妞ゆ洖鎳庨崜楣冩⒑閸涘﹥宕岀紒鐘崇墪椤繘鎼圭憴鍕／闂侀潧枪閸庢煡鎮楁ィ鍐┾拺缂侇垱娲樺▍鍛存煙閾忣個顏囩亱闂佸憡鍔﹂崰鏍偂濞戙垺鐓曢悘鐐村劤閸ゎ剙霉閸忓吋宕屾慨濠勭帛閹峰懘鎳為妷褋鈧﹪姊洪崫銉バｉ柛鏃€鐟ラ锝夊川婵犲嫮鐦堝┑顔斤供閸撴盯顢欓崱娑欌拺缂備焦锚閻忓崬鈹戦鍝勨偓婵嗩嚕閺屻儱绠瑰ù锝呮贡閸樻悂姊虹粙鎸庢拱闁挎岸鏌嶈閸撴岸鎮у⿰鍫濈劦妞ゆ帊鑳堕崯鏌ユ煙閸戙倖瀚�
					MenuMotorParamUpdate(sys_param_un.device_param.use_p_num);//save data							
					if(selectNum==ADJUST_MOTOR_PARAM_PROGRAM_NUM)
					{
						if(sys_param_un.device_param.auto_stop_flag==0&&sys_param_un.device_param.auto_start_flag==0&&sys_param_un.device_param.apical_action_flag==0)
						{	//闂傚倸鍊搁崐宄懊归崶褉鏋栭柡鍥ュ灩缁愭鏌熼悧鍫熺凡闁告垹濮撮埞鎴︽偐鐎圭姴顥濈紓浣哄閸ㄥ爼寮婚敐澶婄闁挎繂鎲涢幘缁樼厸闁告侗鍠楅崐鎰版煛瀹€瀣瘈鐎规洘甯掕灒閻炴稈鈧厖澹曢梺鍝勭▉閸嬧偓闁稿鎸搁～婵嬵敆閸屾簽銊╂⒑閸濆嫯顫﹂柛鏃€鍨块獮鍐閵堝懎绐涙繝鐢靛Т鐎氼亞妲愰弴銏♀拻濞达絿鍎ら崵鈧梺鎼炲€栭悧鐘荤嵁韫囨稒鏅搁柨鐕傛嫹
							menuPage = MENU_MOTOR_WORK_PAGE;
							if(motor_run_cmd==MOTOR_MODE_STOP) // motor control
							{
								motor_run_cmd=MOTOR_MODE_START;
							}									
							App_MotorControl(motor_run_cmd);	
						}
						else
						{
							if(sys_param_un.device_param.apexFunctionLoad!=0)//闂傚倸鍊搁崐宄懊归崶褉鏋栭柡鍥ュ灩缁愭鏌熼悧鍫熺凡闁告垹濮撮埞鎴︽偐鐎圭姴顥濈紓浣哄閸ㄥ爼寮婚敐澶婄闁挎繂鎲涢幘缁樼厸闁告侗鍠楅崐鎰版煛瀹€瀣瘈鐎规洘甯掕灒閻炴稈鈧厖澹曢梺鍝勭▉閸嬧偓闁稿鎸搁～婵嬵敆閸屾簽銊╂⒑閸濆嫯顫﹂柛鏃€鍨块獮鍐閵堝懎绐涙繝鐢靛Т鐎氼亞妲愰弴銏♀拻濞达絽鎽滅粔鐑樸亜閵夛附宕岀€规洘顨呴～婊堝焵椤掆偓椤曪綁顢曢敃鈧粻濠氭偣閸パ冪骇妞ゃ儲绻堝娲濞戞艾顣哄┑鈽嗗亝椤ㄥ﹪銆侀弮鍫濋唶闁哄洨鍟块幏娲煟閻樺厖鑸柛鏂胯嫰閳诲秹骞囬悧鍫㈠幍闂佸憡鍨崐鏍偓姘炬嫹																						
							{		//闂傚倸鍊搁崐宄懊归崶褉鏋栭柡鍥ュ灩缁愭鏌熼悧鍫熺凡闁告垹濮撮埞鎴︽偐鐎圭姴顥濈紓浣哄閸ㄥ爼寮婚敐澶婄闁挎繂鎲涢幘缁樼厸闁告侗鍠楅崐鎰版煛瀹€瀣瘈鐎规洘甯掕灒閻炴稈鈧厖澹曢梺鍝勭▉閸嬧偓闁稿鎸搁～婵嬪箛娴ｅ湱绉风紓鍌欑椤戝懘藝閻㈢ǹ鏄ラ柍褜鍓熼弻銊╂偆閸屾稑顏�							
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
//									motor_run_cmd=MOTOR_MODE_STOP;//闂傚倸鍊搁崐宄懊归崶褉鏋栭柡鍥ュ灩缁愭鏌熼悧鍫熺凡闁告垹濮撮埞鎴︽偐鐎圭姴顥濈紓浣哄閸ㄥ爼寮婚敐澶婄闁挎繂鎲涢幘缁樼厸闁告侗鍠楅崐鎰版煛瀹€瀣М妞ゃ垺枪椤︽彃霉濠婂嫮鐭掗柟顔角圭粻娑㈠箻閺夋垶鐣绘俊銈囧Х閸嬫稓鎹㈠鈧獮鍐ㄢ堪閸喎娈熼梺闈涱檧闂勫嫰顢旈悢鍏尖拻濞达絽鎲￠幆鍫ユ煕婵犲倻鍩ｇ€规洘鍔欏浠嬵敃閻旇渹澹曞┑顔筋焽閸嬫挾鈧熬鎷�
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
					motorOrApexSetFlag=0;	//濠电姵顔栭崰妤冩暜濡ゅ啰鐭欓柟鐑橆殕閺呮繈鏌曡箛瀣偓鏇㈠几娓氣偓閺岀喖骞戦幇闈涙濠碘槅鍋呴敃銏ゅ蓟閺囥垹閱囨繝闈涙祩濡偛顪冮妶鍛闁绘牕銈稿璇测槈濞嗘垹鐦堥梺鍛婃处閸樺綊鍩€椤掆偓濠€顨篛R						
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
									HomePageHandle(motor_setting_updata_signal,ADJUST_MOTOR_PARAM_PROGRAM_NUM);//闂傚倸鍊搁崐宄懊归崶褉鏋栭柡鍥ュ灩缁愭骞栧ǎ顒€鈧垶绂嶈ぐ鎺撶厪濠电姴绻掗悾杈ㄣ亜閺囷繝鍝虹紒缁樼洴瀹曞崬螣閸濆嫷娼撴俊鐐€曠换鎰版偉閻撳寒娼栭柧蹇氼潐瀹曞鏌曟繛鍨姕闁诲酣绠栧娲传閸曢潧鍓板銈庡幘閸忔﹢鎮伴璺ㄧ杸婵炴垶鐟ラ埀顒€顭烽弻銈夊箒閹烘垵濮曞┑鐐叉噷閸ㄨ棄顫忓ú顏勪紶闁告洦鍋€閸嬫捇宕奸弴鐐碉紮闂佸搫绋侀崑鈧柛瀣尭椤繈顢楅崒婧炪劑姊洪崫鍕潶闁告梹鍨块獮鍐閵堝懎绐涙繝鐢靛Т鐎氼亞妲愰弴銏♀拻濞达絽鎽滅粔鐑樸亜閵夛附宕岀€规洘顨呴～婊堝焵椤掆偓椤曪綁顢曢敃鈧粻鑽ょ磽娴ｈ偂鎴濃枍閵忋倖鈷戦悹鎭掑妼濞呮劙鏌熼崙銈嗗
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
						if(rec_Signal==null_signal)//APEX闂傚倸鍊搁崐宄懊归崶褉鏋栭柡鍥ュ灩缁愭鏌熼悧鍫熺凡闁告垹濮撮埞鎴︽偐鐎圭姴顥濈紓浣哄閸ㄥ爼寮婚敐澶婄闁挎繂鎲涢幘缁樼厸闁告侗鍠楅崐鎰版煛瀹€瀣瘈鐎规洘甯掕灒閻炴稈鈧厖澹曢梺鍝勭▉閸嬧偓闁稿鎸搁～婵嬵敆閸屾簽銊╂⒑閸濆嫯顫﹂柛鏃€鍨块獮鍐閵堝懎绐涙繝鐢靛Т鐎氼亞妲愰弴銏♀拻濞达絽鎽滅粔鐑樸亜閵夛附宕岀€殿喚绮鍕箛閳规儳浜鹃柛娑樼摠閺呮繈鏌涚仦鍓с€掗柛娆忔濮婅櫣绱掑Ο鑽ゅ弳闂佺顕滅换婵嬪箖濮椻偓椤㈡岸鍩€椤掑嫬钃熼柣鏂垮濡插綊骞栫划鍏夊亾閸愯尙顓洪梻鍌欐祰濡椼劑鎮為敃鍌氱闁搞儺鍓欓拑鐔烘喐閺傛鍤曢柛顐ｆ礀閸ㄥ倹銇勯弮鍌氫壕妞ゆ梹鍔欏缁樻媴閸濄儳楔濡炪們鍎查幑鍥х暦濡も偓椤粓鍩€椤掆偓椤曪綁顢曢敃鈧粻濠氭偣閸パ冪骇妞ゃ儲绻堝娲濞戞艾顣哄┑鈽嗗亝椤ㄥ﹪銆侀弮鍫熸櫢闁跨噦鎷�
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
										if(apexMotorStartFlag==TRUE)	//闂傚倸鍊搁崐宄懊归崶褉鏋栭柡鍥ュ灩缁愭鏌熼悧鍫熺凡闁告垹濮撮埞鎴︽偐鐎圭姴顥濈紓浣哄閸ㄥ爼寮婚敐澶婄闁挎繂鎲涢幘缁樼厸闁告侗鍠楅崐鎰版煛瀹€瀣瘈鐎规洘甯掕灒閻炴稈鈧厖澹曢梺鍝勭▉閸嬧偓闁稿鎸搁～婵嬵敆閸屾簽銊╂⒑閸濆嫯顫﹂柛鏃€鍨块獮鍐閵堝懎绐涙繝鐢靛Т鐎氼亞妲愰弴銏♀拻濞达絽鎽滅粔鐑樸亜閵夛附宕岀€规洘顨呴～婊堝焵椤掆偓椤曪綁顢曢敃鈧粻濠氭偣閸パ冪骇妞ゃ儲绻堝娲濞戞艾顣哄┑鈽嗗亝椤ㄥ﹪銆侀弮鍫濋唶闁哄洨鍟块幏娲煟閻樺厖鑸柛鏂跨焸瀵悂骞嬮敂鐣屽幐闁诲函缍嗘禍鍫曟偂閸忕⒈娈介柣鎰皺缁犲鏌熼鐣岀煉闁瑰磭鍋ゆ俊鐑芥晜缁涘鎴烽梻鍌氬€风粈渚€骞栭锕€纾归柛顐ｆ礀绾惧綊鏌″搴′簮闁稿鎸搁～婵嬵敆閸屾簽銊╂⒑閸濆嫯顫﹂柛鏃€鍨块獮鍐閵堝懎绐涙繝鐢靛Т鐎氼亞妲愰弴銏♀拻濞达絽鎽滅粔鐑樸亜閵夛附宕岀€规洘顨呴～婊堝焵椤掆偓椤曪綁顢曢敃鈧粻濠氭偣閸パ冪骇妞ゃ儲绻堝娲濞戞艾顣哄┑鈽嗗亝椤ㄥ﹪銆侀弮鍌涘磯閻炴稈鍓濋～宥夋⒑閸︻叀妾搁柛銊у缁傚秵銈ｉ崘鈹炬嫽婵炶揪缍€婵倗娑甸崼鏇熺厱闁绘ê鍟挎慨宥団偓娈垮枛閹诧紕鎹㈠┑鍡╂僵妞ゆ帒鍋嗛崬鐢告⒒娴ｈ姤纭堕柛锝忕畵楠炲繘鏁撻敓锟�
										{	
//										apexMotorStartFlag=FALSE;//闂傚倸鍊搁崐宄懊归崶褉鏋栭柡鍥ュ灩缁愭鏌熼悧鍫熺凡闁告垹濮撮埞鎴︽偐鐎圭姴顥濈紓浣哄閸ㄨ泛顫忓ú顏勫瀭妞ゆ洖鎳庨崜楣冩⒑閸涘﹥宕岀紒鐘崇墪椤繘鎼圭憴鍕／闂侀潧枪閸庢煡鎮楁ィ鍐┾拺缂侇垱娲樺▍鍛存煙閾忣個顏囩亱闂佸憡鍔﹂崰鏍偂濞戙垺鐓曢悘鐐村劤閸ゎ剙霉閸忓吋宕屾慨濠勭帛閹峰懘鎳為妷褋鈧﹪姊洪崫銉バｉ柛鏃€鐟ラ锝夊川婵犲嫮鐦堝┑顔斤供閸撴盯顢欓崱娑欌拺缂備焦锚閻忓崬鈹戦鍝勨偓婵嗩嚕閺屻儱绠瑰ù锝呮贡閸樻悂姊虹粙鎸庢拱闁挎岸鏌嶈閸撴岸鎮у⿰鍫濈劦妞ゆ帊鑳堕崯鏌ユ煙閸戙倖瀚�
											MenuMotorParamUpdate(sys_param_un.device_param.use_p_num);//save data	
											if(GC_depth_vlaue(0, 1)>=0&&sys_param_un.device_param.apical_action_flag!=2)
											{
												if(motor_run_cmd==MOTOR_MODE_STOP) // motor control
												{
														motor_run_cmd=MOTOR_MODE_START;
												}	
												App_MotorControl(motor_run_cmd);			//闂傚倸鍊搁崐宄懊归崶褉鏋栭柡鍥ュ灩缁愭鏌熼悧鍫熺凡闁告垹濮撮埞鎴︽偐鐎圭姴顥濈紓浣哄閸ㄥ爼寮婚敐澶婄闁挎繂鎲涢幘缁樼厸闁告侗鍠楅崐鎰版煙椤旂厧妲婚柍璇查閳诲酣骞嬪┑鍡欏帓婵犵數濮甸鏍窗閹烘鐤鹃柣妯款嚙閽冪喖鏌曟繛鐐珕闁稿瀚伴弻娑樷槈濮楀牆濮涙繛瀵稿帶鐎氭澘顫忓ú顏呭仭闁哄瀵ч鈧梻浣烘嚀閸ゆ牠骞忛敓锟�
											}														
											menuPage = MENU_APEX_AND_MOTOR_PAGE;//闂傚倸鍊搁崐宄懊归崶褉鏋栭柡鍥ュ灩缁愭鏌熼悧鍫熺凡闁告垹濮撮埞鎴︽偐鐎圭姴顥濈紓浣哄閸ㄥ爼寮婚敐澶婄闁挎繂鎲涢幘缁樼厸闁告侗鍠楅崐鎰版煛瀹€瀣瘈鐎规洘甯掕灒閻炴稈鈧厖澹曢梺鍝勭▉閸嬧偓闁稿鎸搁～婵嬵敆閸屾簽銊╂⒑閸濆嫯顫﹂柛鏃€鍨块獮鍐閵堝懎绐涙繝鐢靛Т鐎氼亞妲愰弴銏♀拻濞达絿鍎ら崵鈧梺鎼炲€栭悧鐘荤嵁韫囨稒鏅搁柨鐕傛嫹
											MenuPageTurns(menuPage);
										}
									}	
                					else 
									{
										apexMotorStartFlag=TRUE;//闂傚倸鍊搁崐宄懊归崶褉鏋栭柡鍥ュ灩缁愭鏌熼悧鍫熺凡闁告垹濮撮埞鎴︽偐鐎圭姴顥濈紓浣哄閸ㄥ爼寮婚敐澶婄闁挎繂鎲涢幘缁樼厸闁告侗鍠楅崐鎰版煛瀹€瀣瘈鐎规洘甯掕灒閻炴稈鈧厖澹曢梺鍝勭▉閸嬧偓闁稿鎸搁～婵嬵敆閸屾簽銊╂⒑閸濆嫯顫﹂柛鏃€鍨块獮鍐閵堝懎绐涙繝鐢靛Т鐎氼亞妲愰弴銏♀拻濞达絽鎽滅粔鐑樸亜閵夛附宕岀€规洘顨呴～婊堝焵椤掆偓椤曪綁顢曢敃鈧粻濠氭偣閸パ冪骇妞ゃ儲绻堝娲濞戞艾顣哄┑鈽嗗亝椤ㄥ﹪銆侀弮鍫濋唶闁哄洨鍟块幏娲煟閻樺厖鑸柛鏂跨焸瀵悂骞嬮敂鐣屽幐闁诲函缍嗘禍鍫曟偂閸忕⒈娈介柣鎰皺缁犲鏌℃担瑙勫磳闁轰焦鎹囬弫鎾绘晸閿燂拷
									}												
								}						
							}									
							else
							{
								apexMotorStartFlag=TRUE;//闂傚倸鍊搁崐宄懊归崶褉鏋栭柡鍥ュ灩缁愭鏌熼悧鍫熺凡闁告垹濮撮埞鎴︽偐鐎圭姴顥濈紓浣哄閸ㄥ爼寮婚敐澶婄闁挎繂鎲涢幘缁樼厸闁告侗鍠楅崐鎰版煛瀹€瀣瘈鐎规洘甯掕灒閻炴稈鈧厖澹曢梺鍝勭▉閸嬧偓闁稿鎸搁～婵嬵敆閸屾簽銊╂⒑閸濆嫯顫﹂柛鏃€鍨块獮鍐閵堝懎绐涙繝鐢靛Т鐎氼亞妲愰弴銏♀拻濞达絽鎽滅粔鐑樸亜閵夛附宕岀€规洘顨呴～婊堝焵椤掆偓椤曪綁顢曢敃鈧粻濠氭偣閸パ冪骇妞ゃ儲绻堝娲濞戞艾顣哄┑鈽嗗亝椤ㄥ﹪銆侀弮鍫濋唶闁哄洨鍟块幏娲煟閻樺厖鑸柛鏂跨焸瀵悂骞嬮敂鐣屽幐闁诲函缍嗘禍鍫曟偂閸忕⒈娈介柣鎰皺缁犲鏌℃担瑙勫磳闁轰焦鎹囬弫鎾绘晸閿燂拷
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
						else//闂傚倸鍊搁崐宄懊归崶褉鏋栭柡鍥ュ灩缁愭鏌熼悧鍫熺凡闁告垹濮撮埞鎴︽偐鐎圭姴顥濈紓浣哄閸ㄥ爼寮婚敐澶婄闁挎繂鎲涢幘缁樼厸闁告侗鍠楅崐鎰版煛瀹€瀣瘈鐎规洘甯掕灒閻炴稈鈧厖澹曢梺鍝勭▉閸嬧偓闁稿鎸搁～婵嬫倷椤掆偓椤忥拷
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
				if(sys_param_un.device_param.apexFunctionLoad!=0)//闂傚倸鍊搁崐宄懊归崶褉鏋栭柡鍥ュ灩缁愭鏌熼悧鍫熺凡闁告垹濮撮埞鎴︽偐鐎圭姴顥濈紓浣哄閸ㄥ爼寮婚敐澶婄闁挎繂鎲涢幘缁樼厸闁告侗鍠楅崐鎰版煛瀹€瀣瘈鐎规洘甯掕灒閻炴稈鈧厖澹曢梺鍝勭▉閸嬧偓闁稿鎸搁～婵嬵敆閸屾簽銊╂⒑閸濆嫯顫﹂柛鏃€鍨块獮鍐閵堝懎绐涙繝鐢靛Т鐎氼亞妲愰弴銏♀拻濞达絽鎽滅粔鐑樸亜閵夛附宕岀€规洘顨呴～婊堝焵椤掆偓椤曪綁顢曢敃鈧粻濠氭偣閸パ冪骇妞ゃ儲绻堝娲濞戞艾顣哄┑鈽嗗亝椤ㄥ﹪銆侀弮鍫濋唶闁哄洨鍟块幏娲煟閻樺厖鑸柛鏂胯嫰閳诲秹骞囬悧鍫㈠幍闂佸憡鍨崐鏍偓姘炬嫹
				{					
					menuPage=MENU_APEX_AND_MOTOR_PAGE;	//闂傚倸鍊搁崐宄懊归崶褉鏋栭柡鍥ュ灩缁愭骞栧ǎ顒€鈧垶绂嶈ぐ鎺撶厪濠电姴绻掗悾杈ㄣ亜閺囷繝鍝虹紒缁樼洴瀹曞崬螣閸濆嫷娼撴俊鐐€曠换鎰版偉閻撳寒娼栭柧蹇氼潐瀹曞鏌曟繛鍨姕闁诲酣绠栧娲传閸曢潧鍓板銈庡幘閸忔﹢鎮伴璺ㄧ杸婵炴垶鐟ラ埀顒€顭烽弻銈夊箒閹烘垵濮曞┑鐐叉噷閸ㄨ棄顫忓ú顏勪紶闁告洦鍋€閸嬫捇宕奸弴鐐碉紮闂佸搫绋侀崑鈧柛瀣尭椤繈顢楅崒婧炪劑姊洪崫鍕潶闁告梹鍨块獮鍐閵堝懎绐涙繝鐢靛Т鐎氼亞妲愰弴銏♀拻濞达絽鎽滅粔鐑樸亜閵夛附宕岀€规洘顨呴～婊堝焵椤掆偓椤曪綁顢曢敃鈧粻濠氭偣閸パ冪骇妞ゃ儲绻堝娲濞戞艾顣哄┑鈽嗗亝椤ㄥ﹪銆侀弮鍫濋唶闁哄洨鍟块幏娲煟閻樺厖鑸柛鏂跨焸瀵悂骞嬮敂鐣屽幐闁诲函缍嗘禍鍫曟偂閸忕⒈娈介柣鎰皺缁犲鏌熼鐣岀煉闁瑰磭鍋ゆ俊鐑芥晜缁涘鎴烽梻鍌氬€风粈渚€骞栭锕€纾归柛顐ｆ礀绾惧綊鏌″搴′簮闁稿鎸搁～婵嬵敆閸屾簽銊╂⒑閸濆嫯顫﹂柛鏃€鍨块獮鍐Χ婢跺﹦锛滃┑鐐村灦閿曗晜瀵奸敓锟�	
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
						exitApexFlag=0;//闂傚倸鍊搁崐宄懊归崶褉鏋栭柡鍥ュ灩缁愭鏌熼悧鍫熺凡闁告垹濮撮埞鎴︽偐鐎圭姴顥濈紓浣哄閸ㄥ爼寮婚敐澶婄闁挎繂鎲涢幘缁樼厸闁告侗鍠楅崐鎰版煛瀹€瀣瘈鐎规洘甯掕灒閻炴稈鈧厖澹曢梺鍝勭▉閸嬧偓闁稿鎸搁～婵嬵敆閸屾簽銊╂⒑鐠団€虫灀闁哄懐濞€楠炲﹤饪伴崨顓熺€冲┑鈽嗗灥濡椼劍绔熼弴銏♀拺閻犳亽鍔屽▍鎰版煙閸戙倖瀚�
					}
					if(sys_param_un.device_param.apexFunctionLoad==0&&exitApexFlag==0)//闂傚倸鍊搁崐宄懊归崶褉鏋栭柡鍥ュ灩缁愭鏌￠崶鈺佹灁缂佺娀绠栭弻娑㈠Ψ閵忊剝鐝栨俊鐐额潐婵炲﹪骞冨畡鎵虫瀻闊洦鎼╂禒鐓庮渻閵堝懐绠伴柣鏍帶椤繘鎼圭憴鍕／闂侀潧枪閸庢煡鎮楁ィ鍐┾拺闁告繂瀚烽崕蹇斻亜椤撶姴鍘撮柣娑卞枦缁犳稑鈽夊▎蹇婂亾婵犳碍鐓犻柟顓熷笒閸旀粍绻涢崨顐㈢伈婵﹥妞藉畷顐﹀礋椤愮喎浜鹃柛顐ｆ礀绾惧綊鏌″搴′簮闁稿鎸搁～婵嬵敆閸屾簽銊╂⒑閸濆嫯顫﹂柛鏃€鍨块獮鍐閵堝懎绐涙繝鐢靛Т鐎氼亞妲愰弴銏♀拻濞达絿鍎ら崵鈧梺鎼炲€栭悧鐘荤嵁韫囨稒鏅搁柨鐕傛嫹,闂傚倸鍊搁崐宄懊归崶褉鏋栭柡鍥ュ灩缁愭骞栧ǎ顒€鈧垶绂嶈ぐ鎺撶厪濠电姴绻掗悾杈ㄣ亜閺囷繝鍝虹紒缁樼洴瀹曞崬螣閸濆嫷娼撴俊鐐€曠换鎰版偉閻撳寒娼栭柧蹇氼潐瀹曞鏌曟繛鍨姕闁诲酣绠栧娲传閸曢潧鍓板銈庡幘閸忔﹢鎮伴璺ㄧ杸婵炴垶鐟ラ埀顒€顭烽弻銈夊箒閹烘垵濮曞┑鐐叉噷閸ㄨ棄顫忓ú顏勪紶闁告洦鍋€閸嬫捇宕奸弴鐐碉紮闂佸搫绋侀崑鈧柛瀣尭椤繈顢楅崒婧炪劑姊洪崫鍕潶闁告梹鍨块獮鍐閵堝懎绐涙繝鐢靛Т鐎氼亞妲愰弴銏♀拻濞达絽鎽滅粔鐑樸亜閵夛附宕岀€规洘顨呴～婊堝焵椤掆偓椤曪綁顢曢敃鈧粻濠氭偣閸パ冪骇妞ゃ儲绻堝娲濞戞艾顣哄┑鈽嗗亝椤ㄥ﹪銆侀弮鍫濋唶闁哄洨鍟块幏娲煟閻樺厖鑸柛鏂跨焸瀵悂骞嬮敂鐣屽幐闁诲函缍嗘禍鍫曟偂閸忕⒈娈介柣鎰皺缁犱即鏌ｉ敐蹇曠瘈妤犵偛绻樺顕€鍩€椤掑嫭鍎庢い鏍亼閳ь兛绶氬浠嬪Ω閿斿墽肖婵＄偑鍊栭崝褏寰婇挊澶嗘鐟滄柨顫忔ウ瑁や汗闁圭儤绻冮ˉ鏍⒑缁嬭法绠查柨鏇樺灩椤曪綁顢曢敃鈧粻濠氭偣閸ヮ亜鐨烘い鏂胯嫰椤啴濡堕崱妯洪瀺缂備浇顕х€氼厾绮欐径鎰摕闁靛濡囬崢鍨繆閻愬樊鍎忓Δ鐘虫倐瀹曘垽骞橀鐣屽幐婵炶揪缍€椤鐣峰畝鍕厸濞达綀顫夊畷宀勬煛娴ｈ宕岄柡浣规崌閺佹捇鏁撻敓锟�
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
					apexMotorStartFlag=FALSE;//闂傚倸鍊搁崐宄懊归崶褉鏋栭柡鍥ュ灩缁愭鏌熼悧鍫熺凡闁告垹濮撮埞鎴︽偐鐎圭姴顥濈紓浣哄閸ㄨ泛顫忓ú顏勫瀭妞ゆ洖鎳庨崜楣冩⒑閸涘﹥宕岀紒鐘崇墪椤繘鎼圭憴鍕／闂侀潧枪閸庢煡鎮楁ィ鍐┾拺缂侇垱娲樺▍鍛存煙閾忣個顏囩亱闂佸憡鍔﹂崰鏍偂濞戙垺鐓曢悘鐐村劤閸ゎ剙霉閸忓吋宕屾慨濠勭帛閹峰懘鎳為妷褋鈧﹪姊洪崫銉バｉ柛鏃€鐟ラ锝夊川婵犲嫮鐦堝┑顔斤供閸撴盯顢欓崱娑欌拺缂備焦锚閻忓崬鈹戦鍝勨偓婵嗩嚕閺屻儱绠瑰ù锝呮贡閸樻悂姊虹粙鎸庢拱闁挎岸鏌嶈閸撴岸鎮у⿰鍫濈劦妞ゆ帊鑳堕崯鏌ユ煙閸戙倖瀚�
					if(motor_run_cmd == MOTOR_MODE_START) //motor control
					{						
						motor_run_cmd=MOTOR_MODE_STOP;
					}		
					menuPage=MENU_HOME_PAGE;//page change					
					MenuPageTurns(menuPage);
					App_MotorControl(MOTOR_MODE_STOP);						
				 	//if(rec_Signal==run_button_press_signal&&GC_depth_vlaue(0, 1)<=(3+sys_param_un.device_param.ref_tine)&&sys_param_un.device_param.apical_action_flag==2)//闂傚倸鍊搁崐宄懊归崶褉鏋栭柡鍥ュ灩缁愭鏌熼悧鍫熺凡闁告垹濮撮埞鎴︽偐鐎圭姴顥濈紓浣哄閸ㄥ爼寮婚敐澶婄闁挎繂鎲涢幘缁樼厸闁告侗鍠楅崐鎰版煛瀹€瀣瘈鐎规洘甯掕灒閻炴稈鈧厖澹曢梺鍝勭▉閸嬧偓闁稿鎸搁～婵嬵敆閸屾簽銊╂⒑鐠団€虫灀闁哄懐濞€楠炲﹤饪伴崨顓熺€冲┑鈽嗗灥濡椼劍绔熼弴銏♀拺閻犳亽鍔屽▍鎰版煙閸戙倖瀚�
				//	{//闂傚倸鍊搁崐宄懊归崶褉鏋栭柡鍥ュ灩缁愭鏌熼悧鍫熺凡闁告垹濮撮埞鎴︽偐鐎圭姴顥濈紓浣哄閸ㄥ爼寮婚敐澶婄闁挎繂鎲涢幘缁樼厸闁告侗鍠楅崐鎰版煛瀹€瀣瘈鐎规洘甯掕灒閻炴稈鈧厖澹曢梺鍝勭▉閸嬧偓闁稿鎸搁～婵嬵敆閸屾簽銊╂倵鐟欏嫭绀冮柛銊ф暬閵堫亝瀵奸弶鎴狀槹闂傚倸鐗婄粙鎴λ夊┑瀣拻闁稿本鐟︾粊鐗堢箾婢跺绀嬬€规洑鍗抽獮鍥级閸喖娈ゅ┑鐘垫暩婵敻鎳濋崜褏涓嶆い鏍仦閻撶喖鏌熸潏鍓у埌鐞氭岸姊洪崫鍕櫤闁诡喖鍊垮濠氭晸閻樿尙锛滃┑鈽嗗灥閵嗏偓闁稿鎹囧浠嬵敃閻旇渹澹曞┑顔斤供閸撴氨鎷归敓鐘崇厵闁肩⒈鍓欓。鑲╃磼濡ゅ啫鏋涢柛鈹惧亾濡炪倖甯掔€氼剟寮告笟鈧弻鐔煎箲閹伴潧娈┑鈽嗗亝閿曘垽寮婚弴銏犻唶婵犻潧娴傚Λ鐐差渻閵堝懐绠伴柣鏍帶椤繘鎼圭憴鍕／闂侀潧枪閸庢煡鎮楁ィ鍐┾拺闁告繂瀚烽崕蹇斻亜椤撶姴鍘撮柣娑卞枦缁犳稑鈽夊▎蹇婂亾婵犳碍鐓犻柟顓熷笒閸旀粍绻涢崨顐㈢伈婵﹥妞藉畷顐﹀礋椤愮喎浜鹃柛顐ｆ礀绾惧綊鏌″搴′簮闁稿鎸搁～婵嬵敆閸屾簽銊╂⒑閸濆嫯顫﹂柛鏃€鍨块獮鍐閵堝懎绐涙繝鐢靛Т鐎氼亞妲愰弴銏♀拻濞达絽鎽滅粔鐑樸亜閵夛附宕岀€规洘顨呴～婊堝焵椤掆偓椤曪綁顢曢敃鈧粻濠氭偣閸パ冪骇妞ゃ儲绻堝娲濞戞艾顣哄┑鈽嗗亝椤ㄥ﹪銆侀弮鍫濋唶闁哄洨鍟块幏娲煟閻樺厖鑸柛鏂跨焸瀵悂骞嬮敂鐣屽幐闁诲函缍嗘禍鍫曟偂閸忕⒈娈介柣鎰皺缁犲鏌熼鐣岀煉闁瑰磭鍋ゆ俊鐑芥晜缁涘鎴烽梻鍌氬€风粈渚€骞夐敓鐘茬濞达絿纭堕弸鏍煛閸ャ儱鐒洪柡浣告閺岋紕浠︾拠鎻掑缂備胶濯崹鍫曞蓟閵娾晜鍋嗛柛灞剧☉椤忥拷,闂傚倸鍊搁崐宄懊归崶褉鏋栭柡鍥ュ灩缁愭鏌熼悧鍫熺凡闁告垹濮撮埞鎴︽偐鐎圭姴顥濈紓浣哄閸ㄨ泛顫忓ú顏勫瀭妞ゆ洖鎳庨崜鍗烆渻閵囶垯绀佸ú锕傚煕閹达附鍋ｉ柟顓熷笒婵″ジ鏌＄€ｎ偄鐏撮柡宀嬬磿閳ь剨缍嗘禍鍫曟偂閸忕⒈娈介柣鎰皺缁犲鏌熼鐣岀煉闁瑰磭鍋ゆ俊鐑芥晜缁涘鎴烽梻鍌氬€风粈渚€骞栭锕€纾归柛顐ｆ礀绾惧綊鏌″搴′簮闁稿鎸搁～婵嬵敆閸屾簽銊╂⒑閸濆嫯顫﹂柛鏃€鍨块獮鍐閵堝懎绐涙繝鐢靛Т鐎氼亞妲愰弴銏♀拻濞达絽鎽滅粔鐑樸亜閵夛附宕岀€规洘顨呴～婊堝焵椤掆偓椤曪綁顢曢敃鈧粻濠氭偣閸パ冪骇妞ゃ儲绻堝娲濞戞艾顣哄┑鈽嗗亝椤ㄥ﹪銆侀弮鍫熸櫢闁跨噦鎷�
//					if(motor_run_cmd==MOTOR_MODE_STOP)  //motor control
//					{		
//							motor_run_cmd=MOTOR_MODE_START;
//					}	
//						App_MotorControl(motor_run_cmd);	
				//	}
				}	
				else if(rec_Signal==motor_apex_stop_signal) //闂傚倸鍊搁崐宄懊归崶褉鏋栭柡鍥ュ灩缁愭鏌熼悧鍫熺凡闁告垹濮撮埞鎴︽偐鐎圭姴顥濈紓浣哄閸ㄥ爼寮婚敐澶婄闁挎繂鎲涢幘缁樼厸闁告侗鍠楅崐鎰版煛鐏炲墽娲存鐐搭焽閹叉挳宕熼婧惧亾婵犲偆娓婚柕鍫濇缁楁岸鏌ｉ幙鍕瘈闁绘侗鍠涚粻娑樷槈濞嗗繆鍋撴繝姘厾闁诡厽甯掗崝婊勭箾閸涱偄鐏叉慨濠冩そ瀹曨偊宕熼鐔蜂壕闁割偅娲栫壕褰掓煛瀹ュ骸浜愰柛瀣尭椤繈顢楅崒婧炪劑姊洪崫鍕潶闁告梹鍨块獮鍐閵堝懎绐涙繝鐢靛Т鐎氼亞妲愰弴銏♀拻濞达絿鍎ら崵鈧梺鎼炲€栭悧鐘荤嵁韫囨稒鏅搁柨鐕傛嫹
				{
					apexMotorStartFlag=TRUE;//闂傚倸鍊搁崐宄懊归崶褉鏋栭柡鍥ュ灩缁愭鏌熼悧鍫熺凡闁告垹濮撮埞鎴︽偐鐎圭姴顥濈紓浣哄閸ㄥ爼寮婚敐澶婄闁挎繂鎲涢幘缁樼厸闁告侗鍠楅崐鎰版煛瀹€瀣瘈鐎规洘甯掕灒閻炴稈鈧厖澹曢梺鍝勭▉閸嬧偓闁稿鎸搁～婵嬵敆閸屾簽銊╂⒑閸濆嫯顫﹂柛鏃€鍨块獮鍐閵堝棗鈧粯淇婇姘倯婵炲牓绠栧娲嚒閵堝憛鈩冪箾閺夋垶鍠橀柨婵堝仦瀵板嫮浠︾粙澶稿濠殿喗锕╅崜娑氱矓濞差亝鐓涢悘鐐额嚙婵倿鏌熼鍝勭伈鐎规洏鍔嶇换婵嬪礋椤忓棛锛熼梻鍌氬€风粈浣虹礊婵犲泚澶愬箻鐠囪尙顦у┑顔姐仜閸嬫挾鈧鍣崑濠囩嵁濡吋瀚氶柛娆忣樈濡喖姊绘笟鈧褔鎮ч崱妞㈡稑螖閸滀焦鏅滈梺鍓插亝濞叉﹢鍩涢幋锔藉仯闁诡厽甯掓俊濂告煛鐎ｎ偄鐏撮柡宀嬬磿閳ь剨缍嗘禍鍫曟偂閸忕⒈娈介柣鎰皺缁犲鏌熼鐣岀煉闁瑰磭鍋ゆ俊鐑芥晜缁涘鎴烽梻鍌氬€风粈渚€骞栭锕€纾归柛顐ｆ礀绾惧綊鏌″搴′簮闁稿鎸搁～婵嬫倷椤掆偓椤忥拷	
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
					if(sys_param_un.device_param.apexFunctionLoad==0)//闂傚倸鍊搁崐宄懊归崶褉鏋栭柡鍥ュ灩缁愭鏌￠崶鈺佹灁缂佺娀绠栭弻娑㈠Ψ閵忊剝鐝栨俊鐐额潐婵炲﹪骞冨畡鎵虫瀻闊洦鎼╂禒鐓庮渻閵堝懐绠伴柣鏍帶椤繘鎼圭憴鍕／闂侀潧枪閸庢煡鎮楁ィ鍐┾拺闁告繂瀚烽崕蹇斻亜椤撶姴鍘撮柣娑卞枦缁犳稑鈽夊▎蹇婂亾婵犳碍鐓犻柟顓熷笒閸旀粍绻涢崨顐㈢伈婵﹥妞藉畷顐﹀礋椤愮喎浜鹃柛顐ｆ礀绾惧綊鏌″搴′簮闁稿鎸搁～婵嬵敆閸屾簽銊╂⒑閸濆嫯顫﹂柛鏃€鍨块獮鍐閿涘嫮鎳濆銈嗙墬閻熝兾涢鐐粹拻濞达絿鎳撻婊呯磼鐠囨彃鈧骞戦姀銈呯闁绘﹩鍋勬禍鐐叏濮楀棗澧扮紒澶嬫そ閺岋紕浠︾拠鎻掝潎闂佽鍠撻崹褰掓偩閿熺姵鐒介柨鏃€鏋荤槐鎺旂磽閸屾艾鈧兘鎮為敂鍓х煓闁哄稁鍙忕紞鏍ь熆鐠哄搫顦柛瀣尭椤繈顢楅崒婧炪劑姊洪崫鍕潶闁告梹鍨块獮鍐閵堝懎绐涙繝鐢靛Т鐎氼亞妲愰弴銏♀拻濞达絽鎽滅粔鐑樸亜閵夛附宕岀€规洘顨呴～婊堝焵椤掆偓椤曪綁顢曢敃鈧粻濠氭偣閸パ冪骇妞ゃ儲绻堝娲濞戞艾顣哄┑鈽嗗亝椤ㄥ﹪銆侀弮鍫濋唶闁哄洨鍠撻崢钘夆攽閳藉棗鐏犻柣蹇旂箞閹繝骞囬悧鍫㈠帗閻熸粍绮撳畷婊堝Ω閳轰胶顔嗛梺璺ㄥ櫐閹凤拷
					{	
						apexMotorStartFlag=TRUE;//闂傚倸鍊搁崐宄懊归崶褉鏋栭柡鍥ュ灩缁愭鏌熼悧鍫熺凡闁告垹濮撮埞鎴︽偐鐎圭姴顥濈紓浣哄閸ㄥ爼寮婚敐澶婄闁挎繂鎲涢幘缁樼厸闁告侗鍠楅崐鎰版煛瀹€瀣瘈鐎规洘甯掕灒閻炴稈鈧厖澹曢梺鍝勭▉閸嬧偓闁稿鎸搁～婵嬵敆閸屾簽銊╂⒑閸濆嫯顫﹂柛鏃€鍨块獮鍐閵堝棗鈧粯淇婇姘倯婵炲牓绠栧娲嚒閵堝憛鈩冪箾閺夋垶鍠橀柨婵堝仦瀵板嫮浠︾粙澶稿濠殿喗锕╅崜娑氱矓濞差亝鐓涢悘鐐额嚙婵倿鏌熼鍝勭伈鐎规洏鍔嶇换婵嬪礋椤忓棛锛熼梻鍌氬€风粈浣虹礊婵犲泚澶愬箻鐠囪尙顦у┑顔姐仜閸嬫挾鈧鍣崑濠囩嵁濡吋瀚氶柛娆忣樈濡喖姊绘笟鈧褔鎮ч崱妞㈡稑螖閸滀焦鏅滈梺鍓插亝濞叉﹢鍩涢幋锔藉仯闁诡厽甯掓俊濂告煛鐎ｎ偄鐏撮柡宀嬬磿閳ь剨缍嗘禍鍫曟偂閸忕⒈娈介柣鎰皺缁犲鏌熼鐣岀煉闁瑰磭鍋ゆ俊鐑芥晜缁涘鎴烽梻鍌氬€风粈渚€骞栭锕€纾归柛顐ｆ礀绾惧綊鏌″搴′簮闁稿鎸搁～婵嬫倷椤掆偓椤忥拷							
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
				xSemaphoreGive(xSemaphoreDispRfresh); //闂傚倸鍊搁崐宄懊归崶褉鏋栭柡鍥ュ灩缁愭鏌熼悧鍫熺凡闁告垹濮撮埞鎴︽偐鐎圭姴顥濈紓浣哄閸ㄥ爼寮婚敐澶婄闁挎繂鎲涢幘缁樼厸闁告侗鍠楅崐鎰版煛瀹€瀣瘈鐎规洘甯掕灒閻炴稈鈧厖澹曢梺鍝勭▉閸嬧偓闁稿鎸搁～婵嬵敆閸屾簽銊╂⒑閸濆嫯顫﹂柛鏃€鍨块獮鍐閵堝懎绐涙繝鐢靛Т鐎氼亞妲愰弴銏♀拻濞达絽鎽滅粔鐑樸亜閵夛附宕岀€规洘顨呴～婊堝焵椤掆偓椤曪綁顢曢敃鈧粻鑽ょ磽娴ｈ鐒介柛娆忓閺岋綁濮€閳轰胶浼囬梺鍝ュУ濮樸劎鍒掗弮鍫濋唶闁哄洨鍠撻崢鍨繆閻愬樊鍎忓Δ鐘虫倐瀹曘垽骞橀鐣屽帗閻熸粍绮撳畷婊冾潩椤掑鍍甸梺闈浥堥弲婊堝磻閸岀偞鐓ラ柣鏂挎惈瀛濈紓浣插亾闁糕剝绋掗悡鐔兼煙鏉堝墽鍒扮悮姘舵⒑閸濆嫭鍣洪柟顔煎€垮濠氭晸閻樿尙锛滃┑鈽嗗灥閵嗏偓闁稿鎹囧浠嬵敃閻旇渹澹曞┑顔筋焽閸嬫挾鈧熬鎷�
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
