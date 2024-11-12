#ifndef __APP_MOTORCONTROLTASK
#define __APP_MOTORCONTROLTASK


//#define MOTOR_MODE_CW            	0 //顺时针，达到力矩反转，
//#define MOTOR_MODE_CCW            1 //逆时针，达到力矩反转，
//#define MOTOR_MODE_REC          	2 //往复运动，可调节角度，达到力矩反转
//#define MOTOR_MODE_TORPUE_REV     3  //扭力自适应（ATC）可调
////#define MOTOR_MODE_ATC          4 //扭力自适应
#define MOTOR_MODE_STOP         	0
#define MOTOR_MODE_START        	1
#define MOTOR_MODE_SEARCH_ANGLE   5  //找初始角度
#define MOTOR_SETTING_UPDATE   6  //配置更新
#define MOTOR_MODE_RESTART        	7
#define MOTOR_USART_GC_CONTROL      8//单马达串口根测控制
#define MOTOR_SETTING_ERR   9  //err

extern void MotorParamInit(void);

extern void motor_run_status( unsigned char run_flag);
extern unsigned char App_MotorControl(unsigned char cmd);	

#endif
