#include "customer_control.h"
#include "mp6570.h"
#include "control.h"
#include "init.h"
#include "macros.h"
#include "delay.h"

#include "gpio_port.h"
#include "math.h"

#ifdef DEBUG_RTT
#include "SEGGER_RTT.h"
#endif
//************AT32f413*****************
#ifdef  ZHX
#define NOT_DIRECTION_REVERSE//正反转交换方向		
#endif

#define CURRENT_REVERSE_PARAM     1.041//反向电流修正，forward=1;
extern uint8_t menu_motor_run_mode; //电机运行模式,0，正向，1逆向，2正反转（角度调节）

void PID_select(uint16_t PID_P,uint16_t PID_I);

//*****************************
unsigned short status_mp6570 = 0;

unsigned short forward_speed; 
unsigned short reverse_speed;
int	forward_position;
int reverse_position;
unsigned short upper_threshold;		//iq upper threshold
unsigned short lower_threshold;
u16 threshold_times = 3000;   //delay counter x * 100us
u16 reach_upper_times = 0;
u16 reach_lower_times = 0;
MotorStatus_TypeDef motor_status;
MotorSettings_TypeDef motor_settings;

unsigned char torque_reach(void);
unsigned char toggle_torque_reach(void);
void set_torque_limit(float upper_limit, float lower_limit); //set torque limit in speed mode 
void set_speed(int forward, int reverse);//set forward and backward speed
void set_position(int forward, int reverse);//set forward and backward position
void set_toggle_mode_speed(int speed);
void mode_select(enum EndoMode mode);//set working mode. 0:speed mode, 1: position back and forth mode;

/*****************************set working mode********************************************
  * @brief  Set working mode.
  * @param  mode: EndoModePositionToggle or EndoModeSpeedAutoReverse
  * @retval None
*****************************************************************************************/
void mode_select(enum EndoMode mode)
{
	motor_status.mode = mode;
	if(mode == EndoModePositionToggle)
		regi[0x34] = 0x0131;
    else
        regi[0x34] = 0x0130;
}

/*****************************start the motor********************************************
  * @brief  start the motor.
  * @param  None
  * @retval None
*****************************************************************************************/
void start()
{
		regi[0x70] = 0x0001;
		motor_status.status = Status_START;	
}
/*****************************stop the motor********************************************
  * @brief  stop the motor.
  * @param  None
  * @retval None
*****************************************************************************************/
void stop()
{
	regi[0x70] = 0x0000;
	motor_status.status = Status_STOP;	
}

/*****************************set torque limit in speed mode*****************************
  * @brief  set torque limit in speed mode.
  * @param  upper_limit: when the motor torque higher than this value, motor start to turn backward
	* @Param	lower_limit: when the motor torque lower than this value, motor start to turn forward
  * @retval None
*****************************************************************************************/
void set_torque_limit(float upper_limit, float lower_limit)
{
	if(lower_limit<0.4)  lower_limit=0.4;	
	#ifdef ZHX
		upper_threshold = upper_limit *90;//略高
		lower_threshold = lower_limit *90;//93;//up5%;//略高
	#else
		upper_threshold = upper_limit *130;//鸣志
		lower_threshold = lower_limit *130;//	
	#endif	
}
void set_toggle_mode_speed(int speed)
{
	regi[0x4C] = (speed*7*m_gear_ratio)>>6;
}
/*****************************set forward and backward speed******************************
  * @brief  set forward and backward speed
	* @param  forward: forward speed. 
	* @param  reverse: reverse speed. 
*****************************************************************************************/
void set_speed(int forward, int reverse)
{
	forward_speed = (forward * 7*m_gear_ratio)>>6;
	reverse_speed = (reverse * 7*m_gear_ratio)>>6;
}

/*****************************set forward and backward speed******************************
  * @brief  set speed ramp slope.
	* @param  rpmPerMs: speed slope(unit rpm/ms), range 1-91rpm/ms. 
  * @retval None
*****************************************************************************************/
void set_speed_slope(int rpmPerMs)
{
	regi[0x4F] = rpmPerMs*716;
}

/*****************************set forward and backward position******************************
  * @brief  set forward and backward position
	* @param  forward: forward position. 
	* @param  reverse: reverse position. 
  * @retval None
*****************************************************************************************/
void set_position(int forward, int reverse)
{	
	forward_position = forward * 182 * m_gear_ratio;
	reverse_position = reverse * 182 * m_gear_ratio;
}
/**
  * @brief  Obtain the motor speed.
  * @param  None.
  * @retval the motor speed in rpm.
  */
int get_motor_speed(void)
{
  int32_t speed_fbk;
	uint32_t speed_temp;	
	speed_temp = (regi[0x62]<<16) + regi[0x61];	
	#ifdef  ZHX
	if(!(speed_temp & 0x80000000))
		speed_fbk = speed_temp / 11000;
	else
		speed_fbk = -(4294967296 - speed_temp)  /  11000;
	#else
	if(!(speed_temp & 0x80000000))
		speed_fbk = speed_temp /7158;
	else
		speed_fbk = -(4294967296 - speed_temp)  /7158;
	#endif	
	
	return speed_fbk;
}

/**
  * @brief  Obtain the motor round.
  * @param  None.
  * @retval the motor angle.
  */
int16_t get_postion_round(void)
{
	return regi[0x5F];
}

/**
  * @brief  Obtain the motor angle.
  * @param  None.
  * @retval the motor angle in degree.
  */
uint16_t get_position_angle(void)
{
	return regi[0x60]/182;
}

/*****************************customer control logic******************************
  * @brief  customer control logic function
						Note: Don't add delay in this function and any function called by this function should not
						has delay.
	* @param  None. 
  * @retval None
*****************************************************************************************/

void customer_control()
{
  static int delay_cnt = 0;
	status_mp6570 = MP6570_ReadRegister(0x00,0x54);
	motor_status.reach_target = regi[0x69] & 0x0001;  //whether motor reach target position
	if(motor_settings.mode==EndoModePositionToggle)
	{	
		motor_status.reach_torque = toggle_torque_reach();			
	}
	else motor_status.reach_torque = torque_reach();				//whether motor reach target torque
		//往复模式	
	if(motor_status.mode==EndoModePositionToggle)
	{
			if(motor_status.reach_target && (motor_status.status != Status_STOP))
			{	
				if(motor_status.status == Status_START)
				motor_status.status = Status_FORWARD;	
				if(motor_status.status == Status_REVERSE)
				{
					regi[0x4A] = (forward_position>>16); //position rounds
					regi[0x4B] = forward_position; 
					motor_status.status = Status_FORWARD;
				}
				else if(motor_status.status == Status_FORWARD)
				{
					regi[0x4A] = reverse_position>>16;
					regi[0x4B] = reverse_position; 
					motor_status.status = Status_REVERSE;
				}
				update_command = 1;	
			}
			else if(motor_status.reach_torque==1)//Keep Reverse)
			{
//				stop();
			}				
	}					
	//Keep Reverse
	else if(motor_status.mode == EndoModeSpeedReverse)
	{		
		if(motor_status.status == Status_START)	{		
			motor_status.status = Status_REVERSE;
		}
		else if(motor_status.status == Status_FORWARD)
		{
			regi[0x4d] = forward_speed;	
			update_command = 1;			
			if(motor_status.reach_torque == 2)   {		//回复	
					motor_status.status = Status_REVERSE;						
			}				
		}
		else if(motor_status.status == Status_REVERSE)
		{
			regi[0x4d] = reverse_speed;	
			update_command = 1;			
			if(motor_status.reach_torque == 1)   {		//反转			
				motor_status.status = Status_FORWARD;				
			}			
		}	
	}		
	 //Keep Forward
	else if(motor_status.mode==EndoModeSpeedForward)
	{	
		if(motor_status.status == Status_START)	{	
			motor_status.status = Status_FORWARD;
		}
		else if(motor_status.status == Status_FORWARD)
		{
			regi[0x4d] = forward_speed;	
			update_command = 1;				
			if(motor_status.reach_torque == 1)   {		//力矩反转									
					motor_status.status = Status_REVERSE;														
			}				
		}
		else if(motor_status.status == Status_REVERSE)
		{
			regi[0x4d] = reverse_speed;
			update_command = 1;					
			if(motor_status.reach_torque == 2)   {		//回复							
					motor_status.status = Status_FORWARD;							
			}	
		}							
	}	
//Keep Reverse
	else if(motor_status.mode == EndoModeKeepReverse)
	{		
		if(motor_status.status == Status_STOP)
		{
			delay_cnt++;
			if(delay_cnt >= 100) //delay 1000 ticks = 100ms
			{
				start();
				delay_cnt = 0;
			}
		}
		if(motor_status.status == Status_START)	{		
			motor_status.status = Status_REVERSE;
		}
		else if(motor_status.status == Status_FORWARD)	{ 
			motor_status.status = Status_REVERSE;
		}
		if(motor_status.status == Status_REVERSE)	
		{ 
			regi[0x4d] = reverse_speed;
			update_command = 1;					
		}					
	}		
	//Keep Reverse
	else if(motor_status.mode == EndoModeKeepForward)
	{	
		if(motor_status.status == Status_STOP)
		{
			delay_cnt++;
			if(delay_cnt >= 100) //delay 1000 ticks = 100ms
			{
				start();
				delay_cnt = 0;
			}
		}		
		if(motor_status.status == Status_START)	{		
			motor_status.status = Status_FORWARD;
		}
		else if(motor_status.status == Status_REVERSE)	{ 
			motor_status.status = Status_FORWARD;
		}
		 if(motor_status.status == Status_FORWARD)	 
		{
			regi[0x4d] = forward_speed;				
			update_command = 1;					
		}		
	}		
}		
void update_settings(MotorSettings_TypeDef *setting)
{
	mode_select(setting->mode);
	#ifdef NOT_DIRECTION_REVERSE//CW，CCW交换方向
		set_speed(setting->forward_speed, setting->reverse_speed);
		set_position(setting->forward_position, setting->reverse_position);		
	#else
		set_speed(setting->reverse_speed, setting->forward_speed);
		set_position(setting->reverse_position, setting->forward_position);	
	#endif
	set_torque_limit(setting->upper_threshold, setting->lower_threshold);
	set_toggle_mode_speed(setting->toggle_mode_speed);		
}

unsigned char torque_reach(void)
{	
	u8 status = 0; 							// 1- reach_upper  2- reach_lower  0: middle-state
	threshold_times = 3000;
	if ((iq > upper_threshold)||(iq < -(upper_threshold*CURRENT_REVERSE_PARAM)))
	{
		reach_upper_times ++;
		if(reach_upper_times >=threshold_times)
		{
			reach_upper_times = threshold_times;
			reach_lower_times = 0;
			status = 1; 
		}
	}
	if((iq < lower_threshold)&&(iq > -(lower_threshold*CURRENT_REVERSE_PARAM)))
	{
		reach_lower_times ++;
		if(reach_lower_times >= threshold_times)
		{
			reach_lower_times = threshold_times;
			reach_upper_times = 0;
			status = 2;
		}
	}
	return status;
}

unsigned char toggle_torque_reach(void)
{	
	u8 status = 0; 							// 1- reach_upper  2- reach_lower  0: middle-state
	u8 check_torque = 0;
	threshold_times = 350;
	if(((motor_settings.forward_position > - motor_settings.reverse_position)&&(motor_status.status == Status_FORWARD)) || ((motor_settings.forward_position < - motor_settings.reverse_position)&&(motor_status.status == Status_REVERSE)))
		check_torque = 1;
	else
		check_torque = 0;
	if(check_torque)
	{
		if ((iq > upper_threshold)||(iq < -(upper_threshold*CURRENT_REVERSE_PARAM)))
		{
			reach_upper_times ++;
			if(reach_upper_times >=threshold_times)
			{
				reach_upper_times = threshold_times;
				reach_lower_times = 0;
				status = 1; 
			}
		}
		if((iq < lower_threshold)&&(iq > -(lower_threshold*CURRENT_REVERSE_PARAM)))
		{
			reach_lower_times ++;
			if(reach_lower_times >= threshold_times)
			{
				reach_lower_times = threshold_times;
				reach_upper_times = 0;
				status = 2;
			}
		}
		
	}
	return status;
}
void init_registers()
{
	regi[0x00] = 0x6570; 			//Part Number
	#ifdef ZHX_MOTOR//20K开关频率
	regi[0x01] = 1087;					//Phase resistor mOhm
	regi[0x02] = 68; 					//Phase Inductor uH
	regi[0x03] = 20; 					//Power rating W
	regi[0x04] = 1; 					// Inertia g*cm^2
	regi[0x05] = 11000;//11700; 				//Nominal Speed rpm
	regi[0x06] = 6;    //7 				//Voltage V
	regi[0x07] = 5;  					//rated torque mNm
	regi[0x08] = 1;    				//Pole Pairs p
	regi[0x09] = 8;//7;   		//Maximum input voltage
	regi[0x0A] = 1;    				//Nominal current Arms
	regi[0x0B] =  3;   				//Peak current Arms
	regi[0x0C] = 220;  				//Input Capacitor uF
	regi[0x0D] = 10;   				//Sensing resistor mOhm
	regi[0x0E] = 7;    				//MOS Ron
	regi[0x0F] = 1;    				//interface information 0: RS485 1: RS485+PULSE
	regi[0x10] = 0;    				//Predriver-type 0:MP1907A
	regi[0x11] = 0x0101;			//PCB version
	regi[0x12] = 0;						//reserved	
	regi[0x13] = 0;     			//reserved
	regi[0x14] = 0x0051;			//Software version 
	regi[0x15] = 12;    			//resolution
	regi[0x16] = 100;   			//INL
	regi[0x17] = 20000; 			//maximum rpm
	regi[0x18] = 1;     			//minimum rpm
	regi[0x19] = 0;						//reserved
	#else
	regi[0x01] = 520;					//Phase resistor mOhm
	regi[0x02] = 49; 					//Phase Inductor uH
	regi[0x03] = 20; 					//Power rating W
	regi[0x04] = 1; 					// Inertia g*cm^2
	regi[0x05] = 7180; 				//Nominal Speed rpm
	regi[0x06] = 7;   				//Voltage V
	regi[0x07] = 8;  					//rated torque mNm
	regi[0x08] = 1;    				//Pole Pairs p
	regi[0x09] = 6;   				//Maximum input voltage
	regi[0x0A] = 1;    				//Nominal current Arms
	regi[0x0B] = 3;   				//Peak current Arms
	regi[0x0C] = 220;  				//Input Capacitor uF
	regi[0x0D] = 10;   				//Sensing resistor mOhm
	regi[0x0E] = 7;    				//MOS Ron
	regi[0x0F] = 1;    				//interface information 0: RS485 1: RS485+PULSE
	regi[0x10] = 0;    				//Predriver-type 0:MP1907A
	regi[0x11] = 0x0101;			//PCB version
	regi[0x12] = 0;						//reserved	
	regi[0x13] = 0;     			//reserved
	regi[0x14] = 0x0051;			//Software version 
	regi[0x15] = 12;    			//resolution
	regi[0x16] = 100;   			//INL
	regi[0x17] = 20000; 			//maximum rpm
	regi[0x18] = 1;     			//minimum rpm
	regi[0x19] = 0;						//reserved
	#endif	
	
	#ifdef ZHX_MOTOR
	regi[0x1A] = 0x04F3;			//position loop Kp
	regi[0x1B] = 0x000E;			//position loop Kp gain
	regi[0x1C] = 0x04B1;			//position loop Maximum Limit
	#else
	regi[0x1A] = 0x04F3;			//position loop Kp
	regi[0x1B] = 0x000E;			//position loop Kp gain
	regi[0x1C] = 0x04B1;			//position loop Maximum Limit
	#endif	
	
	regi[0x1D] = 0x001F;			//s-Curve accelaration rate
	regi[0x1E] = 3500;				//Position reach target threshold
	regi[0x1F] = 0;						//reserved
	regi[0x20] = 0;						//reserved
	regi[0x21] = 0;						//reserved
	#ifdef ZHX_MOTOR
	regi[0x22] = 0x4B52;//2804;			//Speed loop Kp
	regi[0x23] = 0x800E;			//Speed loop Kp gain
	regi[0x24] = 0x3C94;//405D;			//Speed loop Ki
	regi[0x25] = 0x8015;			//Speed loop Ki gain
	#else
	regi[0x22] = 0x08D2;			//Speed loop Kp
	regi[0x23] = 0x800C;			//Speed loop Kp gain
	regi[0x24] = 0x0E30;			//Speed loop Ki
	regi[0x25] = 0x8014;			//Speed loop Ki gain
	#endif	
	regi[0x26] = 1000;				//Kc anti-integral gain
	regi[0x27] = 0x03C0;  		//Speed loop maximum limit
	regi[0x28] = 0;						//res erved
	regi[0x29] = 0x000A;			//Speed reach target threshold
	regi[0x2A] = 0;						//reserved
	#ifdef ZHX_MOTOR
	regi[0x2B] = 0x041C;			//Current KP
	regi[0x2C] = 0x1A4D;			//Current Ki
	#else 
	regi[0x2B] = 0x02A5;			//Current KP
	regi[0x2C] = 0x109A;			//Current Ki
	#endif	
	regi[0x2D] = 0x0064;			//high 16 HSB of maximum speed in torque mode, 
	regi[0x2E] = 10;					//Speed limit gain in torque mode
	regi[0x2F] = 0x0000;			//Torque reference IQ
	regi[0x30] = 0x0000;			//Torque reference ID
	regi[0x31] = 0;						//reserved
	regi[0x32] = 0;						//reserved
	regi[0x33] = 0x007F;			//reserved
	regi[0x34] = 0x0131; 			//Control Mode[0:1](00:speed, 01:position, 02:torque) reference Mode[2:3](00:digital, 10:Pulse, 11: PWM)
														//Position Mode[4](1:relative, 0:absolute) [5]: 1=standby mode; 0=non-standby   [7]: NSTEP set by switchs
	regi[0x35] = 0;			
	regi[0x36] = 0;						//reserved	
	regi[0x37] = 0;						//brake IQ current
	regi[0x38] = 0;						//min. brake speed
	regi[0x39] = 0;						//reserved
	regi[0x3A] = 0;						//reserved
	regi[0x3B] = 0;						//reserced
	regi[0x3C] = 0;						//reserved
	regi[0x3D] = 0;						//reserved
	regi[0x3E] = 0x00C8;			//I hold mA 
	regi[0x3F] = 1000;				//T hold ms
	
	regi[0x40] = 0x202B;			//position tracking filter alpha
	regi[0x41] = 0x0008;			//position tracking filter beta 
	regi[0x42] = 0x6082;			//speed tracking filter alpha
	regi[0x43] = 0x0018;			//speed tracking filter beta
	regi[0x44] = 0x6082;			//speed rc filter alpha
	regi[0x45] = 0;						//
	regi[0x46] = 0;						//
	regi[0x47] = 0;						//
	regi[0x48] = 0;						//
	regi[0x49] = 0;						//
	regi[0x4A] = 0x0000;			//poistion reference turns
	regi[0x4B] = 0x0000;			//position reference angle
	regi[0x4C] = 0x04B1;			//position reference slope LSB/100us
	regi[0x4D] = 0x00DA;			//Speed reference LSB/100us  high 16bits
	regi[0x4E] = 0x740D;			//Speed reference `` low 16bits
	regi[0x4F] = 0x3863;			//Speed reference slope
	
	regi[0x50] = 0x0007;			//reserved
	regi[0x51] = 0;					// reserved
	regi[0x52] = 0;					// reserved
	regi[0x53] = 0;					//fault status,[3]: lock, [2]: ocp, [1]: PSFT_6570, [0]: Memory fault_6570
	regi[0x54] = 0x0000;			//fault retry timer
	regi[0x55] = 0x0F01;			//stall detection timer       
	regi[0x56] = 0x00F3;			//VDC protection
	#ifdef ZHX_MOTOR
	regi[0x57] = 40;				//switching frequency kHz
	#else
	regi[0x57] = 20;				//switching frequency kHz
	#endif	

	regi[0x58] = 32;				//Deadtime 12.5*x ns
	regi[0x59] = 7;					//AD_Gain
	regi[0x5A] = 0x03FF;			//OCP current 
	regi[0x5B] = 0;					//reserved
	regi[0x5C] = 0x12;  			//reserved
	regi[0x5D] = 3600;  			//reserved
	regi[0x5E] = 0;     			//reserved
	regi[0x5F] = 0;     			//reserved
	
	regi[0x60] = 0;    				//sensor position after tracking filter
	regi[0x61] = 0;    				//sensor speed after tracking filter and rc filter low 16bits
	regi[0x62] = 0;    				//sensor speed after tracking filter and rc filter high 16bits
	regi[0x63] = 0;    				//IQ_LATCH
	regi[0x64] = 0;    				//ID_LATCH
	regi[0x65] = 0;    				//UQ_LATCH
	regi[0x66] = 0;    				//UD_LATCH
	regi[0x67] = 0x061D;    	//THETA_BIAS
	regi[0x68] = 0x0001;    	//THETA_DIR
	regi[0x69] = 0;    				//[0]:Reach position indication,Position; [1]:Reach speed indication.
	regi[0x6A] = 0;    				//system state, [4]: auto_aligning finished, [0]: 1=run 
	regi[0x6B] = 0;    				//
	regi[0x6C] = 0;    				//
	regi[0x6D] = 0;		 				//
	regi[0x6E] = 0;		 				//
	regi[0x6F] = 0;    				//
	
	regi[0x70] = 0;  					//EN/DISABLE  00: Disable 01: ENable
	regi[0x71] = 0;  					//BRAKE/RUN
	regi[0x72] = 0;  					//reserved
	regi[0x73] = 0;  					//reserved
	regi[0x74] = 0;  					//reserved
	regi[0x75] = 0;  					//reserved
	regi[0x76] = 0;  					//update command
	regi[0x77] = 0;  					//reserved
	regi[0x78] = 0;  					//reserved
	regi[0x79] = 0;  					//reserved
	regi[0x7A] = 0;  					//reserved
	regi[0x7B] = 0;  					//reserved
	regi[0x7C] = 0;  					//reserved
	regi[0x7D] = 0;  					//reserved
	regi[0x7E] = 0;  					//reserved
	regi[0x7F] = 0;  					//reserved
}

/**
  * @brief  MotorDeviceReset
  * @param  none
  * @retval none
  */
void MotorDeviceReset(void)
{
	init_registers();
//	nSLEEP_SET; //enable pre-driver nsleep
	delay_1ms(1);	
	mp6570_enable();  //enable mp6570
	delay_1ms(2);
	mp6570_init(0x00);
	MP6570_WriteRegister(0x00,0x60,0x0000);
	delay_1ms(100);				//delay_1ms 100ms wait for sensor to be stable	
	reset_parameters();
//LOOPTIMER_ENABLE; //enable loop timer
}
/**
  * @brief  GetRealTorque
  * @param  none
  * @retval none
  */
unsigned short int GetRealTorque(void)
{
	unsigned  int retValue;
  unsigned int torqueValue;
	static 	unsigned short int torqueValueBuff[4]={0};
	static unsigned char num;
	num++;
	num%=4;
	if(iq<0) 
	{	
//		retValue=(~iq);//扭矩校准参数(0.1偏低0.1；-0.1偏高0.10)
		torqueValueBuff[num]=(~iq);
	}
	else 
	{		
//		retValue=(iq);//扭矩校准参数(0.1偏低0.1 ；-0.1偏高0.1)
		torqueValueBuff[num]=iq*CURRENT_REVERSE_PARAM;
	}	//filter	
	#ifdef ZHX//中韩鑫电机
	torqueValue=(torqueValueBuff[0]+torqueValueBuff[1]+torqueValueBuff[2]+torqueValueBuff[3])/36;//ZHX;10/(93*4);
	#else
	torqueValue=(torqueValueBuff[0]+torqueValueBuff[1]+torqueValueBuff[2]+torqueValueBuff[3])/51;//10/(130*4);
	#endif
	retValue=torqueValue;	
	return retValue;
}
