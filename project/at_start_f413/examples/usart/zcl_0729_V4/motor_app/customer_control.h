#ifndef _CUSTOMER_CONTROL_H_
#define _CUSTOMER_CONTROL_H_
#include <stdint.h>


#ifndef ZHX   
#define ZHX//电机选择
#endif

extern unsigned short status_mp6570;

enum AutoReverseMode
{
    AutoReverseMode1, //达到转矩后反转，泄力后正转 
    AutoReverseMode2, //达到转矩反转，泄力后停止
    AutoReverseStop,  //达到转矩就停止
};
enum EndoMode
{
		EndoModePositionToggle,     //Two position toggle mode
		EndoModeSpeedForward,       //Keep Forward	
		EndoModeSpeedReverse,       //Keep Reverse	
		EndoModeTorqueATC,  //扭力自适应
		Max_endoMode,
		EndoModeKeepForward,       //Keep Forward	
		EndoModeKeepReverse,       //Keep Reverse	
};

typedef enum
{ 
    Status_STOP = 0x0,        
    Status_START = 0x01,
    Status_FORWARD = 0x02,
    Status_REVERSE = 0x03,	
   
}Status_TypeDef;

typedef struct
{
	Status_TypeDef status; 
	enum EndoMode mode;
	unsigned char need_reverse; //0:no need to reverse  1:need to revese
	unsigned char reach_target;
	unsigned char reach_torque;
	unsigned char delay_cnt;		//delay counter

} MotorStatus_TypeDef;

typedef struct
{
	enum EndoMode mode;
	enum AutoReverseMode autorev_mode;
	int forward_speed; 
	int reverse_speed;
	int	forward_position;
	int reverse_position;
	float upper_threshold;		//iq upper threshold
	float lower_threshold;
	int toggle_mode_speed;
	
} MotorSettings_TypeDef;

void start(void); //start the motr
void stop(void);  //stop the motor
void set_speed_slope(int rpmPerMs);
int get_motor_speed(void);
uint16_t get_position_angle(void);
int16_t get_postion_round(void);

void init_registers(void);//init register values
void customer_control(void);//customer control code, this function will be called by the 100us interrupt 

void update_settings(MotorSettings_TypeDef *setting);

//
extern void MotorDeviceReset(void);
extern  unsigned short int GetRealTorque(void);




#endif
