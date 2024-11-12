#ifndef __PID_MATH_H_
#define __PID_MATH_H_

//typedef struct 
//{
//  float kp;
//	float ki;
//	float kd;
//	float pre;
//	float tar;
//	float err;
//	float lastBias;
//	float out;
//	float intergal_sum;
//	float intergal_max;
//	float intergal_min;
//  float outMax;
//	float outmin;
//}CURRENT_PID_DEF_W;

//typedef struct 
//{
//    float kp;           //比例
//    float kd;           //微分
//    float ki;           //积分
//	  float kb;
//    float tar;          //目标
//    float pre;          //实际
//    float err;         //目标与实际之前差值（目标-实际）
//    float last_err;    //上次差值
//	  float last_last_err;//上上次误差
//    float out;          //输出
//	  float outMax;       //输出限幅
//	  float outMin;
//	  float intergal_sum;
//	  float intergal_max;
//	  float intergal_min;
//	  float speed_out;
//}SPEED_PID_DEF;

//extern SPEED_PID_DEF  Speed_pid;

////extern void CURRENT_PID_W(float pre,float tar,CURRENT_PID_DEF_W* I,float* out);
//extern void SPEED_PID(float pre,float tar,SPEED_PID_DEF* S);

#endif
