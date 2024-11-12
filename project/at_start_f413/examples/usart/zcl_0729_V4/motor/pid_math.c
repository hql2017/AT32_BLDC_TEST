/*
*@file        adc_t.c
*@brief       applation
*@attention   none
*@version     V1.0
*@data        2022/5/24
*@author      xuwenxi
*/
//#include "pid_math.h"

//#define PID_TS_I 1
//#define PID_TS_S 1
//#define SPEED_PID_MODE 1

//SPEED_PID_DEF  Speed_pid;

//void CURRENT_PID_W(float pre,float tar,CURRENT_PID_DEF_W* I,float* out)
//{
//  I->pre =pre;//测量值
//  I->tar =tar;//目标值
//  I->err=I->tar-I->pre;
//  I->out =I->kp*I->err + I->intergal_sum;  
//	if(I->out > I->outMax )
//	{
//	   I->out = I->outMax;
//	}
//	if(I->out < I->outmin)
//	{
//	  I->out = I->outmin;	
//	}	
//	#if PID_TS_I
//	  I->intergal_sum +=(I->ki*I->err)/40000.0f;
//	#else
//	   I->intergal_sum +=(I->ki*I->err);
//	#endif
//  	if(I->intergal_sum > I->intergal_max )
//	{
//	  I->intergal_sum=I->intergal_max;	
//	}
//  if(I->intergal_sum < I->intergal_min )
//	{
//	  I->intergal_sum = I->intergal_min;
//	}
//	*out=I->out;
//}

//#if SPEED_PID_MODE
//void SPEED_PID(float pre,float tar,SPEED_PID_DEF* S)
//{
//	S->pre =pre;//实际值
//	S->tar =tar;//目标
//	S->err =(S->tar - S->pre)*2*3.1415926F/60.0F;
////		S->err =(S->tar - S->pre);
//  S->out =S->kp*S->err + S->intergal_sum ; 
//  	if(S->out > S->outMax )
//	{
//	   S->out = S->outMax;
//	}
//	if(S->out < S->outMin )
//	{
//	  S->out = S->outMin;	
//	}
//	if(S->speed_out<S->out)
//	{
//	  S->speed_out=S->out;
//	}
//	#if PID_TS_S
// 	  S->intergal_sum +=((S->ki*S->err)/1000.0f);
//	#else 
//	   S->intergal_sum +=((S->ki*S->err));
//	#endif
//  if(S->intergal_sum > S->intergal_max )
//	{
//	  S->intergal_sum=S->intergal_max;	
//	}
//  if(S->intergal_sum < S->intergal_min )
//	{
//	  S->intergal_sum = S->intergal_min;
//	}
//}

//#else 
// void SPEED_PID(float pre,float tar,SPEED_PID_DEF* S)
// {
//   	S->pre =pre;//实际值
//	  S->tar =tar;//目标
//	  S->err =S->tar - S->pre;
//    S->out += S->kp*(S->err  - S->last_err ) + S->ki * S->err;
//    S->last_err = S->err;
//	 	if (S->out > S->outMax) {
//		     S->out = S->outMax;
//	} else if (S->out < S->outMin) {
//		     S->out =S->outMin;
//	} 
// }

////#endif



