#include "delay.h"
#include "at32f413_clock.h"
#include "time_init.h"
#include "at32f413_board.h"
#include <string.h>
#include <stdio.h>
uint8_t normal_time_complete_flag = 0;
/* delay macros */

//void delay_1ms( int ms)
//{
//	int i = 0;
//	while(ms--)
//	{
//		i = 8000;
//		while(i--);
//	}
//}
//void delay_1us(int us)
//{
//	int i = 0;
//	while(us--)
//	{
//		i = 7*2;
//		while(i--);
//	}
//}
//Ö÷Æµ200M
void delay_1ms( int ms)
{
//	int i = 0;	
//	while(ms--)
//	{
//		i = 48000+1500;		
//		while(i--);
//	}
	normal_time_complete_flag=0;
	tmr_counter_enable(TMR4,TRUE);	
	while(ms--)
	{
		normal_time_complete_flag=0;
		while(normal_time_complete_flag==0)
		{
			;
		}
	}
		tmr_counter_enable(TMR4,FALSE);
}
void delay_1us( int us)
{
	int i = 0;
	while(us--)
	{
		i = 24*2;//24*2
		while(i--);
	}
}
