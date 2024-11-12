/*
*@file        vofa.c
*@brief       scop
*@attention   none
*@version     V1.0
*@data        2022/5/27
*@author      xuwenxi
*/

#include "vofa.h"
#include "stdint.h"
#include "at32f413.h"
#include "hall_sensor.h"
#include "motor_app.h"

static float tempfloat[3];
static uint8_t tempData[16]={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0x80,0x7f};
uint16_t i;
uint8_t vofa_flag=0;
void Vofa_Updata(void)
{

	if(vofa_flag==0){
	tempfloat[0]=	Ia;
	tempfloat[1]=	Ib;
	tempfloat[2]=	Ic;
	}
	else if(vofa_flag==1){
	//tempfloat[0]=	Hall_handle.Angle;
	//tempfloat[1]=	Hall_handle.Speed/2.0F;
	//tempfloat[2]=	pSvpwm.sector;
	}
  else if(vofa_flag==2){
	//tempfloat[0]=	voltage_alpha_beta.Ualpha;
	//tempfloat[1]=	voltage_alpha_beta.Ubeta;
	//tempfloat[2]=	hall_temp;
	}
  else if(vofa_flag==3)
	{
		tempfloat[0]=	TMR1->c1dt;
		tempfloat[1]=	TMR1->c2dt;
		tempfloat[2]=	TMR1->c3dt;
	}
  memcpy(tempData,(uint8_t*)tempfloat,sizeof(tempfloat) ); 
  for(i=0;i<16;i++)	{
	while(usart_flag_get(USART3, USART_TDBE_FLAG) == RESET){}
  usart_data_transmit(USART3, tempData[i]);
	USART3->dt = (tempData[i] & 0x01FF);

 }
}


