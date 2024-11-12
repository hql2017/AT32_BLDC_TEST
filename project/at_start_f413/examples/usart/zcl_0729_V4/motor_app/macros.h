#ifndef _MACROS_H_
#define _MACROS_H_

#include "at32f413_tmr.h"
#ifndef ZHX_MOTOR
#define ZHX_MOTOR
#endif

#ifdef ZHX_MOTOR
//Motor Parameters
#define m_p 	1						//pole pairs of the motor
#define m_Vin 8						//(V)DC input voltage
#define m_Rs	1.0875			//(ohm)winding resistor(phase-phase)
#define m_Ld	64//54//82//123					//(uH)winding d axis inductor
#define m_Lq	82//61//92//147					//(uH)winding q axis inductor
#define m_J		1//0.65						//(g*cm^2)rotor inertia 
#define m_speed   11000//11700		//(rpm)rated speed of the motor

//Load Parameters
#define m_gear_ratio	6		//gearbox ratio, set to 1 if no gearbox is used.
#define m_TL	0						//(mNm) load torque
#define m_JL	0						//(g*cm^2)load inertia

#define FS	40//20//16				//switching frequency(kHz)
#define DEADTIME	300			//ns
#define PWMMODE		0				//0:ENx+PWMx  1:GLx+GHx(6 gate signals)
 
#else 
//Motor Parameters
#define m_p 	1						//pole pairs of the motor
#define m_Vin 7						//(V)DC input voltage
#define m_Rs	0.41//0.45//1.65				//(ohm)winding resistor(phase-phase)
#define m_Ld	51					//(uH)winding d axis inductor
#define m_Lq	51					//(uH)winding q axis inductor
#define m_J		1						//(g*cm^2)rotor inertia 
#define m_speed	7200			//(rpm)rated speed of the motor

//Load Parameters
#define m_gear_ratio	6		//gearbox ratio, set to 1 if no gearbox is used.
#define m_TL	0						//(mNm) load torque
#define m_JL	0						//(g*cm^2)load inertia

#define FS	20//40 						//switching frequency(kHz)
#define DEADTIME	300			//ns
#define PWMMODE		0				//0:ENx+PWMx  1:GLx+GHx(6 gate signals)
#endif

// AD Sample
#define AD_MOD		1				//0 current sensing resistors, 1 MP654x series
#define AD_GAIN		7				//7:2x  6:3x 5:4x 4:5x 3:6x 2:7x 1:8x 0:12
#define I_OCP			0x3FF		//OCP register value
#define m_Rpull		1000		//(ohm) current sending pull-up resistor when using MP654x power stage(
#define m_Rsense 	50			//(mohm) crrent sensing resistor when using MP653x as power stage

#define ERROR_RESET  	
#define ERROR_SET   		

#define mp6570_enable()				gpio_bits_set(GPIOB,GPIO_MP6570_EN)//GPIOB->BSRR = 0x00000002  // GPIO_WriteOutBits(EN_OUT_GPIO_ID, EN_OUT_AFIO_PIN, SET) //set EN_6570
#define mp6570_disable()		  gpio_bits_reset(GPIOB,GPIO_MP6570_EN)//	GPIOB->BSRR = 0x00020000

//#define NSS_SET				        GPIOA->BSRR = 0x00000010;
//#define NSS_RESET			        GPIOA->BSRR = 0x00100000;

//	nSLEEP_SET; //enable pre-driver nsleep
//	nSLEEP_RESET; //enable pre-driver nsleep

#define PEND_SET			  
#define PEND_RESET		

#define LOOPTIMERINT_ENABLE		__NOP//NVIC_EnableIRQ(SCTM1_IRQn)
#define LOOPTIMER_ENABLE   tmr_counter_enable(TMR3, TRUE)
#define	LOOPTIMER_DISABLE	  tmr_counter_enable(TMR3, FALSE)




#endif
