/**
  **************************************************************************
  * @file     main.c
  * @version  v2.0.7
  * @date     2022-08-16
  * @brief    main program
  **************************************************************************
  *                       Copyright notice & Disclaimer
  *
  * The software Board Support Package (BSP) that is made available to 
  * download from Artery official website is the copyrighted work of Artery. 
  * Artery authorizes customers to use, copy, and distribute the BSP 
  * software and its related documentation for the purpose of design and 
  * development in conjunction with Artery microcontrollers. Use of the 
  * software is governed by this copyright notice and the following disclaimer.
  *
  * THIS SOFTWARE IS PROVIDED ON "AS IS" BASIS WITHOUT WARRANTIES,
  * GUARANTEES OR REPRESENTATIONS OF ANY KIND. ARTERY EXPRESSLY DISCLAIMS,
  * TO THE FULLEST EXTENT PERMITTED BY LAW, ALL EXPRESS, IMPLIED OR
  * STATUTORY OR OTHER WARRANTIES, GUARANTEES OR REPRESENTATIONS,
  * INCLUDING BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY,
  * FITNESS FOR A PARTICULAR PURPOSE, OR NON-INFRINGEMENT.
  *
  **************************************************************************
  */
	
	
	/**
  **************************************************************************
  * @file     main.c
  * @version  v2.0.7
  * @date     2022-08-16
  * @brief    main program
  **************************************************************************
  *                       Copyright notice & Disclaimer
  *
  * The software Board Support Package (BSP) that is made available to
  * download from Artery official website is the copyrighted work of Artery.
  * Artery authorizes customers to use, copy, and distribute the BSP
  * software and its related documentation for the purpose of design and
  * development in conjunction with Artery microcontrollers. Use of the
  * software is governed by this copyright notice and the following disclaimer.
  *
  * THIS SOFTWARE IS PROVIDED ON "AS IS" BASIS WITHOUT WARRANTIES,
  * GUARANTEES OR REPRESENTATIONS OF ANY KIND. ARTERY EXPRESSLY DISCLAIMS,
  * TO THE FULLEST EXTENT PERMITTED BY LAW, ALL EXPRESS, IMPLIED OR
  * STATUTORY OR OTHER WARRANTIES, GUARANTEES OR REPRESENTATIONS,
  * INCLUDING BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY,
  * FITNESS FOR A PARTICULAR PURPOSE, OR NON-INFRINGEMENT.
  *
  **************************************************************************
  */





#include "at32f413_clock.h"
#include "stdint.h"
#include "limits.h"
#include <stdio.h>
#include "stdlib.h"
#include <string.h>

#include <assert.h>
#include "para_list.h"
#include "common_function.h"
#include "time_init.h"
#include "at32f413_board.h"

#include "usart_port.h"
#include "adc_port.h"
#include "gpio_port.h"
#include "main_thread.h"
#include "app_MotorControlTask.h"
#include "control.h"
#include "mp6570.h"
#include "customer_control.h"

#include "delay.h"
#include "wdt.h"
#include "oled_port.h"
#include "key.h"

#include "buzz_port.h"


#include "app_user_storage.h"


	
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

#ifdef   APEX_FUNCTION_EBABLE
#include "apex_gc_port.h"

typedef struct{
	int no8kValue;
	int no400Value;
	unsigned short int refPoint;
	
}GC_SYSPARAM;
GC_SYSPARAM GC_SysParam;
#endif

#define START_STACK_DEPTH 64
#define START_TASK_PRIORITY 1
TaskHandle_t StartTask_Handle;	
	
extern void vTaskStart( void * pvParameters );	

#define EXTEND_64KSRAM 0
#define EXTEND_16KSRAM 1
#define EXTEND_32KSRAM 3

#define SIGNAL_BUFFER_SIZE  64

#ifdef __cplusplus
extern "C" {
#endif


#ifdef __cplusplus
}
#endif
 
volatile unsigned   char usart_rx_dma_status ;
volatile unsigned   char usart_tx_dma_status ;

/** @addtogroup 413_SRAM_extend_sram SRAM_extend_sram
  * @{
  */

/**
  * @brief  to extend sram size
  * @param  none
  * @retval none
  */
 void extend_sram(void)
{
  /* check if ram has been set to expectant size, if not, change eopb0 */
  if(((USD->eopb0) & 0xFF) != EXTEND_64KSRAM)
  {
    flash_unlock();
    /* erase user system data bytes */
    flash_user_system_data_erase();

    /* change sram size */
    flash_user_system_data_program((uint32_t)&USD->eopb0, EXTEND_64KSRAM);

    /* system reset */
    nvic_system_reset();
  }
}
/**
  * @brief  DevicePowerOnSequence
  * @param  none
  * @retval none
  */
static void DevicePowerOnSequence(void)
{
// power check	
	delay_1ms(500);	//postpone 1.5S -1S
  device_power_on();	
	//the preparatory work has been completed	
	BuzzrSimpleSwitch(1);//close buzzer
	delay_1ms(150);	
	BuzzrSimpleSwitch(0);//close buzzer
}
/**
  * @brief  peripheral config
  * @param  none
  * @retval none
  */
static void peripheral_init(void)
{  
		unsigned short int err;  	
		BuzzerInit();		
		ButtonInit();
		user_gpio_init();		
		start_adc_acquisition(); 	
//mp6570
		err=MP6570_PortInit();    
		if(err==0xFFFF)//IIC 模块		
		{
			MP6570_SetModeToSPI();
		}	
		MotorDeviceReset();
		MotorParamInit();
		MotorTimerInit(100,SYSTEM_CLOCK_FREQUENCY);
		//end mp6570
		#ifdef APEX_FUNCTION_EBABLE
			APEXInit();		
		#else //单马达
			user_uart_init();
			config_uart_dma();		
		#endif	
//oled	
		OLED_Init();	
}
/**
  * @brief  DeviceParamCheck
  * @param  none
  * @retval none
  */
static void DeviceParamCheck(void)
{  
	if(get_gc_state())
	{		
		sys_param_un.device_param.apexFunctionLoad=0;		
	}
	else
	{
		sys_param_un.device_param.apexFunctionLoad=1;		
	}
}
int main(void)
{
	SystemInit ();
	system_clock_config();
	extend_sram();
	nvic_priority_group_config(NVIC_PRIORITY_GROUP_4);
#ifndef RTOS_UNUSED
	delay_init(configTICK_RATE_HZ);
	NormalTimerInit(1,SYSTEM_CLOCK_FREQUENCY);//for delay1ms
	Suspend_RTOS_stick(); // stop  stick  before   RTOS   created
#else 
		ConfigSys_Timer(SYSTEM_TIME_BASE_MS,SYSTEM_CLOCK_FREQUENCY);//配置系统计时TMR4时基为5ms	
#endif		
		peripheral_init();	
		start_para_write_read();
		DeviceParamCheck();
		DevicePowerOnSequence();		
#ifdef WDT_ENABLE			
		Wdt_Init();				
#endif 		
#ifdef DEBUG_RTT
		SEGGER_RTT_ConfigUpBuffer(0, NULL, NULL, 0, SEGGER_RTT_MODE_BLOCK_IF_FIFO_FULL);
		SEGGER_RTT_WriteString(0, "PDO-1\r\n");	
#endif
		xTaskCreate(vTaskStart, "vTaskStart", START_STACK_DEPTH, NULL, START_TASK_PRIORITY,&StartTask_Handle);			
		vTaskStartScheduler();
	  for(;;)	{;}			
}
///////////END Line//////////////////



