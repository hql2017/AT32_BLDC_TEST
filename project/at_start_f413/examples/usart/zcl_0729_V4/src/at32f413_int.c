/**
  **************************************************************************
  * @file     at32f413_int.c
  * @version  v2.0.7
  * @date     2022-08-16
  * @brief    main interrupt service routines.
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

/* includes ------------------------------------------------------------------*/
#include "at32f413_int.h"

#include "adc_t.h"
#include "uart_init.h"
#include "base_time.h"
#include "para_list.h"
#include "adc_port.h"
/** @addtogroup AT32F413_periph_examples
  * @{
  */

/** @addtogroup 413_USART_printf
  * @{
  */

/**
  * @brief  this function handles nmi exception.
  * @param  none
  * @retval none
  */
void NMI_Handler(void)
{
}

/**
  * @brief  this function handles hard fault exception.
  * @param  none
  * @retval none
  */
void HardFault_Handler(void)
{
  /* go to infinite loop when hard fault exception occurs */
  while(1)
  {
  }
}

/**
  * @brief  this function handles memory manage exception.
  * @param  none
  * @retval none
  */
void MemManage_Handler(void)
{
  /* go to infinite loop when memory manage exception occurs */
  while(1)
  {
  }
}

/**
  * @brief  this function handles bus fault exception.
  * @param  none
  * @retval none
  */
void BusFault_Handler(void)
{
  /* go to infinite loop when bus fault exception occurs */
  while(1)
  {
  }
}

/**
  * @brief  this function handles usage fault exception.
  * @param  none
  * @retval none
  */
void UsageFault_Handler(void)
{
  /* go to infinite loop when usage fault exception occurs */
  while(1)
  {
  }
}

/**
  * @brief  this function handles svcall exception.
  * @param  none
  * @retval none
  */
//void SVC_Handler(void)
//{
//}

/**
  * @brief  this function handles debug monitor exception.
  * @param  none
  * @retval none
  */
void DebugMon_Handler(void)
{
}

/**
  * @brief  this function handles pendsv_handler exception.
  * @param  none
  * @retval none
  */
//void PendSV_Handler(void)
//{
//}

/**
  * @brief  this function handles systick handler.
  * @param  none
  * @retval none
  */
//void SysTick_Handler(void)
//{
//}
extern uint8_t adc_trans_complete_flag ;
extern uint8_t spi2_tx_dma_status ;
void DMA1_Channel1_IRQHandler(void)
{
  if(dma_flag_get(DMA1_FDT1_FLAG) != RESET)
  {
    dma_flag_clear(DMA1_FDT1_FLAG);
    if(adc_trans_complete_flag==0)
		{
			adc_trans_complete_flag = 1;	
		}
//dma_channel_enable(DMA1_CHANNEL1, FALSE); 		
  }
  //printf("DMA 1 IR\r\n");
}

void DMA2_Channel1_IRQHandler(void) //USART1 TX
{
  if(dma_flag_get(DMA1_FDT2_FLAG))
  {
    usart_tx_dma_status = 1;
    dma_flag_clear(DMA2_FDT1_FLAG);
		dma_channel_enable(DMA2_CHANNEL1, FALSE);    
  }
}
void DMA2_Channel2_IRQHandler(void) //spi2 TX
{
  if(dma_flag_get(DMA2_FDT2_FLAG))
  {
    spi2_tx_dma_status = 1;
    dma_flag_clear(DMA2_FDT2_FLAG);
		dma_channel_enable(DMA2_CHANNEL2, FALSE);    
  }
}
void DMA2_Channel3_IRQHandler(void)//USART3 TX
{
  if(dma_flag_get(DMA2_FDT3_FLAG))
  {
//    usart3_tx_dma_status = 1;
    dma_flag_clear(DMA2_FDT3_FLAG);
		dma_channel_enable(DMA2_CHANNEL3, FALSE);
  }
}
/**
  * @brief  this function handles dma1 channel6 handler.
  * @param  none
  * @retval none
  */
void DMA1_Channel6_IRQHandler(void)
{
  if(dma_flag_get(DMA1_FDT6_FLAG))
  {
  
    dma_flag_clear(DMA1_FDT6_FLAG);
//    dma_channel_enable(DMA1_CHANNEL6, FALSE);
	
  }
}

void DMA1_Channel7_IRQHandler(void)
{
  if(dma_flag_get(DMA1_FDT7_FLAG))
  {
    dma_flag_clear(DMA1_FDT7_FLAG);  
  }
}

void EXINT1_IRQHandler(void)
{
  //GPIO_EXTI_IRQHandler(GPIO_PINS_1);
}

void EXINT2_IRQHandler(void)
{
  //GPIO_EXTI_IRQHandler(GPIO_PINS_2);
}

void EXINT3_IRQHandler(void)
{
  //GPIO_EXTI_IRQHandler(GPIO_PINS_3);
}

/**
  * @}
  */ 
void TMR1_TRG_HALL_TMR11_IRQHandler(void)
{
  if(tmr_flag_get(TMR1, TMR_OVF_FLAG) != RESET)
  {   
    tmr_flag_clear(TMR1, TMR_OVF_FLAG);
  } 
}

extern uint8_t normal_time_complete_flag ;
void TMR4_GLOBAL_IRQHandler(void)
{	
	if(tmr_flag_get(TMR4, TMR_OVF_INT) != RESET)
  {	
		tmr_flag_clear(TMR4, TMR_OVF_INT);
		normal_time_complete_flag=1;	
//		tmr_counter_enable(TMR4,FALSE);		
  } 
}
void TMR3_GLOBAL_IRQHandler(void)
{	
	if(tmr_flag_get(TMR3, TMR_OVF_INT) != RESET)
  {
		tmr_flag_clear(TMR3, TMR_OVF_INT);
		MOTOR_150US_Callback();   //150us  	
  } 
}

/**
  * @brief  exint0 interrupt handler
  * @param  none
  * @retval none
  */
void EXINT9_5_IRQHandler(void)
{
}

/**
  * @}
  */ 
