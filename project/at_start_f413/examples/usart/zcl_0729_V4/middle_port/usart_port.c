
#include "stdio.h"
#include "string.h"
#include "at32f413_usart.h"
#include "at32f413_gpio.h"
#include "at32f413_board.h"

#include "usart_port.h"
#include "uart_init.h"
#include "oled_port.h"

#include "app_queue.h"

#define USART_TX_PIN           GPIO_PINS_9 
#define USART_RX_PIN           GPIO_PINS_10
#define USART_CRM_CLK          CRM_USART1_PERIPH_CLOCK
#define USART_TX_GPIO_CRM_CLK  CRM_GPIOA_PERIPH_CLOCK

#define PRINT_UART  USART1	
uint8_t uart_tx_buf[MAX_FIFO_LEN];
uint8_t uart_rx_buf[MAX_FIFO_LEN];
 
 QUEUE_HandleTypeDef  usart_hqueue;
#ifndef USART_DEBUG
#if (__ARMCC_VERSION > 6000000)
  __asm (".global __use_no_semihosting\n\t");
  void _sys_exit(int x)
  {
    x = x;
  }
  /* __use_no_semihosting was requested, but _ttywrch was */
  void _ttywrch(int ch)
  {
    ch = ch;
  }
  FILE __stdout;
#else
 #ifdef __CC_ARM
  #pragma import(__use_no_semihosting)
  struct __FILE
  {
    int handle;
  };
  FILE __stdout;
  void _sys_exit(int x)
  {
    x = x;
  }
  /* __use_no_semihosting was requested, but _ttywrch was */
  void _ttywrch(int ch)
  {
    ch = ch;
  }
 #endif
#endif
#if defined (__GNUC__) && !defined (__clang__)
  #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
  #define PUTCHAR_PROTOTYPE  int  fputc(int ch, FILE *f)
#endif

/**
  * @brief  retargets the c library printf function to the usart.
  * @param  none
  * @retval none
  */
PUTCHAR_PROTOTYPE
{
#if 0
	while(usart_flag_get(PRINT_UART, USART_TDBE_FLAG) == RESET);
	usart_data_transmit(PRINT_UART, ch);

#else
	static uint32_t status;
	//while(usart_flag_get(PRINT_UART, USART_TDBE_FLAG) == RESET);
	do
	{
	status = PRINT_UART->sts;
	}while((status & USART_TDBE_FLAG) == RESET);

	//usart_data_transmit(PRINT_UART, ch);
	PRINT_UART->dt = (ch & 0x01FF);

#endif
  	return ch;
}
/**
  * @brief  initialize uart gpio
  * @param  baudrate: uart baudrate
  * @retval none
  */
void uart_gpio_init(uint32_t baudrate)
{
  gpio_init_type gpio_init_struct;

  #if defined (__GNUC__) && !defined (__clang__)
  setvbuf(stdout, NULL, _IONBF, 0);
  #endif

  /* enable the uart and gpio clock */
  crm_periph_clock_enable(USART_CRM_CLK,TRUE);
  crm_periph_clock_enable(USART_TX_GPIO_CRM_CLK, TRUE);

  gpio_default_para_init(&gpio_init_struct);
  
  /* configure the usart tx pin  PA9*/
  gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
  gpio_init_struct.gpio_out_type  = GPIO_OUTPUT_PUSH_PULL;
  gpio_init_struct.gpio_mode = GPIO_MODE_MUX;
  gpio_init_struct.gpio_pins = USART_TX_PIN;
  gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
  gpio_init(GPIOA, &gpio_init_struct);

  /* configure the usart rx pin pA10 */
  gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
  gpio_init_struct.gpio_out_type  = GPIO_OUTPUT_PUSH_PULL;
  gpio_init_struct.gpio_mode = GPIO_MODE_INPUT;
  gpio_init_struct.gpio_pins = USART_RX_PIN;
  gpio_init_struct.gpio_pull = GPIO_PULL_UP;
  gpio_init(GPIOA, &gpio_init_struct);

  /* configure uart param */
  usart_init(USART1, baudrate, USART_DATA_8BITS, USART_STOP_1_BIT);
  usart_transmitter_enable(USART1, TRUE);	
	//		usart_dma_transmitter_enable(USART1,TRUE);//DMA，开发送DMA
	usart_receiver_enable(USART1, TRUE); 
	nvic_irq_enable(USART1_IRQn, 4, 0);
	usart_interrupt_enable(USART1, USART_RDBF_INT, TRUE);	
	usart_enable(USART1, TRUE);	 	
}

void config_uart_dma(void)
{
  dma_init_type dma_init_struct;

  /* enable dma1 clock */
  crm_periph_clock_enable(CRM_DMA2_PERIPH_CLOCK, TRUE);
  /* dma1 channel2 for usart1 tx configuration */
  dma_reset(DMA2_CHANNEL1);
  dma_default_para_init(&dma_init_struct);
  dma_init_struct.buffer_size = MAX_FIFO_LEN;
  dma_init_struct.direction = DMA_DIR_MEMORY_TO_PERIPHERAL;
  dma_init_struct.memory_base_addr = (uint32_t)uart_tx_buf;
  dma_init_struct.memory_data_width = DMA_MEMORY_DATA_WIDTH_BYTE;
  dma_init_struct.memory_inc_enable = TRUE;
  dma_init_struct.peripheral_base_addr = (uint32_t)&USART1->dt;
  dma_init_struct.peripheral_data_width = DMA_PERIPHERAL_DATA_WIDTH_BYTE;
  dma_init_struct.peripheral_inc_enable = FALSE;
  dma_init_struct.priority = DMA_PRIORITY_MEDIUM;
  dma_init_struct.loop_mode_enable = FALSE;
  dma_init(DMA2_CHANNEL1, &dma_init_struct);

  /* config flexible dma for usart1 tx */
	usart_dma_transmitter_enable(USART1,TRUE);//DMA，usart TX开发送DMA

  dma_flexible_config(DMA2, FLEX_CHANNEL1, DMA_FLEXIBLE_UART1_TX);
  dma_channel_enable(DMA2_CHANNEL1, TRUE); /* usart1 tx begin dma transmitting */	
	dma_interrupt_enable(DMA2_CHANNEL1,DMA_FDT_INT,TRUE);
	nvic_irq_enable(DMA2_Channel1_IRQn, 0, 0);
  /* dma1 channel3 for usart1 rx configuration */
//	dma_reset(DMA1_CHANNEL3);
//  dma_default_para_init(&dma_init_struct);
//  dma_init_struct.buffer_size = MAX_FIFO_LEN;
//  dma_init_struct.direction = DMA_DIR_MEMORY_TO_PERIPHERAL;
//  dma_init_struct.memory_base_addr = (uint32_t)uart_rx_buf;
//  dma_init_struct.memory_data_width = DMA_MEMORY_DATA_WIDTH_BYTE;
//  dma_init_struct.memory_inc_enable = TRUE;
//  dma_init_struct.peripheral_base_addr = (uint32_t)&USART1->dt;
//  dma_init_struct.peripheral_data_width = DMA_PERIPHERAL_DATA_WIDTH_BYTE;
//  dma_init_struct.peripheral_inc_enable = FALSE;
//  dma_init_struct.priority = DMA_PRIORITY_MEDIUM;
//  dma_init_struct.loop_mode_enable = FALSE;
//  dma_init(DMA1_CHANNEL3, &dma_init_struct);

//  /* config flexible dma for usart1 tx */
//  dma_flexible_config(DMA1, FLEX_CHANNEL3, DMA_FLEXIBLE_UART1_RX);
//  dma_channel_enable(DMA1_CHANNEL3, TRUE); /* usart1 tx begin dma transmitting */	
//	nvic_irq_enable(DMA1_Channel3_IRQn, 0, 0);
}
void user_uart_init(void)
{  
  #if defined (__GNUC__) && !defined (__clang__)
  setvbuf(stdout, NULL, _IONBF, 0);
  #endif	
	uart_gpio_init(9600);//38400);
	config_uart_dma();
	Queue_Init(&usart_hqueue,(QUEUE_DATA_T*)uart_rx_buf, MAX_FIFO_LEN);	
}
void usart_send_data(u8 *buf, u16 len)
{  	
	while(usart_flag_get(USART1, USART_TDC_FLAG) == RESET){;} //	
	memcpy(uart_tx_buf, buf, len); 	
  dma_channel_enable(DMA2_CHANNEL1, FALSE); 	 	 
  dma_data_number_set(DMA2_CHANNEL1, len);  
  dma_channel_enable(DMA2_CHANNEL1, TRUE);	 
}

/**
  * @brief AppGetUsartData
  * @param  none
  * @retval none
  */

void  AppGetUsartData(unsigned short int *pBuff)
{	
	unsigned short int rec;
	QUEUE_StatusTypeDef err;
	err=Queue_Pop(&usart_hqueue,(QUEUE_DATA_T*)&rec);	// out FIFO
	if(err ==QUEUE_OK)
	{
		*pBuff=rec;
	}	
	else 
	{		//no data 
		*pBuff=0;
	}	
}

void USART1_IRQHandler(void)
{		
	uint16_t data;
	if(usart_flag_get(USART1, USART_RDBF_FLAG) != RESET)               
	{
		usart_flag_clear (USART1, USART_RDBF_FLAG );
		data =	usart_data_receive (USART1);
		if(0x04<data&&data<0x0A)
		{
			Queue_Push(&usart_hqueue,data);	// save FIFO
		}
	}		
}  
/**
  * @brief  App usart transmit
  * @param   
  * @retval none
  */
 void AppUsartTransmit(unsigned char *buf,unsigned short int len)
{	
	usart_send_data(buf, len);
}

#endif





