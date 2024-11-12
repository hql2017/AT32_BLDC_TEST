/**
  **************************************************************************
  * @file     at32f413_board.c
  * @version  v1.0
  * @date     2022-02-22
  * @brief    set of firmware functions 
  **************************************************************************

  **************************************************************************
  */

#include "at32f413_board.h"
#include "at32f413.h"

uint32_t sys_time_flag;//系统主时钟戳 ms
/* delay macros */
#define STEP_DELAY_MS                    50

/* delay variable */
static __IO uint32_t fac_us;
static __IO uint32_t fac_ms;

/* support printf function, usemicrolib is unnecessary */


#ifdef USART_DEBUG
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
  * @brief  initialize uart
  * @param  baudrate: uart baudrate
  * @retval none
  */
void uart_print_init(uint32_t baudrate)
{
  gpio_init_type gpio_init_struct;

  #if defined (__GNUC__) && !defined (__clang__)
  setvbuf(stdout, NULL, _IONBF, 0);
  #endif

  /* enable the uart and gpio clock */
  crm_periph_clock_enable(PRINT_UART_CRM_CLK ,TRUE);
  crm_periph_clock_enable(PRINT_UART_TX_GPIO_CRM_CLK, TRUE);

  gpio_default_para_init(&gpio_init_struct);
  
  /* configure the uart tx pin */
  gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
  gpio_init_struct.gpio_out_type  = GPIO_OUTPUT_PUSH_PULL;
  gpio_init_struct.gpio_mode = GPIO_MODE_MUX;
  gpio_init_struct.gpio_pins = PRINT_UART_TX_PIN;
  gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
  gpio_init(GPIOB, &gpio_init_struct);

  /* configure the uart rx pin */
  //gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
  //gpio_init_struct.gpio_out_type  = GPIO_OUTPUT_PUSH_PULL;
  //gpio_init_struct.gpio_mode = GPIO_MODE_INPUT;
  //gpio_init_struct.gpio_pins = GPIO_PINS_11;
  //gpio_init_struct.gpio_pull = GPIO_PULL_UP;
  //gpio_init(GPIOB, &gpio_init_struct);

  /* configure uart param */
  usart_init(PRINT_UART, baudrate, USART_DATA_8BITS, USART_STOP_1_BIT);
  usart_transmitter_enable(PRINT_UART, TRUE);
  usart_enable(PRINT_UART, TRUE);


  /* configure uart param */
  //usart_init(USART3, baudrate, USART_DATA_8BITS, USART_STOP_1_BIT);
  //usart_parity_selection_config(USART3, USART_PARITY_NONE);
  //usart_transmitter_enable(USART3, TRUE);
  //usart_receiver_enable(USART3, TRUE);
  //usart_enable(USART3, TRUE);
  
  printf("run/r/n");
}
void user_uart_init(void)
{
  gpio_init_type gpio_init_struct;

  #if defined (__GNUC__) && !defined (__clang__)
  setvbuf(stdout, NULL, _IONBF, 0);
  #endif

  /* enable the uart and gpio clock */
  crm_periph_clock_enable(CRM_USART3_PERIPH_CLOCK ,TRUE);
  crm_periph_clock_enable(UART3TX_CLOCK, TRUE);

  gpio_default_para_init(&gpio_init_struct);
  
  /* configure the uart tx pin */
  gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
  gpio_init_struct.gpio_out_type  = GPIO_OUTPUT_PUSH_PULL;
  gpio_init_struct.gpio_mode = GPIO_MODE_MUX;
  gpio_init_struct.gpio_pins = UART3TX_IO;
  gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
  gpio_init(UART3TX_PORT, &gpio_init_struct);

  /* configure the uart rx pin */
  gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
  gpio_init_struct.gpio_out_type  = GPIO_OUTPUT_PUSH_PULL;
  gpio_init_struct.gpio_mode = GPIO_MODE_INPUT;
  gpio_init_struct.gpio_pins = UART3RX_IO;
  gpio_init_struct.gpio_pull = GPIO_PULL_UP;
  gpio_init(UART3RX_PORT, &gpio_init_struct);

  /* configure uart param */
  usart_init(USART3, 115200, USART_DATA_8BITS, USART_STOP_1_BIT);
  usart_parity_selection_config(USART3, USART_PARITY_NONE);
  usart_transmitter_enable(USART3, TRUE);
  usart_receiver_enable(USART3, TRUE);
  //usart_dma_transmitter_enable(USART3, TRUE);
  //usart_dma_receiver_enable(USART3, TRUE);
  /* Enable the USART3 */
  usart_enable(USART3, TRUE);

  nvic_irq_enable(USART3_IRQn, 2, 0);
  usart_interrupt_enable(USART3, USART_RDBF_INT, TRUE);
  //usart_interrupt_enable(USART3, USART_TDBE_INT, TRUE);
}


void config_uart_dma(void)
{
  dma_init_type dma_init_struct;

  /* enable dma1 clock */
  crm_periph_clock_enable(CRM_DMA1_PERIPH_CLOCK, TRUE);
  /* dma1 channel3 for usart3 tx configuration */
  dma_reset(DMA1_CHANNEL3);
  dma_default_para_init(&dma_init_struct);
  dma_init_struct.buffer_size = MAX_FIFO_LEN;
  dma_init_struct.direction = DMA_DIR_MEMORY_TO_PERIPHERAL;
  dma_init_struct.memory_base_addr = (uint32_t)uart_tx_buf;
  dma_init_struct.memory_data_width = DMA_MEMORY_DATA_WIDTH_BYTE;
  dma_init_struct.memory_inc_enable = TRUE;
  dma_init_struct.peripheral_base_addr = (uint32_t)&USART3->dt;
  dma_init_struct.peripheral_data_width = DMA_PERIPHERAL_DATA_WIDTH_BYTE;
  dma_init_struct.peripheral_inc_enable = FALSE;
  dma_init_struct.priority = DMA_PRIORITY_MEDIUM;
  dma_init_struct.loop_mode_enable = FALSE;
  dma_init(DMA1_CHANNEL3, &dma_init_struct);

  /* config flexible dma for usart3 tx */
  dma_flexible_config(DMA1, FLEX_CHANNEL3, DMA_FLEXIBLE_UART3_TX);

  /* dma1 channel2 for usart3 rx configuration */
  dma_reset(DMA1_CHANNEL2);
  dma_default_para_init(&dma_init_struct);
  dma_init_struct.buffer_size = MAX_FIFO_LEN;
  dma_init_struct.direction = DMA_DIR_PERIPHERAL_TO_MEMORY;
  dma_init_struct.memory_base_addr = (uint32_t)uart_rx_buf;
  dma_init_struct.memory_data_width = DMA_MEMORY_DATA_WIDTH_BYTE;
  dma_init_struct.memory_inc_enable = TRUE;
  dma_init_struct.peripheral_base_addr = (uint32_t)&USART3->dt;
  dma_init_struct.peripheral_data_width = DMA_PERIPHERAL_DATA_WIDTH_BYTE;
  dma_init_struct.peripheral_inc_enable = FALSE;
  dma_init_struct.priority = DMA_PRIORITY_MEDIUM;
  dma_init_struct.loop_mode_enable = FALSE;
  dma_init(DMA1_CHANNEL2, &dma_init_struct);

  /* config flexible dma for usart3 rx */
  dma_flexible_config(DMA1, FLEX_CHANNEL2, DMA_FLEXIBLE_UART3_RX);

  dma_channel_enable(DMA1_CHANNEL2, TRUE); /* usart3 rx begin dma receiving */
  dma_channel_enable(DMA1_CHANNEL3, TRUE); /* usart3 tx begin dma transmitting */
}

void send_data(u8 *buf, u16 len)
{
  memcpy(uart_tx_buf, buf, len);
  
  while(usart_flag_get(USART3, USART_TDBE_FLAG) == RESET);
  
  dma_channel_enable(DMA1_CHANNEL3, FALSE);
  
  dma_data_number_set(DMA1_CHANNEL3, len);
  
  dma_channel_enable(DMA1_CHANNEL3, TRUE);
}

#endif


//void wdt_init(void)
//{
//  /* disable register write protection */
//  wdt_register_write_enable(TRUE);  
//  
//  /* set the wdt divider value */
//  wdt_divider_set(WDT_CLK_DIV_4);
//  
//  /* set reload value  
//  
//   timeout = reload_value * (divider / lick_freq )    (s)
//  
//   lick_freq    = 40000 Hz
//   divider      = 4
//   reload_value = 3000
// 
//   timeout = 3000 * (4 / 40000 ) = 0.3s = 300ms
//  */
//  wdt_reload_value_set(3000 - 1);
//  
//  /* enable wdt */ 
//  wdt_enable();
//}




/**
  * @brief 中止
  * @param  none
  * @retval none
  */
void Suspend_RTOS_stick(void)
{
	  /* Disable SysTick Interrupt */
//	SysTick->CTRL &= ~SysTick_CTRL_TICKINT_Msk;	
	SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;
}
/**
  * @brief  恢复
  * @param  none
  * @retval none
  */
void Resume_RTOS_stick(void)
{
	/* Enable SysTick Interrupt */
//	SysTick->CTRL |= SysTick_CTRL_TICKINT_Msk ;	
	SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk ;
}

/**
  * @brief  initialize delay function   
  * @param  none
  * @retval none
  */
void delay_init(unsigned int tickFrequency)
{
	
	system_core_clock=(200000000U);//200M,AHB
  /* configure systick */
  systick_clock_source_config(SYSTICK_CLOCK_SOURCE_AHBCLK_DIV8);//200M,for freeRTOS
	
  fac_us =( system_core_clock /8)/ (1000000U);//tickFrequency=1000000U;1us
  fac_ms = fac_us * (1000U);//tickFrequency=1000U;1ms
	
	SysTick->LOAD =( (system_core_clock/8) / tickFrequency);//(uint32_t)(1 * fac_ms);
  SysTick->VAL = 0x00;
	SysTick->CTRL |= SysTick_CTRL_TICKINT_Msk ;
}

/**
  * @brief  inserts a delay time.
  * @param  nus: specifies the delay time length, in microsecond.
  * @retval none
  */
void delay_us(uint32_t nus)
{
  uint32_t temp = 0;
  SysTick->LOAD = (uint32_t)(nus * fac_us);
  SysTick->VAL = 0x00;
  SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk ;
  do
  {
    temp = SysTick->CTRL;
  }while((temp & 0x01) && !(temp & (1 << 16)));

  SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;
  SysTick->VAL = 0x00;
}

/**
  * @brief  inserts a delay time.
  * @param  nms: specifies the delay time length, in milliseconds.
  * @retval none
  */
void delay_ms(uint16_t nms)
{
  uint32_t temp = 0;
  while(nms)
  {
    if(nms > STEP_DELAY_MS)
    {
      SysTick->LOAD = (uint32_t)(STEP_DELAY_MS * fac_ms);
      nms -= STEP_DELAY_MS;
    }
    else
    {
      SysTick->LOAD = (uint32_t)(nms * fac_ms);
      nms = 0;
    }
    SysTick->VAL = 0x00;
    SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;
    do
    {
      temp = SysTick->CTRL;
    }while((temp & 0x01) && !(temp & (1 << 16)));

    SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;
    SysTick->VAL = 0x00;
  }
}

/**
  * @brief  inserts a delay time.
  * @param  sec: specifies the delay time, in seconds.
  * @retval none
  */
void delay_sec(unsigned short int sec)
{
  uint16_t index;
  for(index = 0; index < sec; index++)
  {
    delay_ms(500);
    delay_ms(500);
  }
}


/**
  * @}
  */

/**
  * @}
  */

