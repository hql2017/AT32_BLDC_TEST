/*
*@file        uart.c
*@brief       applation
*@attention   none
*@version     V1.0
*@data        2022/5/24
*@author      xuwenxi
*/
 #include "stdint.h"
 #include "uart_init.h"
 #include "at32f413.h"
 #include "para_list.h"


 #include "main.h"
volatile uint8_t usart_tx_dma_status = 0;
volatile uint8_t usart_rx_dma_status = 0;
u8 uart_tx_buf[MAX_FIFO_LEN] ;
u8 uart_rx_buf[MAX_FIFO_LEN] ;
volatile u8 uart_rx_flag=0;
UI_CTRL_DEF ui;
 
//void UART_DMA_Init(void )
//{
//	dma_init_type dma_init_struct;

//   /* enable dma1 clock */
//  crm_periph_clock_enable(CRM_DMA1_PERIPH_CLOCK, TRUE);
//	/* dma1 channel1 for usart2 tx configuration */
//  dma_reset(DMA1_CHANNEL2);
//  dma_default_para_init(&dma_init_struct);
//  dma_init_struct.buffer_size = MAX_FIFO_LEN;
//  dma_init_struct.direction = DMA_DIR_MEMORY_TO_PERIPHERAL;
//  dma_init_struct.memory_base_addr = (uint32_t)uart_tx_buf;
//  dma_init_struct.memory_data_width = DMA_MEMORY_DATA_WIDTH_BYTE;
//  dma_init_struct.memory_inc_enable = TRUE;
//  dma_init_struct.peripheral_base_addr = (uint32_t)&USART3->dt;
//  dma_init_struct.peripheral_data_width = DMA_PERIPHERAL_DATA_WIDTH_BYTE;
//  dma_init_struct.peripheral_inc_enable = FALSE;
//  dma_init_struct.priority = DMA_PRIORITY_LOW;
//  dma_init_struct.loop_mode_enable = TRUE ;//不循环发生，只有接受到了正确的数据才会回答
//  dma_init(DMA1_CHANNEL2, &dma_init_struct);   
//	  /* enable transfer full data intterrupt */
//  dma_interrupt_enable(DMA1_CHANNEL2, DMA_FDT_INT,TRUE);
//  /* dma1 channel1 interrupt nvic init */
//  nvic_irq_enable(DMA1_Channel2_IRQn, 2, 0);
//  /* config flexible dma for usart2 tx */
//  dma_flexible_config(DMA1, FLEX_CHANNEL2, DMA_FLEXIBLE_UART3_TX);

//     /* dma1 channel2 for usart2 rx configuration */
//  dma_reset(DMA1_CHANNEL3);
//  dma_default_para_init(&dma_init_struct);
//  dma_init_struct.buffer_size = MAX_FIFO_LEN;
//  dma_init_struct.direction = DMA_DIR_PERIPHERAL_TO_MEMORY;
//  dma_init_struct.memory_base_addr = (uint32_t)uart_rx_buf;
//  dma_init_struct.memory_data_width = DMA_MEMORY_DATA_WIDTH_BYTE;
//  dma_init_struct.memory_inc_enable = TRUE;
//  dma_init_struct.peripheral_base_addr = (uint32_t)&USART3->dt;
//  dma_init_struct.peripheral_data_width = DMA_PERIPHERAL_DATA_WIDTH_BYTE;
//  dma_init_struct.peripheral_inc_enable = FALSE;
//  dma_init_struct.priority = DMA_PRIORITY_LOW;
//  dma_init_struct.loop_mode_enable = TRUE;
//  dma_init(DMA1_CHANNEL3, &dma_init_struct);

//  /* enable transfer full data intterrupt */
//  dma_interrupt_enable(DMA1_CHANNEL3, DMA_FDT_INT, TRUE);

//  /* dma1 channel2 interrupt nvic init */
//  nvic_irq_enable(DMA1_Channel3_IRQn, 2, 0);

//  /* config flexible dma for usart2 rx */
//  dma_flexible_config(DMA1, FLEX_CHANNEL3, DMA_FLEXIBLE_UART3_RX);
//	
//	dma_channel_enable(DMA1_CHANNEL3, TRUE); /* usart3 Rx begin dma receiving */
//	dma_channel_enable(DMA1_CHANNEL2, FALSE); /* usart3 Tx begin dma receiving */

//}


//void Uart_Init(void)
//{
//  gpio_init_type gpio_init_structurt;
//	
//	crm_periph_clock_enable(CRM_USART3_PERIPH_CLOCK, TRUE);
//  crm_periph_clock_enable(CRM_GPIOB_PERIPH_CLOCK, TRUE); 
//	//gpio init PB10 TX
//	gpio_init_structurt.gpio_pins =GPIO_PINS_10;
//	gpio_init_structurt.gpio_mode =GPIO_MODE_MUX ;//复用推挽
//	gpio_init_structurt.gpio_drive_strength= GPIO_DRIVE_STRENGTH_STRONGER ;//输出能力
//  gpio_init_structurt.gpio_out_type =GPIO_OUTPUT_PUSH_PULL ;//推挽输出
//	gpio_init_structurt.gpio_pull =GPIO_PULL_NONE;
//  gpio_init(GPIOB,&gpio_init_structurt);
//    /* configure the usart3 rx pin */
//  gpio_init_structurt.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
//  gpio_init_structurt.gpio_out_type  = GPIO_OUTPUT_PUSH_PULL;
//  gpio_init_structurt.gpio_mode = GPIO_MODE_INPUT;
//  gpio_init_structurt.gpio_pins = GPIO_PINS_11;
//  gpio_init_structurt.gpio_pull = GPIO_PULL_UP;
//  gpio_init(GPIOB, &gpio_init_structurt);

//    /* configure usart2 param */
//  usart_init(USART3, 115200, USART_DATA_8BITS, USART_STOP_1_BIT);
//  usart_transmitter_enable(USART3, TRUE);
//  usart_receiver_enable(USART3, TRUE);
//  //usart_dma_transmitter_enable(USART3, TRUE);//使用dma发数据
//  //usart_dma_receiver_enable(USART3, TRUE);
//  usart_enable(USART3, TRUE);
//  
//  printf("run/r/n");
//}


void send_ack(void)
{
  u8 buf[64];
  u8 ptr=0;
  uint16_t speed = (uint16_t)ui.speed;
  uint16_t torque = (uint16_t)ui.torque;
  
  buf[ptr++] = CMD_SOF;
  buf[ptr++] = ',';
  buf[ptr++] = 'A';
  buf[ptr++] = ',';
  buf[ptr++] = ' ';
  buf[ptr++] = ',';
  buf[ptr++] = speed/10000 +'0';
  buf[ptr++] = speed%10000/1000 +'0';
  buf[ptr++] = speed%10000%1000/100 +'0';
  buf[ptr++] = speed%10000%1000%100/10 +'0';
  buf[ptr++] = speed%10000%1000%100%10 +'0';
  buf[ptr++] = ',';
  buf[ptr++] = torque/10000 +'0';
  buf[ptr++] = torque%10000/1000 +'0';
  buf[ptr++] = torque%10000%1000/100 +'0';
  buf[ptr++] = torque%10000%1000%100/10 +'0';
  buf[ptr++] = torque%10000%1000%100%10 +'0';
  buf[ptr++] = ',';
  buf[ptr++] = CMD_EOF;
  
  buf[ptr++] = 0x0d;
  buf[ptr++] = 0x0a;
  buf[ptr++] = 0;
  printf((char*)buf,ptr);
}

void cmd_parse(uint8_t *buf)
{
  uint16_t temp;
  static char *token;
  uint16_t       speed;  
  uint16_t       torque;
  uint16_t       cw_angle;
  uint16_t       ccw_angle;
  float f_temp;
  
  switch(buf[2])
  {
    default:
      break;
    
    case 'P':
      
      break;
    
    case 'R':
      
      break;   
    
    case 'A':
      
      break;
    
    case 'S':
      token = strtok(buf, ",");
      token = strtok(NULL, ",");
      token = strtok(NULL, ",");
      ui.mode = atoi(token);
      token = strtok(NULL, ",");
      speed = atoi(token);

      token = strtok(NULL, ",");
      torque = atoi(token);
      token = strtok(NULL, ",");
      cw_angle = atoi(token);
      token = strtok(NULL, ",");
      ccw_angle = atoi(token); 
    
      ui.speed = (float)speed;
      ui.torque= (float)torque;
      ui.torque = ui.torque/1000;
      ui.ccw_angle = (float)ccw_angle;
      ui.cw_angle = (float)cw_angle;
    
      if(ui.mode == MODE_CW)//update dir
      {
        ui.dir=CW;
      }
      else if(ui.mode == MODE_CCW)
      {
        ui.dir=CCW;
        ui.torque = -ui.torque;
      }
      if(Motor_Flag==0)//首次启动
      {
//         Motor_Start();
//				start();
         Motor_Flag=1;
      }
      else
      {
        //运行中修改参数
      }
      break;
    
    case 'E':
      ui.mode = MODE_OFF;
//      Motor_Stop();
      Motor_Flag=0;
//      MOTOR_Param.spd_lock_cnt = 0;
//      MOTOR_Param.speed_en = 0;
      //ui.on_off = OFF;
      break;    
  }
}

void USART3_IRQHandler(void)
{
  static uint8_t cmd_begin=0,rcv_ptr=0;
  static u8 cmd_buf[MAX_FIFO_LEN];
  u16 temp;

  if(usart_flag_get(USART3, USART_RDBF_FLAG) != RESET)
  {
    //usart_flag_clear(USART3, USART_RDBF_FLAG);
    temp = usart_data_receive(USART3);  //clear idle flag;
    if(cmd_begin == 0)
    {
        if(temp != CMD_SOF)
          return;
        cmd_begin++;
        rcv_ptr =0;
    }
    cmd_buf[rcv_ptr++] = temp&0xff;
    if(temp == CMD_EOF)
    {
      memcpy(uart_rx_buf,cmd_buf,rcv_ptr);
      uart_rx_flag=rcv_ptr;//包括接收数据长度
      rcv_ptr =0;
      cmd_begin =0;
    }
    else if(rcv_ptr >= (MAX_FIFO_LEN-1))
    {
      rcv_ptr =0;
      cmd_begin =0;
    }
  }
}

