/*
*@file        uart.h
*@brief       applation
*@attention   none
*@version     V1.0
*@data        2022/5/24
*@author      xuwenxi
*/
 #ifndef __UART_INIT_H_
 #define __UART_INIT_H_
 #include "stdint.h"
#define MAX_FIFO_LEN    64
#define CMD_SOF   '*'
#define CMD_EOF   '#'



extern uint8_t uart_tx_buf[MAX_FIFO_LEN];
extern uint8_t uart_rx_buf[MAX_FIFO_LEN];
extern volatile uint8_t uart_rx_flag;//包括接收数据长度

extern volatile unsigned   char usart_tx_dma_status ;
extern volatile unsigned   char usart_rx_dma_status ;

void send_ack(void);
//void cmd_parse(uint8_t *buf); 
void UART_DMA_Init(void );
void Uart_Init(void);
 #endif

 /****************uart.h end*****************/
 
 
