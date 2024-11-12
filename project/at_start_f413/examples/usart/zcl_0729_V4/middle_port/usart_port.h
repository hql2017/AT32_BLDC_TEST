#ifndef __USART_PORT
#define __USART_PORT
extern void user_uart_init(void);

extern void config_uart_dma(void);
extern void usart_send_data(unsigned char *buf, unsigned short int len);
extern void  AppGetUsartData(unsigned short int *pBuff);
extern  void AppUsartTransmit(unsigned char *buf,unsigned short int len);
#endif
