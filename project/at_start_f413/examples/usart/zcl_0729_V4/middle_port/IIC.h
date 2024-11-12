#ifndef __IIC_PORT
#define __IIC_PORT
extern void  IICPortInit(void);
//发送一个字节
extern void IIC_WR_Byte(unsigned char addr, unsigned char data1, unsigned char data2);
extern unsigned short int  IIC_RD_Byte(unsigned char slave_addr, unsigned char reg_addr);
#endif
