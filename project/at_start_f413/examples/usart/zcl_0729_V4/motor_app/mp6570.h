#ifndef _MP6570_H_
#define _MP6570_H_

#include "stdint.h"

extern u16  SPI_ByteRead(unsigned char addr) ;
extern void SPI_ByteWrite(unsigned char addr, unsigned char data1, unsigned char data2)     ;
//MP6570 initialize function, default slave_addr = 0x00
void mp6570_init(unsigned char slave_addr);  

//Find theta bias automatically, default slave_addr = 0x00, suggest Ihold_mA = 500, suggest Thold_ms = 2000
unsigned char MP6570_AutoThetaBias(unsigned char slave_addr, unsigned short Ihold_mA , unsigned short Thold_ms);

//Write register value to MP6570, 16bits
void MP6570_WriteRegister(unsigned char slave_addr, unsigned char reg_addr,unsigned short data);

//Read register value from MP6570
unsigned short MP6570_ReadRegister(unsigned char slave_addr, unsigned char reg_addr);


extern unsigned short int   MP6570_PortInit(void);
extern void MP6570_SetModeToSPI(void);
#endif
