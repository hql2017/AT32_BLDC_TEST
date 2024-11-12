/*
*@file        oled_port.h
*@brief       applation
*@attention   none
*@version     V1.0
*@data        2023/2/8
*@author      huqilin
*/

#ifndef __OLED_PORT_H_
#define __OLED_PORT_H_
#include "stdint.h"


void oled_com_init(void);
void oled_set_cmd(uint8_t reg);
void oled_set_dat(uint8_t reg);


void OLED_ClearPoint(u8 x,u8 y);
void OLED_ColorTurn(u8 i);
void OLED_WR_Byte(u8 dat,u8 mode);
void OLED_DisPlay_On(void);
void OLED_DisPlay_Off(void);
void OLED_Refresh(unsigned char handness);
void OLED_Clear_no_fresh(void);
void OLED_Clear(void);
void OLED_DrawPoint(u8 x,u8 y,u8 t);
void OLED_DrawLine(u8 x1,u8 y1,u8 x2,u8 y2,u8 mode);
void OLED_DrawCircle(u8 x,u8 y,u8 r);
void OLED_ShowChar(u8 x,u8 y,u8 chr,u8 size1,u8 mode);
void OLED_ShowChar6x8(u8 x,u8 y,u8 chr,u8 mode);
void OLED_ShowString(u8 x,u8 y,u8 *chr,u8 size1,u8 mode);
void OLED_ShowNum(u8 x,u8 y,u32 num,u8 len,u8 size1,u8 mode);
void OLED_ShowChinese(u8 x,u8 y,u8 num,u8 size1,u8 mode);
void OLED_ScrollDisplay(u8 num,u8 space,u8 mode);
void OLED_ShowPicture(u8 x,u8 y,u8 sizex,u8 sizey,u8 BMP[],u8 mode);
extern void OLED_Init(void);

void charg_bgm(u8 x,u8 y,unsigned char flag,unsigned short int batValu);
void disp_batValue(u8 x,u8 y, unsigned short int  batValue);
void logo_disppaly(void);
 
void main_dir_fill(u8 x,u8 y,uint8_t dir);
 void disp_DrawColumn(u8 x,u8 y,u8 len,u8 t);
  void disp_DrawRow(u8 x,u8 y,u8 len,u8 t);


#endif

