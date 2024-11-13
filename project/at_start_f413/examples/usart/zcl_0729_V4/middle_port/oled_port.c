
#include "at32f413_int.h"
#include "oled_port.h"
#include "stdint.h"
#include "stdlib.h"
#include "oledfont.h"  	 
#include "delay.h"
#include "gpio_port.h"

#include "at32f413_board.h"
#include "para_list.h"


#define OLED_SCL_Clr() gpio_bits_reset(LCD_CLK_PORT,LCD_CLK_IO)//SCK
#define OLED_SCL_Set() gpio_bits_set(LCD_CLK_PORT,LCD_CLK_IO)

#define OLED_SDA_Clr() gpio_bits_reset(LCD_MOSI_PORT,LCD_MOSI_IO)//SDA
#define OLED_SDA_Set() gpio_bits_set(LCD_MOSI_PORT,LCD_MOSI_IO)

#define OLED_DC_Clr()  gpio_bits_reset(LCD_DC_PORT,LCD_DC_IO)//DC
#define OLED_DC_Set()  gpio_bits_set(LCD_DC_PORT,LCD_DC_IO)

#define OLED_RES_Clr() gpio_bits_reset(LCD_RST_PORT,LCD_RST_IO)//RES
#define OLED_RES_Set() gpio_bits_set(LCD_RST_PORT,LCD_RST_IO)

#define OLED_CS_Clr()  gpio_bits_reset(LCD_CS_PORT,LCD_CS_IO)//CS  
#define OLED_CS_Set()  gpio_bits_set(LCD_CS_PORT,LCD_CS_IO)

#define OLED_CMD  0	//写命令
#define OLED_DATA 1	//写数据


#define OLED_LUMINANCE_MASK   0xAA //luminance(0 off ；(1~15)(0x11,0x22....0xFF))

#define ODD_COLUMN_PIXEL_MASK 	0x0F //OLED_LUMINANCE_MASK&0x0F //luminance
#define EVEN_COLUMN_PIXEL_MASK  0xF0 //OLED_LUMINANCE_MASK&0xF0  


#define PIXEL_MAX_COLUMN_NUM   160 
#define PIXEL_MAX_ROW_NUM 64 
#define OLED_DISPLAY_PAGE_NUM    8//PIXEL_MAX_ROW_NUM>>3
//uint8_t OLED_GRAM[PIXEL_MAX_COLUMN_NUM][OLED_DISPLAY_PAGE_NUM]={0x55};//pixel :160*64   ,bytes :80*64

extern union Param_Union sys_param_un; 
extern union Motor_Para_Union motor_param_un; 

static uint8_t OLED_GRAM[PIXEL_MAX_COLUMN_NUM][OLED_DISPLAY_PAGE_NUM]={0x55};//pixel :160*64   ,bytes :80*64
static uint8_t SPI_sendBuff[PIXEL_MAX_COLUMN_NUM>>1]={0};//pixel :160*64   ,bytes :80*64
uint8_t spi2_tx_dma_status=0;
/**
  * @brief  spi_DMA_config
  * @param  none
  * @retval none
  */
// /*static void spi_DMA_config(void)*/
// {	
// 	dma_init_type dma_init_struct;
// 	/* enable dma1 clock */
// 	crm_periph_clock_enable(CRM_DMA2_PERIPH_CLOCK, TRUE);
// 	/* dma1 channel2 for spi2 tx configuration */
// 	dma_reset(DMA2_CHANNEL2);
// 	dma_default_para_init(&dma_init_struct);
// 	dma_init_struct.buffer_size = 80;//(PIXEL_MAX_COLUMN_NUM>>1);//一个像素点半字节
// 	dma_init_struct.direction = DMA_DIR_MEMORY_TO_PERIPHERAL;
// 	dma_init_struct.memory_base_addr = (uint32_t)SPI_sendBuff;
// 	dma_init_struct.memory_data_width = DMA_MEMORY_DATA_WIDTH_BYTE;
// 	dma_init_struct.memory_inc_enable = TRUE;
// 	dma_init_struct.peripheral_base_addr = (uint32_t)&SPI2->dt;
// 	dma_init_struct.peripheral_data_width = DMA_PERIPHERAL_DATA_WIDTH_BYTE;
// 	dma_init_struct.peripheral_inc_enable = FALSE;
// 	dma_init_struct.priority = DMA_PRIORITY_MEDIUM;
// 	dma_init_struct.loop_mode_enable = FALSE;
// 	dma_init(DMA2_CHANNEL2, &dma_init_struct);

// 	/* config flexible dma for spi2tx */
// 	spi_i2s_dma_transmitter_enable(LCDSPI,TRUE);//DMA，spi2 TX开发送DMA

// 	dma_flexible_config(DMA2, FLEX_CHANNEL2, DMA_FLEXIBLE_SPI2_TX);
// 	dma_channel_enable(DMA2_CHANNEL2, TRUE); /* usart1 tx begin dma transmitting */	
// 	dma_interrupt_enable(DMA2_CHANNEL2,DMA_FDT_INT,TRUE);
// 	nvic_irq_enable(DMA2_Channel2_IRQn, 0, 0);
// }

/**
  * @brief  spi for oled
  * @param  none
  * @retval none
  */
static void spi_lcd_init(void)
{
  	gpio_init_type gpio_initstructure;
	spi_init_type spi_init_struct;  
 
	crm_periph_clock_enable(CRM_GPIOB_PERIPH_CLOCK, TRUE); 
	crm_periph_clock_enable(CRM_GPIOC_PERIPH_CLOCK, TRUE);	
	crm_periph_clock_enable(CRM_IOMUX_PERIPH_CLOCK, TRUE);
	gpio_pin_remap_config(SWJTAG_GMUX_010, TRUE);
	gpio_pin_remap_config(SPI2_GMUX_0001, TRUE);
 
	gpio_default_para_init(&gpio_initstructure);
  /* software cs, pa15 as a general io to control flash cs */
	gpio_initstructure.gpio_out_type       = GPIO_OUTPUT_PUSH_PULL;  
	gpio_initstructure.gpio_pull           = GPIO_PULL_NONE;  
	gpio_initstructure.gpio_mode           = GPIO_MODE_OUTPUT;  
	gpio_initstructure.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
	gpio_initstructure.gpio_pins           = LCD_CS_IO;
	gpio_init(LCD_CS_PORT, &gpio_initstructure);
	
  /* sck */ 
	gpio_initstructure.gpio_out_type       = GPIO_OUTPUT_PUSH_PULL;
	gpio_initstructure.gpio_pull           = GPIO_PULL_DOWN;
	gpio_initstructure.gpio_mode           = GPIO_MODE_MUX;
	gpio_initstructure.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
	gpio_initstructure.gpio_pins           = LCD_CLK_IO;
	gpio_init(LCD_CLK_PORT, &gpio_initstructure);
  
  /* miso */
//  gpio_initstructure.gpio_pull           = GPIO_PULL_UP;  
//  gpio_initstructure.gpio_mode           = GPIO_MODE_INPUT;  
//  gpio_initstructure.gpio_pins           = GPIO_PINS_6;
//  gpio_init(GPIOA, &gpio_initstructure);
  
  /* mosi */ 
	gpio_initstructure.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
	gpio_initstructure.gpio_out_type       = GPIO_OUTPUT_PUSH_PULL;
	gpio_initstructure.gpio_pull           = GPIO_PULL_DOWN;
	gpio_initstructure.gpio_mode           = GPIO_MODE_MUX;
	gpio_initstructure.gpio_pins           = LCD_MOSI_IO;
	gpio_init(LCD_MOSI_PORT, &gpio_initstructure);
		
	crm_periph_clock_enable(LCDSPI_CLOCK, TRUE);
			
	spi_default_para_init(&spi_init_struct);
	spi_init_struct.transmission_mode = SPI_TRANSMIT_HALF_DUPLEX_TX;
	spi_init_struct.master_slave_mode = SPI_MODE_MASTER;
	spi_init_struct.mclk_freq_division =SPI_MCLK_DIV_4;//25M
	spi_init_struct.first_bit_transmission = SPI_FIRST_BIT_MSB;
	spi_init_struct.frame_bit_num = SPI_FRAME_8BIT;
	spi_init_struct.clock_polarity = SPI_CLOCK_POLARITY_HIGH;
	spi_init_struct.clock_phase = SPI_CLOCK_PHASE_2EDGE;
	spi_init_struct.cs_mode_selection = SPI_CS_SOFTWARE_MODE;  
	spi_init(LCDSPI, &spi_init_struct);
		
	//	spi_DMA_config();
	spi_enable(LCDSPI, TRUE);
 
}
//反显函数
//static void OLED_ColorTurn(u8 i)
//{
//	if(i==0)
//	{
//		OLED_WR_Byte(0xA6,OLED_CMD);//正常显示
//	}
//	if(i==1)
//	{
//		OLED_WR_Byte(0xA7,OLED_CMD);//反色显示
//	}
//}        

static void OLED_WR_Byte(u8 dat,u8 cmd)
{	
	if(cmd)
	  OLED_DC_Set();
	else 
	  OLED_DC_Clr();
//	delay_1us(1);	
	OLED_CS_Clr();	
	while(spi_i2s_flag_get(SPI2, SPI_I2S_TDBE_FLAG) == RESET)	//等待发送区为空
	{
		;
	}
	spi_i2s_data_transmit(SPI2,dat);																	//发送一个字节的数据
	while(spi_i2s_flag_get(SPI2, SPI_I2S_BF_FLAG));
	OLED_CS_Set();
	OLED_DC_Set();
}
static void OLED_WR_Data_Bytes(u8 *pDat,uint8_t len)
{		
	uint8_t i;
	if(len==0) return;
	OLED_DC_Set();	
	OLED_CS_Clr();
	for(i=0;i<len;i++)
	{
		while(spi_i2s_flag_get(SPI2, SPI_I2S_TDBE_FLAG) == RESET){;}	//等待发送区为空
		spi_i2s_data_transmit(SPI2,pDat[i]);		//发送一个字节的数据
	}
	while(spi_i2s_flag_get(SPI2, SPI_I2S_BF_FLAG));	//等待发送结束		
//	dma
//	while(spi_i2s_flag_get(SPI2, SPI_I2S_TDBE_FLAG) == RESET){;} //
//	spi2_tx_dma_status=0;
//  dma_channel_enable(DMA2_CHANNEL2, FALSE); 	 	 
//  dma_data_number_set(DMA2_CHANNEL2, len);  
//  dma_channel_enable(DMA2_CHANNEL2, TRUE);	
////	while(spi_i2s_flag_get(SPI2, SPI_I2S_BF_FLAG) == RESET){;} //	
//		while(spi2_tx_dma_status!=0);
	OLED_CS_Set();
	OLED_DC_Set();
}
//开启OLED显示 
void OLED_DisPlay_On(void)
{
//	OLED_WR_Byte(0x8D,OLED_CMD);//电荷泵使能
//	OLED_WR_Byte(0x14,OLED_CMD);//开启电荷泵
	OLED_WR_Byte(0xAF,OLED_CMD);//点亮屏幕
}

//关闭OLED显示 
void OLED_DisPlay_Off(void)
{
//	OLED_WR_Byte(0x8D,OLED_CMD);//电荷泵使能
//	OLED_WR_Byte(0x10,OLED_CMD);//关闭电荷泵
	OLED_WR_Byte(0xAE,OLED_CMD);//关闭屏幕
}

//更新显存到OLED	
void OLED_Refresh(unsigned char handness)
{
	u8 i,n,j;

	OLED_WR_Byte(0xaf,OLED_CMD); //设置行起始地址
	OLED_WR_Byte(0xa1,OLED_CMD);   //设置列起始地址
	OLED_WR_Byte(0x00,OLED_CMD);   //设置列起始地址（MAX255）
	if(handness==0)
	{
		for(i=0;i<OLED_DISPLAY_PAGE_NUM;i++)//page for display	
		{
			for(j=0;j<8;j++)//8*page+j row
			{			
				for(n=0;n<80;n++)
				{					
					//2n
					if((OLED_GRAM[PIXEL_MAX_COLUMN_NUM-1-2*n][OLED_DISPLAY_PAGE_NUM-1-i]&(0x80>>j))==0)
					{
						SPI_sendBuff[n]=0;
					}	
					else
					{
						SPI_sendBuff[n]=EVEN_COLUMN_PIXEL_MASK;
					}
					//2n+1			
					if((OLED_GRAM[PIXEL_MAX_COLUMN_NUM-1-2*n-1][OLED_DISPLAY_PAGE_NUM-1-i]&(0x80>>j))!=0)
					{				
						SPI_sendBuff[n]|=ODD_COLUMN_PIXEL_MASK;
					}		
	//				OLED_WR_Byte(freshBuff,OLED_DATA);				
				}				
				OLED_WR_Data_Bytes(SPI_sendBuff,80)	;		
			}		
		}	
	}
	else
	{
		for(i=0;i<OLED_DISPLAY_PAGE_NUM;i++)//page for display
		{
			for(j=0;j<8;j++)//8*page+j row
			{							
				for(n=0;n<80;n++)
				{	//2n
					if((OLED_GRAM[2*n][i]&(0x01<<j))==0)
					{
						SPI_sendBuff[n]=0;							
					}						
					else 
					{
						SPI_sendBuff[n]=EVEN_COLUMN_PIXEL_MASK;						
					}		
					if((OLED_GRAM[2*n+1][i]&(0x01<<j))!=0)
					{	
						SPI_sendBuff[n]|=ODD_COLUMN_PIXEL_MASK;
					}	
	//			OLED_WR_Byte(freshBuff,OLED_DATA);				
				}				
				OLED_WR_Data_Bytes(SPI_sendBuff,80)	;			
			}		
		}	
	}
	
 
}
//清屏函数
void OLED_Clear(void)
{
		//*********大内存*****//
//	u8 i,n;
//	u8 brightness;
//	for(i=0;i<64;i++)
//	{	
//		for(n=0;n<80;n++)
//		{
//			if(n%2==0)
//			{
//				OLED_GRAM[2n][i]&=ODD_PIXEL_MASK;
//				brightness&=EVEN_PIXEL_MASK;	
//				OLED_GRAM[2n][i]|=brightness;//even
//			}				
//			else  
//			{
//				OLED_GRAM[2*n+1][i]&=EVEN_PIXEL_MASK;
//				brightness<<=4;
//				OLED_GRAM[2*n+1][i]|=brightness;//odd
//			}		
//		}
//  }
//********小内存***//
	u8 i,n;
	for(i=0;i<8;i++)
	{
			for(n=0;n<160;n++)
		{
		 OLED_GRAM[n][i]=0;//清除所有数据
		}
  }
	 OLED_Refresh(0);//更新显示
}
//清显示数据
void OLED_Clear_no_fresh(void)
{
	u8 i,n;	
 
	for(i=0;i<8;i++)
	{
		for(n=0;n<160;n++)
		{
		 OLED_GRAM[n][i]=0;//清除所有数据
		}
  }	
}

//画点 
//x:横坐标
//y:纵坐标
//t:1 填充(4bit 亮度(0~15)) 0,清空	
 void OLED_DrawPoint(u8 x,u8 y,u8 t)
{
	//*********大内存*****//
//	u8 i,m;
//	u8 n=x/2;//column，列	
//	m=x%2;	//行
//	if(m==0)
//	{ //even
//		OLED_GRAM[n][y]&=ODD_PIXEL_MASK;//保留奇数列	
//		t&=EVEN_PIXEL_MASK;	
//	}
//	else //odd
//	{
//		OLED_GRAM[n][y]&=EVEN_PIXEL_MASK;//保留偶数列
//		t<<=4;
//	}
//	if(t>0)
//	{	
//		OLED_GRAM[n][y]|=t;
//	}	

	//*********小内存*****//
	u8 i,m,n;
	
	i=y/8;
	m=y%8;
	n=1<<m;
	if(t){OLED_GRAM[x][i]|=n;}
	else
	{
		OLED_GRAM[x][i]=~OLED_GRAM[x][i];
		OLED_GRAM[x][i]|=n;
		OLED_GRAM[x][i]=~OLED_GRAM[x][i];
	}
}

////画线
////x1,y1:起点坐标
////x2,y2:结束坐标
//static void OLED_DrawLine(u8 x1,u8 y1,u8 x2,u8 y2,u8 mode)
//{
//	u16 t; 
//	int xerr=0,yerr=0,delta_x,delta_y,distance;
//	int incx,incy,uRow,uCol;
//	delta_x=x2-x1; //计算坐标增量 
//	delta_y=y2-y1;
//	uRow=x1;//画线起点坐标
//	uCol=y1;
//	if(delta_x>0)incx=1; //设置单步方向 
//	else if (delta_x==0)incx=0;//垂直线 
//	else {incx=-1;delta_x=-delta_x;}
//	if(delta_y>0)incy=1;
//	else if (delta_y==0)incy=0;//水平线 
//	else {incy=-1;delta_y=-delta_y;}
//	if(delta_x>delta_y)distance=delta_x; //选取基本增量坐标轴 
//	else distance=delta_y;
//	for(t=0;t<distance+1;t++)
//	{
//		OLED_DrawPoint(uRow,uCol,mode);//画点
//		xerr+=delta_x;
//		yerr+=delta_y;
//		if(xerr>distance)
//		{
//			xerr-=distance;
//			uRow+=incx;
//		}
//		if(yerr>distance)
//		{
//			yerr-=distance;
//			uCol+=incy;
//		}
//	}
//}
//x,y:圆心坐标
//r:圆的半径
 void OLED_DrawCircle(u8 x,u8 y,u8 r)
{
	int a, b,num;
    a = 0;
    b = r;
    while(2 * b * b >= r * r)      
    {
			OLED_DrawPoint(x + a, y - b,1);
			OLED_DrawPoint(x - a, y - b,1);
			OLED_DrawPoint(x - a, y + b,1);
			OLED_DrawPoint(x + a, y + b,1);

			OLED_DrawPoint(x + b, y + a,1);
			OLED_DrawPoint(x + b, y - a,1);
			OLED_DrawPoint(x - b, y - a,1);
			OLED_DrawPoint(x - b, y + a,1);        
			a++;
			num = (a * a + b * b) - r*r;//计算画的点离圆心的距离
			if(num > 0)
			{
					b--;
					a--;
			}
    }
}



//在指定位置显示一个字符,包括部分字符
//x:0~127
//y:0~63
//size1:选择字体 6x8/6x12/8x16/12x24
//mode:0,反色显示;1,正常显示
 void OLED_ShowChar(u8 x,u8 y,u8 chr,u8 size1,u8 mode)
{
	u8 i,m,temp,size2,chr1;
	u8 x0=x,y0=y;
	if(size1==8)size2=6;
	else size2=(size1/8+((size1%8)?1:0))*(size1/2);  //得到字体一个字符对应点阵集所占的字节数
	chr1=chr-' ';  //计算偏移后的值
		
	for(i=0;i<size2;i++)
	{
		if(size1==8)
			  {temp=asc2_0806[chr1][i];} //调用0806字体
		else if(size1==12)
        {temp=asc2_1206[chr1][i];} //调用1206字体
		else if(size1==16)
        {temp=asc2_1608[chr1][i];} //调用1608字体
		else if(size1==24)
        {temp=asc2_2412[chr1][i];} //调用2412字体
		else return;
		for(m=0;m<8;m++)
		{
			if(temp&0x01)OLED_DrawPoint(x,y,mode);
			else OLED_DrawPoint(x,y,!mode);
			temp>>=1;
			y++;
		}
		x++;
		if((size1!=8)&&((x-x0)==size1/2))
		{x=x0;y0=y0+8;}
		y=y0;
  }
}


//显示字符串
//x,y:起点坐标  
//size1:字体大小 
//*chr:字符串起始地址 
//mode:0,反色显示;1,正常显示
 void OLED_ShowString(u8 x,u8 y,u8 *chr,u8 size1,u8 mode)
{
	if(x>159)  x%=160;
	if(y>63)   y%=64;
	while((*chr>=' ')&&(*chr<='~'))//判断是不是非法字符!
	{
		OLED_ShowChar(x,y,*chr,size1,mode);
		if(size1==8) x+=6;
		else x+=size1/2;
		chr++;
  }
}
//m^n
static u32 OLED_Pow(u8 m,u8 n)
{
	u32 result=1;
	
	while(n--)
	{
	  result*=m;
	}
	return result;
}

//显示数字
//x,y :起点坐标
//num :要显示的数字
//len :数字的位数
//size:字体大小
//mode:0,反色显示;1,正常显示
 void OLED_ShowNum(u8 x,u8 y,u32 num,u8 len,u8 size1,u8 mode)
{
	u8 t,temp,m=0; 
	if(size1==8)m=2;
	for(t=0;t<len;t++)
	{
		temp=(num/OLED_Pow(10,len-t-1))%10;
		if(temp==0)
		{	//?			
		 OLED_ShowChar(x+(size1/2+m)*t,y,'0',size1,mode);	
		}
		else 
		{
			OLED_ShowChar(x+(size1/2+m)*t,y,temp+'0',size1,mode);
		}
  }
}

//显示汉字
//x,y:起点坐标
//num:汉字对应的序号
//mode:0,反色显示;1,正常显示
 void OLED_ShowChinese(u8 x,u8 y,u8 num,u8 size1,u8 mode)
{
	u8 m,temp;

	u8 x0=x,y0=y;	
	
	u16 i,size3=(size1/8+((size1%8)?1:0))*size1;  //得到字体一个字符对应点阵集所占的字节数

	for(i=0;i<size3;i++)
	{
		if(size1==16)
				{temp=Hzk1[num][i];}//调用16*16字体
		else if(size1==24)
				{temp=Hzk2[num][i];}//调用24*24字体
		else if(size1==32)       
				{temp=Hzk3[num][i];}//调用32*32字体
		else if(size1==64)
				{temp=Hzk4[num][i];}//调用64*64字体
		else return;
		for(m=0;m<8;m++)
		{
			if(temp&0x01)OLED_DrawPoint(x,y,mode);
			else OLED_DrawPoint(x,y,!mode);
			temp>>=1;
			y++;
		}
		x++;
		if((x-x0)==size1)
		{x=x0;y0=y0+8;}
		y=y0;
	}
}

//num 显示汉字的个数
//space 每一遍显示的间隔
//mode:0,反色显示;1,正常显示
void OLED_ScrollDisplay(u8 num,u8 space,u8 mode)
{
	u8 i,n,t=0,m=0,r;
	while(1)
	{
		if(m==0)
		{
	    OLED_ShowChinese(64,56,t,16,mode); //写入一个汉字保存在OLED_GRAM[][]数组中
			t++;
		}
		if(t==num)
			{
				for(r=0;r<16*space;r++)      //显示间隔
				 {
					for(i=1;i<80;i++)
						{
							for(n=0;n<16;n++)
							{
								OLED_GRAM[i-1][n]=OLED_GRAM[i][n];
							}
						}
           OLED_Refresh(0);
				 }
        t=0;
      }
		m++;
		if(m==16){m=0;}
		for(i=1;i<80;i++)   //实现左移
		{
			for(n=0;n<16;n++)
			{
				OLED_GRAM[i-1][n]=OLED_GRAM[i][n];
			}
		}
		 OLED_Refresh(0);
	}
}

//x,y：起点坐标
//sizex,sizey,图片长宽
//BMP[]：要写入的图片数组
//mode:0,反色显示;1,正常显示
 void OLED_ShowPicture(u8 x,u8 y,u8 sizex,u8 sizey,unsigned char BMP[],u8 mode)
{
	u16 j=0;
	u8 i,n,temp,m;
	u8 x0=x,y0=y;
	sizey=sizey/8+((sizey%8)?1:0);
	for(n=0;n<sizey;n++)
	{
		 for(i=0;i<sizex;i++)
		 {
				temp=BMP[j];
				j++;
				for(m=0;m<8;m++)
				{
					if(temp&0x01)OLED_DrawPoint(x,y,mode);
					else OLED_DrawPoint(x,y,!mode);
					temp>>=1;
					y++;
				}
				x++;
				if((x-x0)==sizex)
				{
					x=x0;
					y0=y0+8;
				}
				y=y0;
     }
	 }
}


/**
  * @brief  oled init
  * @param  none
  * @retval none
  */
void OLED_Init(void)
{
	gpio_init_type gpio_init_struct;
	
  /* enable the  GPIO clock */
  crm_periph_clock_enable(CRM_GPIOA_PERIPH_CLOCK, TRUE);
  crm_periph_clock_enable(CRM_GPIOB_PERIPH_CLOCK, TRUE);
	
	gpio_pin_remap_config(SWJTAG_GMUX_010, TRUE);
	
	/* enable the  GPIO clock */
	crm_periph_clock_enable(LCD_RST_CLOCK, TRUE);
  /* set default parameter */
  gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
  gpio_init_struct.gpio_out_type  = GPIO_OUTPUT_PUSH_PULL;
  gpio_init_struct.gpio_mode = GPIO_MODE_OUTPUT;
  gpio_init_struct.gpio_pins = LCD_RST_IO;
  gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
  gpio_init(LCD_RST_PORT, &gpio_init_struct);
  
	crm_periph_clock_enable(LCD_DC_CLOCK, TRUE);
  gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
  gpio_init_struct.gpio_out_type  = GPIO_OUTPUT_PUSH_PULL;
  gpio_init_struct.gpio_mode = GPIO_MODE_OUTPUT;
  gpio_init_struct.gpio_pins = LCD_DC_IO;
  gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
  gpio_init(LCD_DC_PORT, &gpio_init_struct);	
	
	spi_lcd_init();	

	OLED_RES_Clr();
	delay_1ms(50);
	OLED_RES_Set();
	delay_1ms(100);
		
	OLED_WR_Byte(0xAE,OLED_CMD); //Set Display Off
	OLED_WR_Byte(0x15,OLED_CMD); // Set Column Address
	OLED_WR_Byte(0x00,OLED_CMD);
	OLED_WR_Byte(0x4F,OLED_CMD);//80,byte
	OLED_WR_Byte(0x75,OLED_CMD); // Set Row Address
	OLED_WR_Byte(0x00,OLED_CMD);
	OLED_WR_Byte(0x3F,OLED_CMD);//64
	OLED_WR_Byte(0x81,OLED_CMD); // Set Contrast Control
	OLED_WR_Byte(0xB0,OLED_CMD);
	OLED_WR_Byte(0xA0,OLED_CMD); // Set Re-map
	OLED_WR_Byte(0xc1,OLED_CMD); //C1横向扫描 //  C4 纵向扫描
	OLED_WR_Byte(0xA1,OLED_CMD); // Set Display Start Line
	OLED_WR_Byte(0x00,OLED_CMD);
	OLED_WR_Byte(0xA2,OLED_CMD); // Set Display Offset
	OLED_WR_Byte(0x00,OLED_CMD);
	OLED_WR_Byte(0xA3,OLED_CMD); // Set Vertical Scroll Area
	OLED_WR_Byte(0x00,OLED_CMD);
	OLED_WR_Byte(0x40,OLED_CMD);
	OLED_WR_Byte(0xA4,OLED_CMD); // Set Display Mode (Normal display)
	OLED_WR_Byte(0xA8,OLED_CMD); // Set MUX Ratio
	OLED_WR_Byte(0x3F,OLED_CMD);
	OLED_WR_Byte(0xAB,OLED_CMD); // Function Selection A
	OLED_WR_Byte(0x01,OLED_CMD); // Select external VDD
	OLED_WR_Byte(0xAD,OLED_CMD); // External /Internal IREF Selection
	OLED_WR_Byte(0x8E,OLED_CMD); // Select external IREF
	OLED_WR_Byte(0xB1,OLED_CMD); // Set Phase Length
	OLED_WR_Byte(0x82,OLED_CMD);
	OLED_WR_Byte(0xB3,OLED_CMD); // Set Front Clock Divider/Oscillator Frequency
	OLED_WR_Byte(0xA0,OLED_CMD);
	OLED_WR_Byte(0xB6,OLED_CMD); // Set Second precharge Period
	OLED_WR_Byte(0x04,OLED_CMD);
	OLED_WR_Byte(0xB9,OLED_CMD); // Linear LUT
	OLED_WR_Byte(0xBC,OLED_CMD); // Set Pre-charge voltage
	OLED_WR_Byte(0x04,OLED_CMD);
	OLED_WR_Byte(0xBD,OLED_CMD); // Pre-charge voltage capacitor Selection
	OLED_WR_Byte(0x01,OLED_CMD);
	OLED_WR_Byte(0xBE,OLED_CMD); // Set VCOMH
	OLED_WR_Byte(0x05,OLED_CMD); // 0.82*VCC
	OLED_Clear();
	OLED_WR_Byte(0xAF,OLED_CMD); // Set Display ON
}

//==================oled app ===============
void dis_num(u8 x,u8 y,u8 len,u8 size1,u8 mode,int data )
{
	uint32_t temp;
	temp=abs(data);
	OLED_ShowNum(56,16,temp,4,16,1);
//	byte1=temp/1000; byte2=(temp/100)%10;byte2=(temp/10)%10;byte2=temp%10;		
}
void torque_picture_apex(u8 x,u8 y,unsigned short int torqueValue,unsigned short int maxTorqueValue)
{    
	u8 i=0,j=0,temp;
	j=maxTorqueValue/10;
	temp=torqueValue/j;
	if(temp<1)//10%
	{
		for(i=0;i<40;i++)
		{
			if(i<2)
			{
				OLED_GRAM[i+x][0]=0x00;
				OLED_GRAM[i+x][1]=0x00;
				OLED_GRAM[i+x][2]=0x00;
				OLED_GRAM[i+x][3]=0x00;
			}
			else
			{
				if(i<4)
				{
					OLED_GRAM[i+x][0]=0x00;
					OLED_GRAM[i+x][1]=0x00;
					OLED_GRAM[i+x][2]=0x00;
					OLED_GRAM[i+x][3]=0x00;
				}
			}
			
		}
	}
}
//画竖列，
 void disp_DrawColumn(u8 x,u8 y,u8 len,u8 t)
{
	u8 i;
	for(i=0;i<len;i++)
	{
		OLED_DrawPoint(x,y+i,t);
	}
}
//画横线
 void disp_DrawRow(u8 x,u8 y,u8 len,u8 t)
{
	u8 i;	
	for(i=0;i<len;i++)
	{
		OLED_DrawPoint(x+i,y,t);
	}
}
void main_dir_fill(u8 x,u8 y,uint8_t dir)
{    
	  u8 i=0,j=0;
	j=y/8;
    if(dir>3)//0,1,2,3 rec cw,ccw ,atc
    {
      for(i=0 ;i<32; i++)
      {
        OLED_GRAM[i+x][0+j]=0x00;
        OLED_GRAM[i+x][1+j]=0x00;
        OLED_GRAM[i+x][2+j]=0x00;
        OLED_GRAM[i+x][3+j]=0x00;
		OLED_GRAM[i+x][4+j]=0x00;
      }     
    }		
    else
    {
		OLED_ShowPicture( x,y,32,40,f_r_dir[dir],1);
	}
}

void logo_disppaly(void)
{
	OLED_ShowPicture(24,16,16,32,asc2_3216[0],1);//LOGO,E
	OLED_ShowPicture(40,16,16,32,asc2_3216[1],1);//LOGO,N
	OLED_ShowPicture(56,16,16,32,asc2_3216[2],1);//LOGO,D
	//OLED_ShowPicture(48,16,16,32,asc2_3216[3],1);//LOGO,O
	OLED_ShowPicture(72,8,16,48,asc_4816[0],1);//LOGO,O
	OLED_ShowPicture(88,16,16,32,asc2_3216[4],1);//LOGO,A
	OLED_ShowPicture(104,16,16,32,asc2_3216[5],1);//LOGO,R
	OLED_ShowPicture(120,16,16,32,asc2_3216[6],1);//LOGO,T		
}

 void disp_batValue(u8 x,u8 y, unsigned short int  batValue)
{
	unsigned char dispLevel,i,j,page;
	static unsigned char blinkFlag;
	static unsigned short int  historyBatValue=0;
	if(historyBatValue>batValue||historyBatValue+200<batValue)//防抖
	{
		historyBatValue=batValue;
	}
	if(historyBatValue<3000)
	{
		blinkFlag=0;//不再闪烁
		dispLevel=0;		
	}
	else if(historyBatValue<3450)
	{
		if(historyBatValue<3350)//低电量闪烁
		{
			blinkFlag=(blinkFlag)?0:1;
		}
		dispLevel=1;
	}
	else if(historyBatValue<3650)
	{
		dispLevel=2;
	}
	else if(historyBatValue<3950)
	{
		dispLevel=3;
	}
	else 
	{
		dispLevel=4;	//0~4;
	}	
	
	if(y%8==0)
	{
		
		j=y/8;
		for(page=0;page<4;page++)
		{
			for(i=0;i<24;i++)
			{
				if(historyBatValue<3350&&blinkFlag!=0)//低电量闪烁
				{					
					OLED_GRAM[x+i][page+j]=little_bat[0][page*24+i];				
				}
				else
				{
					OLED_GRAM[x+i][page+j]=little_bat[dispLevel][page*24+i];
				}				
			}
		}
	}
	else
	{
		if(historyBatValue<3350&&blinkFlag!=0)//低电量闪烁
		{			
			OLED_ShowPicture(x,y,24,32,little_bat[0],1);			
		}
		else
		{
			OLED_ShowPicture(x,y,24,32,little_bat[dispLevel],1);
		}
	}
}
static void chargValue(u8 x,u8 y, unsigned short int  batValue)
{
	unsigned char i,j,dispLevel;
	if(batValue<3100)
	{
		dispLevel=0;		
	}
	else if(batValue<3450)
	{
		dispLevel=1;
	}
	else if(batValue<3650)
	{
		dispLevel=2;
	}
	else if(batValue<3950)
	{
		dispLevel=3;
	}
	else 
	{
		dispLevel=4;	//0~4;
	}
	
	for(i=0;i<4;i++)
	{
		for(j=0;j<17;j++)
		{			
			if(i<dispLevel)
			{		
				if(j>3)
				{
					OLED_GRAM[17*i+x+j][2]=0xFF;
					OLED_GRAM[17*i+x+j][3]=0xFF;
					OLED_GRAM[17*i+x+j][4]=0xFF;	
					OLED_GRAM[17*i+x+j][5]=0xFF;	
				}
				else
				{
					OLED_GRAM[17*i+x+j][2]=0x00;
					OLED_GRAM[17*i+x+j][3]=0x00;
					OLED_GRAM[17*i+x+j][4]=0x00;	
					OLED_GRAM[17*i+x+j][5]=0x00;	
				}
			}
			else 
			{			
				OLED_GRAM[17*i+x+j][2]=0x00;
				OLED_GRAM[17*i+x+j][3]=0x00;
				OLED_GRAM[17*i+x+j][4]=0x00;	
				OLED_GRAM[17*i+x+j][5]=0x00;	
			}			
		}
	}	
}
void charg_bgm(u8 x,u8 y,unsigned char flag,unsigned short int batValue)
{
	unsigned char j;
	if(flag==0)//框
	{
			//page2
		for(j=x;j<x+80;j++)
		{
			if(j<x+4||j>x+76)
			{
				OLED_GRAM[j][1]=0xFF;
			}
			else 
			{
				OLED_GRAM[j][1]=0x0F;
			}		
		}
		//page2
		for(j=x;j<x+80;j++)
		{
			if(j<x+4||j>x+76)
			{
				OLED_GRAM[j][2]=0xFF;
			}
			else 
			{
				OLED_GRAM[j][2]=0x00;
			}		
		}
		//page3
		for(j=x;j<x+80;j++)
		{
			if(j<x+4||j>x+76)
			{
				OLED_GRAM[j][3]=0xFF;
			}
			else 
			{
				OLED_GRAM[j][3]=0x00;
			}		
		}
			//page4
		for(j=x;j<x+80;j++)
		{		if(j<x+4||j>x+76)
			{
				OLED_GRAM[j][4]=0xFF;
			}
			else 
			{
				OLED_GRAM[j][4]=0x00;
			}			
		}
			//page5
		for(j=x;j<x+80;j++)
		{		if(j<x+4||j>x+76)
			{
				OLED_GRAM[j][5]=0xFF;
			}
			else 
			{
				OLED_GRAM[j][5]=0x00;
			}			
		}	
			//page6
		for(j=x;j<x+80;j++)
		{		if(j<x+4||j>x+76)
			{
				OLED_GRAM[j][6]=0xFF;
			}
			else 
			{
				OLED_GRAM[j][6]=0xF0;
			}			
		}		
		//other 
		for(j=x+80;j<x+88;j++)
		{			
			OLED_GRAM[j][3]=0xFF;
			OLED_GRAM[j][4]=0xFF;		
		}		
	}
	else
	{
		chargValue(x+4,y, batValue);	
	}
	
}

