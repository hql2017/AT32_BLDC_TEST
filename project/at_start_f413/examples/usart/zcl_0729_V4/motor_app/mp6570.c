#include "mp6570.h"
#include "init.h"
#include "at32f413_spi.h"
#include "control.h"
#include "macros.h"
#include "delay.h"

#define MP6570_SLAVE_Addr 0x00


#ifndef MP6570_IIC_MODE 
#define MP6570_IIC_MODE
#endif
#ifdef MP6570_IIC_MODE 
#define IIC_SCL_Clr() gpio_bits_reset(GPIOA,GPIO_MP6570_SCK)//SCL
#define IIC_SCL_Set() gpio_bits_set(GPIOA,GPIO_MP6570_SCK)

#define IIC_SDA_Clr() gpio_bits_reset(GPIOA,GPIO_MP6570_MISO)//DIN
#define IIC_SDA_Set() gpio_bits_set(GPIOA,GPIO_MP6570_MISO)


#define IIC_CMD  0	//写命令
#define IIC_DATA 1	//写数据

void IIC_WR_Byte(unsigned char addr, unsigned char data1, unsigned char data2);
u16 IIC_RD_Byte(unsigned char slave_addr, unsigned char reg_addr);

void set_r_in(void)
{
	gpio_init_type gpio_initstructure;
  gpio_initstructure.gpio_pull           = GPIO_PULL_NONE; 
	gpio_initstructure.gpio_mode           = GPIO_MODE_INPUT;  //MASTER  
  gpio_initstructure.gpio_pins           = GPIO_MP6570_MISO;
  gpio_init(GPIOA, &gpio_initstructure);

}
void set_r_out(void)
{
	gpio_init_type gpio_initstructure;
	gpio_initstructure.gpio_drive_strength =  GPIO_DRIVE_STRENGTH_STRONGER;
	gpio_initstructure.gpio_out_type       = GPIO_OUTPUT_OPEN_DRAIN;//GPIO_OUTPUT_PUSH_PULL;
  gpio_initstructure.gpio_pull           = GPIO_PULL_NONE;//GPIO_PULL_UP;
  gpio_initstructure.gpio_mode           = GPIO_MODE_OUTPUT;
  gpio_initstructure.gpio_pins           = GPIO_MP6570_MISO;
  gpio_init(GPIOA, &gpio_initstructure);
}
#endif	

u16 reg[128] = 
{
	0x6570,	//0x00
	0x8000,	//0x01
	0x0000,	//0x02
	0x0000,	//0x03
	0x0101,	//0x04  necessary
	0xFE00,	//0x05  necessary
	0x205D,	//0x06  necessary
	0x02FB,	//0x07  necessary
	0x00C8,	//0x08
	0x00C8,	//0x09
	0x7FF8,	//0x0A
	0xF833,	//0x0B
	0xF847,	//0x0C
	0x7171,	//0x0D
	0x71b5,	//0x0E
	0x014A,	//0x0F
	
	0x0000,	//0x10
	0x0000,	//0x11
	0x0FA0,	//0x12
	0x1F40,	//0x13
	0x0000,	//0x14
	0x0000,	//0x15
	0x0000,	//0x16
	0x0FA0,	//0x17 necessary
	0x0F50,	//0x18 necessary
	0x0000,	//0x19 necessary
	0x0F90,	//0x1A necessary
	0x5050,	//0x1B necessary
	0x0064,	//0x1C 
	0x0064,	//0x1D
	0x0421,	//0x1E
	0x0421,	//0x1F
		
	0x0BB8,	//0x20
	0x00B0,	//0x21
	0x0001,	//0x22
	0x00C8,	//0x23
	0x00C8,	//0x24
	0x00C6,	//0x25
	0x0000,	//0x26
	0x03FF,	//0x27  ocp necessary
	0x021C,	//0x28  ovp necessary
	0x0130,	//0x29  
	0x0480,	//0x2A
	0x0FB1,	//0x2B
	0x0064,	//0x2C
	0x0000,	//0x2D
	0x0000,	//0x2E
	0x0000,	//0x2F
	
	0x000F,	//0x30  necessary
	0x0000,	//0x31
	0x0040,	//0x32
	0xE003,	//0x33  necessary
	0x015A,	//0x34, baud rate, 40000000/fbps - 1
	0x2000,	//0x35
	0x0000,	//0x36
	0x3FF4,	//0x37
	0xE000,	//0x38
	0x0000,	//0x39
	0x0000,	//0x3A
	0x0000,	//0x3B
	0x0000,	//0x3C
	0x0000,	//0x3D
	0x0000,	//0x3E
	0x0000,	//0x3F
	
	0x0000,	//0x40
	0x0000,	//0x41
	0x0000,	//0x42
	0x0000,	//0x43
	0x0000,	//0x44
	0x0000,	//0x45
	0x0000,	//0x46
	0x0000,	//0x47
	0x0000,	//0x48
	0x0000,	//0x49
	0x0000,	//0x4A
	0x0000,	//0x4B
	0x0000,	//0x4C
	0x0000,	//0x4D
	0x0000,	//0x4E
	0x0000,	//0x4F
	
	0x0000,	//0x50
	0x0000,	//0x51
	0x0000,	//0x52
	0x0000,	//0x53
	0x0000,	//0x54
	0x0000,	//0x55
	0x0000,	//0x56
	0x0000,	//0x57
	0x0000,	//0x58
	0x0000,	//0x59
	0x0000,	//0x5A
	0x0000,	//0x5B
	0x0000,	//0x5C
	0x0000,	//0x5D
	0x0000,	//0x5E
	0x0000,	//0x5F
	
	0x0000,	//0x60
	0x0000,	//0x61
	0x0000,	//0x62
	0x0000,	//0x63
	0x0000,	//0x64
	0x0000,	//0x65
	0x0000,	//0x66
	0x0000,	//0x67
	0x0000,	//0x68
	0x0000,	//0x69
	0x0000,	//0x6A
	0x0000,	//0x6B
	0x0000,	//0x6C
	0x0000,	//0x6D
	0x0000,	//0x6E
	0x0000,	//0x6F
};

void SPI_ByteWrite(unsigned char addr, unsigned char data1, unsigned char data2)     
{
	u8 byte1,byte2,byte3;   
  
	addr = (addr<<1) & 0xFE;                                                                                                 
	byte1 = (data1>>4) & 0x0F;                                                                                         
	byte2 = (data1<<4) + (data2>>4);                                                                                           
	byte3 = ((data2<<4) & 0xF0);   
	NSS_RESET;
	spi_i2s_data_transmit(SPI_MASTER,MP6570_SLAVE_Addr);
	while(!spi_i2s_flag_get(SPI_MASTER, SPI_I2S_TDBE_FLAG));
	spi_i2s_data_transmit(SPI_MASTER,addr);
	while(!spi_i2s_flag_get(SPI_MASTER, SPI_I2S_TDBE_FLAG));
	spi_i2s_data_transmit(SPI_MASTER,byte1);
	while(!spi_i2s_flag_get(SPI_MASTER, SPI_I2S_TDBE_FLAG));
	spi_i2s_data_transmit(SPI_MASTER,byte2);
	while(!spi_i2s_flag_get(SPI_MASTER, SPI_I2S_TDBE_FLAG));
	spi_i2s_data_transmit(SPI_MASTER,byte3);
	while(!spi_i2s_flag_get(SPI_MASTER, SPI_I2S_TDBE_FLAG));
	while(spi_i2s_flag_get(SPI_MASTER, SPI_I2S_BF_FLAG));
//	delay_1us(1);
	NSS_SET;	
}
u16 SPI_ByteRead(unsigned char addr)                                                
{
		u8 byte_Low,byte_High, byte1,byte2, byte3; 
	 u16 data;                                                                                        
	 addr = (addr<<1)& 0xFE;                                                                                                   //100ns
	 NSS_RESET;//SPI_NSSInternalSoftwareConfig(SPI_MASTER, SPI_NSSInternalSoft_Reset);
		byte1 = spi_i2s_data_receive(SPI_MASTER);	//h
	 spi_i2s_data_transmit(SPI_MASTER,0x01);		 
	 while(!spi_i2s_flag_get(SPI_MASTER, SPI_I2S_TDBE_FLAG));
	 spi_i2s_data_transmit(SPI_MASTER,addr);
	 while(spi_i2s_flag_get(SPI_MASTER, SPI_I2S_RDBF_FLAG) == RESET);
	 byte1 = spi_i2s_data_receive(SPI_MASTER);
	 while(!spi_i2s_flag_get(SPI_MASTER, SPI_I2S_TDBE_FLAG));
	 spi_i2s_data_transmit(SPI_MASTER,0x00);
	 while(spi_i2s_flag_get(SPI_MASTER, SPI_I2S_RDBF_FLAG) == RESET);
	 byte1 = spi_i2s_data_receive(SPI_MASTER);
	 while(!spi_i2s_flag_get(SPI_MASTER, SPI_I2S_TDBE_FLAG));
	 spi_i2s_data_transmit(SPI_MASTER,0x00);
	 while(spi_i2s_flag_get(SPI_MASTER, SPI_I2S_RDBF_FLAG) == RESET);
	 byte1 = spi_i2s_data_receive(SPI_MASTER);
	 while(!spi_i2s_flag_get(SPI_MASTER, SPI_I2S_TDBE_FLAG));
	 spi_i2s_data_transmit(SPI_MASTER,0x00);
	 while(spi_i2s_flag_get(SPI_MASTER, SPI_I2S_RDBF_FLAG) == RESET);
	 byte2 = spi_i2s_data_receive(SPI_MASTER);		 
	 while(spi_i2s_flag_get(SPI_MASTER, SPI_I2S_RDBF_FLAG) == RESET);
	 byte3 = spi_i2s_data_receive(SPI_MASTER);
	 while(spi_i2s_flag_get(SPI1, SPI_I2S_BF_FLAG));//
		NSS_SET;
	 byte_Low = (byte2<<4)+(byte3>>4);                             
	 byte_High = (byte1<<4)+(byte2>>4);
	 data = ((u16)byte_High<<8) +(u16)byte_Low;  
	 return data;
}

void mp6570_init(unsigned char slave_addr)
{
	int PERIOD;
	u16 MaxOnTime,deadtime;
	MP6570_WriteRegister(slave_addr, 0x62, 0x0000);//Turn on MP6570 internal Clock
	delay_1ms(2);												//wait for internal Clock ON
	MP6570_WriteRegister(slave_addr, 0x04, m_p); //pole pair	
	MP6570_WriteRegister(slave_addr, 0x10, 0x0000); //ID_REF
	MP6570_WriteRegister(slave_addr, 0x11, 0x0001); //IQ_REF
	MP6570_WriteRegister(slave_addr, 0x12, regi[0x2C]); //CURRENT_KI
	MP6570_WriteRegister(slave_addr, 0x13, regi[0x2B]); //CURRENT_KP
	
	 PERIOD = 80000/FS;
	 MaxOnTime = 0;
	if(PERIOD >= 32768) //fix low side on time to 1us
		MaxOnTime = 5000/FS - 5;
	else if(PERIOD >=16384)
		MaxOnTime = 10000/FS - 10;
	else if(PERIOD >=8192)
		MaxOnTime = 20000/FS - 20;
	else if(PERIOD >=4096)
		MaxOnTime = 40000/FS - 40;
	else
		MaxOnTime = 80000/FS - 80;
	 deadtime = DEADTIME/25;
	
	MP6570_WriteRegister(slave_addr, 0x17, PERIOD); //PERIOD
	MP6570_WriteRegister(slave_addr, 0x18, MaxOnTime); //MAX_PERIOD
	MP6570_WriteRegister(slave_addr, 0x19, deadtime); //{SH_DELAY, DTPWM}
	MP6570_WriteRegister(slave_addr, 0x1A, PERIOD-0); //SH_CMP
	
	MP6570_WriteRegister(slave_addr, 0x27, regi[0x5A]); //I_OCP
	MP6570_WriteRegister(slave_addr, 0x28, 0x021C); //FAULT
	MP6570_WriteRegister(slave_addr, 0x29, regi[0x56]); //OVP
	MP6570_WriteRegister(slave_addr, 0x30, (AD_MOD<<3) + AD_GAIN); //{AD_MOD, AD_GAIN}
	MP6570_WriteRegister(slave_addr, 0x33, 0xA003|(PWMMODE<<7)); //MODE....
	MP6570_WriteRegister(slave_addr, 0x37, 0x3FF4); //KALMAN_N
	MP6570_WriteRegister(slave_addr, 0x38, 0xE000); //sensor inital time
	MP6570_WriteRegister(slave_addr, 0x39, 0x0000); //sensor control bits	
	MP6570_WriteRegister(slave_addr, 0x50, 0x0000); //{THETA_GEN_TIME[7:0], THETA_GEN_EN, mem_section[1:0]}		
	MP6570_WriteRegister(slave_addr, 0x51, 0x0000); //{OPEN_LOOP_DBG, UD_SET_DBG[11:0]}				
	MP6570_WriteRegister(slave_addr, 0x52, 0x0000); //{UQ_SET_DBG[11:0], THETA_BIAS_EN}	
	
}

/*********************************************************************************************************//**
  * @brief  Initial theta bias auto detect 
	* @param  None 
  * @retval None
  ***********************************************************************************************************/
u8 MP6570_AutoThetaBias(unsigned char slave_addr, u16 Ihold_mA , u16 Thold_ms)//current(mA)?interval(ms)
{
  u8 INL_STATUS = 0;	
	u16 i55,i55_2,i54,i33,i05;
	u16 THETA_BIAS_TIME;
  int i07_temp;
	u16 CMPA_SET,x_theta_dir;
	LOOPTIMER_DISABLE;							//Turn off SCTM1
	delay_1ms(100);	
//	nSLEEP_SET;
	delay_1ms(100);
	MP6570_WriteRegister(slave_addr, 0x62, 0);			//turn internal clock on
	 i54 = MP6570_ReadRegister(slave_addr, 0x54);		//read register 0x54 to determine if MP6570 is at ON or OFF state
	 i33 = MP6570_ReadRegister(slave_addr, 0x33);		//read register 0x33 to determine standby or not-standby mode
	if (i54 != 0x00)
	{
		if ((i33 & 0x8000) == 0x8000)			//set MP6570 to OFF state
    	MP6570_WriteRegister(slave_addr, 0x61, 0);
		else
     	MP6570_WriteRegister(slave_addr, 0x60, 0);
	} 	
	i05 = MP6570_ReadRegister(slave_addr, 0x05);		//read register 0x05
	reg[0x05] = i05 & 0x7fff; 
	MP6570_WriteRegister(slave_addr, 0x05, reg[0x05]);	//set THETA_DIR = 0
	i05 = MP6570_ReadRegister(slave_addr, 0x05);
	MP6570_WriteRegister(slave_addr, 0x20 , 100);	      //set SPEED_CMD !=0
	MP6570_WriteRegister(slave_addr, 0x11,1); //set iq_ref !=0
	
	CMPA_SET =Ihold_mA * regi[0x01]*2/1000 * 40 / 100 / regi[0x06];
	MP6570_WriteRegister(slave_addr, 0x51, CMPA_SET);    //write CMPA_SET to register 51H Bit0~Bit11
	THETA_BIAS_TIME=(u16)(Thold_ms*0.1);	     // calculate THETA_BIAS_TIME value
	MP6570_WriteRegister(slave_addr, 0x52,(THETA_BIAS_TIME<<1) + 1); 	//write THETA_BIAS_TIME to register 0x52 and set the THETA_BIAS_EN bit
	
	if ((i33 & 0x8000) == 0x8000)			// set MP6570 to ON state
				MP6570_WriteRegister(slave_addr, 0x60,0);
	else
				MP6570_WriteRegister(slave_addr, 0x61,0);	

	delay_1ms( Thold_ms * 0.6);				//delay 0.8 x Thold_ms until the motor is steady at position 1
	i55 = MP6570_ReadRegister(slave_addr, 0x55);	//read the angle at position 1	
	delay_1ms( Thold_ms * 1);				//delay Thold_ms until the motor is steady at position 2
	i55_2 = MP6570_ReadRegister(slave_addr, 0x55);	//read the angle at postion 2
	delay_1ms( Thold_ms * 0.2 + 100);		//delay (Thold_ms * 0.2 +100) to let the auto-theta-bias procedure finish.
	
	//set MP6570 to OFF state
	if ((i33 & 0x8000) == 0x8000) 				
		MP6570_WriteRegister(slave_addr, 0x61, 0);
	else
		MP6570_WriteRegister(slave_addr, 0x60, 0);

	MP6570_WriteRegister(slave_addr, 0x52, 0);			//exit auto-theta-bias procedure
	//calculate the THETA_DIR and THETA_BIAS
	if (((i55 > i55_2) && ((i55 - i55_2) < 0X3FFF)))  // decrease and not over a turn (the zero deg transition)
	{
		x_theta_dir = 1;
		i07_temp = 65536 -((i55 + i55_2)>>1);
	}
	else if ((i55 < i55_2) && (i55_2 - i55 > 0X3FFF)) // decrease and over a turn (the zero deg transition)
	{
		x_theta_dir = 1;
		i07_temp = (i55 + 65536 + i55_2)>>1;
		if (i07_temp > 65536)
		{
			i07_temp = i07_temp - 65535;
		}
		i07_temp = 65536- i07_temp;
	}
	else if ((i55 > i55_2) && (i55 - i55_2 > 0X3FFF))	//increase and over a turn (the zero deg transition)
	{
		x_theta_dir = 0;
		i07_temp = (i55 + 65536 + i55_2)>>1;
		if (i07_temp > 65535)
		{
			i07_temp = i07_temp - 65535;
		}
	}
	else																					// increasing and not over a turn (the zero deg transition)
	{
		x_theta_dir = 0;
		i07_temp = (i55 + i55_2)>>1;
	}
	
	reg[0x07] = i07_temp>>5;
	reg[0x05] = (reg[0x05] | (x_theta_dir<<15));
	MP6570_WriteRegister(0x00,0x05, reg[0x05]);
	MP6570_WriteRegister(0x00,0x07, reg[0x07]);
	MP6570_WriteRegister(0x00,0x64, 0x0000);
	delay_1ms(2000);
	SPI_ByteWrite(0x60,0x00,0x00); //Turn on MP6570 

	delay_1ms(100);
	LOOPTIMER_ENABLE;
	reset_parameters();
	INL_STATUS = 1;
	
	return INL_STATUS;	
}

void MP6570_WriteRegister(unsigned char slave_addr, unsigned char reg_addr,u16 data)
{
	u8 data1, data2;
	reg[reg_addr] = data;
	data2 = (u8)data;
	data1 = (u8)(data>>8);

	SPI_ByteWrite(reg_addr, data1, data2);	
}

u16 MP6570_ReadRegister(unsigned char slave_addr, unsigned char reg_addr)
{
	u16 temp;
	temp = SPI_ByteRead(reg_addr);
	reg[reg_addr] = temp;
	return temp;
}
#ifdef MP6570_IIC_MODE 

void IIC_WriteRegister(unsigned char slave_addr, unsigned char reg_addr,u16 data)
{
	u8 data1, data2;
	reg[reg_addr] = data;
	data2 = (u8)data;
	data1 = (u8)(data>>8);
	IIC_WR_Byte(reg_addr,data1,data2);	
}
u16 IIC_ReadRegister(unsigned char slave_addr, unsigned char reg_addr)
{
	u16 temp;
	temp = IIC_RD_Byte(slave_addr,reg_addr);//SPI_ByteRead(reg_addr);
	reg[reg_addr] = temp;
	return temp;
}

//延时
void IIC_delay(void)
{
	u8 t=48;//8;
	while(t--);
}

//起始信号
void I2C_Start(void)
{
	IIC_SDA_Set();
	IIC_SCL_Set();
	IIC_delay();
	IIC_SDA_Clr();
	IIC_delay();
	IIC_SCL_Clr();
	IIC_delay();
}

//结束信号
void I2C_Stop(void)
{
	IIC_SDA_Clr();
	IIC_delay();
	IIC_SCL_Set();
	IIC_delay();
	IIC_SDA_Set();
}
//发送应答位
void I2C_Ack(void) //测数据信号的电平
{
	IIC_SDA_Set();
	IIC_delay();
	IIC_SCL_Set();
	IIC_delay();
	IIC_SCL_Clr();
	IIC_delay();
}
unsigned char SDA_Read(void)
{
	unsigned char rec;
	rec=gpio_input_data_bit_read(GPIOA,GPIO_MP6570_MISO);
	return rec;
}
//等待信号响应
unsigned char I2C_WaitAck(void) //测数据信号的电平
{
	unsigned char ack;
	unsigned char timeout ;
	IIC_SCL_Clr();
	set_r_in();
	IIC_delay();	
	IIC_SCL_Set();
	
	IIC_delay();
	//wait ack；	
	if(SDA_Read())
	{
		ack=1;
	}
	else
	{
		while(1)
		{
			timeout++;
			if(timeout>240) 
			{
				ack=0;
				break;
			}
		}
	}
	set_r_out();
	IIC_SCL_Clr();
	IIC_delay();
	return ack;
}

void I2C_NoAck(void) //测数据信号的电平
{
	uint32_t ack,num;
	IIC_SCL_Clr();
	IIC_delay();
	IIC_SCL_Set();
	IIC_delay();
	
	IIC_SDA_Clr();
	IIC_delay();
	IIC_SCL_Clr();
	IIC_delay();  
}


//写入一个字节
void Send_Byte(u8 dat)
{
	u8 i;
	for(i=0;i<8;i++)
	{
		if(dat&0x80)//将dat的8位从最高位依次写入
		{
			IIC_SDA_Set();
    }
		else
		{
			IIC_SDA_Clr();
    }
		IIC_delay();
		IIC_SCL_Set();
		IIC_delay();
		IIC_SCL_Clr();//将时钟信号设置为低电平
		dat<<=1;
  }
}
//读
unsigned char Read_Byte(void)
{	
  unsigned char i = 8;
  unsigned char byte = 0;

  IIC_SDA_Set();
	IIC_SCL_Clr();	
  set_r_in();
	IIC_delay();	
  while (i--)
  {
    byte <<= 1;
    IIC_SCL_Clr();
    IIC_delay();
    IIC_SCL_Set();
    IIC_delay();
    if (SDA_Read())
    {
      byte |= 0x01;
    }
  }
  IIC_SCL_Clr();
  IIC_delay();
	set_r_out();
	byte=3;
  return byte;
}
//发送一个字节
void IIC_WR_Byte(unsigned char addr, unsigned char data1, unsigned char data2)
{
	u8 byte1,byte2,byte3;
	I2C_Start();
	Send_Byte((0x12<<1));//设备地址
	I2C_WaitAck();
	Send_Byte(addr);//寄存器地址
	I2C_WaitAck();
	Send_Byte(data1);
	I2C_WaitAck();
	Send_Byte(data2);
	I2C_WaitAck();
	I2C_Stop();
}
u16 IIC_RD_Byte(unsigned char slave_addr, unsigned char reg_addr)
{
	u16 temp;
	u8 byte1,byte2;	
	slave_addr = (slave_addr<<1) & 0xFE;
	I2C_Start();
	Send_Byte(slave_addr);
	I2C_WaitAck();
	Send_Byte(reg_addr);
	I2C_WaitAck();
	I2C_Start();
	Send_Byte(slave_addr|0x01);//RD
	I2C_WaitAck();
	byte1 =Read_Byte();
	I2C_Ack();
	byte2=Read_Byte();
	I2C_NoAck();
	I2C_Stop();
	temp=byte1*256+byte2;
	return temp;
}

void  MP6570_IICPortInit(void)
{
	gpio_init_type gpio_initstructure;
  spi_init_type spi_init_struct;
  
  crm_periph_clock_enable(CRM_GPIOA_PERIPH_CLOCK, TRUE);
  crm_periph_clock_enable(CRM_GPIOB_PERIPH_CLOCK, TRUE);  
  
//  gpio_pin_remap_config(SWJTAG_GMUX_010, TRUE);
//  gpio_pin_remap_config(SPI1_MUX_01, TRUE);//Remap	
//	
	 /* mp6750_EN*/
  gpio_initstructure.gpio_out_type       = GPIO_OUTPUT_PUSH_PULL;  
  gpio_initstructure.gpio_pull           = GPIO_PULL_DOWN;  
  gpio_initstructure.gpio_mode           = GPIO_MODE_OUTPUT;  
  gpio_initstructure.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
  gpio_initstructure.gpio_pins           = GPIO_MP6570_EN;
  gpio_init(GPIOB, &gpio_initstructure);
  
  /* software cs, pB15 as a general io to control flash cs */
	  
  
  gpio_initstructure.gpio_out_type       = GPIO_OUTPUT_PUSH_PULL;  
  gpio_initstructure.gpio_pull           = GPIO_PULL_UP;  
  gpio_initstructure.gpio_mode           = GPIO_MODE_OUTPUT;  
  gpio_initstructure.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
  gpio_initstructure.gpio_pins           = GPIO_MP6570_CS;
  gpio_init(GPIOB, &gpio_initstructure);
  	
	
	  /* sck */ 
  gpio_initstructure.gpio_out_type       = GPIO_OUTPUT_OPEN_DRAIN;//GPIO_OUTPUT_PUSH_PULL;
  gpio_initstructure.gpio_pull           = GPIO_PULL_NONE;//GPIO_PULL_UP;
  gpio_initstructure.gpio_mode           = GPIO_MODE_OUTPUT;
  gpio_initstructure.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
  gpio_initstructure.gpio_pins           = GPIO_MP6570_SCK;
  gpio_init(GPIOA, &gpio_initstructure);
    
  /* miso */
	gpio_initstructure.gpio_out_type       = GPIO_OUTPUT_OPEN_DRAIN;//GPIO_OUTPUT_PUSH_PULL;
  gpio_initstructure.gpio_pull           = GPIO_PULL_NONE;  
	gpio_initstructure.gpio_mode           = GPIO_MODE_OUTPUT;//SLAVE
  gpio_initstructure.gpio_pins           = GPIO_MP6570_MISO;
  gpio_init(GPIOA, &gpio_initstructure);
	
	gpio_bits_set(GPIOB, GPIO_MP6570_EN);	
  
}

#endif

unsigned short int  MP6570_PortInit(void)
{
	unsigned short int  err;

	gpio_init_type gpio_initstructure;
  spi_init_type spi_init_struct;
  
  crm_periph_clock_enable(CRM_GPIOA_PERIPH_CLOCK, TRUE);
  crm_periph_clock_enable(CRM_GPIOB_PERIPH_CLOCK, TRUE);  
	crm_periph_clock_enable(CRM_IOMUX_PERIPH_CLOCK, TRUE);
//	
	 /* mp6750_EN*/
  gpio_initstructure.gpio_out_type       = GPIO_OUTPUT_PUSH_PULL;  
  gpio_initstructure.gpio_pull           = GPIO_PULL_DOWN;  
  gpio_initstructure.gpio_mode           = GPIO_MODE_OUTPUT;  
  gpio_initstructure.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
  gpio_initstructure.gpio_pins           = GPIO_MP6570_EN;
  gpio_init(GPIOB, &gpio_initstructure);
  
  /* software cs, pB15 as a general io to control flash cs */	  
  
  gpio_initstructure.gpio_out_type       = GPIO_OUTPUT_PUSH_PULL;  
  gpio_initstructure.gpio_pull           = GPIO_PULL_UP;  
  gpio_initstructure.gpio_mode           = GPIO_MODE_OUTPUT;  
  gpio_initstructure.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
  gpio_initstructure.gpio_pins           = GPIO_MP6570_CS;
  gpio_init(GPIOB, &gpio_initstructure);
  
  /* sck */ 
  gpio_initstructure.gpio_out_type       = GPIO_OUTPUT_PUSH_PULL;
  gpio_initstructure.gpio_pull           = GPIO_PULL_DOWN;
  gpio_initstructure.gpio_mode           = GPIO_MODE_MUX;
  gpio_initstructure.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
  gpio_initstructure.gpio_pins           = GPIO_MP6570_SCK;
  gpio_init(GPIOA, &gpio_initstructure);
  
  /* miso */
  gpio_initstructure.gpio_pull           = GPIO_PULL_DOWN;  
	gpio_initstructure.gpio_mode           = GPIO_MODE_MUX;//SLAVE
	gpio_initstructure.gpio_mode           = GPIO_MODE_INPUT;  //MASTER
  gpio_initstructure.gpio_pins           = GPIO_MP6570_MISO;
  gpio_init(GPIOA, &gpio_initstructure);
  
  /* mosi */ 
	gpio_initstructure.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
	gpio_initstructure.gpio_out_type       = GPIO_OUTPUT_PUSH_PULL;
  gpio_initstructure.gpio_pull           = GPIO_PULL_DOWN;
  gpio_initstructure.gpio_mode           = GPIO_MODE_MUX;
  gpio_initstructure.gpio_pins           = GPIO_MP6570_MOSI;
  gpio_init(GPIOA, &gpio_initstructure);  

//	NSS_RESET;
  crm_periph_clock_enable(CRM_SPI1_PERIPH_CLOCK, TRUE);
	
  spi_default_para_init(&spi_init_struct);
  spi_init_struct.transmission_mode = SPI_TRANSMIT_FULL_DUPLEX;
  spi_init_struct.master_slave_mode = SPI_MODE_MASTER;
  spi_init_struct.mclk_freq_division =SPI_MCLK_DIV_16;//6.25M //SPI_MCLK_DIV_32;//
  spi_init_struct.first_bit_transmission = SPI_FIRST_BIT_MSB;
  spi_init_struct.frame_bit_num = SPI_FRAME_8BIT;
  spi_init_struct.clock_polarity = SPI_CLOCK_POLARITY_HIGH;
  spi_init_struct.clock_phase = SPI_CLOCK_PHASE_2EDGE;
  spi_init_struct.cs_mode_selection = SPI_CS_SOFTWARE_MODE;
  spi_init(SPI1, &spi_init_struct);	
  spi_enable(SPI1, TRUE); 	
	err	=MP6570_ReadRegister(MP6570_SLAVE_Addr, 0x00);	
	return err;
}
void  MP6570_SetModeToSPI(void)
{	
	unsigned short int  err;
	/***to IIC***/
 //to IIC， 	
		MP6570_IICPortInit();
//	err	=IIC_RD_Byte(0x12, 0x00);
//	if(err=0xFFFF)
//	{
		IIC_WriteRegister(0x12, 0x62, 0);			//turn internal clock on
		IIC_WriteRegister(0x12, 0x61, 0);	
		IIC_WR_Byte(0x60,0x00,0x00); //Turn on MP6570	
//	reg[0x33]=IIC_RD_Byte(0x12,0x33);
//	reg[0x33]=(reg[0x33]&0xFFFC);//IIC->SPI  [2:1]=01;
		reg[0x33]=0xE003;
		IIC_WriteRegister(0x12,0x33, reg[0x33]);
		IIC_WriteRegister(0x12,0x64, 0x0000);//RAM ->nvm		
		delay_1ms(100);
		gpio_bits_reset(GPIOB, GPIO_MP6570_EN);
//	}
	/**end IIC****/
		MP6570_PortInit();
}
