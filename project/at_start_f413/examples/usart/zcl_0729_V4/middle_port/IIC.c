#include "mp6570.h"
#include "at32f413_spi.h"

#include "delay.h"




#define IIC_SCL_Clr() gpio_bits_reset(GPIOA,GPIO_PINS_5)//SCL
#define IIC_SCL_Set() gpio_bits_set(GPIOA,GPIO_PINS_5)

#define IIC_SDA_Clr() gpio_bits_reset(GPIOA,GPIO_PINS_6)//SDA
#define IIC_SDA_Set() gpio_bits_set(GPIOA,GPIO_PINS_6)


#define IIC_CMD  0	//д����
#define IIC_DATA 1	//д����

void  IICPortInit(void)
{
	gpio_init_type gpio_initstructure;  
  
  crm_periph_clock_enable(CRM_GPIOA_PERIPH_CLOCK, TRUE);
  crm_periph_clock_enable(CRM_GPIOB_PERIPH_CLOCK, TRUE);  
	
	  /* sck */ 
  gpio_initstructure.gpio_out_type       = GPIO_OUTPUT_OPEN_DRAIN;//GPIO_OUTPUT_PUSH_PULL;
  gpio_initstructure.gpio_pull           = GPIO_PULL_NONE;//GPIO_PULL_UP;
  gpio_initstructure.gpio_mode           = GPIO_MODE_OUTPUT;
  gpio_initstructure.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
  gpio_initstructure.gpio_pins           = GPIO_PINS_5;
  gpio_init(GPIOA, &gpio_initstructure);
    
  /* SDA */
	gpio_initstructure.gpio_out_type       = GPIO_OUTPUT_OPEN_DRAIN;//GPIO_OUTPUT_PUSH_PULL;
  gpio_initstructure.gpio_pull           = GPIO_PULL_NONE;  
	gpio_initstructure.gpio_mode           = GPIO_MODE_OUTPUT;//SLAVE
  gpio_initstructure.gpio_pins           = GPIO_PINS_6;
  gpio_init(GPIOA, &gpio_initstructure);
  
}
void set_r_in(void)
{
	gpio_init_type gpio_initstructure;
  gpio_initstructure.gpio_pull           = GPIO_PULL_NONE; 
	gpio_initstructure.gpio_mode           = GPIO_MODE_INPUT;  //MASTER  
  gpio_initstructure.gpio_pins           = GPIO_PINS_6;
  gpio_init(GPIOA, &gpio_initstructure);

}
void set_r_out(void)
{
	gpio_init_type gpio_initstructure;
	gpio_initstructure.gpio_drive_strength =  GPIO_DRIVE_STRENGTH_STRONGER;
	gpio_initstructure.gpio_out_type       = GPIO_OUTPUT_OPEN_DRAIN;//GPIO_OUTPUT_PUSH_PULL;
  gpio_initstructure.gpio_pull           = GPIO_PULL_NONE;//GPIO_PULL_UP;
  gpio_initstructure.gpio_mode           = GPIO_MODE_OUTPUT;
  gpio_initstructure.gpio_pins           = GPIO_PINS_6;
  gpio_init(GPIOA, &gpio_initstructure);
}

//��ʱ,����Ƶ��ͨ�������й�
void IIC_delay(void)
{
	u8 t=48;//8;
	while(t--);
}

//��ʼ�ź�
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

//�����ź�
void I2C_Stop(void)
{
	IIC_SDA_Clr();
	IIC_delay();
	IIC_SCL_Set();
	IIC_delay();
	IIC_SDA_Set();
}
//����Ӧ��λ
void I2C_Ack(void) //�������źŵĵ�ƽ
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
	rec=gpio_input_data_bit_read(GPIOA,GPIO_PINS_6);
	return rec;
}
//�ȴ��ź���Ӧ
unsigned char I2C_WaitAck(void) //�������źŵĵ�ƽ
{
	unsigned char ack;
		unsigned char timeout ;
	IIC_SCL_Clr();
	set_r_in();
	IIC_delay();	
	IIC_SCL_Set();
	
	IIC_delay();
	//wait ack��	
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

void I2C_NoAck(void) //�������źŵĵ�ƽ
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


//д��һ���ֽ�
void Send_Byte(u8 dat)
{
	u8 i;
	for(i=0;i<8;i++)
	{
		if(dat&0x80)//��dat��8λ�����λ����д��
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
		IIC_SCL_Clr();//��ʱ���ź�����Ϊ�͵�ƽ
		dat<<=1;
  }
}
//��
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
//����һ���ֽ�
void IIC_WR_Byte(unsigned char addr, unsigned char data1, unsigned char data2)
{
	u8 byte1,byte2,byte3;
	I2C_Start();
	Send_Byte((0x12<<1));//�豸��ַ
	I2C_WaitAck();
	Send_Byte(addr);//�Ĵ�����ַ
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






