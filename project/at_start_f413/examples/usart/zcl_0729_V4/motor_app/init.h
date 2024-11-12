#ifndef _INIT_H
#define _INIT_H


//spi pin definition
#define SPI_MASTER                   	SPI1
#define SPI_MASTER_CLK               	RCC_APB2Periph_SPI1
#define SPI_MASTER_GPIO              	GPIOA
#define SPI_MASTER_GPIO_CLK          	RCC_APB2Periph_GPIOA  
#define SPI_MASTER_PIN_NSS           	GPIO_PINS_12//GPIO_Pin_4
#define SPI_MASTER_PIN_SCK           	GPIO_Pin_5
#define SPI_MASTER_PIN_MISO          	GPIO_Pin_6
#define SPI_MASTER_PIN_MOSI          	GPIO_Pin_7


//Ó²¼þ½Ó¿Ú
#define GPIO_MP6570_CS  GPIO_PINS_12 //PB12
#define GPIO_MP6570_EN GPIO_PINS_14//PB14//GPIO_PINS_0 //PA0
#define GPIO_MP6570_SCK  GPIO_PINS_5 //PA5
#define GPIO_MP6570_MISO  GPIO_PINS_6 //PA6
#define GPIO_MP6570_MOSI  GPIO_PINS_7 //PA7

#define NSS_RESET  gpio_bits_reset(GPIOB,GPIO_MP6570_CS);
#define NSS_SET   gpio_bits_set(GPIOB,GPIO_MP6570_CS);


//#define SPI_CS_HIGH()  gpio_bits_set(GPIOB,GPIO_MP6570_CS)

void CKCU_Configuration(void);
void NVIC_Configuration(void);
void GPIO_Configuration(void);
void SPI1_Configuration(void);
void TIM2_Configuration(void);
void init_peripheral(void);


#endif
