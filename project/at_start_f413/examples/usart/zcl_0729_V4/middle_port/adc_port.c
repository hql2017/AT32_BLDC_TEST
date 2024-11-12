
#include "at32f413_int.h"
#include "at32f413_board.h"

#include "adc_port.h"


#ifdef APEX_FUNCTION_EBABLE	

#define VALUE_8K_SEQUENCE  1
#define VALUE_400_SEQUENCE  2
#define VBAT_SEQUENCE  3

uint16_t adc1_ordinary_valuetab[64]= {0};
static uint32_t adc_value[4] = {0};
#else
#define VBAT_SEQUENCE  1 
uint16_t adc1_ordinary_valuetab[32] = {0};
static uint32_t adc_value[2] = {0};
#endif
uint8_t adc_trans_complete_flag = 0;

//static error_status  ValueChexk(unsigned int data,unsigned int minData,unsigned int maxData)
//{
//	error_status err;
//	
//	if(data<minData) //data err
//	{
//		err=ERROR;
//	}
//	else 
//	{
//		err=SUCCESS;
//	}
//	return 	err;
//}
double Filter_VCC(const double ResrcData,double ProcessNiose_Q,double MeasureNoise_R,double InitialPrediction)
{
	double R = MeasureNoise_R;
	double Q = ProcessNiose_Q;
	
	static	double x_last;

	double x_mid = x_last;
	double x_now;

	static	double p_last;

	double p_mid ;
	double p_now;
	double kg;	

	x_mid=x_last;
	p_mid=p_last+Q;
	kg=p_mid/(p_mid+R);
	x_now=x_mid+kg*(ResrcData-x_mid);
		
	p_now=(1-kg)*p_mid;

	p_last = p_now;
	x_last = x_now;

	return x_now;		
}

/**ADC_gpio_config
  * @brief  ADC_gpio_config
  * @param  none
	* @retval  none
  */
static void ADC_gpio_config(void)
{
		gpio_init_type gpio_initstructure;
	
	#ifdef APEX_FUNCTION_EBABLE	
	
		crm_periph_clock_enable(APEX_8K_CLOCK, TRUE);
		gpio_default_para_init(&gpio_initstructure);//8k
		gpio_initstructure.gpio_mode = GPIO_MODE_ANALOG;//模拟
		gpio_initstructure.gpio_pins = APEX_8K_IO;
		gpio_initstructure.gpio_pull = GPIO_PULL_NONE;
		gpio_init(APEX_8K_PORT, &gpio_initstructure);	
		
		crm_periph_clock_enable(APEX_400_CLOCK, TRUE);
		gpio_default_para_init(&gpio_initstructure);//400HZ
		gpio_initstructure.gpio_mode = GPIO_MODE_ANALOG;//模拟
		gpio_initstructure.gpio_pins = APEX_400_IO;
		gpio_initstructure.gpio_pull = GPIO_PULL_NONE;
		gpio_init(APEX_400_PORT, &gpio_initstructure);	
	
	#endif	
	
		crm_periph_clock_enable(CRM_GPIOA_PERIPH_CLOCK, TRUE);
		gpio_default_para_init(&gpio_initstructure);//Vbat
		gpio_initstructure.gpio_mode = GPIO_MODE_ANALOG;//模拟
		gpio_initstructure.gpio_pins = GPIO_PINS_3;
		gpio_initstructure.gpio_pull = GPIO_PULL_NONE;
		gpio_init(GPIOA, &gpio_initstructure);	


//		crm_periph_clock_enable(CRM_GPIOA_PERIPH_CLOCK, TRUE);
//		gpio_default_para_init(&gpio_initstructure);//VBUS 9V
//		gpio_initstructure.gpio_mode = GPIO_MODE_ANALOG;//模拟
//		gpio_initstructure.gpio_pins = GPIO_PINS_4;
//		gpio_initstructure.gpio_pull = GPIO_PULL_NONE;
//		gpio_init(GPIOA, &gpio_initstructure);		
}
/**ADC DMA CONFIG
  * @brief  ADC_DMA
  * @param  none
	* @retval  none
  */
static void ADC_DMA_Init(void)
{	
		dma_init_type dma_init_struct;
		crm_periph_clock_enable(CRM_DMA1_PERIPH_CLOCK, TRUE);
		nvic_irq_enable(DMA1_Channel1_IRQn, 0, 0);
		dma_reset(DMA1_CHANNEL1);
		dma_default_para_init(&dma_init_struct);
		dma_init_struct.buffer_size =48;//64;//quick response//512;//slowly response
		dma_init_struct.direction = DMA_DIR_PERIPHERAL_TO_MEMORY;
		dma_init_struct.memory_base_addr = (uint32_t)adc1_ordinary_valuetab;
		dma_init_struct.memory_data_width = DMA_MEMORY_DATA_WIDTH_HALFWORD;
		dma_init_struct.memory_inc_enable = TRUE;
		dma_init_struct.peripheral_base_addr = (uint32_t)&(ADC1->odt);
		dma_init_struct.peripheral_data_width = DMA_PERIPHERAL_DATA_WIDTH_HALFWORD;
		dma_init_struct.peripheral_inc_enable = FALSE;
		dma_init_struct.priority = DMA_PRIORITY_HIGH;
		dma_init_struct.loop_mode_enable =TRUE;//FALSE;
		dma_init(DMA1_CHANNEL1, &dma_init_struct);

//	dma_flexible_config(DMA1, FLEX_CHANNEL1, DMA_FLEXIBLE_ADC1);
		dma_interrupt_enable(DMA1_CHANNEL1, DMA_FDT_INT, TRUE);
		dma_channel_enable(DMA1_CHANNEL1, TRUE);
	
}
/**ADC_Init
  * @brief  ADC_init
  * @param  none
	* @retval  none
  */
static void ADC_Init(void)
{	
		adc_base_config_type  adc_base_struct;
		crm_periph_clock_enable(CRM_ADC1_PERIPH_CLOCK, TRUE);
		crm_adc_clock_div_set(CRM_ADC_DIV_6);

		adc_combine_mode_select(ADC_INDEPENDENT_MODE);
		adc_base_default_para_init(&adc_base_struct);	
		adc_base_struct.data_align =ADC_RIGHT_ALIGNMENT;
		adc_base_struct.ordinary_channel_length = 3;//4;//通道数量
		adc_base_struct.repeat_mode =TRUE;//FALSE;//TRUE;//adc重复模式,自动转换
		adc_base_struct.sequence_mode = TRUE;//单一通道flase，多个通道ture
		adc_base_config(ADC1, &adc_base_struct);
		adc_ordinary_channel_set(ADC1, ADC_CHANNEL_1, VALUE_8K_SEQUENCE , ADC_SAMPLETIME_239_5);  //8k
		adc_ordinary_channel_set(ADC1, ADC_CHANNEL_2, VALUE_400_SEQUENCE , ADC_SAMPLETIME_239_5);  //400
		adc_ordinary_channel_set(ADC1, ADC_CHANNEL_3, VBAT_SEQUENCE , ADC_SAMPLETIME_239_5); //vbat
//	adc_ordinary_channel_set(ADC1, ADC_CHANNEL_4, 4, ADC_SAMPLETIME_239_5);  //vbus

		adc_ordinary_conversion_trigger_set(ADC1, ADC12_ORDINARY_TRIG_SOFTWARE, TRUE);//软件自动触发
		adc_dma_mode_enable(ADC1, TRUE);

		adc_enable(ADC1, TRUE);
		adc_calibration_init(ADC1);
		while(adc_calibration_init_status_get(ADC1));
		adc_calibration_start(ADC1);
		while(adc_calibration_status_get(ADC1));	 
}

void ADC1_2_IRQHandler(void)
{
  if(adc_flag_get(ADC1, ADC_PCCE_FLAG) != RESET)
  { 
		
    adc_flag_clear(ADC1, ADC_PCCE_FLAG);//see errata  
  } 
	  if(adc_flag_get(ADC1, ADC_CCE_FLAG) != RESET)
  {    
    adc_flag_clear(ADC1, ADC_CCE_FLAG);//see errata  
  } 
}

/**start_adc_acquisition
  * @brief  start_adc_acquisition
  * @param  none
	* @retval  none
  */
void start_adc_acquisition(void)
{
	unsigned int timeOut;
	timeOut=0;
	ADC_gpio_config();
  ADC_DMA_Init();
  ADC_Init();
	adc_trans_complete_flag=0;
	adc_ordinary_software_trigger_enable(ADC1,TRUE);
  while(adc_trans_complete_flag == 0)
	{
		timeOut++;
		if(timeOut>144000) break;
	}
	fresh_adc_value();
}

#ifdef APEX_FUNCTION_EBABLE	

static unsigned   int LevelValue(unsigned char chanel)//求均值
{
  unsigned int ret=0,temp=0;
  unsigned char j;
	if(chanel==0)	 chanel=1;
	if(chanel>3)	 chanel=4;
	for(j=0;j<16;j++)
	{	
		temp+=adc1_ordinary_valuetab[j*3+chanel-1];	
	}	
	ret=temp>>4;			
	return ret;
}

/**restart_adc_sampling
  * @brief  restart_adc_sampling
  * @param  none
	* @retval  none
  */
 void restart_adc_sampling(void)
{		
//	unsigned  int timeout;
	adc_trans_complete_flag=0;
//	while(adc_trans_complete_flag==0)	
//	{
//		timeout++;
//		if(timeout>144000) break;//3ms	
//	}//等待上轮采集完成

}

/**fresh_adc_value
  * @brief  fresh_adc_value
  * @param  none
	* @retval  none
  */
unsigned char fresh_adc_value(void)
{
	unsigned char ret;	
	ret=0;
	if(adc_trans_complete_flag!=0)//())//保存上次采集值
	{	
		ret=1;	
		adc_value[0]=LevelValue(VALUE_8K_SEQUENCE );//8k
		adc_value[1]=LevelValue(VALUE_400_SEQUENCE );//400
		adc_value[2]=LevelValue(VBAT_SEQUENCE );//vbat
	//adc_value[3]=LevelValue(3);
	//filter			
//	adc_value[2]=(uint32_t)Filter_VCC((double)adc_value[2], 0.001,6.0,0.0);//vbat	
		adc_trans_complete_flag=0;		
	}		
	return ret;
}
/**get_vbat_value
  * @brief  get_vbat_value
  * @param  none
	* @retval  none
  */
unsigned short int get_gc_8k_value(void)
{
	unsigned short int voltage8kValue;
	voltage8kValue=adc_value[0];//*0.8057;//voltage=adc_value[0]*3300/4096;
	return voltage8kValue;
}
/**get_v_motor_value
  * @brief  get_v_motor_value
  * @param  none
	* @retval  value
  */
unsigned short int get_gc_400_value(void)
{
	unsigned short int voltage400Value;
	voltage400Value=adc_value[1];//*0.8057;//voltage=adc_value[1]*3300/4096;
	return voltage400Value;
}
/**get_vbat_value
  * @brief  get_vbat_value
  * @param  none
	* @retval  none
  */
unsigned short int get_vbat_value(void)
{	
	unsigned short int batValue;			
	batValue=adc_value[2]*1.7079;//*0.8057*2;//voltage=2*adc_value[1]*3300/4096;
	return batValue;
}
/**get_v_motor_value
  * @brief  get_v_motor_value
  * @param  none
	* @retval  value
  */
unsigned short int get_v_motor_value(void)
{
	unsigned short int batValue;			
	batValue=adc_value[3]*1.628;//*0.8057*2;//voltage=2*adc_value[1]*3300/4096;
	return batValue;
}
#else
static unsigned   int LevelValue(unsigned char chanel)//求均值
{
  unsigned int temp=0;
  unsigned   int ret,j;
	if(chanel==0)//vbat
	{			
		for(j=0;j<16;j++)
		{
			temp+=adc1_ordinary_valuetab[j*4];
		}		
		ret=temp>>4;	
	}
	else if(chanel==1)//vbus
	{		
		for(j=0;j<16;j++)
		{
			temp+=adc1_ordinary_valuetab[j*4+1];
		}		
		ret=temp>>4;
	}	
	return ret;
}
/**fresh_adc_value
  * @brief  get_vbat_value
  * @param  none
	* @retval  none
  */
void fresh_adc_value(void)
{
		adc_value[0]=LevelValue(0);//vbat
		adc_value[1]=LevelValue(1);//vbus
	//filter	
		adc_value[0]=(uint32_t)Filter_VCC((double)adc_value[0], 0.001,6.0,0.0);//vbat		
}
/**get_vbat_value
  * @brief  get_vbat_value
  * @param  none
	* @retval  none
  */
unsigned short int get_vbat_value(void)
{	
	unsigned short int batValue;			
	batValue=adc_value[0]*1.628;//*0.8057*2;//voltage=2*adc_value[1]*3300/4096;
	return batValue;
}
/**get_v_motor_value
  * @brief  get_v_motor_value
  * @param  none
	* @retval  value
  */
unsigned short int get_v_motor_value(void)
{
	unsigned short int batValue;			
	batValue=adc_value[1]*1.628;//*0.8057*2;//voltage=2*adc_value[1]*3300/4096;
	return batValue;
}
#endif

