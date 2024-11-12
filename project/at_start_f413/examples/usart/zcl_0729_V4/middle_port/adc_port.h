#ifndef __ADC_PORT
#define __ADC_PORT
#include "stdint.h"

void user_adc_init(void);
void ADC_gpio_config(void);
void ADC_DMA_Init(void);
void ADC_Init(void);

extern void start_adc_acquisition(void);
extern unsigned short int get_v_motor_value(void);
extern unsigned short int get_vbat_value(void);
extern  unsigned char fresh_adc_value(void);
extern void restart_adc_sampling(void);

extern unsigned short int get_gc_8k_value(void);
extern unsigned short int get_gc_400_value(void);

extern double Filter_VCC(const double ResrcData,double ProcessNiose_Q,double MeasureNoise_R,double InitialPrediction);
#endif
