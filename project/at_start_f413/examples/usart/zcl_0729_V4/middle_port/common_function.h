#ifndef __COMMON_FUNCTION
#define __COMMON_FUNCTION
#include "stdint.h"

typedef int16_t nrf_saadc_value_t;
 
extern nrf_saadc_value_t adc_val[512];

int32_t compDec(const void *a, const void *b);
void power_off(void);
uint16_t vbat_judge(uint16_t power_mv);


//filter
int32_t filter_level(int32_t value,int32_t* value_buf,uint8_t len);
int32_t filter_buff(int32_t gc_ref_cap_buf, int32_t gc_ref_res_buf,uint8_t buf_num);
int32_t filter_rat(int32_t rat_ref);
//电压转换，
int16_t vbat_data_progress(int16_t *base, int16_t num, int16_t start, int16_t end);

//计算 RMSb
int32_t gc_rms_progress(int16_t *base, int16_t num);
uint16_t cal_foc_curr(uint16_t mv);//foc电压转换为电流
int8_t HALL_GetPhase(void);
void mt_pwm1_inc(void);
void mt_pwm2_inc(void);
void mt_pwm3_inc(void);
void mt_pwm1_dec(void);
void mt_pwm2_dec(void);
void mt_pwm3_dec(void);
#endif
