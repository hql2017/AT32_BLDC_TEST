#include "main.h"
 
nrf_saadc_value_t adc_val[512];
static int32_t    adc_filter_res_val[16]={0};
static int32_t    adc_filter_cap_val[16]={0};
//static int32_t    adc_filter_bat_val[16]={0};
static int32_t    adc_filter_rat_val[8]={0};

//static  int32_t val400;//400hz通道ADC采样值阈值
int32_t gc_num_list[31] = {0};//根测比例数据映射表

int32_t compDec(const void *a, const void *b)
{
	return *(int32_t *)b - *(int32_t *)a;
}
#ifndef DRIVE_BOARD

void power_off(void)
{
  printf("POWER OFF\r\n");
  write_para_judge();
  yellow_off();
  blue_off();
  tmr_channel_value_set(TMR10, TMR_SELECT_CHANNEL_1,0); //beep off
  OLED_Display_Clear();
  
  device_power_off();
  while(1)
  {
    delay_ms(100);
    if(get_insert_state() != RESET)
      break;
  }
}
#endif

uint16_t vbat_judge(uint16_t device_voltage_mv)
{
//  printf("device_voltage_mv IS %d\r\n",device_voltage_mv);
           if(device_voltage_mv>=4000)
            {
                  return 4;
            }
            else if((4000 > device_voltage_mv)&&(device_voltage_mv >=3650))
            {
                  return 3;
            }
            else if((3650 > device_voltage_mv)&&(device_voltage_mv >=3450))
            {
                  return 2;
            }
            else if((3450 > device_voltage_mv)&&(device_voltage_mv >=3200))
            {
                  return 1;
            }
            else if(3200 > device_voltage_mv)
            {
                  return 0;
            } 
            return 0;    
}

//filter
int32_t filter_level(int32_t value,int32_t* value_buf,uint8_t len)
{
	double temp=0;
	int32_t min_value=0,max_value=0;
	uint8_t i;
		temp=0;
		min_value= value_buf[0];
		max_value= value_buf[0];
		for(i=0;i<len;i++){
			temp+=value_buf[i]*value_buf[i];			
			
			if(min_value>value_buf[i])  min_value= value_buf[i];
			if(max_value<value_buf[i])  max_value= value_buf[i];
		}
		if(len>2){
				temp=temp/len;
//			temp=(temp-max_value*max_value-min_value*min_value)/(len-2);	
				temp = sqrt(temp);								
		}
		else temp=value;
		return temp;		
}

int32_t filter_buff(int32_t gc_ref_cap_buf, int32_t gc_ref_res_buf,uint8_t buf_num)
{
	int32_t temp=0;	
	if(buf_num==0){
			adc_filter_cap_val[0]=adc_filter_cap_val[1];
			adc_filter_cap_val[1]=adc_filter_cap_val[2];adc_filter_cap_val[2]=adc_filter_cap_val[3];
			adc_filter_cap_val[3]=adc_filter_cap_val[4];adc_filter_cap_val[4]=adc_filter_cap_val[5];			
			adc_filter_cap_val[5]=adc_filter_cap_val[6];adc_filter_cap_val[6]=adc_filter_cap_val[7];
			adc_filter_cap_val[7]=adc_filter_cap_val[8];adc_filter_cap_val[8]=adc_filter_cap_val[9];
			adc_filter_cap_val[9]=adc_filter_cap_val[10];	adc_filter_cap_val[10]=adc_filter_cap_val[11];
			adc_filter_cap_val[11]=adc_filter_cap_val[12];adc_filter_cap_val[12]=adc_filter_cap_val[13];
			adc_filter_cap_val[13]=adc_filter_cap_val[14];adc_filter_cap_val[14]=adc_filter_cap_val[15];	
			adc_filter_cap_val[15]=gc_ref_cap_buf;
			temp=filter_level(gc_ref_cap_buf,adc_filter_cap_val,16);
			if(temp>gc_ref_cap_buf+6||temp+6<gc_ref_cap_buf) {temp=gc_ref_cap_buf;}
			return temp;	
	}
	else if(buf_num==1){	
			adc_filter_res_val[0]=adc_filter_res_val[1];
			adc_filter_res_val[1]=adc_filter_res_val[2];adc_filter_res_val[2]=adc_filter_res_val[3];
			adc_filter_res_val[3]=adc_filter_res_val[4];adc_filter_res_val[4]=adc_filter_res_val[5];
			adc_filter_res_val[5]=adc_filter_res_val[6];adc_filter_res_val[6]=adc_filter_res_val[7];
			adc_filter_res_val[7]=adc_filter_res_val[8];adc_filter_res_val[8]=adc_filter_res_val[9];
			adc_filter_res_val[9]=adc_filter_res_val[10];adc_filter_res_val[10]=adc_filter_res_val[11];
			adc_filter_res_val[11]=adc_filter_res_val[12];adc_filter_res_val[12]=adc_filter_res_val[13];
			adc_filter_res_val[13]=adc_filter_res_val[14];adc_filter_res_val[14]=adc_filter_res_val[15];
			adc_filter_res_val[15]=gc_ref_res_buf;	
			temp=filter_level(gc_ref_res_buf,adc_filter_res_val,16);			
			if(temp>gc_ref_res_buf+6||temp+6<gc_ref_res_buf)  {temp=gc_ref_res_buf;}
			return temp;			
	}	
	return temp;	
}

int32_t filter_rat(int32_t rat_ref)
{
	double  temp=0;
	int32_t min_value=0,max_value=0;
	uint8_t i;	
		adc_filter_rat_val[0]=adc_filter_rat_val[1]; adc_filter_rat_val[1]=adc_filter_rat_val[2];
		adc_filter_rat_val[2]=adc_filter_rat_val[3];	adc_filter_rat_val[3]=adc_filter_rat_val[4];
		adc_filter_rat_val[4]=adc_filter_rat_val[5]; adc_filter_rat_val[5]=adc_filter_rat_val[6];			
		adc_filter_rat_val[6]=adc_filter_rat_val[7];	
		adc_filter_rat_val[7]=rat_ref;	
		min_value=adc_filter_rat_val[0];
		max_value=adc_filter_rat_val[0];	
		//求均值
		for(i=0;i<8;i++){
			if(min_value>adc_filter_rat_val[i])  min_value= adc_filter_rat_val[i];
			if(max_value<adc_filter_rat_val[i])  max_value= adc_filter_rat_val[i];
//			temp+=adc_filter_rat_val[i]*adc_filter_rat_val[i];
			temp+=adc_filter_rat_val[i];
		}			
				temp=(temp-min_value-max_value)/6;			
//  			temp=temp/8;	
//				temp = sqrt(temp);
		if(temp<rat_ref){
				if(temp+6<rat_ref){ 	
					return rat_ref;  
				}
				else {
					return temp;
				}			
		}else if(temp>=rat_ref){
				if(temp>rat_ref+6){
						return rat_ref;
				}	else {
					return temp;
				}			
		}
    return 0;
}
//电压转换，
int16_t vbat_data_progress(int16_t *base, int16_t num, int16_t start, int16_t end)
{
	int16_t i;
	int32_t tmp = 0;
	double j;
	if((num > start)&&(num > end)) {
		for(i=start;i<end;i++) {
			tmp += base[i];
		}
		j = ((double)tmp/(double)(end - start))*7.2/40.96;
		return ((int32_t)j);
	}
	else {
		return 0;	
	}
}
//根测工作长度计算
//rat_r 8k采样值 rat_c 400采样值
int32_t gc_calc_wl(int32_t rat_r, int32_t rat_c,uint8_t gc_insert_flag)
{
	volatile static int32_t rat;
	volatile static int32_t result = 30;
	
	rat = rat_r * 1000 / rat_c;
	rat=filter_rat(rat);	
	if((rat_r <= 250)&&(rat_c <= 250)) {
		return(-7);//短路
	}
	//400Hz压降达到阀值以下，启动根测测量	
	if(gc_insert_flag == 1) 
	{	
	//根尖区计算
			if(rat < gc_num_list[0]) {
				result = 0;
			}
			else if((gc_num_list[0] <= rat)&&(rat < gc_num_list[1])) {
				result = 0;
			}
			else if((gc_num_list[1] <= rat)&&(rat < gc_num_list[2])) {
				result = 1;
			}
			else if((gc_num_list[2] <= rat)&&(rat < gc_num_list[3])) {
				result = 2;
			}
			else if((gc_num_list[3] <= rat)&&(rat < gc_num_list[4])) {
				result = 3;
			}
			else if((gc_num_list[4] <= rat)&&(rat < gc_num_list[5])) {
				result = 4;
			}
			else if((gc_num_list[5] <= rat)&&(rat < gc_num_list[6])) {
				result = 5;
			}
			else if((gc_num_list[6] <= rat)&&(rat < gc_num_list[7])) {
				result = 6;
			}
			else if((gc_num_list[7] <= rat)&&(rat < gc_num_list[8])) {
				result = 7;
			}
			else if((gc_num_list[8] <= rat)&&(rat < gc_num_list[9])) {
				result = 8;
			}
			else if((gc_num_list[9] <= rat)&&(rat < gc_num_list[10])) {
				result = 9;
			}
			else if((gc_num_list[10] <= rat)&&(rat < gc_num_list[11])) {
				result = 10;
			}
			else if((gc_num_list[11] <= rat)&&(rat < gc_num_list[12])) {
				result = 11;
			}
			else if((gc_num_list[12] <= rat)&&(rat < gc_num_list[13])) {
				result = 12;
			}
			else if((gc_num_list[13] <= rat)&&(rat < gc_num_list[14])) {
				result = 13;
			}
			else if((gc_num_list[14] <= rat)&&(rat < gc_num_list[15])) {
				result = 14;
			}
			else if((gc_num_list[15] <= rat)&&(rat < gc_num_list[16])) {
				result = 15;
			}
			else if((gc_num_list[16] <= rat)&&(rat < gc_num_list[17])) {
				result = 16;
			}
			else if((gc_num_list[17] <= rat)&&(rat < gc_num_list[18])) {
				result = 17;
			}
			else if((gc_num_list[18] <= rat)&&(rat < gc_num_list[19])) {
				result = 18;
			}
			else if((gc_num_list[19] <= rat)&&(rat < gc_num_list[20])) {
				result = 19;
			}
			else if((gc_num_list[20] <= rat)&&(rat < gc_num_list[21])) {
				result = 20;
			}
			else if((gc_num_list[21] <= rat)&&(rat < gc_num_list[22])) {
				result = 21;
			}
			else if((gc_num_list[22] <= rat)&&(rat < gc_num_list[23])) {
				result = 22;
			}
			else if((gc_num_list[23] <= rat)&&(rat < gc_num_list[24])) {
				result = 23;
			}
			else if((gc_num_list[24] <= rat)&&(rat < gc_num_list[25])) {
				result = 24;
			}
			else if((gc_num_list[25] <= rat)&&(rat < gc_num_list[26])) {
				result = 25;
			}
			else if((gc_num_list[26] <= rat)&&(rat < gc_num_list[27])) {
				result = 26;
			}
			else if((gc_num_list[27] <= rat)&&(rat < gc_num_list[28])) {
				result = 27;
			}
			else if((gc_num_list[28] <= rat)&&(rat < gc_num_list[29])) {
				result = 30;
			}
			else if((gc_num_list[29] <= rat)&&(rat < gc_num_list[30])) {
				result = 30;
			}
			else if(gc_num_list[30] <= rat) {
				result = 30;
			}
			else {
				result = 30;
			}
	}
	else {
		result = 30;
	}
	
	return result;
}
void gc_list_init(int32_t n0, int32_t n1)
{
  double gc_rate_g,gc_rate_b,gc_rate_l;
  int32_t gc_val_h;
	int32_t gc_val_l ;		
	gc_val_l = n0-n1;//根测空载比例值减去0点校准比例值
//	double gc_rate_g = (double)(gc_val_l)*0.28*k/3.0;//(double)gc_val_l*0.28/3.0;		
	gc_rate_g = (double)(n0-n1)*1.0/29.0;//(double)gc_val_l*0.28/3.0;
  gc_rate_b = (double)(n0-n1)*0.28/3.0;			
	gc_val_h = (int32_t)((double)n1 + (gc_rate_g * 15) + (gc_rate_b * 3));
  gc_rate_l = (double)(n0-gc_val_h-10)/3.0;	
		
	
	printf("gc_val_l = %d", gc_val_l);	
	printf("gc_rate_g = %f",gc_rate_g);
	printf("gc_rate_b = %f",gc_rate_b);
//	if(gc_rate_g>15.6) gc_rate_g=15.2;
	//NRF_LOG_INFO("gc_rate_b = "NRF_LOG_FLOAT_MARKER"",NRF_LOG_FLOAT(gc_rate_b));
	//NRF_LOG_INFO("gc_rate_l = "NRF_LOG_FLOAT_MARKER"",NRF_LOG_FLOAT(gc_rate_l));

	gc_num_list[0] = (int32_t)((double)n1-gc_rate_g*14.5);
	gc_num_list[1] = (int32_t)((double)n1-gc_rate_g*12.5);
	gc_num_list[2] = (int32_t)((double)n1-gc_rate_g*11.5);
	gc_num_list[3] = (int32_t)((double)n1-gc_rate_g*9.5);
	gc_num_list[4] = (int32_t)((double)n1-gc_rate_g*6.5);
	gc_num_list[5] = (int32_t)((double)n1-gc_rate_g*3.5);			
	gc_num_list[6] = n1-gc_rate_g*1.0;//*0.95;//5%100偏差	
	gc_num_list[7] = (int32_t)((double)n1 + gc_rate_g*1.5);
	gc_num_list[8] = (int32_t)((double)n1 + gc_rate_g*4);
	gc_num_list[9] = (int32_t)((double)n1 + gc_rate_g*5);	
	gc_num_list[10] = (int32_t)((double)n1 + gc_rate_g*6);		
	gc_num_list[11] = (int32_t)((double)n1 + gc_rate_g *7);
	gc_num_list[12] = (int32_t)((double)n1 + gc_rate_g *8);
	gc_num_list[13] = (int32_t)((double)n1 + gc_rate_g *9);
	gc_num_list[14] = (int32_t)((double)n1 + gc_rate_g *10);	
	gc_num_list[15] = (int32_t)((double)gc_num_list[14] + gc_rate_g*0.5);
	gc_num_list[16] = (int32_t)((double)gc_num_list[14] + gc_rate_g *1.5);
	gc_num_list[17] = (int32_t)((double)gc_num_list[14] + gc_rate_g *2);
	gc_num_list[18] = (int32_t)((double)gc_num_list[14] + gc_rate_g *2.5);
	gc_num_list[19] = (int32_t)((double)gc_num_list[14] + gc_rate_g *3);
	gc_num_list[20] = (int32_t)((double)gc_num_list[14] + gc_rate_g *3.5);
	gc_num_list[21] = (int32_t)((double)gc_num_list[14] + gc_rate_g *4);
	gc_num_list[22] = (int32_t)((double)gc_num_list[14] + gc_rate_g *4.5);	
	gc_num_list[23] = (int32_t)((double)gc_num_list[14] + gc_rate_g *5);	
	gc_num_list[24] = (int32_t)((double)gc_num_list[14] + (gc_rate_g * 5.5));
	gc_num_list[25] = (int32_t)((double)gc_num_list[24] + (gc_rate_b * 1));
	gc_num_list[26] = (int32_t)((double)gc_num_list[24] + (gc_rate_b * 2));
	gc_num_list[27] = (int32_t)((double)gc_num_list[24] +  (gc_rate_b * 3));
	gc_num_list[28] = (int32_t)((double)gc_num_list[27] + (gc_rate_l * 1));
	gc_num_list[29] = (int32_t)((double)gc_num_list[27] + (gc_rate_l * 2));
	gc_num_list[30] = (int32_t)((double)gc_num_list[27] + (gc_rate_l * 3));

//	NRF_LOG_INFO("gc -3-2-1 %03d %03d %03d",gc_num_list[0],gc_num_list[1],gc_num_list[2]);
//	NRF_LOG_INFO("gc  0 1 2 %03d %03d %03d",gc_num_list[3],gc_num_list[4],gc_num_list[5]);
//	NRF_LOG_INFO("gc      3 %03d",gc_num_list[6]);
//	NRF_LOG_INFO("gc  4 5 6 %03d %03d %03d",gc_num_list[7],gc_num_list[8],gc_num_list[9]);
//	NRF_LOG_INFO("gc  7 8 9 %03d %03d %03d",gc_num_list[10],gc_num_list[11],gc_num_list[12]);
//	NRF_LOG_INFO("gc 101112 %03d %03d %03d",gc_num_list[13],gc_num_list[14],gc_num_list[15]);
//	NRF_LOG_INFO("gc 131415 %03d %03d %03d",gc_num_list[16],gc_num_list[17],gc_num_list[18]);
//	NRF_LOG_INFO("gc 161718 %03d %03d %03d",gc_num_list[19],gc_num_list[20],gc_num_list[21]);
//	NRF_LOG_INFO("gc 192021 %03d %03d %03d",gc_num_list[22],gc_num_list[23],gc_num_list[24]);
//	NRF_LOG_INFO("gc 222324 %03d %03d %03d",gc_num_list[25],gc_num_list[26],gc_num_list[27]);
//	NRF_LOG_INFO("gc 252627 %03d %03d %03d",gc_num_list[28],gc_num_list[29],gc_num_list[30]);

}
//计算 RMSb
int32_t gc_rms_progress(int16_t *base, int16_t num)
{
	double sum = 0;
	double sum2 = 0.0;
	uint16_t i = 0;
	for(i=8;i<(num-3);i++) {
		sum += (double)(base[i]*base[i]);
	}
	sum2 = (double)sum / (double)(num-10);
	sum2 = sqrt(sum2);
	return ((int32_t)sum2);
}


uint16_t cal_foc_curr(uint16_t mv)//foc电压转换为电流
{
	return mv;
}
