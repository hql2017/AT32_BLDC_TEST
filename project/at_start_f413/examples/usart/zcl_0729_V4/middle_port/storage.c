#include "main.h"


union Para_Union pa_un; 
union Pattern_Union pat_un; 
union Motor_Para_Union mt_un; 

union Para_Union default_pa_un; 
union Pattern_Union default_pat_un; 
union Motor_Para_Union default_mt_un; 

void default_para_write_buff(void)
{
  pa_un.device_para.W_flag=20721;
  pa_un.device_para.err_num=0;
  pa_un.device_para.sys_num=0;  
  pa_un.device_para.empty_rate=1000;  
  pa_un.device_para.use_hand=right_hand;  
  pa_un.device_para.use_num=0;  
  pa_un.device_para.vol=high_vol;  
  pa_un.device_para.apical_action_flag=1;
  pa_un.device_para.auto_start_flag=0;  
  pa_un.device_para.motor_max_current=2800;
  pa_un.device_para.auto_stop_flag=0;
  pa_un.device_para.ref_rate=700;  
  pa_un.device_para.ref_tine=3;
  
  pat_un.Pattern[0].work_mode=only_motor;
  pat_un.Pattern[0].speed_num=4;
  pat_un.Pattern[0].torque_num=4;
  pat_un.Pattern[0].work_dir=foreward;  
  pat_un.Pattern[0].fv=270;
  pat_un.Pattern[0].rv=30;

  pat_un.Pattern[1].work_mode=only_motor;
  pat_un.Pattern[1].speed_num=4;
  pat_un.Pattern[1].torque_num=4;
  pat_un.Pattern[1].work_dir=foreward;  
  pat_un.Pattern[1].fv=270;
  pat_un.Pattern[1].rv=30;
  
  pat_un.Pattern[2].work_mode=only_motor;
  pat_un.Pattern[2].speed_num=4;
  pat_un.Pattern[2].torque_num=4;
  pat_un.Pattern[2].work_dir=foreward;  
  pat_un.Pattern[2].fv=270;
  pat_un.Pattern[2].rv=30;
  
  pat_un.Pattern[3].work_mode=only_motor;
  pat_un.Pattern[3].speed_num=4;
  pat_un.Pattern[3].torque_num=4;
  pat_un.Pattern[3].work_dir=foreward;  
  pat_un.Pattern[3].fv=270;
  pat_un.Pattern[3].rv=30;
  
  pat_un.Pattern[4].work_mode=only_motor;
  pat_un.Pattern[4].speed_num=4;
  pat_un.Pattern[4].torque_num=4;
  pat_un.Pattern[4].work_dir=foreward;  
  pat_un.Pattern[4].fv=270;
  pat_un.Pattern[4].rv=30;
  
  pat_un.Pattern[5].work_mode=only_motor;
  pat_un.Pattern[5].speed_num=4;
  pat_un.Pattern[5].torque_num=4;
  pat_un.Pattern[5].work_dir=foreward;  
  pat_un.Pattern[5].fv=270;
  pat_un.Pattern[5].rv=30;
  
  pat_un.Pattern[6].work_mode=only_motor;
  pat_un.Pattern[6].speed_num=4;
  pat_un.Pattern[6].torque_num=4;
  pat_un.Pattern[6].work_dir=foreward;  
  pat_un.Pattern[6].fv=270;
  pat_un.Pattern[6].rv=30;
  
  pat_un.Pattern[7].work_mode=only_motor;
  pat_un.Pattern[7].speed_num=4;
  pat_un.Pattern[7].torque_num=4;
  pat_un.Pattern[7].work_dir=foreward;  
  pat_un.Pattern[7].fv=270;
  pat_un.Pattern[7].rv=30;
  
  pat_un.Pattern[8].work_mode=only_motor;
  pat_un.Pattern[8].speed_num=4;
  pat_un.Pattern[8].torque_num=4;
  pat_un.Pattern[8].work_dir=foreward;  
  pat_un.Pattern[8].fv=270;
  pat_un.Pattern[8].rv=30;
  
  pat_un.Pattern[9].work_mode=only_motor;
  pat_un.Pattern[9].speed_num=4;
  pat_un.Pattern[9].torque_num=4;
  pat_un.Pattern[9].work_dir=foreward;  
  pat_un.Pattern[9].fv=270;
  pat_un.Pattern[9].rv=30;
//100,150,200,250,300,350,400,450,500,550,600,800,1000,1200,1500,1800,2000,2200,2500

}

void para_print(void)
{
  printf("*******sys_para******\r\n");
  printf("W_flag %d , err_num %d , sys_num %d ,\r\n empty_rate %d , use_hand %d , use_num %d ,\r\n vol %d , apical_action_flag %d , auto_start_flag %d ,\r\n mt_i %d , auto_stop_flag %d , ref_rate %d , ref_tine %d\r\n", 
  pa_un.device_para.W_flag,
  pa_un.device_para.err_num,
  pa_un.device_para.sys_num,
  pa_un.device_para.empty_rate,  
  pa_un.device_para.use_hand,  
  pa_un.device_para.use_num, 
  pa_un.device_para.vol,  
  pa_un.device_para.apical_action_flag,
  pa_un.device_para.auto_start_flag,
  pa_un.device_para.motor_max_current,
  pa_un.device_para.auto_stop_flag,
  pa_un.device_para.ref_rate, 
  pa_un.device_para.ref_tine);
  printf("************************\r\n");
   
  printf("*******patttern_para******\r\n");
  printf("PAT0 work_mode %d speed_num %d torque_num %d work_dir %d fv %d rv %d\r\n ", 
  pat_un.Pattern[0].work_mode,
  pat_un.Pattern[0].speed_num,
  pat_un.Pattern[0].torque_num,
  pat_un.Pattern[0].work_dir,
  pat_un.Pattern[0].fv,
  pat_un.Pattern[0].rv);
  printf("PAT1 work_mode %d speed_num %d torque_num %d work_dir %d fv %d rv %d\r\n ", 
  pat_un.Pattern[1].work_mode,
  pat_un.Pattern[1].speed_num,
  pat_un.Pattern[1].torque_num,
  pat_un.Pattern[1].work_dir,
  pat_un.Pattern[1].fv,
  pat_un.Pattern[1].rv);
  printf("PAT2 work_mode %d speed_num %d torque_num %d work_dir %d fv %d rv %d\r\n ", 
  pat_un.Pattern[2].work_mode,
  pat_un.Pattern[2].speed_num,
  pat_un.Pattern[2].torque_num,
  pat_un.Pattern[2].work_dir,
  pat_un.Pattern[2].fv,
  pat_un.Pattern[2].rv);
  printf("PAT3 work_mode %d speed_num %d torque_num %d work_dir %d fv %d rv %d\r\n ", 
  pat_un.Pattern[3].work_mode,
  pat_un.Pattern[3].speed_num,
  pat_un.Pattern[3].torque_num,
  pat_un.Pattern[3].work_dir,
  pat_un.Pattern[3].fv,
  pat_un.Pattern[3].rv);
  printf("PAT4 work_mode %d speed_num %d torque_num %d work_dir %d fv %d rv %d\r\n ", 
  pat_un.Pattern[4].work_mode,
  pat_un.Pattern[4].speed_num,
  pat_un.Pattern[4].torque_num,
  pat_un.Pattern[4].work_dir,
  pat_un.Pattern[4].fv,
  pat_un.Pattern[4].rv);
  printf("PAT5 work_mode %d speed_num %d torque_num %d work_dir %d fv %d rv %d\r\n ", 
  pat_un.Pattern[5].work_mode,
  pat_un.Pattern[5].speed_num,
  pat_un.Pattern[5].torque_num,
  pat_un.Pattern[5].work_dir,
  pat_un.Pattern[5].fv,
  pat_un.Pattern[5].rv);
  printf("PAT6 work_mode %d speed_num %d torque_num %d work_dir %d fv %d rv %d\r\n ", 
  pat_un.Pattern[6].work_mode,
  pat_un.Pattern[6].speed_num,
  pat_un.Pattern[6].torque_num,
  pat_un.Pattern[6].work_dir,
  pat_un.Pattern[6].fv,
  pat_un.Pattern[6].rv);
  printf("PAT7 work_mode %d speed_num %d torque_num %d work_dir %d fv %d rv %d\r\n ", 
  pat_un.Pattern[7].work_mode,
  pat_un.Pattern[7].speed_num,
  pat_un.Pattern[7].torque_num,
  pat_un.Pattern[7].work_dir,
  pat_un.Pattern[7].fv,
  pat_un.Pattern[7].rv);
  printf("PAT8 work_mode %d speed_num %d torque_num %d work_dir %d fv %d rv %d\r\n ", 
  pat_un.Pattern[8].work_mode,
  pat_un.Pattern[8].speed_num,
  pat_un.Pattern[8].torque_num,
  pat_un.Pattern[8].work_dir,
  pat_un.Pattern[8].fv,
  pat_un.Pattern[8].rv);
  printf("PAT9 work_mode %d speed_num %d torque_num %d work_dir %d fv %d rv %d\r\n ", 
  pat_un.Pattern[9].work_mode,
  pat_un.Pattern[9].speed_num,
  pat_un.Pattern[9].torque_num,
  pat_un.Pattern[9].work_dir,
  pat_un.Pattern[9].fv,
  pat_un.Pattern[9].rv);
  printf("************************\r\n");
  printf("*******motor_para******\r\n");
  printf("MT0 noload_current %d noload_voltage %d para1 %d para2 %d\r\n", 
  mt_un.motor_para[0].noload_current,
  mt_un.motor_para[0].noload_voltage,
  mt_un.motor_para[0].repara1,
  mt_un.motor_para[0].repara2);
  printf("MT1 noload_current %d noload_voltage %d para1 %d para2 %d\r\n", 
  mt_un.motor_para[1].noload_current,
  mt_un.motor_para[1].noload_voltage,
  mt_un.motor_para[1].repara1,
  mt_un.motor_para[1].repara2);
  printf("MT2 noload_current %d noload_voltage %d para1 %d para2 %d\r\n", 
  mt_un.motor_para[2].noload_current,
  mt_un.motor_para[2].noload_voltage,
  mt_un.motor_para[2].repara1,
  mt_un.motor_para[2].repara2);  
  printf("MT3 noload_current %d noload_voltage %d para1 %d para2 %d\r\n", 
  mt_un.motor_para[3].noload_current,
  mt_un.motor_para[3].noload_voltage,
  mt_un.motor_para[3].repara1,
  mt_un.motor_para[3].repara2);
  printf("MT4 noload_current %d noload_voltage %d para1 %d para2 %d\r\n", 
  mt_un.motor_para[4].noload_current,
  mt_un.motor_para[4].noload_voltage,
  mt_un.motor_para[4].repara1,
  mt_un.motor_para[4].repara2);
  printf("MT5 noload_current %d noload_voltage %d para1 %d para2 %d\r\n", 
  mt_un.motor_para[5].noload_current,
  mt_un.motor_para[5].noload_voltage,
  mt_un.motor_para[5].repara1,
  mt_un.motor_para[5].repara2);
  printf("MT6 noload_current %d noload_voltage %d para1 %d para2 %d\r\n", 
  mt_un.motor_para[6].noload_current,
  mt_un.motor_para[6].noload_voltage,
  mt_un.motor_para[6].repara1,
  mt_un.motor_para[6].repara2);
  printf("MT7 noload_current %d noload_voltage %d para1 %d para2 %d\r\n", 
  mt_un.motor_para[7].noload_current,
  mt_un.motor_para[7].noload_voltage,
  mt_un.motor_para[7].repara1,
  mt_un.motor_para[7].repara2);
  printf("MT8 noload_current %d noload_voltage %d para1 %d para2 %d\r\n", 
  mt_un.motor_para[8].noload_current,
  mt_un.motor_para[8].noload_voltage,
  mt_un.motor_para[8].repara1,
  mt_un.motor_para[8].repara2);
  printf("MT9 noload_current %d noload_voltage %d para1 %d para2 %d\r\n", 
  mt_un.motor_para[9].noload_current,
  mt_un.motor_para[9].noload_voltage,
  mt_un.motor_para[9].repara1,
  mt_un.motor_para[9].repara2);
  printf("MT10 noload_current %d noload_voltage %d para1 %d para2 %d\r\n", 
  mt_un.motor_para[10].noload_current,
  mt_un.motor_para[10].noload_voltage,
  mt_un.motor_para[10].repara1,
  mt_un.motor_para[10].repara2);
  printf("MT11 noload_current %d noload_voltage %d para1 %d para2 %d\r\n", 
  mt_un.motor_para[11].noload_current,
  mt_un.motor_para[11].noload_voltage,
  mt_un.motor_para[11].repara1,
  mt_un.motor_para[11].repara2);
  printf("MT12 noload_current %d noload_voltage %d para1 %d para2 %d\r\n", 
  mt_un.motor_para[12].noload_current,
  mt_un.motor_para[12].noload_voltage,
  mt_un.motor_para[12].repara1,
  mt_un.motor_para[12].repara2);
  printf("MT13 noload_current %d noload_voltage %d para1 %d para2 %d\r\n", 
  mt_un.motor_para[13].noload_current,
  mt_un.motor_para[13].noload_voltage,
  mt_un.motor_para[13].repara1,
  mt_un.motor_para[13].repara2);
  printf("MT14 noload_current %d noload_voltage %d para1 %d para2 %d\r\n", 
  mt_un.motor_para[14].noload_current,
  mt_un.motor_para[14].noload_voltage,
  mt_un.motor_para[14].repara1,
  mt_un.motor_para[14].repara2);
  printf("MT15 noload_current %d noload_voltage %d para1 %d para2 %d\r\n", 
  mt_un.motor_para[15].noload_current,
  mt_un.motor_para[15].noload_voltage,
  mt_un.motor_para[15].repara1,
  mt_un.motor_para[15].repara2);
  printf("MT16 noload_current %d noload_voltage %d para1 %d para2 %d\r\n", 
  mt_un.motor_para[16].noload_current,
  mt_un.motor_para[16].noload_voltage,
  mt_un.motor_para[16].repara1,
  mt_un.motor_para[16].repara2);
  printf("MT17 noload_current %d noload_voltage %d para1 %d para2 %d\r\n", 
  mt_un.motor_para[17].noload_current,
  mt_un.motor_para[17].noload_voltage,
  mt_un.motor_para[17].repara1,
  mt_un.motor_para[17].repara2);
  printf("MT18 noload_current %d noload_voltage %d para1 %d para2 %d\r\n", 
  mt_un.motor_para[18].noload_current,
  mt_un.motor_para[18].noload_voltage,
  mt_un.motor_para[18].repara1,
  mt_un.motor_para[18].repara2);
  printf("************************\r\n");
}
void start_para_write_read(void)
{
  flash_read(PARA_ADDRESS_BASE, pa_un.para_buff, 13);
  printf("first read w_flag is %d \r\n",pa_un.device_para.W_flag);
  if(pa_un.device_para.W_flag==20721)
  {
    printf("NO DO WRITE\r\n");
    flash_read(PATTERN_ADDRESS_BASE,pat_un.pattern_buff,60); 
    flash_read(MOTOR_PARA_ADDRESS_BASE,mt_un.motor_para_buff,76);
  }
  else
  {   
   default_para_write_buff();
   flash_write(PARA_ADDRESS_BASE, pa_un.para_buff, 13);
   flash_write(PATTERN_ADDRESS_BASE,pat_un.pattern_buff,60); 
   flash_write(MOTOR_PARA_ADDRESS_BASE,mt_un.motor_para_buff,76);
   printf("DO ONE WRITE\r\n"); 
   flash_read(PARA_ADDRESS_BASE, pa_un.para_buff, 13);
   flash_read(PATTERN_ADDRESS_BASE,pat_un.pattern_buff,60); 
   flash_read(MOTOR_PARA_ADDRESS_BASE,mt_un.motor_para_buff,76);   
   printf("DO ONE READ\r\n");     
  }
   para_copy_default();
   para_print();
}

void para_copy_default(void)//将开机后使用参数拷贝到默认参数为下次存储参数做比较
{
  uint16_t i=0;
  for(i=0;i<13;i++)
  {
    default_pa_un.para_buff[i]=pa_un.para_buff[i];
  }
  for(i=0;i<60;i++)
  {
    default_pat_un.pattern_buff[i]=pat_un.pattern_buff[i];
  }
  for(i=0;i<76;i++)
  {
    default_mt_un.motor_para_buff[i]=mt_un.motor_para_buff[i];
  }
}

void write_para_judge(void)
{
  error_status err;
  err=buffer_compare(default_pa_un.para_buff,pa_un.para_buff,13);
  if(ERROR==err) //参数已经被篡改
  {
     flash_write(PARA_ADDRESS_BASE, pa_un.para_buff, 13);
  }
  else
  {
    printf("PARA NO CHANGE\r\n");
  }
  err=buffer_compare(default_pat_un.pattern_buff,pat_un.pattern_buff,60);
  if(ERROR==err) //参数已经被篡改
  {
   flash_write(PATTERN_ADDRESS_BASE,pat_un.pattern_buff,60); 
  }
  else
  {
    printf("PATTERN NO CHANGE\r\n");
  }
  err=buffer_compare(default_mt_un.motor_para_buff,mt_un.motor_para_buff,76);
  if(ERROR==err) //参数已经被篡改
  {
   flash_write(MOTOR_PARA_ADDRESS_BASE,mt_un.motor_para_buff,76);
  }
  else
  {
    printf("MOTOR PARA NO CHANGE\r\n");
  }
}


error_status buffer_compare(uint16_t* p_buffer1, uint16_t* p_buffer2, uint16_t buffer_length)
{
  while(buffer_length--)
  {
    if(*p_buffer1 != *p_buffer2)
    {
      return ERROR;
    }
    p_buffer1++;
    p_buffer2++;
  }
  return SUCCESS;
}

