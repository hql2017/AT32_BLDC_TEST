#include "main.h"
#include "oled_port.h"
#include "gpio_port.h"
#include "customer_control.h"
#include "macros.h"
#include "delay.h"

#include "key.h"

#ifndef RTOS_UNUSED
	#include "FreeRTOS.h"
	#include "task.h"
	#include "queue.h"
	#include "semphr.h"

	#define KEY_SCAN_STACK_DEPTH  64
	#define KEY_SCAN_TASK_PRIORITY 	5
	TaskHandle_t  keyScanTask_Handle;
	
	#define APP_OLED_STACK_DEPTH 320
	#define APP_OLED_TASK_PRIORITY 	5
	TaskHandle_t  appOledTask_Handle;
	
	#define DEVICE_CONTROL_STACK_DEPTH 180
	#define DEVICE_CONTROL_TASK_PRIORITY 	5
	TaskHandle_t  deviceControlTask_Handle;
	
	#define PERIODIC_STACK_DEPTH 128
	#define PERIODIC_TASK_PRIORITY 	4
	TaskHandle_t  periodicTask_Handle; //周期任务	
	
	#define COMMUNICATION_STACK_DEPTH 128
	#define COMMUNICATION_TASK_PRIORITY 	4
	TaskHandle_t  communicationTask_Handle; //周期任务,heartbeat	
	
	QueueHandle_t  xQueueKeyMessage;//key
	QueueHandle_t  xQueueBeepMode;//beep

	#else	
		PT_THREAD(task_menu(struct pt *pt));//菜单进程处理函数
		PT_THREAD(task_button(struct pt *pt));
		PT_THREAD(task_buzz(struct pt *pt));
		PT_THREAD(task_led(struct pt *pt));
		PT_THREAD(task_idle(struct pt *pt));
		/*
		PT_THREAD(task_state(struct pt *pt));
		PT_THREAD(task_adc(struct pt *pt));//ADC进程处理函数
		PT_THREAD(task_gc(struct pt *pt));
		PT_THREAD(task_uart(struct pt *pt));
		PT_THREAD(task_fast_adc(struct pt *pt));
		PT_THREAD(task_motor(struct pt *pt));
		struct pt pt_uart;
		struct pt pt_motor;
		struct pt pt_state;
		struct pt pt_adc;
		struct pt pt_gc;
		struct pt pt_fast_adc;
		*/
		struct pt pt_button;
		struct pt pt_led;
		struct pt pt_menu;
		struct pt pt_buzz;
		struct pt pt_idle;
#endif


uint8_t foc_flag=0;
uint8_t buzz_button_flag=0;//按键鸣叫标志  0不叫 1鸣叫 由buzz线程自我约束
uint8_t buzz_gc_flag=0;    //根测鸣叫标志  0不叫 1普通叫 2急促叫 即将到达根尖 3最急催已达根尖 由gc线程约束
uint8_t buzz_torque_flag=0;//力矩超限鸣叫标志 0不叫 1普通叫达到设定力矩 0.8处 2急促叫超出设定力矩 由电机线程约束
uint8_t buzz_power_flag=0;// 低电量鸣叫标志   0不叫 1普通叫 由buzz线程自我约束
uint8_t buzz_charge_flag=0;//充电插拔标志     0不叫 1普通叫 有buzz线程自我约束
uint8_t insert_flag=0;//插入标志位 0未插入 1已插入 
uint8_t work_flag=0; //工作标志 0 停止工作 1 开始工作
uint8_t charge_flag=0;//充电标志位 0已充满 1正充电
uint8_t sys_volatge_refresh=0;//系统电压更新标志
uint8_t motor_current_refresh=0;//电机电流更新标志
uint8_t vbus_volt_refresh=0;//VBUS电压更新标志
uint8_t blink_500MS=0;
uint8_t gc_insert_flag=0;//根测线插入标志 0未插入 1已插入 

uint16_t battery_status=2;//电池电量剩余 0 1 2 3 4

uint16_t real_time_torque=0;//实时扭矩  Ncm*10
uint16_t vbus_voltage_mv;//VBUS电压
uint16_t device_voltage_mv=0;//系统电压
uint16_t motor_current_ma=0; //电机电流
uint16_t foc_current_ma[3]={0}; //foc电机电流
uint16_t ref_set_flag=0;//单根测设置标志
uint16_t mt_gc_set_flag=0;//马达和二合一设置标志 0 1 2 3
uint16_t setting_for_flag=0;//设置选择标志 0 motor 1 apex
uint16_t motor_calc_flag=0;//自动校准选择标志
uint16_t calc_motor_count=20;//自动校准倒计时
uint16_t default_set_flag=0;//恢复出厂设置标志
uint16_t default_set_count=20;//恢复出厂设置倒计时

uint8_t motor_run_flag=0;

uint8_t menu_motor_run_mode=1; //电机运行模式,0，ATC模式，1，正向，,2逆向，3正反转（角度调节）

#ifndef ZHX
#define MAX_MOTOR_SPEED   2450//中韩星(max 2450*6=14700)
#define MAX_TORQUE_UUPER_THRESHOLD   4.2//中韩星(max 4.2N.cm)

#else 
#define MAX_MOTOR_SPEED    3000//moons
#define MAX_TORQUE_UUPER_THRESHOLD   5.0//moons(max 5.0N.cm)？
#endif

/*==================================key task===================================
			* @brief:  key task.
			* @param:  None
			* @retval: None
==============================================================================*/
void vAppKeyTask( void * pvParameters )
{ 	
	static uint8_t countTimeMs=10;//scan
	static uint8_t button_Event[MAX_BUTTON_NUM]={BUTTON_EVENT_NONE};
	static uint8_t keyMessage=0;
	uint8_t i;
	for(;;)
	{	
		vTaskDelay(2);//	10ms	
    ButtonScan(countTimeMs);
		for(i=0;i<MAX_BUTTON_NUM;i++)//get key value
		{
			button_Event[RUN_BOTTON] = GetButtonEvent(RUN_BOTTON);	
		}
		if(button_Event[RUN_BOTTON]==BUTTON_EVENT_CLICK)		
		{
			//handle  or send to key message queue
			keyMessage=run_button_press_signal;
			button_Event[RUN_BOTTON]=BUTTON_EVENT_NONE;
		}
		//handle  or send to key message queue
	}
}
/*==================================initiating task===================================
			* @brief:  initiating task.
			* @param:  None
			* @retval: None
====================================================================================*/
void vTaskStart( void * pvParameters )
{
		
		taskENTER_CRITICAL();   
	 //queue
		xQueueKeyMessage=xQueueCreate(2,sizeof(uint16_t));	
		xQueueBeepMode=	xQueueCreate(2,sizeof(uint8_t));
	//Semaphore	
	
		xTaskCreate( vAppKeyTask, "key scan", KEY_SCAN_STACK_DEPTH, NULL, KEY_SCAN_TASK_PRIORITY, &keyScanTask_Handle );			
		vTaskDelete( NULL);		
		taskEXIT_CRITICAL();  //退出临界区			
}

#ifdef DRIVE_BOARD


//PT_THREAD(task_uart(struct pt *pt))//uart通信进程处理函数
//{
//	static uint32_t local_time_flag=0;
//  static uint8_t len=0;
//  static uint8_t buf[64];
//  PT_BEGIN(pt); 
//  local_time_flag = sys_time_flag;
//	PT_WAIT_UNTIL(pt, ((sys_time_flag-local_time_flag) >= 400));
//	while(1) 
//          {
//            local_time_flag = sys_time_flag;   
//            PT_WAIT_UNTIL(pt, ((sys_time_flag-local_time_flag) >=10));//延迟到10ms 
//            if(uart_rx_flag != 0) //包括接收数据长度
//            {
//              len = uart_rx_flag;
//              uart_rx_flag = 0;
//              //printf(uart_rx_buf,len);
//              memcpy(buf,uart_rx_buf,len);
//              send_ack();
////              cmd_parse(buf);
//            }
//            PT_YIELD(pt);
//          }
//	PT_END(pt);
//}

#endif

PT_THREAD(task_menu(struct pt *pt))//菜单进程处理函数
{
	static uint32_t menu_idle_times =0;//空闲计数
	static uint32_t local_time_flag=0;//启动菜单 设置声音 设置语言 设置程序号 设置参考点
  static SCREEN_ENUM menu_flag_a=MAIN; //0 main 1 setting_for  2 cw_angle 3 ccw_angle 4 auto_start 5 auto_stop 6 apical_action 7 advanced_sett 8 calibration 
  //9 default_sett 10 work 11 engineer 
	static uint8_t sub_menu_motor_param=0; //电机参数,0:，速度，1扭矩,2，方向，3正转角度，4反转角度
  uint32_t rec_signal=0;
  static QUEUE_StatusTypeDef err_que=0;
	PT_BEGIN(pt);
	
	local_time_flag = sys_time_flag;
	PT_WAIT_UNTIL(pt, ((sys_time_flag-local_time_flag)>=1000)); 	
//tmr_interrupt_enable(TMR3, TMR_OVF_INT, TRUE);
  while(1)
  {
		local_time_flag = sys_time_flag;
		PT_WAIT_UNTIL(pt, ((sys_time_flag-local_time_flag)>=50)); 

    err_que=Queue_Pop(&signal_que,&rec_signal);
//    printf("err_que IS %d\r\n",err_que);
    if(0!=err_que)
    {
//    printf("rec_signal IS %d\r\n",rec_signal);
      rec_signal=0;
    }   
		if(menu_flag_a==MAIN)
			{	
				if(rec_signal==run_button_press_signal)
				{	
					if(motor_run_flag==MOTOR_MODE_STOP)		
					{
							//motor start
							motor_run_flag	=menu_motor_run_mode;// 运行态			
							motor_run_status(motor_run_flag);
							if(foc_flag==0){
									foc_flag=1;	
									tmr_counter_enable(TMR3, TRUE);									
							}		
							LedFunctionSet( LED_B ,500,LED_T_MOTOR,LED_KEEP_ON);
					}
					else	{
							motor_run_flag=MOTOR_MODE_STOP;
							motor_run_status(motor_run_flag);
							foc_flag=0;	
							LedFunctionSet( LED_B ,500,LED_T_MOTOR,0);						
					}					
				}	       
        else if(rec_signal==add_button_press_signal)
        {
						if(sub_menu_motor_param==0){//正反转
							menu_motor_run_mode++;//0,1,2,3
							if(menu_motor_run_mode>3)		menu_motor_run_mode=0;
						}	
						else if(sub_menu_motor_param==1){//speed
								if(menu_motor_run_mode==MOTOR_FORWORD||menu_motor_run_mode==MOTOR_TORQUE_ATC){
									motor_settings.forward_speed+=50;
									if(motor_settings.forward_speed>MAX_MOTOR_SPEED) motor_settings.forward_speed=MAX_MOTOR_SPEED;	
								}
								else if(menu_motor_run_mode==MOTOR_REVERSAL){//反向
										motor_settings.reverse_speed-=50;
										if(motor_settings.reverse_speed<-MAX_MOTOR_SPEED)	 motor_settings.reverse_speed=-MAX_MOTOR_SPEED;									
								}
								else if(menu_motor_run_mode==MOTOR_FORWORD_AND_REVERSAL){
									motor_settings.forward_speed+=50;
									if(motor_settings.forward_speed>MAX_MOTOR_SPEED) motor_settings.forward_speed=MAX_MOTOR_SPEED;
									motor_settings.reverse_speed-=50;
									if(motor_settings.reverse_speed<-MAX_MOTOR_SPEED)	 motor_settings.reverse_speed=-MAX_MOTOR_SPEED;
								}
						}
						else if(sub_menu_motor_param==2){//torque
								if(menu_motor_run_mode==MOTOR_FORWORD_AND_REVERSAL){
									motor_settings.upper_threshold=MAX_TORQUE_UUPER_THRESHOLD;
									motor_settings.lower_threshold=motor_settings.upper_threshold*0.6;
								}else if(menu_motor_run_mode==MOTOR_TORQUE_ATC) {										
									motor_settings.lower_threshold+= 0.1;									
									if(motor_settings.lower_threshold>2.0) { motor_settings.lower_threshold=2.0;motor_settings.upper_threshold=MAX_TORQUE_UUPER_THRESHOLD;}								
								}
								else{
									motor_settings.upper_threshold+= 0.1;									
									if(motor_settings.upper_threshold>MAX_TORQUE_UUPER_THRESHOLD)  motor_settings.upper_threshold=MAX_TORQUE_UUPER_THRESHOLD;
									motor_settings.lower_threshold=motor_settings.upper_threshold*0.6;
								}										
					 }
						else if(sub_menu_motor_param==3){//正向角度
									motor_settings.forward_position+=5;								
									if(motor_settings.forward_position>170) motor_settings.forward_position=170;									
						}
						else if(sub_menu_motor_param==4){//反向角度							
								motor_settings.reverse_position-=5;
								if(	motor_settings.reverse_position<-50) motor_settings.forward_position=-50;
						}							
        }
        else if(rec_signal==sub_button_press_signal)
        { 
					if(sub_menu_motor_param==0){//电机运行模式
								menu_motor_run_mode++;//0,1,2,3
								if(menu_motor_run_mode>3)		menu_motor_run_mode=0;
						}	
						else if(sub_menu_motor_param==1){//speed
								if(menu_motor_run_mode==MOTOR_FORWORD||menu_motor_run_mode==MOTOR_TORQUE_ATC){
									motor_settings.forward_speed-=50;
									if(motor_settings.forward_speed<50) motor_settings.forward_speed=50;	
								}
								else if(menu_motor_run_mode==MOTOR_REVERSAL){//反向
										motor_settings.reverse_speed+=50;
										if(motor_settings.reverse_speed>-50)	 motor_settings.reverse_speed=-50;									
								}
								else if(menu_motor_run_mode==MOTOR_FORWORD_AND_REVERSAL){
									motor_settings.forward_speed-=50;
									if(motor_settings.forward_speed<50) motor_settings.forward_speed=50;
									motor_settings.reverse_speed+=50;
										if(motor_settings.reverse_speed>-50)	 motor_settings.reverse_speed=-50;
								}
						}
						else if(sub_menu_motor_param==2){//torque
								if(menu_motor_run_mode==MOTOR_FORWORD_AND_REVERSAL){
										motor_settings.upper_threshold=MAX_TORQUE_UUPER_THRESHOLD;
										motor_settings.lower_threshold=motor_settings.upper_threshold*0.6;
									}else if(menu_motor_run_mode==MOTOR_TORQUE_ATC) {										
										motor_settings.lower_threshold-= 0.1;									
										if(motor_settings.lower_threshold<0.5) { motor_settings.lower_threshold=0.5;motor_settings.upper_threshold=MAX_TORQUE_UUPER_THRESHOLD;}								
									}
									else{
										motor_settings.upper_threshold-= 0.1;									
										if(motor_settings.upper_threshold<0.5)  motor_settings.upper_threshold=0.5;
										motor_settings.lower_threshold=motor_settings.upper_threshold*0.6;
									}
					 }
						else if(sub_menu_motor_param==3){//正向角度
								motor_settings.forward_position-=5;								
								if(motor_settings.forward_position<30) motor_settings.forward_position=30;									
						}
						else if(sub_menu_motor_param==4){//反向角度							
								motor_settings.reverse_position+=5;
								if(	motor_settings.reverse_position>-30) motor_settings.forward_position=-30;
						}							
        }
				else if(rec_signal==s_button_press_signal){
					sub_menu_motor_param++;						
					if(menu_motor_run_mode==1||menu_motor_run_mode==2) {//正反转
						if(sub_menu_motor_param>2) sub_menu_motor_param=0;//0~2
					}
					else{	
						if(sub_menu_motor_param>4)  sub_menu_motor_param=0;//0~2 ,3,4             						
					}	
				}
				if(rec_signal!=null_signal){
						rec_signal=0;
						update_settings(&motor_settings);
				}
				OLED_Display_MIAN(menu_motor_run_mode,sub_menu_motor_param);
      }
/*************************************************************************************************************************/       
      else if(menu_flag_a==CALIBRATION_TWO)
      {          
      }
/*************************************************************************************************************************/       
      else if(menu_flag_a==DEFAULT_SETT)
      {
        if(default_set_count==0)
        {            
          device_power_on();
          //this start restore default setting
        }
      }
/*************************************************************************************************************************/       
      else if(menu_flag_a==WORK)
      {         
					if(rec_signal==run_button_press_signal)
					{
							menu_flag_a=MAIN;
							rec_signal=0; 
					}
      }      
/*************************************************************************************************************************/  
				if(rec_signal!=null_signal){
					rec_signal=0;					
				}
			PT_YIELD(pt);
  }
	PT_END(pt);
}

PT_THREAD(task_button(struct pt *pt))
{
static uint32_t local_time_flag=0;
  static task_notify_enum send_signal=null_signal;
  PT_BEGIN(pt);
  local_time_flag = sys_time_flag;
	PT_WAIT_UNTIL(pt, ((sys_time_flag-local_time_flag) >= 800));
  while(1)
		{
//			local_time_flag = sys_time_flag;
//			PT_WAIT_UNTIL(pt, ((sys_time_flag-local_time_flag) >= 10));
//			ButtonScan(10);
			if(0== get_run_button_state())
			{
					local_time_flag = sys_time_flag;
					PT_WAIT_UNTIL(pt, ((sys_time_flag-local_time_flag) >= 5));
					if(0== get_run_button_state())
					{						
						buzz_button_flag=1;                   
						local_time_flag = sys_time_flag;
						PT_WAIT_UNTIL(pt, ((sys_time_flag-local_time_flag) >= 100));
						if(0== get_run_button_state())
						{
							local_time_flag = sys_time_flag;
							PT_WAIT_UNTIL(pt, ((sys_time_flag-local_time_flag) >= 150));
							if(0== get_run_button_state())
							{
									local_time_flag = sys_time_flag;
									PT_WAIT_UNTIL(pt, ((sys_time_flag-local_time_flag) >= 300));
									if(0== get_run_button_state())
									{           
										buzz_button_flag=1; 
										send_signal=run_button_long_press_signal;                        
//                  printf("run_button_long_press_signal\r\n");
										Queue_Push(&signal_que,send_signal);									
//									printf("send signal is %d\r\n",send_signal);
										send_signal=null_signal;
										local_time_flag = sys_time_flag;
										PT_WAIT_UNTIL(pt, ((sys_time_flag-local_time_flag) >= 500));
									}
									else
									{ 
										send_signal=run_button_press_signal;                          
//                  printf("run_button_press_signal\r\n");
									}
							}
							else
							{
										send_signal=run_button_press_signal;                                            
//                   printf("run_button_press_signal\r\n");
							}
						}
						else
						{     
										send_signal=run_button_press_signal;                    
//                  printf("run_button_press_signal\r\n");
						}
					}
					else
					{     
							send_signal=run_button_press_signal;                  
//            printf("run_button_press_signal\r\n");
					}
			}
			
			if(0== get_add_button_state())
			{
					local_time_flag = sys_time_flag;
					PT_WAIT_UNTIL(pt, ((sys_time_flag-local_time_flag) >= 5));
					if(0== get_add_button_state())
					{                  
						buzz_button_flag=1; 
						local_time_flag = sys_time_flag;
						PT_WAIT_UNTIL(pt, ((sys_time_flag-local_time_flag) >= 100));
						if(0== get_add_button_state())
						{
							local_time_flag = sys_time_flag;
							PT_WAIT_UNTIL(pt, ((sys_time_flag-local_time_flag) >= 150));
							if(0== get_add_button_state())
							{
									local_time_flag = sys_time_flag;
									PT_WAIT_UNTIL(pt, ((sys_time_flag-local_time_flag) >= 300));
									if(0== get_add_button_state())
									{          
										buzz_button_flag=1; 
										send_signal=add_button_long_press_signal;                           
//                          printf("add_button_long_press_signal\r\n");
										local_time_flag = sys_time_flag;
										PT_WAIT_UNTIL(pt, ((sys_time_flag-local_time_flag) >= 500));
									}
									else
									{   
										send_signal=add_button_press_signal;                                                     
//                          printf("add_button_press_signal\r\n");
									}
							}
							else
							{                        
									 send_signal=add_button_press_signal;
//                          printf("add_button_press_signal\r\n");
							}
						}
						else
						{  
							send_signal=add_button_press_signal;                    
//                    printf("add_button_press_signal\r\n");
						}
					}
					else
					{     
							send_signal=add_button_press_signal;                  
//                    printf("add_button_press_signal\r\n");
					}
			}
			if(0== get_sub_button_state())
			{
					local_time_flag = sys_time_flag;
					PT_WAIT_UNTIL(pt, ((sys_time_flag-local_time_flag) >= 5));
					if(0== get_sub_button_state())
					{                 
						buzz_button_flag=1; 
						local_time_flag = sys_time_flag;
						PT_WAIT_UNTIL(pt, ((sys_time_flag-local_time_flag) >= 100));
						if(0== get_sub_button_state())
						{
							local_time_flag = sys_time_flag;
							PT_WAIT_UNTIL(pt, ((sys_time_flag-local_time_flag) >= 150));
							if(0== get_sub_button_state())
							{
									local_time_flag = sys_time_flag;
									PT_WAIT_UNTIL(pt, ((sys_time_flag-local_time_flag) >= 300));
									if(0== get_sub_button_state())
									{
//buzz                    
										buzz_button_flag=1;                           
										local_time_flag = sys_time_flag;
										PT_WAIT_UNTIL(pt, ((sys_time_flag-local_time_flag) >= 500));
										if(0== get_sub_button_state())
										{
											local_time_flag = sys_time_flag;
											PT_WAIT_UNTIL(pt, ((sys_time_flag-local_time_flag) >= 500));
											if(0== get_sub_button_state())
											{
												local_time_flag = sys_time_flag;
												PT_WAIT_UNTIL(pt, ((sys_time_flag-local_time_flag) >= 500));												
													send_signal=sub_button_long_press_signal;                                 
//                        printf("sub_button_long_press_signal\r\n");  
													local_time_flag = sys_time_flag;
													PT_WAIT_UNTIL(pt, ((sys_time_flag-local_time_flag) >= 500)); 
											}
											else
											{     
												send_signal=sub_button_long_press_signal;                               
//                      printf("sub_button_long_press_signal\r\n");
												local_time_flag = sys_time_flag;
												PT_WAIT_UNTIL(pt, ((sys_time_flag-local_time_flag) >= 500));
											}
										}
										else
										{       
											send_signal=sub_button_long_press_signal;                             
//                    printf("sub_button_long_press_signal\r\n");
											local_time_flag = sys_time_flag;
											PT_WAIT_UNTIL(pt, ((sys_time_flag-local_time_flag) >= 500));
										}
									}
									else
									{        
										send_signal=sub_button_press_signal;                           
//                    printf("sub_button_press_signal\r\n");
									}
							}
							else
							{
								 send_signal=sub_button_press_signal;  
//                    printf("sub_button_press_signal\r\n");
							}
						}
						else
						{        
							send_signal=sub_button_press_signal;                      
//                    printf("sub_button_press_signal\r\n");
						}
					}
					else
					{
							send_signal=sub_button_press_signal;  
//                    printf("sub_button_press_signal\r\n");
					}                
			}
			if(0== get_s_button_state())
			{
					 local_time_flag = sys_time_flag;
					PT_WAIT_UNTIL(pt, ((sys_time_flag-local_time_flag) >= 5));
					if(0== get_s_button_state())
					{                  
						buzz_button_flag=1; 
						local_time_flag = sys_time_flag;
						PT_WAIT_UNTIL(pt, ((sys_time_flag-local_time_flag) >= 100));
						if(0== get_s_button_state())
						{
							local_time_flag = sys_time_flag;
							PT_WAIT_UNTIL(pt, ((sys_time_flag-local_time_flag) >= 300));
							if(0== get_s_button_state())
							{
									local_time_flag = sys_time_flag;
									PT_WAIT_UNTIL(pt, ((sys_time_flag-local_time_flag) >= 300));
									if(0== get_s_button_state())
									{          
										buzz_button_flag=1; 
										send_signal=s_button_long_press_signal;                           
//                  printf("s_button_press_signal\r\n");
										local_time_flag = sys_time_flag;
										PT_WAIT_UNTIL(pt, ((sys_time_flag-local_time_flag) >= 500));
									}
									else
									{   
//										send_signal=s_button_press_signal;    //long                                                 
//                    printf("s_button_press_signal\r\n");
									}
							}
							else
							{                        
									 send_signal=s_button_press_signal;
//                 printf("s_button_press_signal\r\n");
							}
						}
						else
						{  
									send_signal=s_button_press_signal;                    
//                printf("s_button_press_signal\r\n");
						}
					}
					else
					{     
									send_signal=s_button_press_signal;                  
//                printf("s_button_press_signal\r\n");
					}                     
			}
			if(0!=send_signal)
			{
//			printf("send signal is %d\r\n",send_signal);
				Queue_Push(&signal_que,send_signal);
				send_signal=null_signal;
			}
			PT_YIELD(pt);
		}
	PT_END(pt);
}
PT_THREAD(task_buzz(struct pt *pt))
{
  static uint32_t local_time_flag=0;
  static uint16_t pr_value=350;
  static uint16_t low_power_sing_cout=0;
	PT_BEGIN(pt);
  buzz_gc_flag=0;
  buzz_torque_flag=0;
  buzz_power_flag=0;
	local_time_flag = sys_time_flag;
	PT_WAIT_UNTIL(pt, ((sys_time_flag-local_time_flag) >= 800));
	while(1) 
          {
            if(((1==buzz_button_flag)||(buzz_charge_flag==1))&&(buzz_power_flag==0)&&(buzz_torque_flag==0)&&(buzz_gc_flag==0))
            {
              buzz_button_flag=0;
              buzz_charge_flag=0;
              if(pr_value!=249){pr_value=249;tmr_period_value_set(TMR10, pr_value);}
              if(pa_un.device_para.vol==no_vol)
              {
                //do nothing
              }
              else if(pa_un.device_para.vol==low_vol)
              {
                  tmr_channel_value_set(TMR10, TMR_SELECT_CHANNEL_1,5);
                	local_time_flag = sys_time_flag;
	                PT_WAIT_UNTIL(pt, ((sys_time_flag-local_time_flag) >= 150));
                  tmr_channel_value_set(TMR10, TMR_SELECT_CHANNEL_1,0);
                
              }
              else if(pa_un.device_para.vol==middle_vol)
              {
                  tmr_channel_value_set(TMR10, TMR_SELECT_CHANNEL_1,10);
                  local_time_flag = sys_time_flag;
	                PT_WAIT_UNTIL(pt, ((sys_time_flag-local_time_flag) >= 150));
                  tmr_channel_value_set(TMR10, TMR_SELECT_CHANNEL_1,0);
              }
              else if(pa_un.device_para.vol==high_vol)
              {
                  tmr_channel_value_set(TMR10, TMR_SELECT_CHANNEL_1,30);  
                  local_time_flag = sys_time_flag;
	                PT_WAIT_UNTIL(pt, ((sys_time_flag-local_time_flag) >= 150));
                  tmr_channel_value_set(TMR10, TMR_SELECT_CHANNEL_1,0);
       
              }
            }
            if(buzz_gc_flag!=0)
            {
              if(pr_value!=220){pr_value=220;tmr_period_value_set(TMR10, pr_value);}
              if(buzz_gc_flag==1) //普通叫 进入根管一定位置
              {
                  tmr_channel_value_set(TMR10, TMR_SELECT_CHANNEL_1,40);  
                  local_time_flag = sys_time_flag;
	                PT_WAIT_UNTIL(pt, ((sys_time_flag-local_time_flag) >= 80));
                  tmr_channel_value_set(TMR10, TMR_SELECT_CHANNEL_1,0);
                  local_time_flag = sys_time_flag;
                	PT_WAIT_UNTIL(pt, ((sys_time_flag-local_time_flag) >= 400));
              }
              else if(buzz_gc_flag==2)//急促叫 靠近根尖点
              {
                  tmr_channel_value_set(TMR10, TMR_SELECT_CHANNEL_1,40);  
                  local_time_flag = sys_time_flag;
	                PT_WAIT_UNTIL(pt, ((sys_time_flag-local_time_flag) >= 80));
                  tmr_channel_value_set(TMR10, TMR_SELECT_CHANNEL_1,0);
                  local_time_flag = sys_time_flag;
                	PT_WAIT_UNTIL(pt, ((sys_time_flag-local_time_flag) >= 200));
              }
              else if(buzz_gc_flag==3)//超级急促叫 到达根尖点
              {
                  tmr_channel_value_set(TMR10, TMR_SELECT_CHANNEL_1,40);  
                  local_time_flag = sys_time_flag;
	                PT_WAIT_UNTIL(pt, ((sys_time_flag-local_time_flag) >= 80));
                  tmr_channel_value_set(TMR10, TMR_SELECT_CHANNEL_1,0);
                  local_time_flag = sys_time_flag;
                	PT_WAIT_UNTIL(pt, ((sys_time_flag-local_time_flag) >= 80));
              }           
            }
            if((buzz_torque_flag!=0)&&(buzz_gc_flag==0)) //仅当根测不发声音时 电机才允许发音 buzz_torque_flag为1和2均采用同一套声音
            {
                  if(pr_value!=180){pr_value=180;tmr_period_value_set(TMR10, pr_value);}
                  tmr_channel_value_set(TMR10, TMR_SELECT_CHANNEL_1,60);  
                  local_time_flag = sys_time_flag;
	                PT_WAIT_UNTIL(pt, ((sys_time_flag-local_time_flag) >= 200));
                  tmr_channel_value_set(TMR10, TMR_SELECT_CHANNEL_1,0);
                  local_time_flag = sys_time_flag;
                	PT_WAIT_UNTIL(pt, ((sys_time_flag-local_time_flag) >= 400));
            }
            if((buzz_power_flag==1)&&(buzz_torque_flag==0)&&(buzz_gc_flag==0)) //仅当根测电机不发声音时 低电量才允许发音 
            {
                  low_power_sing_cout=10;
                  buzz_power_flag=0;
                  if(pr_value!=150){pr_value=150;tmr_period_value_set(TMR10, pr_value);}
                  while(low_power_sing_cout)
                  {           
                    tmr_channel_value_set(TMR10, TMR_SELECT_CHANNEL_1,25);  
                    local_time_flag = sys_time_flag;
                    PT_WAIT_UNTIL(pt, ((sys_time_flag-local_time_flag)>= 250));
                    tmr_channel_value_set(TMR10, TMR_SELECT_CHANNEL_1,0);
                    local_time_flag = sys_time_flag;
                    PT_WAIT_UNTIL(pt, ((sys_time_flag-local_time_flag) >= 250));
                    low_power_sing_cout--;
                 }
            }
            PT_YIELD(pt);
          }
	PT_END(pt);
}

PT_THREAD(task_led(struct pt *pt))
{
	  static uint32_t local_time_flag=0;
		static uint32_t LedPerTime=0;

	  PT_BEGIN(pt);	
	  local_time_flag = sys_time_flag;
		LedPerTime=sys_time_flag;
	  PT_WAIT_UNTIL(pt, ((sys_time_flag-local_time_flag) >= 600));	
			LedFunctionSet(LED_Y ,LED_OFF,0,LED_KEEP_ON);
			LedFunctionSet(LED_B ,LED_OFF,0,LED_KEEP_ON);
	  while(1) 
			{
				//led 
				if((sys_time_flag-LedPerTime) >= 100)
				 {
							LedPerTime=sys_time_flag;
							LedPeriodicProcessing(100);
				 }
				if(insert_flag==0)//未插入时刻
				{
						if(battery_status<1)//电池缺电
						{
								LedFunctionSet( LED_Y ,500,0,LED_KEEP_ON);
								LedFunctionSet( LED_B ,LED_OFF,0,LED_KEEP_ON);
						}
						else if(battery_status>0)
						{
								LedFunctionSet( LED_B ,LED_KEEP_ON,0,LED_KEEP_ON);
								LedFunctionSet( LED_Y ,LED_OFF,0,0);					
						}
				}
				else if(insert_flag==1)
				{
					if(charge_flag==1)//正在充电
					{
							LedFunctionSet( LED_B ,LED_OFF,0,LED_KEEP_ON);
							LedFunctionSet( LED_Y ,500,0,LED_KEEP_ON);	
					}
					else//已经充满 保持蓝灯常亮
					{
							LedFunctionSet( LED_B ,LED_KEEP_ON,0,LED_KEEP_ON);
							LedFunctionSet( LED_Y ,LED_OFF,0,LED_KEEP_ON);	
					}			   
				}			
				PT_YIELD(pt);
			}
	  PT_END(pt);

}

PT_THREAD(task_idle(struct pt *pt))
{
  static uint32_t local_time_flag=0;
  PT_BEGIN(pt);
  local_time_flag = sys_time_flag;
  PT_WAIT_UNTIL(pt, ((sys_time_flag-local_time_flag) >= 500));
  while(1)
		{
				local_time_flag = sys_time_flag;
				PT_WAIT_UNTIL(pt, ((sys_time_flag-local_time_flag) >= 500));
	//    wdt_counter_reload();
				if(0==blink_500MS)
				{
					 blink_500MS=1;
				}
				else
				{
					 blink_500MS=0;
				}
						 
				PT_YIELD(pt);
		}	
	PT_END(pt);
}

#ifndef DRIVE_BOARD
PT_THREAD(task_adc(struct pt *pt))//ADC进程处理函数
{
	static uint32_t local_time_flag=0;
  uint8_t index;
	static int32_t adc_voltage[10];
	int16_t volt_temp;
  PT_BEGIN(pt); 
  local_time_flag = sys_time_flag;
	PT_WAIT_UNTIL(pt, ((sys_time_flag-local_time_flag) >= 400));
  adc_ordinary_software_trigger_enable(ADC1, TRUE);//开始规则组采样
	while(1) 
			{
				local_time_flag = sys_time_flag;   
				PT_WAIT_UNTIL(pt, ((sys_time_flag-local_time_flag) >=500));//延迟到500ms 				
				if(adc_trans_complete_flag == 0)
				{
						break;
				}
				adc_trans_complete_flag = 0;
				for(index = 0; index < 10; index++)
				{
						adc_voltage[index] = adc1_ordinary_valuetab[index][0];
						//printf("adc_vbat[%d][0] = %d\r\n", index, adc_voltage[index]);
				} 
				qsort(adc_voltage, 10, sizeof(int32_t), compDec);
				volt_temp = (adc_voltage[4]+adc_voltage[5]+adc_voltage[6]+adc_voltage[7]) >> 2;
				device_voltage_mv = volt_temp*(3300*2)/4095;//100K:100K
				sys_volatge_refresh=1;
	
				battery_status=vbat_judge(device_voltage_mv);
				//printf("battery_status IS %d\r\n",battery_status);
				PT_YIELD(pt);
			}
	PT_END(pt);
}
PT_THREAD(task_motor(struct pt *pt))
{
  static uint32_t local_time_flag=0;
  static uint32_t count_click=0;
  static uint8_t start_lock_fg=0;

  PT_BEGIN(pt); 
  local_time_flag = sys_time_flag;
	PT_WAIT_UNTIL(pt, ((sys_time_flag-local_time_flag) >= 1200));
	while(1) 
	{     
		local_time_flag = sys_time_flag;
		PT_WAIT_UNTIL(pt, ((sys_time_flag-local_time_flag) >= 5));	

//		motor_run_status(motor_run_flag);		
		
		PT_YIELD(pt);
	}	
	PT_END(pt);
}
PT_THREAD(task_state(struct pt *pt))
{
  static uint32_t local_time_flag=0,local_charge_full_time_flag=0;
	PT_BEGIN(pt);
  local_time_flag = sys_time_flag;
	PT_WAIT_UNTIL(pt, ((sys_time_flag-local_time_flag) >= 500));
	while(1) 
          {
            if(0==insert_flag)
            {
             if(1==get_insert_state())
             {
                  printf("charger insert\r\n");
                  buzz_charge_flag=1;
                  insert_flag=1;
                  charge_flag=1;
                  work_flag=0;
                  local_charge_full_time_flag=sys_time_flag;
             }
            }
            if(1==insert_flag)
            {
                  if(0==get_insert_state())//如果插入检测为低电平,则已经拔出
                  {
                      local_time_flag = sys_time_flag;
                      PT_WAIT_UNTIL(pt, ((sys_time_flag-local_time_flag) >= 200));//延迟到500ms
                      if(0==get_insert_state())
                      {
                        printf("charger pull out\r\n");
                        buzz_charge_flag=1; 
                        insert_flag=0;
                        charge_flag=0;
                      }
                   }
                   else if(charge_flag==1)
                   {
                       if(1==get_charge_state())
                       {
                           charge_flag=0;
                           printf("charger full\r\n");
                       }
                   }
            }
/*			
            if((1==get_gc_insert_state())&&(gc_insert_flag==1))
            {
              gc_insert_flag=0;
//              printf("gc is not insert");
              buzz_charge_flag=1;
            }
            if((0==get_gc_insert_state())&&(gc_insert_flag==0))
            {
              gc_insert_flag=1;
//              printf("gc is insert");
              buzz_charge_flag=1;
            }
*/			
            local_time_flag = sys_time_flag;
            PT_WAIT_UNTIL(pt, ((sys_time_flag-local_time_flag) >=1000));//延迟到500ms
            PT_YIELD(pt);
          }	
	PT_END(pt);
}

//#define no_vol     0
//#define low_vol    1
//#define middle_vol 2
//#define high_vol   3

//uint8_t buzz_button_flag=0;//按键鸣叫标志  0不叫 1鸣叫 由buzz线程自我约束
//uint8_t buzz_gc_flag=0;    //根测鸣叫标志  0不叫 1普通叫 2急促叫 即将到达根尖 3最急催已达根尖 由gc线程约束
//uint8_t buzz_torque_flag=0;//力矩超限鸣叫标志 0不叫 1普通叫达到设定力矩 0.8处 2急促叫超出设定力矩 由电机线程约束
//uint8_t buzz_power_flag=0;// 低电量鸣叫标志   0不叫 1普通叫 由buzz线程自我约束


#endif
