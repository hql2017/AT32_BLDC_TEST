#ifndef __MAIN_THREAD
#define __MAIN_THREAD
#include "pt.h"
#include "stdint.h"
#include "common_function.h"
#include "para_list.h"
#include "queue.h"
extern PT_THREAD(task_menu(struct pt *pt));//菜单进程处理函数
extern PT_THREAD(task_adc(struct pt *pt));//ADC进程处理函数
extern PT_THREAD(task_button(struct pt *pt));
extern PT_THREAD(task_motor(struct pt *pt));
extern PT_THREAD(task_state(struct pt *pt));
extern PT_THREAD(task_fast_adc(struct pt *pt));
extern PT_THREAD(task_buzz(struct pt *pt));
extern PT_THREAD(task_led(struct pt *pt));
extern PT_THREAD(task_gc(struct pt *pt));
extern PT_THREAD(task_idle(struct pt *pt));
extern PT_THREAD(task_uart(struct pt *pt));

extern struct pt pt_uart;
extern struct pt pt_adc;
extern struct pt pt_menu;
extern struct pt pt_button;
extern struct pt pt_motor;
extern struct pt pt_state;
extern struct pt pt_fast_adc;
extern struct pt pt_buzz;
extern struct pt pt_led;
extern struct pt pt_gc;
extern struct pt pt_idle;

extern QUEUE_HandleTypeDef signal_que;

extern union Para_Union pa_un; 
extern union Pattern_Union pat_un; 
extern union Motor_Para_Union mt_un;
extern uint8_t work_flag; //工作标志 0 停止工作 1 开始工作
#endif
