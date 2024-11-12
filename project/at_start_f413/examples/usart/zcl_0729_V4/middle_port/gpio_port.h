#ifndef __GPIO_PORT
#define __GPIO_PORT

#include "at32f413.h"


void user_gpio_init(void);

void device_power_on(void);
void device_power_off(void);

extern flag_status get_run_button_state(void);
extern flag_status get_p_button_state(void);
extern flag_status get_add_button_state(void);
extern flag_status get_sub_button_state(void);
extern flag_status get_s_button_state(void);

 extern flag_status get_insert_state(void);
 extern flag_status get_charge_state(void);
 extern flag_status get_gc_insert_state(void);
extern flag_status get_gc_state(void);


extern void  LedFunctionSet(unsigned char num ,unsigned short int  keepTimeMs,unsigned char ledPriority,unsigned short int stillTimeMs);
extern void  LedPeriodicProcessing( unsigned short int realTimeMs,unsigned short int perTimeMs);

#endif
