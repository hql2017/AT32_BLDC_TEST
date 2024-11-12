#ifndef __APP_USER_STORAGE
#define __APP_USER_STORAGE

#include "at32f413.h"


#ifdef USE_EXIT_FLASH
//"W25Q64"
	extern void flash_read(unsigned  int read_addr, unsigned short int *p_buffer, unsigned short int num_read);
	extern unsigned char flash_write(unsigned  int write_addr, unsigned short int *p_buffer, unsigned short int num_write);
#else


extern void flash_read(unsigned  int read_addr, unsigned short int *p_buffer, unsigned short int num_read);
extern error_status flash_write(unsigned  int write_addr, unsigned short int *p_buffer, unsigned short int num_write);
	
void default_para_write_buff(void);
void start_para_write_read(void);
void para_copy_default(void);
void write_para_judge(void);
error_status buffer_compare(uint16_t* p_buffer1, uint16_t* p_buffer2, uint16_t buffer_length);

#endif

//#define FLASH_SYSTEM_PARAM_ADDR                 (0x08000000 + 1024 * 204) 
//#define FLASH_MOTOR_PARAM_ADDR           (0x08000000 + 1024 * 208) 

#define FLASH_SYSTEM_PARAM_ADDR                 (0x08000000 + 1024 * 200) 
#define FLASH_MOTOR_PARAM_ADDR           (0x08000000 + 1024 * 204)

#endif
