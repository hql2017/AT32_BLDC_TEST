#ifndef __STORAGE
#define __STORAGE
#include "stdint.h"

void default_para_write_buff(void);
void start_para_write_read(void);
void para_copy_default(void);
void write_para_judge(void);
error_status buffer_compare(uint16_t* p_buffer1, uint16_t* p_buffer2, uint16_t buffer_length);

#endif
