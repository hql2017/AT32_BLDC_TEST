#ifndef _CONTROL_H_
#define _CONTROL_H_


extern unsigned short regi[128];
extern unsigned char update_command; //set to 1 to update speed or position command
extern unsigned char errorStatusLatch; 
extern int iq; //motor iq
extern int position; //motor position, unit:LSB. 65536LSBs/revolution

void reset_parameters(void);
void control_loop(void);
void clearError(void);





#endif
