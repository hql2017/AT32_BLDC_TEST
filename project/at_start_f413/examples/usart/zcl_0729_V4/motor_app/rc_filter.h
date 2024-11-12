#ifndef _RC_FILTER_H_
#define _RC_FILTER_H_


#define RC_DEFAULTS { 0, \
											0,\
                       0, \
                       0, \
              		   (void (*)(long))rc_calc }

typedef struct
{
	unsigned short alpha;
	int e;
	int in;
	int out;
	void  (*calc)();
} RC_TypeDef;

typedef RC_TypeDef *RC_handle;

void rc_calc(RC_handle);
#endif
