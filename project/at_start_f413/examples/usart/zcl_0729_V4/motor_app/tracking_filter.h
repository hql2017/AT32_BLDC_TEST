#ifndef _TRACKING_FILTER_H_
#define _TRACKING_FILTER_H_



#define TRACKING_DEFAULTS { 0, \
                       0, \
											 0, \
                       0, \
                       0, \
                       0, \
											 0, \
											 0, \
											0,\
											0,\
              		   (void (*)(long))tracking_calc}

typedef struct
{
	unsigned short alpha;
	unsigned short beta;
	int e;
	int angle;
	int Un;
	int position;
	int velocity; 
	long long int temp;
	long long int temp1;
	long long int temp2;
	void  (*calc)();
} TRACKING_TypeDef;

typedef TRACKING_TypeDef *Tracking_handle;

void tracking_calc(Tracking_handle);
#endif
