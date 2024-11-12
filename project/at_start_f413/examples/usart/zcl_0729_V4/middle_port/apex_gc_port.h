#ifndef __APEX_GC__PORT
#define __APEX_GC__PORT


extern void APEXInit(void);
extern void gc_in_or_exit(unsigned char inOrExit);
extern void apex_frequency_set(unsigned short int frequency);
extern  int apex_depth_calculate( unsigned short int GC_8kValue,unsigned short int GC_400Value);
extern int GC_depth_vlaue(int depthValue, unsigned char readOrWrite);

extern void gc_list_init(int n0, int n1);


#endif
