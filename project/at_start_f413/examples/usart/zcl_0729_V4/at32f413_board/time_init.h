/*

*/

#ifndef __TIME_INIT_H_
#define __TIME_INIT_H_

void MotorTimerInit(unsigned int periodicTimeUs,unsigned int pystemClockFrequency);
void ConfigSys_Timer(unsigned int seriodicTimeMs,unsigned int systemClockFrequency);
void NormalTimerInit(unsigned int periodicTimeMs,unsigned int systemClockFrequency);

#endif

