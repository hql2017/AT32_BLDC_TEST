/*
*@file        WDT.c
*@brief       applation
*@attention   none
*@version     V1.0
*@data        2022/5/24
*@author      xuwenxi
*/
#include "wdt.h"
#include "at32f413.h"
/**
  * @brief  window dog init
  * @param  none
  * @retval none
  */
void Wdt_Init(void)
{
	
	wdt_register_write_enable(TRUE);
//	wdt_divider_set(WDT_CLK_DIV_4);
//	wdt_reload_value_set(3000 - 1);//300ms,value<4096	
	
	wdt_divider_set(WDT_CLK_DIV_8);//MAX_WDT_FEED_TIME_MS==300*2
	wdt_reload_value_set(3000 - 1);//MAX_WDT_FEED_TIME_MS*2=300*2;value<4096
	wdt_enable();	
}
/**
  * @brief  window dog reset
  * @param  timeMs=MAX_WDT_FEED_TIME_MS*n
  * @retval none
  */
void Wdt_Reset(unsigned int timeMs)
{
	
	wdt_register_write_enable(TRUE);
//	wdt_divider_set(WDT_CLK_DIV_4);
//	wdt_reload_value_set(3000 - 1);//300ms,value<4096		
	wdt_divider_set(WDT_CLK_DIV_128);//MAX_WDT_FEED_TIME_MS==300*32Ms=9.6S
	wdt_reload_value_set(3000 - 1);//MAX_WDT_FEED_TIME_MS*2=300*2;value<4096
	wdt_enable();	
}

