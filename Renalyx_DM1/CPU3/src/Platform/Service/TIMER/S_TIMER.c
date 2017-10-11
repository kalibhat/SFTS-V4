/*
 * S_TIMER.c
 *
 * Created: 12/11/2014 11:25:13 AM
 *  Author: wid7
 */ 
#include "S_TIMER.h"


uint8_t CPU3_S_START_TIMER(uint8_t timer_no, uint32_t timer_res)
{
	// 	timer_no = 2;
	// 	timer_res = 100;
	// 	printf("inside sv start");
	CPU3_D_CONFIG_TIMER(timer_no, timer_res);
	
	return 0;
}

uint8_t CPU3_S_STOP_TIMER(uint8_t timer_no)
{
	
	CPU3_D_STOP_TIMER(timer_no);
	
	return 0;
}