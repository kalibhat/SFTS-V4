/*
 * S_TIMER.c
 *
 * Created: 12/3/2014 2:16:47 PM
 *  Author: wid7
 */ 
#include "S_TIMER.h"


uint8_t CPU2_S_START_TIMER(uint8_t timer_no, uint32_t timer_res)
{
	// 	timer_no = 2;
	// 	timer_res = 100;
	// 	printf("inside sv start");
	CPU2_D_CONFIG_TIMER(timer_no, timer_res);
	
	return 0;
}

uint8_t CPU2_S_STOP_TIMER(uint8_t timer_no)
{
	
	CPU2_D_STOP_TIMER(timer_no);
	
	return 0;
}