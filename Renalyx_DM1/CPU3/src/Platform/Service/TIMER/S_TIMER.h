/*
 * S_TIMER.h
 *
 * Created: 12/11/2014 11:25:25 AM
 *  Author: wid7
 */ 


#ifndef S_TIMER_H_
#define S_TIMER_H_

#include "asf.h"
#include "Platform/Driver/Driver_call.h"

uint8_t CPU3_S_START_TIMER(uint8_t timer_no, uint32_t timer_res);
uint8_t CPU3_S_STOP_TIMER(uint8_t timer_no);



#endif /* S_TIMER_H_ */