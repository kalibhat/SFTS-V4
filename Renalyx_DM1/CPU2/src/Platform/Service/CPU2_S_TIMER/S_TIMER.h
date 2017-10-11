/*
 * S_TIMER.h
 *
 * Created: 12/3/2014 2:16:34 PM
 *  Author: wid7
 */ 


#ifndef S_TIMER_H_
#define S_TIMER_H_

#include "asf.h"
#include "Platform/Driver/Driver_calls.h"

uint8_t CPU2_S_START_TIMER(uint8_t timer_no, uint32_t timer_res);
uint8_t CPU2_S_STOP_TIMER(uint8_t timer_no);



#endif /* S_TIMER_H_ */