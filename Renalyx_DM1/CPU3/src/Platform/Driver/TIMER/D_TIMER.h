/*
 * D_TIMER.h
 *
 * Created: 12/9/2014 11:03:56 AM
 *  Author: wid7
 */ 


#ifndef D_TIMER_H_
#define D_TIMER_H_

#include "asf.h"


void CPU3_D_INIT_TIMER(void);
uint8_t CPU3_D_MAIN_TIMER(Tc *tc, uint32_t channel, IRQn_Type irq, uint32_t rc);
void acid_startTimer1(Tc *tc, uint32_t channel, IRQn_Type irq, uint32_t frequency);
void bicar_startTimer1(Tc *tc, uint32_t channel, IRQn_Type irq, uint32_t frequency);
void startTimer1(Tc *tc, uint32_t channel, IRQn_Type irq, uint32_t stepcount);
void startTimer2(Tc *tc, uint32_t channel, IRQn_Type irq, uint32_t stepcount);
void startTimerSec(Tc *tc, uint32_t channel, IRQn_Type irq, uint32_t stepcount);


#endif /* D_TIMER_H_ */