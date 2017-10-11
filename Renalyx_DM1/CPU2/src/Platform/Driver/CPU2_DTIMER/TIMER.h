/*
 * TIMER.h
 *
 * Created: 12/2/2014 3:38:21 PM
 *  Author: wid7
 */ 


#ifndef TIMER_H_
#define TIMER_H_

#include "asf.h"


void CPU2_D_INIT_TIMER(void);
uint8_t CPU2_D_MAIN_TIMER(Tc *tc, uint32_t channel, IRQn_Type irq, uint32_t rc);
void startTimer1(Tc *tc, uint32_t channel, IRQn_Type irq, uint32_t frequency);
void startTimer2(Tc *tc, uint32_t channel, IRQn_Type irq, uint32_t frequency);
void startTimer3(Tc *tc, uint32_t channel, IRQn_Type irq, uint32_t frequency);

#endif /* TIMER_H_ */