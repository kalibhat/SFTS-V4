/*
 * D_TIMER.c
 *
 * Created: 12/9/2014 11:04:09 AM
 *  Author: wid7
 */ 

#include "D_TIMER.h"


extern uint32_t ra,rc;

void CPU3_D_INIT_TIMER()
{
	TC0->TC_WPMR = 0x54494D00;
	TC1->TC_WPMR = 0x54494D00;
	TC2->TC_WPMR = 0x54494D00;
	pmc_enable_periph_clk(ID_TC0);
	pmc_enable_periph_clk(ID_TC1);
	pmc_enable_periph_clk(ID_TC3);
	pmc_enable_periph_clk(ID_TC7);
	pmc_enable_periph_clk(ID_TC6);
	TC0->TC_CHANNEL[0].TC_CMR = TC_CMR_WAVE|TC_CMR_WAVSEL_UP_RC |TC_CMR_TCCLKS_TIMER_CLOCK4;
	TC0->TC_CHANNEL[1].TC_CMR = TC_CMR_WAVE|TC_CMR_WAVSEL_UP_RC |TC_CMR_TCCLKS_TIMER_CLOCK4;
	
	TC1->TC_CHANNEL[0].TC_CMR = TC_CMR_WAVE|TC_CMR_WAVSEL_UP_RC |TC_CMR_TCCLKS_TIMER_CLOCK4;
	TC1->TC_CHANNEL[1].TC_CMR = TC_CMR_WAVE|TC_CMR_WAVSEL_UP_RC |TC_CMR_TCCLKS_TIMER_CLOCK4;
	
	TC2->TC_CHANNEL[0].TC_CMR = TC_CMR_WAVE|TC_CMR_WAVSEL_UP_RC |TC_CMR_TCCLKS_TIMER_CLOCK4;
	TC2->TC_CHANNEL[1].TC_CMR = TC_CMR_WAVE|TC_CMR_WAVSEL_UP_RC |TC_CMR_TCCLKS_TIMER_CLOCK4;
	
	TC0->TC_CHANNEL[0].TC_IER = TC_IER_CPCS;
	TC0->TC_CHANNEL[1].TC_IER = TC_IER_CPCS;
	TC1->TC_CHANNEL[0].TC_IER = TC_IER_CPCS;
	TC1->TC_CHANNEL[1].TC_IER = TC_IER_CPCS;
	TC2->TC_CHANNEL[0].TC_IER = TC_IER_CPCS;
//	TC2->TC_CHANNEL[1].TC_IER = TC_IER_CPCS;
}


uint8_t CPU3_D_MAIN_TIMER(Tc *tc, uint32_t channel, IRQn_Type irq, uint32_t fre)
{
	pmc_set_writeprotect(0);
	channel=0;
	TC2->TC_CHANNEL[0].TC_CMR = TC_CMR_WAVE | TC_CMR_WAVSEL_UP_RC |TC_CMR_TCCLKS_TIMER_CLOCK4;
	tc_write_rc(tc, channel,fre);
	tc_start(tc, channel);
	tc_enable_interrupt(tc, channel, TC_IER_CPCS);
}

void acid_startTimer1(Tc *tc, uint32_t channel, IRQn_Type irq, uint32_t stepcount)
{
	//printf("h\n");
	pmc_set_writeprotect(0);
	pmc_enable_periph_clk(ID_TC0);
	//printf("%d\n",stepcount);
	tc_init(tc, channel, TC_CMR_WAVE | TC_CMR_WAVSEL_UP_RC |TC_CMR_TCCLKS_TIMER_CLOCK3);
	TC0->TC_CHANNEL[0].TC_CMR = TC_CMR_WAVE | TC_CMR_WAVSEL_UP_RC |TC_CMR_TCCLKS_TIMER_CLOCK3;
	tc_write_rc(tc, channel, stepcount);
	tc_enable_interrupt(TC0,0,TC_IER_CPCS);
	tc_start(tc, channel);
}

void bicar_startTimer1(Tc *tc, uint32_t channel, IRQn_Type irq, uint32_t stepcount)
{
	pmc_set_writeprotect(0);
	pmc_enable_periph_clk(ID_TC1);
	tc_init(tc, channel, TC_CMR_WAVE | TC_CMR_WAVSEL_UP_RC |TC_CMR_TCCLKS_TIMER_CLOCK3);
	TC0->TC_CHANNEL[1].TC_CMR = TC_CMR_WAVE | TC_CMR_WAVSEL_UP_RC |TC_CMR_TCCLKS_TIMER_CLOCK3;
	tc_write_rc(tc, channel, stepcount);
	tc_enable_interrupt(TC0,1,TC_IER_CPCS);
	tc_start(tc, channel);
}

//
// void startTimerSec(Tc *tc, uint32_t channel, IRQn_Type irq, uint32_t stepcount)
// {
// 	//printf("h\n");
// 	pmc_set_writeprotect(0);
// 	pmc_enable_periph_clk(ID_TC7);
// 	tc_init(tc, channel, TC_CMR_WAVE | TC_CMR_WAVSEL_UP_RC |TC_CMR_TCCLKS_TIMER_CLOCK4);
// 	TC2->TC_CHANNEL[1].TC_CMR = TC_CMR_WAVE | TC_CMR_WAVSEL_UP_RC |TC_CMR_TCCLKS_TIMER_CLOCK4;
// 	tc_write_rc(tc, channel, stepcount);
// 	tc_enable_interrupt(TC2,1,TC_IER_CPCS);
// 	tc_start(tc, channel);
// }

void startTimer1(Tc *tc, uint32_t channel, IRQn_Type irq, uint32_t stepcount)
{
	//printf("h\n");
	pmc_set_writeprotect(0);
	pmc_enable_periph_clk(ID_TC2);
	tc_init(tc, channel, TC_CMR_WAVE | TC_CMR_WAVSEL_UP_RC |TC_CMR_TCCLKS_TIMER_CLOCK4);
	TC0->TC_CHANNEL[2].TC_CMR = TC_CMR_WAVE | TC_CMR_WAVSEL_UP_RC |TC_CMR_TCCLKS_TIMER_CLOCK4;
	tc_write_rc(tc, channel, stepcount);
	tc_enable_interrupt(TC0,2,TC_IER_CPCS);
	tc_start(tc, channel);
}
void startTimer2(Tc *tc, uint32_t channel, IRQn_Type irq, uint32_t stepcount)
{
	//printf("h\n");
	pmc_set_writeprotect(0);
	pmc_enable_periph_clk(ID_TC3);
	tc_init(tc, channel, TC_CMR_WAVE | TC_CMR_WAVSEL_UP_RC |TC_CMR_TCCLKS_TIMER_CLOCK4);
	TC1->TC_CHANNEL[0].TC_CMR = TC_CMR_WAVE | TC_CMR_WAVSEL_UP_RC |TC_CMR_TCCLKS_TIMER_CLOCK4;
	tc_write_rc(tc, channel, stepcount);
	tc_enable_interrupt(TC1,0,TC_IER_CPCS);
	tc_start(tc, channel);
}

