/*
 * TIMER.c
 *
 * Created: 12/2/2014 3:38:36 PM
 *  Author: wid7
 */ 
#include "TIMER.h"


extern uint32_t ra,rc;

void CPU2_D_INIT_TIMER()
{
	TC0->TC_WPMR = 0x54494D00;
	TC1->TC_WPMR = 0x54494D00;
//	TC2->TC_WPMR = 0x54494D00;
	pmc_enable_periph_clk(ID_TC0);
	pmc_enable_periph_clk(ID_TC1);
//	pmc_enable_periph_clk(ID_TC2);
	pmc_enable_periph_clk(ID_TC6);
	TC0->TC_CHANNEL[0].TC_CMR = TC_CMR_WAVE|TC_CMR_WAVSEL_UP_RC |TC_CMR_TCCLKS_TIMER_CLOCK4;
	TC0->TC_CHANNEL[1].TC_CMR = TC_CMR_WAVE|TC_CMR_WAVSEL_UP_RC |TC_CMR_TCCLKS_TIMER_CLOCK4;
	TC0->TC_CHANNEL[2].TC_CMR = TC_CMR_WAVE|TC_CMR_WAVSEL_UP_RC |TC_CMR_TCCLKS_TIMER_CLOCK4;
//	TC2->TC_CHANNEL[0].TC_CMR = TC_CMR_WAVE|TC_CMR_WAVSEL_UP_RC |TC_CMR_TCCLKS_TIMER_CLOCK4;
	
	TC0->TC_CHANNEL[0].TC_IER = TC_IER_CPCS;
	TC0->TC_CHANNEL[1].TC_IER = TC_IER_CPCS;
	TC0->TC_CHANNEL[2].TC_IER = TC_IER_CPCS;
	TC1->TC_CHANNEL[0].TC_IER = TC_IER_CPCS;
//	TC2->TC_CHANNEL[0].TC_IER = TC_IER_CPCS;
}


uint8_t CPU2_D_MAIN_TIMER(Tc *tc, uint32_t channel, IRQn_Type irq, uint32_t fre)
{
	pmc_set_writeprotect(0);
	channel=0;
	TC2->TC_CHANNEL[0].TC_CMR = TC_CMR_WAVE | TC_CMR_WAVSEL_UP_RC |TC_CMR_TCCLKS_TIMER_CLOCK4;
	tc_write_rc(tc, channel, fre);
	tc_start(tc, channel);
	tc_enable_interrupt(tc, channel, TC_IER_CPCS);
	/*if (SysTick_Config(sysclk_get_cpu_hz() / 1000)) {
		puts("-F- Systick configuration error\r");
		while (1);
	}*/
}

void startTimer1(Tc *tc, uint32_t channel, IRQn_Type irq, uint32_t stepcount)
{
	pmc_set_writeprotect(0);
	pmc_enable_periph_clk(ID_TC0);
	tc_init(tc, channel, TC_CMR_WAVE | TC_CMR_WAVSEL_UP_RC |TC_CMR_TCCLKS_TIMER_CLOCK4);
	TC0->TC_CHANNEL[0].TC_CMR = TC_CMR_WAVE | TC_CMR_WAVSEL_UP_RC |TC_CMR_TCCLKS_TIMER_CLOCK4;
	tc_write_rc(tc, channel, stepcount);
	tc_enable_interrupt(TC0,0,TC_IER_CPCS);
	tc_start(tc, channel);
}

void startTimer2(Tc *tc, uint32_t channel, IRQn_Type irq, uint32_t rampingcount)
{
	pmc_set_writeprotect(0);
	pmc_enable_periph_clk(ID_TC1);
	tc_init(tc, channel, TC_CMR_WAVE | TC_CMR_WAVSEL_UP_RC |TC_CMR_TCCLKS_TIMER_CLOCK4);
	TC0->TC_CHANNEL[1].TC_CMR = TC_CMR_WAVE | TC_CMR_WAVSEL_UP_RC |TC_CMR_TCCLKS_TIMER_CLOCK4;
	tc_write_rc(tc, channel, rampingcount);
	tc_enable_interrupt(TC0,1,TC_IER_CPCS);
	tc_start(tc, channel);
}
void startTimer3(Tc *tc, uint32_t channel, IRQn_Type irq, uint32_t rampingcount)
{
	pmc_set_writeprotect(0);
	pmc_enable_periph_clk(ID_TC2);
	tc_init(tc, channel, TC_CMR_WAVE | TC_CMR_WAVSEL_UP_RC |TC_CMR_TCCLKS_TIMER_CLOCK4);
	TC0->TC_CHANNEL[2].TC_CMR = TC_CMR_WAVE | TC_CMR_WAVSEL_UP_RC |TC_CMR_TCCLKS_TIMER_CLOCK4;
	tc_write_rc(tc, channel, rampingcount);
	tc_enable_interrupt(TC0,2,TC_IER_CPCS);
	tc_start(tc, channel);
}