/*
 * S_SYS.c
 *
 * Created: 12/11/2014 11:22:57 AM
 *  Author: wid7
 */ 
#include "S_SYS.h"
#include "Platform/Service/TIMER/S_TIMER.h"

void CPU3_S_SYSTEM_INIT()
{
	CPU3_D_INIT_CLOCK();
//	CPU3_D_INIT_ADC();
	CPU3_D_INIT_INT();
	CPU3_D_INIT_PIO();
	CPU3_D_INIT_PWM();
	CPU3_D_INIT_TIMER();
	DD_INIT_CAN();
	//CPU3_D_INIT_IIC();
	
	
	CPU3_S_START_TIMER(0,1000);			//1ms means 1000Hz interrupt frequency
}
