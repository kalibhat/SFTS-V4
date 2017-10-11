/*
 * SYS.c
 *
 * Created: 12/3/2014 2:15:56 PM
 *  Author: wid7
 */ 

#include "S_SYS.h"
#include "Platform/Service/CPU2_S_TIMER/S_TIMER.h"
#include "Platform/Driver/Driver_calls.h"

void CPU2_S_SYSTEM_INIT()
{
	CPU2_D_INIT_CLOCK();
//	CPU2_D_INIT_ADC();
	CPU2_D_INIT_INT();
	CPU2_D_INIT_PIO();
	CPU2_D_INIT_PWM();
	CPU2_D_INIT_TIMER();
	DD_INIT_CAN();
	CPU2_DD_INIT_SPI();
	//CPU2_D_INIT_IIC();
	
	CPU2_S_START_TIMER(0,1000);			//1ms means 1000Hz interrupt frequency
}

