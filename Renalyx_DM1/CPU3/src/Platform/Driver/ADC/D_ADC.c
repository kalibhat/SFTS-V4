/*
 * D_ADC.c
 *
 * Created: 12/9/2014 10:42:14 AM
 *  Author: wid7
 */ 
#include "D_ADC.h"

void CPU3_D_INIT_ADC()
{
	ADC->ADC_WPMR = 0x41444300;
	ADC->ADC_MR	  = 0x11040400;
	ADC->ADC_CHER = 0x000018FF;
}

uint16_t CPU3_D_READ_ADC( uint8_t channel_number)
{
	int16_t ADC_data;
	ADC->ADC_CR = 0x00000002;

	while((ADC->ADC_ISR) & (1<<channel_number) == 0); // do nothing

	ADC_data = ADC->ADC_CDR[channel_number];

	return ADC_data;
	
}