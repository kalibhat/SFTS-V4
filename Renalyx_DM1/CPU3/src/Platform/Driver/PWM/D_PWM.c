/*
 * D_PWM.c
 *
 * Created: 12/8/2014 6:02:36 PM
 *  Author: wid7
 */ 

#include "D_PWM.h"

void CPU3_D_INIT_PWM()
{
	PWM->PWM_WPCR = 0x50574D00;
	
	PWM->PWM_CH_NUM[4].PWM_CMR = PWM_CMR_CPOL|PWM_CMR_CPRE_MCK_DIV_32|PWM_CMR_CES|PWM_CMR_DTE|PWM_CMR_DTHI|PWM_CMR_DTLI;
	PWM->PWM_CH_NUM[5].PWM_CMR = PWM_CMR_CPOL|PWM_CMR_CPRE_MCK_DIV_32|PWM_CMR_CES|PWM_CMR_DTE|PWM_CMR_DTHI|PWM_CMR_DTLI;

	PWM->PWM_DIS = 0x000000FF;
	
	PWM->PWM_CH_NUM[4].PWM_CPRD = 0x00000A41;
	PWM->PWM_CH_NUM[4].PWM_CDTY = 0x00000700;
	
	PWM->PWM_CH_NUM[5].PWM_CPRD = 0x00000A41;
	PWM->PWM_CH_NUM[5].PWM_CDTY = 0x00000700;
	PWM->PWM_ENA = 0x000000FF;
}


uint32_t CPU3_D_SET_DUTY(const pwm_chanel CHANNEL_NUMBER,uint32_t DUTY)
{
	PWM->PWM_CH_NUM[CHANNEL_NUMBER].PWM_CDTY = DUTY;
	
	return 0;
}

uint32_t CPU3_D_SET_PRDY(const pwm_chanel CHANNEL_NUMBER,uint32_t PERIOD_VALUE)
{
	PWM->PWM_CH_NUM[CHANNEL_NUMBER].PWM_CPRDUPD = PERIOD_VALUE;
	
	return 0;
}