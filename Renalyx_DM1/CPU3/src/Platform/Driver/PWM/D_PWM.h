/*
 * D_PWM.h
 *
 * Created: 12/8/2014 6:02:49 PM
 *  Author: wid7
 */ 


#ifndef D_PWM_H_
#define D_PWM_H_

#include "asf.h"

typedef enum _pwm_channel
{
	#if (SAM3XA)
	CH_0	= PWM_CHANNEL_0,
	CH_1	= PWM_CHANNEL_1,
	CH_2	= PWM_CHANNEL_2,
	CH_4	= PWM_CHANNEL_4,
	CH_5    = PWM_CHANNEL_5,
	CH_6    = PWM_CHANNEL_6,
	CH_7    = PWM_CHANNEL_7,
	#endif
} pwm_chanel;


void CPU3_D_INIT_PWM(void);
uint32_t CPU3_D_SET_PRDY(const pwm_chanel CHANNEL_NUMBER,uint32_t prd_val);
uint32_t CPU3_D_SET_DUTY(const pwm_chanel CHANNEL_NUMBER,uint32_t dty_val);



#endif /* D_PWM_H_ */