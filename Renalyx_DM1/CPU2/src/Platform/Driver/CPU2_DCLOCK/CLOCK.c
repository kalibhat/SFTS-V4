/*
 * CLOCK.c
 *
 * Created: 12/2/2014 3:17:47 PM
 *  Author: wid7
 */ 
#include "CLOCK.h"

void CPU2_D_INIT_CLOCK()
{
	sysclk_init();
	PMC->PMC_WPMR  = 0x504D4300;
	PMC->PMC_PCER0 = 0xF8407900;
	PMC->PMC_PCER1 = 0x0000083F;
}