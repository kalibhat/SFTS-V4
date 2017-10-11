/*
 * D_CLOCK.c
 *
 * Created: 12/8/2014 5:49:40 PM
 *  Author: wid7
 */ 
#include "D_CLOCK.h"

void CPU3_D_INIT_CLOCK()
{
	sysclk_init();
	PMC->PMC_WPMR  = 0x504D4300;
	PMC->PMC_PCER0 = 0xF8407900;
	PMC->PMC_PCER1 = 0x0000083F;
}