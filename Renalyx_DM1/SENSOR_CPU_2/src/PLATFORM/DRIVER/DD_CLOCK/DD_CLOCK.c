/*
 * DD_CLOCK.c
 *
 * Created: 3/7/2017 4:01:05 PM
 *  Author: Electrical
 */ 
#include "DD_CLOCK.h"


void SENSOR_D_INIT_CLOCK() {
	sysclk_init();
	PMC->PMC_WPMR  = 0x504D4300;	
	
	dd_pmc_enable_periph_clk(ID_PIOA);
	dd_pmc_enable_periph_clk(ID_PIOB);
	dd_pmc_enable_periph_clk(ID_PIOC);
    dd_pmc_enable_periph_clk(ID_PIOD);
    dd_pmc_enable_periph_clk(ID_SPI0);
	dd_pmc_enable_periph_clk(ID_CAN0);
	
	 dd_pmc_enable_periph_clk(ID_PMC);
	 dd_pmc_enable_periph_clk(ID_UART);
	 
	 dd_pmc_enable_periph_clk(ID_TC0); 
	 dd_pmc_enable_periph_clk(ID_TC6);		
	 
    PMC->PMC_WPMR  = 0x504D4301;
}
	
	
	
uint32_t dd_pmc_enable_periph_clk(uint32_t ul_id)	{
		if (ul_id > MAX_PERIPH_ID) {
			return 1;
		}
		if (ul_id < 32) {
			if ((PMC->PMC_PCSR0 & (1u << ul_id)) != (1u << ul_id)) {
				PMC->PMC_PCER0 = 1 << ul_id;
			}			
		} else {
			ul_id -= 32;
			if ((PMC->PMC_PCSR1 & (1u << ul_id)) != (1u << ul_id)) {
				PMC->PMC_PCER1 = 1 << ul_id;
			}			
		}
		return 0;
}
	