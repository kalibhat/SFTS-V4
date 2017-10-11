/*
 * DD_CLOCK.h
 *
 * Created: 3/7/2017 4:01:19 PM
 *  Author: Electrical
 */ 


#ifndef DD_CLOCK_H_
#define DD_CLOCK_H_

#include "asf.h"

# define MAX_PERIPH_ID    44

void SENSOR_D_INIT_CLOCK(void);
uint32_t dd_pmc_enable_periph_clk(uint32_t ul_id);






#endif /* DD_CLOCK_H_ */