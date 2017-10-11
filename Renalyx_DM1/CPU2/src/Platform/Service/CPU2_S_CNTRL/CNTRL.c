/*
 * CNTRL.c
 *
 * Created: 12/3/2014 2:15:05 PM
 *  Author: wid7
 */ 
#include "CNTRL.h"


uint8_t cpu2_sv_cntrl_activatepump(sv_pumptype sv_pump_id)
{
	CPU2_D_SET_PUMP(sv_pump_id);
	
	return 0;
}

uint8_t cpu2_sv_cntrl_deactivatepump(sv_pumptype sv_pump_id)
{
	CPU2_D_RESET_PUMP(sv_pump_id);
	
	return 0;
}


uint8_t cpu2_sv_cntrl_activate_pin(sv_valvetype sv_pin_id)
{
	CPU2_D_SET_VALVE(sv_pin_id);
	
	return 0;
}

uint8_t cpu2_sv_cntrl_deactivate_pin(sv_valvetype sv_pin_id)
{
	CPU2_D_RESET_VALVE(sv_pin_id);
	
	return 0;
}