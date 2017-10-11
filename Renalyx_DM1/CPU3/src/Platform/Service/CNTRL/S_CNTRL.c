/*
 * S_CNTRL.c
 *
 * Created: 12/10/2014 5:29:56 PM
 *  Author: wid7
 */ 

#include "S_CNTRL.h"


uint8_t cpu3_sv_cntrl_activatepump(sv_pumptype sv_pump_id)
{
	CPU3_D_SET_PUMP(sv_pump_id);
	
	return 0;
}

uint8_t cpu3_sv_cntrl_deactivatepump(sv_pumptype sv_pump_id)
{
	CPU3_D_RESET_PUMP(sv_pump_id);
	
	return 0;
}


uint8_t cpu3_sv_cntrl_activate_pin(sv_valvetype sv_pin_id)
{
	CPU3_D_SET_VALVE(sv_pin_id);
	
	return 0;
}

uint8_t cpu3_sv_cntrl_deactivate_pin(sv_valvetype sv_pin_id)
{
//	CPU3_D_RESET_VALVE(sv_pin_id);
	
	return 0;
}