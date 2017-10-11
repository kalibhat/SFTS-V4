/*
 * S_CNTRL.h
 *
 * Created: 12/10/2014 5:29:45 PM
 *  Author: wid7
 */ 


#ifndef S_CNTRL_H_
#define S_CNTRL_H_

#include "asf.h"
#include "Platform/Service/sv_interface.h"
#include "Platform/Driver/Driver_call.h"

uint8_t cpu3_sv_cntrl_activatepump(sv_pumptype sv_pump_id);
uint8_t cpu3_sv_cntrl_deactivatepump(sv_pumptype sv_pump_id);
uint8_t cpu3_sv_cntrl_activate_pin(sv_valvetype sv_pin_id);
uint8_t cpu3_sv_cntrl_deactivate_pin(sv_valvetype sv_pin_id);




#endif /* S_CNTRL_H_ */