/*
 * CNTRL.h
 *
 * Created: 12/3/2014 2:15:19 PM
 *  Author: wid7
 */ 


#ifndef CNTRL_H_
#define CNTRL_H_

#include "asf.h"
#include "Platform/Service/CPU2_S_interface.h"
#include "Platform/Driver/Driver_calls.h"

uint8_t cpu2_sv_cntrl_activatepump(sv_pumptype sv_pump_id);
uint8_t cpu2_sv_cntrl_deactivatepump(sv_pumptype sv_pump_id);
uint8_t cpu2_sv_cntrl_activate_pin(sv_valvetype sv_pin_id);
uint8_t cpu2_sv_cntrl_deactivate_pin(sv_valvetype sv_pin_id);

#endif /* CNTRL_H_ */