/*
 * DD_CLOCK.c
 *
 * Created: 1/6/2014 11:42:37 AM
 *  Author: Gopalkrishna Kalibhat
 */ 
#include "DD_CLOCK.h"
// #include "cl_app/cl_bc_cntrl/inc/Cl_bc_controller.h"
// #include "cl_app/inc/cl_types.h"
// #include "sv_interface.h"
// void pin_edge_handler(const uint32_t id, const uint32_t index);
// extern Cl_ReturnCodeType  Cl_bc_controller(Cl_BC_EventType cl_bc_event);
// void check_interrupt(void);
// extern uint8_t sv_cntrl_activate_valve(sv_valvetype sv_valve_id);
// extern uint8_t sv_cntrl_deactivate_valve(sv_valvetype sv_valve_id);
// static chk_flag_toggle =0;
void DD_INIT_CLOCK()
{
	
	sysclk_init();
	PMC->PMC_WPMR  = 0x504D4300;
	PMC->PMC_PCER0 = 0xF8007900;
	PMC->PMC_PCER1 = 0x0000083F;
	
}
