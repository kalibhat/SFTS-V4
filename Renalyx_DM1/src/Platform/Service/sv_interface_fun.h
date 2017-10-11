/*
 * sv_interface_fun.h
 *
 * Created: 4/25/2017 12:23:10 PM
 *  Author: user
 */ 


#ifndef SV_INTERFACE_FUN_H_
#define SV_INTERFACE_FUN_H_

#include "sv_interface.h"

extern uint8_t sv_cntrl_setyellowalarm(void);
extern uint8_t sv_cntrl_resetyellowalarm(void);
extern uint8_t sv_cntrl_incheater(int32_t dty_val);
extern uint8_t sv_cntrl_poweronheater(void);
extern uint8_t sv_cntrl_enable_bypass( void);
extern uint8_t sv_cntrl_enable_loopback(void);
extern uint8_t sv_cntrl_setflowpath(sv_flowpathtype);
extern void sv_prop_stopopenfill(void);
extern uint8_t sv_cntrl_setpumpspeed(sv_pumptype sv_pump_id,uint32_t speed);
extern uint8_t sv_cntrl_activate_valve(sv_valvetype );
extern uint8_t sv_cntrl_deactivate_valve(sv_valvetype );
extern void sv_prop_startpropeo_aligning(void);
extern void sv_prop_startmixing(void);
extern void sv_prop_stopmixing(void);
extern void sv_prop_startacidpump(void);
extern void sv_prop_bicarb_selftest(void);   //BICAR SELFTEST
extern void sv_prop_bicarb_selftest_f_b(void);
extern void sv_prop_startbicarbpump(void);
extern uint8_t sv_cs_setpotvalue(uint32_t resistance) ;
void SV_CS_SET_POT(uint16_t* data);
extern uint8_t  sv_cntrl_deactivatepump(sv_pumptype);
extern uint8_t  sv_cntrl_activatepump(sv_pumptype);
extern uint8_t sv_cntrl_setredalarm(void);
extern uint8_t sv_cntrl_setgreenalarm(void);
extern uint8_t sv_cntrl_setyellowalarm(void);
extern uint8_t sv_cntrl_buzzer(void);
extern uint8_t sv_cntrl_resetredalarm(void);
extern uint8_t sv_cntrl_resetyellowalarm(void);
extern uint8_t sv_cntrl_resetgreenalarm(void);
extern uint8_t sv_cntrl_nobuzzer(void);
extern uint8_t sv_cntrl_activate_valve(sv_valvetype sv_valve_id);
extern uint8_t sv_cntrl_deactivate_valve(sv_valvetype sv_valve_id);
extern uint8_t sv_cntrl_enable_bypass(void);
extern uint8_t sv_cntrl_disable_bypass(void);
extern void sv_prop_start_disinfect_intake(void);
extern void sv_prop_stop_disinfect_intake(void);
extern void sv_prop_start_citricdisinfect_intake(void);
extern void sv_prop_stop_citricdisinfect_intake(void);
extern uint8_t sv_cntrl_enable_loopback(void);
extern uint8_t sv_cntrl_disable_loopback(void);
extern uint8_t sv_cntrl_poweroffheater(void);
extern uint8_t sv_cntrl_deactivatevenousclamp(void);

extern uint8_t sv_cntrl_setredalarm(void);
extern uint8_t sv_cntrl_setgreenalarm(void);
extern uint8_t sv_cntrl_setyellowalarm(void);
extern uint8_t sv_cntrl_buzzer(void);
extern uint8_t sv_cntrl_resetredalarm(void);
extern uint8_t sv_cntrl_resetgreenalarm(void);
extern uint8_t sv_cntrl_resetyellowalarm(void);
extern uint8_t sv_cntrl_nobuzzer(void);
extern uint8_t sv_cntrl_activatevenousclamp(void);
extern uint8_t sv_cntrl_disable_loopback(void);
extern uint8_t sv_cntrl_decheater(int32_t dty_val);
extern uint16_t sv_status_getsystemstatus(Sys_statusType* sensor_struct);
extern void sv_prop_startopenfill(void);
extern uint8_t sv_cntrl_setHepa_dir(void);
extern uint8_t sv_cntrl_resetHepa_dir(void);
extern void SV_SYSTEM_INIT(void);



#endif /* SV_INTERFACE_FUN_H_ */