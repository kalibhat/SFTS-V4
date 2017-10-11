/*
 * cl_heparinfeedback_proto.h
 *
 * Created: 4/25/2017 7:24:38 PM
 *  Author: user
 */ 


#ifndef CL_HEPARINFEEDBACK_PROTO_H_
#define CL_HEPARINFEEDBACK_PROTO_H_

#include "cl_heparinfeedback.h"

Cl_ReturnCodeType cl_hep_pumpFeedback_timer(void);
Cl_ReturnCodeType cl_hep_pumpFeedback_start(void);
Cl_ReturnCodeType cl_hep_selftest_start(void);
Cl_ReturnCodeType cl_hep_selftest_stop(void);
Cl_ReturnCodeType cl_hep_bolus_start(void);
Cl_ReturnCodeType cl_hep_bolus_stop(void);
Cl_ReturnCodeType cl_hep_pumpFeedback_stop(void);
Cl_ReturnCodeType cl_hep_pumpFeedback_get_delta(int16_t* delta);
Cl_ReturnCodeType cl_hep_pumpFeedback_get_state(cl_hep_pump_states* pstate);
Cl_ReturnCodeType cl_hep_pump_enddetction_timer(void); // 20 ms clock



#endif /* CL_HEPARINFEEDBACK_PROTO_H_ */