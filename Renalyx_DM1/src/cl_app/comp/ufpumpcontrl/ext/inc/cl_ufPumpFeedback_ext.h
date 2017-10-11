/*
 * cl_ufPumpFeedback_ext.h
 *
 * Created: 4/25/2017 4:35:53 PM
 *  Author: user
 */ 


#ifndef CL_UFPUMPFEEDBACK_EXT_H_
#define CL_UFPUMPFEEDBACK_EXT_H_

#include "cl_ufPumpFeedback.h"
extern Cl_ReturnCodeType cl_ufpumpFeedback_timer(void);
extern Cl_ReturnCodeType cl_ufpumpFeedback_start(void);
extern Cl_ReturnCodeType cl_ufpumpFeedback_get_state(cl_ufpump_states* pstate);
extern Cl_ReturnCodeType cl_ufpumpFeedback_stop(void);
extern Cl_ReturnCodeType cl_ufpumpFeedback_get_delta(int16_t* pdelta);
extern Cl_ReturnCodeType cl_selftestFeedback_start(void);
extern Cl_ReturnCodeType cl_selftestFeedback_stop(void);
#endif /* CL_UFPUMPFEEDBACK_EXT_H_ */