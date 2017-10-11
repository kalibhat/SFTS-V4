/*
 * cl_ufPumpFeedback_proto.h
 *
 * Created: 4/25/2017 7:39:09 PM
 *  Author: user
 */ 


#ifndef CL_UFPUMPFEEDBACK_PROTO_H_
#define CL_UFPUMPFEEDBACK_PROTO_H_

Cl_ReturnCodeType cl_ufpumpFeedback_timer(void);
Cl_ReturnCodeType cl_ufpumpFeedback_start(void);
Cl_ReturnCodeType cl_ufpumpFeedback_stop(void);
Cl_ReturnCodeType cl_ufpumpFeedback_get_delta(int16_t* delta);
Cl_ReturnCodeType cl_ufpumpFeedback_get_state(cl_ufpump_states* pstate);
#endif /* CL_UFPUMPFEEDBACK_PROTO_H_ */