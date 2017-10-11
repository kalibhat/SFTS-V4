/*
 * cl_bloodpumpFeedback_proto.h
 *
 * Created: 4/25/2017 7:14:16 PM
 *  Author: user
 */ 


#ifndef CL_BLOODPUMPFEEDBACK_PROTO_H_
#define CL_BLOODPUMPFEEDBACK_PROTO_H_




Cl_ReturnCodeType cl_bp_pumpFeedback_timer(void);
Cl_ReturnCodeType cl_bp_pumpFeedback_start(void);
Cl_ReturnCodeType cl_bp_pumpFeedback_stop(void);
Cl_ReturnCodeType cl_bp_selftest_start(void);
Cl_ReturnCodeType cl_bp_selftest_stop(void);
Cl_ReturnCodeType cl_bp_pumpFeedback_get_delta(int16_t* delta);
Cl_ReturnCodeType cl_bp_pumpFeedback_get_state(cl_bp_pump_states* pstate);
Cl_ReturnCodeType cl_bp_feedback_hemo(void);


#endif /* CL_BLOODPUMPFEEDBACK_PROTO_H_ */