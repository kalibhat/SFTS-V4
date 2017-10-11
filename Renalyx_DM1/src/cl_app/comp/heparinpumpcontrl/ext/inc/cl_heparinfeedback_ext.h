/*
 * cl_heparinfeedback_ext.h
 *
 * Created: 4/25/2017 4:36:28 PM
 *  Author: user
 */ 


#ifndef CL_HEPARINFEEDBACK_EXT_H_
#define CL_HEPARINFEEDBACK_EXT_H_



extern Cl_ReturnCodeType cl_hep_pumpFeedback_timer(void);
extern Cl_ReturnCodeType cl_hep_pumpFeedback_set_expected_period(int16_t period);
extern Cl_ReturnCodeType cl_hep_pumpFeedback_start(void);
extern Cl_ReturnCodeType cl_hep_selftest_start(void);
extern Cl_ReturnCodeType cl_hep_selftest_stop(void);
extern Cl_ReturnCodeType cl_hep_bolus_start(void);
extern Cl_ReturnCodeType cl_hep_bolus_stop(void);

#endif /* CL_HEPARINFEEDBACK_EXT_H_ */