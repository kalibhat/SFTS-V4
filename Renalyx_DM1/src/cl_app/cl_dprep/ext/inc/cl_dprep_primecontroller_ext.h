/*
 * cl_dprep_primecontroller_ext.h
 *
 * Created: 4/25/2017 4:14:40 PM
 *  Author: user
 */ 


#ifndef CL_DPREP_PRIMECONTROLLER_EXT_H_
#define CL_DPREP_PRIMECONTROLLER_EXT_H_

extern Cl_ReturnCodeType cl_dprep_primecontroller(Cl_Dprep_PrimeEvents prime_event , int16_t data);
extern Cl_ReturnCodeType cl_dprep_activate_prime_related_alarms(void);
extern Cl_ReturnCodeType cl_dprep_primeInit(void);

#endif /* CL_DPREP_PRIMECONTROLLER_EXT_H_ */