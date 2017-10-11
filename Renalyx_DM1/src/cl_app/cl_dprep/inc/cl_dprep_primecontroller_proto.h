/*
 * cl_dprep_primecontroller_proto.h
 *
 * Created: 4/25/2017 4:16:21 PM
 *  Author: user
 */ 


#ifndef CL_DPREP_PRIMECONTROLLER_PROTO_H_
#define CL_DPREP_PRIMECONTROLLER_PROTO_H_


Cl_ReturnCodeType cl_dprep_primecontroller(Cl_Dprep_PrimeEvents,int16_t);
Cl_ReturnCodeType Cl_Dprep_primeUpdatePrimeTimeInfo(void);
Cl_ReturnCodeType Cl_Dprep_UpdateDialyserPrimeTimeInfo(void);
Cl_ReturnCodeType cl_dprep_activate_prime_related_alarms(void);


#endif /* CL_DPREP_PRIMECONTROLLER_PROTO_H_ */