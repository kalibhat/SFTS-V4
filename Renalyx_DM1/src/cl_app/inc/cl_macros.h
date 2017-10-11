/*
 * cl_macros.h
 *
 * Created: 4/28/2017 12:31:45 PM
 *  Author: user
 */ 


#ifndef CL_MACROS_H_
#define CL_MACROS_H_


#define ASSERT_ERROR(X)	if ( (X) != CL_OK) { Cl_Error_Handle_Error();}


#define CL_HOLDER1OPEN 1
#define CL_HOLDER1CLOSED 0
#define CL_HOLDER2OPEN 1
#define CL_HOLDER2CLOSED 0

#define ACID_INLET_OPEN 0
#define ACID_INLET_CLOSED 1
#define BICARB_INLET_OPEN 0
#define BICARB_INLET_CLOSED 1

#endif /* CL_MACROS_H_ */