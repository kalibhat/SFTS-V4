/*
 * cl_status.h
 *
 * Created: 12/17/2015 12:33:36 PM
 *  Author: user
 */ 


#ifndef CL_STATUS_H_
#define CL_STATUS_H_

#include "cl_types.h"
#ifndef DATASTREAMTYPE
#define DATASTREAMTYPE
typedef union{
	
	Cl_Uint8Type bytearray[4] ;
	Cl_Uint16Type Twobyte  ;
	Cl_Uint32Type word ;
	
}cl_Datastreamtype;

#endif
#endif /* CL_STATUS_H_ */
