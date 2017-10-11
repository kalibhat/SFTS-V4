/*
 * cl_dprep_controller_ext.h
 *
 * Created: 4/25/2017 11:11:40 AM
 *  Author: user
 */ 


#ifndef CL_DPREP_CONTROLLER_EXT_H_
#define CL_DPREP_CONTROLLER_EXT_H_

#include "cl_types.h"

extern  Cl_ReturnCodeType		 Cl_dprep_init(void);
extern	Cl_ReturnCodeType Cl_dprep_controller(Cl_Mac_EventsType Cl_MacDprepEvent);
extern Cl_ReturnCodeType Cl_Dprep_SendPrepStateData(Cl_Console_bulkdatatype datatype);

#endif /* CL_DPREP_CONTROLLER_EXT_H_ */