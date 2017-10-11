/*
 * Cl_PressureSensors_ext.h
 *
 * Created: 4/25/2017 11:55:41 AM
 *  Author: user
 */ 


#ifndef CL_PRESSURESENSORS_EXT_H_
#define CL_PRESSURESENSORS_EXT_H_

#include "Cl_PressureSensors.h"


extern Cl_ReturnCodeType  Cl_Pressure_Controller_Init(void);
extern Cl_ReturnCodeType  Cl_Pressure_Controller_Handler(Cl_PressureSensor_EventType Cl_Ps_Event);

#endif /* CL_PRESSURESENSORS_EXT_H_ */