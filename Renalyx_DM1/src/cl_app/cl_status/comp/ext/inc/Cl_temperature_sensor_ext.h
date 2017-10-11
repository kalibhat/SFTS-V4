/*
 * Cl_temperature_sensor_ext.h
 *
 * Created: 4/25/2017 11:55:54 AM
 *  Author: user
 */ 


#ifndef CL_TEMPERATURE_SENSOR_EXT_H_
#define CL_TEMPERATURE_SENSOR_EXT_H_
#include "Cl_temperature_sensor.h"

extern Cl_ReturnCodeType  Cl_Temperature_Controller_Init(void);
extern Cl_ReturnCodeType  Cl_get_Temperature_Value(sensor_id  ,float , float* );
extern Cl_ReturnCodeType  Cl_Temperature_Controller_Handler(Cl_Temp_EventType Cl_Tc_Event);
extern void res_temp_lookuptable(Cl_Uint32Type );


#endif /* CL_TEMPERATURE_SENSOR_EXT_H_ */