/*
 * cl_heatercontroller_ext.h
 *
 * Created: 4/25/2017 12:39:43 PM
 *  Author: user
 */ 


#ifndef CL_HEATERCONTROLLER_EXT_H_
#define CL_HEATERCONTROLLER_EXT_H_


extern Cl_ReturnCodeType SetHeaterState(HeaterStateType Param_HeaterState);
extern Cl_ReturnCodeType Cl_Heater_GetHeaterStableStatus(Cl_BoolType* Stable);
extern Cl_ReturnCodeType Cl_Heater_Event_Handler(HeaterEventsType HeaterEvent, float);


#endif /* CL_HEATERCONTROLLER_EXT_H_ */