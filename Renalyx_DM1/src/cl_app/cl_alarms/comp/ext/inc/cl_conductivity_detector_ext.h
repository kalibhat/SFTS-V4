/*
 * cl_conductivity_detector_ext.h
 *
 * Created: 4/25/2017 11:49:29 AM
 *  Author: user
 */ 


#ifndef CL_CONDUCTIVITY_DETECTOR_EXT_H_
#define CL_CONDUCTIVITY_DETECTOR_EXT_H_

#include "cl_types.h"
#include "cl_alarmdetector.h"

Cl_ReturnCodeType cl_get_conductivity_info(Cl_NewAlarmIdType* cl_alarmID);
Cl_ReturnCodeType cl_get_dac_conductivity_info(Cl_NewAlarmIdType* cl_dac_alarmID);

#endif /* CL_CONDUCTIVITY_DETECTOR_EXT_H_ */