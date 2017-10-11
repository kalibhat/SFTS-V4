/*
 * cl_status_ext.h
 *
 * Created: 4/25/2017 11:19:32 AM
 *  Author: user
 */ 


#ifndef CL_STATUS_EXT_H_
#define CL_STATUS_EXT_H_

#include "cl_alarmdetector.h"

extern Cl_ReturnCodeType 	Cl_SysStat_System_Status_Query(void);
extern Cl_ReturnCodeType	Cl_SysStat_GetSensor_Status_Query(Cl_SensorDeviceIdType, uint16_t*);
extern void Cl_SysStat_mean_status_update(void);


#endif /* CL_STATUS_EXT_H_ */