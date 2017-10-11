/*
 * cl_status_proto.h
 *
 * Created: 4/25/2017 6:39:20 PM
 *  Author: user
 */ 


#ifndef CL_STATUS_PROTO_H_
#define CL_STATUS_PROTO_H_

Cl_ReturnCodeType 		Cl_SysStat_GetSensor_Status_Query(Cl_SensorDeviceIdType , uint16_t*);
void Cl_SysStat_mean_status_update_mean_status_update(void);



#endif /* CL_STATUS_PROTO_H_ */