/*
 * S_STUS.h
 *
 * Created: 12/11/2014 11:09:01 AM
 *  Author: wid7
 */ 


#ifndef S_STUS_H_
#define S_STUS_H_

#include "asf.h"
#include "Platform/Service/sv_interface.h"
#include "Platform/Driver/Driver_call.h"


uint16_t cpu3_sv_status_getsystemstatus(Sys_statusType* sensor_struct);
void SV_SEND_CAN_MAILBOX(DD_CAN_MAILBOX_SENSOR_DATA_REG_TYPE*  SV_CAN_MAILBOX_SENSOR_DATA_STRUCTURE);
bool SV_put_sensor_data(Sys_statusType* sensor_struct);

typedef union  {
	uint32_t fourbyte[2];
	uint8_t  bytearray[8] ;
	uint16_t Twobyte[4]  ;
}sv_data_size_type;


#endif /* S_STUS_H_ */