/*
 * STATUS.h
 *
 * Created: 23/5/2017 2:15:32 PM
 *  Author: Gopalkrishna Kalibhat
 */ 


#ifndef STATUS_H_
#define STATUS_H_

#include "asf.h"
#include "Platform/Service/CPU2_S_interface.h"
#include "Platform/Driver/Driver_calls.h"
//#include "Platform/Driver/CPU2_DHANDLER/HANDLER.h"
uint16_t cpu2_sv_status_getsystemstatus(Sys_statusType* sensor_struct);
void SV_SEND_CAN_MAILBOX(DD_CAN_MAILBOX_SENSOR_DATA_REG_TYPE*  SV_CAN_MAILBOX_SENSOR_DATA_STRUCTURE);
bool SV_put_sensor_data(Sys_statusType* sensor_struct);

typedef union  {
	uint32_t fourbyte[2];
	uint8_t  bytearray[8] ;
	uint16_t Twobyte[4]  ;
}sv_data_size_type;

#endif /* STATUS_H_ */