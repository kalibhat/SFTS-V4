/*
 * SV_STATUS.h
 *
 * Created: 3/8/2017 2:51:04 PM
 *  Author: Electrical
 */ 


#ifndef SV_STATUS_H_
#define SV_STATUS_H_

#include "asf.h"
#include "PLATFORM/SERVICE//SENSOR_SV_INTERFACE.h"
#include "PLATFORM//DRIVER/DRIVER_CALLS.h"

typedef struct {
	uint8_t  CPU_SENDER_type_reg;
	uint8_t  SENSOR_GROUP_ID_type_reg;
	uint16_t SENSOR_1;
	uint16_t SENSOR_2;
	uint16_t SENSOR_3;
} SV_CAN_MAILBOX_SENSOR_DATA_REG_TYPE;



typedef enum   {
	SENSOR_GROUP_ID_1 = 1,
	SENSOR_GROUP_ID_2,
	SENSOR_GROUP_ID_3,
	SENSOR_GROUP_ID_4,
	SENSOR_GROUP_ID_5,
	SENSOR_GROUP_ID_6,
	SENSOR_GROUP_ID_7,
	SENSOR_GROUP_ID_8,
	SENSOR_GROUP_ID_9	
}GROUPE_ID_TYPE;
typedef union  {
	uint32_t fourbyte[2];
	uint8_t  bytearray[8] ;
	uint16_t Twobyte[4]  ;
}sv_data_size_type;

//uint16_t Sensor_sv_status_getsystemstatus(SV_Sensor_status_type* sensor_struct);
void SV_get_sensor_data(SV_Sensor_status_type* sensor_struct);
bool SV_put_sensor_data(SV_Sensor_status_type* sensor_struct);
void SV_SENSOR_DATA(void);
uint16_t Temp_Averaging(uint16_t temp,uint8_t temp_ID);




#endif /* SV_STATUS_H_ */