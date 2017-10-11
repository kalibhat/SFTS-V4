/*
 * Cl_temperature_sensor.h
 *
 * Created: 4/13/2016 6:07:04 PM
 *  Author: user
 */ 


#ifndef CL_TEMPERATURE_SENSOR_H_
#define CL_TEMPERATURE_SENSOR_H_
#include "cl_types.h"

typedef enum {
	TS_EVENT_50MS,
	TS_EVENT_100MS,
	TS_EVENT_MIN,
	TS_EVENT_5SEC,
	TS_EVENT_MAX,
} Cl_Temp_EventType;

typedef enum {
	TS_STATE_IDLE,
	TS_STATE_INITIALISING,
	TS_STATE_ACTIVE,
	TS_STATE_MAX,
} Cl_Temp_StateType;
typedef enum
{
	TS1_SPI,
	TS2_SPI,
	TS3_SPI
}sensor_id;

#endif /* CL_TEMPERATURE_SENSOR_H_ */