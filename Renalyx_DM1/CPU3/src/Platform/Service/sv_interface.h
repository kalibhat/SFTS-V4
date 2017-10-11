/*
 * sv_interface.h
 *
 * Created: 12/9/2014 5:44:05 PM
 *  Author: wid7
 */ 


#ifndef SV_INTERFACE_H_
#define SV_INTERFACE_H_

#include "asf.h"

typedef enum {
	VALVE_NULL = 0,
	RUDNCY_INT = 1,
	CPU_INT = 2,
	PROPO_IF3 = 3,
	PROPO_IF4 = 4,
	
}sv_valvetype;

typedef enum
{
	PUMP_NULL,
	ACID_PUMP,
	BICAR_PUMP
	
}sv_pumptype;

typedef enum {
	SV_SENSOR_NULL = 0,
	SV_SENSOR_ID1 ,
	SV_SENSOR_ID2 ,
	SV_SENSOR_ID3 ,
	SV_SENSOR_ID4 ,
	SV_SENSOR_ID5
}sv_sensortype;

typedef struct{
	
	uint16_t cond1_status_voltage;
	uint16_t cond1_status_current;
	uint16_t cond2_status_voltage;
	uint16_t cond2_status_current;
	uint16_t Tempstatus;
	uint16_t rudncy_irq;
}Sys_statusType;

typedef enum {
	SV_SENSIN_NULL = 0,
	SV_SENSIN_ID1 = 1,
	SV_SENSIN_ID2 = 2,
	SV_SENSIN_ID3 = 3,
	SV_SENSIN_ID4 = 4,
	SV_SENSIN_ID5 = 5,
	SV_SENSIN_ID6 = 6,
	SV_SENSIN_ID7 = 7,
}sv_sensintype;




#endif /* SV_INTERFACE_H_ */