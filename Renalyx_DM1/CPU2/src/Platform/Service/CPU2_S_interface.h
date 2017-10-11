/*
 * CPU2_S_interface.h
 *
 * Created: 12/3/2014 3:35:55 PM
 *  Author: wid7
 */ 

#ifndef CPU2_S_INTERFACE_H_
#define CPU2_S_INTERFACE_H_

#include "asf.h"

typedef enum {
	VALVE_NULL = 0,
	HEMO_IF4 ,
	HEMO_IF3 ,
	ART_PUMP ,
	ART_VALVE ,
	CLAMP_PWR ,
	BYPS_PWR ,
	RUDNCY_INT,
	CPU_INT,
	VALVE_MAX = 10
	
}sv_valvetype;

typedef enum 
{
	PUMP_NULL,
	BLOODPUMP
	
}sv_pumptype;

typedef enum {
	SV_SENSOR_NULL = 0,
	SV_APT ,
	SV_VPT ,
	SV_SENSOR_ID3 ,
	SV_SENSOR_ID4 ,
	SV_SENSOR_ID5 
}sv_sensortype;

typedef struct{
	
// 	uint16_t cond1_status;
// 	uint16_t cond2_status;
// 	uint16_t Tempstatus;
	uint16_t aptstatus;
	uint16_t vptstatus;
	uint16_t abdstatus;
	uint16_t bdstatus;
	uint16_t bp_door;
	uint16_t bp_sense;
	uint16_t clamp_fdbk;	
	uint16_t clamp_door;
	uint16_t rudncy_irq;
}Sys_statusType;

typedef enum {
	SV_SENSIN_NULL = 0,
	SV_BP_DOOR = 1,
	SV_BP_FB = 2,
	SV_CLAMP_FB = 3,
	SV_CLAMP_DOOR = 4,
	SV_BD = 5,
	SV_ABD = 6,
	SV_REDUNDANCY_IRQ = 7,
}sv_sensintype;

#endif /* CPU2_S_INTERFACE_H_ */