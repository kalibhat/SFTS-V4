/*
 * CPU2.h
 *
 * Created: 12/4/2014 10:06:11 AM
 *  Author: wid7
 */ 


#ifndef CPU2_H_
#define CPU2_H_

#include "asf.h"

typedef union
{
	uint8_t byte[2];
	uint16_t Twobyte;
}speed_data;

// typedef enum
// {
// 	START_PUMP = 0x01,
// 	STOP_PUMP = 0x02,
// 	COMPLETED = 0x03,
// 	SPEED_CHANGE = 0x04,
// 	DAC_REF = 0x05,
// 	BP_INC = 0x06,
// 	BP_DEC = 0x07,
// 	TEMP_READ = 0x08,
// 	RECOVER = 0x0A
// 	
// }cntrl_name;

typedef union
{
	uint8_t bytearray[2] ;
	uint16_t Twobyte ;
}TwobyteData;

typedef enum
{
	CPU2_STANDBY,
	CPU2_BLOODPUMP,
	CPU2_SAFETYSTATE
}cpu2_state;

typedef struct
{
	uint32_t A;
	uint32_t B;
}table;

typedef struct  
{
	uint8_t abd;
	uint8_t bd;
	uint8_t bldpump_door;
	uint8_t bldpump_sense;
	uint8_t clamp_fdbk;
	uint8_t clamp_door;
	uint8_t rdncy_irq;
	uint32_t cond1;
	uint32_t cond2;
	uint32_t temp;
	uint32_t apt;
	uint32_t vpt;
	
}cpu2_status;

void start_bloodpump(void);
void stop_bloodpump(void);
void speedchange_bloodpump(uint32_t spd_change);
void dacinput_bloodpump(uint32_t var1);
void defaultspeed_bloodpump(uint32_t speedcount);
void rampingspeed_bloodpump(uint32_t rampingspeed);
void cpu2_sensorstatus(void);
void safety_measure_activate(void);
void safety_measure_deactivate(void);
#endif /* CPU2_H_ */