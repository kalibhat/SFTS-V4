/*
 * CPU3.h
 *
 * Created: 12/9/2014 2:41:33 PM
 *  Author: wid7
 */ 


#ifndef CPU3_H_
#define CPU3_H_

// typedef enum
// {
// 	PROP_START_ALIGMENT = 0x00,
// 	PROP_START_MIXING = 0x01,
// 	PROP_STOP_MIXING = 0x02,
// 	PROP_START_ACID = 0x03,
// 	PROP_STOP_ACID = 0x04,
// 	PROP_START_OPENFILL = 0x05,
// 	PROP_STOP_OPENFILL = 0x06,
// 	PROP_COMPLETED = 0x07,
// 	PROP_START_BICARB = 0x08,
// 	PROP_STOP_BICARB = 0x09,
// 	//SPEED_CHANGE = 0x03,
// 	//DAC_REF = 0x05,
// 	PROP_RECOVER = 0x0A
// }Master_cpu_command_type;

typedef union
{
	uint8_t bytearray[2] ;
	uint16_t Twobyte ;
}TwobyteData;

typedef enum
{
	IDLE_ACID,
	IDLE_BICAR,
	IDLE_NRMAL,
	ALIGNMENT_ACID,
	ALIGNMENT_BICAR,
	CPU3_STANDBY,
	CPU3_ACID_ROSTATE,
	CPU3_BICAR_STATE,
	CPU3_MONITORING,
	CPU3_SAFETY_MEASURE
}cpu3_state;


typedef enum
{
STATE_IDLE,
STATE_OPEN_FILL,
STATE_STATE_MAX
}openfill_state;


typedef struct
{
	uint32_t dir;
	uint32_t AB;
}table;

typedef struct
{
	uint8_t rdncy_irq;
	uint32_t cond1_voltage;
	uint32_t cond1_current;
	uint32_t cond2_voltage;
	uint32_t cond2_current;
	uint32_t temp;
}cpu3_status;

#endif /* CPU3_H_ */