/*
 * SENSOR_SV_INTERFACE.h
 *
 * Created: 3/8/2017 2:46:27 PM
 *  Author: Electrical
 */ 


#ifndef SENSOR_SV_INTERFACE_H_
#define SENSOR_SV_INTERFACE_H_

#include "asf.h"

typedef enum {
	SV_SENSOR_NULL = 0,
	SV_PS1_ID ,
	SV_PS2_ID ,
	SV_PS3_ID ,
	SV_THERMOCOUPLE_ID ,
	SV_TS1_ID,
	SV_TS2_ID ,
	SV_CS1_TS_ID ,
	SV_CS2_TS_ID,
	SV_TS3_ID ,
	SV_CS3_TS_ID ,
	SV_CS1_ID,
	SV_CS2_ID ,
	SV_CS3_ID ,
	SV_DAC1_ID ,
	SV_DAA_ID ,
	SV_DAB_ID,
	SV_DAC2_ID ,
	SV_FLOW_SENSOR_ID 		
}sv_sensortype;

typedef enum {
	CHIP_SEL_GROUP_1, 
	CHIP_SEL_GROUP_2	
	}CHIP_SEL_GROUP_TYPE;


typedef enum {
	THERMOCOUPLE_CS_ID=0,
	ISOLTR_1_CS_ID=1,
	ISOLTR_2_CS_ID,
	ISOLTR_3_CS_ID,
	ISOLTR_4_CS_ID,
	PS1_CS_N_ID,
	PS2_CS_N_ID,
	PS3_CS_N_ID,
}SPI_DECODER_MUX_ID_TYPE;

typedef struct {	
	uint16_t ps1status;
	uint16_t ps2status;        //GROUP 1
	uint16_t ps3status;
	
	uint16_t thermocouple_status;
	uint16_t Temp1status;      //GROUP 2
	uint16_t Temp2status;	
	uint16_t CS1_Tempstatus;    //GROUP 3
	uint16_t CS2_Tempstatus;		
	uint16_t Temp3status;	
	uint16_t CS3_Tempstatus;
	uint16_t CS1status;
	uint16_t CS2status;         //GROUP 4	
	uint16_t CS3status;
	uint16_t DAC1status;        //GROUP 5
	uint16_t DAAstatus;	
	uint16_t DABstatus;
	uint16_t DAC2status;         //GROUP 6
	uint16_t flow_sensor_status;
	
	//uint16_t aptstatus;
	//uint16_t vptstatus;	         //GROUP 7	
} SV_Sensor_status_type;







#endif /* SENSOR_SV_INTERFACE_H_ */