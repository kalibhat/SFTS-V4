/*
 * SV_CAN.c
 *
 * Created: 4/7/2017 4:05:27 PM
 *  Author: Gopalkrishna Kalibhat
 */ 


#include "SV_CAN.h"
#include "cl_types.h"

extern Cl_ReturnCodeType cl_wait(Cl_Uint32Type );

 volatile uint8_t array1[10] = {0,0,0,0,0,0,0,0,0};
	volatile uint32_t delay_cnt = 0;
	volatile uint32_t delay = 0;
	extern volatile uint32_t g_ul_ms_ticks ;

void SV_CAN_READ_PROPO(uint8_t PROPORTIONING_CPU_id, uint16_t data)

{
	
	
}


void SV_CAN_WRITE_PROPO(uint16_t propo_cmd, uint16_t data)

{
	 DD_CAN_WRITE_PROPO(propo_cmd, data);							// Gopal: command to send Acid Rc, Bicarb Rc and the user set Conductivity value
	
}

uint32_t SV_CAN_READ_HEMO()

{
	DD_CAN_READ_APTVPT();
	return 0;
}


void SV_CAN_WRITE_HEMO(uint8_t HEMO_CPU_id, uint16_t data, uint16_t speed)

{
	DD_CAN_WRITE_HEMO(data, speed);
	
}

void SV_CAN_READ_SENSOR(uint8_t SENSOR_CPU_id, uint16_t* data)

{
	
	
}


void SV_CAN_WRITE_SENSOR(uint8_t SENSOR_CPU_id, uint16_t* data)

{
	DD_CAN_WRITE_SENSOR(data);
	
}

void SV_CAN_REQUEST_SENSORDATA(void)
{
	
		
	delay_cnt = g_ul_ms_ticks ;
	delay = 0;	
	cl_wait(10);
	SV_CAN_WRITE_SENSOR(SENSOR_CPU_id,SB_Sensor_status_query_group1);
// 	for(int i=0; i<100000; i++)
// 	{
// 		
// 	}
	array1[1]	= 0;
	
//	while(	!array1[1]);
	cl_wait(10);

	SV_CAN_WRITE_SENSOR(SENSOR_CPU_id,SB_Sensor_status_query_group2);
// 	for(int i=0; i<100000; i++)
// 	{
// 		
// 	}

	array1[2]	= 0;
//	while(	!array1[2]);
cl_wait(10);
	SV_CAN_WRITE_SENSOR(SENSOR_CPU_id,SB_Sensor_status_query_group3);
// 	for(int i=0; i<100000; i++)
// 	{
// 		
// 	}
	array1[3]	= 0;
//	while(	!array1[3]);
cl_wait(10);
	SV_CAN_WRITE_SENSOR(SENSOR_CPU_id,SB_Sensor_status_query_group4);
// 	for(int i=0; i<100000; i++)
// 	{
// 		
// 	}
	array1[4]	= 0;
//	while(	!array1[4]);
cl_wait(10);
	SV_CAN_WRITE_SENSOR(SENSOR_CPU_id,SB_Sensor_status_query_group5);
// 	for(int i=0; i<100000; i++)
// 	{
// 		
// 	}
	array1[5]	= 0;
//	while(	!array1[5]);
cl_wait(10);
	SV_CAN_WRITE_SENSOR(SENSOR_CPU_id,SB_Sensor_status_query_group6);
// 	for(int i=0; i<100; i++)
// 	{
// 		
// 	}
// 	
cl_wait(10);
	array1[6]	= 0;
//	while(	!array1[6]);
	
	
}

void SV_INIT_CAN()
{
	 DD_INIT_CAN();
}