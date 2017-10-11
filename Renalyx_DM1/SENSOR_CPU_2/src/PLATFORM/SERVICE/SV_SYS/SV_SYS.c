/*
 * SV_SYS.c
 *
 * Created: 3/7/2017 4:57:12 PM
 *  Author: Electrical
 */ 
#include "SV_SYS.h"
//#include "Platform/Service/SV_SYS//SV_TIMER.h"



void SV_SYSTEM_INIT()   {
	
	SENSOR_D_INIT_CLOCK();
	SENSOR_D_INIT_UART();
	SENSOR_D_INIT_PIO();
	SENSOR_DD_INIT_SPI();
	SENSOR_DD_INIT_CAN();	
	DD_INIT_ALL_SENSORS();
}