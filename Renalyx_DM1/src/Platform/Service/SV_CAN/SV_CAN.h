/*
 * SV_CAN.h
 *
 * Created: 4/7/2017 4:05:59 PM
 *  Author: Gopalkrishna Kalibhat
 */ 


#ifndef SV_CAN_H_
#define SV_CAN_H_

#include "asf.h"
#include "Platform/Drivers/DD_CAN/DD_CAN.h"

void SV_INIT_CAN();
void SV_CAN_READ_PROPO(uint8_t PROPORTIONING_CPU_id, uint16_t data);
void SV_CAN_WRITE_PROPO(uint16_t data, uint16_t rc_count);
void SV_CAN_WRITE_HEMO(uint8_t HEMO_CPU_id, uint16_t data, uint16_t speed);
uint32_t SV_CAN_READ_HEMO();
void SV_CAN_REQUEST_SENSORDATA(void);


#endif /* SV_CAN_H_ */