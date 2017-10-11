/*
 * SV_CALIBRATION.h
 *
 * Created: 9/1/2017 12:17:11 PM
 *  Author: Electrical
 */ 


#ifndef SV_CALIBRATION_H_
#define SV_CALIBRATION_H_


typedef struct
{
	float resistance;
	float temperature;
}res_lut;

void res_temp_lookuptable(float res);





#endif /* SV_CALIBRATION_H_ */