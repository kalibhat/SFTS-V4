/*
 * calibration.h
 *
 * Created: 4/19/2016 11:11:26 AM
 *  Author: renalyx
 */ 


#ifndef CALIBRATION_H_
#define CALIBRATION_H_

// typedef struct
// {
// 	Cl_Uint32Type volts;
// 	Cl_Uint32Type temperature;
// }temptre;
// 
// typedef struct
// {
// 	Cl_Uint32Type volts;
// 	Cl_Uint32Type conductivity;
// }conduc;
// 
// typedef struct
// {
// 	Cl_Uint32Type ml;
// 	Cl_Uint32Type speed;
// }hep;

// typedef enum
// {
// 	TS1,
// 	TS2,
// 	TS3
// }temp_state;

#include "calibration_ext.h"
void calibration_apt(int16_t sensordata);
void calibration_ps1(int16_t sensordata);
void calibration_ps2(int16_t sensordata);
void calibration_ps3(int16_t sensordata);
void calibration_vpt(int16_t sensordata);
void calibrate_uf(int16_t ufrate);
void calibration_cond(float millivolts);
//void calibration_tmp(float millivolts , temp_state temp_var);
Cl_Uint32Type hep_calibration(Cl_Uint32Type inpt_ml);



#endif /* CALIBRATION_H_ */
