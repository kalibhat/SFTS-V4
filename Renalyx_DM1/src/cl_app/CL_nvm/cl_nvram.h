/*
 * cl_nvram.h
 *
 * Created: 5/11/2017 12:05:02 PM
 *  Author: intel
 */ 


#ifndef CL_NVRAM_H_
#define CL_NVRAM_H_


typedef struct
{
	short ml;
	short duty_value;
	short fdbck;
}fb;

typedef struct
{
	short pressure;
	 short millivolt;
}lut;

typedef struct
{
	unsigned short volts;
	unsigned short temperature;
}temptre;

typedef struct
{
	unsigned short volts;
	unsigned short conductivity;
}conduc;

typedef struct
{
	unsigned short ml;
	unsigned short speed;
	int fblower;
	int fbupper;
}hep;

typedef struct
{
	unsigned short rate;
	unsigned short speed;
}uf;

typedef enum
{
	CAL_TEMP_TS1,
	CAL_TEMP_TS2,
	CAL_TEMP_TS3,
	CAL_PS1,
	CAL_PS2,
	CAL_PS3,
	CAL_APT,
	CAL_VPT,
	CAL_COND,
	CAL_HEP,
	CAL_CHECKSUM
	
}ID_NAME;

void nvram_read(ID_NAME module_id);


#endif /* CL_NVRAM_H_ */

