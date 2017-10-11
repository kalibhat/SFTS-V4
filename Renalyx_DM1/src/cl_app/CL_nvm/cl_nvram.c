/*
 * cl_nvram.c
 *
 * Created: 5/11/2017 12:04:43 PM
 *  Author: Gopalkrishna Kalibhat
 */ 

#include "cl_nvram.h"
#include "asf.h"
#include "cl_types.h"
#include "conf_board.h"
#include "conf_clock.h"
#include "stdio_serial.h"
#include "twi.h"
#include "tc.h"
#include "cl_app/cl_cal/calibration.h"
#include "cl_app/inc/cl_types.h"
#define NVRAM_BUS_ADDR       0x51        //!< TWI slave bus address (digital pot)
#define TWI_SPEED           100000       //!< TWI data transfer rate(100Khz)
#define PATTERN_TEST_LENGTH     sizeof(test_pattern)
#define TWI_EXAMPLE TWI0
uint32_t addr_var,size_var=0;
typedef struct
{
	ID_NAME id;
	uint32_t addr;
	uint32_t size;
}nvram_id;

nvram_id array[11]=
{
	{CAL_TEMP_TS1,0x0000,468},
	{CAL_TEMP_TS2,0x01D4,468},
	{CAL_TEMP_TS3,0x03A8,468},
	{CAL_PS1,0x057C,148},
	{CAL_PS2,0x0610,148},
	{CAL_PS3,0x06A4,148},
	{CAL_APT,0x0738,56},
	{CAL_VPT,0x0770,56},
	{CAL_COND,0x07A8,92},
	{CAL_HEP,0x0804,44},
	{CAL_CHECKSUM,0x0830,20}
};

//int16_t array_dummy[400]={0};
extern volatile temptre Temp1[117];
extern volatile temptre Temp2[117];
extern volatile temptre Temp3[117];

extern volatile lut apt1[30];
extern volatile lut vpt1[30];
extern volatile lut ps_1[50];
extern volatile lut ps_2[50];
extern volatile lut ps_3[50];
extern volatile hep hp[50];
extern volatile conduc cond[50];

uint32_t checksum=0;
uint8_t sum[11]={0};
uint8_t checksum_all[11]={0};
extern Cl_ReturnCodeType cl_wait(uint32_t );
void checksum_cal(uint32_t val);
void nvram_read(ID_NAME module_id)
{
	static temptre Temp1_1[117]= {0,0};
	static temptre Temp2_1[117]= {0,0};
	static temptre Temp3_1[117]= {0,0};

	static lut apt1_1[30]={0,0};
	static lut vpt1_1[30]={0,0};
	static lut ps_1_1[40]={0,0};
	static lut ps_2_1[40]={0,0};
	static lut ps_3_1[40]={0,0};
	static hep hp_1[40]={0,0};
	static conduc cond_1[40]={0,0};
	size_var = 0;
	addr_var = 0;
	uint32_t i=0;
	//static uint32_t j =0;
	int16_t data_received[400] = {0};
	for (i=0;i<400;i++)
	{
		data_received[i]=0;
	}
	//printf("read\n");
	addr_var = array[module_id].addr;
	size_var = array[module_id].size;
	
	twi_options_t opt = {
		.master_clk = sysclk_get_cpu_hz(),
		.speed = TWI_SPEED,
		.chip  = NVRAM_BUS_ADDR
	};

	twi_master_init(TWI_EXAMPLE, &opt);
	
	twi_packet_t packet_received = {
		#if SAM
		.addr[0]      = addr_var >> 8, // TWI slave memory address data MSB
		.addr[1]      = addr_var,      // TWI slave memory address data LSB
		.addr_length  = sizeof (uint16_t),    // TWI slave memory address data size
		#else
		.addr[0]      = addr_var,      // TWI slave memory address data MSB
		.addr_length  = sizeof (uint8_t),     // TWI slave memory address data size
		#endif
		.chip         = NVRAM_BUS_ADDR,
		// TWI slave bus address
		.buffer       = data_received,        // transfer data destination buffer
		.length       = size_var   // transfer data size (bytes)
	};
	// Perform a multi-byte read access then check the result.
	while (twi_master_read(TWI_EXAMPLE, &packet_received) != TWI_SUCCESS);

	// Verify that the received data matches the sent data.
	switch (module_id)
	{
		case CAL_TEMP_TS1:
		
		for (i=0;i<size_var/2;i++)
		{
			Temp1_1[i].volts = data_received[i*2];
			Temp1_1[i].temperature = data_received[(i*2)+1];
		}
		for (i=0;i<size_var/2;i++)
		{
			Temp1[i].volts = Temp1_1[i].volts;
			Temp1[i].temperature = Temp1_1[i].temperature;
		}
		for (i=0;i<117;i++)
		{
			Temp2[i].volts = Temp1[i].volts;
			Temp2[i].temperature = Temp1[i].temperature;
			Temp3[i].volts = Temp1[i].volts;
			Temp3[i].temperature = Temp1[i].temperature;
		}
		for (i=0;i<117;i++)
		{
			checksum = checksum+Temp1[i].volts;
			checksum = checksum+Temp1[i].temperature;
		}
		checksum_cal(checksum);
		checksum = 0;
		break;
		case CAL_PS1:
		
		for (i=0;i<size_var/2;i++)
		{
			ps_1_1[i].pressure = data_received[i*2];
			ps_1_1[i].millivolt = data_received[(i*2)+1];
		}
		for (i=0;i<40;i++)
		{
			ps_1[i].pressure = ps_1_1[i].pressure;
			ps_1[i].millivolt = ps_1_1[i].millivolt;
		}
		for (i=0;i<40;i++)
		{
			ps_2[i].pressure = ps_1[i].pressure;
			ps_2[i].millivolt = ps_1[i].millivolt;
			ps_3[i].pressure = ps_1[i].pressure;
			ps_3[i].millivolt = ps_1[i].millivolt;
			
		}
		for (i=0;i<40;i++)
		{
			checksum = checksum+ps_1[i].pressure;
			checksum = checksum+ps_1[i].millivolt;
		}
		checksum_cal(checksum);
		checksum=0;
		break;
		case CAL_APT:
		for (i=0;i<size_var/2;i++)
		{
			apt1_1[i].pressure = data_received[i*2];
			apt1_1[i].millivolt = data_received[(i*2)+1];
		}
		for (i=0;i<30;i++)
		{
			apt1[i].pressure = apt1_1[i].pressure;
			apt1[i].millivolt = apt1_1[i].millivolt;
		}
		for (i=0;i<30;i++)
		{
			vpt1[i].pressure = apt1[i].pressure;
			vpt1[i].millivolt = apt1[i].millivolt;
		}
		for (i=0;i<20;i++)
		{
			checksum = checksum+apt1[i].pressure;
			checksum = checksum+apt1[i].millivolt;
		}
		checksum_cal(checksum);
		checksum=0;
		break;
		case CAL_COND:
		for (i=0;i<size_var/2;i++)
		{
			cond_1[i].volts = data_received[i*2];
			cond_1[i].conductivity = data_received[(i*2)+1];
		}
		for (i=0;i<size_var/2;i++)
		{
			cond[i].volts = cond_1[i].volts;
			cond[i].conductivity = cond_1[i].conductivity;
		}
		for (i=0;i<50;i++)
		{
			checksum = checksum + cond[i].volts;
			checksum = checksum + cond[i].conductivity;
		}
		checksum_cal(checksum);
		checksum=0;
		break;
		case CAL_HEP:
		for (i=0;i<size_var/2;i++)
		{
			hp_1[i].ml = data_received[i*2];
			hp_1[i].speed = data_received[(i*2)+1];
		}
		for (i=0;i<size_var/2;i++)
		{
			hp[i].ml = hp_1[i].ml;
			hp[i].speed = hp_1[i].speed;
		}
		for (i=0;i<20;i++)
		{
			checksum = checksum + hp[i].ml;
			checksum = checksum + hp[i].speed;
		}
		checksum_cal(checksum);
		checksum=0;
		break;
		case CAL_CHECKSUM:
		for (i=0;i<size_var/2;i++)
		{
			checksum_all[i] = data_received[i];
		}
		break;
		default:
		break;
	}
	
}

void checksum_cal(uint32_t val)
{
	static uint8_t i=0;
	uint32_t val1, rem=0;
	val1=val;
	
	while (val1>10)
	{
		val1 = val1/10;
		rem = val1%10;
		sum[i]=sum[i]+rem;
	}
	i++;
	//printf("%d\n",sum);
}
