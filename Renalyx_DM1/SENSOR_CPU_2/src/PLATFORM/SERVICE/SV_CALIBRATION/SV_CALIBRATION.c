/*
 * SV_CALIBRATION.c
 *
 * Created: 9/1/2017 12:16:51 PM
 *  Author: Electrical
 */ 


#include "SV_CALIBRATION.h"
#include "PLATFORM/DRIVER/DD_CAN/DD_CAN.h"
#include "Platform/Driver/DRIVER_CALLS.h"


float res_temp_value=0;





res_lut  res_temp[150]=
{
	{10779,2000},{10818,2100},{10857,2200},{10896,2300},{10935,2400},{10973,2500},{11012,2600},{11051,2700},{11090,2800},{11128,2900},{11167,3000},{11206,3100},{11245,3200},{11283,3300},{11322,3400},{11361,3500},{11399,3600},{11438,3700},{11477,3800},{11515,3900},{11554,4000},{11593,4100},{11631,4200},{11670,4300},
	{11708,4400},{11747,4500},{11785,4600},{11824,4700},{11862,4800},{11901,4900},{11940,5000},{11978,5100},{12016,5200},{12055,5300},{12093,5400},{12132,5500},{12170,5600},{12209,5700},{12247,5800},{12286,5900},{12324,6000},{12362,6100},{12401,6200},
	{12439,6300},{12477,6400},{12516,6500},{12554,6600},{12592,6700},{12631,6800},{12669,6900},{12707,7000},{12745,7100},{12784,7200},{12822,7300},{12860,7400},{12898,7500},{12937,7600},{12975,7700},{13013,7800},{13051,7900},{13089,8000},{13127,8100},
	{13166,8200},{13204,8300},{13242,8400},{13280,8500},{13318,8600},{13356,8700},{13394,8800},{13432,8900},{13470,9000},{13508,9100},{13546,9200}
};
void res_temp_lookuptable(float res)
{
	int i;
	float slope=0;
	//Cl_Uint16Type tempdata=sensordata;
	for (i=0;i<150;i++)
	{
		if (res == res_temp[i].resistance)
		{
			res_temp_value=res_temp[i].temperature;
			break;
		}
		else if ((res > res_temp[i].resistance) && (res < res_temp[i+1].resistance))
		{
			slope = ((res_temp[i+1].temperature-res_temp[i].temperature)/(res_temp[i+1].resistance-res_temp[i].resistance));
			res_temp_value = slope * (res-res_temp[i].resistance) + res_temp[i].temperature;
			//sv_cntrl_setpumpspeed(HEPARINPUMP,hep_speed);
			break;
		}
		else
		{
			res_temp_value=0;
		}
	}
	
}