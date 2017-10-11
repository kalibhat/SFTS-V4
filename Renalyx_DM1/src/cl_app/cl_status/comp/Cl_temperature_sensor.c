/*
 * Cl_temprature_sensor.c
 *
 * Created: 4/13/2016 5:24:31 PM
 *  Author: user
 */ 

#include "inc/Cl_temperature_sensor.h"
#include "cl_alarmdetector.h"
#include "cl_consolecontroller.h"
#include "calibration.h"
#include "cl_status.h"

 static Cl_Uint16Type Temp1 = 0, Temp2 = 0, Temp4=0,counter =0;
 static float Temp3=0, temp3_correcton =0 ;
extern volatile float temprature_final_value_1,temprature_final_value_2,temprature_final_value_3;
static Cl_Temp_StateType Cl_Temp_State = TS_STATE_IDLE;
extern Cl_ReturnCodeType Cl_SysStat_GetSensor_Status_Query(Cl_SensorDeviceIdType, uint16_t*);
extern Cl_ReturnCodeType  Cl_SendDatatoconsole(Cl_ConsoleTxCommandtype , Cl_Uint8Type* ,Cl_Uint8Type );
Cl_ReturnCodeType  Cl_Temperature_Controller_Handler(Cl_Temp_EventType Cl_Tc_Event);
extern void sv_spi_blood_read_temp(uint16_t* tempdata);
float res_temp_value=0;
typedef struct
{
	float resistance;
	float temperature;
}res_lut;


res_lut  res_temp[150]=
{
	{10779,2000},{10818,2100},{10857,2200},{10896,2300},{10935,2400},{10973,2500},{11012,2600},{11051,2700},{11090,2800},{11128,2900},{11167,3000},{11206,3100},{11245,3200},{11283,3300},{11322,3400},{11361,3500},{11399,3600},{11438,3700},{11477,3800},{11515,3900},{11554,4000},{11593,4100},{11631,4200},{11670,4300},
	{11708,4400},{11747,4500},{11785,4600},{11824,4700},{11862,4800},{11901,4900},{11940,5000},{11978,5100},{12016,5200},{12055,5300},{12093,5400},{12132,5500},{12170,5600},{12209,5700},{12247,5800},{12286,5900},{12324,6000},{12362,6100},{12401,6200},
	{12439,6300},{12477,6400},{12516,6500},{12554,6600},{12592,6700},{12631,6800},{12669,6900},{12707,7000},{12745,7100},{12784,7200},{12822,7300},{12860,7400},{12898,7500},{12937,7600},{12975,7700},{13013,7800},{13051,7900},{13089,8000},{13127,8100},
	{13166,8200},{13204,8300},{13242,8400},{13280,8500},{13318,8600},{13356,8700},{13394,8800},{13432,8900},{13470,9000},{13508,9100},{13546,9200}
};
void res_temp_lookuptable(Cl_Uint32Type res)
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
	}
	
}
Cl_ReturnCodeType  Cl_Temperature_Controller_Handler(Cl_Temp_EventType Cl_Tc_Event)
{
float temp=0,cal_data=0;
	Cl_ReturnCodeType cl_retval = CL_OK;
	 Cl_Uint16Type temp_val  = 0;
	Cl_Uint16Type data=0;
	if(Cl_Temp_State == TS_STATE_INITIALISING)
	{
			switch(Cl_Tc_Event)
			{
				
				case TS_EVENT_50MS:
				break;
				case TS_EVENT_100MS:
				if ( counter == 0 )
				{
					Cl_SysStat_GetSensor_Status_Query(SENSOR_TEMP1STATUS, &Temp1);
					Cl_SysStat_GetSensor_Status_Query(SENSOR_TEMP2STATUS, &Temp2);
					Cl_SysStat_GetSensor_Status_Query(SENSOR_TEMP3STATUS, &Temp3);
				}
				else
				{
					Cl_SysStat_GetSensor_Status_Query(SENSOR_TEMP1STATUS, &temp_val);
					Temp1 = ( Temp1 +temp_val)/counter+1 ;
					Cl_SysStat_GetSensor_Status_Query(SENSOR_TEMP2STATUS, &temp_val);
					Temp2 = ( Temp2 +temp_val)/counter+1 ;
					Cl_SysStat_GetSensor_Status_Query(SENSOR_TEMP3STATUS, &temp_val);
					Temp3 = ( Temp3 +temp_val)/counter+1 ;
				}

				counter++;
				if(counter > 10)
				{
					counter = 0;
					Cl_Temp_State = TS_STATE_ACTIVE;
				}
				break;
				case TS_EVENT_5SEC:
					
					sv_spi_blood_read_temp(&data);
					temp3_correcton = data - Temp3;
				break;
				default:break;
			}
	}
	else if(Cl_Temp_State == TS_STATE_ACTIVE)
	{
			switch(Cl_Tc_Event)
			{
				
				case TS_EVENT_50MS:
				break;
				case TS_EVENT_100MS:
				Cl_SysStat_GetSensor_Status_Query(SENSOR_TEMP1STATUS, &temp_val);
				
////				(402 *100* temp_val)/(2*32768);								// resistance of PT100
//				res_temp_lookuptable((402 *100* temp_val)/(2*32768));		// temperature from look up table in 4 digits
//				Temp1 = res_temp_value/100;									// Temp3 value in XX.yy format
//				Temp1 = Temp1 - 2.5 + 0.25;										// sensor offset 
				Temp1 = (float)temp_val /100;
				Temp1 = (Temp1 *9 + temp_val)/10;
				
				Cl_SysStat_GetSensor_Status_Query(SENSOR_TEMP2STATUS, &temp_val);
// 				//				(402 *100* temp_val)/(2*32768);								// resistance of PT100
// 				res_temp_lookuptable((402 *100* temp_val)/(2*32768));		// temperature from look up table in 4 digits
// 				Temp2 = res_temp_value/100;									// Temp3 value in XX.yy format
// 				Temp2 = Temp2 - 0.4;                                            // sensor offset
				Temp2 = (float)temp_val /100;
				Temp2 = (Temp2 *9 + temp_val)/10;
				
				Cl_SysStat_GetSensor_Status_Query(SENSOR_TEMP3STATUS, &temp_val);
// 																					// resistance of PT100
// 				res_temp_lookuptable((402 *100* temp_val)/(2*32768));		// temperature from look up table in 4 digits
// 				Temp3 = res_temp_value/100;									// Temp3 value in XX.yy format
// //				Temp3 = Temp3 - 3.1 + 1.4;										// sensor offset 
// 				Temp3 = Temp3 - 0.4 ;										// sensor offset 
// 				
// 				
// // 				temp = temp_val * 0.8036;
// // 				calibration_tmp(temp,TS3);
				Temp3 = (float)temp_val /100;
				Temp3 =(Temp3*5 + temprature_final_value_3)/6;
				break;
				case TS_EVENT_5SEC:
				data = 0;
				{
					float temp=0;
					Cl_Uint16Type test_data =0 ;
					
				Cl_SysStat_GetSensor_Status_Query(SENSOR_TEMP3STATUS, &data);
// 				res_temp_lookuptable((402 *100* temp_val)/(2*32768));		// temperature from look up table in 4 digits
// 				temp = res_temp_value/100;									// Temp3 value in XX.yy format
// //				temp = temp - 31 + 14;										// sensor offset
// 				temp = temp - 0.4;										// sensor offset
				temp = (float)data /100;
				cl_Datastreamtype cl_tdata;
				cl_tdata.word =0;
				cl_tdata.Twobyte = temp;
				cl_tdata.bytearray[2] = 31;
				Cl_SendDatatoconsole(CON_TX_COMMAND_COMMAND_SCRIPT_PRNIT,&cl_tdata,4);
				
				}
				
				break;
				default:break;
			}
			
	}

	
	
	return cl_retval;
}

Cl_ReturnCodeType  Cl_Temperature_Controller_Init(void)
{

	Cl_ReturnCodeType cl_retval = CL_OK;
		 Temp1 = 0;
		 Temp2 = 0;
		 Temp3 = 0;
		 Cl_Temp_State = TS_STATE_INITIALISING;
		 
		 return cl_retval;
}

Cl_ReturnCodeType  Cl_get_Temperature_Value(sensor_id id, float tempdata, float* correctedtemp)
{
	switch (id)
	{
		case TS1_SPI:
			*correctedtemp=Temp1;
		break;
		
		case TS2_SPI:
		*correctedtemp = Temp2;
		break;
		
		case TS3_SPI:
		/*
		if ((tempdata < 30) & (tempdata >45))
		{
			*correctedtemp = tempdata;
		}
		else
		{
			*correctedtemp=temp3_correcton+tempdata;
		}*/
		*correctedtemp = res_temp_value/100;
		break;
	}
return CL_OK;
}
