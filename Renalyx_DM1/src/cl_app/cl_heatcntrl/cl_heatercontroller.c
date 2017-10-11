/*
 * cl_heatercontroller.c
 *
 * Created: 10/31/2014 10:54:30 AM
 *  Author: user
 */ 

#include "cl_heatercontroller_include.h"
#include "cl_heatercontroller_proto.h"
#include "sv_interface_fun.h"

//#define HEATER_CONSTANT 1380
//#define TS2_REF 1300


extern Cl_Mac_StatesType Cl_MacState;
//extern Cl_BoolType heater_update_disble ;
extern Cl_BoolType cl_temp3_stable;
//extern Cl_BoolType cl_temp3_range_stable;
extern Cl_Uint8Type hotrinse_flag;


extern HeaterStateType HeaterStateDummy;
 HeaterStateType HeaterState = CL_HEATER_STATE_INACTIVE, HeaterPrevState = CL_HEATER_STATE_INACTIVE;
float dummy1;

Cl_Uint32Type heater_duty = 0x555;
static int16_t avg_duty = 0 ;//,fisrt_dip_duty=0, duty_min = 0,duty_max = 0,avg_duty_bak = 0,stable_avg_dty = 0,duty_temp = 0;
static HeaterSubStateType heater_substate = CL_HEATER_SUBSTATE_OFF,prev_heater_substate = CL_HEATER_SUBSTATE_OFF;
static HeaterDistributionSubStateType  DistributionSubState = CL_DISTRIB_HEATER_SUBSTATE_IDLE;
static int16_t prev_temp1val = 0, prev_temp2val = 0, prev_temp3val = 0;
static float  avgtmp3_cel = 0,prescribed_temp = 37;
float Tmp1Val=0,Tmp2Val = 0,Tmp3Val=0 ;
extern float res_temp_value;


// New definitions for Heater Control
static uint16_t count_500ms = 0,stability_count = 0,TS2_stability_count = 0, count_at_PastTS2 = 0, iterations = 0, wait_4_TS2Stability = 180, wait_4_stability = 240, post_stability_count =0, TS3_post_stability_count =0, wait_4_TS3NeuStability=120, Neupost_stability_count = 120,TS3_neustability_count=0;
 
float Tmp2Val_prev =0, Tmp3Val_prev,Past_TS2 =0,TS2_slope = 0;
float Prescribed_T2 ;
float Prescribed_T3 ;
// uint16_t SYSTEM_FLOW = 500;
 Cl_Uint32Type SYSTEM_FLOW = 300;					// Gopal:  will get the system flow rate from the user
uint8_t TS3_Direction, TS2_Direction,duty_count = 0;
extern Cl_Uint32Type Treatdata[ID_MAX_TREAT_PARAM] ;
int16_t Heat_Correction(void);
void Hot_Rinse(void);
bool  TS3_Stable, TS2_Stable;
bool flag_5min =0;

bool Check_4_TS3_Stable(void);
bool Check_4_TS2_Stable(void); 
bool Check_4_Neutralstability(void);
void Coarse_DutyCorrection(void);
void Fine_DutyCorrection(void);
void Calculate_slope (void);
void Calculate_direction(void);
void DataDisplay(void);


#define TS2_SLOPE1  0.03
#define TS2_SLOPE2  0.3
#define TS2_SLOPE3  0.5
#define TS2_SLOPE4  0.8
#define WAIT_4_TS2STABILITY_300  240
#define WAIT_4_TS2STABILITY_500  180
#define WAIT_4_TS2STABILITY_800  180

#define WAIT_4_TS3STABILITY_300  120
#define WAIT_4_TS3STABILITY_500  180
#define WAIT_4_TS3STABILITY_800  180

#define WAIT_4_STABILITY_300	 120
#define WAIT_4_STABILITY_500	 180
#define WAIT_4_STABILITY_800	 180

#define TS2_STABILITY_COUNT_300  30
#define TS2_STABILITY_COUNT_500  30
#define TS2_STABILITY_COUNT_800  30

#define TS3_STABILITY_COUNT_300  30
#define TS3_STABILITY_COUNT_500  60
#define TS3_STABILITY_COUNT_800  60

#define PRESCRIBED_TS2_300		 40
#define PRESCRIBED_TS2_500		 39
#define PRESCRIBED_TS2_800		 39

#define TEMP2_COARSE_MIN_300     37
#define TEMP2_COARSE_MAX_300     39
#define TEMP2_FINE_MIN_300
#define TEMP2_FINE_MAX_300


Cl_ReturnCodeType Cl_Heater_GetHeaterStableStatus(Cl_BoolType* Stable)
{
	if((avgtmp3_cel > prescribed_temp - 0.2) &&(avgtmp3_cel < prescribed_temp + 0.2))
	{
		*Stable =  cl_temp3_stable;
	}
	else
	{
		*Stable =  false;
	}
return CL_OK;
}

Cl_ReturnCodeType Cl_Heater_Event_Handler(HeaterEventsType HeaterEvent, float data)
{
	Cl_ReturnCodeType Cl_heeater_retcode = CL_OK;
	float cl_temp_delta = 0;

switch(HeaterEvent)
{
	
	case CL_HEATER_EVENT_500MS:
		if (hotrinse_flag != 1)
		{
			Heat_Correction();
		}
		
		else if (hotrinse_flag ==1)
		{
			
			Hot_Rinse();
			
		}
	break;
	
	case CL_HEATER_EVENT_TARGET_TEMP:
		prescribed_temp = data;
		if (hotrinse_flag == 1)
		{
			avg_duty = 4000;
		}
 	break;
	
	case CL_HEATER_EVENT_ON:
		cl_temp3_stable = false;
	break;
	
	default:break;
}
	
	return (Cl_heeater_retcode);
}

Cl_ReturnCodeType SetHeaterState(HeaterStateType Param_HeaterState)
{
	Cl_ReturnCodeType Cl_rinseretcode = CL_OK;
	int16_t sensordata;
	
	HeaterPrevState = HeaterState;
	HeaterState = Param_HeaterState;
	if(Param_HeaterState == CL_HEATER_STATE_OFF)
	{
		sv_cntrl_poweroffheater();
		heater_substate = CL_HEATER_SUBSTATE_OFF;
	}
	if(Param_HeaterState == CL_HEATER_STATE_ON)
	{
    	cl_temp3_stable = false;
		heater_substate = CL_HEATER_SUBSTATE_OFF;
	}	

	Cl_SysStat_GetSensor_Status_Query(SENSOR_TEMP1STATUS,&sensordata);

	if((Param_HeaterState != CL_HEATER_STATE_DIALYSER_PRIME_ON ) && (Param_HeaterState != CL_HEATER_STATE_DIALYSIS_ON))
	{
					
		Tmp1Val = (Tmp1Val*9+ (float)sensordata/10)/10;
	 
        SYSTEM_FLOW = Treatdata[ID_dflow];					// Gopal:  will get the system flow rate from the user
        Prescribed_T3 = Treatdata[ID_settemp]/10;				// Gopal:  will get the user set temperature to prescribed T3
 
	    if (hotrinse_flag == 1)
		{
		  avg_duty = 4000;
		}
		  
		if (SYSTEM_FLOW == 300 )                   // Gopal: if initial inlet temperature in less than 26 or flow rate is 300 then avg duty is 2400
		{
		  avg_duty = 2100;
		  wait_4_TS2Stability = WAIT_4_TS2STABILITY_300;
		  wait_4_stability = WAIT_4_TS3STABILITY_300;
		  post_stability_count = TS2_STABILITY_COUNT_300;
		  TS3_post_stability_count = TS3_STABILITY_COUNT_300;
		  Prescribed_T2 = PRESCRIBED_TS2_300;
		}
		else if (SYSTEM_FLOW == 500)
		{
		  avg_duty = 2300;
		  if (Tmp1Val <= 26)
		  {
		   avg_duty = 2400;
		  }
		  wait_4_TS2Stability = WAIT_4_TS2STABILITY_500;
		  wait_4_stability = WAIT_4_STABILITY_500;
		  post_stability_count =  TS2_STABILITY_COUNT_500;
		  TS3_post_stability_count = TS3_STABILITY_COUNT_500;
		  Prescribed_T2 = PRESCRIBED_TS2_500;
		}
		else if (SYSTEM_FLOW == 800)
		{
			avg_duty = 2300;
			if (Tmp1Val <= 26)
			{
				avg_duty = 2400;
			}
			wait_4_TS2Stability = WAIT_4_TS2STABILITY_800;
			wait_4_stability = WAIT_4_STABILITY_800;
			post_stability_count =  TS2_STABILITY_COUNT_800;
			TS3_post_stability_count = TS3_STABILITY_COUNT_800;
			Prescribed_T2 = PRESCRIBED_TS2_800;
		}
			
		avg_duty = avg_duty ;
 	}

	Cl_SysStat_GetSensor_Status_Query(SENSOR_TEMP1STATUS,&sensordata);
    {
		Tmp1Val = (Tmp1Val*9+ (float)sensordata/100)/10;
	}

	Cl_SysStat_GetSensor_Status_Query(SENSOR_TEMP2STATUS,&sensordata);
	{
	   Tmp2Val = (Tmp2Val*9+ (float)sensordata/100)/10;
  	   prev_temp2val = Tmp2Val;
	}
		 	
	Cl_SysStat_GetSensor_Status_Query(SENSOR_TEMP3STATUS,&sensordata);
	{
		Tmp3Val = (Tmp3Val*9+ (float)sensordata/100)/10;
    	prev_temp3val = Tmp3Val;
	}
		 
		 	
	if (HeaterState == CL_HEATER_STATE_CLOSED_HEATING)
	{
		avg_duty = 100;
		
	}
	if((HeaterPrevState == CL_HEATER_STATE_CLOSED_HEATING))
	{
	//	avg_duty = stable_avg_dty;
	}

	
	return (Cl_rinseretcode);
}

int16_t Heat_Correction()
{
	count_500ms ++;
    stability_count++;				//Count used to check if TS3 is stable, is being reset so we need this and cannot use count_500ms
	TS2_stability_count++;
		
	if (!(count_500ms % 600))           // wait for 30 sec for storing TS2 value, used in slope calculation
	{
		Past_TS2 = Tmp2Val;
		count_at_PastTS2 = count_500ms;  //for use in slope calculation
	}
	
	// test code to detect pattern
	int16_t sensordata;
	
	
	if(HeaterState == CL_HEATER_STATE_OFF  )
	{
		sv_cntrl_poweroffheater();
		return;
	}
	if((HeaterState == CL_HEATER_STATE_INACTIVE) || (HeaterState == CL_HEATER_SUBSTATE_OFF))
	{
		return;
	}
		
    // Sunil: We are calling power on heater every 500msec, HeaterState is set by Cl_RinseController. Should be turned ON only once?
	if((HeaterState == CL_HEATER_STATE_ON) || (HeaterState == CL_HEATER_STATE_DIALYSER_PRIME_ON) || (HeaterState == CL_HEATER_STATE_DIALYSIS_ON))
	{
	 		sv_cntrl_poweronheater();
			sv_cntrl_incheater(avg_duty);
		   
	}
	
	Cl_SysStat_GetSensor_Status_Query(SENSOR_TEMP1STATUS,&sensordata);      // sensor data of T1
	{
		Tmp1Val = (Tmp1Val*9+ (float)sensordata/100)/10;
	}
	
	if (!(count_500ms %40))    // 40 28072017
		Tmp2Val_prev = Tmp2Val;  //For calculating if TS2 stable, read once in every 2 sec
					                                             
	Cl_SysStat_GetSensor_Status_Query(SENSOR_TEMP2STATUS,&sensordata);     // sensor data of T2
	{
		Tmp2Val = (Tmp2Val*9+ (float)sensordata/100)/10;
	}
			 	
	if (!(count_500ms %20))
	{
		Tmp3Val_prev = Tmp3Val;		//Sunil: Used for checking TS3 stability
	}				 
				 
	Cl_SysStat_GetSensor_Status_Query(SENSOR_TEMP3STATUS,&sensordata);                 // sensor data of T3
	{
		Tmp3Val = (Tmp3Val*9+ (float)sensordata/100)/10;
	}
	
	DataDisplay();			 
	 	
	Calculate_slope();                     // calculate slope for TS2 curve to be used in Duty correction
	Calculate_direction();					// Calculate if temperature is raising or falling based on T2
	/* 
	Algorithm:
	- Course correction is applied based on T2 ; Fine correction is applied based on T3
	- At start keep duty cycle at 2300. On Setting Heater on and applying Duty cycle, T2 is found to increase with very less slope
	- T3 follows T2 but has a lag of certain time(230 sec). The time depends on initial T3 temperature. So time is not a factor to rely on.
	- T3 lags T2 by almost 2 degrees.
	- Allow TS2 to stabilize at certain temperature, i.e after say about 2 mins, keep checking if difference in current and 
	  previous value of TS2 is +/- 2 degrees.  Previous value is sampled at every 1 sec.
	- If stable at certain temperature, control T2 to increase till 39 deg i.e T3 at 37 degrees. Till then apply course correction
	  i.e increase/decrease duty count by large steps.T2 is monitored to be within +/- 2 degrees of 39 deg.
	- On T3 reaching 37 deg, we apply fine correction i.e by increasing/decreasing duty cycle by small steps.T3 is monitored to be within 
	  37 +/- 2 degrees of 39 deg.
	- T2 slope is used for calculating the steps.Slope = (T2(Sampled at 30 secs) - Current T2 )/ (current time - Time at sampling).
	  Higher the slope, lesser the duty count and vice versa. The duty cycle value is decreased if Temperature is increasing and 
	  increased if temperature is decreasing.
	*/
	TS2_Stable = Check_4_TS2_Stable();
	
	if (SYSTEM_FLOW == 300)
	{
		if ((Tmp2Val >= (Prescribed_T2+0.5))  && (Tmp3Val <= 32))      // Restricting forced coarse correction only at the beginning and any time TS3 falls below 32
		{
			if (!(count_500ms % 20))                  // force coarse correction if TS2 greater 39 and continues to remain for more than 1 sec
			{
				Coarse_DutyCorrection();
			}
			
		}
	}
	if (SYSTEM_FLOW == 500)
	{
		if ((Tmp2Val >= (Prescribed_T2+0.5))  && (Tmp3Val <= 32))      // Restricting forced coarse correction only at the beginning and any time TS3 falls below 32
		{
			if (!(count_500ms % 20))                  // force coarse correction if TS2 greater 39 and continues to remain for more than 1 sec
			{
				Coarse_DutyCorrection();
			}
			
		}
	}
	if (SYSTEM_FLOW == 800)
	{
		if ((Tmp2Val >= (Prescribed_T2+0.5))  && (Tmp3Val <= 32))      // Restricting forced coarse correction only at the beginning and any time TS3 falls below 32
		{
			if (!(count_500ms % 20))                  // force coarse correction if TS2 greater 39 and continues to remain for more than 1 sec
			{
				Coarse_DutyCorrection();
			}
			
		}
	}
	
	
	if (SYSTEM_FLOW == 300)
	{
	  if(TS2_Stable)
	  {
		//Now TS2 is not rapidly increasing and is at certain Temperature +/- 2 deg. 
		//Start tuning T2 to be at 37 +/- 2 deg.
		if ( ((Tmp2Val - Prescribed_T2) > 0.5) || ((Prescribed_T2 - Tmp2Val) > 2) )
		{
			Coarse_DutyCorrection(); 
		}
		else 
		{
			//T2 is stable and within 37 +/- 2 deg, check for T3 stability and apply fine correction
			TS3_Stable = Check_4_TS3_Stable();
			if(TS3_Stable)
			{
			   if ( ((Tmp3Val - Prescribed_T3) > 0.1) || ((Prescribed_T3 - Tmp3Val) > 0.1) )
			      Fine_DutyCorrection();
			   else
			   {
				 // TS2, TS3 both are stable, keep applying same avg_duty	  
			   }	
			} 
			else
			{
				//TS2 is stable, and is within 39 +/- 2 deg; TS3 is not stable yet.
				//same avg_duty is applied and wait for some more time
			}
	    }
	 }
	 else
	 {
	  // TS2 not stable, wait for some more time
	  // Same avg_duty is being applied, no change till stability not achieved
	 }	
	}
	
	if (SYSTEM_FLOW == 500)
	{
	  if(TS2_Stable)
	  {
		//Now TS2 is not rapidly increasing and is at certain Temperature +/- 2 deg. 
		//Start tuning T2 to be at 37 +/- 2 deg.
		if ( ((Tmp2Val - Prescribed_T2) > 1) || ((Prescribed_T2 - Tmp2Val) > 1) )
		{
			Coarse_DutyCorrection(); 
		}
		else 
		{
			//T2 is stable and within 37 +/- 2 deg, check for T3 stability and apply fine correction
			TS3_Stable = Check_4_TS3_Stable();
			if(TS3_Stable)
			{
			   if ( ((Tmp3Val - Prescribed_T3) > 0.1) || ((Prescribed_T3 - Tmp3Val) > 0.1) )
			      Fine_DutyCorrection();
			   else
			   {
				 // TS2, TS3 both are stable, keep applying same avg_duty	  
			   }	
			} 
			else
			{
				//TS2 is stable, and is within 39 +/- 2 deg; TS3 is not stable yet.
				//same avg_duty is applied and wait for some more time
			}
	    }
	 }
	 else
	 {
	  // TS2 not stable, wait for some more time
	  // Same avg_duty is being applied, no change till stability not achieved
	 }	
	}
	if (SYSTEM_FLOW == 800)
	{
		
		if(TS2_Stable)
		{
			//Now TS2 is not rapidly increasing and is at certain Temperature +/- 2 deg.
			//Start tuning T2 to be at 37 +/- 2 deg.
			if ( ((Tmp2Val - Prescribed_T2) > 1) || ((Prescribed_T2 - Tmp2Val) > 1) )
			{
				Coarse_DutyCorrection();
			}
			else
			{
				//T2 is stable and within 37 +/- 2 deg, check for T3 stability and apply fine correction
				TS3_Stable = Check_4_TS3_Stable();
				if(TS3_Stable)
				{
					if ( ((Tmp3Val - Prescribed_T3) > 0.1) || ((Prescribed_T3 - Tmp3Val) > 0.1) )
					Fine_DutyCorrection();
					else
					{
						// TS2, TS3 both are stable, keep applying same avg_duty
					}
				}
				else
				{
					//TS2 is stable, and is within 39 +/- 2 deg; TS3 is not stable yet.
					//same avg_duty is applied and wait for some more time
				}
			}
		}
		else
		{
			// TS2 not stable, wait for some more time
			// Same avg_duty is being applied, no change till stability not achieved
		}
	}

}

bool Check_4_TS2_Stable()
{
	bool ts2_stable = false;
	
	if (TS2_stability_count >= wait_4_TS2Stability) // Based on the profiling curve, it's assumed after wait_4_stability secs,
	// TS2 is not increasing sharply; 180 secs
	{
		if (Tmp2Val_prev == Tmp2Val)
		{
			ts2_stable = true;
			
		}
		else if(((Tmp2Val - Tmp2Val_prev) <= 2) || ((Tmp2Val_prev - Tmp2Val) <= 2))
		{
			ts2_stable = true;
		}
		wait_4_TS2Stability = post_stability_count;  // Sunil: to ensure new value if temp difference not in band and apply for next stability
													// Count 240 = 2 mins, Count 120 = 1 min
													 // Gopal: 30
		TS2_stability_count = 0;
	}
	return ts2_stable;
}

bool Check_4_TS3_Stable()
{
	bool ts3_stable = false;
	if (stability_count >= wait_4_stability) // Based on the profiling curve, it's assumed after wait_4_stability secs, 
											 // TS3 is not increasing sharply
	{
		if (Tmp3Val_prev == Tmp3Val)
		{
			ts3_stable = true;
			
		}
		else if(((Tmp3Val - Tmp3Val_prev) <= 2) || ((Tmp3Val_prev - Tmp3Val) <= 2))  
		{
			ts3_stable = true;
		}
		wait_4_stability = TS3_post_stability_count;   // Sunil: to ensure new value if temp difference not in band and apply for next stability
		                                             // Gopal: 60
								 // Count 240 = 2 mins, Count 120 = 1 min
		stability_count = 0;
	}
 return ts3_stable;
}

bool Check_4_Neutralstability()
{
		bool neuts3_stable = false;
	//wait_4_TS3NeuStability=240, Neupost_stability_count = 120,TS3_neustability_count
	
	if (TS3_neustability_count >= wait_4_TS3NeuStability) 
	{
		if (Tmp3Val_prev == Tmp3Val)
		{
			neuts3_stable = true;
			
		}
		else if(((Tmp3Val - Tmp3Val_prev) <= 0.1) || ((Tmp3Val_prev - Tmp3Val) <= 0.1))
		{
			neuts3_stable = true;
		}
		wait_4_TS3NeuStability = Neupost_stability_count;  
		TS3_neustability_count = 0;
	}
	return neuts3_stable;
}

void Calculate_slope ()
{
	TS2_slope = 0;
	
	// Since slope is calculated every 500msec, we should use 500ms_Count - count at storing Past_TS2.
	 TS2_slope = (Tmp2Val - Past_TS2) / (count_500ms - count_at_PastTS2); 
}

Calculate_direction()
{
	if((Tmp2Val - Tmp2Val_prev) > 0)
	    TS2_Direction = UP;
	else if ((Tmp2Val_prev - Tmp2Val) > 0)
	    TS2_Direction = DOWN;
	else if ((Tmp2Val_prev - Tmp2Val) == 0)
		TS2_Direction = NUETRAL;

}

Coarse_DutyCorrection()                                  // to be fine tuned based on further testing
{
	if (TS2_slope <= TS2_SLOPE1)
	{
		duty_count = 60;					//Sunil: Lesser the slope i.e rising very fast, so decrease count; Earlier value = 80, wrong

	}
	if ((TS2_slope <= TS2_SLOPE2) && (TS2_slope > TS2_SLOPE1))
	{
		duty_count = 40;

	}
	if (TS2_slope >= TS2_SLOPE2)
	{
		duty_count = 8;
	
	}
	
	if (SYSTEM_FLOW == 800)
	{
		if ((TS2_Direction == UP)  && (Tmp2Val >= (Prescribed_T2+2)) )   // if direction is upward restrict correction if TS2 not yet 39
		{													// Prescribed_T2 = 37 ; changed from 39 25071977
			
			avg_duty = avg_duty - duty_count;
			
			if((avg_duty <= 1000) )   // Gopal: added check of Ts3 to avoid avg duty setting to 1800 when TS3 is more than 37
			{
				avg_duty = 1800;    // avoiding negATIVE values for the avg duty, which shouldnt happen in regular run
			}
		}
		else if ((TS2_Direction == UP)  && ((Tmp2Val >= (Prescribed_T2+0.5)) && (Tmp3Val <= Prescribed_T3+0.1)))
		{
			avg_duty = avg_duty - 10;
			if((avg_duty <= 1000) && (Tmp3Val < Prescribed_T3) )   // Gopal: added check of Ts3 to avoid avg duty setting to 1800 when TS3 is more than 37
			{
				avg_duty = 1800;    // avoiding negATIVE values for the avg duty, which shouldnt happen in regular run
			}
		}
		else if ((TS2_Direction == UP)  && ((Tmp2Val >= (Prescribed_T2-1)) && (Tmp2Val <= Prescribed_T2+3)))
		{
			if((Tmp1Val <= 36) && (Tmp3Val < Prescribed_T3))  
				avg_duty = avg_duty + duty_count;            //To avoid TS2 getting stuck at 36 and 40
		}
		else if ((TS2_Direction == DOWN) && (Tmp2Val <= Prescribed_T2) )
		{
			avg_duty = avg_duty + duty_count;
			
			if ((avg_duty > 2300) && (Tmp3Val >= Prescribed_T3))  //Sunil limiting to 2300
			{
				avg_duty = 2300;
				sv_cntrl_poweroffheater();
				//			  HeaterState = CL_HEATER_STATE_OFF;
			}
		}
		else if (TS2_Direction == NUETRAL)    //Sunil: This means TS2 is neither increasing nor decreasing
		avg_duty = avg_duty;
		
	}

	else if (SYSTEM_FLOW == 500)
	{
		if ((TS2_Direction == UP)  && (Tmp2Val >= Prescribed_T2+2) )   // if direction is upward restrict correction if TS2 not yet 39
		{													// Prescribed_T2 = 39 
			
			avg_duty = avg_duty - duty_count;
			
			if((avg_duty <= 1000) )   // Gopal: added check of Ts3 to avoid avg duty setting to 1800 when TS3 is more than 37
			{
				avg_duty = 1800;    // avoiding negATIVE values for the avg duty, which shouldnt happen in regular run
			}
		}
		else if ((TS2_Direction == UP)  && ((Tmp2Val >= (Prescribed_T2+0.5)) && (Tmp3Val > (Prescribed_T3+0.1))))  // gOPAL: EARLIER 37 CHANGED ON 07082017
		{
			avg_duty = avg_duty - 10; 
			if((avg_duty <= 1000) && (Tmp3Val < Prescribed_T3) )   // Gopal: added check of Ts3 to avoid avg duty setting to 1800 when TS3 is more than 37
			{
				avg_duty = 1800;    // avoiding negATIVE values for the avg duty, which shouldnt happen in regular run
			}
		
		}
		else if ((TS2_Direction == UP)  && ((Tmp2Val >= (Prescribed_T2-3)) && (Tmp2Val <=(Prescribed_T2+1))))  // gOPAL: EARLIER 37 CHANGED ON 07082017
		{
			if((Tmp1Val <= 36) && (Tmp3Val < Prescribed_T3))
			{
				avg_duty = avg_duty + duty_count;            //To avoid TS2 getting stuck at 36 or 36.5
			}           //To avoid TS2 getting stuck at 36 or 36.5
		}
		else if ((TS2_Direction == DOWN) && (Tmp2Val <= Prescribed_T2-2) )  
		{
			if((Tmp1Val <= 36) && (Tmp3Val < Prescribed_T3))   //07082017 Gopal: added check of Ts3 to avoid avg duty increment when TS3 is more than 37
			{
				avg_duty = avg_duty + duty_count;            
			}
			
			if ((avg_duty > 2300) && (Tmp3Val >= Prescribed_T3))  //Sunil limiting to 2300
			{
				avg_duty = 2300;
				sv_cntrl_poweroffheater();
				//			  HeaterState = CL_HEATER_STATE_OFF;
			}
		}
		else if (TS2_Direction == NUETRAL)    //Sunil: This means TS2 is neither increasing nor decreasing
		avg_duty = avg_duty;
		
	}
	
	else if (SYSTEM_FLOW == 300)
	{
//	if ((TS2_Direction == UP)  && (Tmp2Val >= 39) && (Tmp3Val >= Prescribed_T3))   // if direction is upward restrict correction if TS2 not yet 39
		if (((TS2_Direction == UP)  && (Tmp2Val >= (Prescribed_T2+2))  ) )// || (Tmp3Val >= (Prescribed_T3+0.5)))   // if direction is upward restrict correction if TS2 not yet 39
		{                                      // earlier was 41 01082017
												// earlier 42.5 04082017			
			avg_duty = avg_duty - duty_count;
				
			if ((avg_duty <= 1000) && (Tmp3Val < Prescribed_T3) )   // Gopal: added check of Ts3 to avoid avg duty setting to 1800 when TS3 is more than 37
			{
				avg_duty = 1800;    // avoiding negATIVE values for the avg duty, which shouldnt happen in regular run
			}
		}
		else if ((TS2_Direction == UP)  && ((Tmp2Val >= (Prescribed_T2 +1)) && (Tmp3Val > (Prescribed_T3+0.1))))  // gOPAL: EARLIER 37 CHANGED ON 07082017
		{
			avg_duty = avg_duty - 10;
			if((avg_duty <= 1000) && (Tmp3Val < Prescribed_T3) )   // Gopal: added check of Ts3 to avoid avg duty setting to 1800 when TS3 is more than 37
			{
				avg_duty = 1800;    // avoiding negATIVE values for the avg duty, which shouldnt happen in regular run
			}
			
		}
		else if ((TS2_Direction == UP)  && ((Tmp2Val >= (Prescribed_T2-3)) && (Tmp2Val <= Prescribed_T2)))     
		{
			if((Tmp1Val <= 36) && (Tmp3Val < Prescribed_T3))
			{
				avg_duty = avg_duty + duty_count;            //To avoid TS2 getting stuck at 36 or 36.5
			}
			
		}
		else if ((TS2_Direction == DOWN) && (Tmp2Val <= Prescribed_T2) )        // last was 39
		{
			if((Tmp1Val <= 36) && (Tmp3Val < Prescribed_T3))   // Gopal: added check of Ts3 to avoid avg duty increment when TS3 is more than 37
			{
				avg_duty = avg_duty + duty_count;            
			}
			
			if (avg_duty > 2500)  //Sunil limiting to 2300
			{
				avg_duty = 2500;
				sv_cntrl_poweroffheater();
				//			  HeaterState = CL_HEATER_STATE_OFF;
			}
		}
		else if (TS2_Direction == NUETRAL){
			if (Check_4_Neutralstability())
			{
				if ((Tmp3Val - Prescribed_T3) >0.2)
				{
					avg_duty = avg_duty-6;
				}
				else if ((Prescribed_T3- Tmp3Val) >0.2)
				{
					avg_duty = avg_duty+6;
				}
			}
		}
	
		avg_duty = avg_duty;
		
	}
}

Fine_DutyCorrection()                                  // to be fine tuned based on further testing
{
	if (TS2_slope <= TS2_SLOPE1)
	{
		duty_count = 42;												
	}
	if ((TS2_slope <= TS2_SLOPE2) && (TS2_slope > TS2_SLOPE1))
	{
		duty_count = 20;
	}
	if (TS2_slope >= TS2_SLOPE2)
	{
		duty_count = 4;
	}
	
	if (SYSTEM_FLOW == 800)
	{
		if ((TS2_Direction == UP ) && (Tmp2Val >= (Prescribed_T2+0.5)) && (Tmp3Val >= (Prescribed_T3+0.5)))     // Gopal added TS3 check as to reduce duty if ts3 goes above 37.5
		{
			avg_duty = avg_duty - duty_count;
			
			if ((avg_duty <= 1000) )   // Gopal: added check of Ts3 to avoid avg duty setting to 1800 when TS3 is more than 37
			{
				avg_duty = 1800;    // avoiding negative values for the avg duty, which shouldn/t happen in regular run
			}
		}
		else if ((TS2_Direction == UP)  && ((Tmp2Val >= (Prescribed_T2-2)) && (Tmp2Val <= (Prescribed_T2+2.5))) && (Tmp3Val <= (Prescribed_T3+0.5)) )  // Gopal added TS3 check as to increase duty if ts3 goes below 38
		{																						// earlier value 38 chged on 07082017
			avg_duty = avg_duty + duty_count;            //To avoid TS2 getting stuck at 36 or 36.5
		}
		else if ((TS2_Direction == DOWN ) && (Tmp2Val <= (Prescribed_T2+0.5)) )
		{
		   if((Tmp1Val <= 36) && (Tmp3Val < Prescribed_T3+0.2))
			  avg_duty = avg_duty + duty_count;
		   if (avg_duty > 2300)  //Sunil limiting to 2300
			{
				avg_duty = 2300;
				sv_cntrl_poweroffheater();
				//			HeaterState = CL_HEATER_STATE_OFF;
			}
		}
		else if (TS2_Direction == NUETRAL)    //Sunil: This means TS2 is neither increasing nor decreasing
				avg_duty = avg_duty;
	}
	else if (SYSTEM_FLOW == 500)
	{
		if ((TS2_Direction == UP ) && (Tmp2Val >= (Prescribed_T2+0.1)) && (Tmp3Val >= (Prescribed_T3+0.2)))     // Gopal added TS3 check as to reduce duty if ts3 goes above 37.5
		{
			avg_duty = avg_duty - duty_count;
			
			if ((avg_duty <= 1000) )   // Gopal: added check of Ts3 to avoid avg duty setting to 1800 when TS3 is more than 37
			{
				avg_duty = 1800;    // avoiding negative values for the avg duty, which shouldn/t happen in regular run
			}
		}
		else if ((TS2_Direction == UP)  && ((Tmp2Val >= (Prescribed_T2-2)) && (Tmp2Val <= (Prescribed_T2+0.5))) && (Tmp3Val <= (Prescribed_T3-0.2)) ) 
		 // Gopal added TS3 check as to increase duty if ts3 goes below 38
		{																                          //38    changed on 07082017
			avg_duty = avg_duty + duty_count;            //To avoid TS2 getting stuck at 36 or 36.5
		}
		else if ((TS2_Direction == DOWN ) && (Tmp2Val <= (Prescribed_T2+0.5)) )  
		{
			if ((Tmp1Val <=36)  && (Tmp3Val < Prescribed_T3+0.1 )) // Gopal : added to avoid TS3 going beyond 37 when inlet temperature is above 36
			{
				avg_duty = avg_duty + duty_count;            //To avoid TS2 getting stuck at 36 or 36.5
			}
			
			if (avg_duty > 2300)  //Sunil limiting to 2300
			{
				avg_duty = 2300;
				sv_cntrl_poweroffheater();
				//			HeaterState = CL_HEATER_STATE_OFF;
			}
		}
		else if (TS2_Direction == NUETRAL)    //Sunil: This means TS2 is neither increasing nor decreasing
			{
				if (Check_4_Neutralstability())
				{
					if ((Tmp3Val - Prescribed_T3) >0.2)
					{
						avg_duty = avg_duty-6;
					}
					else if ((Prescribed_T3- Tmp3Val) >0.2)
					{
						avg_duty = avg_duty+6;
					}
				}		
			}
		
		
				avg_duty = avg_duty;
	}
	
	
	else if (SYSTEM_FLOW == 300)                                        // Gopal: if the flow rate is set to 300 temperature
	{
//		if ((TS2_Direction == UP ) && (Tmp2Val >= 39) && (Tmp3Val >= Prescribed_T3))     // Gopal: added TS3 check as to reduce duty if ts3 goes above 37.5
		if (((TS2_Direction == UP)  && (Tmp2Val >= (Prescribed_T2+1))) && (Tmp3Val >= (Prescribed_T3 + 0.1))) // Gopal: added TS3 check as to reduce duty if ts3 goes above 37.5 29072017
									     //Gopal: changed to 41 07092017 ; 
		{                                
			avg_duty = avg_duty - duty_count;
			
			if ((avg_duty <= 1000) && (Tmp3Val < Prescribed_T3) )   // Gopal: added check of Ts3 to avoid avg duty setting to 1800 when TS3 is more than 37
			{
				avg_duty = 1800;    // avoiding negative values for the avg duty, which shouldn/t happen in regular run
			}
		}
		else if ((TS2_Direction == UP)  && ((Tmp2Val >= (Prescribed_T2-2)) && (Tmp2Val <= (Prescribed_T2+1))) && (Tmp3Val < (Prescribed_T3 +0.2)))
		 // Gopal added TS3 check as to increase duty if ts3 goes below 37.2
		{                                                  // 37  41 020817  // CHANGED TO 37 FROM 38 ON 24072017 AS TS1 IS GETTING ALMOST PAR WITH TS3
			//if ((Tmp1Val <=36) && (Tmp2Val <= 39))    // Gopal : added to avoid TS3 going beyond 37 when inlet temperature is above 36
			if ((Tmp1Val <=36) )    // Gopal : added to avoid TS3 going beyond 37 when inlet temperature is above 36
			{
				avg_duty = avg_duty + duty_count;            //To avoid TS2 getting stuck at 36 or 36.5
			}
			
		}
		else if ((TS2_Direction == DOWN ) && (Tmp2Val <= (Prescribed_T2+1)) )  // 39.5
		{
//			if ((Tmp1Val <=36) && (Tmp2Val <= 39) && (Tmp3Val <= 37.5)) // Gopal : added to avoid TS3 going beyond 37 when inlet temperature is above 36
			if ((Tmp1Val <=36)  && (Tmp3Val < Prescribed_T3+0.2 )) // Gopal : added to avoid TS3 going beyond 37 when inlet temperature is above 36
			{
				avg_duty = avg_duty + duty_count;            //To avoid TS2 getting stuck at 36 or 36.5
			}
			
			if (avg_duty > 2500)  //Sunil limiting to 2300
			{
				avg_duty = 2500;
				sv_cntrl_poweroffheater();
				//			HeaterState = CL_HEATER_STATE_OFF;
			}
		}
		else if (TS2_Direction == NUETRAL)    //Sunil: This means TS2 is neither increasing nor decreasing
		{
				
			if (Check_4_Neutralstability())
			{
				if ((Tmp3Val - Prescribed_T3) >0.2)
				{
					avg_duty = avg_duty-6;
				}
				else if ((Prescribed_T3- Tmp3Val) >0.2)
				{
					avg_duty = avg_duty+6;
				}
			}
			

		}
	  avg_duty = avg_duty;
	}
	
}

DataDisplay()
{
	//************************************** display
	cl_Datastreamtype cl_tdata;
	cl_tdata.word = 0;
	//cl_tdata.Twobyte = sensordata;
	//cl_tdata.Twobyte = mid_meantemp2;
	cl_tdata.Twobyte = Tmp1Val;
	cl_tdata.bytearray[2] = 4;
	Cl_SendDatatoconsole(CON_TX_COMMAND_COMMAND_SCRIPT_PRNIT,&cl_tdata,4);
	
	cl_tdata.word = 0;
	//cl_tdata.Twobyte = sensordata;
	//cl_tdata.Twobyte = mid_meantemp2;
	cl_tdata.Twobyte = Tmp2Val;
	cl_tdata.bytearray[2] = 5;
	Cl_SendDatatoconsole(CON_TX_COMMAND_COMMAND_SCRIPT_PRNIT,&cl_tdata,4);
	
	cl_tdata.word = 0;
	//cl_tdata.Twobyte = sensordata;
	//cl_tdata.Twobyte = mid_meantemp2;
	cl_tdata.Twobyte = Tmp3Val;
	cl_tdata.bytearray[2] = 6;
	Cl_SendDatatoconsole(CON_TX_COMMAND_COMMAND_SCRIPT_PRNIT,&cl_tdata,4);
	
	cl_tdata.word = 0;
	//cl_tdata.Twobyte = sensordata;
	cl_tdata.Twobyte = avg_duty;
	cl_tdata.bytearray[2] = 7;
	Cl_SendDatatoconsole(CON_TX_COMMAND_COMMAND_SCRIPT_PRNIT,&cl_tdata,4);
	
// 	cl_tdata.word = 0;
// 	//cl_tdata.Twobyte = sensordata;
// 	cl_tdata.Twobyte = TS2_Direction;
// 	cl_tdata.bytearray[2] = 14;
// 	Cl_SendDatatoconsole(CON_TX_COMMAND_COMMAND_SCRIPT_PRNIT,&cl_tdata,4);
// 	
// 	cl_tdata.word = 0;
// 	//cl_tdata.Twobyte = sensordata;
// 	cl_tdata.Twobyte = TS2_slope * 100;
// 	cl_tdata.bytearray[2] = 15;
// 	Cl_SendDatatoconsole(CON_TX_COMMAND_COMMAND_SCRIPT_PRNIT,&cl_tdata,4);
	
//****************************************
}


void Hot_Rinse()
{
		int16_t sensordata;
		
		count_500ms ++;
		if (count_500ms > 600)
		{
			flag_5min =1;
		}
		if(HeaterState == CL_HEATER_STATE_OFF  )
		{
			sv_cntrl_poweroffheater();
			return;
		}
		if((HeaterState == CL_HEATER_STATE_INACTIVE) || (HeaterState == CL_HEATER_SUBSTATE_OFF))
		{
			return;
		}
		//avg_duty is set to value 2240 for TS3 = 37.4 at global level
		
		
		if(HeaterState == CL_HEATER_STATE_ON)
		{
			sv_cntrl_poweronheater();
			sv_cntrl_incheater(avg_duty);
			
		}
		
// 		Cl_SysStat_GetSensor_Status_Query(SENSOR_TEMP1STATUS,&sensordata);      // sensor data of T1
// 		{
// 			uint16_t cal_data  = (402 *100* sensordata)/(2*32768);             // resistance of PT100
// 			res_temp_lookuptable(cal_data);                                    // temperature from look up table in 4 digits
// 			Tmp1Val = res_temp_value/100;                                      // Temp1 value in XX.yy format
// 			Tmp1Val = Tmp1Val - 2.5 + 1.2;
// 		}
// 		
// 		if (!(count_500ms %40))
		Tmp2Val_prev = Tmp2Val;  //For calculating if TS2 stable, read once in every 2 sec
		
		Cl_SysStat_GetSensor_Status_Query(SENSOR_TEMP2STATUS,&sensordata);     // sensor data of T2
		{
// 			uint16_t cal_data  = (402 *100* sensordata)/(2*32768);			   // resistance of PT100
// 			res_temp_lookuptable(cal_data);									   // temperature from look up table in 4 digits
// 			Tmp2Val = res_temp_value/100;									   // Temp2 value in XX.yy format
// 			Tmp2Val = Tmp2Val - - 0.4;                                     // sensor offset

			Tmp2Val = (Tmp2Val*9+ (float)sensordata/100)/10;
//			Tmp2Val = (float)sensordata/100;
		}
		
		if (!(count_500ms %20))
		{
			Tmp3Val_prev = Tmp3Val;		//Sunil: Used for checking TS3 stability
		}
		
		Cl_SysStat_GetSensor_Status_Query(SENSOR_TEMP3STATUS,&sensordata);                 // sensor data of T3
		{
// 			uint16_t cal_data  = (402 *100* sensordata)/(2*32768);						// resistance of PT100
// 			res_temp_lookuptable(cal_data);									// temperature from look up table in 4 digits
// 			Tmp3Val = res_temp_value/100;									// Temp3 value in XX.yy format
// //			Tmp3Val = Tmp3Val - 3.1 + 1.4;										// sensor offset
// 			Tmp3Val = Tmp3Val - - 0.4;										// sensor offset
			Tmp3Val = (Tmp3Val*9+ (float)sensordata/100)/10;
//			Tmp3Val = (float)sensordata /100;
		}
		
	if (flag_5min)
	{
	
		
		if (!(count_500ms %30))
		{
			
		
			if ((Tmp2Val - 80) > 5 )     
			{
				avg_duty = avg_duty - 100;
				if (avg_duty <2500)
				{
					avg_duty = 2500;
				}
			
			}
			else if ((75 - Tmp2Val) > 5 && (Tmp3Val <= 70))
			{
				avg_duty = avg_duty + 80;
				if (avg_duty >3500)
				{
					avg_duty = 3500;
				}
			}
		
			else if ((Tmp3Val <= 80) && (Tmp3Val >= 70))
			{
				avg_duty = avg_duty;
			}
		
			sv_cntrl_incheater(avg_duty);
		
		}
	}
	
}






