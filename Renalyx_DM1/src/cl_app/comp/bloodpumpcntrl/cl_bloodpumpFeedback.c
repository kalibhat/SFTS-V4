/*
 * cl_bloodpumpFeedback.c
 *
 * Created: 11/6/2014 9:06:12 AM
 *  Author: user
 */ 

#include "cl_bloodpumpFeedback_include.h"
#include "cl_bloodpumpFeedback_proto.h"

extern Cl_ReturnCodeType Cl_SysStat_GetSensor_Status_Query(Cl_SensorDeviceIdType, uint16_t*);
extern Cl_ReturnCodeType  Cl_SendDatatoconsole(Cl_ConsoleTxCommandtype , Cl_Uint8Type* ,Cl_Uint8Type );


extern uint16_t BPCanFeedBackValue;
extern uint32_t fed_value;
static Cl_Uint16Type bp_expected_period = 0;
static cl_bp_pump_states  cl_bp_pump_state = CL_BP_P_STATE_IDLE;
static Cl_Uint16Type bp_rotataion_marker = 1,prev_bp_rotataion_marker = 1;
static Cl_Uint16Type twentymscounter=0,bp_period = 0,bp_avg_period = 0, bp_freq_in_min = 0;
static Cl_Uint16Type bp_rotation_counter = 0,bp_selftest_rotation_counter = 0;
static Cl_BoolType count_state = false;


Cl_ReturnCodeType cl_bp_selftest_start(void)
{
	cl_bp_pump_state = CL_BP_P_SELFTEST_START;
	bp_periodvalue = 0;
	return 0;
}
Cl_ReturnCodeType cl_bp_selftest_stop(void)
{
	cl_bp_pump_state = CL_BP_P_STATE_IDLE;
	return 0;
}



Cl_ReturnCodeType cl_bp_pumpFeedback_start(void)
{
	cl_bp_pump_state = CL_BP_P_STATE_STARTED;
	return 0;
}
Cl_ReturnCodeType cl_bp_pumpFeedback_stop(void)
{
	
	return 0;
}
Cl_ReturnCodeType cl_bp_pumpFeedback_get_state(cl_bp_pump_states* pstate)
{
	*pstate = cl_bp_pump_state;
	return 0;
}
Cl_ReturnCodeType cl_bp_pumpFeedback_get_delta(int16_t* pdelta)
{
	*pdelta = bp_period - bp_expected_period;
	return 0;
}
Cl_ReturnCodeType cl_bp_pumpFeedback_set_expected_period(int16_t period)
{
	bp_expected_period = period;
	return 0;
}


Cl_ReturnCodeType cl_bp_feedback_hemo(void){
	
	// read expected feedback from can   fed_value 
	// read current feedback from can  BPCanFeedBackValue

if((cl_bp_pump_state == CL_BP_P_STATE_STARTED) && (cl_bp_pump_state !=CL_BP_P_SELFTEST_START))
{	
	if(BPCanFeedBackValue > (fed_value + 30)){
		
		Cl_Alarm_TriggerAlarm(BP_UNDERRUN,1);
		
	}
	else if(BPCanFeedBackValue < (fed_value - 30)){
		
		Cl_Alarm_TriggerAlarm(BP_OVERRUN,1);
	}

}
}


Cl_ReturnCodeType cl_bp_pumpFeedback_timer(void) // 5 ms clock
{
	//cl_Datastreamtype cl_data;
	//cl_testgetsensordata(BP_ROTATION_MARKER,&bp_rotataion_marker);

	if(cl_bp_pump_state != CL_BP_P_STATE_IDLE)
	{
	Cl_SysStat_GetSensor_Status_Query(SENSOR_BP_FB, &bp_rotataion_marker);

	twentymscounter++;
			if(twentymscounter > 1000)
			{
				// raise BP alarm
				Cl_Alarm_TriggerAlarm(BP_UNDERRUN,1);
			}
// 			if((cl_bp_pump_state ==  CL_BP_P_STATE_STARTED))  /* && (bp_rotation_counter > 2))*/
// 			{
// 				cl_bp_pump_state = CL_BP_P_STATE_RUNNING;
// 			}
	
		
			if(count_state == false)
			{
				if(bp_rotataion_marker == 0 )
				{
					if(prev_bp_rotataion_marker != bp_rotataion_marker)
					{
						count_state = true;
						//bp_rotation_counter++;
						twentymscounter = 0;
					}
				}
			}
			else
			{
				if(bp_rotataion_marker == 0 )
				{
					if(prev_bp_rotataion_marker != bp_rotataion_marker)
					{
						count_state = true;
						bp_rotation_counter++;
						bp_selftest_rotation_counter++;
						
						bp_period = twentymscounter*20;
						
								
								if (bp_selftest_rotation_counter >= 20)
								{
									
									bp_periodvalue = bp_period;
									bp_selftest_rotation_counter = 0;
										twentymscounter = 0;
								}
						
						
						
				
						if((cl_bp_pump_state != CL_BP_P_STATE_STARTED) && (cl_bp_pump_state !=CL_BP_P_SELFTEST_START))
						{
							if((bp_period > (1.01 *bp_expected_period)) || bp_period > 3000 )
							{
						//		cl_bp_pump_state = CL_BP_P_STATE_RUNNING_SLOW;
					//		Cl_Alarm_TriggerAlarm(BP_UNDERRUN,1);
							}
							else if(bp_period < (0.99 *bp_expected_period))
							{
						//		cl_bp_pump_state = CL_BP_P_STATE_RUNNING_FAST;
						//	Cl_Alarm_TriggerAlarm(BP_OVERRUN,1);
							}
							else
							{
						//		cl_bp_pump_state = CL_BP_P_STATE_RUNNING_NORMAL;
							}
						}

				
						bp_freq_in_min  = ( 60000) / bp_period;
						twentymscounter = 0;
				
						//	 Cl_SendDatatoconsole(CON_TX_COMMAND_PRINTTEXT,"HP_PERIOD",9);
						//	 Cl_SendDatatoconsole(CON_TX_COMMAND_PRINTDATA,&bp_period,2);
						//	 Cl_SendDatatoconsole(CON_TX_COMMAND_PRINTDATA,&bp_expected_period,2);
				
				
						//	Cl_SendDatatoconsole(CON_TX_COMMAND_PRINTTEXT,"BP_RPM",6);
						//	Cl_SendDatatoconsole(CON_TX_COMMAND_PRINTDATA,&bp_freq_in_min,2);
						//	Cl_SendDatatoconsole(CON_TX_COMMAND_PRINTDATA,&bp_rotation_counter,2);
				
						//	cl_data.Twobyte = bp_freq_in_min;
						//	dataarray[0] = CON_TX_PARAM_DATA_BP_RPM;
						//	dataarray[1] = cl_data.bytearray[0];
						//	dataarray[2] = cl_data.bytearray[1];
						//	Cl_SendDatatoconsole(CON_TX_COMMAND_SYSDATA,&dataarray,3);
				
					}
				}
		
			}
			prev_bp_rotataion_marker = bp_rotataion_marker;
	
	}
	
	return 0;
}
