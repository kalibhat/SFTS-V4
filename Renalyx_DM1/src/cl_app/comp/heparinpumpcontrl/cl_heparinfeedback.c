/*
 * cl_heparinfeedback.c
 *
 * Created: 11/6/2014 9:05:08 AM
 *  Author: user
 */ 

#include "cl_heparinfeedback_include.h"
#include "cl_heparinfeedback_proto.h"
#include "cl_heparinfeedback.h"
#include "cl_types.h"

// extern Cl_ReturnCodeType Cl_SysStat_GetSensor_Status_Query(Cl_SensorDeviceIdType, uint16_t*);
// extern Cl_ReturnCodeType  Cl_SendDatatoconsole(Cl_ConsoleTxCommandtype , Cl_Uint8Type* ,Cl_Uint8Type );

// extern Cl_Uint8Type sv_cntrl_resetHepa_dir(void);
// extern Cl_Uint8Type sv_cntrl_setHepa_dir(void);



static Cl_Uint16Type hep_expected_period = 0;
static cl_hep_pump_states  cl_hep_pump_state = CL_HEP_P_STATE_IDLE;
static Cl_Uint16Type hep_rotataion_marker = 1,prev_hep_rotataion_marker = 1 , prevhep = 1;
static Cl_Uint16Type twentymscounter=0,hep_period = 0,hep_avg_period = 0, hep_freq_in_min = 0 , temptwentycounter = 0 , newhep_period = 0;
static Cl_Uint16Type hep_rotation_counter = 0,hep_selftest_rotation_counter = 0;
static Cl_BoolType count_state = false;
Cl_Uint16Type cl_hep_bolus_count_rate =100;
static Cl_Uint16Type pin_status = 1,hp_start_status = 0,prev_hp_start_status = 0,hp_end_status = 0,prev_hp_end_status = 0;

Cl_Uint16Type temparray[35];
Cl_Uint16Type icont = 0;
extern Cl_Uint32Type hep_fblower , hep_fbupper;
//static Cl_Uint16Type counter = 0;
Cl_ReturnCodeType cl_hep_pumpFeedback_start(void)
{
	cl_hep_pump_state = CL_HEP_P_STATE_STARTED;
	hep_rotation_counter=0;
	return 0;
}


Cl_ReturnCodeType cl_hep_selftest_start(void)
{
	cl_hep_pump_state = CL_HEP_SELFTEST;
	hep_periodvalue =0;
	twentymscounter = 0;
	return 0;
}

Cl_ReturnCodeType cl_hep_selftest_stop(void)
{
	cl_hep_pump_state = CL_HEP_P_STATE_IDLE;
	return 0;
}

Cl_ReturnCodeType cl_hep_bolus_start(void)
{
	cl_hep_pump_state = CL_HEP_BOLUS;
	hep_rotation_counter=0;
	return 0;
	
}

Cl_ReturnCodeType cl_hep_bolus_stop(void)
{
	cl_hep_pump_state = CL_HEP_P_STATE_IDLE;
	return 0;
	
}
Cl_ReturnCodeType cl_hep_pumpFeedback_stop(void)
{
	
		cl_hep_pump_state = CL_HEP_P_STATE_IDLE;
	return 0;
}
Cl_ReturnCodeType cl_hep_pumpFeedback_get_state(cl_hep_pump_states* pstate)
{
	*pstate = cl_hep_pump_state;
	return 0;
}
Cl_ReturnCodeType cl_hep_pumpFeedback_get_delta(int16_t* pdelta)
{
	*pdelta = hep_period - hep_expected_period;
	return 0;
}
Cl_ReturnCodeType cl_hep_pumpFeedback_set_expected_period(int16_t period)
{
	hep_expected_period = period;
	return 0;
}

Cl_ReturnCodeType cl_hep_pumpFeedback_timer(void) // 20 ms clock
{
	
	if(cl_hep_pump_state != CL_HEP_P_STATE_IDLE)
	{
		
	
		Cl_SysStat_GetSensor_Status_Query(SENSOR_HP_FB, &hep_rotataion_marker);

		twentymscounter++;
		temptwentycounter++;
				
				if(hep_rotataion_marker == 1){
					
					if(prevhep == 0){
						
						newhep_period = temptwentycounter*100 ;
						
			/*			if(icont < 30){
							temparray[icont] = temptwentycounter*100 ;
						}
						if(icont == 5){
							
							int16_t klm = 0;
							
							for(int k =0 ; k < 30 ; k++){
								
								temparray[k] = 0;
							}
							icont = 0;
						}
						icont++;
				*/
			
							if((cl_hep_pump_state == CL_HEP_P_STATE_STARTED))
							{
								
								if(newhep_period < 200){
									
									Cl_Alarm_TriggerAlarm(HP_UNDERRUN,1);
								}
								
								if((newhep_period > hep_fbupper))
								{
									
									//	cl_hep_pump_state = CL_HEP_P_STATE_RUNNING_SLOW;
									Cl_Alarm_TriggerAlarm(HP_UNDERRUN,1);
								}
								else if((newhep_period < hep_fblower))
								{
									//	cl_hep_pump_state = CL_HEP_P_STATE_RUNNING_FAST;
									Cl_Alarm_TriggerAlarm(HP_OVERRUN,1);
								}
								else
								{
									//	cl_hep_pump_state = CL_HEP_P_STATE_STARTED;//CL_HEP_P_STATE_RUNNING_NORMAL;
								}
							}

			
			
			
						temptwentycounter = 0;
						
					}
					else{
						temptwentycounter = 0;
					}
					
				}
					
				prevhep = hep_rotataion_marker ;
				
// 		//if(twentymscounter > 10000)
// 		{
// 			Cl_Alarm_TriggerAlarm(HP_UNDERRUN,1);
// 		}
//		if((cl_hep_pump_state ==  CL_HEP_P_STATE_STARTED));           /* && (hep_rotation_counter > 5))*/
// 		{
// 			cl_hep_pump_state = CL_HEP_P_STATE_RUNNING;
// 		}
//               if( (hep_rotation_counter > 1))
// 		{
// 			cl_hep_pump_state = CL_HEP_P_STATE_RUNNING;
//  		}


		if(count_state == false)
		{
			if(hep_rotataion_marker == 0 )
			{
				if(prev_hep_rotataion_marker != hep_rotataion_marker)
				{
					count_state = true;
					//hep_rotation_counter++;
					twentymscounter = 0;
				}
			}
		}
		else
		{
			if(hep_rotataion_marker == 0 )
			{
				if(prev_hep_rotataion_marker != hep_rotataion_marker)
				{
					count_state = true;
					hep_rotation_counter++;
					
					hep_period =(twentymscounter*100);
					
					if(cl_hep_pump_state==CL_HEP_BOLUS)
					{
						
				
					if (hep_rotation_counter >= cl_hep_bolus_count_rate)
					
					{
						cl_hep_controller(CL_HEP_EVENT_STOP_BOLUS,0);
						hep_rotation_counter=0;
						hep_selftest_rotation_counter = 0;
					}
					}
					if(cl_hep_pump_state == CL_HEP_SELFTEST)
					{							
						hep_selftest_rotation_counter++;
						
						if (hep_selftest_rotation_counter >= 4)
						{
							
							hep_periodvalue = hep_period;
							hep_selftest_rotation_counter = 0;
							twentymscounter = 0;
						}
					}
					
						//Cl_SendDatatoconsole(CON_TX_COMMAND_PRINTDATA,&hep_period,2);
						//Cl_SendDatatoconsole(CON_TX_COMMAND_PRINTDATA,&hep_expected_period,2);
 
/*				
					if((cl_hep_pump_state == CL_HEP_P_STATE_STARTED) && (cl_hep_pump_state != CL_HEP_SELFTEST))
					{
					
// 					 	{
// 						 	cl_hep_pump_state = CL_HEP_P_STATE_RUNNING_NORMAL;
// 					 	}
// 						
						if(hep_period > (1.01 *hep_expected_period))
						{
							
						//	cl_hep_pump_state = CL_HEP_P_STATE_RUNNING_SLOW;
							Cl_Alarm_TriggerAlarm(HP_UNDERRUN,1);
						}
						else if(hep_period < (0.99 *hep_expected_period))
						{
						//	cl_hep_pump_state = CL_HEP_P_STATE_RUNNING_FAST;
							Cl_Alarm_TriggerAlarm(HP_OVERRUN,1);
						}
						else
						{
						//	cl_hep_pump_state = CL_HEP_P_STATE_STARTED;//CL_HEP_P_STATE_RUNNING_NORMAL;
						}
					}
*/
				
					hep_freq_in_min  = ( 60000) / hep_period;
					twentymscounter = 0;

				
				}
			}
		
		}
		prev_hep_rotataion_marker = hep_rotataion_marker;
	
	}
	
	 return 0;
}




//Cl_ReturnCodeType cl_hep_pump_enddetction_timer(void) // 20 ms clock
Cl_ReturnCodeType cl_hep_pump_enddetction_timer(void) // 100 ms clock
{
		// START END  DETECTION
		{
			Cl_SysStat_GetSensor_Status_Query(SENSOR_HP_START,&hp_start_status);
			
			if(	prev_hp_start_status != hp_start_status)
			{
				prev_hp_start_status = hp_start_status;
				if(hp_start_status == 0)
				{
						cl_hep_controller(CL_HEP_EVENT_RIGHT_END_REACHED , 0);
						
						//cl_hep_controller(CL_HEP_EVENT_LEFT_END_REACHED , 0);
						//sv_cntrl_resetHepa_dir();
						//	sv_cntrl_setHepa_dir();
					
				}
			}
				else if(hp_start_status == 0)
				{
					cl_hep_controller(CL_HEP_EVENT_AT_RIGHT_END , 0); // here the information that we are at right end is sent to the controller.
					// The controller can use this event to change its state accordingly
				}
				



			Cl_SysStat_GetSensor_Status_Query(SENSOR_HP_END,&hp_end_status);
			
			
			if(	prev_hp_end_status != hp_end_status)
			{
				prev_hp_end_status = hp_end_status;
				if(hp_end_status == 0)
				{
					cl_hep_controller(CL_HEP_EVENT_LEFT_END_REACHED , 0);
					//cl_hep_controller(CL_HEP_EVENT_RIGHT_END_REACHED , 0);
					
					//sv_cntrl_setHepa_dir();
					//	sv_cntrl_resetHepa_dir();
					
					//	Cl_SendDatatoconsole(CON_TX_COMMAND_PRINTTEXT,"HP_END",8);
					//	Cl_SendDatatoconsole(CON_TX_COMMAND_PRINTDATA,&hp_end_status,1);
				}
			}
			else if(hp_end_status == 0)
			{
				cl_hep_controller(CL_HEP_EVENT_AT_LEFT_END , 0);
				
				
			}
			
		}
			
		
		
		return 0;
}