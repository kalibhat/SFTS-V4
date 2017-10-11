/*
 * cl_heparincontroller.c
 *
 * Created: 11/5/2014 9:47:32 AM
 *  Author: user
 */ 
#include "cl_heparincontroller_include.h"
#include "cl_heparincontroller_proto.h"

extern Cl_ReturnCodeType  Cl_SendDatatoconsole(Cl_ConsoleTxCommandtype , Cl_Uint8Type* ,Cl_Uint8Type );
extern  Cl_Uint32Type hep_calibration(Cl_Uint32Type inpt_ml);


// extern Cl_Uint8Type  sv_cntrl_activatepump(sv_pumptype);
// extern Cl_Uint8Type  sv_cntrl_setHepa_dir(void);
// extern Cl_Uint8Type sv_cntrl_resetHepa_dir(void);
// extern Cl_Uint8Type  sv_cntrl_deactivatepump(sv_pumptype);
// extern Cl_Uint8Type  sv_cntrl_setpumpspeed(sv_pumptype sv_pump_id,Cl_Uint32Type speed);
extern Cl_ReturnCodeType cl_hep_pumpFeedback_get_state(cl_hep_pump_states* pstate);
extern Cl_ReturnCodeType cl_hep_pumpFeedback_get_delta(int16_t* pdelta);
extern Cl_ReturnCodeType cl_hep_pumpFeedback_stop(void);
extern Cl_ReturnCodeType cl_wait(Cl_Uint32Type );
//extern Cl_Uint32Type hep_calibration(int32_t data);
//extern Cl_Uint32Type rin_calibration(int32_t data);
cl_hep_states hp_State = CL_HEP_STATE_IDLE;

static Cl_Uint16Type cl_hep_bolusrate;
static Cl_Uint32Type cl_hep_deliveryrate = 100;
static Cl_Uint16Type cl_hep_seconds = 0;
static Cl_Uint16Type cl_hep_minutes = 0;
static Cl_Uint16Type cl_hep_100mseconds = 0;
static Cl_Uint16Type CL_hep_bolustime = 0;
extern  Cl_Uint16Type hep_rotation_counter = 0;

Cl_ReturnCodeType cl_hep_start_heparindelivery(void)
{
	return CL_OK;
}
Cl_ReturnCodeType cl_hep_set_heparin_deliveryrate(void)
{
	return CL_OK;
}
Cl_ReturnCodeType cl_hep_start_heparin_bolus(void)
{
	return CL_OK;
}

Cl_ReturnCodeType cl_hep_set_heparin_bolusrate(void)
{
	return CL_OK;
}
Cl_ReturnCodeType cl_hep_stop_heparindelivery(void)
{
	return CL_OK;
}
Cl_ReturnCodeType cl_hep_set_heparindeliveryperiod(void)
{
	return CL_OK;
}
Cl_ReturnCodeType cl_hep_set_heparinbolusperiod(void)
{
	return CL_OK;
}

Cl_ReturnCodeType cl_hep_controller(cl_hep_events hp_event , int32_t data)
{
	Cl_ReturnCodeType cl_hep_retcode = CL_OK;
		cl_hep_pump_states hep_pumpstate = CL_HEP_STATE_IDLE;
		int16_t hep_delta_value = 0 ;
		static Cl_Uint32Type pwm_period = 0;
	    static Cl_Uint32Type pwm_count = 0;
	switch(hp_State)
	{	
		case	CL_HEP_STATE_IDLE:
		switch(hp_event)
			{
				
				case CL_HEP_EVENT_STARTBOLUS:
					pwm_period =  cl_hep_deliveryrate ;
					pwm_period = 2;
				//	cl_wait(100);
					
				//	cl_wait(100);
					sv_cntrl_activatepump(HEPARINPUMP);
					sv_cntrl_setHepa_dir();
					cl_wait(100);
					sv_cntrl_setpumpspeed(HEPARINPUMP,pwm_period);
	//				cl_hep_bolus_count_rate = cl_hep_bolus_count_rate;//300;
				//	cl_hep_pumpFeedback_start();
				    cl_hep_bolus_start();
				hp_State = CL_HEP_STATE_BOLUS_ON;
				break;
				case CL_HEP_EVENT_STOPBOLUS:
					cl_hep_retcode =  sv_cntrl_deactivatepump(HEPARINPUMP);
					hp_State = CL_HEP_STATE_IDLE;
				break;
				case CL_HEP_EVENT_START_NORMAL_DELIVERY:
						//pwm_period = Treatdata[ID_heprate] ;
							pwm_period = hep_calibration(data);
							
						//	cl_wait(100);
						//	pwm_count = rin_calibration(data);
						
						
						sv_cntrl_setHepa_dir();
						sv_cntrl_activatepump(HEPARINPUMP);
					//	pwm_period = 10 ;
						cl_wait(100);
						sv_cntrl_setpumpspeed(HEPARINPUMP,pwm_period);
					//	cl_hep_pumpFeedback_start();
					//	cl_wait(100);
						
					//	cl_wait(100);
						
					//	cl_hep_pumpFeedback_set_expected_period(cl_hep_deliveryrate);
						cl_hep_pumpFeedback_start();
						Cl_AlarmActivateAlarms(HP_OVERRUN,true);
						Cl_AlarmActivateAlarms(HP_UNDERRUN,true);

				hp_State = CL_HEP_STATE_NORMAL_DELIVERY_ON;

				break;
				
				case CL_HEP_EVENT_STOP_NORMAL_DELIVERY:
				sv_cntrl_deactivatepump(HEPARINPUMP);
				Cl_AlarmActivateAlarms(HP_OVERRUN,false);
				Cl_AlarmActivateAlarms(HP_UNDERRUN,false);
				hp_State =  CL_HEP_STATE_IDLE;
				break;
				case CL_HEP_EVENT_GO_TO_LEFTEND:
				pwm_period = 2 ;
				
				sv_cntrl_setHepa_dir();
				sv_cntrl_activatepump(HEPARINPUMP);
				//cl_hep_pumpFeedback_start();
				cl_wait(100);
				Cl_AlarmActivateAlarms(HP_OVERRUN,false);
				Cl_AlarmActivateAlarms(HP_UNDERRUN,false);
				sv_cntrl_setpumpspeed(HEPARINPUMP,pwm_period);
				hp_State = CL_HEP_STATE_MOVING_LEFT;
				break;
				case CL_HEP_EVENT_GO_TO_RIGHTEND:
				pwm_period = 2 ;
				
					sv_cntrl_resetHepa_dir();
					sv_cntrl_activatepump(HEPARINPUMP);
				//	cl_hep_pumpFeedback_start();
					cl_wait(100);
					Cl_AlarmActivateAlarms(HP_OVERRUN,false);
					Cl_AlarmActivateAlarms(HP_UNDERRUN,false);
					sv_cntrl_setpumpspeed(HEPARINPUMP,pwm_period);
				hp_State = CL_HEP_STATE_MOVING_RIGHT;
				break;
				case CL_HEP_EVENT_LEFT_END_REACHED:
				sv_cntrl_deactivatepump(HEPARINPUMP);
				Cl_AlarmActivateAlarms(HP_OVERRUN,false);
				Cl_AlarmActivateAlarms(HP_UNDERRUN,false);
				hp_State = CL_HEP_STATE_IDLE;
				break;
				
			
				case CL_HEP_EVENT_RIGHT_END_REACHED:
				sv_cntrl_deactivatepump(HEPARINPUMP);
				Cl_AlarmActivateAlarms(HP_OVERRUN,false);
				Cl_AlarmActivateAlarms(HP_UNDERRUN,false);
				hp_State = CL_HEP_STATE_IDLE;
				break;
// 				case CL_HEP_EVENT_SET_BOLUS_RATE:
// 				
// 				cl_hep_bolusrate = data;
// 				pwm_period =  cl_hep_bolusrate ;
// 				//sv_cntrl_setpumpspeed(HEPARINPUMP,pwm_period);
// 				break;
// 				case CL_HEP_EVENT_SET_NORMAL_DELIVERY_RATE:
// 				cl_hep_deliveryrate = data;
// 				pwm_period =  cl_hep_deliveryrate ;
// 				//sv_cntrl_setpumpspeed(HEPARINPUMP,pwm_period);
// 				
// 				break;
// 					
// 					case CL_HEP_EVENT_SET_SPEED:
// 					cl_hep_deliveryrate = data;
// 					pwm_period =  cl_hep_deliveryrate ;
// 					break;
					case CL_HEP_EVENT_AT_LEFT_END:
				//	sv_cntrl_deactivatepump(HEPARINPUMP);
					//sv_cntrl_setHepa_dir();
					//sv_cntrl_activatepump(HEPARINPUMP);
					
					
					
					break;
					
					
					
					case CL_HEP_EVENT_AT_RIGHT_END:
					
					
					//	sv_cntrl_setHepa_dir();
					//	sv_cntrl_activatepump(HEPARINPUMP);
					
					break;
			}
		break;
		case	CL_HEP_STATE_REACHED_LEFTEND:
		switch(hp_event)
		{
			
			
			case CL_HEP_EVENT_GO_TO_RIGHTEND:
			sv_cntrl_resetHepa_dir();   ///change direction 24 july
		//	sv_cntrl_setHepa_dir();
			pwm_period = 2;
			sv_cntrl_activatepump(HEPARINPUMP);
		//	cl_hep_pumpFeedback_start();
			cl_wait(100);
			sv_cntrl_setpumpspeed(HEPARINPUMP,pwm_period);
			hp_State = CL_HEP_STATE_MOVING_RIGHT;
			break;
			case CL_HEP_EVENT_AT_LEFT_END:
			case CL_HEP_EVENT_LEFT_END_REACHED:
		    sv_cntrl_deactivatepump(HEPARINPUMP);
		   hp_State = CL_HEP_STATE_IDLE;
		    break;
			case CL_HEP_EVENT_RIGHT_END_REACHED:
			sv_cntrl_deactivatepump(HEPARINPUMP);
			hp_State = CL_HEP_STATE_REACHED_RIGHTEND;
			break;
			
			case CL_HEP_EVENT_SET_BOLUS_RATE:
			cl_hep_bolusrate = data ;
			break;
			case CL_HEP_EVENT_SET_NORMAL_DELIVERY_RATE:
			cl_hep_deliveryrate = data;
			break;
		}
		break;
		case	CL_HEP_STATE_REACHED_RIGHTEND:
		switch(hp_event)
		{
// 			case CL_HEP_EVENT_SEC_TICK:
// 			hp_State = CL_HEP_STATE_IDLE;
// 			break;
// 			case CL_HEP_EVENT_STARTBOLUS:
// 			hp_State = CL_HEP_STATE_BOLUS_ON;
// 			break;

			case CL_HEP_EVENT_START_NORMAL_DELIVERY:
			hp_State =  CL_HEP_STATE_IDLE;
			//hp_State = CL_HEP_STATE_NORMAL_DELIVERY_ON;/24july
			break;
			case CL_HEP_EVENT_GO_TO_LEFTEND:
			sv_cntrl_setHepa_dir();     //change direction 24 july
		//	sv_cntrl_resetHepa_dir();
			sv_cntrl_activatepump(HEPARINPUMP);
		//	cl_hep_pumpFeedback_start();
			cl_wait(100);
			sv_cntrl_setpumpspeed(HEPARINPUMP,pwm_period);
			hp_State = CL_HEP_STATE_MOVING_LEFT;
			break;
			case CL_HEP_EVENT_LEFT_END_REACHED:
				sv_cntrl_deactivatepump(HEPARINPUMP);
				hp_State = CL_HEP_STATE_REACHED_LEFTEND;
				break;
				case CL_HEP_EVENT_AT_RIGHT_END:
			    case CL_HEP_EVENT_RIGHT_END_REACHED:
			    sv_cntrl_deactivatepump(HEPARINPUMP);
			    hp_State = CL_HEP_STATE_IDLE;
			    break;
// 			case CL_HEP_EVENT_SET_BOLUS_RATE:
// 			cl_hep_bolusrate = data ;
// 			break;
// 			case CL_HEP_EVENT_SET_NORMAL_DELIVERY_RATE:
// 			cl_hep_deliveryrate = data;
// 			break;
		}
		break;

		case CL_HEP_STATE_MOVING_RIGHT:
		switch(hp_event)
		{
			case CL_HEP_EVENT_RIGHT_END_REACHED:
			case CL_HEP_EVENT_AT_RIGHT_END:
			sv_cntrl_deactivatepump(HEPARINPUMP);
			hp_State = CL_HEP_STATE_IDLE;
			break;

			case CL_HEP_EVENT_STOP_NORMAL_DELIVERY:
			case CL_HEP_EVENT_STOPBOLUS:
			cl_hep_retcode =  sv_cntrl_deactivatepump(HEPARINPUMP);
			hp_State = CL_HEP_STATE_IDLE;
			break;
			default:break;

		}
		break;
		case CL_HEP_STATE_MOVING_LEFT:
		switch(hp_event)
		{
// 			case CL_HEP_EVENT_SEC_TICK:
// 			cl_hep_pumpFeedback_get_state(&hep_pumpstate);
// 			switch(hep_pumpstate)
// 			{
// 				case CL_HEP_P_STATE_READY_AT_START:
// 		////		sv_cntrl_setHepa_dir();
// 			//	sv_cntrl_deactivatepump(HEPARINPUMP);
// 			//	hp_State = CL_HEP_STATE_REACHED_LEFTEND;
// 				break;
// 			}
// 			break;
 			case CL_HEP_EVENT_LEFT_END_REACHED:
			case CL_HEP_EVENT_AT_LEFT_END:
			sv_cntrl_deactivatepump(HEPARINPUMP);
			hp_State = CL_HEP_STATE_IDLE;
			break;
			case CL_HEP_EVENT_STOP_NORMAL_DELIVERY:
			case CL_HEP_EVENT_STOPBOLUS:
			cl_hep_retcode =  sv_cntrl_deactivatepump(HEPARINPUMP);
			hp_State = CL_HEP_STATE_IDLE;
			break;
			default:break;

		}
		break;
		case	CL_HEP_STATE_READY_FORBOLUS:
		switch(hp_event)
		{
			case CL_HEP_EVENT_STARTBOLUS:
			hp_State = CL_HEP_STATE_BOLUS_ON;
			break;
			case CL_HEP_EVENT_GO_TO_LEFTEND:
			hp_State = CL_HEP_STATE_MOVING_LEFT;
			break;
			case CL_HEP_EVENT_GO_TO_RIGHTEND:
			hp_State = CL_HEP_STATE_MOVING_RIGHT;
			break;
			case CL_HEP_EVENT_LEFT_END_REACHED:
			hp_State = CL_HEP_STATE_REACHED_LEFTEND;
			break;
			case CL_HEP_EVENT_RIGHT_END_REACHED:
			hp_State = CL_HEP_STATE_REACHED_RIGHTEND;
			break;
			case CL_HEP_EVENT_SET_BOLUS_RATE:
			cl_hep_bolusrate = data ;
			break;
			case CL_HEP_EVENT_SET_NORMAL_DELIVERY_RATE:
			cl_hep_deliveryrate = data;
			break;
		}
		break;
		case	CL_HEP_STATE_BOLUS_ON:
		switch(hp_event)
		{
	//	case CL_HEP_EVENT_SEC_TICK:
			case CL_HEP_EVENT_100ms_TICK:
			cl_hep_100mseconds++;
	//		Cl_SysStat_GetSensor_Status_Query(SENSOR_HP_FB, &hep_rotataion_marker);
			
			if(cl_hep_100mseconds >=600)
			{
				//cl_hep_seconds = 0;
				//if(++cl_hep_minutes > HEPARIN_BOLUS_TIMEOUT)
			//	if(++cl_hep_seconds > HEPARIN_BOLUS_TIMEOUT)
//				{
					cl_hep_retcode =  sv_cntrl_deactivatepump(HEPARINPUMP);
					hp_State = CL_HEP_STATE_IDLE;
					cl_hep_pumpFeedback_stop();
					cl_hep_seconds = 0;
					cl_hep_100mseconds=0;
					//cl_hep_minutes = 0;
					//Cl_mac_apprequesthandler(MACREQ_HEPARIN_BOLUS_COMPLETED);
	//			}
			}

			break;
			
			case CL_HEP_EVENT_STOP_BOLUS:
			cl_hep_retcode =  sv_cntrl_deactivatepump(HEPARINPUMP);
			hp_State = CL_HEP_STATE_IDLE;
			cl_hep_pumpFeedback_stop();
			cl_hep_seconds = 0;
			cl_hep_100mseconds=0;
			break;
			
			case CL_HEP_EVENT_LEFT_END_REACHED:
			hp_State = CL_HEP_STATE_REACHED_LEFTEND;
			break;
			case CL_HEP_EVENT_RIGHT_END_REACHED:
			hp_State = CL_HEP_STATE_REACHED_RIGHTEND;
			break;
// 			case CL_HEP_EVENT_SET_BOLUS_RATE:
// 			cl_hep_bolusrate = data ;
// 			break;
// 			case CL_HEP_EVENT_SET_NORMAL_DELIVERY_RATE:
// 			cl_hep_deliveryrate = data;
// 			break;
// 			case 	CL_HEP_EVENT_PAUSEBOLUS:
// 			hp_State = CL_HEP_STATE_PAUSED;
// 			break;
// 			case CL_HEP_EVENT_STOPBOLUS:
// 			cl_hep_seconds = 0;
// 			cl_hep_retcode =  sv_cntrl_deactivatepump(HEPARINPUMP);
// 			hp_State = CL_HEP_STATE_IDLE;
// 			break;
// 			
			case CL_HEP_EVENT_STOP_NORMAL_DELIVERY:
			case	CL_HEP_EVENT_STOPBOLUS:
			cl_hep_seconds = 0;
			cl_hep_retcode =  sv_cntrl_deactivatepump(HEPARINPUMP);
			hp_State = CL_HEP_STATE_IDLE;
			break;
		}
		break;
		case	CL_HEP_STATE_READY_FOR_DELIVERY:
		break;
		case	CL_HEP_STATE_NORMAL_DELIVERY_ON:
			switch(hp_event)
			{
				case CL_HEP_EVENT_SEC_TICK:

					cl_hep_pumpFeedback_get_state(&hep_pumpstate);
					cl_hep_pumpFeedback_get_delta(&hep_delta_value);
			//		Cl_SendDatatoconsole(CON_TX_COMMAND_PRINTTEXT,"HP_RPM",6);
			//		Cl_SendDatatoconsole(CON_TX_COMMAND_PRINTDATA,&hep_delta_value,2);
					switch(hep_pumpstate)
					{
						case CL_HEP_P_STATE_RUNNING_FAST:
							//apply correction
					//		Cl_SendDatatoconsole(CON_TX_COMMAND_PRINTTEXT,"HP_FAST",7);
						//	pwm_period = pwm_period + 5;
						//	sv_cntrl_setpumpspeed(HEPARINPUMP,pwm_period );
						break;
						case CL_HEP_P_STATE_RUNNING_SLOW:
							//apply correction
						//	Cl_SendDatatoconsole(CON_TX_COMMAND_PRINTTEXT,"HP_SLOW",7);
						//	pwm_period = pwm_period - 5;
						//	sv_cntrl_setpumpspeed(HEPARINPUMP,pwm_period);
						break;
						case CL_HEP_P_STATE_STOPPED_AT_END:
							//Cl_SendDatatoconsole(CON_TX_COMMAND_PRINTTEXT,"HP_END",6);
							//sv_cntrl_deactivatepump(HEPARINPUMP);
							//hp_State = CL_HEP_STATE_REACHED_RIGHTEND;
						break;
						
						default:break;
					}


				break;
				case CL_HEP_EVENT_LEFT_END_REACHED:
				cl_hep_retcode =  sv_cntrl_deactivatepump(HEPARINPUMP);
					
				
				hp_State = CL_HEP_STATE_REACHED_LEFTEND;
				break;
				case CL_HEP_EVENT_RIGHT_END_REACHED:
				cl_hep_retcode =  sv_cntrl_deactivatepump(HEPARINPUMP);
				hp_State = CL_HEP_STATE_REACHED_RIGHTEND;
				break;
// 				case CL_HEP_EVENT_SET_BOLUS_RATE:
// 				cl_hep_bolusrate = data ;
// 				break;
 				case CL_HEP_EVENT_SET_NORMAL_DELIVERY_RATE:
 				
				 pwm_period = hep_calibration(data);
				 sv_cntrl_setHepa_dir();
				 sv_cntrl_activatepump(HEPARINPUMP);
				 cl_wait(100);
				 sv_cntrl_setpumpspeed(HEPARINPUMP,pwm_period);
				 
 				break;
// 				case 	CL_HEP_EVENT_PAUSEBOLUS:
// 				hp_State = CL_HEP_STATE_PAUSED;
// 				break;
 				case	CL_HEP_EVENT_STOPBOLUS:
				 	cl_hep_retcode =  sv_cntrl_deactivatepump(HEPARINPUMP);      //added for normal delivery stop
			//	 	hp_State = CL_HEP_STATE_IDLE;
				 
				 
 				hp_State = CL_HEP_STATE_IDLE;
 				break;
				 case CL_HEP_EVENT_STOP_NORMAL_DELIVERY:
				 	cl_hep_retcode =  sv_cntrl_deactivatepump(HEPARINPUMP); 
					hp_State = CL_HEP_STATE_IDLE;
 				break; 
				 
				 case CL_HEP_EVENT_STARTBOLUS:
				 pwm_period =  cl_hep_deliveryrate ;
				 pwm_period = 2;
				 //	cl_wait(100);
				 
				 //	cl_wait(100);
				 sv_cntrl_activatepump(HEPARINPUMP);
				 sv_cntrl_setHepa_dir();
				 cl_wait(100);
				 sv_cntrl_setpumpspeed(HEPARINPUMP,pwm_period);
				 //				cl_hep_bolus_count_rate = cl_hep_bolus_count_rate;//300;
			//	 cl_hep_pumpFeedback_start();
		    	cl_hep_bolus_start();
				 hp_State = CL_HEP_STATE_BOLUS_ON;
				 break;
			}
		
		break;
		case	CL_HEP_STATE_BOLUS_PAUSED:
		break;
		case	CL_HEP_STATE_BOLUS_STOPPED:
		break;
		case	CL_HEP_STATE_NORMAL_PAUSED:
		break;
		case	CL_HEP_STATE_NORMAL_STOPPED:
		break;
		case	CL_HEP_STATE_BOLUS_COMPLETED:
		break;
		case	CL_HEP_STATE_PAUSED:
		break;
		case	CL_HEP_STATE_STOPPED:
		break;

		case	CL_HEP_STATE_COMPLETED:
		break;
		default: break;
	}
return CL_OK;	
}