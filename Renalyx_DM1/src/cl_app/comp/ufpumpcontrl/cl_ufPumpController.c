/*
 * UFPumpController.c
 *
 * Created: 11/5/2014 3:36:33 PM
 *  Author: user
 */ 

#include "cl_ufPumpController_include.h"
#include "cl_mac_include.h"
#include "calibration.h"
#include "cl_dlsiscontroller_include.h"
#include "cl_ufPumpController_include.h"

cl_uf_states uf_state = CL_UF_STATE_IDLE;

static Cl_Uint32Type cl_uf_rate = 0;
extern Cl_Uint32Type Treatdata[ID_MAX_TREAT_PARAM] ;
extern Cl_Mac_StatesType Cl_MacState;
extern volatile Cl_Uint32Type uf_final;
extern Cl_ReturnCodeType cl_ufpumpFeedback_start(void);
extern Cl_ReturnCodeType  Cl_SendDatatoconsole(Cl_ConsoleTxCommandtype , Cl_Uint8Type* ,Cl_Uint8Type );
extern Cl_ReturnCodeType cl_ufpumpFeedback_set_expected_period(int16_t period);
extern Cl_ReturnCodeType cl_wait(Cl_Uint32Type );
int16_t intufrate2 =0;
float ufRemoved =0;
int ufTimeElapsed = 0;
int minCounter = 0 ;
extern bool isouf;
Cl_ReturnCodeType cl_uf_controller(cl_uf_events uf_event , int32_t data )
{
	Cl_ReturnCodeType cl_uf_retcode = CL_OK;
	cl_ufpump_states uf_pumpstate = CL_UFP_STATE_IDLE;
	Cl_Uint16Type uf_delta_value = 0 ;
	static Cl_Uint16Type pwm_period = 0;
	Cl_Uint32Type temp = 0;
	Cl_Uint32Type temptime = 0;
	
	switch(uf_state)
	{
			case CL_UF_STATE_IDLE:
			switch (uf_event)
				{
					case CL_UF_EVENT_SET_UF_RATE:
					
					cl_uf_rate = data;
					pwm_period =  cl_uf_rate ;
					sv_cntrl_setpumpspeed(UFPUMP,pwm_period);
					break;
					case CL_UF_EVENT_START:
				
				if(Cl_MacState == MAC_DIALYSIS)
				{
					
					sv_cntrl_setpumpspeed(UFPUMP,12000);
					cl_wait(100);
					sv_cntrl_setpumpspeed(UFPUMP,8000);
					cl_wait(100);
					sv_cntrl_setpumpspeed(UFPUMP,6000);
					cl_wait(100);
					sv_cntrl_setpumpspeed(UFPUMP,200);
					cl_wait(100);
					
					
					 temp = Treatdata[ID_ufgoal];
					
					 temptime = Treatdata[ID_treattime]; // added to get the dialysis duration
					
					float tempUfRate = temp/((temptime)/60);
					int16_t intufrate =tempUfRate * 10;
					
					Cl_Uint32Type calcufrate = tempUfRate * 10;
					Treatdata[ID_ufrate] = calcufrate;
					calibrate_uf(intufrate);
					// based on this uf rate calculate the PWM
					// below logic needs to be changed lookup table needs to be implemented between uf rate and pwm

					if(temp > 0){
	
					
					sv_cntrl_setpumpspeed(UFPUMP,uf_final); // enter pwm data from the lookuop table
				//	cl_ufpumpFeedback_set_expected_period(uf_final); // not required for now
					cl_uf_retcode =  sv_cntrl_activatepump(UFPUMP);
					cl_ufpumpFeedback_start();
					uf_state = CL_UF_STATE_STARTED;
					}
					
				}
					else{
				
					sv_cntrl_setpumpspeed(UFPUMP,pwm_period); // enter pwm data from the lookuop table
					cl_uf_retcode =  sv_cntrl_activatepump(UFPUMP);
					cl_ufpumpFeedback_start();
					uf_state = CL_UF_STATE_STARTED;
				
						
					}
					
					break;
					
					case CL_UF_EVENT_RESET:
					
					
					 temp = Treatdata[ID_ufgoal];
					 temptime = CL_DLSIS_TIMEOUT_MIN;//Treatdata[ID_treattime]; // added to get the dialysis duration
					
					float tempUfRate2 = temp/((temptime)/60);
					intufrate2 =tempUfRate2 * 10;
					
					Cl_Uint32Type calcufrate = tempUfRate2 * 10;
					Treatdata[ID_ufrate] = calcufrate;
					
					calibrate_uf(intufrate2);
					// based on this uf rate calculate the PWM
					// below logic needs to be changed lookup table needs to be implemented between uf rate and pwm

					if(temp > 0){
						
						
						sv_cntrl_setpumpspeed(UFPUMP,uf_final); // enter pwm data from the lookuop table
						cl_ufpumpFeedback_set_expected_period(uf_final); // not required
						cl_uf_retcode =  sv_cntrl_activatepump(UFPUMP);
						cl_ufpumpFeedback_start();
						uf_state = CL_UF_STATE_STARTED;
					}
					
				
					break;
					
					default:break;
					
				}
			break;
			case CL_UF_STATE_STARTED:
			switch (uf_event)
			{
				
				case CL_UF_EVENT_MINUTE:
				
				Cl_SendUfdata();
				break;
				
				case CL_UF_EVENT_SECOND:
				//cl_ufpumpFeedback_get_state(&uf_pumpstate);
				//cl_ufpumpFeedback_get_delta(&uf_delta_value);
			
				
					ufTimeElapsed++;
			
// 					if(minCounter >= 60){
// 					
// 					minCounter = 0;
// 					}
// 					else{
// 						minCounter++;
// 					}
// 			
			
			
			//	Cl_SendDatatoconsole(CON_TX_COMMAND_PRINTTEXT,"UF_RPM",6);
			//	Cl_SendDatatoconsole(CON_TX_COMMAND_PRINTDATA,&uf_delta_value,2);

// 				if(uf_pumpstate == CL_UFP_STATE_RUNNING_FAST )
// 				{
// 					//apply correction
// 			//		Cl_SendDatatoconsole(CON_TX_COMMAND_PRINTTEXT,"UF_FAST",7);
// 				//	pwm_period = pwm_period + 100;
// 				//	sv_cntrl_setpumpspeed(UFPUMP,pwm_period );
// 				}
// 				else if (uf_pumpstate == CL_UFP_STATE_RUNNING_SLOW )
// 				{
// 					//apply correction
// 			//		Cl_SendDatatoconsole(CON_TX_COMMAND_PRINTTEXT,"UF_SLOW",7);
// 				//	pwm_period = pwm_period - 100;
// 				//	sv_cntrl_setpumpspeed(UFPUMP,pwm_period);
// 				}
// 				else
// 				{
// 					
// 				}
				break;
				
				
				case CL_UF_EVENT_SET_UF_RATE:
					cl_uf_rate = data;
					sv_cntrl_setpumpspeed(UFPUMP,data);
				break;
				case CL_UF_EVENT_PAUSE:
				break;
				case CL_UF_EVENT_STOP:
				cl_uf_retcode =  sv_cntrl_deactivatepump(UFPUMP);
				cl_uf_retcode =  cl_ufpumpFeedback_stop();
				uf_state = CL_UF_STATE_STOPPED;
				break;
				case CL_UF_EVENT_COMPLETED:
				uf_state = CL_UF_STATE_IDLE;
				break;
				case CL_UF_EVENT_RESET:				
				
				sv_cntrl_setpumpspeed(UFPUMP,12000);
				cl_wait(100);
				sv_cntrl_setpumpspeed(UFPUMP,8000);
				cl_wait(100);
				sv_cntrl_setpumpspeed(UFPUMP,6000);
				cl_wait(100);
				sv_cntrl_setpumpspeed(UFPUMP,200);
				cl_wait(100);
				
										// added this event
				
					
					
					temptime = Treatdata[ID_treattime] - ufTimeElapsed;//Treatdata[ID_treattime]; // added to get the dialysis duration	
						temp = Treatdata[ID_ufgoal] - ufRemoved;
					
					
					
					float tempUfRate = temp/((temptime)/60);
					intufrate2 =tempUfRate * 10;
					
					calibrate_uf(intufrate2);
					// based on this uf rate calculate the PWM
					// below logic needs to be changed lookup table needs to be implemented between uf rate and pwm

					if(temp > 0){
						
						
						sv_cntrl_setpumpspeed(UFPUMP,uf_final); // enter pwm data from the lookuop table
						cl_ufpumpFeedback_set_expected_period(uf_final); // not required
						cl_uf_retcode =  sv_cntrl_activatepump(UFPUMP);
						cl_ufpumpFeedback_start();
						uf_state = CL_UF_STATE_STARTED;
					}
					
				break;
				default:break;
				
			}
			break;
			case CL_UF_STATE_STOPPED:
			switch (uf_event)
			{
				case CL_UF_EVENT_SET_UF_RATE:
					cl_uf_rate = data;
					sv_cntrl_setpumpspeed(UFPUMP,data);
				break;
				case CL_UF_EVENT_START:													// added this event
				//cl_uf_retcode =  sv_cntrl_activatepump(UFPUMP);
				//uf_state = CL_UF_STATE_STARTED;
				
				sv_cntrl_setpumpspeed(UFPUMP,12000);
				cl_wait(100);
				sv_cntrl_setpumpspeed(UFPUMP,8000);
				cl_wait(100);
				sv_cntrl_setpumpspeed(UFPUMP,6000);
				cl_wait(100);
				sv_cntrl_setpumpspeed(UFPUMP,200);
				cl_wait(100);
				
			
			temp = Treatdata[ID_ufgoal] ;//- ufRemoved;
			temptime = Treatdata[ID_treattime]; //- ufTimeElapsed;//Treatdata[ID_treattime]; // added to get the dialysis duration
						
				float tempUfRate = temp/((temptime)/60);
				intufrate2 =tempUfRate * 10;
				Cl_Uint32Type calcufrate = tempUfRate * 10;
					Treatdata[ID_ufrate] = calcufrate;
				calibrate_uf(intufrate2);
				// based on this uf rate calculate the PWM
				// below logic needs to be changed lookup table needs to be implemented between uf rate and pwm

				if(temp > 0){
					
					
		//			sv_cntrl_setpumpspeed(UFPUMP,uf_final); // enter pwm data from the lookuop table
//					cl_ufpumpFeedback_set_expected_period(uf_final); // not required
					cl_uf_retcode =  sv_cntrl_activatepump(UFPUMP);
					sv_cntrl_setpumpspeed(UFPUMP,uf_final);
//					cl_ufpumpFeedback_start();
					uf_state = CL_UF_STATE_STARTED;
				}
				
				
				break;
				default:break;
				
			}
			break;
			case CL_UF_STATE_PAUSED:
			switch (uf_event)
			{
				case CL_UF_EVENT_SET_UF_RATE:
					cl_uf_rate = data;
					sv_cntrl_setpumpspeed(UFPUMP,data);
				break;
				case CL_UF_EVENT_START:
				cl_uf_retcode =  sv_cntrl_activatepump(UFPUMP);
				uf_state = CL_UF_STATE_STARTED;
				break;
				case CL_UF_EVENT_STOP:
				cl_uf_retcode =  sv_cntrl_deactivatepump(UFPUMP);
				uf_state = CL_UF_STATE_STOPPED;
				break;
				default:break;
				
			}
			break;
			case CL_UF_STATE_COMPLETED:
			break;
			
			default:break;
			
			
	}
	return CL_OK;
	
	
}


Cl_ReturnCodeType Cl_SendUfdata(){
	
	ufRemoved =  ufRemoved + (intufrate2/10);
	
	return CL_OK;
	
}

