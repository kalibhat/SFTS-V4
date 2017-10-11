/*
 * cl_bloodpumpcontroller.c
 *
 * Created: 11/6/2014 9:07:00 AM
 *  Author: user
 */ 

#include "cl_bloodpumpcontroller_include.h"
#include "cl_bloodpumpcontroller_proto.h"


cl_bp_states bp_state = CL_BP_STATE_IDLE;

static Cl_Uint32Type cl_bp_rate = 0;
static Cl_Uint16Type cl_bp_seconds = 0;
static Cl_Uint16Type cl_bp_minutes = 0;

extern Cl_Uint8Type  sv_cntrl_activatepump(sv_pumptype);
extern Cl_Uint8Type  sv_cntrl_deactivatepump(sv_pumptype);
extern Cl_Uint8Type sv_cntrl_setpumpspeed(sv_pumptype sv_pump_id,Cl_Uint32Type speed);
extern Cl_ReturnCodeType cl_bp_pumpFeedback_get_state(cl_bp_pump_states* pstate);
extern Cl_ReturnCodeType cl_bp_pumpFeedback_stop(void);
extern Cl_ReturnCodeType cl_bp_pumpFeedback_get_delta(int16_t* pdelta);
extern Cl_ReturnCodeType 		Cl_SysStat_GetSensor_Status_Query(Cl_SensorDeviceIdType dataID, uint16_t* pdata);
extern Cl_ReturnCodeType cl_bp_pumpFeedback_start(void);
extern Cl_ReturnCodeType cl_bp_selftest_start(void);
extern Cl_ReturnCodeType cl_bp_selftest_stop(void);
//extern Cl_ReturnCodeType  Cl_SendDatatoconsole(Cl_ConsoleTxCommandtype , Cl_Uint8Type* ,Cl_Uint8Type );
extern Cl_ReturnCodeType cl_bp_pumpFeedback_set_expected_period(int16_t period);
extern Cl_ReturnCodeType  Cl_SendDatatoconsole(Cl_ConsoleTxCommandtype , Cl_Uint8Type* ,Cl_Uint8Type );
extern Cl_ReturnCodeType cl_bp_controller(cl_bp_events bp_event , Cl_Uint32Type data);
extern Cl_ReturnCodeType cl_wait(Cl_Uint32Type ul_dly_ticks);
static Cl_Uint32Type pwm_period = 200;
Cl_ReturnCodeType cl_bp_event_handler(cl_bp_events bp_event )
{
		int16_t sensordata=0,temp2=0;
		static Cl_Uint8Type temp_data=1,temp_data2=1,temp_data3=1,count_dummy=0,count_dummy1=0;
		volatile Cl_Uint8Type flag_bp=0,flag_inc=0,flag_dec=0;
	switch(bp_event)
	{
		case CL_BP_EVENT_INSTANT:
			Cl_SysStat_GetSensor_Status_Query(SENSOR_BP_ON_OFF_M,&sensordata);
			{
				temp_data = sensordata;
				
				if (temp_data == 0)
				{
					flag_bp=1;
				}
				if (flag_bp == 1 && temp_data == 0)
				{
					count_dummy1++;
					if (count_dummy1<=1)
					{
						cl_bp_controller(CL_BP_EVENT_START,0);
						temp_data=1;
						flag_bp=0;
						cl_wait(1000);
					}
					else if (count_dummy1>=2)
					{
						cl_bp_controller(CL_BP_EVENT_STOP,0);
						temp_data=1;
						flag_bp=0;
						count_dummy1=0;
						cl_wait(1000);
						
					}
				}
			}
			Cl_SysStat_GetSensor_Status_Query(SENSOR_BP_INC_M,&sensordata);
			{
				temp_data2 = sensordata;
				if (temp_data2==0)
				{

					count_dummy++;
					flag_inc=1;
				}
				if (flag_inc==1 && temp_data2 == 0)
				{
					pwm_period = pwm_period + 10 ;
					sv_cntrl_setpumpspeed(BLOODPUMP,pwm_period);
					//sv_cntrl_incpumpspeed(BLOODPUMP,1);
					flag_inc=0;
					count_dummy=0;
				}
			}
			Cl_SysStat_GetSensor_Status_Query(SENSOR_BP_DEC_M,&sensordata);
			{
				temp_data3 = sensordata;
				if (temp_data3==0)
				{
					count_dummy++;
					flag_dec=1;
				}
				if (flag_dec==1 && temp_data3 == 0)
				{
					pwm_period = pwm_period - 10 ;
					sv_cntrl_setpumpspeed(BLOODPUMP,pwm_period);
					//sv_cntrl_decpumpspeed(BLOODPUMP,1);
					flag_dec=0;
					count_dummy=0;
				}
				
			}
		// find out which button is pressed
		//(if  button is INC or DEC calculate new speed based on current speed stored in global variable
		// depending on the button call function "cl_bp_controller(cl_bp_events bp_event , Cl_Uint32Type data)"
		
		break;
		case CL_BP_EVENT_SECOND:
		
		//check for any pending speed change and if so call "cl_bp_controller(cl_bp_events bp_event , Cl_Uint32Type data)"
		
		break;
		default:break;
	}
	return CL_OK;
}

Cl_ReturnCodeType cl_bp_controller(cl_bp_events bp_event , Cl_Uint32Type data)
{
	Cl_ReturnCodeType cl_bp_retcode = CL_OK;
	cl_bp_pump_states bp_pumpstate = CL_BP_P_STATE_IDLE;
	Cl_Uint16Type bp_delta_value = 0 ;
	
	
	switch(bp_state)
	{
		case CL_BP_STATE_IDLE:
		switch (bp_event)
		{
			case CL_BP_EVENT_SET_BP_RATE:
			cl_bp_rate = data;
			pwm_period = cl_bp_rate ;
			speedchange_bloodpump_feedback(data);
			sv_cntrl_setpumpspeed(BLOODPUMP,pwm_period);
			break;
			case CL_BP_EVENT_START:
			cl_bp_retcode =  Cl_AlarmActivateAlarms(BLOODDOOR_STATUS_OPEN,true );
			cl_bp_retcode =  sv_cntrl_activatepump(BLOODPUMP);
			
			cl_bp_pumpFeedback_start();
			cl_bp_retcode = Cl_SendDatatoconsole(CONT_TX_COMMAND_BLOODPUMP_ON,&data,0);
			bp_state = CL_BP_STATE_STARTED;
			break;
			
			default:break;
			
		}
		break;
		case CL_BP_STATE_STARTED:
		switch (bp_event)
		{
			case CL_BP_EVENT_START:
			
			cl_bp_retcode =  sv_cntrl_activatepump(BLOODPUMP);
			cl_bp_retcode =  Cl_AlarmActivateAlarms(BLOODDOOR_STATUS_OPEN,true );
			cl_bp_pumpFeedback_start();
		//	speedchange_bloodpump_feedback(data);
			cl_bp_retcode = Cl_SendDatatoconsole(CONT_TX_COMMAND_BLOODPUMP_ON,&data,0);
			bp_state = CL_BP_STATE_STARTED;
			
			break;
			
			case CL_BP_EVENT_SECOND:
	//		cl_bp_pumpFeedback_get_state(&bp_pumpstate);
	//		cl_bp_pumpFeedback_get_delta(&bp_delta_value);
			//	Cl_SendDatatoconsole(CON_TX_COMMAND_PRINTTEXT,"UF_RPM",6);
			//	Cl_SendDatatoconsole(CON_TX_COMMAND_PRINTDATA,&bp_delta_value,2);

			if(bp_pumpstate == CL_BP_P_STATE_RUNNING_FAST )
			{
				//apply correction
				//		Cl_SendDatatoconsole(CON_TX_COMMAND_PRINTTEXT,"UF_FAST",7);
			//	pwm_period = pwm_period + 100;
			//	sv_cntrl_setpumpspeed(BLOODPUMP,pwm_period );
			}
			else if (bp_pumpstate == CL_BP_P_STATE_RUNNING_SLOW )
			{
				//apply correction
				//		Cl_SendDatatoconsole(CON_TX_COMMAND_PRINTTEXT,"UF_SLOW",7);
			//	pwm_period = pwm_period - 100;
			//	sv_cntrl_setpumpspeed(BLOODPUMP,pwm_period);
			}
			else
			{
				
			}
			break;
			case CL_BP_EVENT_SET_BP_RATE:
			speedchange_bloodpump_feedback(data);
			sv_cntrl_setpumpspeed(BLOODPUMP,data);
			
			bp_state =  CL_BP_STATE_STARTED;
			break;
			case CL_BP_EVENT_PAUSE:
			break;
			case CL_BP_EVENT_STOP:
		//	cl_wait(200);
			cl_bp_retcode =  sv_cntrl_deactivatepump(BLOODPUMP);
			cl_bp_pumpFeedback_start();
		//	cl_wait(20);
			cl_bp_retcode = Cl_SendDatatoconsole(CONT_TX_COMMAND_BLOODPUMP_OFF,&data,0);
			cl_bp_retcode =  cl_bp_pumpFeedback_stop();
			bp_state = CL_BP_STATE_STOPPED;
			break;
			case CL_BP_EVENT_COMPLETED:
			bp_state = CL_BP_STATE_IDLE;
			break;
			default:break;
			
		}
		break;
		case CL_BP_STATE_STOPPED:
		switch (bp_event)
		{
			case CL_BP_EVENT_SET_BP_RATE:
			cl_bp_rate = data;
			speedchange_bloodpump_feedback(data);
			sv_cntrl_setpumpspeed(BLOODPUMP,data);
			break;
			case CL_BP_EVENT_START:
			cl_bp_retcode =  sv_cntrl_activatepump(BLOODPUMP);
			cl_bp_retcode =  Cl_AlarmActivateAlarms(BLOODDOOR_STATUS_OPEN,true );
			cl_bp_pumpFeedback_start();
	//		speedchange_bloodpump_feedback(data);
			cl_bp_retcode = Cl_SendDatatoconsole(CONT_TX_COMMAND_BLOODPUMP_ON,&data,0);
			bp_state = CL_BP_STATE_STARTED;
			break;
			default:break;
			
		}
		break;
		case CL_BP_STATE_PAUSED:
		switch (bp_event)
		{
			case CL_BP_EVENT_SET_BP_RATE:
			cl_bp_rate = data;
	//		sv_cntrl_setpumpspeed(BLOODPUMP,data);
			break;
			case CL_BP_EVENT_START:
			cl_bp_retcode =  sv_cntrl_activatepump(BLOODPUMP);
			cl_bp_retcode =  Cl_AlarmActivateAlarms(BLOODDOOR_STATUS_OPEN,true );
			cl_bp_pumpFeedback_start();
		//	speedchange_bloodpump_feedback(data);
			cl_bp_retcode = Cl_SendDatatoconsole(CONT_TX_COMMAND_BLOODPUMP_ON,&data,0);
			bp_state = CL_BP_STATE_STARTED;
			break;
			case CL_BP_EVENT_STOP:
			cl_bp_retcode =  sv_cntrl_deactivatepump(BLOODPUMP);
			cl_bp_pumpFeedback_start();
			bp_state = CL_BP_STATE_STOPPED;
			cl_bp_retcode = Cl_SendDatatoconsole(CONT_TX_COMMAND_BLOODPUMP_OFF,&data,0);
			break;
			default:break;
			
		}
		break;
		case CL_BP_STATE_COMPLETED:
		break;
		default:break;
		
		
	}
	return CL_OK;
}