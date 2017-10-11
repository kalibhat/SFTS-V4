/*
 * cl_isoufPumpController.c
 *
 * Created: 7/12/2017 2:53:53 PM
 *  Author: Manav
 */ 

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
#include "sv_interface_fun.h"


cl_isouf_states cl_isouf_state = CL_ISOUF_STATE_IDLE;

typedef union {
	Cl_Uint32Type fourbytedata;
	int16_t twobytedata;
	Cl_Uint8Type bytedata[4] ;
}StateDatatype;

Cl_ReturnCodeType cl_isouf_retcode = CL_OK;
static Cl_Uint32Type cl_uf_rate = 0;
extern Cl_Uint32Type Treatdata[ID_MAX_TREAT_PARAM] ;
extern Cl_Mac_StatesType Cl_MacState;
extern volatile Cl_Uint32Type uf_final;
//extern Cl_ReturnCodeType cl_ufpumpFeedback_start(void);
extern Cl_ReturnCodeType  Cl_SendDatatoconsole(Cl_ConsoleTxCommandtype , Cl_Uint8Type* ,Cl_Uint8Type );
//extern Cl_ReturnCodeType cl_ufpumpFeedback_set_expected_period(int16_t period);
extern Cl_ConsoleMsgType Cl_ConsoleRxMsg;
extern float res_temp_value;
extern volatile float pressure_final_apt,pressure_final_vpt,pressure_final_ps1,pressure_final_ps2,pressure_final_ps3;
Cl_ReturnCodeType cl_dialysate_ready_isouf();

int Cl_ISOufTotalSecondscounter=0 ;
 extern Cl_ReturnCodeType cl_wait(Cl_Uint32Type );
bool isouf = false;
extern float ufRemoved;
float isoufRemoved =0;
Cl_NewAlarmIdType isoufalarm;
Cl_NewAlarmIdType isoufalarm1;
Cl_Uint16Type isouf_powerFailSensorData = 0;  // manav 03 added

int isopowerFailureFlag = 0;

IsoufAlarmsType Cl_IsoufAlarmTable[CL_ISOUF_ALRM_MAX] =
{
	{HOLDER1STATUS_CLOSED,CL_ALARM_ALARM,false,false,false},
	{HOLDER2STATUS_CLOSED,CL_ALARM_ALARM,false,false,false},
	{ACID_IN_CLOSED,CL_ALARM_ALARM,false,false,false},
	{BICARB_IN_CLOSED,CL_ALARM_ALARM,false,false,false},
	{FLOW_NO_FLOW,CL_ALARM_ALARM,false,false,false},
	{FLOW_LOW_FLOWRATE,CL_ALARM_ALARM,false,false,false},
	{BLOODDOOR_STATUS_OPEN,CL_ALARM_ALARM,false,false,false},
	{PS1_HIGH_THRESHOLD,CL_ALARM_ALARM,false,false,false},
	{PS1_LOW_THRESHOLD,CL_ALARM_ALARM,false,false,false},
	{PS2_HIGH_THRESHOLD,CL_ALARM_ALARM,false,false,false},
	{PS2_LOW_THRESHOLD,CL_ALARM_ALARM,false,false,false},
	{PS3_HIGH_THRESHOLD,CL_ALARM_ALARM,false,false,false},
	{PS3_LOW_THRESHOLD,CL_ALARM_ALARM,false,false,false},
	{BD_EVENT,CL_ALARM_ALARM,false,false,false},
	{ABD_EVENT,CL_ALARM_ALARM,false,false,false},
	{BLD_EVENT,CL_ALARM_ALARM,false,false,false},
	{APTSTATUS_HIGH,CL_ALARM_ALARM,false,false,false},
	{APTSTATUS_LOW,CL_ALARM_ALARM,false,false,false},
	{VPTSTATUS_HIGH,CL_ALARM_ALARM,false,false,false},
	{VPTSTATUS_LOW,CL_ALARM_ALARM,false,false,false},
	{COND_STATUS_LOW,CL_ALARM_ALARM,false,false,true},
	{COND_STATUS_HIGH,CL_ALARM_ALARM,false,false,true},
	{TEMP3_HIGH_THRESHOLD,CL_ALARM_ALARM,false,false,false},
	{TEMP3_LOW_THRESHOLD,CL_ALARM_ALARM,false,false,false},
	{TEMP2_HIGH_THRESHOLD,CL_ALARM_ALARM,false,false,false},
	{TEMP2_LOW_THRESHOLD,CL_ALARM_ALARM,false,false,false},
	{TEMP1_HIGH_THRESHOLD,CL_ALARM_ALARM,false,false,false},
	{TEMP1_LOW_THRESHOLD,CL_ALARM_ALARM,false,false,false},
	{POWER_FAILURE,CL_ALARM_ALARM,false,false,false},
	{BP_STOPPED,CL_ALARM_ALARM,false,false,false},	
		
	//{PS4STATUS,false,false,false},
	/*{TEMP1_HIGH_THRESHOLD,false,false,false},
	{TEMP1_LOW_THRESHOLD,false,false,false},
	{TEMP2_HIGH_THRESHOLD,false,false,false},
	{TEMP2_LOW_THRESHOLD,false,false,false},
	{TEMP3_HIGH_THRESHOLD,false,false,false},
	{TEMP3_LOW_THRESHOLD,false,false,false},
	{FPCURRENTSTATUS,false,false,false},
	*/
};

Cl_ReturnCodeType cl_isouf_controller(Cl_Mac_EventsType Cl_MacDlsisEvent)
{
	
//	cl_ufpump_states uf_pumpstate = CL_UFP_STATE_IDLE;
	Cl_Uint16Type uf_delta_value = 0 ;
	static Cl_Uint16Type pwm_period = 0;
	Cl_Uint32Type temp = 0;
	Cl_Uint32Type temptime = 0;
	
	StateDatatype abfdata;
	Cl_Uint8Type abfdataarray[5] =  {0,0,0,0,0};
	
	Cl_ConsoleTxCommandtype command = CON_TX_COMMAND_COMMAND_MAX;
	Cl_Uint8Type data = 0;
	Cl_Uint8Type datasize = 0;
	Cl_Uint8Type dataarray[4] =  {0,0,0,0};
	
	cl_isouf_events cl_isouf_event = EVENT_ISOUF_NULL_EVENT;
	
	cl_isouf_retcode =  cl_isouf_translatemacevent( Cl_MacDlsisEvent, &cl_isouf_event);
	
	
	switch(cl_isouf_event){
		
		case EVENT_ISOUF_HEAPARIN_BOLUS:
		cl_hep_controller(CL_HEP_EVENT_STARTBOLUS,0);
		break;
		
		
		case EVENT_ISOUF_HEP_RATE:
		if(Cl_ConsoleRxMsg.msgready == true)
		{
			
			cl_Datastreamtype cl_temp  ;
			cl_temp.bytearray[0] = Cl_ConsoleRxMsg.data.byte[0];
			cl_temp.bytearray[1] = Cl_ConsoleRxMsg.data.byte[1];
			cl_temp.bytearray[2] = Cl_ConsoleRxMsg.data.byte[2];
			cl_temp.bytearray[3] = Cl_ConsoleRxMsg.data.byte[3];
			Treatdata[ID_heprate] = cl_temp.Twobyte;
			//	hep_calibration(Treatdata[ID_heprate]);
			cl_hep_controller(CL_HEP_EVENT_SET_NORMAL_DELIVERY_RATE,Treatdata[ID_heprate]);
		}
		break;
		case EVENT_ISOUF_HEP_ON:
		
		if(Cl_ConsoleRxMsg.msgready == true)
		
		{
			cl_Datastreamtype cl_temp  ;
			cl_temp.bytearray[0] = Cl_ConsoleRxMsg.data.byte[1];
			cl_temp.bytearray[1] = Cl_ConsoleRxMsg.data.byte[2];
			cl_temp.bytearray[2] = Cl_ConsoleRxMsg.data.byte[3];
			cl_temp.bytearray[3] = Cl_ConsoleRxMsg.data.byte[4];
			
			if(Cl_ConsoleRxMsg.data.byte[0] == 1)
			{
				cl_hep_controller(CL_HEP_EVENT_GO_TO_LEFTEND,0);
				
			}
			else if(Cl_ConsoleRxMsg.data.byte[0] == 2)
			{
				cl_hep_controller(CL_HEP_EVENT_GO_TO_RIGHTEND,0);
			}
			else if(Cl_ConsoleRxMsg.data.byte[0] == 0)
			{
				cl_hep_controller(CL_HEP_EVENT_START_NORMAL_DELIVERY,Treatdata[ID_heprate]);
			}
			
		}
		
		
		break;
		case EVENT_ISOUF_HEP_OFF:
		cl_hep_controller(CL_HEP_EVENT_STOPBOLUS,0);
		break;
		case EVENT_ISOUF_COMMAND_SET_BLDPMP_ON:
		cl_bp_controller(CL_BP_EVENT_START,0);
		cl_isouf_retcode =  Cl_AlarmActivateAlarms(BLOODDOOR_STATUS_OPEN,true);
		break;
		case EVENT_ISOUF_COMMAND_SET_BLDPMP_OFF:
		cl_bp_controller(CL_BP_EVENT_STOP,0);
		Cl_Alarm_TriggerAlarm(BP_STOPPED,1);
		cl_isouf_retcode =  Cl_AlarmActivateAlarms(BLOODDOOR_STATUS_OPEN,false);
		break;
		case EVENT_ISOUF_COMMAND_SET_BLDPUMPRATE:
		if(Cl_ConsoleRxMsg.msgready == true)
		{
			
			cl_Datastreamtype cl_temp  ;
			cl_temp.bytearray[0] = Cl_ConsoleRxMsg.data.byte[0];
			cl_temp.bytearray[1] = Cl_ConsoleRxMsg.data.byte[1];
			cl_temp.bytearray[2] = Cl_ConsoleRxMsg.data.byte[2];
			cl_temp.bytearray[3] = Cl_ConsoleRxMsg.data.byte[3];
			cl_bp_controller(CL_BP_EVENT_SET_BP_RATE,cl_temp.Twobyte);
			Treatdata[ID_bloodratetreat] = cl_temp.Twobyte;
		}
		break;
		
		case EVT_CONSOLE_COMMAND_SET_BLDPUMPINC_MANUAL:
		
		if(Treatdata[ID_bloodrate] <= 450){
			
			Treatdata[ID_bloodrate] = Treatdata[ID_bloodrate] + 10;
			cl_bp_controller(CL_BP_EVENT_SET_BP_RATE,Treatdata[ID_bloodrate]);
			
			// send blood rate data to console
			
			
			abfdataarray[0] = CON_RX_PARAM_DATA_ABF_RATE;
			abfdata.fourbytedata = Treatdata[ID_bloodrate];
			abfdataarray[1] = abfdata.bytedata[0];
			abfdataarray[2] = abfdata.bytedata[1];
			abfdataarray[3] = abfdata.bytedata[2];
			abfdataarray[4] = abfdata.bytedata[3];
			
			Cl_SendDatatoconsole(CON_TX_COMMAND_CL_SYS_STATE_DATA,&abfdataarray,5);
			
		}
		
		cl_isouf_event == EVENT_ISOUF_NULL_EVENT;
		break;
		
		case EVT_CONSOLE_COMMAND_SET_BLDPUMPDEC_MANUAL:
		
		if(Treatdata[ID_bloodrate] >= 100){
			
			Treatdata[ID_bloodrate] = Treatdata[ID_bloodrate] - 10;
			cl_bp_controller(CL_BP_EVENT_SET_BP_RATE,Treatdata[ID_bloodrate]);
			
			
			abfdataarray[0] = CON_RX_PARAM_DATA_ABF_RATE;
			abfdata.fourbytedata = Treatdata[ID_bloodrate];
			abfdataarray[1] = abfdata.bytedata[0];
			abfdataarray[2] = abfdata.bytedata[1];
			abfdataarray[3] = abfdata.bytedata[2];
			abfdataarray[4] = abfdata.bytedata[3];
			
			Cl_SendDatatoconsole(CON_TX_COMMAND_CL_SYS_STATE_DATA,&abfdataarray,5);
		}
		
		cl_isouf_event == EVENT_ISOUF_NULL_EVENT;
		break;
		
		
		default : break;
	}
	
	
	switch(cl_isouf_state)
	{
			case CL_ISOUF_STATE_IDLE:
			switch (cl_isouf_event)
				
				{
					case EVENT_ISOUF_START:
					
					//cl_dialysate_ready_isouf();
					ufRemoved = 0;
					Cl_Isouf_start();
				
					cl_isouf_state = CL_ISOUF_STATE_STARTED;
					break;
					default: break;
				}
				break;
				
					
			case CL_ISOUF_STATE_STARTED:
			
			switch(cl_isouf_event){
				
// 				case  EVENT_ISOUF_STOP :
// 				cl_uf_controller(CL_UF_EVENT_STOP,0);
// 				cl_hep_controller(CL_HEP_EVENT_STOP_NORMAL_DELIVERY,0);
// 				break;
				
				case EVENT_ISOUF_TICK_500MS :
				
					cl_isouf_retcode == cl_checkforblooddetection();
				
				break;
				
				case EVENT_ISOUF_TICK_MINUTE:
				
				//isoufRemoved =  isoufRemoved + (Treatdata[ID_ufrate]/10);
				
				cl_uf_controller(CL_UF_EVENT_MINUTE,0);
			break;
			
			case EVENT_ISOUF_ALARM:
			cl_isouf_retcode = Cl_Isouf_ProcessAlarms();
				break;
				case EVENT_ISOUF_TICK_SECOND :
				
					Cl_ISOufTotalSecondscounter ++ ;
				
				//	if(cl_isouf_retcode == CL_OK){    // manav03 commented
					
					// check for completion
					//cl_sendisoufdata();
					
						Cl_Iso_Senddata();
						if(Cl_ISOufTotalSecondscounter >= Treatdata[ID_treattime]){
						
						// complete
						cl_isouf_retcode == cl_isoufcompleted();
						if(cl_isouf_retcode == CL_OK){
							
												
							cl_isouf_state = CL_ISOUF_STATE_COMPLETED;
							isouf = false;
							command = CON_TX_COMMAND_COMMAND_ISOUF_COMPLETED;
							data = (Cl_Uint8Type)COMMAND_RESULT_SUCCESS;
							cl_isouf_retcode = Cl_SendDatatoconsole(command,&data,1);
							cl_isouf_retcode = Cl_mac_apprequesthandler(MACREQ_ISOUF_COMPLETED);
							
							
						}
					}
					
					// check for alarms
			//	}	
			//	else{
					// raise alarm
			//	}
				
				break;
				
				
				case EVENT_ISOUF_STOP :
				
					Cl_Isouf_stop();
				// stop uf pump
					
				
				break;
				
// 				case EVENT_ISOUF_BYPASS_ON :
// 				
// 				CL_Isoufpause();
// 				
// 				break;
// 				
				// implement iso uf stop state
				// implement bp on event
				// implement bp off event  , iso uf should stop
				
				
				default: break;
				
			}
			
			break;
			
			case CL_ISOUF_STATE_CRITICAL_ALARM:
			
			switch(cl_isouf_event){
				
				case EVENT_ISOUF_START:
				
				//Cl_ISOufTotalSecondscounter = 0;
				//	cl_dialysate_ready_isouf();
				Cl_Isouf_start();
				cl_isouf_state = CL_ISOUF_STATE_STARTED;
				break;
				
				case EVENT_ISOUF_CLR_ALARM:
				
				//Cl_ISOufTotalSecondscounter = 0;
				//	cl_dialysate_ready_isouf();
				// manav03 added
				if(isopowerFailureFlag == 1){
					
					Cl_SysStat_GetSensor_Status_Query(SENSOR_POWER_FAILURE,&isouf_powerFailSensorData);
					
					if(isouf_powerFailSensorData == 0){
						
						sv_cntrl_resetyellowalarm();
						sv_cntrl_resetredalarm();
						sv_cntrl_nobuzzer();
						
						isopowerFailureFlag = 0;
						command = CON_TX_COMMAND_ALARM_CLEARED;
						Cl_SendDatatoconsole(command,&data,0);
						Cl_Isouf_ResetAlertsforReassertion();
						sv_cntrl_activatevenousclamp();
					
					}
				}
				else{
					
					command = CON_TX_COMMAND_ALARM_CLEARED;
					Cl_SendDatatoconsole(command,&data,0);
					
					Cl_Isouf_ResetAlertsforReassertion();
					sv_cntrl_activatevenousclamp();
					Cl_Isouf_start();
					cl_isouf_state = CL_ISOUF_STATE_STARTED;
				}
				
				
				
				break;
				
				
				default: break;
			}
			
			break;
			
			case CL_ISOUF_STATE_STOPPED:
			
				switch(cl_isouf_event){
					
					case EVENT_ISOUF_START:
				
				//Cl_ISOufTotalSecondscounter = 0;
				//	cl_dialysate_ready_isouf();
				ufRemoved = 0;
					Cl_Isouf_start();
					cl_isouf_state = CL_ISOUF_STATE_STARTED;
				break;
					
				default: break;	
					
					
				}
			
			break;
			
			case CL_ISOUF_STATE_COMPLETED:
			switch(cl_isouf_event){
				
				case EVENT_ISOUF_TICK_SECOND :
				
				break;
				
				case EVENT_ISOUF_START:
				
				//Cl_ISOufTotalSecondscounter = 0;
					//cl_dialysate_ready_isouf();
					ufRemoved = 0;
					Cl_Isouf_start();
					cl_isouf_state = CL_ISOUF_STATE_STARTED;
				break;
			
			default : break;	
			
			}
			break;
			
			case CL_ISOUF_STATE_POST_COMPLETION:
			
			switch(cl_isouf_event){
				
				case EVENT_ISOUF_TICK_SECOND :
				
				break;
				
				case EVENT_ISOUF_START_RETURN:
				
				break;
				
				
				
				default:break;
			}
			
			break;
			
			default:break;
			
			
	}
	return CL_OK;
	
	
}




Cl_ReturnCodeType Cl_Isouf_ProcessAlarms()
{
	Cl_ReturnCodeType 	Cl_isoufretcode = CL_OK;
	ClIsoufAlarmIdType  ClIsoufAlarmId;
	Cl_Uint8Type data;
	Cl_ConsoleTxCommandtype command = CON_TX_COMMAND_COMMAND_MAX;
	Cl_NewAlarmIdType cl_alarmid;
	
	cl_isouf_retcode = Cl_Isouf_UpdateAlarmTable(&ClIsoufAlarmId);
	//   	Cl_rinseretcode =	Cl_Alarm_GetLastAlarm(&cl_rinsealarmid);
	//   	data = (Cl_Uint8Type)cl_rinsealarmid;
	//   	command = CON_TX_COMMAND_ALARM;
	//   	Cl_rinseretcode = Cl_SendDatatoconsole(command,&data,0);
	
	return (cl_isouf_retcode );
	
}

Cl_ReturnCodeType Cl_Isouf_UpdateAlarmTable(ClIsoufAlarmIdType *ClIsoufAlarmId)
{
	Cl_ReturnCodeType 	Cl_isoufretcode = CL_OK;
	Cl_NewAlarmIdType cl_alarmId;
	Cl_Uint8Type tempcount = 0;
	Cl_Uint8Type data[2];
	Cl_ConsoleTxCommandtype command = CON_TX_COMMAND_COMMAND_MAX;
	
	for (tempcount = 0 ; tempcount < CL_ISOUF_ALRM_MAX ; tempcount++)
	{
		
		Cl_Alarm_GetAlarmStatus(Cl_IsoufAlarmTable[tempcount].Cl_IsoufAlarmId,&Cl_IsoufAlarmTable[tempcount].IsActive);
		if (Cl_IsoufAlarmTable[tempcount].IsActive)
		{
			if(!Cl_IsoufAlarmTable[tempcount].IsRaised)
			{
				//	cl_alarmId = (Cl_Uint8Type)Cl_DlsisAlarmTable[tempcount].Cl_DlsisAlarmId;

				//	data[0] = (Cl_Uint8Type)Cl_DlsisAlarmTable[tempcount].Cl_DlsisAlarmId;
				//	data[1] = (Cl_Uint8Type) CL_ALARM_TRIGGERED;
				//	data[1] = (Cl_Uint8Type) CRITICAL;
				//	command = CON_TX_COMMAND_ALARM;
				//	Cl_dlsisretcode = Cl_SendDatatoconsole(command,&data,3);
				Cl_IsoufAlarmTable[tempcount].IsRaised = true;
				Cl_isoufretcode = CL_IsoufAlarmActon(Cl_IsoufAlarmTable[tempcount].Cl_IsoufAlarmId);
				
				
			}
			
		}
		else
		{
			// alarm was present before , but not active now.
			if(Cl_IsoufAlarmTable[tempcount].IsRaised == true)
			{
				Cl_IsoufAlarmTable[tempcount].IsRaised = false;
				command = CON_TX_COMMAND_ALARM;
				data[0] = (Cl_Uint8Type)(Cl_IsoufAlarmTable[tempcount].Cl_IsoufAlarmId);
				data[1] = (Cl_Uint8Type) 0;
				Cl_isoufretcode = Cl_SendDatatoconsole(command,&data,2);
			}

		}
		
		
		
		

	}
	

	

	return (Cl_isoufretcode );
}


Cl_ReturnCodeType  CL_IsoufAlarmActon(Cl_NewAlarmIdType cl_isoufalarmid)
{
		Cl_ReturnCodeType 	Cl_isoufretcode = CL_OK;
			Cl_Uint8Type data[3] ={0, 0} ,*data1 = NULL;
			Cl_Uint8Type data3;
			Cl_ConsoleTxCommandtype command = CON_TX_COMMAND_COMMAND_MAX;
			Cl_Uint16Type levelswitchstatus = 0;	
			static Cl_Uint8Type fillseccounter = 0;
			Cl_NewAlarmIdType NewAlarmId = _NO_ALARM;
		

			

							
		switch(cl_isoufalarmid)
		{
			
			case POWER_FAILURE:
			
			isopowerFailureFlag = 1;
			sv_cntrl_setredalarm();
			sv_cntrl_buzzer();
			Cl_Isouf_stop();
			
			 if(cl_isoufalarmid != _NO_ALARM)
			 {
				 isoufalarm = cl_isoufalarmid;
				 isoufalarm1 = isoufalarm;
				 data[0] = (Cl_Uint8Type)cl_isoufalarmid;
				 data[1] = (Cl_Uint8Type) CL_ALARM_TRIGGERED;
				 data[2] = (Cl_Uint8Type)CRITICAL;
				 command = CON_TX_COMMAND_ALARM;
				 cl_wait(10);
				 Cl_isoufretcode = Cl_SendDatatoconsole(command,&data,3);
				 
				 //
				 // 			command = CON_TX_COMMAND_CL_SYS_STATE;
				 // 			data[0] = (Cl_Uint8Type)CLEANING;
				 // 			data[1] = (Cl_Uint8Type)CLEAN_CLEAN_ALARM_STOPPED;
				 // 			Cl_dlsisretcode = Cl_SendDatatoconsole(command,&data,2);
			 }
			
			break;
			
			case BP_STOPPED:
			case BLOODDOOR_STATUS_OPEN:
			case APTSTATUS_LOW:
			case APTSTATUS_HIGH:
			case VPTSTATUS_LOW:
			case VPTSTATUS_HIGH:
			case ABD_EVENT:
			case BLD_EVENT:
							
				sv_cntrl_setredalarm();
				sv_cntrl_buzzer();
				Cl_Isouf_safesate();	
				 if(cl_isoufalarmid != _NO_ALARM)
				 {
					 isoufalarm = cl_isoufalarmid;
					 isoufalarm1 = isoufalarm;
					 data[0] = (Cl_Uint8Type)cl_isoufalarmid;
					 data[1] = (Cl_Uint8Type) CL_ALARM_TRIGGERED;
					 data[2] = (Cl_Uint8Type)CRITICAL;
					 command = CON_TX_COMMAND_ALARM;
					 cl_wait(10);
					 Cl_isoufretcode = Cl_SendDatatoconsole(command,&data,3);
					 
					 //
					 // 			command = CON_TX_COMMAND_CL_SYS_STATE;
					 // 			data[0] = (Cl_Uint8Type)CLEANING;
					 // 			data[1] = (Cl_Uint8Type)CLEAN_CLEAN_ALARM_STOPPED;
					 // 			Cl_dlsisretcode = Cl_SendDatatoconsole(command,&data,2);
				 }
			break;
			
			case ACID_IN_CLOSED:
			case BICARB_IN_CLOSED:
			case HOLDER1STATUS_CLOSED:
			case HOLDER2STATUS_CLOSED:
			case TEMP2_HIGH_THRESHOLD:
			case TEMP2_LOW_THRESHOLD:
			case TEMP3_HIGH_THRESHOLD:
			case TEMP3_LOW_THRESHOLD:
			case 	PS1_HIGH_THRESHOLD:
			case    PS1_LOW_THRESHOLD:
			case 	PS2_HIGH_THRESHOLD:
			case    PS2_LOW_THRESHOLD:
			case 	PS3_HIGH_THRESHOLD:
			case    PS3_LOW_THRESHOLD:
			case FLOW_NO_FLOW:
			case FLOW_HIGH_FLOWRATE:
			case FLOW_LOW_FLOWRATE:
			case COND_DAC_OPEN:
			case COND_DAC_HIGH:
			case 	UFP_OVERRUN:
			case	UFP_UNDERRUN:
			case 	BC_OVERRUN:
			case	BC_UNDERRUN:
			case	BC_FAILED:
			case BD_EVENT:
			case BC_ALARM:
			case TEMP1_HIGH_THRESHOLD:
			case TEMP1_LOW_THRESHOLD:
			case COND_STATUS_LOW:
			case COND_STATUS_HIGH:	
				// do nothing as the alarm limits were increased
			
			sv_cntrl_setyellowalarm();
			sv_cntrl_buzzer();
			
				Cl_bc_controller(BC_EVENT_STOP);
				sv_cntrl_deactivate_valve(VALVE_ID15);
				sv_cntrl_deactivate_valve(VALVE_ID13);
				sv_cntrl_activate_valve(VALVE_ID14);
				//DD_SET_VALVE(VALVE_ID1);
				sv_cntrl_deactivatepump(DCMOTOR1);
				sv_cntrl_deactivatepump(DCMOTOR2);
				// send alert message
				Cl_isoufretcode = sv_cntrl_poweroffheater();
				Cl_isoufretcode = SetHeaterState(CL_HEATER_STATE_OFF);
				 isoufalarm = cl_isoufalarmid;
				 isoufalarm1 = isoufalarm;
				 data[0] = (Cl_Uint8Type)cl_isoufalarmid;
				 data[1] = (Cl_Uint8Type) CL_ALARM_TRIGGERED;
				 data[2] = (Cl_Uint8Type)CRITICAL;
				 command = CON_TX_COMMAND_ALERT;
				 cl_wait(10);
				 Cl_isoufretcode = Cl_SendDatatoconsole(command,&data,3);
				
			break;
			
			case DE_CHAMBER_LOW: //23
			break;
			case SYSTEM_NOT_READY: //24
			break;
			//case POWER_FAILURE: //25
			//break;

			case WATCHDOG_TIMER: //27
			//	NewAlarmId = _WATCHDOG_TIMER;
			break;
			
			
			
			/*case COND_STATUS_LOW:
			case COND_STATUS_HIGH:
			Cl_Dlsis_StopDialysis();
			Cl_SendDatatoconsole(CON_TX_COMMAND_PRINTTEXT,"COND",4);
			cl_dlsis_state = CL_DLSIS_STATE_CRITICAL_ALARM;
			break;*/
			
			default:
			break;

		}
		
return CL_OK;			
}

Cl_ReturnCodeType Cl_Isouf_ResetAlertsforReassertion(void )
{
	Cl_ReturnCodeType 	Cl_dlsisretcode = CL_OK;
	ClDlsisAlarmIdType ClDlsisAlarmId;
	Cl_Uint8Type data;
	Cl_ConsoleTxCommandtype command = CON_TX_COMMAND_COMMAND_MAX;
	Cl_NewAlarmIdType cl_dlsisalarmid;
	
	Cl_IsoufAlarmTable[CL_DLSIS_ALARM_HOLDER1STATUS_CLOSE].IsRaised = false;
	Cl_IsoufAlarmTable[CL_DLSIS_ALARM_HOLDER2STATUS_CLOSE].IsRaised = false;
	Cl_AlarmResetAlarm(HOLDER1STATUS_CLOSED);
	Cl_AlarmResetAlarm(HOLDER2STATUS_CLOSED);
	Cl_IsoufAlarmTable[CL_DLSIS_ALARM_BLOODDOOR_STATUS_OPEN].IsRaised = false;
	Cl_AlarmResetAlarm(BLOODDOOR_STATUS_OPEN);
	//Cl_DlsisAlarmTable[CL_RINSE_ALARM_BYPASSDOOR_STATUS_OPEN].IsRaised = false;
	//Cl_AlarmResetAlarm(BYPASSDOOR_STATUS_OPEN);

	//Cl_RinseAlarmTable[CL_RINSE_ALARM_COND_STATUS_LOW].IsRaised = false;
	//Cl_AlarmResetAlarm(COND_STATUS_LOW);
	//Cl_RinseAlarmTable[CL_RINSE_ALARM_COND_STATUS_HIGH].IsRaised = false;
	//Cl_AlarmResetAlarm(COND_STATUS_HIGH);
	//Cl_RinseAlarmTable[CL_RINSE_ALARM_COND_DAC_OPEN].IsRaised = false;
	//Cl_RinseAlarmTable[CL_RINSE_ALARM_COND_DAC_RO].IsRaised = false;
	//Cl_RinseAlarmTable[CL_RINSE_ALARM_COND_DAC_HIGH].IsRaised = false;
	Cl_IsoufAlarmTable[CL_Dlsis_ALARM_PS1_HIGH_THRESHOLD].IsRaised = false;
	Cl_IsoufAlarmTable[CL_Dlsis_ALARM_PS1_LOW_THRESHOLD].IsRaised = false;
	Cl_AlarmResetAlarm(PS1_HIGH_THRESHOLD);
	Cl_AlarmResetAlarm(PS1_LOW_THRESHOLD);
	Cl_IsoufAlarmTable[CL_Dlsis_ALARM_PS2_HIGH_THRESHOLD].IsRaised = false;
	Cl_IsoufAlarmTable[CL_Dlsis_ALARM_PS2_LOW_THRESHOLD].IsRaised = false;
	Cl_AlarmResetAlarm(PS2_HIGH_THRESHOLD);
	Cl_AlarmResetAlarm(PS2_LOW_THRESHOLD);
	Cl_IsoufAlarmTable[CL_Dlsis_ALARM_PS3_HIGH_THRESHOLD].IsRaised = false;
	Cl_IsoufAlarmTable[CL_Dlsis_ALARM_PS3_LOW_THRESHOLD].IsRaised = false;
	Cl_AlarmResetAlarm(PS3_HIGH_THRESHOLD);
	Cl_AlarmResetAlarm(PS3_LOW_THRESHOLD);
	Cl_IsoufAlarmTable[CL_DLSIS_BDSTATUS].IsRaised = false;
	Cl_AlarmResetAlarm(BD_EVENT);
	Cl_IsoufAlarmTable[CL_DLSIS_ABDSTATUS].IsRaised = false;
	Cl_AlarmResetAlarm(ABD_EVENT);
	Cl_IsoufAlarmTable[CL_DLSIS_BLDSTATUS].IsRaised = false;
	Cl_AlarmResetAlarm(BLD_EVENT);
	Cl_IsoufAlarmTable[CL_DLSIS_APTSTATUS_HIGH].IsRaised = false;
	Cl_IsoufAlarmTable[CL_DLSIS_APTSTATUS_LOW].IsRaised = false;
	Cl_AlarmResetAlarm(APTSTATUS_HIGH);
	Cl_AlarmResetAlarm(APTSTATUS_LOW);
	Cl_IsoufAlarmTable[CL_DLSIS_VPTSTATUS_HIGH].IsRaised = false;
	Cl_IsoufAlarmTable[CL_DLSIS_VPTSTATUS_LOW].IsRaised = false;
	Cl_AlarmResetAlarm(VPTSTATUS_HIGH);
	Cl_AlarmResetAlarm(VPTSTATUS_LOW);
	Cl_IsoufAlarmTable[CL_DLSIS_COND_STATUS_LOW].IsRaised = false;
	Cl_IsoufAlarmTable[CL_DLSIS_COND_STATUS_HIGH].IsRaised = false;
	Cl_AlarmResetAlarm(COND_STATUS_LOW);
	Cl_AlarmResetAlarm(COND_STATUS_HIGH);
	Cl_IsoufAlarmTable[CL_Dlsis_ALARM_TEMP3_HIGH_THRESHOLD].IsRaised = false;
	Cl_IsoufAlarmTable[CL_Dlsis_ALARM_TEMP3_LOW_THRESHOLD].IsRaised = false;
	Cl_AlarmResetAlarm(TEMP3_HIGH_THRESHOLD);
	Cl_AlarmResetAlarm(TEMP3_LOW_THRESHOLD);
	Cl_IsoufAlarmTable[CL_Dlsis_ALARM_TEMP2_HIGH_THRESHOLD].IsRaised = false;
	Cl_IsoufAlarmTable[CL_Dlsis_ALARM_TEMP2_LOW_THRESHOLD].IsRaised = false;
	Cl_AlarmResetAlarm(TEMP2_HIGH_THRESHOLD);
	Cl_AlarmResetAlarm(TEMP2_LOW_THRESHOLD);
	Cl_IsoufAlarmTable[CL_Dlsis_ALARM_TEMP1_HIGH_THRESHOLD].IsRaised = false;
	Cl_IsoufAlarmTable[CL_Dlsis_ALARM_TEMP1_LOW_THRESHOLD].IsRaised = false;
	Cl_AlarmResetAlarm(TEMP1_HIGH_THRESHOLD);
	Cl_AlarmResetAlarm(TEMP1_LOW_THRESHOLD);
	Cl_IsoufAlarmTable[CL_Dlsis_ALARM_BP_STOPPED].IsRaised = false;
	Cl_AlarmResetAlarm(BP_STOPPED);
	Cl_IsoufAlarmTable[CL_Dlsis_ALARM_POWER_FAILURE].IsRaised = false;
	Cl_AlarmResetAlarm(POWER_FAILURE);
	/*Cl_RinseAlarmTable[CL_RINSE_ALARM_TEMP2_HIGH_THRESHOLD].IsRaised = false;
	Cl_AlarmResetAlarm(TEMP2_LOW_THRESHOLD);
	Cl_AlarmResetAlarm(TEMP2_HIGH_THRESHOLD);
	Cl_RinseAlarmTable[CL_RINSE_ALARM_TEMP3_HIGH_THRESHOLD].IsRaised = false;
	Cl_RinseAlarmTable[CL_RINSE_ALARM_TEMP3_LOW_THRESHOLD].IsRaised = false;
	Cl_AlarmResetAlarm(TEMP2_HIGH_THRESHOLD);
	Cl_AlarmResetAlarm(TEMP2_LOW_THRESHOLD);
	Cl_RinseAlarmTable[CL_RINSE_ALARM_HOLDER1STATUS_OPEN].IsRaised = false;
	Cl_AlarmResetAlarm(HOLDER1STATUS_OPEN);
	Cl_AlarmResetAlarm(HOLDER1STATUS_CLOSED);
	Cl_RinseAlarmTable[CL_RINSE_ALARM_HOLDER2STATUS_OPEN].IsRaised = false;
	Cl_AlarmResetAlarm(HOLDER2STATUS_OPEN);
	Cl_AlarmResetAlarm(HOLDER2STATUS_CLOSED);
	Cl_RinseAlarmTable[CL_RINSE_ALARM_FLOW_NO_FLOW].IsRaised = false;
	Cl_RinseAlarmTable[CL_RINSE_ALARM_FLOW_LOW_FLOWRATE].IsRaised = false;
	Cl_RinseAlarmTable[CL_RINSE_ALARM_LEVEL_SWITCH_LOW_TOGGLERATE].IsRaised = false;*/
	
	
	
	return (Cl_dlsisretcode);
}


Cl_ReturnCodeType Cl_Isouf_safesate(){
	
	Cl_Isouf_stop();
	cl_wait(10);
	cl_isouf_state = CL_ISOUF_STATE_CRITICAL_ALARM;
}


Cl_ReturnCodeType CL_Isoufpause(){

Cl_Uint8Type data;

cl_uf_controller(CL_UF_EVENT_STOP,0);
isouf = false;
cl_isouf_retcode = Cl_SendDatatoconsole(CON_TX_COMMAND_COMMAND_ISOUF_PAUSED,&data,0);
	
}

Cl_ReturnCodeType CL_Isoufresume(){

	Cl_Uint8Type data;

sv_cntrl_deactivate_valve(VALVE_ID13);
sv_cntrl_activate_valve(VALVE_ID14);
sv_cntrl_activate_valve(VALVE_ID15);

	cl_uf_controller(CL_UF_EVENT_START,0);
	isouf = true;
	cl_isouf_retcode = Cl_SendDatatoconsole(CON_TX_COMMAND_COMMAND_ISOUF_RESUMED,&data,0);
	
}



Cl_ReturnCodeType Cl_Isouf_stop(){
	
	Cl_Uint8Type data;
	
	cl_isouf_retcode =  Cl_AlarmActivateAlarms(BLOODDOOR_STATUS_OPEN,false);
// 	sv_cntrl_deactivate_valve(VALVE_ID13);
// 	sv_cntrl_deactivate_valve(VALVE_ID14);
// 	sv_cntrl_activate_valve(VALVE_ID15);
	cl_bp_controller(CL_BP_EVENT_STOP,0);
	sv_cntrl_deactivatevenousclamp();
	
	cl_uf_controller(CL_UF_EVENT_STOP,0);
	isouf = false;
	sv_cntrl_enable_bypass();
	cl_isouf_retcode = Cl_SendDatatoconsole(CON_TX_COMMAND_COMMAND_ISOUF_STOPPED,&data,0);
	
	isouf = false;
	cl_isouf_state = CL_ISOUF_STATE_STOPPED;
	
}

Cl_ReturnCodeType Cl_Isouf_start(){
	
	Cl_Uint8Type data;
	
	
	sv_cntrl_nobuzzer();
	sv_cntrl_resetyellowalarm();
	sv_cntrl_resetgreenalarm();
	sv_cntrl_resetredalarm();
	
	cl_isouf_retcode =  Cl_AlarmActivateAlarms(BLOODDOOR_STATUS_OPEN,true);
	
	cl_isouf_retcode =  Cl_AlarmConfigureAlarmType(BLOODDOOR_STATUS_OPEN,LOGIC_HIGH,0,0,0);
	cl_isouf_retcode =  Cl_AlarmConfigureAlarmType(ACID_IN_OPEN,LOGIC_LOW,0,0,0);
	cl_isouf_retcode =  Cl_AlarmConfigureAlarmType(BICARB_IN_OPEN,LOGIC_LOW,0,0,0);
	
	
	//Cl_dlsisretcode =  Cl_AlarmActivateAlarms(BLOODDOOR_STATUS_OPEN,true );
	cl_isouf_retcode =  Cl_AlarmActivateAlarms(LEVELSWITCH_OFF_TO_ON,true );
	cl_isouf_retcode =  Cl_AlarmActivateAlarms(LEVELSWITCH_ON_TO_OFF ,true);
	cl_isouf_retcode =  Cl_AlarmActivateAlarms( HOLDER1STATUS_CLOSED,true );
	cl_isouf_retcode =  Cl_AlarmActivateAlarms( HOLDER2STATUS_CLOSED,true );
	cl_isouf_retcode =  Cl_AlarmActivateAlarms( HOLDER1STATUS_OPEN,false );
	cl_isouf_retcode =  Cl_AlarmActivateAlarms( HOLDER2STATUS_OPEN,false );
	cl_isouf_retcode =  Cl_AlarmActivateAlarms(COND_STATUS_LOW,false );
	cl_isouf_retcode =  Cl_AlarmActivateAlarms(COND_STATUS_HIGH,false );
	//	Cl_dlsisretcode =  Cl_AlarmActivateAlarms(COND_DAC_OPEN,true );
	//	Cl_dlsisretcode =  Cl_AlarmActivateAlarms(COND_DAC_RO,true );
	//	Cl_dlsisretcode =  Cl_AlarmActivateAlarms(COND_DAC_HIGH,true );
	cl_isouf_retcode =  Cl_AlarmActivateAlarms( ABD_EVENT,true );
	cl_isouf_retcode =  Cl_AlarmActivateAlarms( BD_EVENT,false );
	cl_isouf_retcode =  Cl_AlarmActivateAlarms(APTSTATUS_HIGH,false );
	cl_isouf_retcode =  Cl_AlarmActivateAlarms(VPTSTATUS_HIGH ,false);
	cl_isouf_retcode =  Cl_AlarmActivateAlarms(APTSTATUS_LOW,false );
	cl_isouf_retcode =  Cl_AlarmActivateAlarms(VPTSTATUS_LOW,false);
	cl_isouf_retcode =  Cl_AlarmActivateAlarms( BLD_EVENT,true );
	cl_isouf_retcode =  Cl_AlarmActivateAlarms(PS2_HIGH_THRESHOLD,false );
	cl_isouf_retcode =  Cl_AlarmActivateAlarms(PS2_LOW_THRESHOLD,false );
	cl_isouf_retcode =  Cl_AlarmActivateAlarms(PS1_HIGH_THRESHOLD,false );
	cl_isouf_retcode =  Cl_AlarmActivateAlarms(PS1_LOW_THRESHOLD,false );
	cl_isouf_retcode =  Cl_AlarmActivateAlarms(PS3_HIGH_THRESHOLD ,false);
	cl_isouf_retcode =  Cl_AlarmActivateAlarms(PS3_LOW_THRESHOLD ,false);
	
	cl_isouf_retcode =  Cl_AlarmActivateAlarms(SENSOR_TEMP3STATUS,false );
	cl_isouf_retcode =  Cl_AlarmActivateAlarms(SENSOR_TEMP2STATUS,false );
	
	cl_isouf_retcode =  Cl_AlarmActivateAlarms(TEMP1_HIGH_THRESHOLD,false );
	cl_isouf_retcode =  Cl_AlarmActivateAlarms(TEMP1_LOW_THRESHOLD,false );
	cl_isouf_retcode =  Cl_AlarmActivateAlarms(TEMP2_HIGH_THRESHOLD,false );
	cl_isouf_retcode =  Cl_AlarmActivateAlarms(TEMP2_LOW_THRESHOLD,false );
	cl_isouf_retcode =  Cl_AlarmActivateAlarms(TEMP3_HIGH_THRESHOLD,false );
	cl_isouf_retcode =  Cl_AlarmActivateAlarms(TEMP3_LOW_THRESHOLD,false );
	
	Cl_ISOufTotalSecondscounter = 0;
	//	cl_dialysate_ready_isouf();
	sv_cntrl_deactivate_valve(VALVE_ID13);
	sv_cntrl_activate_valve(VALVE_ID14);
	sv_cntrl_deactivate_valve(VALVE_ID15);
	cl_hep_controller(CL_HEP_EVENT_START_NORMAL_DELIVERY,0);
	cl_bp_controller(CL_BP_EVENT_START,0);
	cl_uf_controller(CL_UF_EVENT_START,0);
	
	
	isouf = true;
	cl_isouf_retcode = Cl_SendDatatoconsole(CON_TX_COMMAND_COMMAND_ISOUF_STARTED,&data,0);
	cl_wait(10);
		Cl_Iso_UpdateTimeInfo();
}

Cl_ReturnCodeType Cl_Iso_Senddata(){
	
	static float cond = 0;
	Cl_Uint8Type systemdataarray[44] =  {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
	cl_IsoUfDatatype tempdata;
	Cl_Uint16Type temp=0, temp1=0;
	static float avgtmp3=0,apt=0,vpt=0,ps1=0,ps2=0;
	static Cl_Uint32Type timecount = 0;
	Cl_ReturnCodeType Cl_dlsisretcode = CL_OK;
	Cl_Uint8Type count = 0;
	Cl_ConsoleTxCommandtype command = CON_TX_COMMAND_COMMAND_MAX;
	Cl_Uint16Type sensordatamillivolts;
	command = CON_TX_COMMAND_CL_SYS_STATE_DATA ;
	systemdataarray[0] = ISOUF_DATA;
	count++;
/*	
	COND, // eg 138
	TEMP3, // eg 370
	TEMP3,
	APT,
	VPT,
	TMP,
	UF_REMOVED
	*/
		Cl_SysStat_GetSensor_Status_Query(SENSOR_TEMP3STATUS,&temp);
		{
//							float sensordata,ftemp1,temp1;
// 							sensordata = temp;
// 							uint16_t cal_data  = (402 *100* sensordata)/(2*32768);
// 							res_temp_lookuptable(cal_data);
// 							//	tmp3 =	(tmp3*5 + res_temp_value)/6;
// //							temp1 = res_temp_value/10;
// //							temp1 = temp1 - 31 -20;
// //							avgtmp3 =(avgtmp3*5 + temp1)/6;
// 							avgtmp3 = (res_temp_value  - 0.4)/10;
							avgtmp3 = (float)temp/10;
		}
		Cl_SysStat_GetSensor_Status_Query(SENSOR_COND_STATUS,&temp);
		{
			float cond_comp;
				float temp2;
				
				if (temp > 50)
			{
				//	calibration_cond(temp);
					cond =  -0.0001 * temp* temp  + 0.36 * temp  - 38.39 ;
				//	cond =(cond*5 + cond_final_cs3)/6;
					cond_comp= cond/(1+(avgtmp3/10-25.0)*0.020);
				
					
					tempdata.word = (cond_comp);
					systemdataarray[count++] = tempdata.bytearray[0];
					systemdataarray[count++] = tempdata.bytearray[1];
					systemdataarray[count++] = tempdata.bytearray[2];
					systemdataarray[count++] = tempdata.bytearray[3];
			}
				else
				{
					tempdata.word = 0;
			systemdataarray[count++] = tempdata.bytearray[0];
			systemdataarray[count++] = tempdata.bytearray[1];
			systemdataarray[count++] = tempdata.bytearray[2];
			systemdataarray[count++] = tempdata.bytearray[3];
		}
					
		}
		{
			
		//	float ftemp,ftemp1;
		//	ftemp = temp * 0.803;
		//	calibration_tmp(ftemp,TS3);
		//	avgtmp3 =(avgtmp3*5 + temprature_final_value_3)/6;
			tempdata.word = (avgtmp3);
			systemdataarray[count++] = tempdata.bytearray[0];
			systemdataarray[count++] = tempdata.bytearray[1];
			systemdataarray[count++] = tempdata.bytearray[2];
			systemdataarray[count++] = tempdata.bytearray[3];
		}
		Cl_SysStat_GetSensor_Status_Query(SENSOR_APTSTATUS,&tempdata.Twobyte);
		{
			
			sensordatamillivolts = (tempdata.Twobyte * 0.8036) ;
			calibration_apt(sensordatamillivolts);
			apt =(apt*5 + pressure_final_apt)/6;
			tempdata.word	= apt;
			systemdataarray[count++] = tempdata.bytearray[0] ;
			systemdataarray[count++] = tempdata.bytearray[1] ;
			systemdataarray[count++] = tempdata.bytearray[2] ;
			systemdataarray[count++] = tempdata.bytearray[3] ;
		}
		Cl_SysStat_GetSensor_Status_Query(SENSOR_VPTSTATUS,&tempdata.Twobyte);
		{
			sensordatamillivolts = (tempdata.Twobyte * 0.8036) ;
			calibration_vpt(sensordatamillivolts);
			vpt =(vpt*5 + pressure_final_vpt)/6;
			tempdata.word	 = vpt;
			systemdataarray[count++] = tempdata.bytearray[0] ;
			systemdataarray[count++] = tempdata.bytearray[1] ;
			systemdataarray[count++] = tempdata.bytearray[2] ;
			systemdataarray[count++] = tempdata.bytearray[3] ;
		}
			Cl_SysStat_GetSensor_Status_Query(SENSOR_PS1STATUS,&tempdata.Twobyte);
			sensordatamillivolts = (tempdata.Twobyte * 0.8036) ;
			calibration_ps1(sensordatamillivolts);
			ps1=(ps1*5 + pressure_final_ps1)/6;
	
			Cl_SysStat_GetSensor_Status_Query(SENSOR_PS2STATUS,&tempdata.Twobyte);
			sensordatamillivolts = (tempdata.Twobyte * 0.8036) ;
			calibration_ps2(sensordatamillivolts);
			ps2=(ps2*5 + pressure_final_ps2)/6;
			
			tempdata.word	 = (((apt + vpt) - (ps1+ps2))/2)-25;


			systemdataarray[count++] = tempdata.bytearray[0] ;
			systemdataarray[count++] = tempdata.bytearray[1] ;
			systemdataarray[count++] = tempdata.bytearray[2] ;
			systemdataarray[count++] = tempdata.bytearray[3] ;
			
// 			systemdataarray[count++] = 0x0A; // tmp
// 			systemdataarray[count++] = 0x0A; // tmp
// 			systemdataarray[count++] = 0x00;
// 			systemdataarray[count++] = 0x00;
// 			
			tempdata.word = ufRemoved ;
			
			systemdataarray[count++] = tempdata.bytearray[0] ;
			systemdataarray[count++] = tempdata.bytearray[1] ;
			systemdataarray[count++] = tempdata.bytearray[2] ;
			systemdataarray[count++] = tempdata.bytearray[3] ;
			
			tempdata.word = Treatdata[ID_ufrate] ;
			
			systemdataarray[count++] = tempdata.bytearray[0] ;
			systemdataarray[count++] = tempdata.bytearray[1] ;
			systemdataarray[count++] = tempdata.bytearray[2] ;
			systemdataarray[count++] = tempdata.bytearray[3] ;
			
			cl_isouf_retcode = Cl_SendDatatoconsole(CON_TX_COMMAND_CL_SYS_STATE_DATA,&systemdataarray,count);
	//Cl_dlsisretcode = Cl_SendDatatoconsole(command,&systemdataarray,count);
	
	return CL_OK;
	
	
}

Cl_ReturnCodeType Cl_Iso_UpdateTimeInfo(void)
{
	
	Cl_ReturnCodeType Cl_rinseretcode = CL_ERROR;
	Cl_ConsoleTxCommandtype command = CON_TX_COMMAND_COMMAND_MAX;
	Cl_Uint8Type data[7] = {0,0,0,0,0 ,0,0};
	
	command = CON_TX_COMMAND_REM_TIME;

	
	data[0]= (Cl_Uint8Type)ISOUF_DATA;
	data[1]= (Cl_Uint8Type)1;

	
	Cl_rinseretcode = Cl_SendDatatoconsole(command,&data,2);
	return CL_OK;
	
}

Cl_ReturnCodeType cl_checkforblooddetection(){
	
	Cl_Uint16Type bldtemp = 0;
	
	Cl_SysStat_GetSensor_Status_Query(SENSOR_BDSTATUS , &bldtemp);
	
	if(bldtemp == 1){
		return CL_OK;
	}
	else{
		return CL_OK; // should be CL_REJECTED
	}
	
	
}

Cl_ReturnCodeType  cl_isouf_translatemacevent(Cl_Mac_EventsType Cl_MacISOufEvt,cl_isouf_events* cl_isouf_event)
{
	switch(Cl_MacISOufEvt)
	{
		
		case EVT_CONSOLE_COMMAND_ISOUF_START :
		*cl_isouf_event = EVENT_ISOUF_START;
		break;
		case EVT_CONSOLE_COMMAND_ISOUF_STOP :
		*cl_isouf_event = EVENT_ISOUF_STOP;
		break;
		
		case EVT_CONSOLE_COMMAND_CLR_ALARM:
		*cl_isouf_event = EVENT_ISOUF_CLR_ALARM;
		break;
		case 	EVT_CONSOLE_COMMAND_SET_BLDPUMPRATE:
		*cl_isouf_event = EVENT_ISOUF_COMMAND_SET_BLDPUMPRATE;
		break;
		case EVT_CONSOLE_COMMAND_SET_BLDPMP_ON:
		*cl_isouf_event = EVENT_ISOUF_COMMAND_SET_BLDPMP_ON;
		break;
		case EVT_CONSOLE_COMMAND_SET_BLDPMP_OFF:
		*cl_isouf_event = EVENT_ISOUF_COMMAND_SET_BLDPMP_OFF;
		break;
		
		case EVT_CONSOLE_COMMAND_GET_DATA:
		*cl_isouf_event = EVENT_ISOUF_GET_DATA;
		break;
		case EVT_CONSOLE_COMMAND_SET_DATA:
		*cl_isouf_event = EVENT_ISOUF_SET_DATA;
		break;
		case EVT_CONSOLE_COMMAND_RINSE_START:
		*cl_isouf_event= EVENT_ISOUF_START_RINSE;
		break;
		case  EVT_CONSOLE_COMMAND_DIALYSIS_START:
		*cl_isouf_event= EVENT_ISOUF_START_DIALYSIS;
		break;
		case  EVT_TIMER_EXPIRED:
		
		break;
		case EVT_TICK_50M:
		*cl_isouf_event =   EVENT_ISOUF_TICK_50MS;
		break;
		case EVT_TICK_500M:
		*cl_isouf_event = EVENT_ISOUF_TICK_500MS;
		break;
		case EVT_TICK_SEC:
		*cl_isouf_event =   EVENT_ISOUF_TICK_SECOND;
		break;
		case EVT_TICK_MIN:
		*cl_isouf_event =   EVENT_ISOUF_TICK_MINUTE;
		break;
		case EVT_TICK_HOUR:
		*cl_isouf_event =   EVENT_ISOUF_TICK_HOUR;
		break;
		case EVT_ALARM_TRIGGERED:
		*cl_isouf_event =   EVENT_ISOUF_ALARM;
		break;
		case EVT_ALERT_TRIGGERED:
		*cl_isouf_event =   EVENT_ISOUF_ALERT;
		break;
		
		case EVT_CONSOLE_COMMAND_START_RETURN:
		*cl_isouf_event =   EVENT_ISOUF_STOP_RETURN;
		break;
		case EVT_CONSOLE_COMMAND_STOP_RETURN:
		*cl_isouf_event =   EVENT_ISOUF_STOP_RETURN;
		break;
		
		case EVT_CONSOLE_COMMAND_HEP_PMP_RATE:
		*cl_isouf_event =   EVENT_ISOUF_HEP_RATE;
		break;
		case EVT_CONSOLE_COMMAND_HEPARIN_START:
		*cl_isouf_event =   EVENT_ISOUF_HEP_ON;
		break;
		case EVT_CONSOLE_COMMAND_HEPARIN_STOP:
		*cl_isouf_event =   EVENT_ISOUF_HEP_OFF;
		
		break;
		
		case MACREQ_ISOUF_COMPLETED:
		*cl_isouf_event =   CL_ISOUF_STATE_POST_COMPLETION;
		break;
		case MACREQ_POST_DIALYSIS_ISOUF_REQ:
		*cl_isouf_event =   EVENT_ISOUF_START;
		break;
		case 	EVT_CONSOLE_COMMAND_HEPARIN_BOLUS:
		*cl_isouf_event = EVENT_ISOUF_HEAPARIN_BOLUS;
		break;
		
		case EVT_CONSOLE_COMMAND_SET_BLDPUMPINC_MANUAL:
		*cl_isouf_event = EVT_CONSOLE_COMMAND_SET_BLDPUMPINC_MANUAL;
		break;
		case EVT_CONSOLE_COMMAND_SET_BLDPUMPDEC_MANUAL:
		*cl_isouf_event = EVT_CONSOLE_COMMAND_SET_BLDPUMPDEC_MANUAL;
		break;
		
		case EVT_CONSOLE_COMMAND_BYPASS_ON:
		*cl_isouf_event = EVENT_ISOUF_BYPASS_ON;
		break;
		case EVT_CONSOLE_COMMAND_BYPASS_OFF:
		*cl_isouf_event = EVENT_ISOUF_BYPASS_OFF;
		break;
		default:
		break;
	}
	return CL_OK;
}


Cl_ReturnCodeType cl_isoufcompleted(){
	
	// stop uf pump
	cl_uf_controller(CL_UF_EVENT_STOP,0);
	
	cl_isouf_retcode =  sv_cntrl_deactivatepump(UFPUMP);
	isouf = false;
	return CL_OK;
}

Cl_ReturnCodeType cl_dialysate_ready_isouf(){

						sv_cntrl_deactivatepump(DCMOTOR1 );
						sv_cntrl_deactivatepump(DCMOTOR2 );
						Cl_bc_controller(BC_EVENT_STOP);
						
						
						
						sv_cntrl_poweroffheater();
						sv_cntrl_deactivate_valve(VALVE_ID1);
						cl_flow_switchcontroller(FLOW_SWITCH_CNTRLER_EVENT_STOP);

						sv_cntrl_deactivate_valve(VALVE_ID1);
						sv_cntrl_activate_valve(VALVE_ID18);
						//cl_hep_controller(CL_HEP_EVENT_START_NORMAL_DELIVERY,0);
						//cl_uf_controller(CL_UF_EVENT_START,0);
						
						Cl_Isouf_start();


return CL_OK;

}

 Cl_ReturnCodeType Cl_isouf_init(void)
 {
	 
	 cl_isouf_state = CL_ISOUF_STATE_IDLE;

	 //cl_isouf_event = EVENT_NULL_EVENT;
	 
	 Cl_ISOufTotalSecondscounter=0 ;
	 return CL_OK;
 }

