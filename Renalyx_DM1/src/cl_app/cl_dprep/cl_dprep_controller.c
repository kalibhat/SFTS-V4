/*
 * cl_dprep_controller.c
 *
 * Created: 12/27/2013 12:24:08 PM
 *  Author: user
 */ 

#include "cl_dprep_controller_include.h"
#include "cl_dprep_controller_proto.h"
#include "sv_interface_fun.h"


volatile Cl_Uint8Type dialyzerprime_flag =0;
extern Cl_ReturnCodeType  Cl_get_Temperature_Value(sensor_id id, float tempdata, float* correctedtemp);
extern Cl_ReturnCodeType cl_flow_switchcontroller(flowswitchcntrl_event_type cntrl_event);
extern Cl_Uint8Type sv_cs_setpotvalue(Cl_Uint32Type resistance) ;
extern Cl_Uint32Type Treatdata[ID_MAX_TREAT_PARAM] ;

 volatile Cl_Uint8Type flag=0;
 extern volatile Cl_Uint32Type hep_speed;
extern volatile float cond_final_cs3;


typedef union {
	Cl_Uint32Type fourbytedata;
	int16_t twobytedata;
	Cl_Uint8Type bytedata[4] ;
}StateDatatype;

extern Cl_ReturnCodeType  sv_nvmgetdata(Cl_Uint8Type,Cl_Uint8Type*);
extern Cl_ReturnCodeType  sv_nvmsetdata(Cl_Uint8Type ,Cl_Uint8Type* ,Cl_Uint8Type datasize);
extern Cl_ReturnCodeType cl_wait(Cl_Uint32Type ul_dly_ticks);
extern Cl_Uint32Type flow_rate;
extern void calibration_apt(int16_t sensordata);
extern volatile float temprature_final_value_1,temprature_final_value_2,temprature_final_value_3;

extern Cl_ConsoleMsgType Cl_ConsoleRxMsg;
extern Cl_Dprep_PrimeStates cl_dprep_prime_state;
extern Cl_BoolType BC_window; //test
extern Cl_BoolType g_testbcfreeze; //test
extern Cl_BoolType Current_sense_trigger; // test
extern float dummy3 ;
extern Cl_Dprep_States cl_dprepstatedummy;
extern Cl_AlarmThresholdType  Cl_alarmThresholdTable;
extern volatile float pressure_final_apt,pressure_final_vpt,pressure_final_ps1,pressure_final_ps2,pressure_final_ps3;
extern float res_temp_value;
Cl_BoolType dprep_heater_stable_status = false;
extern float cl_utilities_CalConductivity(uint16_t ,float);
extern Cl_Sys_statusType cl_sys_statbuffer;
int powerFailureFlag = 0;
Cl_Uint16Type dprep_powerFailSensorData = 0;
		
extern uint16_t Set_RC;                   // Gopal setting start point for set rc, in event manager

int stability_counter = 0;
int wait_timeout_counter = 0;
int state_counter = 0;
	int 	Cl_Dprepsecondscounter = 0;
	int 	Cl_DprepMinutescounter= 0;
	int 	Cl_Dprephourscounter= 0;
	int 	Cl_DprepTotalMinutescounter= 0;
	int 	Cl_DprepTotalhourscounter=0;
	int16_t Cl_Dprep_hepbolus_secondscounter=0;
	int16_t		Cl_Dprep_filling_secondscounter = 0;
	int16_t		Cl_Dprep_filling_Minutescounter = 0;
	int16_t		Cl_Dprep_filling_TotalMinutescounter = 0;
	
	int16_t		Cl_Dprep_Prime_secondscounter = 0;
	int16_t		Cl_Dprep_Prime_Minutescounter = 0;
	int16_t		Cl_Dprep_Prime_TotalMinutescounter = 0;
	
	int16_t		Cl_Dprep_DialyserPrime_secondscounter = 0;
	int16_t		Cl_Dprep_DialyserPrime_Minutescounter = 0;
	int16_t		Cl_Dprep_DialyserPrime_TotalMinutescounter = 0;
	Cl_Uint16Type hep_bolus_flag = 0,dprep_count=0;
float temp3_cel_backup=0;
Cl_BoolType cl_temp3_stable_confirm = false;
Cl_BoolType cl_temp3_stable= false,cl_temp3_range_stable= false, cl_temp3_37stable = false;
Cl_BoolType Cl_Prime_state ;
static Cl_Uint16Type cl_50ms_timer = 0;
Cl_PatientStateType Cl_PatientState = CL_DPREP_PATIENT_STATE_NOT_CONNECTED;
Cl_Dprep_States cl_dprepstate = CL_DPREP_STATE_IDLE;
Cl_Dprep_BypassStates cl_dprepbypassstate = CL_DPREP_BYPASS_NONE;
Cl_Dprep_filling_States cl_dprepfillingState = CL_DPREP_FILLING_IDLE;
DprepAlarmsType Cl_DprepAlarmTable[CL_DPREP_ALRM_MAX] =
{
	
	{BLOODDOOR_STATUS_OPEN,CL_ALARM_ALARM,false,false,false},
	{HOLDER1STATUS_CLOSED,CL_ALARM_ALARM,false,false,false},
	{HOLDER2STATUS_CLOSED,CL_ALARM_ALARM,false,false,false},
	{HOLDER1STATUS_OPEN,CL_ALARM_ALARM,false,false,false},
	{HOLDER2STATUS_OPEN,CL_ALARM_ALARM,false,false,false},
	{ACID_IN_CLOSED,CL_ALARM_ALARM,false,false,false},
	{BICARB_IN_CLOSED,CL_ALARM_ALARM,false,false,false},
	{COND_STATUS_LOW,CL_ALARM_ALARM,false,false,false},
	{COND_STATUS_HIGH,CL_ALARM_ALARM,false,false,false},
	{COND_DAC_OPEN,CL_ALARM_ALARM,false,false,false},
	{COND_DAC_RO,CL_ALARM_ALARM,false,false,false},
	{COND_DAC_HIGH,CL_ALARM_ALARM,false,false,false},
	{FLOW_NO_FLOW,CL_ALARM_ALARM,false,false,false},
	{FLOW_LOW_FLOWRATE,CL_ALARM_ALARM,false,false,false},
	{FLOW_HIGH_FLOWRATE,CL_ALARM_ALARM,false,false,false},    // manav02 added
	{ABD_EVENT,CL_ALARM_ALARM,false,false,false},
	{BD_EVENT,CL_ALARM_ALARM,false,false,false},
	{BLD_EVENT,CL_ALARM_ALARM,false,false,false},
	{APTSTATUS_HIGH,CL_ALARM_ALARM,false,false,false},
	{VPTSTATUS_HIGH,CL_ALARM_ALARM,false,false,false},
	{PS1_HIGH_THRESHOLD,CL_ALARM_ALARM,false,false,false},
	{PS1_LOW_THRESHOLD,CL_ALARM_ALARM,false,false,false},
	{PS2_HIGH_THRESHOLD,CL_ALARM_ALARM,false,false,false},
	{PS2_LOW_THRESHOLD,CL_ALARM_ALARM,false,false,false},
	{PS3_HIGH_THRESHOLD,CL_ALARM_ALARM,false,false,false},
	{PS3_LOW_THRESHOLD,CL_ALARM_ALARM,false,false,false},
	{TEMP1_HIGH_THRESHOLD,CL_ALARM_ALARM,false,false,false},
	{TEMP1_LOW_THRESHOLD,CL_ALARM_ALARM,false,false,false},	
	{TEMP2_HIGH_THRESHOLD,CL_ALARM_ALARM,false,false,false},
	{TEMP2_LOW_THRESHOLD,CL_ALARM_ALARM,false,false,false},
	{TEMP3_HIGH_THRESHOLD,CL_ALARM_ALARM,false,false,false},
	{TEMP3_LOW_THRESHOLD,CL_ALARM_ALARM,false,false,false},
	{FPCURRENTSTATUS,CL_ALARM_ALARM,false,false,false},
    {POWER_FAILURE,CL_ALARM_ALARM,false,false,false},
	{BP_STOPPED,CL_ALARM_ALARM,false,false,false},	
	{BC_ALARM,CL_ALARM_ALARM,false,false,false}	          // manav03 added
};

 int Cl_dprepsecondscounter =0 ,Cl_dprepMinutescounter=0, Cl_dprephourscounter=0;
 int Cl_dprepTotalMinutescounter=0, Cl_dprepTotalhourscounter=0;

 Cl_ReturnCodeType Cl_dprep_init(void)
 {
	 

 	Cl_Dprepsecondscounter = 0;
 	Cl_DprepMinutescounter= 0;
 	Cl_Dprephourscounter= 0;
 	Cl_DprepTotalMinutescounter= 0;
 	Cl_DprepTotalhourscounter=0;
	Cl_Dprep_hepbolus_secondscounter=0;
	Cl_Dprep_filling_secondscounter = 0;
	Cl_Dprep_filling_Minutescounter = 0;
	Cl_Dprep_filling_TotalMinutescounter = 0;
powerFailureFlag = 0;
	Cl_Dprep_Prime_secondscounter = 0;
	Cl_Dprep_Prime_Minutescounter = 0;
	Cl_Dprep_Prime_TotalMinutescounter = 0;
	Cl_Dprep_DialyserPrime_secondscounter = 0;
	Cl_Dprep_DialyserPrime_Minutescounter = 0;
	Cl_Dprep_DialyserPrime_TotalMinutescounter = 0;
	hep_bolus_flag = 0;
	dprep_count=0;
	temp3_cel_backup=0;
	cl_temp3_stable_confirm = false;
	cl_temp3_stable= false;
	cl_temp3_range_stable= false;
	 cl_temp3_37stable = false;
	Cl_Prime_state ;
	 cl_50ms_timer = 0;
	 Cl_PatientState = CL_DPREP_PATIENT_STATE_NOT_CONNECTED;
	 cl_dprepstate = CL_DPREP_STATE_IDLE;
	 cl_dprepfillingState = CL_DPREP_FILLING_IDLE;
	dprep_heater_stable_status = false;
	 stability_counter = 0;
	  wait_timeout_counter = 0;
	  state_counter = 0;
	cl_dprep_primeInit();

	 return CL_OK;
 }
 

 
Cl_ReturnCodeType Cl_dprep_controller(Cl_Mac_EventsType Cl_MacDprepEvent)
{
	
	Cl_ReturnCodeType  Cl_dprepretcode = CL_OK;
	Cl_Dprep_Events cl_dprepevent = EVENT_DPREP_EVENT_NULL;
	Cl_ConsoleTxCommandtype command = CON_TX_COMMAND_COMMAND_MAX;
	Cl_Uint8Type data = 0;
	Cl_Uint8Type datasize = 0;
	Cl_Uint8Type dataarray[4] =  {0,0,0,0};
	Cl_Uint8Type systemdataarray[40] =  {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
	cl_PrepDatatype tempdata;
	Cl_BoolType cl_status;
	cl_PrepDatatype cl_PrepData;															
	Cl_BoolType alarmstatus1 = false,alarmstatus2 = false,alarmstatus3 = false,flowstatus = false;
	Cl_Uint16Type static cl_temp3;
	Cl_Uint16Type temp_temp3;
	float temp3_cel=0;
	Cl_Uint16Type temp = 0;
	Cl_Uint16Type tempcount;
	StateDatatype abfdata;
	Cl_Uint8Type abfdataarray[5] =  {0,0,0,0,0};
	cl_dprep_translatemacevent( Cl_MacDprepEvent, &cl_dprepevent);
	
	
	switch(cl_dprepevent)
	{
		case EVENT_DPREP_TICK_SECOND:
		//Cl_dprepretcode = Cl_bc_controller(BC_EVENT_SECOND);
		//Cl_dprepretcode = cl_dprep_primecontroller(CL_DPREP_PRIME_PRIME_TICK_SEC,0);
		cl_dprepstatedummy = cl_dprepstate;
		break;
		case EVENT_DPREP_TICK_50MS:
		//Cl_dprepretcode = Cl_bc_controller(BC_EVENT_50MS);
		break;
		case EVENT_DPREP_COMMAND_SET_BLDPMP_ON:
		Cl_dprepretcode =  Cl_AlarmActivateAlarms(BLOODDOOR_STATUS_OPEN,true);
		cl_bp_controller(CL_BP_EVENT_START,1);
		break;
		case EVENT_DPREP_COMMAND_SET_BLDPMP_OFF:
		Cl_dprepretcode =  Cl_AlarmActivateAlarms(BLOODDOOR_STATUS_OPEN,false);
		cl_bp_controller(CL_BP_EVENT_STOP,1);
		if(cl_dprepstate == CL_DPREP_STATE_DIALISER_PRIME || cl_dprepstate == CL_DPREP_STATE_DIALISER_PRIME_STOPPED){
		Cl_Alarm_TriggerAlarm(BP_STOPPED,1);	
		}
		
		
		
		break;
		
		case EVENT_DPREP_MUTE :
		sv_cntrl_nobuzzer();
		break;
		
		case  EVENT_DPREP_COMMAND_SET_BLDPUMPRATE:
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
			case EVENT_DPREP_HEP_PMP_START:
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
			case EVENT_DPREP_HEP_PMP_STOP:
			cl_hep_controller(CL_HEP_EVENT_STOPBOLUS,0);
			break;
			case EVENT_HEAPARIN_BOLUS:
			cl_hep_controller(CL_HEP_EVENT_STARTBOLUS,0);
			break;
					
// 		case EVENT_DPREP_HEP_PMP_STOP:
// 		break;
// 		case EVENT_HEAPARIN_BOLUS:
// 		break;
		case EVENT_DPREP_HEP_PMP_RATE:
		
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
		
		cl_dprepevent == EVENT_DPREP_EVENT_NULL;
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
		
		cl_dprepevent == EVENT_DPREP_EVENT_NULL;
		break;
		

		default:break;
		
	}
	if(cl_dprepevent == EVENT_DPREP_TICK_SECOND)
	{

		//Cl_dprepretcode =  Cl_AlarmResetAlarm( SENSOR_TEMP3STATUS );
		//Cl_dprepretcode =  Cl_AlarmResetAlarm( SENSOR_TEMP2STATUS );
		Cl_dprepretcode =  Cl_AlarmResetAlarm( FLOW_NO_FLOW );
		//	Cl_rinseretcode =  Cl_AlarmResetAlarm( FLOWSTATUS_FLOWOFF );
	}


 	switch(cl_dprepstate)
	{

		case CL_DPREP_STATE_INIT:
		break;
		case CL_DPREP_STATE_IDLE:
		case CL_DPREP_STATE_STOPPED:
		switch (cl_dprepevent)
		{
			
				case EVENT_DPREP_CLR_ALARM:             // manav03 added
								
								if(powerFailureFlag == 1){
					
					Cl_SysStat_GetSensor_Status_Query(SENSOR_POWER_FAILURE,&dprep_powerFailSensorData);
					
								if(dprep_powerFailSensorData == 0){
						
						powerFailureFlag = 0;
						command = CON_TX_COMMAND_ALARM_CLEARED;
							Cl_SendDatatoconsole(command,&data,0);
							sv_cntrl_resetyellowalarm();
							sv_cntrl_resetredalarm();
							sv_cntrl_nobuzzer();
							Cl_Dprep_ResetAlertsforReassertion();
					}
					}
					else{			
								
								command = CON_TX_COMMAND_ALARM_CLEARED;
							    Cl_SendDatatoconsole(command,&data,0);
							
					
										sv_cntrl_resetyellowalarm();
										sv_cntrl_resetredalarm();
										sv_cntrl_nobuzzer();
										Cl_Dprep_ResetAlertsforReassertion();
					}
								break;
			
			case EVENT_DPREP_START_RECIRC:
						if((cl_dprep_prime_state != CL_DPREP_PRIME_STATE_PRIMING) || (cl_dprep_prime_state != CL_DPREP_PRIME_STATE_PRIME_RCIRC_STARTED) || (cl_dprep_prime_state !=  CL_DPREP_PRIME_STATE_DIALYSER_PRIMING))
						{
							Cl_SendDatatoconsole(CON_TX_COMMAND_PRINTTEXT,"RCIRC",6);
							cl_dprep_primecontroller(CL_DPREP_PRIME_RCIRC_START,0);
							Cl_DprepAlarmTable[CL_DPREP_ALARM_BLOODDOOR_STATUS_OPEN].IsRaised = false;
						}
				break;
			case EVENT_DPREP_COMMAND_GET_DATA:

				if(Cl_ConsoleRxMsg.msgready == true)
				{
					switch(Cl_ConsoleRxMsg.data.byte[0])
					{
						case	CON_RX_PARAM_DATA_PRIME_STATUS:
						command = CON_TX_COMMAND_SYSDATA;
						dataarray[0] = CON_TX_PARAM_DATA_PRIME_STATUS;
						Cl_dprepretcode = (Cl_Uint8Type)sv_nvmgetdata(NV_NVM_PRIME_STATUS, &dataarray[1]);
						Cl_dprepretcode = Cl_SendDatatoconsole(command,&dataarray,2);
						break;
						case	CON_RX_PARAM_DATA_DIALYSIS_STATUS:
						command = CON_TX_COMMAND_SYSDATA;
						dataarray[0] = CON_TX_PARAM_DATA_DIALYSIS_STATUS;
						Cl_dprepretcode = (Cl_Uint8Type)sv_nvmgetdata(NV_NVM_DIALYSIS_STATUS, &dataarray[1]);
						Cl_dprepretcode = Cl_SendDatatoconsole(command,&dataarray,2);
						break;
						default:
						break;
					}
				}
			break;
	/*		case EVENT_DPREP_COMMAND_SET_DATA:
					if(Cl_ConsoleRxMsg.msgready == true)
							{
										cl_Datastreamtype cl_temp  ;	
										cl_temp.bytearray[0] = Cl_ConsoleRxMsg.data.byte[1];
										cl_temp.bytearray[1] = Cl_ConsoleRxMsg.data.byte[2];
										cl_temp.bytearray[2] = Cl_ConsoleRxMsg.data.byte[3];
										cl_temp.bytearray[3] = Cl_ConsoleRxMsg.data.byte[4];
											
								switch(Cl_ConsoleRxMsg.data.byte[0])
								{
										case ID_dflow:
										break;
										case ID_settemp:
										break;
										case ID_heprate:
										break;
										case ID_setcond:
										break;
										case ID_ufrate:
										break;
										case ID_ufgoal:
										break;
										case ID_bolusvol:
										break;
										case ID_bloodrate:
										break;
										case ID_hepstoptime:
										break;
										case ID_syringetype:
										break;
										case ID_heparincheck:
										break;
										case ID_minufrate:
										break;
										case ID_treattime:
										break;
										case ID_bloodratereturn:
										break;
										case ID_bloodratetreat:
										break;
										case ID_tempulimit:
											Cl_alarmThresholdTable.temp3_high_threshold =  (cl_temp.word)/10;
										break;
										case ID_templlimit:
											Cl_alarmThresholdTable.temp3_low_threshold =  (cl_temp.word)/10;
										break;
										case ID_tmpllimit:
										Cl_alarmThresholdTable.tmp_low_threshold =  cl_temp.word;
										break;
										case ID_tmpulimit:
										Cl_alarmThresholdTable.tmp_high_threshold =  cl_temp.word;
										break;
										case ID_vptllimit:
											Cl_alarmThresholdTable.vpt_low_threshold =  cl_temp.word;
										break;
										case ID_vptulimit:
											Cl_alarmThresholdTable.vpt_high_threshold =  cl_temp.word;
										break;
										case ID_ufllimit:
										break;
										case ID_ufulimit:
										break;
										case ID_dflowllimit:
										break;
										case ID_dflowulimit:
										break;
										case ID_condllimit:
										Cl_alarmThresholdTable.cond_low_threshold =  cl_temp.word;
										break;
										case ID_condulimit:
										Cl_alarmThresholdTable.cond_high_threshold =  cl_temp.word;
										break;
										case ID_aptllimit:
											Cl_alarmThresholdTable.apt_low_threshold =  cl_temp.word;
										break;
										case ID_aptulimit:
										Cl_alarmThresholdTable.apt_high_threshold =  cl_temp.word;
										break;

									default:break;
								}
							}
						
	
			break;   */
			case EVENT_DPREP_DIALYSIS_PREP:
			
			Cl_dprepretcode = Cl_Dprep_StartPreparation();
			if(Cl_dprepretcode == CL_REJECTED)
			{
				return CL_OK;//Cl_dprepretcode;   // manav03 added
			}
			break;
			case EVENT_DPREP_MIXING_PREP_START:
			Cl_dprepretcode = Cl_Dprep_StartPreparation();
			if(Cl_dprepretcode == CL_REJECTED)
			{
				return CL_OK;//Cl_dprepretcode;   // // manav03 added
			}
			break;
			case EVENT_DPREP_ALARM:
			Cl_dprepretcode = Cl_Dprep_ProcessAlarms();
			break;
			case EVENT_DPREP_START_PRIME:
			if(cl_dprep_prime_state != CL_DPREP_PRIME_STATE_PRIMING)
			{
				Cl_SendDatatoconsole(CON_TX_COMMAND_PRINTTEXT,"PRIME1",6);
				cl_dprep_activate_prime_related_alarms();
				cl_dprep_primecontroller(CL_DPREP_PRIME_PRIME_START,0);
				Cl_DprepAlarmTable[CL_DPREP_ALARM_BLOODDOOR_STATUS_OPEN].IsRaised = false;
			}
			
			break;
			default:
			break;
		}
		break;
		case CL_DPREP_STATE_DPREP_FILLING:
		switch (cl_dprepevent)
		{
			
			case EVENT_DPREP_HEP_PMP_START:
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
								cl_hep_controller(CL_HEP_EVENT_START_NORMAL_DELIVERY,0);					
							}
								
						}
				
			
			break;
			case EVENT_DPREP_HEP_PMP_STOP:
			cl_hep_controller(CL_HEP_EVENT_STOP_NORMAL_DELIVERY,0);
			break;
			case EVENT_HEAPARIN_BOLUS:		
			cl_hep_controller(CL_HEP_EVENT_STARTBOLUS,0);
			break;

			case EVENT_DPREP_START_RECIRC:
						if((cl_dprep_prime_state != CL_DPREP_PRIME_STATE_PRIMING) || (cl_dprep_prime_state != CL_DPREP_PRIME_STATE_PRIME_RCIRC_STARTED) || (cl_dprep_prime_state !=  CL_DPREP_PRIME_STATE_DIALYSER_PRIMING))
						{
							Cl_SendDatatoconsole(CON_TX_COMMAND_PRINTTEXT,"RCIRC",6);
							cl_dprep_primecontroller(CL_DPREP_PRIME_RCIRC_START,0);
							Cl_DprepAlarmTable[CL_DPREP_ALARM_BLOODDOOR_STATUS_OPEN].IsRaised = false;
						}
			break;
			case EVENT_DPREP_COMMAND_GET_DATA:

				if(Cl_ConsoleRxMsg.msgready == true)
				{
					switch(Cl_ConsoleRxMsg.data.byte[0])
					{
						case	CON_RX_PARAM_DATA_PRIME_STATUS:
						command = CON_TX_COMMAND_SYSDATA;
						dataarray[0] = CON_TX_PARAM_DATA_PRIME_STATUS;
						Cl_dprepretcode = (Cl_Uint8Type)sv_nvmgetdata(NV_NVM_PRIME_STATUS, &dataarray[1]);
						Cl_dprepretcode = Cl_SendDatatoconsole(command,&dataarray,2);
						break;
						case	CON_RX_PARAM_DATA_DIALYSIS_STATUS:
						command = CON_TX_COMMAND_SYSDATA;
						dataarray[0] = CON_TX_PARAM_DATA_DIALYSIS_STATUS;
						Cl_dprepretcode = (Cl_Uint8Type)sv_nvmgetdata(NV_NVM_DIALYSIS_STATUS, &dataarray[1]);
						Cl_dprepretcode = Cl_SendDatatoconsole(command,&dataarray,2);
						break;
						default:
						break;
					}
				}
			break;
			case EVENT_DPREP_COMMAND_SET_DATA:
					 if(Cl_ConsoleRxMsg.msgready == true)
					 {
						 
						 
						 Cl_ConsoleRxDataType dataId;
						 Cl_Uint8Type count;
						 dataId = Cl_ConsoleRxMsg.data.byte[0];
						 for (count =0; count < Cl_ConsoleRxMsg.datasize;count++)
						 {
							 cl_PrepData.bytearray[count] = Cl_ConsoleRxMsg.data.byte[count+1];
							 
						 }
						 Cl_Dprep_setdata(dataId, cl_PrepData,count);
						 
						 
						 
					 }
	
			break;
			case EVENT_DPREP_TICK_50MS:
			
			cl_50ms_timer++;
			if(cl_50ms_timer == 2)
			{
				cl_50ms_timer =0;
			//	sv_cntrl_deactivate_valve(VALVE_ID19);
			}
			
			break;
			case EVENT_DPREP_TICK_500MS:
				//UpdateHeaterControls();
	
			break;
			case EVENT_DPREP_TICK_MINUTE:
				Cl_Dprep_ResetAlertsforReassertion();
				Cl_SysStat_GetSensor_Status_Query(SENSOR_TEMP3STATUS,&temp_temp3);
// 				int16_t sensordata;
// 				uint16_t cal_data  = (402 *100* sensordata)/(2*32768);			// resistance of PT100
// 				res_temp_lookuptable(cal_data);									// temperature from look up table in 4 digits
// 				temp_temp3 = res_temp_value/100;									// Temp3 value in XX.yy format
// //				temp_temp3 = temp_temp3 - 3.1 + 1.4;										// sensor offset
// 				temp_temp3 = temp_temp3- 0.4;										// sensor offset
				temp3_cel = (float)temp_temp3/100;
				
// 					{
// 						float temp1=0;
// //						calibration_tmp(temp_temp3,TS3);
// //						Cl_get_Temperature_Value(TS3_SPI, temprature_final_value_3 , &temp1);
// 						temp3_cel = temp_temp3;
// 					}
// 					//calibration_tmp(temp_temp3,TS3);
// 					//temp3_cel = temprature_final_value_3;
// 					//temp3_cel = 0.0000116 * temp_temp3 *temp_temp3 + 0.0035 *temp_temp3 + 11.157;
				if((temp3_cel) > 36.8 && (temp3_cel < 37.2))
				{
					if( cl_temp3_37stable == true) 
					{
					 cl_temp3_stable_confirm = true;	
					}
					else
					{
					 cl_temp3_37stable = true;
					 cl_temp3_stable_confirm = false;
					}
				}
				else
				{
					cl_temp3_37stable = false;
					cl_temp3_stable_confirm = false;
					
				}
			


			break;
			case EVENT_DPREP_TICK_SECOND:
			#if 0
						Cl_SysStat_GetSensor_Status_Query(SENSOR_TEMP3STATUS,&temp_temp3);
					
					cal_data  = (402 *100* sensordata)/(2*32768);			// resistance of PT100
					res_temp_lookuptable(cal_data);									// temperature from look up table in 4 digits
					temp_temp3 = res_temp_value/100;									// Temp3 value in XX.yy format
					temp_temp3 = temp_temp3 - 0.6;										// sensor offset

					temp3_cel = 0.0000116 * temp_temp3 *temp_temp3 + 0.0035 *temp_temp3 + 11.157;
					
					
					if(!(Cl_Dprep_filling_secondscounter%20))
					{
										if(((temp3_cel_backup < temp3_cel) &&(temp3_cel_backup > temp3_cel - 0.3)) || ((temp3_cel_backup < temp3_cel + 0.3)&&(temp3_cel_backup > temp3_cel)))
										{
											cl_temp3_stable = true;
										}
										else
										{
											cl_temp3_stable =false;
										}
										if(((temp3_cel_backup < temp3_cel) &&(temp3_cel_backup > temp3_cel - 1)) || ((temp3_cel_backup < temp3_cel + 1)&&(temp3_cel_backup > temp3_cel)))
										{
											cl_temp3_range_stable = true;
										}
										else
										{
											cl_temp3_range_stable =false;
										}
										
										temp3_cel_backup = temp3_cel;
					}
					#endif
					Cl_Dprep_filling_secondscounter++;
					if(Cl_Dprep_filling_secondscounter == 60)
					{
						UpdateDprepFillingMinuteTick();
					}
					
					if(Cl_Dprep_filling_secondscounter == 30){ // activate flow alarms only after 30 seconds into rinse
						
						
						Cl_dprepretcode =  Cl_AlarmActivateAlarms(FLOW_NO_FLOW,true );
						Cl_dprepretcode =  Cl_AlarmActivateAlarms(FLOW_LOW_FLOWRATE,true );
						Cl_dprepretcode =  Cl_AlarmActivateAlarms(FLOW_HIGH_FLOWRATE,true );
						
						
					}
					
					
					Cl_Dprep_SendPrepStateData(DIALYSIS_PREP_DATA);
				//	Cl_Dprep_ResetAlertsforReassertion();
				//	Cl_dprepretcode = Cl_Dprep_ProcessAlarms();

					if(Cl_Dprep_CheckforfillingCompletion() == CL_OK )
					{
						sv_cntrl_setgreenalarm();
						Cl_Uint32Type temp = Treatdata[ID_dflow];
						Cl_Uint16Type potvalue = 0;
						//if(cl_dprep_prime_state == CL_DPREP_PRIME_STATE_PRIME_COMPLETED)
						if(cl_dprep_prime_state == CL_DPREP_PRIME_STATE_PRIME_RCIRC_COMPLETED)
						{
						//	SetHeaterState(CL_HEATER_STATE_CLOSED_HEATING);
						//	sv_cntrl_enable_loopback();
							// sv_prop_stopmixing();
// 							switch (temp)
// 							{
// 								case 800:
// 								potvalue = (612 * 1024)/10000;
// 								sv_cs_setpotvalue(potvalue);
// 								break;
// 								
// 								case 500:
// 								Cl_SendDatatoconsole(CON_TX_COMMAND_PRINTTEXT,"500",3);
// 								potvalue = (670 * 1024)/10000;
// 								sv_cs_setpotvalue(potvalue);
// 								break;
// 								
// 								case 300:
// 								potvalue = (800 * 1024)/10000;
// 								sv_cs_setpotvalue(potvalue);
// 								break;
// 								
// 								default:
// 								break;
// 							}
							 sv_cntrl_enable_bypass();
							 cl_dprepbypassstate = CL_DPREP_BYPASS_ON;
							 
							cl_dprepstate = CL_DPREP_STATE_POST_PRIME_STANDBY;
									 
						}
						else
						{
// 							switch (temp)
// 							{
// 								case 800:
// 								potvalue = (612 * 1024)/10000;
// 								sv_cs_setpotvalue(potvalue);
// 								break;
// 								
// 								case 500:
// 								Cl_SendDatatoconsole(CON_TX_COMMAND_PRINTTEXT,"500",3);
// 								potvalue = (670 * 1024)/10000;
// 								sv_cs_setpotvalue(potvalue);
// 								break;
// 								
// 								case 300:
// 								potvalue = (800 * 1024)/10000;
// 								sv_cs_setpotvalue(potvalue);
// 								break;
// 								
// 								default:
// 								break;
// 							}
							sv_cntrl_enable_bypass();
							cl_dprepbypassstate = CL_DPREP_BYPASS_ON;

							cl_dprepstate = CL_DPREP_STATE_DPREP_FILLING_DONE;
							
						}
						
						//	Cl_rinseretcode = (Cl_ReturnCodeType)sv_setflowpath(Default path ?);//set appropirate flow path configuration
					//	Cl_dprepretcode = sv_cntrl_poweroffheater();							
					//	Cl_dprepretcode =  sv_cntrl_deactivatepump(DCMOTOR1);
					//	Cl_dprepretcode =  sv_cntrl_deactivatepump(DCMOTOR2);
						command = CON_TX_COMMAND_DIALYSATE_FILLING_COMPLETED;
						data = (Cl_Uint8Type)COMMAND_RESULT_SUCCESS;
						Cl_Dprep_filling_secondscounter = 0;
						Cl_Dprep_filling_Minutescounter = 0;
						Cl_Dprep_filling_TotalMinutescounter = 0;

						Cl_deactivate_prep_alarms();


					//	Cl_dprepretcode =  Cl_AlarmConfigureAlarmType(COND_STATUS_LOW,BELOW,13.0,0,0);
						// Appy treatment cond alarm and temerature alarm  limits here
						// Cl_dprepretcode =  Cl_AlarmActivateAlarms(COND_STATUS_LOW,true );
							
						Cl_dprepretcode = Cl_SendDatatoconsole(command,&data,1);
					}
					
			break;
			case EVENT_DPREP_MIXING_PREP_STOP:
			case EVENT_DPREP_STOP_DIALYSATE_FILL:
					Cl_dprep_StopMixing();
					cl_dprepstate = CL_DPREP_STATE_STOPPED;
			break;
			break;
			case EVENT_DPREP_ALARM:
					Cl_dprepretcode = Cl_Dprep_ProcessAlarms();
			break;
			case EVENT_DPREP_ALERT:
					Cl_Alarm_GetAlarmStatus(FPCURRENTSTATUS,&alarmstatus1);
					if(alarmstatus1)
					{
						//Cl_dprepretcode = Cl_bc_controller(BC_EVENT_CS);
					//	sv_cntrl_activate_valve(VALVE_ID19);
						cl_50ms_timer  = 1;
					
					}
					Cl_Alarm_GetAlarmStatus(TEMP2_HIGH_THRESHOLD,&alarmstatus3);
					if(alarmstatus3)
					{
						//	UpdateHeaterControls();
					}
			
					Cl_Alarm_GetAlarmStatus(FLOW_NO_FLOW,&flowstatus);
					//	if(flowstatus)
					//	{
					//		Cl_rinseretcode = sv_cntrl_poweroffheater();
					//		Cl_SendDatatoconsole(CON_TX_COMMAND_PRINTTEXT,"FLOW_OFF",8);
					//	}
			break;
			case EVENT_DPREP_START_PRIME:
			if(cl_dprep_prime_state != CL_DPREP_PRIME_STATE_PRIMING)
			{
				Cl_SendDatatoconsole(CON_TX_COMMAND_PRINTTEXT,"PRIME1",6);
				cl_dprep_primecontroller(CL_DPREP_PRIME_PRIME_START,0);
				Cl_DprepAlarmTable[CL_DPREP_ALARM_BLOODDOOR_STATUS_OPEN].IsRaised = false;
			}
			
			break;
			
				case EVENT_DPREP_CLR_ALARM:             // manav03 added
								
											if(powerFailureFlag == 1){
					
					Cl_SysStat_GetSensor_Status_Query(SENSOR_POWER_FAILURE,&dprep_powerFailSensorData);
					
								if(dprep_powerFailSensorData == 0){
						
						powerFailureFlag = 0;
						command = CON_TX_COMMAND_ALARM_CLEARED;
							Cl_SendDatatoconsole(command,&data,0);
							sv_cntrl_resetyellowalarm();
							sv_cntrl_resetredalarm();
							sv_cntrl_nobuzzer();
							Cl_Dprep_ResetAlertsforReassertion();
					}
					}
					else{			
					
								
								command = CON_TX_COMMAND_ALARM_CLEARED;
							    Cl_SendDatatoconsole(command,&data,0);
							
					
										sv_cntrl_resetyellowalarm();
										sv_cntrl_resetredalarm();
										sv_cntrl_nobuzzer();
										Cl_Dprep_ResetAlertsforReassertion();
					}
								break;

		}
		break;
		case CL_DPREP_STATE_DREP_PRIME_ALARM_BYPASS:
					switch (cl_dprepevent)
					{
						case EVENT_DPREP_START_DIALISER_PRIME:
					
							//	if(cl_dprepbypassstate == CL_DPREP_BYPASS_ON)
								
								sv_cntrl_disable_loopback();
								sv_cntrl_disable_bypass();
								cl_dprepstate = CL_DPREP_STATE_DIALISER_PRIME;
								 
						break;
						case EVENT_DPREP_TICK_MINUTE:
						Cl_dprepretcode = Cl_Dprep_ResetAlertsforReassertion();
						break;
						case EVENT_DPREP_ALARM:
						Cl_dprepretcode = Cl_Dprep_ProcessAlarms();
						break;
						default:break;
					}
		break;
		
		case CL_DPREP_STATE_CRITICAL_ALARM:
					switch (cl_dprepevent)
					{
						case EVENT_DPREP_DIALYSIS_PREP:
						//Cl_dprepretcode = Cl_Dprep_StartPreparation();
						if(Cl_dprepretcode == CL_REJECTED)
							{
								return Cl_dprepretcode;
							}
						break;
						case EVENT_DPREP_START_DIALISER_PRIME:
							//	if(cl_dprepbypassstate == CL_DPREP_BYPASS_ON)
							sv_prop_startmixing();
							Cl_bc_controller(BC_EVENT_RESUME);
								Cl_Dprep_StartDialyserPrime();
								sv_cntrl_disable_loopback();
								 sv_prop_startmixing();
						break;
						case EVENT_DPREP_MIXING_PREP_START:
						Cl_dprepretcode = Cl_Dprep_StartPreparation();
						if(Cl_dprepretcode == CL_REJECTED)
							{
								return Cl_dprepretcode;
							}
						break;
						case EVENT_DPREP_TICK_MINUTE:
						Cl_dprepretcode = Cl_Dprep_ResetAlertsforReassertion();
						break;
						case EVENT_DPREP_ALARM:
						Cl_dprepretcode = Cl_Dprep_ProcessAlarms();
						break;
						case EVENT_DPREP_CLR_ALARM:
					
					if(powerFailureFlag == 1){
					
					Cl_SysStat_GetSensor_Status_Query(SENSOR_POWER_FAILURE,&dprep_powerFailSensorData);
					
								if(dprep_powerFailSensorData == 0){
						
						powerFailureFlag = 0;
						command = CON_TX_COMMAND_ALARM_CLEARED;
							Cl_SendDatatoconsole(command,&data,0);
						sv_cntrl_resetyellowalarm();
						sv_cntrl_resetredalarm();
						sv_cntrl_nobuzzer();
						Cl_Dprep_ResetAlertsforReassertion();
						sv_cntrl_disable_bypass();
					}
					}
					else{
						
							command = CON_TX_COMMAND_ALARM_CLEARED;
							Cl_SendDatatoconsole(command,&data,0);
						sv_cntrl_resetyellowalarm();
						sv_cntrl_resetredalarm();
						sv_cntrl_nobuzzer();
						Cl_Dprep_ResetAlertsforReassertion();
						sv_cntrl_disable_bypass();
					}
						
						break;						
						
						case EVENT_DPREP_TICK_SECOND:
// 							Cl_SendDatatoconsole(CON_TX_COMMAND_PRINTTEXT,"ENTER",8);
// 						for (tempcount = 0 ; tempcount < CL_DPREP_ALRM_MAX ; tempcount++)
// 						{
// 							
// 							Cl_Alarm_GetAlarmStatus(Cl_DprepAlarmTable[tempcount].Cl_DprepAlarmId,&Cl_DprepAlarmTable[tempcount].IsActive);
// 							if (Cl_DprepAlarmTable[tempcount].IsActive)
// 							{
// 								Cl_SendDatatoconsole(CON_TX_COMMAND_PRINTTEXT,"ENTER_1",8);
// 								Cl_Dprep_Stoppreparation();
// 								Cl_dprepretcode = CL_DPREP_STATE_CRITICAL_ALARM;
// 								tempcount=0;
// 								break;
// 							}
// 							else if (tempcount == CL_DPREP_ALRM_MAX - 1)
// 							{
// 								if(!Cl_DprepAlarmTable[tempcount].IsActive)
// 								{
// 									Cl_dprepretcode = Cl_Dprep_StartPreparation();
// 									if(Cl_dprepretcode == CL_REJECTED)
// 										{
// 											return Cl_dprepretcode;
// 										}
// 										else
// 										{
// 											Cl_SendDatatoconsole(CON_TX_COMMAND_PRINTTEXT,"SAFE",8);
// 											sv_cntrl_resetyellowalarm();
// 											sv_cntrl_resetredalarm();
// 											sv_cntrl_nobuzzer();
// 											Cl_dprepretcode = CL_DPREP_STATE_DPREP_FILLING;
// 											
// 										}
// 
// 								}
// 							}
// 						}
						break;
						default:break;
					}
		break;
		case CL_DPREP_STATE_DPREP_FILLING_DONE:
					switch (cl_dprepevent)
					{
						case EVENT_DPREP_START_DIALISER_PRIME:
						//sv_prop_startmixing();
								Cl_Dprep_StartDialyserPrime();
								sv_cntrl_disable_loopback();
								cl_dprepstate = CL_DPREP_STATE_DIALISER_PRIME;
						break;
						case EVENT_DPREP_START_RECIRC:
									if((cl_dprep_prime_state != CL_DPREP_PRIME_STATE_PRIMING) || (cl_dprep_prime_state != CL_DPREP_PRIME_STATE_PRIME_RCIRC_STARTED) || (cl_dprep_prime_state !=  CL_DPREP_PRIME_STATE_DIALYSER_PRIMING))
									{
										Cl_SendDatatoconsole(CON_TX_COMMAND_PRINTTEXT,"RCIRC",6);
										cl_dprep_primecontroller(CL_DPREP_PRIME_RCIRC_START,0);
										Cl_DprepAlarmTable[CL_DPREP_ALARM_BLOODDOOR_STATUS_OPEN].IsRaised = false;
									}
						break;
						case EVENT_DPREP_START_PRIME:
						if(cl_dprep_prime_state != CL_DPREP_PRIME_STATE_PRIMING)
						{
							Cl_SendDatatoconsole(CON_TX_COMMAND_PRINTTEXT,"PRIME1",6);
							cl_dprep_primecontroller(CL_DPREP_PRIME_PRIME_START,0);
							Cl_DprepAlarmTable[CL_DPREP_ALARM_BLOODDOOR_STATUS_OPEN].IsRaised = false;
						}
						
						break;
						case EVENT_DPREP_MAC_PRIME_COMPLETED:
					//	cl_dprepstate = CL_DPREP_STATE_POST_PRIME_STANDBY;
						break;

						case EVENT_DPREP_TICK_MINUTE:
							Cl_Dprep_ResetAlertsforReassertion();
											
											#if 0
											if(((temp3_cel_backup < temp3_cel) &&(temp3_cel_backup > temp3_cel - 0.3)) || ((temp3_cel_backup < temp3_cel + 0.3)&&(temp3_cel_backup > temp3_cel)))
											{
												cl_temp3_stable = true;
											}
											else
											{
												cl_temp3_stable =false;
											}
											if(((temp3_cel_backup < temp3_cel) &&(temp3_cel_backup > temp3_cel - 1)) || ((temp3_cel_backup < temp3_cel + 1)&&(temp3_cel_backup > temp3_cel)))
											{
												cl_temp3_range_stable = true;
											}
											else
											{
												cl_temp3_range_stable =false;
											}
											
											temp3_cel_backup = temp3_cel
											
											#endif
										
						break;
						
						case EVENT_DPREP_TICK_SECOND:
					//	if(cl_dprep_prime_state == CL_DPREP_PRIME_STATE_PRIME_COMPLETED)
						if(cl_dprep_prime_state == CL_DPREP_PRIME_STATE_PRIME_RCIRC_COMPLETED)
					
						{
							cl_dprepstate = CL_DPREP_STATE_POST_PRIME_STANDBY;
									 
						}
						break;


						case EVENT_DPREP_ALARM:
						Cl_dprepretcode = Cl_Dprep_ProcessAlarms();
						break;
						case EVENT_DPREP_DILYSER_CONNECTED:
		
						// check for micro switches
						Cl_dprepretcode = Cl_SendDatatoconsole(CON_TX_COMMAND_DIALYSER_CONNECTED_CONFIRMED,&data,0);
		
						break;
						case EVENT_DPREP_COMMAND_GET_DATA:

							if(Cl_ConsoleRxMsg.msgready == true)
							{
								switch(Cl_ConsoleRxMsg.data.byte[0])
								{
									case	CON_RX_PARAM_DATA_PRIME_STATUS:
									command = CON_TX_COMMAND_SYSDATA;
									dataarray[0] = CON_TX_PARAM_DATA_PRIME_STATUS;
									Cl_dprepretcode = (Cl_Uint8Type)sv_nvmgetdata(NV_NVM_PRIME_STATUS, &dataarray[1]);
									Cl_dprepretcode = Cl_SendDatatoconsole(command,&dataarray,2);
									break;
									case	CON_RX_PARAM_DATA_DIALYSIS_STATUS:
									command = CON_TX_COMMAND_SYSDATA;
									dataarray[0] = CON_TX_PARAM_DATA_DIALYSIS_STATUS;
									Cl_dprepretcode = (Cl_Uint8Type)sv_nvmgetdata(NV_NVM_DIALYSIS_STATUS, &dataarray[1]);
									Cl_dprepretcode = Cl_SendDatatoconsole(command,&dataarray,2);
									break;
									default:
									break;
								}
							}
						break;
						default:break;
					}

		break;
		
		case CL_DPREP_STATE_DPRIME_WAIT_STABILITY:
		
		 
		 Cl_AlarmActivateAlarms(COND_STATUS_HIGH,false ); // manav02 added
		 Cl_AlarmActivateAlarms(COND_STATUS_LOW,false ); // manav02 added
		 Cl_AlarmActivateAlarms(TEMP3_HIGH_THRESHOLD,false ); // manav02 added
		 Cl_AlarmActivateAlarms(TEMP3_LOW_THRESHOLD,false ); // manav02 added
		 
		 wait_timeout_counter = 0;
		 
		 if(state_counter > 3){
			 
// 			 sv_cntrl_enable_bypass();
// 			 cl_dprep_primecontroller(CL_DPREP_PRIME_PRIME_STOP,0);
// 			 cl_dprepbypassstate = CL_DPREP_BYPASS_ON;
// 			 Cl_dprep_StopMixing();
// 			 cl_dprep_primecontroller(CL_DPREP_PRIME_PRIME_STOP_DIALYSER_PRIMING,0);
			Cl_Dprep_Priming_Stop();
			
		//	Cl_dprep_StopMixing();
			cl_dprep_primecontroller(CL_DPREP_PRIME_PRIME_STOP_DIALYSER_PRIMING,0);
			cl_dprepstate = CL_DPREP_STATE_DIALISER_PRIME_STOPPED;
			Cl_deactivate_prep_alarms();
			
			 state_counter = 0;
			 stability_counter = 0;
			 wait_timeout_counter = 0;
		 }
		 else{
			 
		 
		switch(cl_dprepevent){
		
		case EVENT_DPREP_TICK_SECOND:
		
		wait_timeout_counter++;
		// disable cond and temp alarm
		// if 3 times it comes then stop the dialyzer priming
		// send data to console to wait for stability
		
		
		Cl_Dprep_SendPrepStateData(DIALYSER_PRIME_DATA);
		if(Cl_Check_Temp_Cond_Stability() == CL_OK){
			
			stability_counter++ ;
		}
		
		if(stability_counter == 10){
		
		Cl_AlarmActivateAlarms(COND_STATUS_HIGH,true ); // true manav02 changed 
		Cl_AlarmActivateAlarms(COND_STATUS_LOW,true ); // true manav02 changed 
		Cl_AlarmActivateAlarms(TEMP3_HIGH_THRESHOLD,true ); // true manav02 changed 
		Cl_AlarmActivateAlarms(TEMP3_LOW_THRESHOLD,true ); //true manav02 changed 
		
		stability_counter = 0;	
		wait_timeout_counter = 0;
				Cl_dprepretcode =  sv_cntrl_disable_bypass();
				cl_dprepbypassstate = CL_DPREP_BYPASS_OFF;
				cl_dprepstate = CL_DPREP_STATE_DIALISER_PRIME;
						
			}
		
		if(wait_timeout_counter > 15){
			
			// stop preparation
			Cl_Dprep_Priming_Stop();
			//Cl_dprep_StopMixing();
			cl_dprep_primecontroller(CL_DPREP_PRIME_PRIME_STOP_DIALYSER_PRIMING,0);
			cl_dprepstate = CL_DPREP_STATE_DIALISER_PRIME_STOPPED;
			Cl_deactivate_prep_alarms();
// 			sv_cntrl_enable_bypass();
// 			cl_dprep_primecontroller(CL_DPREP_PRIME_PRIME_STOP,0);
// 			cl_dprepbypassstate = CL_DPREP_BYPASS_ON;
// 			Cl_dprep_StopMixing();
// 			cl_dprep_primecontroller(CL_DPREP_PRIME_PRIME_STOP_DIALYSER_PRIMING,0);
// 				//cl_dprepstate = CL_DPREP_STATE_DPRIME_WAIT_STABILITY;
		}
		
		 
		
		break;
		
		case EVENT_DPREP_ALARM:
		Cl_dprepretcode = Cl_Dprep_ProcessAlarms();
		break;
		
		case EVENT_DPREP_STOP_DIALYSER_PRIME:
		
		Cl_dprep_StopMixing();
		
		cl_dprep_primecontroller(CL_DPREP_PRIME_PRIME_STOP_DIALYSER_PRIMING,0);
		cl_dprepstate = CL_DPREP_STATE_DIALISER_PRIME_STOPPED;
		Cl_deactivate_prep_alarms();
	//	Cl_Dprep_Priming_Stop();
// 		Cl_dprep_StopMixing();
// 		cl_dprep_primecontroller(CL_DPREP_PRIME_PRIME_STOP_DIALYSER_PRIMING,0);
// 		cl_dprepstate = CL_DPREP_STATE_DIALISER_PRIME_STOPPED;
		break;
		
		case EVENT_DPREP_CLR_ALARM:
		
					if(powerFailureFlag == 1){
					
					Cl_SysStat_GetSensor_Status_Query(SENSOR_POWER_FAILURE,&dprep_powerFailSensorData);
					
								if(dprep_powerFailSensorData == 0){
						
						powerFailureFlag = 0;
						command = CON_TX_COMMAND_ALARM_CLEARED;
							Cl_SendDatatoconsole(command,&data,0);
							sv_cntrl_resetyellowalarm();
							sv_cntrl_resetredalarm();
							sv_cntrl_nobuzzer();
							Cl_Dprep_ResetAlertsforReassertion();
					}
					}
					else{			
					
		
			command = CON_TX_COMMAND_ALARM_CLEARED;
			Cl_SendDatatoconsole(command,&data,0);
						
		sv_cntrl_resetyellowalarm();
		sv_cntrl_resetredalarm();
		sv_cntrl_nobuzzer();
		Cl_Dprep_ResetAlertsforReassertion();
		//sv_cntrl_disable_bypass();
					}
		break;
			
		
		}
		}
		
		break;
		case CL_DPREP_STATE_POST_PRIME_STANDBY:
		switch(cl_dprepevent)
					{
						case EVENT_DPREP_COMMAND_GET_DATA:
							if(Cl_ConsoleRxMsg.msgready == true)
							{
								switch(Cl_ConsoleRxMsg.data.byte[0])
								{
									case CON_RX_PARAM_DATA_DIALYSER_PRIME_STATUS:
									command = CON_TX_COMMAND_SYSDATA;
									dataarray[0] = CON_TX_PARAM_DATA_DIALYSER_PRIME_STATUS;
									dataarray[1] = 0;									
									Cl_dprepretcode = Cl_SendDatatoconsole(command,&dataarray,2);
									break;
									case	CON_RX_PARAM_DATA_DIALYSIS_STATUS:
									command = CON_TX_COMMAND_SYSDATA;
									dataarray[0] = CON_TX_PARAM_DATA_DIALYSIS_STATUS;
									Cl_dprepretcode = (Cl_Uint8Type)sv_nvmgetdata(NV_NVM_DIALYSIS_STATUS, &dataarray[1]);
									Cl_dprepretcode = Cl_SendDatatoconsole(command,&dataarray,2);
									break;
									default:break;
								}
							}
						break;
						case EVENT_DPREP_COMMAND_SET_DATA:
								 if(Cl_ConsoleRxMsg.msgready == true)
								 {
						 
						 
									 Cl_ConsoleRxDataType dataId;
									 Cl_Uint8Type count;
									 dataId = Cl_ConsoleRxMsg.data.byte[0];
									 for (count =0; count < Cl_ConsoleRxMsg.datasize;count++)
									 {
										 cl_PrepData.bytearray[count] = Cl_ConsoleRxMsg.data.byte[count+1];
							 
									 }
									 Cl_Dprep_setdata(dataId, cl_PrepData,count);
						 
						 
						 
								 }
	
						break;
						case EVENT_DPREP_DILYSER_CONNECTED:
		
						// check for micro switches
						Cl_dprepretcode = Cl_SendDatatoconsole(CON_TX_COMMAND_DIALYSER_CONNECTED_CONFIRMED,&data,0);
		
						break;
						case EVENT_DPREP_START_DIALISER_PRIME:
				//		sv_prop_startmixing();
				//		Cl_bc_controller(BC_EVENT_RESUME);
								Cl_Dprep_StartDialyserPrime();
								sv_cntrl_disable_loopback();
								 //sv_prop_startmixing();
						break;
						case EVENT_DPREP_ALERT:
						Cl_Alarm_GetAlarmStatus(FPCURRENTSTATUS,&alarmstatus1);
						if(alarmstatus1)
						{
							//Cl_dprepretcode = Cl_bc_controller(BC_EVENT_CS);		
				
						}

						break;
						case EVENT_DPREP_ALARM:
						Cl_dprepretcode = Cl_Dprep_ProcessAlarms();
						break;
						case EVENT_DPREP_COMMAND_SET_BLDPUMPRATE:
											if(Cl_ConsoleRxMsg.msgready == true)
											{
												
												cl_Datastreamtype cl_temp  ;
												cl_temp.bytearray[0] = Cl_ConsoleRxMsg.data.byte[0];
												cl_temp.bytearray[1] = Cl_ConsoleRxMsg.data.byte[0];
												cl_temp.bytearray[2] = Cl_ConsoleRxMsg.data.byte[0];
												cl_temp.bytearray[3] = Cl_ConsoleRxMsg.data.byte[0];
												cl_dprep_primecontroller(CL_DPREP_PRIME_BLOODPUMP_SETRATE,cl_temp.Twobyte);
											}
						break;
						
						case EVENT_DPREP_CLR_ALARM:
						
									if(powerFailureFlag == 1){
					
					Cl_SysStat_GetSensor_Status_Query(SENSOR_POWER_FAILURE,&dprep_powerFailSensorData);
					
								if(dprep_powerFailSensorData == 0){
						
						powerFailureFlag = 0;
						command = CON_TX_COMMAND_ALARM_CLEARED;
							Cl_SendDatatoconsole(command,&data,0);
							sv_cntrl_resetyellowalarm();
							sv_cntrl_resetredalarm();
							sv_cntrl_nobuzzer();
							Cl_Dprep_ResetAlertsforReassertion();
					}
					}
					else{			
					
						
						
							command = CON_TX_COMMAND_ALARM_CLEARED;
							Cl_SendDatatoconsole(command,&data,0);
						
						sv_cntrl_resetyellowalarm();
						sv_cntrl_resetredalarm();
						sv_cntrl_nobuzzer();
						Cl_Dprep_ResetAlertsforReassertion();
						//sv_cntrl_disable_bypass();
					}
						break;
						
						
						case EVENT_DPREP_TICK_500MS:
						//UpdateHeaterControls();
						break;
						case  EVENT_DPREP_TICK_MINUTE:
						{
							float temp1=0;
							Cl_SysStat_GetSensor_Status_Query(SENSOR_TEMP3STATUS,&temp_temp3);

// 							uint16_t cal_data  = (402 *100* temp_temp3)/(2*32768);			// resistance of PT100
// 							res_temp_lookuptable(cal_data);									// temperature from look up table in 4 digits
// 							temp_temp3 = res_temp_value/100;									// Temp3 value in XX.yy format
// 	//						temp_temp3 = temp_temp3 - 3.1 + 1.4;										// sensor offset
// 							temp_temp3 = temp_temp3 - 0.4;										// sensor offset
							temp3_cel = (float)temp_temp3 /100;
							Cl_Dprep_filling_secondscounter++;
						}
						
					#if 0
							//	if(!(Cl_Dprep_filling_secondscounter%20))
								{
													if(((temp3_cel_backup < temp3_cel) &&(temp3_cel_backup > temp3_cel - 0.6)) || ((temp3_cel_backup < temp3_cel + 0.6)&&(temp3_cel_backup > temp3_cel)))
													{
														cl_temp3_stable = true;
													}
													else
													{
														cl_temp3_stable =false;
													}
													if(((temp3_cel_backup < temp3_cel) &&(temp3_cel_backup > temp3_cel - 1)) || ((temp3_cel_backup < temp3_cel + 1)&&(temp3_cel_backup > temp3_cel)))
													{
														cl_temp3_range_stable = true;
													}
													else
													{
														cl_temp3_range_stable =false;
													}
										
													temp3_cel_backup = temp3_cel;
								}
					#endif
						break;
						default:
						break;
					}
		break;

		case CL_DPREP_STATE_DIALISER_PRIME:
					switch(cl_dprepevent)
					{
						
						case EVENT_DPREP_TICK_50MS:
									
						break;
						case EVENT_DPREP_TICK_500MS:
					//	UpdateHeaterControls();
						break;
						case EVENT_DPREP_TICK_MINUTE:
							Cl_Dprep_ResetAlertsforReassertion();
						break;
						case EVENT_DPREP_TICK_SECOND:
						//	UpdateHeaterControls();
						{
							float temp1=0;
							Cl_SysStat_GetSensor_Status_Query(SENSOR_TEMP3STATUS,&temp_temp3);
					
// 						uint16_t cal_data  = (402 *100* temp_temp3)/(2*32768);			// resistance of PT100
// 						res_temp_lookuptable(cal_data);									// temperature from look up table in 4 digits
// 						temp_temp3 = res_temp_value/100;									// Temp3 value in XX.yy format
// //						temp_temp3 = temp_temp3 - 3.1 + 1.4;										// sensor offset
// //						temp_temp3 = temp_temp3 - 3.1 - 2;										// sensor offset
						temp3_cel = (float)temp_temp3;
						
							Cl_Dprep_filling_secondscounter++;
						}
						
					#if 0
					if(!(Cl_Dprep_DialyserPrime_secondscounter%20))
					{
										if(((temp3_cel_backup < temp3_cel) &&(temp3_cel_backup > temp3_cel - 0.3)) || ((temp3_cel_backup < temp3_cel + 0.3)&&(temp3_cel_backup > temp3_cel)))
										{
											cl_temp3_stable = true;
										}
										else
										{
											cl_temp3_stable =false;
										}
										if(((temp3_cel_backup < temp3_cel) &&(temp3_cel_backup > temp3_cel - 1)) || ((temp3_cel_backup < temp3_cel + 1)&&(temp3_cel_backup > temp3_cel)))
										{
											cl_temp3_range_stable = true;
										}
										else
										{
											cl_temp3_range_stable =false;
										}
										
										temp3_cel_backup = temp3_cel;
					}
					
					#endif

							Cl_Dprep_DialyserPrime_secondscounter++;
							
							
							if(Cl_Dprep_DialyserPrime_secondscounter == 30){ // activate flow alarms only after 30 seconds into rinse
								
								
								Cl_dprepretcode =  Cl_AlarmActivateAlarms(FLOW_NO_FLOW,true );
								Cl_dprepretcode =  Cl_AlarmActivateAlarms(FLOW_LOW_FLOWRATE,true );
								Cl_dprepretcode =  Cl_AlarmActivateAlarms(FLOW_HIGH_FLOWRATE,true );
								
								
							}
							
							if(Cl_Dprep_DialyserPrime_secondscounter == 60)
							{
								UpdateDprepDialyserPrimeMinuteTick();
							
							}
							Cl_Dprep_SendPrepStateData(DIALYSER_PRIME_DATA);
						//	Cl_Dprep_ResetAlertsforReassertion();
						//	Cl_dprepretcode = Cl_Dprep_ProcessAlarms();

					// check for sub state time out and transition
					

							if( Cl_Dprep_DialyserPrime_Minutescounter >= CL_DPREP_DIALISER_PRIME_TIMEOUT_MIN)
							{
								Cl_Dprep_DialyserPrime_Minutescounter = 0;
							//	sv_cntrl_deactivatepump(BLOODPUMP);
							//	cl_dprep_primecontroller(CL_DPREP_PRIME_BLOODPUMP_STOP,0);
								cl_dprep_primecontroller(CL_DPREP_PRIME_PRIME_STOP_DIALYSER_PRIMING, 1); //1 == STOPPED BECAUSE COMPLTED

								//	Cl_rinseretcode = (Cl_ReturnCodeType)sv_setflowpath(Default path ?);//set appropirate flow path configuration
							//	Cl_dprepretcode = sv_cntrl_poweroffheater();
							//	Cl_dprepretcode =  sv_cntrl_deactivatepump(DCMOTOR1);
							//	Cl_dprepretcode =  sv_cntrl_deactivatepump(DCMOTOR2);
								//sv_cntrl_enable_loopback();
								 //sv_prop_stopmixing();
								 Cl_Uint32Type temp = Treatdata[ID_dflow];
								 Cl_Uint16Type potvalue=0;
// 								 switch (temp)
// 								 {
// 									 case 800:
// 									 potvalue = (612 * 1024)/10000;
// 									 sv_cs_setpotvalue(potvalue);
// 									 break;
// 									 
// 									 case 500:
// 									 Cl_SendDatatoconsole(CON_TX_COMMAND_PRINTTEXT,"500",3);
// 									 potvalue = (670 * 1024)/10000;
// 									 sv_cs_setpotvalue(potvalue);
// 									 break;
// 									 
// 									 case 300:
// 									 potvalue = (800 * 1024)/10000;
// 									 sv_cs_setpotvalue(potvalue);
// 									 break;
// 									 
// 									 default:
// 									 break;
// 								 }
								sv_cntrl_enable_bypass();
								Cl_dprepretcode =  	cl_dprep_notifydacandgotodpreptandby();	
								cl_dprepbypassstate = CL_DPREP_BYPASS_ON;
								cl_dprepstate = CL_DPREP_STATE_POST_DPREP_STANDBY;	
							}
						break;
						case EVENT_DPREP_STOP_DIALYSER_PRIME:
						//Cl_Stop_Dialyzer_Prime();
						Cl_Dprep_Priming_Stop();
					//	Cl_dprep_StopMixing();
					//	cl_dprep_primecontroller(CL_DPREP_PRIME_PRIME_STOP_DIALYSER_PRIMING,0);
					//	cl_dprepstate = CL_DPREP_STATE_DIALISER_PRIME_STOPPED;
						break;
						break;
						case EVENT_DPREP_ALARM:
						Cl_dprepretcode = Cl_Dprep_ProcessAlarms();
						break;
						case EVENT_DPREP_ALERT:
						Cl_Alarm_GetAlarmStatus(FPCURRENTSTATUS,&alarmstatus1);
						if(alarmstatus1)
						{
							//Cl_dprepretcode = Cl_bc_controller(BC_EVENT_CS);
						}

						break;
						
						case EVENT_DPREP_CLR_ALARM:
						
									if(powerFailureFlag == 1){
					
					Cl_SysStat_GetSensor_Status_Query(SENSOR_POWER_FAILURE,&dprep_powerFailSensorData);
					
								if(dprep_powerFailSensorData == 0){
						
						powerFailureFlag = 0;
						command = CON_TX_COMMAND_ALARM_CLEARED;
							Cl_SendDatatoconsole(command,&data,0);
							sv_cntrl_resetyellowalarm();
							sv_cntrl_resetredalarm();
							sv_cntrl_nobuzzer();
							Cl_Dprep_ResetAlertsforReassertion();
							sv_cntrl_disable_bypass();
					}
									}
					else{			
					
						
						
							command = CON_TX_COMMAND_ALARM_CLEARED;
							Cl_SendDatatoconsole(command,&data,0);
						
						sv_cntrl_resetyellowalarm();
						sv_cntrl_resetredalarm();
						sv_cntrl_nobuzzer();
						Cl_Dprep_ResetAlertsforReassertion();
						sv_cntrl_disable_bypass();
					}
						break;
						
					}
					
					
		break;
		case CL_DPREP_STATE_DIALISER_PRIME_STOPPED:
						switch(cl_dprepevent)
							{
								
								case EVENT_DPREP_CLR_ALARM:
								
												if(powerFailureFlag == 1){
					
					Cl_SysStat_GetSensor_Status_Query(SENSOR_POWER_FAILURE,&dprep_powerFailSensorData);
					
								if(dprep_powerFailSensorData == 0){
						
						powerFailureFlag = 0;
						command = CON_TX_COMMAND_ALARM_CLEARED;
							Cl_SendDatatoconsole(command,&data,0);
							sv_cntrl_resetyellowalarm();
							sv_cntrl_resetredalarm();
							sv_cntrl_nobuzzer();
							Cl_Dprep_ResetAlertsforReassertion();
						//	sv_cntrl_disable_bypass();
					}
												}
					else{			
					
								
								command = CON_TX_COMMAND_ALARM_CLEARED;
							    Cl_SendDatatoconsole(command,&data,0);
							
					
										sv_cntrl_resetyellowalarm();
										sv_cntrl_resetredalarm();
										sv_cntrl_nobuzzer();
										Cl_Dprep_ResetAlertsforReassertion();
					}
												
								break;
								
								
								case EVENT_DPREP_START_DIALISER_PRIME:
								//sv_prop_startmixing();
								Cl_bc_controller(BC_EVENT_RESUME);
								Cl_Dprep_StartDialyserPrime();
								sv_cntrl_disable_loopback();
								sv_prop_startmixing();
								Cl_dprepretcode = sv_cntrl_poweronheater();               
					 			Cl_dprepretcode = SetHeaterState(CL_HEATER_STATE_ON);
								 break;
								 default:break;
							}
		break;
		case CL_DPREP_STATE_POST_DPREP_STANDBY:
		
			switch(cl_dprepevent)
			{
				case EVENT_DPREP_ALERT:
				Cl_Alarm_GetAlarmStatus(FPCURRENTSTATUS,&alarmstatus1);
				if(alarmstatus1)
				{
				//	Cl_dprepretcode = Cl_bc_controller(BC_EVENT_CS);
				}

				break;
				case EVENT_DPREP_ALARM:
				Cl_dprepretcode = Cl_Dprep_ProcessAlarms();
				break;
				case EVENT_DPREP_PATIENT_READY:
				cl_bp_controller(CL_BP_EVENT_STOP,0);
				Cl_PatientState = CL_DPREP_PATIENT_STATE_WAITING_FOR_BD;
				break;
				case EVENT_DPREP_PATIENT_CONNECTED:
				if(Cl_PatientState == CL_DPREP_PATIENT_STATE_BLOOD_DETECTED )
				{
					Cl_dprepretcode = Cl_mac_apprequesthandler(MACREQ_DIALYSER_PRIME_COMPLETED);
					cl_dprepstate = CL_DPREP_STATE_READY_FOR_DALYSIS;
				}
				
				break;

				case EVENT_DPREP_TICK_MINUTE:
					Cl_Dprep_ResetAlertsforReassertion();
				break;
				case EVENT_DPREP_TICK_SECOND:
				
				
					if(Cl_PatientState == CL_DPREP_PATIENT_STATE_WAITING_FOR_BD )
					{
						Cl_SysStat_GetSensor_Status_Query(SENSOR_BDSTATUS , &temp);
				
					if(temp == 1)        // manav02 changed to 1
					{
					Cl_SendDatatoconsole(CON_TX_COMMAND_PRINTTEXT,"BLOOD",5);
					//	Cl_dprepretcode = Cl_SendDatatoconsole(CON_TX_COMMAND_READY_FOR_DIALYSIS,NULL,0);
					//	cl_dprepstate = CL_DPREP_STATE_READY_FOR_DALYSIS;
									Cl_dprepretcode = Cl_SendDatatoconsole(CON_TX_COMMAND_BLOOD_DETECTED,NULL,0);
									Cl_PatientState = CL_DPREP_PATIENT_STATE_BLOOD_DETECTED;
									
							
										Cl_Uint32Type temp = Treatdata[ID_bolusvol];
										hep_calibration(temp);
										Cl_SendDatatoconsole(CON_TX_COMMAND_PRINTDATA,&temp,2);
										Cl_SendDatatoconsole(CON_TX_COMMAND_PRINTDATA,&hep_speed,2);
										cl_hep_controller(CL_HEP_EVENT_SET_NORMAL_DELIVERY_RATE,hep_speed);
										cl_wait(100);
										cl_hep_controller(CL_HEP_EVENT_START_NORMAL_DELIVERY,0);
										Cl_SendDatatoconsole(CON_TX_COMMAND_PRINTTEXT,"HEPARIN",7);
										
									
					}
				
					}
					if(Cl_PatientState == CL_DPREP_PATIENT_STATE_BLOOD_DETECTED )
					{
					Cl_Dprep_hepbolus_secondscounter++;
					if (Cl_Dprep_hepbolus_secondscounter == 60)
					{
						Cl_Dprep_hepbolus_secondscounter = 0;
						Cl_SendDatatoconsole(CON_TX_COMMAND_PRINTTEXT,"DONE_HP",7);
						cl_hep_controller(CL_HEP_EVENT_STOPBOLUS,0);
					}
					}
				break;
			}
		
		break;
		case CL_DPREP_STATE_READY_FOR_DALYSIS:
				switch(cl_dprepevent)
				{
					
					case EVENT_DPREP_ALERT:
					Cl_Alarm_GetAlarmStatus(FPCURRENTSTATUS,&alarmstatus1);
					if(alarmstatus1)
					{
					//	Cl_dprepretcode = Cl_bc_controller(BC_EVENT_CS);
					}

					break;
					case EVENT_DPREP_TICK_MINUTE:
						Cl_Dprep_ResetAlertsforReassertion();
					break;
					case EVENT_DPREP_TICK_SECOND:
									
					Cl_Dprep_SendPrepStateData(DIALYSIS_PREP_DATA);
					Cl_SysStat_GetSensor_Status_Query(SENSOR_BDSTATUS , &temp);
					if( temp == 0)
					{
						
							Cl_SendDatatoconsole(CON_TX_COMMAND_PRINTTEXT,"BLOOD",5);
					}
					break;
					// manav 05 commented legacy code
				/*	case 	EVENT_DPREP_COMMAND_GET_DATA:
							if(Cl_ConsoleRxMsg.msgready == true)
							{
								switch(Cl_ConsoleRxMsg.data.byte[0])
								{
									case CON_RX_PARAM_DATA_DIALYSER_PRIME_STATUS:
									command = CON_TX_COMMAND_SYSDATA;
									dataarray[0] = CON_TX_PARAM_DATA_DIALYSER_PRIME_STATUS;
									dataarray[1] = 0;
									Cl_dprepretcode = Cl_SendDatatoconsole(command,&dataarray,2);
									break;
									case	CON_RX_PARAM_DATA_DIALYSIS_STATUS:
									command = CON_TX_COMMAND_SYSDATA;
									dataarray[0] = CON_TX_PARAM_DATA_DIALYSIS_STATUS;
									Cl_dprepretcode = (Cl_Uint8Type)sv_nvmgetdata(NV_NVM_DIALYSIS_STATUS, &dataarray[1]);
									Cl_dprepretcode = Cl_SendDatatoconsole(command,&dataarray,2);
									break;
									default:break;
								}
							}
				
						break;
						case EVENT_DPREP_COMMAND_SET_DATA:
								 if(Cl_ConsoleRxMsg.msgready == true)
								 {
						 
						 
									 Cl_ConsoleRxDataType dataId;
									 Cl_Uint8Type count;
									 dataId = Cl_ConsoleRxMsg.data.byte[0];
									 for (count =0; count < Cl_ConsoleRxMsg.datasize;count++)
									 {
										 cl_PrepData.bytearray[count] = Cl_ConsoleRxMsg.data.byte[count+1];
							 
									 }
									 Cl_Dprep_setdata(dataId, cl_PrepData,count);
						 
						 
						 
								 }
	
						break;
			*/
						case EVENT_DPREP_ALARM:
						Cl_dprepretcode = Cl_Dprep_ProcessAlarms();
						break;
					default:
					break;
				}
		break;

		}

	return Cl_dprepretcode;
}
 
 
Cl_ReturnCodeType  cl_dprep_translatemacevent(Cl_Mac_EventsType Cl_MacDprepEvt,Cl_Dprep_Events* cl_dprepevent)
{
	switch(Cl_MacDprepEvt)
	{
	
		case EVT_CONSOLE_COMMAND_CLR_ALARM:
		*cl_dprepevent =EVENT_DPREP_CLR_ALARM;
		break;
		
		case EVT_CONSOLE_COMMAND_DIALYSIS_PREP:
		*cl_dprepevent = EVENT_DPREP_DIALYSIS_PREP;
		break;

		case EVT_CONSOLE_COMMAND_SET_BLDPMP_ON:
		*cl_dprepevent = EVENT_DPREP_COMMAND_SET_BLDPMP_ON;
		break;
		case EVT_CONSOLE_COMMAND_SET_BLDPMP_OFF:
		*cl_dprepevent = EVENT_DPREP_COMMAND_SET_BLDPMP_OFF;
		break;
		case 	EVT_CONSOLE_COMMAND_SET_BLDPUMPRATE:
		*cl_dprepevent = EVENT_DPREP_COMMAND_SET_BLDPUMPRATE;
		break;
		case EVT_CONSOLE_COMMAND_SET_DATA:
		*cl_dprepevent =  EVENT_DPREP_COMMAND_SET_DATA;
		break;
		case EVT_CONSOLE_COMMAND_GET_DATA:
		*cl_dprepevent = EVENT_DPREP_COMMAND_GET_DATA;
		break;
		
		case EVT_CONSOLE_COMMAND_START_DIALYSATE_FILLING:
				*cl_dprepevent = EVENT_DPREP_START_DIALYSATE_FILL;
				break;
		case EVT_CONSOLE_COMMAND_START_PRIME:
		*cl_dprepevent = EVENT_DPREP_START_PRIME;
		break;
		
		case EVT_CONSOLE_COMMAND_STOP_PRIME:
		*cl_dprepevent = EVENT_DPREP_STOP_PRIME;
		break;
		case EVT_CONSOLE_COMMAND_DILYSER_CONNECTED:
		*cl_dprepevent = EVENT_DPREP_DILYSER_CONNECTED;
		break;
		case EVT_CONSOLE_COMMAND_START_DIALISER_PRIME:
		*cl_dprepevent = EVENT_DPREP_START_DIALISER_PRIME;
		break;
		case EVT_CONSOLE_COMMAND_STOP_DIALISER_PRIME:
		*cl_dprepevent = EVENT_DPREP_STOP_DIALYSER_PRIME;
		break;	
		

		case EVT_CONSOLE_COMMAND_MAN_PREP_COMPLETED:
		*cl_dprepevent =   EVENT_DPREP_MAN_PREP_COMPLETED;
		break;
		
		case EVT_TICK_50M:
		*cl_dprepevent =   EVENT_DPREP_TICK_50MS;
		break;
		case  EVT_TICK_100M:
		*cl_dprepevent =   EVENT_DPREP_TICK_100MS;
		break;
		
		case EVT_TICK_500M:
		*cl_dprepevent =   EVENT_DPREP_TICK_500MS;
		break;
		case EVT_TICK_SEC:
		*cl_dprepevent =   EVENT_DPREP_TICK_SECOND;
		break;
		case EVT_TICK_MIN:
		*cl_dprepevent =   EVENT_DPREP_TICK_MINUTE;
		break;			 
		case EVT_TICK_HOUR:
		*cl_dprepevent =   EVENT_DPREP_TICK_HOUR;
		break;
		
		case MACREQ_PRIME_COMPLETED:
		*cl_dprepevent = EVENT_DPREP_MAC_PRIME_COMPLETED;
		break;
		case MACREQ_DIALYSER_PRIME_COMPLETED:
			*cl_dprepevent = EVENT_DPREP_MAC_DIALYSER_PRIME_COMPLETED;
		break;
		case EVT_ALARM_TRIGGERED:
		*cl_dprepevent =  EVENT_DPREP_ALARM;
		break;
		case EVT_ALERT_TRIGGERED:
		*cl_dprepevent =  EVENT_DPREP_ALERT;
		break;
		case EVT_CONSOLE_COMMAND_PATIENT_CONNECTED:
		*cl_dprepevent =  EVENT_DPREP_PATIENT_CONNECTED;
		break;
		case EVT_CONSOLE_COMMAND_PATIENT_READY:
		*cl_dprepevent = EVENT_DPREP_PATIENT_READY;
		break;
		case EVT_SEND_MIXING_PREP_START:
		*cl_dprepevent =  EVENT_DPREP_MIXING_PREP_START;
		break;
		case EVT_SEND_MIXING_PREP_STOP:
		*cl_dprepevent = EVENT_DPREP_MIXING_PREP_STOP;
		break;
		case EVT_CONSOLE_COMMAND_RCIRC_START:
		*cl_dprepevent = EVENT_DPREP_START_RECIRC;
		break;
		case 	EVT_CONSOLE_COMMAND_HEPARIN_BOLUS:
		*cl_dprepevent = EVENT_HEAPARIN_BOLUS;
		break;
		case EVT_CONSOLE_COMMAND_HEPARIN_START:
		*cl_dprepevent = EVENT_DPREP_HEP_PMP_START;
		break;
		case EVT_CONSOLE_COMMAND_HEPARIN_STOP:
		*cl_dprepevent = EVENT_DPREP_HEP_PMP_STOP;
		break;
		case EVT_CONSOLE_COMMAND_HEP_PMP_RATE:
		*cl_dprepevent = EVENT_DPREP_HEP_PMP_RATE;
		break;
		case EVT_CONSOLE_COMMAND_SET_BLDPUMPINC_MANUAL:
		*cl_dprepevent = EVT_CONSOLE_COMMAND_SET_BLDPUMPINC_MANUAL;
		break;
		case EVT_CONSOLE_COMMAND_SET_BLDPUMPDEC_MANUAL:
		*cl_dprepevent = EVT_CONSOLE_COMMAND_SET_BLDPUMPDEC_MANUAL;
		break;
		case EVT_CONSOLE_COMMAND_MUTE_ALARM:
		*cl_dprepevent = EVENT_DPREP_MUTE;
		break;
		default:
		break;
	}
		 return CL_OK;
}
Cl_ReturnCodeType Cl_Dprep_UpdateTimeInfo(void)
{
	
	Cl_ReturnCodeType  Cl_dprepretcode = CL_ERROR;
	Cl_ConsoleTxCommandtype command = CON_TX_COMMAND_COMMAND_MAX;
	Cl_Uint8Type data[7] = {0,0,0,0,0,0,0};
	
	command = CON_TX_COMMAND_REM_TIME;

	
	data[1]= (Cl_Uint8Type)Cl_dprepTotalMinutescounter;
	data[2]= (Cl_Uint8Type)Cl_dprepTotalhourscounter;
	data[3]= (Cl_Uint8Type)Cl_dprepsecondscounter;
	data[4]= (Cl_Uint8Type) (CL_DPREP_PRIME_TIMEOUT_MIN - Cl_dprepTotalMinutescounter );
	data[5]= (Cl_Uint8Type) (CL_DPREP_PRIME_TIMEOUT_HRS - Cl_dprepTotalhourscounter );
	data[6]= (Cl_Uint8Type) (60 - Cl_dprepsecondscounter );
	
	Cl_dprepretcode = Cl_SendDatatoconsole(command,&data,7);
	
	return CL_OK;
	}
	
	Cl_ReturnCodeType Cl_Dprep_UpdateFillingTimeInfo(void)
{
	
	Cl_ReturnCodeType  Cl_dprepretcode = CL_ERROR;
	Cl_ConsoleTxCommandtype command = CON_TX_COMMAND_COMMAND_MAX;
	Cl_Uint8Type data[7] = {0,0,0,0,0,0,0};
	#if 0
	command = CON_TX_COMMAND_REM_TIME;

	data[0]= (Cl_Uint8Type) DIALYSATE_FILLING_DATA;
	data[1]= (Cl_Uint8Type)Cl_Dprep_filling_Minutescounter;
	data[2]= (Cl_Uint8Type)0;
	data[3]= (Cl_Uint8Type)Cl_Dprep_filling_secondscounter;
	data[4]= (Cl_Uint8Type) (CL_DPREP_PRIME_TIMEOUT_MIN - Cl_Dprep_filling_TotalMinutescounter );
	data[5]= (Cl_Uint8Type) 0;
	data[6]= (Cl_Uint8Type) 0;
	
	Cl_dprepretcode = Cl_SendDatatoconsole(command,&data,7);
	#endif
	return CL_OK;
	}
	
#if 0
		Cl_ReturnCodeType Cl_Dprep_UpdatePrimeTimeInfo(void)	
			{
	
				Cl_ReturnCodeType  Cl_dprepretcode = CL_ERROR;
				Cl_ConsoleTxCommandtype command = CON_TX_COMMAND_COMMAND_MAX;
				Cl_Uint8Type data[7] = {0,0,0,0,0,0,0};
	
				command = CON_TX_COMMAND_REM_TIME;

				data[0] = (Cl_Uint8Type) PRIMING_DATA;

				data[1]= (Cl_Uint8Type)Cl_Dprep_Prime_Minutescounter;
				data[2]= (Cl_Uint8Type)0;
				data[3]= (Cl_Uint8Type)Cl_Dprep_Prime_secondscounter;
				data[4]= (Cl_Uint8Type) (CL_DPREP_PRIME_TIMEOUT_MIN - Cl_Dprep_Prime_TotalMinutescounter );
				data[5]= (Cl_Uint8Type) 0;
				data[6]= (Cl_Uint8Type) 0;
	
				Cl_dprepretcode = Cl_SendDatatoconsole(command,&data,7);
	
				return CL_OK;
			}
		Cl_ReturnCodeType Cl_Dprep_UpdateDialyserPrimeTimeInfo(void)	
			{
				
				Cl_ReturnCodeType  Cl_dprepretcode = CL_ERROR;
				Cl_ConsoleTxCommandtype command = CON_TX_COMMAND_COMMAND_MAX;
				Cl_Uint8Type data[7] = {0,0,0,0,0,0,0};
				
				command = CON_TX_COMMAND_REM_TIME;
				data[0] = (Cl_Uint8Type)DIALYSER_PRIME_DATA;

				data[1]= (Cl_Uint8Type)Cl_Dprep_DialyserPrime_Minutescounter;
				data[2]= (Cl_Uint8Type)0;
				data[3]= (Cl_Uint8Type)Cl_Dprep_DialyserPrime_secondscounter;
				data[4]= (Cl_Uint8Type) (CL_DPREP_PRIME_TIMEOUT_MIN - Cl_Dprep_DialyserPrime_TotalMinutescounter );
				data[5]= (Cl_Uint8Type) 0;
				data[6]= (Cl_Uint8Type) 0;
				
				Cl_dprepretcode = Cl_SendDatatoconsole(command,&data,7);
				
				return CL_OK;
			}	
		
	#endif
	
	Cl_ReturnCodeType cl_dprep_notifydacandgotodprep_postprimetandby(void)
	{
		Cl_ReturnCodeType 	Cl_dprepretcode = CL_OK;
		
		Cl_Uint8Type data =0;
		//inform DAC about rinse completed state.
		Cl_dprepsecondscounter = 0;
		Cl_dprepMinutescounter= 0;
		Cl_dprephourscounter= 0;
		Cl_dprepTotalMinutescounter= 0;
		Cl_dprepTotalhourscounter=0;
		data = 1;
		Cl_dprepretcode = sv_nvmsetdata(NV_NVM_PRIME_STATUS,&data,1);
		
		Cl_dprepretcode = Cl_SendDatatoconsole(CON_TX_COMMAND_PRIME_COMPLETED,&data,0);
	//	Cl_dprepretcode = Cl_mac_apprequesthandler(MACREQ_PRIME_COMPLETED);
		
		return Cl_dprepretcode;
		
	}
	Cl_ReturnCodeType cl_dprep_notifydacandgotodpreptandby(void)
	{
		Cl_ReturnCodeType 	Cl_dprepretcode = CL_OK;
		
		Cl_Uint8Type data =0;
		//inform DAC about rinse completed state.
		Cl_dprepsecondscounter = 0;
		Cl_dprepMinutescounter= 0;
		Cl_dprephourscounter= 0;
		Cl_dprepTotalMinutescounter= 0;
		Cl_dprepTotalhourscounter=0;
		data = 1;
		Cl_dprepretcode = sv_nvmsetdata(NV_NVM_PRIME_STATUS,&data,1);
		
		Cl_dprepretcode = Cl_SendDatatoconsole(CON_TX_COMMAND_DIALISYS_PRIME_COMPLETED,&data,0);
		Cl_dprepretcode = Cl_mac_apprequesthandler(MACREQ_DIALYSER_PRIME_COMPLETED);
		
		
		return Cl_dprepretcode;
	}
	
	Cl_ReturnCodeType Cl_dprep_prime_FSM(Cl_Dprep_Events cl_dprepevent)
	{
		return CL_OK;	
	}
	
	Cl_ReturnCodeType Cl_dprep_filling_FSM_Init(Cl_Dprep_Events cl_dprepevent)
	{
		cl_dprepfillingState = CL_DPREP_FILLING_IDLE;
			Cl_Dprep_filling_Minutescounter = 0;
			Cl_Dprep_filling_TotalMinutescounter = 0;
		return CL_OK;								
	}
	
	
	Cl_ReturnCodeType Cl_Dprep_ProcessAlarms(void )
	{
		Cl_ReturnCodeType 	Cl_dprepretcode = CL_OK;
		ClDprepAlarmIdType CldprepAlarmId;
		Cl_Uint8Type data;
		Cl_ConsoleTxCommandtype command = CON_TX_COMMAND_COMMAND_MAX;
		Cl_NewAlarmIdType cl_dprepalarmid;
		//	cl_wait(200);
		Cl_dprepretcode = Cl_Dprep_UpdateAlarmTable();
		//   	Cl_rinseretcode =	Cl_Alarm_GetLastAlarm(&cl_rinsealarmid);
		//   	data = (Cl_Uint8Type)cl_rinsealarmid;
		//   	command = CON_TX_COMMAND_ALARM;
		//   	Cl_rinseretcode = Cl_SendDatatoconsole(command,&data,0);
		
		
		return (Cl_dprepretcode);
	}
	
	
	Cl_ReturnCodeType Cl_Dprep_UpdateAlarmTable( void)
	{
		Cl_ReturnCodeType 	Cl_dprepretcode = CL_OK;
		Cl_Uint8Type tempcount = 0;
		Cl_Uint8Type data[2] ={0, 0} ,*data1 = NULL;
		Cl_Uint8Type data3;
		Cl_NewAlarmIdType cl_alarmId;
		
		Cl_ConsoleTxCommandtype command = CON_TX_COMMAND_COMMAND_MAX;
		
		
		for (tempcount = 0 ; tempcount < CL_DPREP_ALRM_MAX ; tempcount++)
		{
			
			Cl_Alarm_GetAlarmStatus(Cl_DprepAlarmTable[tempcount].Cl_DprepAlarmId,&Cl_DprepAlarmTable[tempcount].IsActive);
			//	if(Cl_RinseAlarmTable[tempcount].Cl_RinseAlarmType == CL_ALARM_ALARM)
			//	{
			
			
			if (Cl_DprepAlarmTable[tempcount].IsActive)
			{
				if(!Cl_DprepAlarmTable[tempcount].IsRaised)
				{
				//	cl_alarmId = Cl_DprepAlarmTable[tempcount].Cl_DprepAlarmId;

				//	data[0] = (Cl_Uint8Type)(Cl_DprepAlarmTable[tempcount].Cl_DprepAlarmId);
				//	data[1] = (Cl_Uint8Type) CL_ALARM_TRIGGERED;
				//	data[1] = (Cl_Uint8Type) CRITICAL;
					//	data1 = (Cl_Uint8Type) cl_alarmId;

				//	command = CON_TX_COMMAND_ALARM;

				//	Cl_dprepretcode = Cl_SendDatatoconsole(command,&data,3);
					Cl_DprepAlarmTable[tempcount].IsRaised = true;
					
					Cl_dprepretcode = CL_DrepAlarmActon(Cl_DprepAlarmTable[tempcount].Cl_DprepAlarmId);
					
					
				}
				
			}
			else
			{
				// alarm was present before , but not active now.
				if(Cl_DprepAlarmTable[tempcount].IsRaised == true)
				{
					Cl_DprepAlarmTable[tempcount].IsRaised = false;
					command = CON_TX_COMMAND_ALARM;
					data[0] = (Cl_Uint8Type)(Cl_DprepAlarmTable[tempcount].Cl_DprepAlarmId);
					data[1] = (Cl_Uint8Type) 0;
				//	Cl_dprepretcode = Cl_SendDatatoconsole(command,&data,2);
				}
				Cl_DprepAlarmTable[tempcount].IsActive = true;

			}
			//}

		}

		return (Cl_dprepretcode );
	}

Cl_ReturnCodeType  CL_DrepAlarmActon(Cl_NewAlarmIdType cl_dprepalarmid)
{
	Cl_ReturnCodeType 	Cl_dprepretcode = CL_OK;
	Cl_Uint16Type levelswitchstatus = 0;
	Cl_Uint16Type wait_cnt = 0,TmpVal=0;
	static Cl_Uint8Type fillseccounter=0;
	Cl_Uint8Type data[3] ={0, 0};
	Cl_NewAlarmIdType NewAlarmId = _NO_ALARM;
				Cl_Uint8Type data3;
				Cl_ConsoleTxCommandtype command = CON_TX_COMMAND_COMMAND_MAX;
	
	switch(cl_dprepalarmid)
	{
		
		case POWER_FAILURE:
		
		powerFailureFlag = 1 ;
			sv_cntrl_buzzer();	
				sv_cntrl_setredalarm();
				if((cl_dprepstate == CL_DPREP_STATE_DIALISER_PRIME) ||(cl_dprepstate == CL_DPREP_STATE_DPRIME_WAIT_STABILITY) || (cl_dprepstate == CL_DPREP_STATE_DREP_PRIME_ALARM_BYPASS))
				{
					//Cl_Dprep_Stoppreparation();
					Cl_Dprep_Priming_Stop();
				//	Cl_dprep_StopMixing();
			//		cl_dprep_primecontroller(CL_DPREP_PRIME_PRIME_STOP_DIALYSER_PRIMING,0);
			//		cl_dprepstate = CL_DPREP_STATE_DIALISER_PRIME_STOPPED;
			//		Cl_deactivate_prep_alarms();
				}
				else{
					
					Cl_Dprep_Stoppreparation();
					cl_dprepstate = CL_DPREP_STATE_CRITICAL_ALARM;
				}
			
		
		break;
		
			case BP_STOPPED:
			//case POWER_FAILURE:		
			case FLOW_NO_FLOW:
			case FLOW_LOW_FLOWRATE:
			case FLOW_HIGH_FLOWRATE:
			case TEMP2_HIGH_THRESHOLD:
			case TEMP2_LOW_THRESHOLD:
			case TEMP1_HIGH_THRESHOLD:
			case TEMP1_LOW_THRESHOLD:
			case 	PS1_HIGH_THRESHOLD:
			case	PS1_LOW_THRESHOLD:
			case    BC_ALARM:
			case 	PS2_HIGH_THRESHOLD:
			case	PS2_LOW_THRESHOLD:
			case 	PS3_HIGH_THRESHOLD:
			case	PS3_LOW_THRESHOLD:
				
				//sv_cntrl_setredalarm();
	        	sv_cntrl_buzzer();	
				sv_cntrl_setredalarm();
				if((cl_dprepstate == CL_DPREP_STATE_DIALISER_PRIME) ||(cl_dprepstate == CL_DPREP_STATE_DPRIME_WAIT_STABILITY) || (cl_dprepstate == CL_DPREP_STATE_DREP_PRIME_ALARM_BYPASS))
				{
					//Cl_Dprep_Stoppreparation();
					Cl_Dprep_Priming_Stop();
				//	Cl_dprep_StopMixing();
			//		cl_dprep_primecontroller(CL_DPREP_PRIME_PRIME_STOP_DIALYSER_PRIMING,0);
			//		cl_dprepstate = CL_DPREP_STATE_DIALISER_PRIME_STOPPED;
			//		Cl_deactivate_prep_alarms();
				}
				else{
					
					Cl_Dprep_Stoppreparation();
					cl_dprepstate = CL_DPREP_STATE_CRITICAL_ALARM;
				}
				
			
			break;
			
			case BLOODDOOR_STATUS_OPEN:
				sv_cntrl_setyellowalarm();
				sv_cntrl_buzzer();
				//Cl_Dprep_Stoppreparation();
				cl_bp_controller(CL_BP_EVENT_STOP,0);
				
				sv_cntrl_buzzer();	
				sv_cntrl_setredalarm();
				if((cl_dprepstate == CL_DPREP_STATE_DIALISER_PRIME) ||(cl_dprepstate == CL_DPREP_STATE_DPRIME_WAIT_STABILITY) || (cl_dprepstate == CL_DPREP_STATE_DREP_PRIME_ALARM_BYPASS))
				{
					//Cl_Dprep_Stoppreparation();
					Cl_Dprep_Priming_Stop();
				//	Cl_dprep_StopMixing();
			//		cl_dprep_primecontroller(CL_DPREP_PRIME_PRIME_STOP_DIALYSER_PRIMING,0);
			//		cl_dprepstate = CL_DPREP_STATE_DIALISER_PRIME_STOPPED;
			//		Cl_deactivate_prep_alarms();
				}
				else{
					
				
				}
				
// 				sv_cntrl_enable_bypass();
// 				cl_dprepbypassstate = CL_DPREP_BYPASS_ON;
// 				cl_dprep_primecontroller(CL_DPREP_PRIME_PRIME_ALARM,0);
// 				if((cl_dprepstate == CL_DPREP_STATE_DIALISER_PRIME) ||(cl_dprepstate == CL_DPREP_STATE_DPRIME_WAIT_STABILITY) || (cl_dprepstate == CL_DPREP_STATE_DREP_PRIME_ALARM_BYPASS))
// 				{	
// 				
// 					cl_dprepstate = CL_DPREP_STATE_DREP_PRIME_ALARM_BYPASS;
// 				}
				// 
			break;

// 			case BP_STOPPED:
// 			sv_cntrl_setredalarm();
// 			
// 			break;

			case ACID_IN_OPEN:
			case BICARB_IN_OPEN:
			case HOLDER1STATUS_OPEN:
			case HOLDER2STATUS_OPEN:
				
				sv_cntrl_buzzer();
				sv_cntrl_setgreenalarm();
				sv_cntrl_setredalarm();
				if((cl_dprepstate == CL_DPREP_STATE_DIALISER_PRIME) ||(cl_dprepstate == CL_DPREP_STATE_DPRIME_WAIT_STABILITY) || (cl_dprepstate == CL_DPREP_STATE_DREP_PRIME_ALARM_BYPASS))
				{
				sv_cntrl_setyellowalarm();
				sv_cntrl_enable_bypass();
				cl_dprep_primecontroller(CL_DPREP_PRIME_PRIME_STOP,0);
				cl_dprepbypassstate = CL_DPREP_BYPASS_ON;	
			//		cl_dprepstate = CL_DPREP_STATE_DREP_PRIME_ALARM_BYPASS;
					
				//	Cl_dprep_StopMixing();
				//	cl_dprep_primecontroller(CL_DPREP_PRIME_PRIME_STOP_DIALYSER_PRIMING,0);
				//	cl_dprepstate = CL_DPREP_STATE_DPRIME_WAIT_STABILITY;
				}
				else{
					
				//sv_cntrl_setredalarm();
				
				Cl_Dprep_Stoppreparation();
				cl_dprepstate = CL_DPREP_STATE_CRITICAL_ALARM;	
				}
				
			break;
			
			case HOLDER1STATUS_CLOSED:
			case HOLDER2STATUS_CLOSED:
			case ACID_IN_CLOSED:
			case BICARB_IN_CLOSED:
				
		//	sv_cntrl_setyellowalarm();
			sv_cntrl_buzzer();
			if((cl_dprepstate == CL_DPREP_STATE_DIALISER_PRIME)||(cl_dprepstate == CL_DPREP_STATE_DPRIME_WAIT_STABILITY) || (cl_dprepstate == CL_DPREP_STATE_DREP_PRIME_ALARM_BYPASS))
			{
				sv_cntrl_setyellowalarm();
				sv_cntrl_enable_bypass();
				cl_dprepbypassstate = CL_DPREP_BYPASS_ON;
			//	cl_dprep_primecontroller(CL_DPREP_PRIME_PRIME_STOP,0);
			//	cl_dprepstate = CL_DPREP_STATE_DREP_PRIME_ALARM_BYPASS;
				
			}
			else{
				
				Cl_Dprep_Stoppreparation();
				cl_dprepstate = CL_DPREP_STATE_CRITICAL_ALARM;
			}
			
			break;
			
			
			case LEVELSWITCH_OFF_TO_ON:
			// TURN OFF  WATER INLET
		//	Cl_SysStat_GetSensor_Status_Query(LEVELSWITCH_OFF_TO_ON,&levelswitchstatus);
			{
				if(levelswitchstatus == 1)
				{
	
					//fillseccounter++;

			//		Cl_DprepFlowOff();
					//cl_gfillinprogress = false;		
					
				}
			}
			break;
			case LEVELSWITCH_ON_TO_OFF:
			// TURN ON WATER INLET
			//Cl_SysStat_GetSensor_Status_Query(LEVELSWITCH_ON_TO_OFF,&levelswitchstatus);
			{
			//	if(levelswitchstatus == 0)
				{
					
					//fillseccounter++;

			//		Cl_DprepFlowOn();
					//cl_gfillinprogress = true;
					
				}
			}
			break;
			case TEMP3_HIGH_THRESHOLD:
			case TEMP3_LOW_THRESHOLD:
			case 	COND_STATUS_HIGH:
			case	COND_STATUS_LOW:
			
				
				sv_cntrl_buzzer();
			
			if((cl_dprepstate == CL_DPREP_STATE_DIALISER_PRIME) ||(cl_dprepstate == CL_DPREP_STATE_DPRIME_WAIT_STABILITY) || (cl_dprepstate == CL_DPREP_STATE_DREP_PRIME_ALARM_BYPASS))
			{
				//Cl_Dprep_Stoppreparation();
				sv_cntrl_setyellowalarm();
				sv_cntrl_enable_bypass();
				cl_dprepbypassstate = CL_DPREP_BYPASS_ON;
				//cl_dprep_primecontroller(CL_DPREP_PRIME_PRIME_STOP,0);
				state_counter++;
				Cl_AlarmActivateAlarms(COND_STATUS_HIGH,false );
				Cl_AlarmActivateAlarms(COND_STATUS_LOW,false );
				Cl_AlarmActivateAlarms(TEMP3_HIGH_THRESHOLD,false );
				Cl_AlarmActivateAlarms(TEMP3_LOW_THRESHOLD,false );
				
				cl_dprepstate = CL_DPREP_STATE_DPRIME_WAIT_STABILITY;
			}
			else{
				//sv_cntrl_setredalarm();
				Cl_Dprep_Stoppreparation();
				cl_dprepstate = CL_DPREP_STATE_CRITICAL_ALARM;
			}
			break;
			
		
			default:
			cl_dprepalarmid = _NO_ALARM;
			break;
	}
		 if(cl_dprepalarmid != _NO_ALARM)
		 {
			data[0] = (Cl_Uint8Type)cl_dprepalarmid;
			data[1] = (Cl_Uint8Type) CL_ALARM_TRIGGERED;
			data[2] = (Cl_Uint8Type)CRITICAL;
			command = CON_TX_COMMAND_ALARM;

			Cl_dprepretcode = Cl_SendDatatoconsole(command,&data,3);
			
					
			command = CON_TX_COMMAND_CL_SYS_STATE;
			data[0] = (Cl_Uint8Type)POST_CLEAN_STANDBY;
			data[1] = (Cl_Uint8Type)PRIME_IDLE;
			Cl_dprepretcode = Cl_SendDatatoconsole(command,&data,2);
		 }
			
	return CL_OK;	
}
	
	

Cl_ReturnCodeType  Cl_dprepSendAlarm(Cl_NewAlarmIdType cl_dprepalarmId )
{
	
			Cl_ReturnCodeType Cl_RetVal = CL_OK;
			 
			 
			Cl_Uint8Type data[3] ={0, 0} ,*data1 = NULL;
			Cl_Uint8Type data3;
			Cl_ConsoleTxCommandtype command = CON_TX_COMMAND_COMMAND_MAX;
			sv_cntrl_setredalarm();
			sv_cntrl_buzzer();
			if(cl_dprepalarmId != _NO_ALARM)
			 {
				data[0] = (Cl_Uint8Type)cl_dprepalarmId;
				data[1] = (Cl_Uint8Type) CL_ALARM_TRIGGERED;
				data[2] = (Cl_Uint8Type)CRITICAL;
				command = CON_TX_COMMAND_ALARM;
			
				Cl_RetVal = Cl_SendDatatoconsole(command,&data,3);
				cl_wait(10);

			 }
}	
	Cl_ReturnCodeType Cl_Dprep_CheckforfillingCompletion(void)
	{
		Cl_ReturnCodeType Cl_dprepretcode = CL_ERROR;
		Cl_Int16Type cl_cond,cl_temp3;
		float cond,cond_comp,temp;
		float cl_cond_ms,cl_temp3_cel;
		Cl_SendDatatoconsole(CON_TX_COMMAND_PRINTDATA,&Cl_Dprep_filling_TotalMinutescounter,2);
		if(Cl_Dprep_filling_TotalMinutescounter > CL_DPREP_FILLING_TIMEOUT_MIN )
		{
			
			//Cl_dprepretcode = CL_REJECTED;
			// Rise alarm
			Cl_dprepretcode = CL_OK;
	//	}                                           // manav02 commented else statement , timeout has to be finished
	//	else
//		{
			Cl_SysStat_GetSensor_Status_Query(SENSOR_TEMP3STATUS,&cl_temp3);
			Cl_SysStat_GetSensor_Status_Query(SENSOR_COND_STATUS,&cl_cond);
	
// 			uint16_t cal_data  = (402 *100* cl_temp3)/(2*32768);
// 			res_temp_lookuptable(cal_data);
// 			float temp = res_temp_value/100;
// //			cl_temp3_cel = temp - 3.1 + 1.4;										// sensor offset 
// 			cl_temp3_cel = temp - 0.4;										// sensor offset 
// 			
// 			//temp3 = ((temp3* 4) +temp)/5;
// 			
// 							
// 			//cl_temp3_cel = 0.0000116 * temp *temp + 0.0035 *temp + 11.157;

		cl_temp3_cel = cl_temp3 /100;
			
									//	cond =(cond*5 + cond_final_cs3)/6;
			cond =  -0.0001 * cl_cond* cl_cond  + 0.36 * cl_cond  - 38.39 ;
			cond_comp= (cond/(1+(cl_temp3_cel-25.0)*0.020))/10;
			//tempdata.word = (cond_comp);
			  
			if((cond_comp > 13.3) && (cond_comp < 13.8))
			{
				if ( cl_temp3_cel > 36.5 && cl_temp3_cel < 37.5)
				{
					if(cl_temp3_stable_confirm == true)
							Cl_dprepretcode = CL_OK;
				}
				
			}

			
		}
		
		return Cl_dprepretcode;
		
	}
	
	
	Cl_ReturnCodeType cl_prep_checkforprimecompletion(void)
	{
		Cl_ReturnCodeType Cl_dprepretcode = CL_ERROR;

		
		if(Cl_Dprep_Prime_TotalMinutescounter > CL_DPREP_PRIME_TIMEOUT_MIN )
		{
			
			Cl_dprepretcode = CL_OK;
		}

		
		return Cl_dprepretcode;
		
	}
	
	
	Cl_ReturnCodeType  Cl_DprepSelectDialysateInlet(void)
	{
		Cl_ReturnCodeType Cl_RetVal = CL_OK;
		//sv_cntrl_deactivate_valve(VALVE_ID18);
		//sv_cntrl_activate_valve(VALVE_ID19);
		sv_cntrl_activate_valve(VALVE_ID18);
		return Cl_RetVal;
	}
	Cl_ReturnCodeType  Cl_DprepFlowOn(void )
	{
		Cl_ReturnCodeType Cl_RetVal = CL_OK;
		sv_cntrl_activate_valve(VALVE_ID1);
		
		return Cl_RetVal;
	
	}

	Cl_ReturnCodeType  Cl_DprepFlowOff(void )
	{
		Cl_ReturnCodeType Cl_RetVal = CL_OK;
		sv_cntrl_deactivate_valve(VALVE_ID1);
		
		return Cl_RetVal;
	
	}
		//	#endif
		
		
Cl_ReturnCodeType UpdateDprepMinuteTick(void)
{
					Cl_ReturnCodeType Cl_dpreptcode = CL_OK;
					Cl_Dprepsecondscounter = 0;
					Cl_DprepMinutescounter++;
					Cl_DprepTotalMinutescounter++;
					Cl_dpreptcode = Cl_Dprep_UpdateTimeInfo();
					return Cl_dpreptcode;
}

Cl_ReturnCodeType UpdateDprepFillingMinuteTick(void)
{
					Cl_ReturnCodeType Cl_dpreptcode = CL_OK;

					Cl_Dprep_filling_secondscounter = 0;
					Cl_Dprep_filling_Minutescounter++;
					Cl_Dprep_filling_TotalMinutescounter++;

			//		Cl_dpreptcode = Cl_Dprep_UpdateFillingTimeInfo();
					return Cl_dpreptcode;
					

}

Cl_ReturnCodeType UpdateDprepPrimeMinuteTick(void)
{
	Cl_ReturnCodeType Cl_dpreptcode = CL_OK;

	Cl_Dprep_Prime_secondscounter = 0;
	Cl_Dprep_Prime_Minutescounter++;
	Cl_Dprep_Prime_TotalMinutescounter++;

//	Cl_dpreptcode = Cl_Dprep_UpdatePrimeTimeInfo();
	return Cl_dpreptcode;
	

}

Cl_ReturnCodeType UpdateDprepDialyserPrimeMinuteTick(void)
{
	Cl_ReturnCodeType Cl_dpreptcode = CL_OK;

	Cl_Dprep_DialyserPrime_secondscounter = 0;
	Cl_Dprep_DialyserPrime_Minutescounter++;
	Cl_Dprep_DialyserPrime_TotalMinutescounter++;

	Cl_dpreptcode = Cl_Dprep_UpdateDialyserPrimeTimeInfo();
	return Cl_dpreptcode;
	

}


Cl_ReturnCodeType Cl_Dprep_SendtreatementData(void)
{

	Cl_Uint8Type systemdataarray[40] =  {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
	cl_PrepDatatype tempdata;
	static Cl_Uint32Type timecount = 0;
	Cl_ReturnCodeType Cl_dprepretcode = CL_OK;
	Cl_Uint8Type count = 0;
	Cl_ConsoleTxCommandtype command = CON_TX_COMMAND_COMMAND_MAX;
	
	//Cl_Console_bulkdatatype
	
	
						
	command = CON_TX_COMMAND_CL_SYS_STATE_DATA ;
	systemdataarray[0] = TREATMENT_DATA;
	count++;


/*
	Cl_dprepretcode = (Cl_Uint8Type)sv_nvmgetdata(NV_DIALYSATE_FLOW, &tempdata);
	systemdataarray[count++] = tempdata.bytearray[0];
	systemdataarray[count++] = tempdata.bytearray[1];
	Cl_dprepretcode = (Cl_Uint8Type)sv_nvmgetdata(NV_ARTERIAL_BLOODFLOW_RATE, &tempdata);
	systemdataarray[count++] = tempdata.bytearray[0];
	systemdataarray[count++] = tempdata.bytearray[1];
	Cl_dprepretcode = (Cl_Uint8Type)sv_nvmgetdata(NV_HEPARIN_FLOW_RATE, &tempdata);
	systemdataarray[count++] = tempdata.bytearray[0];
	systemdataarray[count++] = tempdata.bytearray[1];
	Cl_dprepretcode = (Cl_Uint8Type)sv_nvmgetdata(NV_UF_REMOVAL_RATE, &tempdata);

	systemdataarray[count++] = tempdata.bytearray[0];
	systemdataarray[count++] = tempdata.bytearray[1];
	Cl_dprepretcode = (Cl_Uint8Type)sv_nvmgetdata(NV_HEPARIN_BOLUS, &tempdata);
						
	systemdataarray[count++] = tempdata.bytearray[0];
	systemdataarray[count++] = tempdata.bytearray[1];
	Cl_dprepretcode = (Cl_Uint8Type)sv_nvmgetdata(NV_UF_GOAL, &tempdata);
	systemdataarray[count++] = tempdata.bytearray[0];
	systemdataarray[count++] = tempdata.bytearray[1];
	Cl_dprepretcode = (Cl_Uint8Type)sv_nvmgetdata(NV_CONDUCTIVITY, &tempdata);
	Cl_SysStat_GetSensor_Status_Query(COND_STATUS_HIGH,&tempdata);
	systemdataarray[count++] = tempdata.bytearray[0];
	systemdataarray[count++] = tempdata.bytearray[1];
	//Cl_dprepretcode = (Cl_Uint8Type)sv_nvmgetdata(NV_DIALYSATE_TEMP, &tempdata);
	Cl_SysStat_GetSensor_Status_Query(SENSOR_TEMP3STATUS,&tempdata);
	systemdataarray[count++] = tempdata.bytearray[0];
	systemdataarray[count++] = tempdata.bytearray[1];
						
	//Cl_dprepretcode = (Cl_Uint8Type)sv_nvmgetdata(APTSTATUS_HIGH, &tempdata);
	Cl_SysStat_GetSensor_Status_Query(APTSTATUS_HIGH,&tempdata);
	systemdataarray[count++] = tempdata.bytearray[0];
	systemdataarray[count++] = tempdata.bytearray[1];
	//Cl_dprepretcode = (Cl_Uint8Type)sv_nvmgetdata(VPTSTATUS_HIGH, &tempdata);
	Cl_SysStat_GetSensor_Status_Query(VPTSTATUS_HIGH,&tempdata);
	systemdataarray[count++] = tempdata.bytearray[0];
	systemdataarray[count++] = tempdata.bytearray[1];
						
	systemdataarray[count++] = 0xAA;
	systemdataarray[count++] = 0xAA;
*/

//	Cl_dprepretcode = Cl_SendDatatoconsole(command,&systemdataarray,count);
return CL_OK;	
	
}
Cl_ReturnCodeType Cl_Dprep_SendPrepStateData(Cl_Console_bulkdatatype datatype)
{
	static float cond = 0;
	Cl_Uint8Type systemdataarray[40] =  {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
	cl_PrepDatatype tempdata;
	Cl_Uint16Type temp=0, temp1=0, sensordata = 0 , temp2 = 0 , temp2ui = 0 , apt=0,vpt=0,TMP =0;
	static Cl_Uint32Type timecount = 0;
	static float tmp3=0;
	Cl_ReturnCodeType Cl_dprepretcode = CL_OK;
	Cl_Uint8Type count = 0;
	Cl_ConsoleTxCommandtype command = CON_TX_COMMAND_COMMAND_MAX;
	Cl_Uint16Type sensordatamillivolts;
	
	
	/*
	COND, // eg 138
	TEMP3, // eg 370
	APT,
	VPT,
	TMP,
	*/
	
	

					command = CON_TX_COMMAND_CL_SYS_STATE_DATA ;
					systemdataarray[0] = datatype;
					count++;
					
						Cl_SysStat_GetSensor_Status_Query(SENSOR_TEMP3STATUS,&sensordata);
						{
							
// 							float ftemp,ftemp1,temp1;
// 
// 							uint16_t cal_data  = (402 *100* sensordata)/(2*32768);
// 							res_temp_lookuptable(cal_data);
// 							uint16_t temp = res_temp_value/10;
// //							temp = temp - 3.1 + 1.4;										// sensor offset 
// //							temp = temp - 3.1 -2;										// sensor offset 
// //							tmp3 =(tmp3*5 + temp)/6;
// 							tmp3 = temp - 0.4;
							tmp3 = (float)sensordata/10;

							
						}
						// manav02 added
						Cl_SysStat_GetSensor_Status_Query(SENSOR_COND_STATUS,&sensordata);
					
								tempdata.word = (sensordata)/100;
								
								systemdataarray[count++] = tempdata.bytearray[0];
								systemdataarray[count++] = tempdata.bytearray[1];
								systemdataarray[count++] = tempdata.bytearray[2];
								systemdataarray[count++] = tempdata.bytearray[3];
								
					
				// manav02 commented	
			/*		Cl_SysStat_GetSensor_Status_Query(SENSOR_COND_STATUS,&sensordata);
					{
					
						float cond_comp=0;
						float temp2=0;
						float	 cond1,	cond_final, cond1x100, cond_final_X10000;
					//	temp2 = temp * 0.8036;
					sensordata = sensordata/10;
						if (sensordata > 50)
						{
								cond1 = sensordata/100;
								cond_final = 0.0001*cond1*cond1 + 0.032*cond1 +0.91 + 0.4;
								cond1x100 = cond_final*100; ;
								cond_final_X10000= (cond1x100/(1+(tmp3/10  -25.0)*0.021));
								cond_final_X10000 =	cond_final_X10000 * 5.3 ;
								tempdata.word = (cond_final_X10000/10);
								
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

					
		
					} */
					// temperature is already calculated and now store into system data
					{

						tempdata.word = tmp3;
				//		tempdata.word = (tmp3 * 10);
				//		tempdata.word = (tmp3/10 - 25 ); // sanjeer : 25 is offset work around for intergation testing
						systemdataarray[count++] = tempdata.bytearray[0];
						systemdataarray[count++] = tempdata.bytearray[1];
						systemdataarray[count++] = tempdata.bytearray[2];
						systemdataarray[count++] = tempdata.bytearray[3];
					}
					Cl_SysStat_GetSensor_Status_Query(SENSOR_APTSTATUS,&apt);     // manav03 changed from tempdata.Twobyte to apt
					{
						
						tempdata.word	 = apt;
						systemdataarray[count++] = tempdata.bytearray[0] ;
						systemdataarray[count++] = tempdata.bytearray[1] ;
						systemdataarray[count++] = tempdata.bytearray[2] ;
						systemdataarray[count++] = tempdata.bytearray[3] ;
					}
					Cl_SysStat_GetSensor_Status_Query(SENSOR_VPTSTATUS,&vpt);    // manav03 changed from tempdata.Twobyte to vpt
					{
						
						tempdata.word	 = vpt;
						systemdataarray[count++] = tempdata.bytearray[0] ;
						systemdataarray[count++] = tempdata.bytearray[1] ;
						systemdataarray[count++] = tempdata.bytearray[2] ;
						systemdataarray[count++] = tempdata.bytearray[3] ;
					}


					if( datatype == DIALYSIS_PREP_DATA)
					{
						TMP = 5000;
						tempdata.word	 = (TMP);
						systemdataarray[count++] = tempdata.bytearray[0] ;
						systemdataarray[count++] = tempdata.bytearray[1] ;
						systemdataarray[count++] = tempdata.bytearray[2] ;
						systemdataarray[count++] = tempdata.bytearray[3] ;
					}
		
		Cl_SysStat_GetSensor_Status_Query(SENSOR_TEMP2STATUS,&temp2ui);    // manav03 changed variable name
		{
			
			tempdata.word = temp2ui/10 ;
			systemdataarray[count++] = tempdata.bytearray[0] ;
			systemdataarray[count++] = tempdata.bytearray[1] ;
			systemdataarray[count++] = tempdata.bytearray[2] ;
			systemdataarray[count++] = tempdata.bytearray[3] ;
		}
		
					Cl_dprepretcode = Cl_SendDatatoconsole(CON_TX_COMMAND_CL_SYS_STATE_DATA,&systemdataarray,count);
					//Cl_Dlsis_SenddlsisData();
	return CL_OK;
}

// Cl_ReturnCodeType Cl_Dprep_SendPrepStateData(Cl_Console_bulkdatatype datatype)
// {
// 	static float cond = 0;
// 	Cl_Uint8Type systemdataarray[40] =  {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
// 	cl_PrepDatatype tempdata;
// 	Cl_Uint16Type temp=0, temp1=0, sensordata = 0;
// 	static Cl_Uint32Type timecount = 0;
// 	static float tmp3=0,apt=0,vpt=0,TMP =0;
// 	Cl_ReturnCodeType Cl_dprepretcode = CL_OK;
// 	Cl_Uint8Type count = 0;
// 	Cl_ConsoleTxCommandtype command = CON_TX_COMMAND_COMMAND_MAX;
// 	Cl_Uint16Type sensordatamillivolts;
// 	
// 	
// 	/*
// 	COND, // eg 138
// 	TEMP3, // eg 370
// 	APT,
// 	VPT,
// 	TMP,
// 	*/
// 	
// 	
// 
// 					command = CON_TX_COMMAND_CL_SYS_STATE_DATA ;
// 					systemdataarray[0] = datatype;
// 					count++;
// 					
// 						Cl_SysStat_GetSensor_Status_Query(SENSOR_TEMP3STATUS,&sensordata);
// 						{
// 							
// // 							float ftemp,ftemp1,temp1;
// // 
// // 							uint16_t cal_data  = (402 *100* sensordata)/(2*32768);
// // 							res_temp_lookuptable(cal_data);
// // 							uint16_t temp = res_temp_value/10;
// // //							temp = temp - 3.1 + 1.4;										// sensor offset 
// // //							temp = temp - 3.1 -2;										// sensor offset 
// // //							tmp3 =(tmp3*5 + temp)/6;
// // 							tmp3 = temp - 0.4;
// 							tmp3 = (float)sensordata/10;
// 
// 							
// 						}
// 						
// // 					Cl_SysStat_GetSensor_Status_Query(SENSOR_COND_STATUS,&sensordata);
// // 			{
// // 			float conductivity_CS3;
// // 			conductivity_CS3 = cl_utilities_CalConductivity(sensordata,tmp3);
// // //			avg_conductivity = (avg_conductivity *9 + conductivity_CS3)/10;
// //      		avgcond = avgcond + conductivity_CS3;
// // 			
// // 			if (Count_500ms == 11)
// // 			{
// // 				avgcond = avgcond/10;
// // 				Count_500ms =0;
// // 				avg_conductivity = avgcond;
// // 
// // 			}
// // 			
// // 			tempdata.word = avg_conductivity;
// // 								
// // 								systemdataarray[count++] = tempdata.bytearray[0];
// // 								systemdataarray[count++] = tempdata.bytearray[1];
// // 								systemdataarray[count++] = tempdata.bytearray[2];
// // 								systemdataarray[count++] = tempdata.bytearray[3];	
// // 			}
// 					Cl_SysStat_GetSensor_Status_Query(SENSOR_COND_STATUS,&sensordata);
// 					{
// 					
// 						float cond_comp=0;
// 						float temp2=0;
// 						float	 cond1,	cond_final, cond1x100, cond_final_X10000;
// 					//	temp2 = temp * 0.8036;
// 					sensordata = sensordata/10;
// 						if (sensordata > 50)
// 						{
// 						//	calibration_cond(temp2);
// 						//	cond =(cond*5 + cond_final_cs3)/6;
// 						
// 								
// 								//	sensordata  = 25600;
// 								
// 								cond1 = sensordata/100;
// 								cond_final = 0.0001*cond1*cond1 + 0.032*cond1 +0.91 + 0.4;
// 								cond1x100 = cond_final*100; ;
// 								cond_final_X10000= (cond1x100/(1+(tmp3/10  -25.0)*0.021));
// 								cond_final_X10000 =	cond_final_X10000 * 5.3 ;
// 								
// 								
// 							//	cond =  -0.0001 * temp* temp  + 0.36 * temp  - 38.39 ;
// 							//	cond_comp= cond/(1+(tmp3/100-25.0)*0.020);
// 								tempdata.word = (cond_final_X10000/10);
// 								
// 								systemdataarray[count++] = tempdata.bytearray[0];
// 								systemdataarray[count++] = tempdata.bytearray[1];
// 								systemdataarray[count++] = tempdata.bytearray[2];
// 								systemdataarray[count++] = tempdata.bytearray[3];
// 							}
// 							else
// 							{
// 								tempdata.word = 0;
// 							systemdataarray[count++] = tempdata.bytearray[0];
// 							systemdataarray[count++] = tempdata.bytearray[1];
// 							systemdataarray[count++] = tempdata.bytearray[2];
// 							systemdataarray[count++] = tempdata.bytearray[3];
// 						}
// 
// 					
// 		
// 					} 
// 					// temperature is already calculated and now store into system data
// 					{
// 
// 						tempdata.word = tmp3;
// 				//		tempdata.word = (tmp3 * 10);
// 				//		tempdata.word = (tmp3/10 - 25 ); // sanjeer : 25 is offset work around for intergation testing
// 						systemdataarray[count++] = tempdata.bytearray[0];
// 						systemdataarray[count++] = tempdata.bytearray[1];
// 						systemdataarray[count++] = tempdata.bytearray[2];
// 						systemdataarray[count++] = tempdata.bytearray[3];
// 					}
// 					Cl_SysStat_GetSensor_Status_Query(SENSOR_APTSTATUS,&tempdata.Twobyte);
// 					{
// 						sensordatamillivolts = (tempdata.Twobyte * 0.8036) ;
// 						calibration_apt(sensordatamillivolts);
// 						apt =(apt*5 + pressure_final_apt)/6;
// 						tempdata.word	 = apt;
// 						systemdataarray[count++] = tempdata.bytearray[0] ;
// 						systemdataarray[count++] = tempdata.bytearray[1] ;
// 						systemdataarray[count++] = tempdata.bytearray[2] ;
// 						systemdataarray[count++] = tempdata.bytearray[3] ;
// 					}
// 					Cl_SysStat_GetSensor_Status_Query(SENSOR_VPTSTATUS,&tempdata.Twobyte);
// 					{
// 						sensordatamillivolts = (tempdata.Twobyte * 0.8036) ;
// 						calibration_vpt(sensordatamillivolts);
// 						vpt =(vpt*5 + pressure_final_vpt)/6;
// 						tempdata.word	 = (vpt);
// 						systemdataarray[count++] = tempdata.bytearray[0] ;
// 						systemdataarray[count++] = tempdata.bytearray[1] ;
// 						systemdataarray[count++] = tempdata.bytearray[2] ;
// 						systemdataarray[count++] = tempdata.bytearray[3] ;
// 					}
// 
// 
// 					if( datatype == DIALYSIS_PREP_DATA)
// 					{
// 						TMP = 5000;
// 						tempdata.word	 = (TMP);
// 						systemdataarray[count++] = tempdata.bytearray[0] ;
// 						systemdataarray[count++] = tempdata.bytearray[1] ;
// 						systemdataarray[count++] = tempdata.bytearray[2] ;
// 						systemdataarray[count++] = tempdata.bytearray[3] ;
// 					}
// 		
// 					Cl_dprepretcode = Cl_SendDatatoconsole(CON_TX_COMMAND_CL_SYS_STATE_DATA,&systemdataarray,count);
// 					//Cl_Dlsis_SenddlsisData();
// 	return CL_OK;
// }


Cl_ReturnCodeType Cl_Dprep_Stoppreparation(void)
{
	
	Cl_ReturnCodeType Cl_dprepretcode = CL_OK;
	Cl_Uint8Type data;
	
	sv_cntrl_setredalarm();
//	sv_cntrl_nobuzzer(); // manav01 added
	// deactivate all alarms
				if(!((cl_dprepstate == CL_DPREP_STATE_IDLE ) || (cl_dprepstate == CL_DPREP_STATE_STOPPED ) ||(cl_dprepstate == CL_DPREP_STATE_CRITICAL_ALARM )  ))
				{
					
					stability_counter = 0;
					wait_timeout_counter = 0;
					state_counter = 0;
					 	Cl_Dprepsecondscounter = 0;
					 	Cl_DprepMinutescounter= 0;
					 	Cl_Dprephourscounter= 0;
					 	Cl_DprepTotalMinutescounter= 0;
					 	Cl_DprepTotalhourscounter=0;
							Cl_Dprep_filling_secondscounter = 0;
							Cl_Dprep_filling_Minutescounter = 0;
							Cl_Dprep_filling_TotalMinutescounter = 0;
					
							Cl_Dprep_Prime_secondscounter = 0;
							Cl_Dprep_Prime_Minutescounter = 0;
							Cl_Dprep_Prime_TotalMinutescounter = 0;
					
							Cl_Dprep_DialyserPrime_secondscounter = 0;
							Cl_Dprep_DialyserPrime_Minutescounter = 0;
							Cl_Dprep_DialyserPrime_TotalMinutescounter = 0;
						
					Cl_deactivate_prep_alarms();							
							
							
							
				Cl_dprepretcode =  sv_cntrl_deactivatevenousclamp();
				Cl_dprepretcode = sv_cntrl_setflowpath(FLOW_PATH_IDLE_RINSE);
				Cl_dprepretcode = sv_cntrl_poweroffheater();
				Cl_dprepretcode = SetHeaterState(CL_HEATER_STATE_OFF);
				cl_dprep_primecontroller(CL_DPREP_PRIME_PRIME_STOP,0);
//Cl_dprepretcode = Cl_bc_controller(BC_EVENT_PAUSE);
	Cl_dprep_StopMixing();			
				if(cl_dprepstate == CL_DPREP_STATE_DIALISER_PRIME){
					cl_dprep_primecontroller(CL_DPREP_PRIME_PRIME_STOP_DIALYSER_PRIMING,0);
				}
				else{
					Cl_dprepretcode = Cl_SendDatatoconsole(CON_TX_COMMAND_DIALYSATE_FILLING_STOPPED,&data,0);
				}
			//	cl_dprep_primecontroller(CL_DPREP_PRIME_PRIME_STOP_DIALYSER_PRIMING,0);
				
			
				cl_dprepstate = CL_DPREP_STATE_IDLE;
				//Cl_dprepretcode = Cl_SendDatatoconsole(CON_TX_COMMAND_DIALYSATE_FILLING_STOPPED,&data,0);
				
					
				}
		return CL_OK;
}


Cl_ReturnCodeType Cl_Dprep_ResetAlertsforReassertion(void )
{
	Cl_ReturnCodeType 	Cl_dprepretcode = CL_OK;
	ClDprepAlarmIdType CldprepAlarmId;
	Cl_Uint8Type data;
	Cl_ConsoleTxCommandtype command = CON_TX_COMMAND_COMMAND_MAX;
	Cl_NewAlarmIdType cl_dprepalarmid;
	
	Cl_DprepAlarmTable[CL_DPREP_ALARM_BLOODDOOR_STATUS_OPEN].IsRaised = false;
	Cl_AlarmResetAlarm(BLOODDOOR_STATUS_OPEN);
	Cl_DprepAlarmTable[CL_DPREP_ALARM_HOLDER1STATUS_CLOSED].IsRaised = false;
	Cl_DprepAlarmTable[CL_DPREP_ALARM_HOLDER2STATUS_CLOSED].IsRaised = false;
	Cl_AlarmResetAlarm(HOLDER1STATUS_CLOSED);
	Cl_AlarmResetAlarm(HOLDER2STATUS_CLOSED);
	Cl_DprepAlarmTable[CL_DPREP_ALARM_HOLDER1STATUS_OPEN].IsRaised = false;
	Cl_DprepAlarmTable[CL_DPREP_ALARM_HOLDER2STATUS_OPEN].IsRaised = false;
	Cl_AlarmResetAlarm(HOLDER1STATUS_OPEN);
	Cl_AlarmResetAlarm(HOLDER2STATUS_OPEN);
	Cl_DprepAlarmTable[CL_DPREP_ALARM_ACID_IN].IsRaised = false;
	Cl_AlarmResetAlarm(ACID_IN_CLOSED);
	Cl_DprepAlarmTable[CL_DPREP_ALARM_BICAR_IN].IsRaised = false;
	Cl_AlarmResetAlarm(BICARB_IN_CLOSED);
	
	
	
	Cl_DprepAlarmTable[CL_DPREP_ALARM_COND_STATUS_LOW].IsRaised = false;
	Cl_DprepAlarmTable[CL_DPREP_ALARM_COND_STATUS_HIGH].IsRaised = false;
	Cl_AlarmResetAlarm(COND_STATUS_HIGH);
	Cl_AlarmResetAlarm(COND_STATUS_LOW);
	Cl_DprepAlarmTable[CL_DPREP_ALARM_COND_DAC_OPEN].IsRaised = false;
	Cl_DprepAlarmTable[CL_DPREP_ALARM_COND_DAC_RO].IsRaised = false;
	Cl_DprepAlarmTable[CL_DPREP_ALARM_COND_DAC_HIGH].IsRaised = false;
	Cl_DprepAlarmTable[CL_DPREP_ALARM_ABD_EVENT].IsRaised = false;
	Cl_DprepAlarmTable[CL_DPREP_ALARM_BD_EVENT].IsRaised = false;
	Cl_DprepAlarmTable[CL_DPREP_ALARM_BLD_EVENT].IsRaised = false;
	Cl_DprepAlarmTable[CL_DPREP_ALARM_APTSTATUS_HIGH].IsRaised = false;
	Cl_DprepAlarmTable[CL_DPREP_ALARM_VPTSTATUS_HIGH].IsRaised = false;
	Cl_DprepAlarmTable[CL_DPREP_ALARM_PS1_HIGH_THRESHOLD].IsRaised = false;
	Cl_DprepAlarmTable[CL_DPREP_ALARM_PS1_LOW_THRESHOLD].IsRaised = false;
	Cl_AlarmResetAlarm(PS1_HIGH_THRESHOLD);
	Cl_AlarmResetAlarm(PS1_LOW_THRESHOLD);
	Cl_DprepAlarmTable[CL_DPREP_ALARM_PS2_HIGH_THRESHOLD].IsRaised = false;
	Cl_DprepAlarmTable[CL_DPREP_ALARM_PS2_LOW_THRESHOLD].IsRaised = false;
	Cl_AlarmResetAlarm(PS2_HIGH_THRESHOLD);
	Cl_AlarmResetAlarm(PS2_LOW_THRESHOLD);
	Cl_DprepAlarmTable[CL_DPREP_ALARM_PS3_HIGH_THRESHOLD].IsRaised = false;
	Cl_DprepAlarmTable[CL_DPREP_ALARM_PS3_LOW_THRESHOLD].IsRaised = false;
	Cl_AlarmResetAlarm(PS3_HIGH_THRESHOLD);
	Cl_AlarmResetAlarm(PS3_LOW_THRESHOLD);
	
	Cl_DprepAlarmTable[CL_DPREP_ALARM_FLOW_NO_FLOW].IsRaised = false;
	Cl_DprepAlarmTable[CL_DPREP_ALARM_FLOW_LOW_FLOWRATE].IsRaised = false;
	Cl_DprepAlarmTable[CL_DPREP_ALARM_FLOW_HIGH_FLOWRATE].IsRaised = false;
	Cl_AlarmResetAlarm(FLOW_NO_FLOW);
	Cl_AlarmResetAlarm(FLOW_HIGH_FLOWRATE);
	Cl_AlarmResetAlarm(FLOW_LOW_FLOWRATE);
	
	
	Cl_DprepAlarmTable[CL_DPREP_ALARM_TEMP1_HIGH_THRESHOLD].IsRaised = false;
	Cl_DprepAlarmTable[CL_DPREP_ALARM_TEMP1_LOW_THRESHOLD].IsRaised = false;
	
	Cl_AlarmResetAlarm(TEMP1_HIGH_THRESHOLD);
	Cl_AlarmResetAlarm(TEMP1_LOW_THRESHOLD);
	Cl_DprepAlarmTable[CL_DPREP_ALARM_TEMP2_HIGH_THRESHOLD].IsRaised = false;
	Cl_DprepAlarmTable[CL_DPREP_ALARM_TEMP2_LOW_THRESHOLD].IsRaised = false;
	Cl_AlarmResetAlarm(TEMP2_HIGH_THRESHOLD);
	Cl_AlarmResetAlarm(TEMP2_LOW_THRESHOLD);
	Cl_DprepAlarmTable[CL_DPREP_ALARM_TEMP3_HIGH_THRESHOLD].IsRaised = false;
	Cl_DprepAlarmTable[CL_DPREP_ALARM_TEMP3_LOW_THRESHOLD].IsRaised = false;
	Cl_AlarmResetAlarm(TEMP3_HIGH_THRESHOLD);
	Cl_AlarmResetAlarm(TEMP3_LOW_THRESHOLD);

Cl_DprepAlarmTable[CL_DPREP_ALARM_BP_STOPPED].IsRaised = false;
	Cl_AlarmResetAlarm(BP_STOPPED);

Cl_DprepAlarmTable[CL_DPREP_ALARM_BCALARM].IsRaised = false;
Cl_AlarmResetAlarm(BC_ALARM);

	return (Cl_dprepretcode);
	
}


Cl_ReturnCodeType Cl_Dprep_Get_data(Cl_ConsoleRxDataType DataId, Cl_Uint8Type size)
{
	Cl_ReturnCodeType Cl_dprepretcode = CL_OK;
	Cl_ConsoleTxCommandtype command = CON_TX_COMMAND_COMMAND_MAX;
	Cl_Uint8Type dataarray[8] =  {0,0,0,0};
	Cl_Uint8Type  datasize = 0;
	
	if(DataId == CON_RX_PARAM_DATA_TREATMENT_DATA )
	{
		Cl_Dprep_SendtreatementData();
	}
	else
	{
			switch (DataId)
	{

						case	CON_RX_PARAM_DATA_RINSE_STATUS:
							//cl_wait(10);
						
							command = CON_TX_COMMAND_SYSDATA;
							//cl_stby_retval = (Cl_Uint8Type)sv_nvmgetdata(NV_RINSE_NVM_RINSE_STATUS, &data);
							dataarray[0] = CON_TX_PARAM_DATA_RINSE_STATUS;
							Cl_dprepretcode = (Cl_Uint8Type)sv_nvmgetdata(NV_NVM_RINSE_STATUS, &dataarray[1]);
							datasize = 2;
							//Reset the  OPENFILL time count
							//Cl_dprepretcode = Cl_SendDatatoconsole(command,&dataarray,1);
							//cl_wait(10);
							break;
							case	CON_RX_PARAM_DATA_PRIME_STATUS:
						//	cl_wait(10);
						
							command = CON_TX_COMMAND_SYSDATA;
							//cl_stby_retval = (Cl_Uint8Type)sv_nvmgetdata(NV_RINSE_NVM_RINSE_STATUS, &data);
							dataarray[0] = CON_TX_PARAM_DATA_PRIME_STATUS;
							Cl_dprepretcode = (Cl_Uint8Type)sv_nvmgetdata(NV_NVM_PRIME_STATUS, &dataarray[1]);
							datasize = 2;
							//Reset the  OPENFILL time count
							//Cl_dprepretcode = Cl_SendDatatoconsole(command,&dataarray,1);
							//cl_wait(10);
						break;
									
						case	CON_RX_PARAM_DATA_DIALYSIS_STATUS:
						//cl_wait(10);
			
						command = CON_TX_COMMAND_SYSDATA;

						dataarray[0] = CON_TX_PARAM_DATA_DIALYSIS_STATUS;
						Cl_dprepretcode = (Cl_Uint8Type)sv_nvmgetdata(NV_NVM_DIALYSIS_STATUS, &dataarray[1]);
						datasize = 2;

						break;
						case	CON_RX_PARAM_DATA_DIALYSIS_FLOW:
							//cl_wait(10);
						
							command = CON_TX_COMMAND_SYSDATA;
							//cl_stby_retval = (Cl_Uint8Type)sv_nvmgetdata(NV_RINSE_NVM_RINSE_STATUS, &data);
							dataarray[0] = CON_TX_PARAM_DATA_DIALYSIS_FLOW;
							Cl_dprepretcode = (Cl_Uint8Type)sv_nvmgetdata(NV_DIALYSATE_FLOW, &dataarray[1]);
							datasize = 2;
							//Reset the  OPENFILL time count
							//Cl_dprepretcode = Cl_SendDatatoconsole(command,&dataarray,3);
							//cl_wait(10);
							break;
							case	CON_RX_PARAM_DATA_DIALYSIS_TEMP:
							//cl_wait(10);
						
							command = CON_TX_COMMAND_SYSDATA;
							//cl_stby_retval = (Cl_Uint8Type)sv_nvmgetdata(NV_RINSE_NVM_RINSE_STATUS, &data);
							dataarray[0] = CON_TX_PARAM_DATA_DIALYSIS_TEMP;
							Cl_dprepretcode = (Cl_Uint8Type)sv_nvmgetdata(NV_DIALYSATE_TEMP, &dataarray[1]);
							datasize = 2;
							//Reset the  OPENFILL time count
							//Cl_dprepretcode = Cl_SendDatatoconsole(command,&dataarray,1);
							//cl_wait(10);
						break;
						case	CON_RX_PARAM_DATA_HEPARIN_RATE:
							//cl_wait(10);
						
							command = CON_TX_COMMAND_SYSDATA;
							//cl_stby_retval = (Cl_Uint8Type)sv_nvmgetdata(NV_RINSE_NVM_RINSE_STATUS, &data);
							dataarray[0] = CON_TX_PARAM_DATA_HEPARIN_RATE;
							//				Cl_dprepretcode = (Cl_Uint8Type)sv_nvmgetdata(NV_HEPARIN_RATE, &dataarray[1]);
						
						datasize = 2;
						break;
						case	CON_RX_PARAM_DATA_APT_VALUE:
							//cl_wait(10);
						
							command = CON_TX_COMMAND_SYSDATA;
							//cl_stby_retval = (Cl_Uint8Type)sv_nvmgetdata(NV_RINSE_NVM_RINSE_STATUS, &data);
							dataarray[0] = CON_TX_PARAM_DATA_APT_VALUE;
							//			Cl_dprepretcode = (Cl_Uint8Type)sv_nvmgetdata(NV_APT_VALUE, &dataarray[1]);
						
							//Reset the  OPENFILL time count
							//Cl_dprepretcode = Cl_SendDatatoconsole(command,&dataarray,1);
							//cl_wait(10);
							datasize = 2;
						break;
						case	CON_RX_PARAM_DATA_VPT_VALUE:
							//cl_wait(10);
						
							command = CON_TX_COMMAND_SYSDATA;
							//cl_stby_retval = (Cl_Uint8Type)sv_nvmgetdata(NV_RINSE_NVM_RINSE_STATUS, &data);
							dataarray[0] = CON_TX_PARAM_DATA_VPT_VALUE;
							//	Cl_dprepretcode = (Cl_Uint8Type)sv_nvmgetdata(NV_VPT_VALUE, &dataarray[1]);
						
							//Reset the  OPENFILL time count
							//Cl_dprepretcode = Cl_SendDatatoconsole(command,&dataarray,1);
							//cl_wait(10);
							datasize = 2;
						break;
						case	CON_RX_PARAM_DATA_TMP_VALUE:
							//cl_wait(10);
						
							command = CON_TX_COMMAND_SYSDATA;
							//cl_stby_retval = (Cl_Uint8Type)sv_nvmgetdata(NV_RINSE_NVM_RINSE_STATUS, &data);
							dataarray[0] = CON_TX_PARAM_DATA_TMP_VALUE;
							//			Cl_dprepretcode = (Cl_Uint8Type)sv_nvmgetdata(NV_RINSE_NVM_RINSE_STATUS, &dataarray[1]);
						
							//Reset the  OPENFILL time count
							//Cl_dprepretcode = Cl_SendDatatoconsole(command,&dataarray,1);
							//cl_wait(10);
							datasize = 2;
						break;
						case	CON_RX_PARAM_DATA_COND_VALUE:
							//cl_wait(10);
						
							command = CON_TX_COMMAND_SYSDATA;
							//cl_stby_retval = (Cl_Uint8Type)sv_nvmgetdata(NV_RINSE_NVM_RINSE_STATUS, &data);
							dataarray[0] = CON_TX_PARAM_DATA_COND_VALUE;
							//				Cl_dprepretcode = (Cl_Uint8Type)sv_nvmgetdata(NV_RINSE_NVM_RINSE_STATUS, &dataarray[1]);
						
							//Reset the  OPENFILL time count
							//Cl_dprepretcode = Cl_SendDatatoconsole(command,&dataarray,1);
							//cl_wait(10);
							datasize = 2;
						break;
						case	CON_RX_PARAM_DATA_UF_RATE:
						//	cl_wait(10);
						
							command = CON_TX_COMMAND_SYSDATA;
							//cl_stby_retval = (Cl_Uint8Type)sv_nvmgetdata(NV_RINSE_NVM_RINSE_STATUS, &data);
							dataarray[0] = CON_TX_PARAM_DATA_UF_RATE;
							Cl_dprepretcode = (Cl_Uint8Type)sv_nvmgetdata(NV_UF_REMOVAL_RATE, &dataarray[1]);
						
							//Reset the  OPENFILL time count
							//Cl_dprepretcode = Cl_SendDatatoconsole(command,&dataarray,1);
						//	cl_wait(10);
						datasize = 2;
						break;
						case	CON_RX_PARAM_DATA_ABF_RATE:
							//cl_wait(10);
						
							command = CON_TX_COMMAND_SYSDATA;
							//cl_stby_retval = (Cl_Uint8Type)sv_nvmgetdata(NV_RINSE_NVM_RINSE_STATUS, &data);
							dataarray[0] = CON_TX_PARAM_DATA_ABF_RATE;
							Cl_dprepretcode = (Cl_Uint8Type)sv_nvmgetdata(NV_ARTERIAL_BLOODFLOW_RATE, &dataarray[1]);
						
							//Reset the  OPENFILL time count
							//Cl_dprepretcode = Cl_SendDatatoconsole(command,&dataarray,1);
							//cl_wait(10);
							datasize = 2;
						break;
						#if 0
						case	CON_RX_PARAM_DATA_ABD_THRESHOLD:
						//cl_wait(10);
						
							command = CON_TX_COMMAND_SYSDATA;
							//cl_stby_retval = (Cl_Uint8Type)sv_nvmgetdata(NV_RINSE_NVM_RINSE_STATUS, &data);
							dataarray[0] = CON_TX_PARAM_DATA_ABD_THRESHOLD;
							Cl_dprepretcode = (Cl_Uint8Type)sv_nvmgetdata(NV_ABD_THRESHOLD, &dataarray[1]);
						
							//Reset the  OPENFILL time count
							//Cl_dprepretcode = Cl_SendDatatoconsole(command,&dataarray,1);
						//	cl_wait(10);
						break;
						case	CON_RX_PARAM_DATA_BLD_THRESHOLD:
						//cl_wait(10);
						
							command = CON_TX_COMMAND_SYSDATA;
							//cl_stby_retval = (Cl_Uint8Type)sv_nvmgetdata(NV_RINSE_NVM_RINSE_STATUS, &data);
							dataarray[0] = CON_TX_PARAM_DATA_BLD_THRESHOLD;
							Cl_dprepretcode = (Cl_Uint8Type)sv_nvmgetdata(NV_BLD_THRESHOLD, &dataarray[1]);
						
							//Reset the  OPENFILL time count
							//Cl_dprepretcode = Cl_SendDatatoconsole(command,&dataarray,1);
							//cl_wait(10);
							datasize = 2;
						break;
						#endif
						
						
						case CON_RX_PARAM_DATA_DIALYSIS_FLOW_CONFIG: //0x13
						
						
			
						
						dataarray[0] = CON_TX_PARAM_DATA_DIALYSIS_FLOW_CONFIG;
						Cl_dprepretcode = (Cl_Uint8Type)sv_nvmgetdata(NV_DIALYSATE_FLOW, &dataarray[1]);	
						Cl_dprepretcode = (Cl_Uint8Type)sv_nvmgetdata(NV_DIALYSIS_FLOW_CONFIG_UPPER, &dataarray[3]);
						Cl_dprepretcode = (Cl_Uint8Type)sv_nvmgetdata(NV_DIALYSIS_FLOW_CONFIG_LOWER, &dataarray[5]);	
						
						datasize = 	7;
							
					break;
					case CON_RX_PARAM_DATA_DIALYSIS_TEMP_CONFIG://0x14
											

											dataarray[0] = CON_TX_PARAM_DATA_DIALYSIS_TEMP_CONFIG;
											Cl_dprepretcode = (Cl_Uint8Type)sv_nvmgetdata(NV_DIALYSATE_TEMP, &dataarray[1]);
											Cl_dprepretcode = (Cl_Uint8Type)sv_nvmgetdata(NV_DIALYSIS_TEMP_CONFIG_UPPER, &dataarray[3]);
											Cl_dprepretcode = (Cl_Uint8Type)sv_nvmgetdata(NV_DIALYSIS_TEMP_CONFIG_LOWER, &dataarray[5]);
											datasize = 	7;
					break;
					case CON_RX_PARAM_DATA_HEPARIN_RATE_CONFIG://0x15

									dataarray[0] = CON_TX_PARAM_DATA_HEPARIN_RATE_CONFIG;
									Cl_dprepretcode = (Cl_Uint8Type)sv_nvmgetdata(NV_HEPARIN_FLOW_RATE, &dataarray[1]);
									Cl_dprepretcode = (Cl_Uint8Type)sv_nvmgetdata(NV_HEPARIN_RATE_CONFIG_UPPER, &dataarray[3]);
									Cl_dprepretcode = (Cl_Uint8Type)sv_nvmgetdata(NV_HEPARIN_RATE_CONFIG_LOWER, &dataarray[5]);
									datasize = 	7;
					break;
					case CON_RX_PARAM_DATA_APT_VALUE_CONFIG://0x16
							
									dataarray[0] = CON_TX_PARAM_DATA_APT_VALUE_CONFIG;
									Cl_dprepretcode = (Cl_Uint8Type)sv_nvmgetdata(NV_APT_VALUE, &dataarray[1]);
									Cl_dprepretcode = (Cl_Uint8Type)sv_nvmgetdata(NV_APT_VALUE_CONFIG_UPPER, &dataarray[3]);
									Cl_dprepretcode = (Cl_Uint8Type)sv_nvmgetdata(NV_APT_VALUE_CONFIG_LOWER, &dataarray[5]);
									datasize = 	7;
									
					break;
					case CON_RX_PARAM_DATA_VPT_VALUE_CONFIG://0x17

									dataarray[0] = CON_TX_PARAM_DATA_VPT_VALUE_CONFIG;
									Cl_dprepretcode = (Cl_Uint8Type)sv_nvmgetdata(NV_VPT_VALUE, &dataarray[1]);
									Cl_dprepretcode = (Cl_Uint8Type)sv_nvmgetdata(NV_VPT_VALUE_CONFIG_UPPER, &dataarray[3]);
									Cl_dprepretcode = (Cl_Uint8Type)sv_nvmgetdata(NV_VPT_VALUE_CONFIG_LOWER, &dataarray[5]);
									datasize = 	7;
					break;
					case CON_RX_PARAM_DATA_TMP_VALUE_CONFIG://0x18

									dataarray[0] = CON_TX_PARAM_DATA_TMP_VALUE_CONFIG;
									Cl_dprepretcode = (Cl_Uint8Type)sv_nvmgetdata(NV_TMP, &dataarray[1]);
									Cl_dprepretcode = (Cl_Uint8Type)sv_nvmgetdata(NV_TMP_VALUE_CONFIG_UPPER, &dataarray[3]);
									Cl_dprepretcode = (Cl_Uint8Type)sv_nvmgetdata(NV_TMP_VALUE_CONFIG_LOWER, &dataarray[5]);
									datasize = 	7;
					break;
					case CON_RX_PARAM_DATA_COND_VALUE_CONFIG://0x19

									dataarray[0] = CON_TX_PARAM_DATA_COND_VALUE_CONFIG;
									Cl_dprepretcode = (Cl_Uint8Type)sv_nvmgetdata(NV_CONDUCTIVITY, &dataarray[1]);
									Cl_dprepretcode = (Cl_Uint8Type)sv_nvmgetdata(NV_COND_VALUE_CONFIG_UPPER, &dataarray[3]);
									Cl_dprepretcode = (Cl_Uint8Type)sv_nvmgetdata(NV_COND_VALUE_CONFIG_LOWER, &dataarray[5]);
									datasize = 	7;
					break;
					case CON_RX_PARAM_DATA_UF_RATE_CONFIG://0x1A

									dataarray[0] = CON_TX_PARAM_DATA_UF_RATE_CONFIG;
									Cl_dprepretcode = (Cl_Uint8Type)sv_nvmgetdata(NV_UF_REMOVAL_RATE, &dataarray[1]);
									Cl_dprepretcode = (Cl_Uint8Type)sv_nvmgetdata(NV_UF_RATE_CONFIG_UPPER, &dataarray[3]);
									Cl_dprepretcode = (Cl_Uint8Type)sv_nvmgetdata(NV_UF_RATE_CONFIG_LOWER, &dataarray[5]);
									datasize = 	7;
					break;
					case CON_RX_PARAM_DATA_UF_GOAL_CONFIG://0x1B,

									dataarray[0] = CON_TX_PARAM_DATA_UF_GOAL_CONFIG;
									Cl_dprepretcode = (Cl_Uint8Type)sv_nvmgetdata(NV_UF_GOAL, &dataarray[1]);
									Cl_dprepretcode = (Cl_Uint8Type)sv_nvmgetdata(NV_UF_GOAL_CONFIG_UPPER, &dataarray[3]);
									Cl_dprepretcode = (Cl_Uint8Type)sv_nvmgetdata(NV_UF_GOAL_CONFIG_LOWER, &dataarray[5]);
									datasize = 	7;
					break;
					case CON_RX_PARAM_DATA_BOLUS_VOLUME_CONFIG://1C

									dataarray[0] = CON_TX_PARAM_DATA_BOLUS_VOLUME_CONFIG;
									Cl_dprepretcode = (Cl_Uint8Type)sv_nvmgetdata(NV_HEPARIN_BOLUS, &dataarray[1]);
									Cl_dprepretcode = (Cl_Uint8Type)sv_nvmgetdata(NV_BOLUS_VOLUME_CONFIG_UPPER, &dataarray[3]);
									Cl_dprepretcode = (Cl_Uint8Type)sv_nvmgetdata(NV_BOLUS_VOLUME_CONFIG_LOWER, &dataarray[5]);
									datasize = 	7;
					break;
					case CON_RX_PARAM_DATA_ABF_RATE_CONFIG://0X1D

									dataarray[0] = CON_TX_PARAM_DATA_ABF_RATE_CONFIG;
									Cl_dprepretcode = (Cl_Uint8Type)sv_nvmgetdata(NV_HEPARIN_BOLUS, &dataarray[1]);
									Cl_dprepretcode = (Cl_Uint8Type)sv_nvmgetdata(NV_ABF_RATE_CONFIG_UPPER, &dataarray[3]);
									Cl_dprepretcode = (Cl_Uint8Type)sv_nvmgetdata(NV_ABF_RATE_CONFIG_LOWER, &dataarray[5]);
									datasize = 	7;
					break;
						default:
						break;
	}
					command = CON_TX_COMMAND_SYSDATA;
					Cl_dprepretcode = Cl_SendDatatoconsole(command,&dataarray,datasize);
		
	}
return CL_OK;
}
Cl_ReturnCodeType Cl_Dprep_setdata(Cl_ConsoleRxDataType DataId,cl_PrepDatatype cl_PrepData , Cl_Uint8Type size)
{
	Cl_ReturnCodeType Cl_dprepretcode = CL_OK;
	Cl_ConsoleTxCommandtype command = CON_TX_COMMAND_COMMAND_MAX;
	Cl_Uint8Type dataarray[8] =  {0,0,0,0};
	Cl_Uint8Type  databytecnt = 0;

				switch(DataId)
				{
	 
					 case	CON_RX_PARAM_DATA_DIALYSIS_FLOW:
					 Cl_dprepretcode =sv_nvmsetdata(NV_DIALYSATE_FLOW,&cl_PrepData.bytearray[0],Cl_ConsoleRxMsg.datasize);					 
					 dataarray[0] = CON_TX_PARAM_DATA_DIALYSIS_FLOW;
					 Cl_dprepretcode = (Cl_Uint8Type)sv_nvmgetdata(NV_DIALYSATE_FLOW, &dataarray[1]);
					
					 break;
					 case	CON_RX_PARAM_DATA_DIALYSIS_TEMP:
					 Cl_dprepretcode =sv_nvmsetdata(NV_DIALYSATE_TEMP,&cl_PrepData.bytearray[0],Cl_ConsoleRxMsg.datasize);
					 dataarray[0] = CON_TX_PARAM_DATA_DIALYSIS_TEMP; 
					 Cl_dprepretcode = (Cl_Uint8Type)sv_nvmgetdata(NV_DIALYSATE_TEMP, &dataarray[1]);

					 break;
	 
					 case	CON_RX_PARAM_DATA_HEPARIN_RATE:
					 Cl_dprepretcode =sv_nvmsetdata(NV_HEPARIN_FLOW_RATE,&cl_PrepData.bytearray[0],Cl_ConsoleRxMsg.datasize);
					 dataarray[0] = CON_TX_PARAM_DATA_HEPARIN_RATE;
					 Cl_dprepretcode = (Cl_Uint8Type)sv_nvmgetdata(NV_HEPARIN_FLOW_RATE, &dataarray[1]);
					 break;
					 
					case CON_RX_PARAM_DATA_BOLUS_VOLUME:

					 Cl_dprepretcode =sv_nvmsetdata(NV_HEPARIN_BOLUS,&cl_PrepData.bytearray[0],Cl_ConsoleRxMsg.datasize);
					 dataarray[0] = CON_TX_PARAM_DATA_BOLUS_VOLUME;
					 Cl_dprepretcode = (Cl_Uint8Type)sv_nvmgetdata(NV_HEPARIN_BOLUS, &dataarray[1]);

					break;
					 case	CON_RX_PARAM_DATA_APT_VALUE:
					 Cl_dprepretcode =sv_nvmsetdata(NV_APT_VALUE,&cl_PrepData.bytearray[0],Cl_ConsoleRxMsg.datasize);
					 dataarray[0] = CON_TX_PARAM_DATA_APT_VALUE;	 
					 Cl_dprepretcode = (Cl_Uint8Type)sv_nvmgetdata(NV_APT_VALUE, &dataarray[1]);

					 break;
					 case	CON_RX_PARAM_DATA_VPT_VALUE:
					 Cl_dprepretcode =sv_nvmsetdata(NV_VPT_VALUE,&cl_PrepData.bytearray[0],Cl_ConsoleRxMsg.datasize);
					 dataarray[0] = CON_TX_PARAM_DATA_VPT_VALUE;	 
					 Cl_dprepretcode = (Cl_Uint8Type)sv_nvmgetdata(NV_VPT_VALUE, &dataarray[1]);

					 break;
					 case	CON_RX_PARAM_DATA_TMP_VALUE:

					 break;
					 case	CON_RX_PARAM_DATA_COND_VALUE:
					 Cl_dprepretcode =sv_nvmsetdata(NV_CONDUCTIVITY,&cl_PrepData.bytearray[0],Cl_ConsoleRxMsg.datasize);
					 dataarray[0] = CON_TX_PARAM_DATA_COND_VALUE;
					 Cl_dprepretcode = (Cl_Uint8Type)sv_nvmgetdata(NV_CONDUCTIVITY, &dataarray[1]);

					 break;

					 case	CON_RX_PARAM_DATA_UF_RATE:
					 Cl_dprepretcode =sv_nvmsetdata(NV_UF_REMOVAL_RATE,&cl_PrepData.bytearray[0],Cl_ConsoleRxMsg.datasize);
					 dataarray[0] = CON_TX_PARAM_DATA_UF_RATE;
					 Cl_dprepretcode = (Cl_Uint8Type)sv_nvmgetdata(NV_UF_REMOVAL_RATE, &dataarray[1]);

					 break;
					 case CON_RX_PARAM_DATA_UF_GOAL:
					 Cl_dprepretcode =sv_nvmsetdata(NV_UF_GOAL,&cl_PrepData.bytearray[0],Cl_ConsoleRxMsg.datasize);
					 dataarray[0] = CON_TX_PARAM_DATA_UF_GOAL;
					 Cl_dprepretcode = (Cl_Uint8Type)sv_nvmgetdata(NV_UF_GOAL, &dataarray[1]);

					 break;
					 case	CON_RX_PARAM_DATA_ABF_RATE:
					 Cl_dprepretcode =sv_nvmsetdata(NV_HEPARIN_FLOW_RATE,&cl_PrepData.bytearray[0],Cl_ConsoleRxMsg.datasize);					 
					 command = CON_TX_COMMAND_SYSDATA;					 
					 dataarray[0] = CON_TX_PARAM_DATA_ABF_RATE;
					 Cl_dprepretcode = (Cl_Uint8Type)sv_nvmgetdata(NV_ARTERIAL_BLOODFLOW_RATE, &dataarray[1]);					 
					 break;
					
					#if 0
					 case CON_RX_PARAM_DATA_ABD_THRESHOLD://0x0C
					 Cl_dprepretcode =sv_nvmsetdata(NV_ABD_THRESHOLD,&cl_PrepData.bytearray[0],Cl_ConsoleRxMsg.datasize);
					 command = CON_TX_COMMAND_SYSDATA;
					 dataarray[0] = CON_TX_PARAM_DATA_ABD_THRESHOLD;
					 Cl_dprepretcode = (Cl_Uint8Type)sv_nvmgetdata(NV_ABD_THRESHOLD, &dataarray[1]);		
					 
					 break;
					 case CON_RX_PARAM_DATA_BLD_THRESHOLD://0x0D
					 Cl_dprepretcode =sv_nvmsetdata(NV_BLD_THRESHOLD,&cl_PrepData.bytearray[0],Cl_ConsoleRxMsg.datasize);
					 command = CON_TX_COMMAND_SYSDATA;
					 dataarray[0] = CON_TX_PARAM_DATA_ABF_RATE;
					 Cl_dprepretcode = (Cl_Uint8Type)sv_nvmgetdata(NV_BLD_THRESHOLD, &dataarray[1]);
					 break;
					 
					#endif
					case CON_RX_PARAM_DATA_DIALYSIS_FLOW_CONFIG: //0x13
						
						
						
						Cl_dprepretcode =sv_nvmsetdata(NV_DIALYSATE_FLOW,&cl_PrepData.bytearray[0],2);
						Cl_dprepretcode =sv_nvmsetdata(NV_DIALYSIS_FLOW_CONFIG_UPPER,&cl_PrepData.bytearray[2],2);
						Cl_dprepretcode =sv_nvmsetdata(NV_DIALYSIS_FLOW_CONFIG_LOWER,&cl_PrepData.bytearray[4],2);
						
						dataarray[0] = CON_TX_PARAM_DATA_DIALYSIS_FLOW_CONFIG;
						Cl_dprepretcode = (Cl_Uint8Type)sv_nvmgetdata(NV_DIALYSATE_FLOW, &dataarray[1]);	
						Cl_dprepretcode = (Cl_Uint8Type)sv_nvmgetdata(NV_DIALYSIS_FLOW_CONFIG_UPPER, &dataarray[3]);
						Cl_dprepretcode = (Cl_Uint8Type)sv_nvmgetdata(NV_DIALYSIS_FLOW_CONFIG_LOWER, &dataarray[5]);		
							
					break;
					case CON_RX_PARAM_DATA_DIALYSIS_TEMP_CONFIG://0x14
											
											Cl_dprepretcode =sv_nvmsetdata(NV_DIALYSATE_TEMP,&cl_PrepData.bytearray[0],2);
											Cl_dprepretcode =sv_nvmsetdata(NV_DIALYSIS_TEMP_CONFIG_UPPER,&cl_PrepData.bytearray[2],2);
											Cl_dprepretcode =sv_nvmsetdata(NV_DIALYSIS_TEMP_CONFIG_LOWER,&cl_PrepData.bytearray[4],2);
											
											dataarray[0] = CON_TX_PARAM_DATA_DIALYSIS_TEMP_CONFIG;
											Cl_dprepretcode = (Cl_Uint8Type)sv_nvmgetdata(NV_DIALYSATE_TEMP, &dataarray[1]);
											Cl_dprepretcode = (Cl_Uint8Type)sv_nvmgetdata(NV_DIALYSIS_TEMP_CONFIG_UPPER, &dataarray[3]);
											Cl_dprepretcode = (Cl_Uint8Type)sv_nvmgetdata(NV_DIALYSIS_TEMP_CONFIG_LOWER, &dataarray[5]);
					break;
					case CON_RX_PARAM_DATA_HEPARIN_RATE_CONFIG://0x15
									Cl_dprepretcode =sv_nvmsetdata(NV_HEPARIN_FLOW_RATE,&cl_PrepData.bytearray[0],2);
									Cl_dprepretcode =sv_nvmsetdata(NV_HEPARIN_RATE_CONFIG_UPPER,&cl_PrepData.bytearray[2],2);
									Cl_dprepretcode =sv_nvmsetdata(NV_HEPARIN_RATE_CONFIG_LOWER,&cl_PrepData.bytearray[4],2);
									
									dataarray[0] = CON_TX_PARAM_DATA_HEPARIN_RATE_CONFIG;
									Cl_dprepretcode = (Cl_Uint8Type)sv_nvmgetdata(NV_HEPARIN_FLOW_RATE, &dataarray[1]);
									Cl_dprepretcode = (Cl_Uint8Type)sv_nvmgetdata(NV_HEPARIN_RATE_CONFIG_UPPER, &dataarray[3]);
									Cl_dprepretcode = (Cl_Uint8Type)sv_nvmgetdata(NV_HEPARIN_RATE_CONFIG_LOWER, &dataarray[5]);
					break;
					case CON_RX_PARAM_DATA_APT_VALUE_CONFIG://0x16
									Cl_dprepretcode =sv_nvmsetdata(NV_APT_VALUE,&cl_PrepData.bytearray[0],2);
									Cl_dprepretcode =sv_nvmsetdata(NV_APT_VALUE_CONFIG_UPPER,&cl_PrepData.bytearray[2],2);
									Cl_dprepretcode =sv_nvmsetdata(NV_APT_VALUE_CONFIG_LOWER,&cl_PrepData.bytearray[4],2);
																
									dataarray[0] = CON_TX_PARAM_DATA_APT_VALUE_CONFIG;
									Cl_dprepretcode = (Cl_Uint8Type)sv_nvmgetdata(NV_APT_VALUE, &dataarray[1]);
									Cl_dprepretcode = (Cl_Uint8Type)sv_nvmgetdata(NV_APT_VALUE_CONFIG_UPPER, &dataarray[3]);
									Cl_dprepretcode = (Cl_Uint8Type)sv_nvmgetdata(NV_APT_VALUE_CONFIG_LOWER, &dataarray[5]);
					break;
					case CON_RX_PARAM_DATA_VPT_VALUE_CONFIG://0x17
									Cl_dprepretcode =sv_nvmsetdata(NV_VPT_VALUE,&cl_PrepData.bytearray[0],2);
									Cl_dprepretcode =sv_nvmsetdata(NV_VPT_VALUE_CONFIG_UPPER,&cl_PrepData.bytearray[2],2);
									Cl_dprepretcode =sv_nvmsetdata(NV_VPT_VALUE_CONFIG_LOWER,&cl_PrepData.bytearray[4],2);
									
									dataarray[0] = CON_TX_PARAM_DATA_VPT_VALUE_CONFIG;
									Cl_dprepretcode = (Cl_Uint8Type)sv_nvmgetdata(NV_VPT_VALUE, &dataarray[1]);
									Cl_dprepretcode = (Cl_Uint8Type)sv_nvmgetdata(NV_VPT_VALUE_CONFIG_UPPER, &dataarray[3]);
									Cl_dprepretcode = (Cl_Uint8Type)sv_nvmgetdata(NV_VPT_VALUE_CONFIG_LOWER, &dataarray[5]);
					break;
					case CON_RX_PARAM_DATA_TMP_VALUE_CONFIG://0x18
									Cl_dprepretcode =sv_nvmsetdata(NV_TMP,&cl_PrepData.bytearray[0],2);
									Cl_dprepretcode =sv_nvmsetdata(NV_TMP_VALUE_CONFIG_UPPER,&cl_PrepData.bytearray[0],2);
									Cl_dprepretcode =sv_nvmsetdata(NV_TMP_VALUE_CONFIG_LOWER,&cl_PrepData.bytearray[4],2);
									
									dataarray[0] = CON_TX_PARAM_DATA_TMP_VALUE_CONFIG;
									Cl_dprepretcode = (Cl_Uint8Type)sv_nvmgetdata(NV_TMP, &dataarray[1]);
									Cl_dprepretcode = (Cl_Uint8Type)sv_nvmgetdata(NV_TMP_VALUE_CONFIG_UPPER, &dataarray[3]);
									Cl_dprepretcode = (Cl_Uint8Type)sv_nvmgetdata(NV_TMP_VALUE_CONFIG_LOWER, &dataarray[5]);
					break;
					case CON_RX_PARAM_DATA_COND_VALUE_CONFIG://0x19
									Cl_dprepretcode =sv_nvmsetdata(NV_CONDUCTIVITY,&cl_PrepData.bytearray[0],2);
									Cl_dprepretcode =sv_nvmsetdata(NV_COND_VALUE_CONFIG_UPPER,&cl_PrepData.bytearray[2],2);
									Cl_dprepretcode =sv_nvmsetdata(NV_COND_VALUE_CONFIG_LOWER,&cl_PrepData.bytearray[4],2);
									
									dataarray[0] = CON_TX_PARAM_DATA_COND_VALUE_CONFIG;
									Cl_dprepretcode = (Cl_Uint8Type)sv_nvmgetdata(NV_CONDUCTIVITY, &dataarray[1]);
									Cl_dprepretcode = (Cl_Uint8Type)sv_nvmgetdata(NV_COND_VALUE_CONFIG_UPPER, &dataarray[3]);
									Cl_dprepretcode = (Cl_Uint8Type)sv_nvmgetdata(NV_COND_VALUE_CONFIG_LOWER, &dataarray[5]);
					break;
					case CON_RX_PARAM_DATA_UF_RATE_CONFIG://0x1A
									Cl_dprepretcode =sv_nvmsetdata(NV_UF_REMOVAL_RATE,&cl_PrepData.bytearray[0],2);
									Cl_dprepretcode =sv_nvmsetdata(NV_UF_RATE_CONFIG_UPPER,&cl_PrepData.bytearray[2],2);
									Cl_dprepretcode =sv_nvmsetdata(NV_UF_RATE_CONFIG_LOWER,&cl_PrepData.bytearray[4],2);
									
									dataarray[0] = CON_TX_PARAM_DATA_UF_RATE_CONFIG;
									Cl_dprepretcode = (Cl_Uint8Type)sv_nvmgetdata(NV_UF_REMOVAL_RATE, &dataarray[1]);
									Cl_dprepretcode = (Cl_Uint8Type)sv_nvmgetdata(NV_UF_RATE_CONFIG_UPPER, &dataarray[3]);
									Cl_dprepretcode = (Cl_Uint8Type)sv_nvmgetdata(NV_UF_RATE_CONFIG_LOWER, &dataarray[5]);
					break;
					case CON_RX_PARAM_DATA_UF_GOAL_CONFIG://0x1B,
									Cl_dprepretcode =sv_nvmsetdata(NV_UF_GOAL,&cl_PrepData.bytearray[0],2);
									Cl_dprepretcode =sv_nvmsetdata(NV_UF_GOAL_CONFIG_UPPER,&cl_PrepData.bytearray[2],2);
									Cl_dprepretcode =sv_nvmsetdata(NV_UF_GOAL_CONFIG_LOWER,&cl_PrepData.bytearray[4],2);
									
									dataarray[0] = CON_TX_PARAM_DATA_UF_GOAL_CONFIG;
									Cl_dprepretcode = (Cl_Uint8Type)sv_nvmgetdata(NV_UF_GOAL, &dataarray[1]);
									Cl_dprepretcode = (Cl_Uint8Type)sv_nvmgetdata(NV_UF_GOAL_CONFIG_UPPER, &dataarray[3]);
									Cl_dprepretcode = (Cl_Uint8Type)sv_nvmgetdata(NV_UF_GOAL_CONFIG_LOWER, &dataarray[5]);
					break;
					case CON_RX_PARAM_DATA_BOLUS_VOLUME_CONFIG://1C
									Cl_dprepretcode =sv_nvmsetdata(NV_HEPARIN_BOLUS,&cl_PrepData.bytearray[0],2);
									Cl_dprepretcode =sv_nvmsetdata(NV_BOLUS_VOLUME_CONFIG_UPPER,&cl_PrepData.bytearray[2],2);
									Cl_dprepretcode =sv_nvmsetdata(NV_BOLUS_VOLUME_CONFIG_LOWER,&cl_PrepData.bytearray[4],2);
									
									dataarray[0] = CON_TX_PARAM_DATA_BOLUS_VOLUME_CONFIG;
									Cl_dprepretcode = (Cl_Uint8Type)sv_nvmgetdata(NV_HEPARIN_BOLUS, &dataarray[1]);
									Cl_dprepretcode = (Cl_Uint8Type)sv_nvmgetdata(NV_BOLUS_VOLUME_CONFIG_UPPER, &dataarray[3]);
									Cl_dprepretcode = (Cl_Uint8Type)sv_nvmgetdata(NV_BOLUS_VOLUME_CONFIG_LOWER, &dataarray[5]);
					break;
					case CON_RX_PARAM_DATA_ABF_RATE_CONFIG://0X1D
									Cl_dprepretcode =sv_nvmsetdata(NV_ARTERIAL_BLOODFLOW_RATE,&cl_PrepData.bytearray[0],2);
									Cl_dprepretcode =sv_nvmsetdata(NV_ABF_RATE_CONFIG_UPPER,&cl_PrepData.bytearray[2],2);
									Cl_dprepretcode =sv_nvmsetdata(NV_ABF_RATE_CONFIG_LOWER,&cl_PrepData.bytearray[4],2);
									
									dataarray[0] = CON_TX_PARAM_DATA_ABF_RATE_CONFIG;
									Cl_dprepretcode = (Cl_Uint8Type)sv_nvmgetdata(NV_ARTERIAL_BLOODFLOW_RATE, &dataarray[1]);
									Cl_dprepretcode = (Cl_Uint8Type)sv_nvmgetdata(NV_ABF_RATE_CONFIG_UPPER, &dataarray[3]);
									Cl_dprepretcode = (Cl_Uint8Type)sv_nvmgetdata(NV_ABF_RATE_CONFIG_LOWER, &dataarray[5]);
					break;
					
					 default:
					 break;
				}
					command = CON_TX_COMMAND_SYSDATA;
					Cl_dprepretcode = Cl_SendDatatoconsole(command,&dataarray,size);
	return CL_OK;
}

conf_activate_pressure_alarms(int dflow){
	
	switch(dflow){
		
		case 800 :
				Cl_AlarmConfigureAlarmType(PS1_HIGH_THRESHOLD,ABOVE,1600,0,0);
				Cl_AlarmConfigureAlarmType(PS2_HIGH_THRESHOLD,ABOVE,1600,0,0);
				Cl_AlarmConfigureAlarmType(PS3_HIGH_THRESHOLD,ABOVE,3000,0,0);
				Cl_AlarmConfigureAlarmType(PS1_LOW_THRESHOLD,BELOW,0,500,0);
				Cl_AlarmConfigureAlarmType(PS2_LOW_THRESHOLD,BELOW,0,500,0);
				Cl_AlarmConfigureAlarmType(PS3_LOW_THRESHOLD,BELOW,0,500,0);
				
					
					Cl_AlarmActivateAlarms(PS1_HIGH_THRESHOLD,true );
					Cl_AlarmActivateAlarms(PS1_LOW_THRESHOLD,true );
					Cl_AlarmActivateAlarms(PS2_HIGH_THRESHOLD,true );
					Cl_AlarmActivateAlarms(PS2_LOW_THRESHOLD,true );
					Cl_AlarmActivateAlarms(PS3_HIGH_THRESHOLD,true );	
					
					break;
		case 700 :
		Cl_AlarmConfigureAlarmType(PS1_HIGH_THRESHOLD,ABOVE,1600,0,0);
		Cl_AlarmConfigureAlarmType(PS2_HIGH_THRESHOLD,ABOVE,1600,0,0);
		Cl_AlarmConfigureAlarmType(PS3_HIGH_THRESHOLD,ABOVE,3000,0,0);
		Cl_AlarmConfigureAlarmType(PS1_LOW_THRESHOLD,BELOW,0,500,0);
		Cl_AlarmConfigureAlarmType(PS2_LOW_THRESHOLD,BELOW,0,500,0);
		Cl_AlarmConfigureAlarmType(PS3_LOW_THRESHOLD,BELOW,0,500,0);
		
		
		Cl_AlarmActivateAlarms(PS1_HIGH_THRESHOLD,true );
		Cl_AlarmActivateAlarms(PS1_LOW_THRESHOLD,true );
		Cl_AlarmActivateAlarms(PS2_HIGH_THRESHOLD,true );
		Cl_AlarmActivateAlarms(PS2_LOW_THRESHOLD,true );
		Cl_AlarmActivateAlarms(PS3_HIGH_THRESHOLD,true );
		
		break;
		
		case 500 :
					Cl_AlarmConfigureAlarmType(PS1_HIGH_THRESHOLD,ABOVE,1500,0,0); // 1600
					Cl_AlarmConfigureAlarmType(PS2_HIGH_THRESHOLD,ABOVE,1450,0,0); //1600
					Cl_AlarmConfigureAlarmType(PS3_HIGH_THRESHOLD,ABOVE,3000,0,0); 
					Cl_AlarmConfigureAlarmType(PS1_LOW_THRESHOLD,BELOW,0,600,0); // 500
					Cl_AlarmConfigureAlarmType(PS2_LOW_THRESHOLD,BELOW,0,500,0); // 500
					Cl_AlarmConfigureAlarmType(PS3_LOW_THRESHOLD,BELOW,0,500,0); // 500
					
					
					Cl_AlarmActivateAlarms(PS1_HIGH_THRESHOLD,true );
					Cl_AlarmActivateAlarms(PS1_LOW_THRESHOLD,true );
					Cl_AlarmActivateAlarms(PS2_HIGH_THRESHOLD,true );
					Cl_AlarmActivateAlarms(PS2_LOW_THRESHOLD,true );
					Cl_AlarmActivateAlarms(PS3_HIGH_THRESHOLD,true );
					break;
		case 300 :
					Cl_AlarmConfigureAlarmType(PS1_HIGH_THRESHOLD,ABOVE,1500,0,0);
					Cl_AlarmConfigureAlarmType(PS2_HIGH_THRESHOLD,ABOVE,1450,0,0);
					Cl_AlarmConfigureAlarmType(PS3_HIGH_THRESHOLD,ABOVE,3000,0,0);
					Cl_AlarmConfigureAlarmType(PS1_LOW_THRESHOLD,BELOW,0,600,0);
					Cl_AlarmConfigureAlarmType(PS2_LOW_THRESHOLD,BELOW,0,500,0);
					Cl_AlarmConfigureAlarmType(PS3_LOW_THRESHOLD,BELOW,0,500,0);
					
					Cl_AlarmActivateAlarms(PS1_HIGH_THRESHOLD,true );
					Cl_AlarmActivateAlarms(PS1_LOW_THRESHOLD,true );
					Cl_AlarmActivateAlarms(PS2_HIGH_THRESHOLD,true );
					Cl_AlarmActivateAlarms(PS2_LOW_THRESHOLD,true );
					Cl_AlarmActivateAlarms(PS3_HIGH_THRESHOLD,true );
					break;
		
		
		
	}
	
}



Cl_ReturnCodeType	Cl_Dprep_StartPreparation(void)
	{
		
			Cl_ReturnCodeType Cl_dprepretcode = CL_OK;
			Cl_Uint8Type data =0;
			Cl_Uint16Type cl_dprep_acidstatus = 0, cl_dprep_bicarbstatus =0 , cl_dprep_holder1status = 0 , cl_dprep_holder2status = 0;
			Cl_BoolType Cl_dprepreaduinessflag = true;
					
				//Check for  dprep readiness and alarm status
				
				//Cl_checkdprepReadyness();
 				Cl_SysStat_GetSensor_Status_Query(SENSOR_ACID_INLET,&cl_dprep_acidstatus);
 				Cl_SysStat_GetSensor_Status_Query(SENSOR_BICARB_INLET,&cl_dprep_bicarbstatus);
 				Cl_SysStat_GetSensor_Status_Query(SENSOR_HOLDER1STATUS,&cl_dprep_holder1status);
 				Cl_SysStat_GetSensor_Status_Query(SENSOR_HOLDER2STATUS,&cl_dprep_holder2status);
 				 if(cl_dprep_acidstatus == ACID_INLET_CLOSED )   // manav03 commented to test
 				 {
 					 Cl_dprepSendAlarm(ACID_IN_CLOSED);
 					 Cl_dprepreaduinessflag = false;
 				 } 
 				 if(cl_dprep_bicarbstatus == BICARB_INLET_CLOSED )
 				 {
 					  Cl_dprepSendAlarm(BICARB_IN_CLOSED);
 					 Cl_dprepreaduinessflag = false;
 				 }	
			//	 #if 0
				 if(cl_dprep_holder1status == CL_HOLDER1OPEN )
				 {
					 Cl_dprepSendAlarm(HOLDER1STATUS_OPEN);
					 Cl_dprepreaduinessflag = false;
				 }
				 if(cl_dprep_holder2status == CL_HOLDER1OPEN )
				 {
					 Cl_dprepSendAlarm(HOLDER2STATUS_OPEN);
					 Cl_dprepreaduinessflag = false;
				 }		
			//	 #endif		 
				 if(Cl_dprepreaduinessflag == false)
				 {
					 return CL_REJECTED;
				 }
				 	
							sv_cntrl_resetgreenalarm();					
							Cl_dprepretcode = Cl_DprepSelectDialysateInlet();
							//Cl_DprepFlowOn();
							Cl_Dprep_ResetAlertsforReassertion();
							data = 1;
							Cl_dprepretcode = Cl_SendDatatoconsole(CON_TX_COMMAND_DLSIS_PREP_CNFRM,&data,0);	
							Cl_dprepretcode = Cl_SendDatatoconsole(CON_TX_COMMAND_DIALYSATE_FILLING_STARTED,&data,0);																			
							Cl_dprepretcode = cl_flow_switchcontroller(FLOW_SWITCH_CNTRLER_EVENT_START);
				
				
							Cl_dprepretcode =  Cl_AlarmConfigureAlarmType(ACID_IN_CLOSED,LOGIC_HIGH,0,0,0);
							Cl_dprepretcode =  Cl_AlarmConfigureAlarmType(BICARB_IN_CLOSED,LOGIC_HIGH,0,0,0);
							//Cl_dprepretcode =  Cl_AlarmActivateAlarms(BLOODDOOR_STATUS_OPEN,true );
							Cl_dprepretcode =  Cl_AlarmConfigureAlarmType(HOLDER2STATUS_OPEN,LOGIC_HIGH,0,0,0);
							Cl_dprepretcode =  Cl_AlarmConfigureAlarmType(HOLDER1STATUS_OPEN,LOGIC_HIGH,0,0,0);
							Cl_dprepretcode =  Cl_AlarmConfigureAlarmType(COND_STATUS_HIGH,ABOVE,140,0,0);
							Cl_dprepretcode =  Cl_AlarmConfigureAlarmType(COND_STATUS_LOW,BELOW,0,0,0);
							Cl_dprepretcode =  Cl_AlarmConfigureAlarmType(TEMP1_HIGH_THRESHOLD,ABOVE,400,0,0);
							Cl_dprepretcode =  Cl_AlarmConfigureAlarmType(TEMP2_HIGH_THRESHOLD,ABOVE,450,0,0);
							Cl_dprepretcode =  Cl_AlarmConfigureAlarmType(TEMP3_HIGH_THRESHOLD,ABOVE,400,0,0);
							Cl_dprepretcode =  Cl_AlarmConfigureAlarmType(TEMP1_LOW_THRESHOLD,BELOW,0,200,0);
							Cl_dprepretcode =  Cl_AlarmConfigureAlarmType(TEMP2_LOW_THRESHOLD,BELOW,0,200,0);
							Cl_dprepretcode =  Cl_AlarmConfigureAlarmType(TEMP3_LOW_THRESHOLD,BELOW,0,0,0);
						
							
							Cl_dprepretcode =  Cl_AlarmConfigureAlarmType(FLOW_LOW_FLOWRATE,0,0,0,0);
							Cl_dprepretcode =  Cl_AlarmConfigureAlarmType(FLOW_NO_FLOW,0,0,0,0);
							Cl_dprepretcode =  Cl_AlarmConfigureAlarmType(FLOW_HIGH_FLOWRATE,0,1500,0,0);
							
						//	Cl_dprepretcode =  Cl_AlarmActivateAlarms(LEVELSWITCH_OFF_TO_ON,true );
						//	Cl_dprepretcode =  Cl_AlarmActivateAlarms(LEVELSWITCH_ON_TO_OFF,true );
							//Cl_dprepretcode =  sv_cntrl_activatevenousclamp(); // manav : ???
							Cl_dprepretcode =  Cl_AlarmActivateAlarms(ACID_IN_CLOSED,true );  // manav03 disabled for testing
							Cl_dprepretcode =  Cl_AlarmActivateAlarms(BICARB_IN_CLOSED,true );
							Cl_dprepretcode =  Cl_AlarmActivateAlarms(HOLDER1STATUS_OPEN,true );    // manav03 alarm disabled for testing
							Cl_dprepretcode =  Cl_AlarmActivateAlarms(HOLDER2STATUS_OPEN,true );    // manav03 alarm disabled for testing
							
							//Cl_dprepretcode =  Cl_AlarmActivateAlarms(FLOW_HIGH_FLOWRATE,true );
							Cl_dprepretcode =  Cl_AlarmActivateAlarms(FLOW_NO_FLOW,true );					
							Cl_dprepretcode =  Cl_AlarmActivateAlarms(FLOW_LOW_FLOWRATE,true );
							Cl_dprepretcode =  Cl_AlarmActivateAlarms(FLOW_HIGH_FLOWRATE,true );
							
						//	Cl_dprepretcode =  Cl_AlarmActivateAlarms(COND_STATUS_HIGH,true );
						//	Cl_dprepretcode =  Cl_AlarmActivateAlarms(COND_STATUS_LOW,true );
							
 							Cl_dprepretcode =  Cl_AlarmActivateAlarms(TEMP1_HIGH_THRESHOLD,true );
							 Cl_dprepretcode =  Cl_AlarmActivateAlarms(TEMP1_LOW_THRESHOLD,true );
 							Cl_dprepretcode =  Cl_AlarmActivateAlarms(TEMP2_HIGH_THRESHOLD,true );
							 Cl_dprepretcode =  Cl_AlarmActivateAlarms(TEMP2_LOW_THRESHOLD,true );
// 							Cl_dprepretcode =  Cl_AlarmActivateAlarms(TEMP3_HIGH_THRESHOLD,true );
 							Cl_dprepretcode =  Cl_AlarmActivateAlarms(BP_STOPPED,false );
							 Cl_AlarmActivateAlarms(BC_ALARM,true);

						
							if(Current_sense_trigger)
							{
									Cl_dprepretcode =  Cl_AlarmActivateAlarms( FPCURRENTSTATUS,true );
							}
							else
							{
								//			Cl_rinseretcode =  Cl_AlarmActivateAlarms( PS3STATUS_HIGH,true );
							}
							
							Set_RC =0;
													
							Cl_dprepretcode =  sv_cntrl_activatepump(DCMOTOR1);
							Cl_dprepretcode =  sv_cntrl_activatepump(DCMOTOR2);
							sv_cntrl_enable_bypass();
							cl_dprepbypassstate = CL_DPREP_BYPASS_ON;
						//	sv_cntrl_disable_bypass();
						//	cl_dprepbypassstate = CL_DPREP_BYPASS_OFF;
							Cl_dprepretcode = Cl_bc_controller(BC_EVENT_RESUME);
							//sv_cntrl_enable_bypass();
							sv_prop_startmixing();
							Cl_Uint32Type temp = Treatdata[ID_dflow];
							Cl_Uint16Type potvalue = 0;
							switch (temp)
							{
								case 800:
								sv_cntrl_setpumpspeed(DCMOTOR2,900);
								sv_cntrl_setpumpspeed(DCMOTOR1,800);   //960
								potvalue = (2600 * 1024)/10000;
								sv_cs_setpotvalue(potvalue);
								
								conf_activate_pressure_alarms(800);
								
							
								
								break;
								
								case 500:
								Cl_SendDatatoconsole(CON_TX_COMMAND_PRINTTEXT,"500",3);
								sv_cntrl_setpumpspeed(DCMOTOR2,1000);
								sv_cntrl_setpumpspeed(DCMOTOR1,1000);   //960
								potvalue = (600 * 1024)/10000;
								sv_cs_setpotvalue(potvalue);
								conf_activate_pressure_alarms(500);
								
								break;

								case 300:
								sv_cntrl_setpumpspeed(DCMOTOR2,1000);
								sv_cntrl_setpumpspeed(DCMOTOR1,600);
								potvalue = (650 * 1024)/10000;
								sv_cs_setpotvalue(potvalue);
								conf_activate_pressure_alarms(300);
										
								break;
								
								
								default:
								sv_cntrl_setpumpspeed(DCMOTOR2,1000);
								sv_cntrl_setpumpspeed(DCMOTOR1,1100);   //960
								potvalue = (600 * 1024)/10000;
								sv_cs_setpotvalue(potvalue);
								
								break;
							}
						//	#if 0
						//    sv_cntrl_poweronheater();
							Cl_dprepretcode = SetHeaterState(CL_HEATER_STATE_ON);
						//	#endif
							cl_dprepstate = CL_DPREP_STATE_DPREP_FILLING;
							
							return 0;
				
	}
	
	
	Cl_ReturnCodeType	Cl_Dprep_StartDialyserPrime(void)
	{
			sv_cntrl_resetgreenalarm();
			dialyzerprime_flag =1;
			Cl_ReturnCodeType Cl_dprepretcode = CL_OK;
			Cl_Uint8Type data =0;
				
			Cl_dprepMinutescounter = 0;
			Cl_dprepsecondscounter = 0;
			Cl_Dprep_ResetAlertsforReassertion();
// 			Cl_dprepretcode = sv_cntrl_poweronheater();               // Gopal commented as duty assigned 2300
// 			Cl_dprepretcode = SetHeaterState(CL_HEATER_STATE_ON);
			//cl_dprepstate = CL_DPREP_STATE_DIALISER_PRIME;
			//Cl_dprepretcode = Cl_SendDatatoconsole(CON_TX_COMMAND_START_DIALISER_PRIME_CNFRM,&data,0);
			
			//check bypass switches
		//	cl_dprep_primecontroller(CL_DPREP_PRIME_BLOODPUMP_START,0);
			Cl_dprepretcode = Cl_DprepSelectDialysateInlet();
			//Cl_dprepretcode =  Cl_AlarmConfigureAlarmType(BLOODDOOR_STATUS_OPEN,LOGIC_HIGH,0,0,0);
			Cl_dprepretcode =  Cl_AlarmConfigureAlarmType(HOLDER1STATUS_CLOSED,LOGIC_LOW,0,0,0);
			Cl_dprepretcode =  Cl_AlarmConfigureAlarmType(HOLDER2STATUS_CLOSED,LOGIC_LOW,0,0,0);
			Cl_dprepretcode =  Cl_AlarmConfigureAlarmType(ACID_IN_CLOSED,LOGIC_HIGH,0,0,0);
			Cl_dprepretcode =  Cl_AlarmConfigureAlarmType(BICARB_IN_CLOSED,LOGIC_HIGH,0,0,0);
			Cl_dprepretcode =  Cl_AlarmConfigureAlarmType(COND_STATUS_LOW,BELOW,(Treatdata[ID_setcond] - 0.2),0,0);
			Cl_dprepretcode =  Cl_AlarmConfigureAlarmType(COND_STATUS_HIGH,BELOW,(Treatdata[ID_setcond] + 0.2),0,0);
			
	//		Cl_dprepretcode =  Cl_AlarmActivateAlarms(BLOODDOOR_STATUS_OPEN,true );
	//		Cl_dprepretcode =  Cl_AlarmActivateAlarms(LEVELSWITCH_OFF_TO_ON,true );
	//		Cl_dprepretcode =  Cl_AlarmActivateAlarms(LEVELSWITCH_ON_TO_OFF,true );
	
// 			Cl_dprepretcode = Cl_AlarmActivateAlarms(PS1_HIGH_THRESHOLD,false );
// 			Cl_dprepretcode = Cl_AlarmActivateAlarms(PS1_LOW_THRESHOLD,false );
// 			Cl_dprepretcode = Cl_AlarmActivateAlarms(PS2_HIGH_THRESHOLD,false );
// 			Cl_dprepretcode = Cl_AlarmActivateAlarms(PS2_LOW_THRESHOLD,false );
// 			Cl_dprepretcode = Cl_AlarmActivateAlarms(PS3_HIGH_THRESHOLD,false );
// 	        Cl_dprepretcode = Cl_AlarmActivateAlarms(PS3_LOW_THRESHOLD,false );
			
			Cl_dprepretcode =  Cl_AlarmActivateAlarms(ACID_IN_OPEN,false );
			Cl_dprepretcode =  Cl_AlarmActivateAlarms(BICARB_IN_OPEN,false );
			Cl_dprepretcode =  Cl_AlarmActivateAlarms(HOLDER1STATUS_OPEN,false );
			Cl_dprepretcode =  Cl_AlarmActivateAlarms(HOLDER2STATUS_OPEN,false );
			
			
			Cl_dprepretcode =  Cl_AlarmActivateAlarms(ACID_IN_CLOSED,true );
			Cl_dprepretcode =  Cl_AlarmActivateAlarms(BICARB_IN_CLOSED,true );
			Cl_dprepretcode =  Cl_AlarmActivateAlarms(HOLDER1STATUS_CLOSED,true );
			Cl_dprepretcode =  Cl_AlarmActivateAlarms(HOLDER2STATUS_CLOSED,true );
			Cl_dprepretcode =  Cl_AlarmActivateAlarms(BP_STOPPED,true );
			Cl_dprepretcode =  Cl_AlarmActivateAlarms(FLOW_NO_FLOW,true );
			Cl_dprepretcode =  Cl_AlarmActivateAlarms(FLOW_LOW_FLOWRATE,true );
			Cl_dprepretcode =  Cl_AlarmActivateAlarms(FLOW_HIGH_FLOWRATE,true );
			Cl_AlarmActivateAlarms(BC_ALARM,true);
// 			Cl_dprepretcode =  Cl_AlarmActivateAlarms(PS1_HIGH_THRESHOLD,true );
// 			Cl_dprepretcode =  Cl_AlarmActivateAlarms(PS1_LOW_THRESHOLD,true );
// 			Cl_dprepretcode =  Cl_AlarmActivateAlarms(PS2_HIGH_THRESHOLD,true );
// 			Cl_dprepretcode =  Cl_AlarmActivateAlarms(PS2_LOW_THRESHOLD,true );
// 			Cl_dprepretcode =  Cl_AlarmActivateAlarms(PS3_HIGH_THRESHOLD,true );
// 			Cl_dprepretcode =  Cl_AlarmActivateAlarms(PS3_LOW_THRESHOLD,true );
				
// 			Cl_dprepretcode =  Cl_AlarmActivateAlarms(TEMP1_HIGH_THRESHOLD,true );
// 			Cl_dprepretcode =  Cl_AlarmActivateAlarms(TEMP1_LOW_THRESHOLD,true );
// 			Cl_dprepretcode =  Cl_AlarmActivateAlarms(TEMP2_HIGH_THRESHOLD,true );
// 			Cl_dprepretcode =  Cl_AlarmActivateAlarms(TEMP2_LOW_THRESHOLD,true );
// 			Cl_dprepretcode =  Cl_AlarmActivateAlarms(TEMP3_HIGH_THRESHOLD,true );
// 			Cl_dprepretcode =  Cl_AlarmActivateAlarms(TEMP3_LOW_THRESHOLD,true );
// 			Cl_dprepretcode =  Cl_AlarmActivateAlarms(COND_STATUS_HIGH,true );
// 			Cl_dprepretcode =  Cl_AlarmActivateAlarms(COND_STATUS_LOW,true );
// 		
			if(Current_sense_trigger)
			{
				Cl_dprepretcode =  Cl_AlarmActivateAlarms( FPCURRENTSTATUS,true );
			}
			else
			{
				//			Cl_rinseretcode =  Cl_AlarmActivateAlarms( PS3STATUS_HIGH,true );
			}
			
			Cl_dprepretcode =  sv_cntrl_activatepump(DCMOTOR1);
			Cl_dprepretcode =  sv_cntrl_activatepump(DCMOTOR2);
			
			// wait for the cond and temperature to be stable then disable bypass and run it for 2 mins
		//	Cl_dprepretcode =  sv_cntrl_disable_bypass();
		//	cl_dprepbypassstate = CL_DPREP_BYPASS_OFF;
			Cl_Uint32Type temp = Treatdata[ID_dflow];
				flow_rate = temp;
			Cl_Uint16Type potvalue = 0;
			switch (temp)
			{
				case 800:
				sv_cntrl_setpumpspeed(DCMOTOR2,1000);
				sv_cntrl_setpumpspeed(DCMOTOR1,1395);
				potvalue = (612 * 1024)/10000;
				sv_cs_setpotvalue(potvalue);
				
				conf_activate_pressure_alarms(800);
				break;
				
				case 700:
				sv_cntrl_setpumpspeed(DCMOTOR2,900);
				sv_cntrl_setpumpspeed(DCMOTOR1,960);
				potvalue = (2600 * 1024)/10000;
				sv_cs_setpotvalue(potvalue);
				
				conf_activate_pressure_alarms(700);
				break;
				
				
				case 500:
				Cl_SendDatatoconsole(CON_TX_COMMAND_PRINTTEXT,"500",3);
				sv_cntrl_setpumpspeed(DCMOTOR2,1000);
				sv_cntrl_setpumpspeed(DCMOTOR1,1000);   //960
				potvalue = (600 * 1024)/10000;
				sv_cs_setpotvalue(potvalue);
				
				conf_activate_pressure_alarms(500);
				break;

				case 300:
				sv_cntrl_setpumpspeed(DCMOTOR2,1000);
				sv_cntrl_setpumpspeed(DCMOTOR1,600);
				potvalue = (650 * 1024)/10000;
				sv_cs_setpotvalue(potvalue);
				
				conf_activate_pressure_alarms(300);
				
				break;
				
				
				default:
				Cl_SendDatatoconsole(CON_TX_COMMAND_PRINTTEXT,"500",3);
				sv_cntrl_setpumpspeed(DCMOTOR2,1000);
				sv_cntrl_setpumpspeed(DCMOTOR1,1100);                           // 420
				potvalue = (600 * 1024)/10000;
				sv_cs_setpotvalue(potvalue);
				break;
			}
			
		//	if(cl_dprep_prime_state != CL_DPREP_PRIME_STATE_DIALYSER_PRIMING)
		//	{
				
				cl_dprep_primecontroller(CL_DPREP_PRIME_PRIME_START_DIALYSER_PRIMING,0);
				 Cl_SendDatatoconsole(CON_TX_COMMAND_DIALYSER_PRIME_STARTED,&data,0); // manav01 added 29-8-17 removed from prime controller
				Cl_DprepAlarmTable[CL_DPREP_ALARM_BLOODDOOR_STATUS_OPEN].IsRaised = false;
		//	}
			
			
			Cl_dprepretcode = sv_cntrl_disable_loopback(); 
			//sv_prop_startmixing();                                 // manav03 removed
		//	Cl_dprepretcode = Cl_bc_controller(BC_EVENT_RESUME);                      // manav01 commented 29-8-17
		//	Cl_dprepretcode = SetHeaterState(CL_HEATER_STATE_DIALYSER_PRIME_ON);      // manav01 commented 29-8-17
			
			UpdateDprepFillingMinuteTick();
	
			// change state to wait for stability to check for temp and cond stability if yes then change state to dializer prime and diable bypass
		//	Cl_Check_Temp_Cond_Stability();
			
			cl_dprepstate = CL_DPREP_STATE_DPRIME_WAIT_STABILITY;
			
		//	cl_dprepstate = CL_DPREP_STATE_DIALISER_PRIME;
		//
	//	Cl_dprepretcode =  sv_cntrl_disable_bypass();
	//	cl_dprepbypassstate = CL_DPREP_BYPASS_OFF;					
			return 0;
	}

Cl_ReturnCodeType Cl_Dprep_Priming_Stop(){
	
	sv_cntrl_setredalarm();      //manav01 added
	//sv_cntrl_nobuzzer();        //manav01 added
	// Cl_mac_apprequesthandler(EVT_CONSOLE_COMMAND_STOP_DIALISER_PRIME);          // manav03 commented
// manav03 added
						Cl_dprep_StopMixing();
						cl_dprep_primecontroller(CL_DPREP_PRIME_PRIME_STOP_DIALYSER_PRIMING,0);
						cl_dprepstate = CL_DPREP_STATE_DIALISER_PRIME_STOPPED;

Cl_deactivate_prep_alarms();
//Cl_dprep_controller(EVENT_DPREP_STOP_DIALYSER_PRIME);

// 		sv_cntrl_enable_bypass();
// 		cl_dprep_primecontroller(CL_DPREP_PRIME_PRIME_STOP,0);
// 		cl_dprepbypassstate = CL_DPREP_BYPASS_ON;
// 		
// 		
// 		Cl_dprep_StopMixing();
// 		cl_dprep_primecontroller(CL_DPREP_PRIME_PRIME_STOP_DIALYSER_PRIMING,0);
// 	
}
	
	
	
Cl_ReturnCodeType Cl_Check_Temp_Cond_Stability(){
	
	
	// manav02 changed
	
//	Cl_Heater_GetHeaterStableStatus(&dprep_heater_stable_status);
	Cl_Uint16Type temp = 0;
	Cl_SysStat_GetSensor_Status_Query(SENSOR_COND_STATUS , &temp);
	
	if(Cl_alarmThresholdTable.cond_low_threshold < temp < Cl_alarmThresholdTable.cond_high_threshold){
		
		Cl_SysStat_GetSensor_Status_Query(SENSOR_TEMP3STATUS , &temp);
		
		if(Cl_alarmThresholdTable.temp3_low_threshold < temp < Cl_alarmThresholdTable.temp3_high_threshold){
		
		return CL_OK;	
			
		}
		
	}
	
// 		if ((dprep_heater_stable_status) &&  (temp < (Treatdata[ID_setcond]+0.2)) && (temp > (Treatdata[ID_setcond]-0.2)))
// 		{
// 	
// 			return CL_OK;	
// 		}
// 		else{
// 			return CL_REJECTED;
// 		}

return CL_OK;	  // manav02 needs to be changed to cl_rejected
}
	
	
	
Cl_ReturnCodeType Cl_dprep_StopMixing(void)
{
	
	sv_cntrl_setredalarm();
				Cl_ReturnCodeType Cl_dprepretcode = CL_OK;
				Cl_Uint8Type data;
				if(cl_dprepstate == CL_DPREP_STATE_DPREP_FILLING)
				{
					
				Cl_dprepMinutescounter = 0;
				Cl_dprepsecondscounter = 0;

				Cl_dprepretcode = sv_cntrl_poweroffheater();
				Cl_dprepretcode = SetHeaterState(CL_HEATER_STATE_OFF);
				Cl_bc_controller(BC_EVENT_STOP);
				//Cl_rinseretcode = sv_cntrl_setflowpath(FLOW_PATH_IDLE_RINSE);
				cl_dprepstate = CL_DPREP_STATE_STOPPED;
				Cl_dprepretcode = Cl_SendDatatoconsole(CON_TX_COMMAND_DIALYSATE_FILLING_STOPPED,&data,0);
				
					
				}
				if(cl_dprepstate == CL_DPREP_STATE_DIALISER_PRIME){
					
					Cl_dprepMinutescounter = 0;
					Cl_dprepsecondscounter = 0;
					
					Cl_dprepretcode = sv_cntrl_poweroffheater();
					Cl_dprepretcode = SetHeaterState(CL_HEATER_STATE_OFF);
					Cl_bc_controller(BC_EVENT_STOP);
					
					
				}

Cl_deactivate_prep_alarms();
				return Cl_dprepretcode;
}

Cl_ReturnCodeType Cl_deactivate_prep_alarms(){
	
	
	Cl_ReturnCodeType Cl_dprepretcode = CL_OK;
		
		
			Cl_dprepretcode =  Cl_AlarmActivateAlarms(ACID_IN_CLOSED,false );
			Cl_dprepretcode =  Cl_AlarmActivateAlarms(BICARB_IN_CLOSED,false );
			Cl_dprepretcode =  Cl_AlarmActivateAlarms(HOLDER1STATUS_CLOSED,false );
			Cl_dprepretcode =  Cl_AlarmActivateAlarms(HOLDER2STATUS_CLOSED,false );
		
		
			Cl_dprepretcode =  Cl_AlarmActivateAlarms(ACID_IN_OPEN,false );
			Cl_dprepretcode =  Cl_AlarmActivateAlarms(BICARB_IN_OPEN,false );
			Cl_dprepretcode =  Cl_AlarmActivateAlarms(HOLDER1STATUS_OPEN,false );
			Cl_dprepretcode =  Cl_AlarmActivateAlarms(HOLDER2STATUS_OPEN,false );
			
			Cl_dprepretcode =  Cl_AlarmActivateAlarms(PS1_HIGH_THRESHOLD,false );
			Cl_dprepretcode =  Cl_AlarmActivateAlarms(PS1_LOW_THRESHOLD,false );
			Cl_dprepretcode =  Cl_AlarmActivateAlarms(PS2_HIGH_THRESHOLD,false );
			Cl_dprepretcode =  Cl_AlarmActivateAlarms(PS2_LOW_THRESHOLD,false );
			Cl_dprepretcode =  Cl_AlarmActivateAlarms(PS3_HIGH_THRESHOLD,false );
			Cl_dprepretcode =  Cl_AlarmActivateAlarms(PS3_LOW_THRESHOLD,false );
			
			Cl_dprepretcode =  Cl_AlarmActivateAlarms(TEMP1_HIGH_THRESHOLD,false );
			Cl_dprepretcode =  Cl_AlarmActivateAlarms(TEMP1_LOW_THRESHOLD,false );
			Cl_dprepretcode =  Cl_AlarmActivateAlarms(TEMP2_HIGH_THRESHOLD,false );
			Cl_dprepretcode =  Cl_AlarmActivateAlarms(TEMP2_LOW_THRESHOLD,false );
			Cl_dprepretcode =  Cl_AlarmActivateAlarms(TEMP3_HIGH_THRESHOLD,false );
			Cl_dprepretcode =  Cl_AlarmActivateAlarms(TEMP3_LOW_THRESHOLD,false );
			Cl_dprepretcode =  Cl_AlarmActivateAlarms(COND_STATUS_HIGH,false );
			Cl_dprepretcode =  Cl_AlarmActivateAlarms(COND_STATUS_LOW,false );
			Cl_AlarmActivateAlarms(BC_ALARM,false);
			
			
			Cl_dprepretcode =  Cl_AlarmActivateAlarms(FLOW_NO_FLOW,false );
			Cl_dprepretcode =  Cl_AlarmActivateAlarms(FLOW_LOW_FLOWRATE,false );
			Cl_dprepretcode =  Cl_AlarmActivateAlarms(FLOW_HIGH_FLOWRATE,false );
			Cl_dprepretcode =  Cl_AlarmActivateAlarms(BP_STOPPED,false );
	
	
	
}

Cl_ReturnCodeType Cl_activate_dialyzer_prep_alarms(){
	
	
	Cl_ReturnCodeType Cl_dprepretcode = CL_OK;
	
	
	Cl_dprepretcode =  Cl_AlarmActivateAlarms(ACID_IN_CLOSED,true );
	Cl_dprepretcode =  Cl_AlarmActivateAlarms(BICARB_IN_CLOSED,true );
	Cl_dprepretcode =  Cl_AlarmActivateAlarms(HOLDER1STATUS_CLOSED,true );
	Cl_dprepretcode =  Cl_AlarmActivateAlarms(HOLDER2STATUS_CLOSED,true );
	
	
	Cl_dprepretcode =  Cl_AlarmActivateAlarms(ACID_IN_OPEN,false );
	Cl_dprepretcode =  Cl_AlarmActivateAlarms(BICARB_IN_OPEN,false );
	Cl_dprepretcode =  Cl_AlarmActivateAlarms(HOLDER1STATUS_OPEN,false );
	Cl_dprepretcode =  Cl_AlarmActivateAlarms(HOLDER2STATUS_OPEN,false );
	
 	Cl_dprepretcode =  Cl_AlarmActivateAlarms(PS1_HIGH_THRESHOLD,false );
 	Cl_dprepretcode =  Cl_AlarmActivateAlarms(PS1_LOW_THRESHOLD,false );
 	Cl_dprepretcode =  Cl_AlarmActivateAlarms(PS2_HIGH_THRESHOLD,false );
 	Cl_dprepretcode =  Cl_AlarmActivateAlarms(PS2_LOW_THRESHOLD,false );
 	Cl_dprepretcode =  Cl_AlarmActivateAlarms(PS3_HIGH_THRESHOLD,false );
 	Cl_dprepretcode =  Cl_AlarmActivateAlarms(PS3_LOW_THRESHOLD,false );
	
	Cl_Uint32Type temp = Treatdata[ID_dflow];
	conf_activate_pressure_alarms((int)temp);
	
	Cl_dprepretcode =  Cl_AlarmActivateAlarms(TEMP1_HIGH_THRESHOLD,true );
	Cl_dprepretcode =  Cl_AlarmActivateAlarms(TEMP1_LOW_THRESHOLD,true );
	Cl_dprepretcode =  Cl_AlarmActivateAlarms(TEMP2_HIGH_THRESHOLD,true );
	Cl_dprepretcode =  Cl_AlarmActivateAlarms(TEMP2_LOW_THRESHOLD,true );
	Cl_dprepretcode =  Cl_AlarmActivateAlarms(TEMP3_HIGH_THRESHOLD,true );
	Cl_dprepretcode =  Cl_AlarmActivateAlarms(TEMP3_LOW_THRESHOLD,true );
	Cl_dprepretcode =  Cl_AlarmActivateAlarms(COND_STATUS_HIGH,true );
	Cl_dprepretcode =  Cl_AlarmActivateAlarms(COND_STATUS_LOW,true );
	
	
	Cl_dprepretcode =  Cl_AlarmActivateAlarms(FLOW_HIGH_FLOWRATE,true );
	Cl_dprepretcode =  Cl_AlarmActivateAlarms(FLOW_NO_FLOW,true );
	Cl_dprepretcode =  Cl_AlarmActivateAlarms(FLOW_LOW_FLOWRATE,true );
	Cl_dprepretcode =  Cl_AlarmActivateAlarms(FLOW_HIGH_FLOWRATE,true );
	Cl_dprepretcode =  Cl_AlarmActivateAlarms(BP_STOPPED,true );
	Cl_AlarmActivateAlarms(BC_ALARM,true);
	
	
	
}
	
