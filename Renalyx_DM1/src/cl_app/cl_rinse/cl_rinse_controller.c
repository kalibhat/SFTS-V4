#include "cl_rinse_controller_include.h"
#include "cl_rinse_controller_proto.h"
#include "sv_interface_fun.h"

// extern Cl_Uint8Type sv_cntrl_enable_bypass(void);
// extern Cl_Uint8Type sv_cntrl_disable_bypass(void);
// extern Cl_Uint8Type sv_cntrl_disable_loopback(void);
extern Cl_Uint32Type Treatdata[ID_MAX_TREAT_PARAM];
Cl_Uint16Type tempcount;
volatile Cl_Uint8Type blood_flag=0;
volatile Cl_Uint8Type var1=0;
extern Cl_Uint8Type flag_hotrinse;
typedef union {
	int16_t TMP;
	int16_t twobyte;
	Cl_Uint8Type byte[2] ;
}Datatype;

typedef union {
	Cl_Uint32Type fourbytedata;
	int16_t twobytedata;
	Cl_Uint8Type bytedata[4] ;
}StateDatatype;
extern Cl_Uint8Type Cl_ConsoleTxMsg[40];
extern Sys_statusType cl_sys_statbuffer;// test
extern Cl_BoolType g_testbcfreeze; //test
extern Cl_BoolType BC_window; //test
extern Cl_BoolType Current_sense_trigger; // test
extern Cl_BoolType gtest_short_rinse ; //test
extern Cl_Dprep_PrimeStates cl_dprep_prime_state;
extern Cl_AlarmThresholdType  Cl_alarmThresholdTable;
extern Cl_Rinse_States Rinsestatedummy;

extern Cl_Uint32Type flow_rate;
extern Cl_ReturnCodeType  Cl_get_Temperature_Value(sensor_id id, float tempdata, float* correctedtemp);
//extern Cl_Uint8Type DD_RESET_VALVE(sv_valvetype );
//extern Cl_Uint8Type DD_SET_VALVE(sv_valvetype );

extern Cl_ReturnCodeType cl_wait(Cl_Uint32Type );
extern Cl_ReturnCodeType Cl_mac_apprequesthandler(Cl_Mac_EventsType);
// extern Cl_ReturnCodeType Cl_AlarmActivateAlarms(Cl_NewAlarmIdType,Cl_BoolType );
// extern Cl_ReturnCodeType Cl_AlarmResetAlarm(Cl_NewAlarmIdType  );
extern Cl_ConsoleMsgType Cl_ConsoleRxMsg;
extern Cl_ReturnCodeType  sv_nvmgetdata(Cl_Uint8Type,Cl_Uint8Type*);
//extern Cl_ReturnCodeType Cl_Alarm_GetLastAlarm(Cl_NewAlarmIdType*);
//extern Cl_ReturnCodeType Cl_Rinse_UpdateAlarmTable(ClRinseAlarmIdType*);
//extern Cl_ReturnCodeType Cl_Alarm_GetAlarmStatus(Cl_NewAlarmIdType  , Cl_BoolType* );
extern Cl_ReturnCodeType  sv_nvmsetdata(Cl_Uint8Type ,Cl_Uint8Type* ,Cl_Uint8Type datasize);
//extern Cl_Uint8Type sv_cntrl_setflowpath(sv_flowpathtype sv_flowpath);
//extern Cl_ReturnCodeType 		Cl_SysStat_GetSensor_Status_Query(Cl_NewAlarmIdType, uint16_t*);
// extern Cl_Uint8Type sv_cntrl_deactivate_valve(sv_valvetype );
// extern Cl_Uint8Type sv_cntrl_activate_valve(sv_valvetype );
// extern Cl_Uint8Type  sv_cntrl_deactivatepump(sv_pumptype);
// extern Cl_Uint8Type  sv_cntrl_activatepump(sv_pumptype);
// extern Cl_Uint8Type sv_cntrl_poweronheater(void);
// extern Cl_Uint8Type sv_cntrl_poweroffheater(void);
// extern Cl_Uint8Type sv_cntrl_incheater(int32_t dty_val);
//extern Cl_ReturnCodeType cl_uf_controller(cl_uf_events uf_event , int32_t data);
//extern Cl_ReturnCodeType Cl_AlarmConfigureAlarmType(Cl_NewAlarmIdType,Cl_AlarmTriggerType,uint16_t,uint16_t,Cl_Uint8Type);
//extern Cl_ReturnCodeType UpdateHeaterControls(void);
//extern Cl_ReturnCodeType SetHeaterState(HeaterStateType Param_HeaterState);
//extern Cl_ReturnCodeType  Cl_bc_controller(Cl_BC_EventType cl_bc_event);
//extern Cl_ReturnCodeType cl_flow_switchcontroller(flowswitchcntrl_event_type cntrl_event);
//extern Cl_ReturnCodeType cl_dprep_primecontroller(Cl_Dprep_PrimeEvents prime_event , int16_t data);
//extern Cl_ReturnCodeType cl_bp_controller(cl_bp_events bp_event , int16_t data);
//extern Cl_Uint8Type sv_cntrl_setpumpspeed(sv_pumptype sv_pump_id,Cl_Uint32Type speed);
//extern Cl_ReturnCodeType cl_deaeration_controller(cl_deaeration_event_type cntrl_event);
//extern Cl_Uint8Type sv_cs_setpotvalue(Cl_Uint32Type resistance) ;
//extern Cl_ReturnCodeType Cl_Dlsis_SenddlsisData(void);
// extern Cl_Uint8Type sv_cntrl_activatevenousclamp(void); 
// extern Cl_Uint8Type sv_cntrl_decheater(int32_t dty_val);
// Cl_ReturnCodeType cl_rinse_processtimeoutevets(Cl_Rinse_Eevents cl_rinseevent);
// Cl_ReturnCodeType  cl_rinse_translatemacevent(Cl_Mac_EventsType ,Cl_Rinse_Eevents* );
// Cl_ReturnCodeType Cl_rinse_init(void);
// Cl_ReturnCodeType Cl_Rinse_UpdateTimeInfo(void);
// Cl_ReturnCodeType Cl_Rinse_CheckforRinseCompletion(void);
// Cl_ReturnCodeType cl_rinse_notifydacandgotorinsestandby(void);
// Cl_ReturnCodeType Cl_Rinse_Controller(Cl_Mac_EventsType);
// Cl_ReturnCodeType  Cl_Rinse_ProcessAlarms(void);
// Cl_ReturnCodeType Cl_RinseConvertalarmidtouint8data(Cl_NewAlarmIdType cl_alarmId,Cl_Uint8Type* data );
// Cl_ReturnCodeType  CL_RinseAlarmActon(Cl_NewAlarmIdType cl_rinsealarmid);
// Cl_ReturnCodeType  Cl_RinseFlowOn(void );
// Cl_ReturnCodeType  Cl_RinseFlowOff(void );
// Cl_ReturnCodeType  Cl_RinseStop(void);
// Cl_ReturnCodeType Cl_Rinse_ResetAlertsforReassertion(void );
// void Cl_Rinse_100msTMP(void);
// Cl_ReturnCodeType Cl_Rinse_UpdateSecTick(void);
// Cl_ReturnCodeType Cl_Rinse_StartRinse(void);
// Cl_ReturnCodeType Cl_Rinse_StopRinse(void);
// Cl_ReturnCodeType UpdateRinseMinuteTick(void);
// Cl_ReturnCodeType Cl_Rinse_SendRinseStateData(void);
// Cl_ReturnCodeType UpdateHeaterControls(void);
// Cl_ReturnCodeType SetHeaterState(HeaterStateType Param_HeaterState);
// Cl_ReturnCodeType cl_heater_heat(void);
// extern void sv_prop_startpropeo_aligning(void);
// extern void sv_prop_startmixing(void);
// extern void sv_prop_stopmixing(void);
// extern void sv_prop_startopenfill(void);
// extern void sv_prop_stopopenfill(void);
// extern Cl_Uint8Type sv_cntrl_setredalarm(void);
// extern Cl_Uint8Type sv_cntrl_setyellowalarm(void);
// extern Cl_Uint8Type sv_cntrl_buzzer(void);
// extern Cl_Uint8Type sv_cntrl_resetredalarm(void);
// extern Cl_Uint8Type sv_cntrl_resetyellowalarm(void);
// extern Cl_Uint8Type sv_cntrl_setgreenalarm(void);
// extern Cl_Uint8Type sv_cntrl_resetgreenalarm(void);
// extern Cl_Uint8Type sv_cntrl_nobuzzer(void);
//extern  Cl_ReturnCodeType cl_dprep_activate_prime_related_alarms(void);
//extern Cl_ReturnCodeType  Cl_bc_controller(Cl_BC_EventType cl_bc_event);
//extern Cl_ReturnCodeType Cl_Alarm_GetAlarmStatus(Cl_NewAlarmIdType  , Cl_BoolType* );
//static void mdelay(Cl_Uint32Type );
volatile Cl_Uint8Type rinse1=0,rinse2=0,rinse3=0,rinse4=0;
//extern volatile Cl_Uint32Type g_ul_ms_ticks ;
Cl_Rinse_States cl_rinsestate = CL_RINSE_STATE_IDLE;
Cl_Rinsing_SubStates cl_rinsing_substate = CL_RINSE_RINSING_IDLE;
//HeaterStateType HeaterState = CL_HEATER_STATE_OFF;
//Cl_Rinse_Eevents cl_rinseevent;
//extern SV_SetflowPath();
extern volatile float temprature_final_value_1,temprature_final_value_2,temprature_final_value_3,cond_final_cs3;
 int Cl_Rinsesecondscounter =0 ,Cl_RinseMinutescounter=0, Cl_Rinseshourscounter=0,Cl_RinseOpenFillsecondscounter=0;
 int Cl_RinseTotalMinutescounter=0, Cl_RinsesTotalhourscounter=0;
 Cl_BoolType Cl_RinseOpenFillTimeOut = false;
 Cl_BoolType cl_gfillinprogress = false;
 Cl_Uint16Type wait_cnt = 0,TmpVal=0;
 	float temp1,temp2;
	 static Cl_Uint16Type cl_50ms_timer = 0;
Cl_NewAlarmIdType prevrinsealarm_id=_NO_ALARM;
Cl_NewAlarmIdType rinsealarm;
Cl_NewAlarmIdType rinsealarm1;
volatile Cl_Uint8Type hotrinse_flag=0,minute=0;
extern float avgtemp2_dummy;
extern float res_temp_value;
cl_Datastreamtype cl_local ;
int prime_alarm_flag = 0;
extern float cl_utilities_CalConductivity(uint16_t ,float);
int cl_rinse_powerFailureFlag = 0;
Cl_Uint16Type powerFailSensorData = 0;
int disinfectionrinseFlag = 0;
RinseAlarmsType Cl_RinseAlarmTable[CL_RINSE_ALRM_MAX] = 
{	
	{BLOODDOOR_STATUS_OPEN,CL_ALARM_ALARM,false,false,false,false}, 
	//{BYPASSDOOR_STATUS_OPEN,CL_ALARM_ALARM,false,false,false}, 
	{ACID_IN_OPEN,CL_ALARM_ALARM,false,false,false,true},
	{BICARB_IN_OPEN,CL_ALARM_ALARM,false,false,false,true},			
	{LEVELSWITCH_OFF_TO_ON,CL_ALARM_ALARM,false,false,false,true}, 
	{LEVELSWITCH_ON_TO_OFF,CL_ALARM_ALARM,false,false,false,true},
	{HOLDER1STATUS_OPEN,CL_ALARM_ALARM,false,false,false,true},
	{HOLDER2STATUS_OPEN,CL_ALARM_ALARM,false,false,false,true},
	{TEMP1_LOW_THRESHOLD,CL_ALARM_ALARM,false,false,false},
	{TEMP1_HIGH_THRESHOLD,CL_ALARM_ALARM,false,false,false},
	{TEMP2_LOW_THRESHOLD,CL_ALARM_ALARM,false,false,false},
	{TEMP2_HIGH_THRESHOLD,CL_ALARM_ALARM,false,false,false},
	{TEMP3_LOW_THRESHOLD,CL_ALARM_ALARM,false,false,false},
	{TEMP3_HIGH_THRESHOLD,CL_ALARM_ALARM,false,false,false},
	{PS1_HIGH_THRESHOLD,CL_ALARM_ALARM,false,false,false},
	{PS1_LOW_THRESHOLD,CL_ALARM_ALARM,false,false,false},
	{PS2_HIGH_THRESHOLD,CL_ALARM_ALARM,false,false,false},
	{PS2_LOW_THRESHOLD,CL_ALARM_ALARM,false,false,false},
	{PS3_HIGH_THRESHOLD,CL_ALARM_ALARM,false,false,false},
	{PS3_LOW_THRESHOLD,CL_ALARM_ALARM,false,false,false},
	//{COND_STATUS_LOW,CL_ALARM_ALARM,false,false,false},
	{COND_STATUS_HIGH,CL_ALARM_ALARM,false,false,false},
			{POWER_FAILURE,CL_ALARM_ALARM,false,false,false}  ,
	//{COND_DAC_OPEN,CL_ALARM_ALARM,false,false,false},
	//{COND_DAC_RO,CL_ALARM_ALERT,false,false,false},
	//{COND_DAC_HIGH,CL_ALARM_ALERT,false,false,false},
	{FLOW_NO_FLOW,CL_ALARM_ALARM,false,false,false},
	{FLOW_LOW_FLOWRATE,CL_ALARM_ALARM,false,false,false},
	{FLOW_HIGH_FLOWRATE,CL_ALARM_ALARM,false,false,false},
	{BD_EVENT,CL_ALARM_ALARM,false,false,false},
	{BC_ALARM,CL_ALARM_ALARM,false,false,false},
		{BP_STOPPED,CL_ALARM_ALARM,false,false,false},
	//{LEVEL_SWITCH_LOW_TOGGLERATE,CL_ALARM_ALARM,false,false,false},
//	{FPCURRENTSTATUS,CL_ALARM_ALERT,false,false,false},                              // added 17.03.17
	{CONSOLE_SYNC_LOST,CL_ALARM_ALERT,false,false,false}                        // added on 18.03.17
			
};

Cl_ReturnCodeType Cl_Rinse_Controller(Cl_Mac_EventsType Cl_MacRinseEvent   )
{
	
	Cl_ReturnCodeType  Cl_rinseretcode = CL_OK;
	Cl_Rinse_Eevents cl_rinseevent = EVENT_NULL;
	Cl_NewAlarmIdType cl_rinsealarmid;
	ClRinseAlarmIdType ClRinseAlarmId;

	
	Cl_ConsoleTxCommandtype command = CON_TX_COMMAND_COMMAND_MAX;
	Cl_Uint8Type dataarray[4] = {0,0,0,0};
	Cl_Uint8Type data = 0;
	Cl_BoolType alarmstatus1 = false,alarmstatus2 = false,alarmstatus3 = false, flowstatus = false;

StateDatatype abfdata;
Cl_Uint8Type abfdataarray[5] =  {0,0,0,0,0};


cl_rinse_translatemacevent( Cl_MacRinseEvent, &cl_rinseevent);

switch(cl_rinseevent)
{
	
	case EVT_RINSE_COMMAND_SET_BLDPMP_ON:
	Cl_rinseretcode =  Cl_AlarmActivateAlarms(BLOODDOOR_STATUS_OPEN,true);
	//Cl_RinseAlarmTable[CL_RINSE_ALARM_BLOODDOOR_STATUS_OPEN].IsRaised = false;
	cl_bp_controller(CL_BP_EVENT_START,0);
	break;
	case EVT_RINSE_COMMAND_SET_BLDPMP_OFF:
	Cl_rinseretcode =  Cl_AlarmActivateAlarms(BLOODDOOR_STATUS_OPEN,false);
	cl_bp_controller(CL_BP_EVENT_STOP,0);
	// manav02 added to stop saline priming if blood pump is stopped
	if(blood_flag == 1){
		cl_dprep_primecontroller(CL_DPREP_PRIME_PRIME_STOP,0); 
		Cl_Alarm_TriggerAlarm(BP_STOPPED,1);
	}
		
	
//	Cl_RinseAlarmTable[CL_RINSE_ALARM_BLOODDOOR_STATUS_OPEN].IsActive = false;
	break;
	
	case EVT_RINSE_MUTE :
	sv_cntrl_nobuzzer();
	break;
	
	case EVT_RINSE_COMMAND_SET_BLDPUMPRATE:
	if(Cl_ConsoleRxMsg.msgready == true)
	{
		
		cl_Datastreamtype cl_temp  ;
		cl_temp.bytearray[0] = Cl_ConsoleRxMsg.data.byte[0];
		cl_temp.bytearray[1] = Cl_ConsoleRxMsg.data.byte[1];
		cl_temp.bytearray[2] = Cl_ConsoleRxMsg.data.byte[2];
		cl_temp.bytearray[3] = Cl_ConsoleRxMsg.data.byte[3];
		cl_bp_controller(CL_BP_EVENT_SET_BP_RATE,cl_temp.Twobyte);
		Treatdata[ID_bloodrate] = cl_temp.Twobyte;
	}
	break;
	
	case EVENT_RINSE_HEP_PMP_START:
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
	case EVENT_RINSE_HEP_PMP_STOP:
	cl_hep_controller(CL_HEP_EVENT_STOP_NORMAL_DELIVERY,0);
	break;
	case EVENT_RINSE_HEP_PMP_BLOUS:
	cl_hep_controller(CL_HEP_EVENT_STARTBOLUS,0);
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
		
		cl_rinseevent == EVENT_NULL;
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
		
		cl_rinseevent == EVENT_NULL;
		break;
		
	
	
	default:break;	
}

if(cl_rinseevent == EVENT_RINSE_TICK_SECOND)
{
	Rinsestatedummy = cl_rinsestate;
	//Cl_rinseretcode =  Cl_AlarmResetAlarm( SENSOR_TEMP3STATUS );
	Cl_rinseretcode =  Cl_AlarmResetAlarm( LEVELSWITCH_OFF_TO_ON );
	Cl_rinseretcode =  Cl_AlarmResetAlarm( LEVELSWITCH_ON_TO_OFF );


}


switch(cl_rinsestate)
{

		case CL_RINSE_STATE_INIT:
		break;
		case CL_RINSE_STATE_IDLE:
		switch (cl_rinseevent)
			{
				case EVT_RINSE_GET_DATA:
						if(Cl_ConsoleRxMsg.msgready == true)
						{
							
							switch(Cl_ConsoleRxMsg.data.byte[0])
							{
								case CON_RX_PARAM_DATA_RINSE_STATUS:
								command = CON_TX_COMMAND_SYSDATA;
								dataarray[0] = CON_TX_PARAM_DATA_RINSE_STATUS;
								Cl_rinseretcode = (Cl_Uint8Type)sv_nvmgetdata(NV_NVM_RINSE_STATUS, &dataarray[1]);
								Cl_rinseretcode = Cl_SendDatatoconsole(command,&dataarray,2);
								
								break;
								case CON_RX_PARAM_DATA_PRIME_STATUS:
								command = CON_TX_COMMAND_SYSDATA;
								dataarray[0] = CON_TX_PARAM_DATA_PRIME_STATUS;
								Cl_rinseretcode = (Cl_Uint8Type)sv_nvmgetdata(NV_NVM_PRIME_STATUS, &dataarray[1]);
								Cl_rinseretcode = Cl_SendDatatoconsole(command,&dataarray,2);	
								break;
								case CON_RX_PARAM_DATA_DISINF_STATUS:
								command = CON_TX_COMMAND_SYSDATA;
								dataarray[0] = CON_TX_PARAM_DATA_DISINF_STATUS;
								Cl_rinseretcode = (Cl_Uint8Type)sv_nvmgetdata(NV_NVM_DISINF_STATUS, &dataarray[1]);
								Cl_rinseretcode = Cl_SendDatatoconsole(command,&dataarray,2);	
								break;							
								default:
								break;
							}
						}
				break;
				case EVENT_START_RINSE:
					if(Cl_ConsoleRxMsg.msgready == true)
					{
//						cl_Datastreamtype cl_temp  ;

						cl_local.bytearray[0] = Cl_ConsoleRxMsg.data.byte[0];
						if (cl_local.bytearray[0] == 1)
						{
						
						}
						else if (cl_local.bytearray[0] == 2)
						{
							hotrinse_flag = 1;
						}
					
					}
					if( CL_OK == Cl_Rinse_StartRinse())
					{
											
						command = CON_TX_COMMAND_RINSE_CONFIRM;
						data = (Cl_Uint8Type)COMMAND_RESULT_SUCCESS;
						Cl_rinseretcode = Cl_SendDatatoconsole(command,&data,1);					
						command = CON_TX_COMMAND_RINSE_STARTED;
						Cl_rinseretcode = Cl_SendDatatoconsole(command,&data,0);
						Cl_rinseretcode = Cl_Rinse_UpdateTimeInfo();
						cl_rinsestate = CL_RINSE_STATE_RINSING;
						cl_rinsing_substate = CL_RINSE_RINSING_OPENFILL;
					}
					else
					{
						return CL_REJECTED;
					}
				break;
				case EVENT_START_PRE_DISINF_RINSE:
					disinfectionrinseFlag = 1;
					Cl_Rinse_StartRinse();
					
					cl_rinsestate = CL_RINSE_STATE_PRE_DISINFECT_RINSING;
					cl_rinsing_substate = CL_RINSE_RINSING_OPENFILL;
				break;
				case EVENT_START_POST_DISINF_RINSE:
						disinfectionrinseFlag = 1;
						Cl_Rinse_StartRinse();
						cl_rinsestate = CL_RINSE_STATE_POST_DISINFECT_RINSING;
						cl_rinsing_substate = CL_RINSE_RINSING_OPENFILL;
				break;
				case EVENT_RINSE_ALARM:
				// check for rinse alarms and update rinse alarm table
					Cl_rinseretcode = Cl_Rinse_ProcessAlarms();
				break;
				
					case EVENT_RINSE_CLR_ALARM:
						
						command = CON_TX_COMMAND_ALARM_CLEARED;
						data = (Cl_Uint8Type)COMMAND_RESULT_SUCCESS;
						Cl_rinseretcode = Cl_SendDatatoconsole(command,&data,0);
						
										sv_cntrl_resetyellowalarm();
										sv_cntrl_resetredalarm();
										sv_cntrl_nobuzzer();
										Cl_Rinse_ResetAlertsforReassertion();
										
						break;
				
				default:
				break;
			}                                 	
		break;
		
		case CL_RINSE_STATE_RINSING:
		case CL_RINSE_STATE_PRE_DISINFECT_RINSING:
		case CL_RINSE_STATE_POST_DISINFECT_RINSING:
				switch (cl_rinseevent)
				{
// 						case EVT_RINSE_SET_DATA:
// 							if(Cl_ConsoleRxMsg.msgready == true)
// 							{
// 										cl_Datastreamtype cl_temp  ;	
// 										cl_temp.bytearray[0] = Cl_ConsoleRxMsg.data.byte[1];
// 										cl_temp.bytearray[1] = Cl_ConsoleRxMsg.data.byte[2];
// 										cl_temp.bytearray[2] = Cl_ConsoleRxMsg.data.byte[3];
// 										cl_temp.bytearray[3] = Cl_ConsoleRxMsg.data.byte[4];
// 											
// 								switch(Cl_ConsoleRxMsg.data.byte[0])
// 								{
// 										case ID_dflow:
// 										break;
// 										case ID_settemp:
// 										break;
// 										case ID_heprate:
// 										break;
// 										case ID_setcond:
// 										break;
// 										case ID_ufrate:
// 										break;
// 										case ID_ufgoal:
// 										break;
// 										case ID_bolusvol:
// 										break;
// 										case ID_bloodrate:
// 										break;
// 										case ID_hepstoptime:
// 										break;
// 										case ID_syringetype:
// 										break;
// 										case ID_heparincheck:
// 										break;
// 										case ID_minufrate:
// 										break;
// 										case ID_treattime:
// 										break;
// 										case ID_bloodratereturn:
// 										break;
// 										case ID_bloodratetreat:
// 										break;
// 										case ID_tempulimit:
// 											Cl_alarmThresholdTable.temp3_high_threshold =  (cl_temp.word)/10;
// 										break;
// 										case ID_templlimit:
// 											Cl_alarmThresholdTable.temp3_low_threshold =  (cl_temp.word)/10;
// 										break;
// 										case ID_tmpllimit:
// 										Cl_alarmThresholdTable.tmp_low_threshold =  cl_temp.word;
// 										break;
// 										case ID_tmpulimit:
// 										Cl_alarmThresholdTable.tmp_high_threshold =  cl_temp.word;
// 										break;
// 										case ID_vptllimit:
// 											Cl_alarmThresholdTable.vpt_low_threshold =  cl_temp.word;
// 										break;
// 										case ID_vptulimit:
// 											Cl_alarmThresholdTable.vpt_high_threshold =  cl_temp.word;
// 										break;
// 										case ID_ufllimit:
// 										break;
// 										case ID_ufulimit:
// 										break;
// 										case ID_dflowllimit:
// 										break;
// 										case ID_dflowulimit:
// 										break;
// 										case ID_condllimit:
// 										Cl_alarmThresholdTable.cond_low_threshold =  cl_temp.word;
// 										break;
// 										case ID_condulimit:
// 										Cl_alarmThresholdTable.cond_high_threshold =  cl_temp.word;
// 										break;
// 										case ID_aptllimit:
// 											Cl_alarmThresholdTable.apt_low_threshold =  cl_temp.word;
// 										break;
// 										case ID_aptulimit:
// 										Cl_alarmThresholdTable.apt_high_threshold =  cl_temp.word;
// 										break;
// 
// 									default:break;
// 								}
// 							}
// 						
// 						break;
// 						case EVT_RINSE_GET_DATA:
// 								if(Cl_ConsoleRxMsg.msgready == true)
// 								{
// 							
// 									switch(Cl_ConsoleRxMsg.data.byte[0])
// 									{
// 
// 										case CON_RX_PARAM_DATA_PRIME_STATUS:
// 										command = CON_TX_COMMAND_SYSDATA;
// 										dataarray[0] = CON_TX_PARAM_DATA_PRIME_STATUS;
// 										Cl_rinseretcode = (Cl_Uint8Type)sv_nvmgetdata(NV_NVM_PRIME_STATUS, &dataarray[1]);
// 										Cl_rinseretcode = Cl_SendDatatoconsole(command,&dataarray,2);
// 										break;
// 
// 										case CON_RX_PARAM_DATA_RECIRC_STATUS:
// 										command = CON_TX_COMMAND_SYSDATA;
// 										dataarray[0] = CON_TX_PARAM_DATA_RCIRC;
// 										//Cl_rinseretcode = (Cl_Uint8Type)sv_nvmgetdata(NV_NVM_PRIME_STATUS, &dataarray[1]);
// 										dataarray[1] = 0;
// 										Cl_rinseretcode = Cl_SendDatatoconsole(command,&dataarray,2);
// 										break;		
// 										case CON_RX_PARAM_DATA_DISINF_STATUS:
// 										command = CON_TX_COMMAND_SYSDATA;
// 										dataarray[0] = CON_TX_PARAM_DATA_DISINF_STATUS;
// 										Cl_rinseretcode = (Cl_Uint8Type)sv_nvmgetdata(NV_NVM_DISINF_STATUS, &dataarray[1]);
// 										Cl_rinseretcode = Cl_SendDatatoconsole(command,&dataarray,2);	
// 										break;			
// 															
// 										default:
// 										break;
// 									}
// 								}
// 						break;
						case EVENT_RINSE_TICK_50MS:
						if(cl_rinsing_substate == CL_RINSE_RINSING_BC_SWITCHING)
						{
									cl_50ms_timer++;
									if(cl_50ms_timer == 2)
									{
										cl_50ms_timer =0;
									//	sv_cntrl_deactivate_valve(VALVE_ID19);
									}
						} 
																	
						break;
						case EVENT_RINSE_TICK_500MS:
							if ((avgtemp2_dummy > 75) && (avgtemp2_dummy < 76))
							{
								sv_cntrl_decheater(2200);
							}
							else if ((avgtemp2_dummy > 76) && (avgtemp2_dummy < 77))
							{
								sv_cntrl_decheater(2000);
							}
							else if (avgtemp2_dummy > 77)
							{
								sv_cntrl_decheater(1800);
							}
						//UpdateHeaterControls();
						break;
						case EVENT_RINSE_TICK_SECOND:
						
						if(cl_rinsestate == CL_RINSE_STATE_RINSING)
						{
							Cl_Rinse_SendRinseStateData();	
						}
						
						

					//	UpdateHeaterControls();
						Cl_Rinsesecondscounter++;
						
						
						if(Cl_Rinsesecondscounter == 60){ // activate flow alarms only after 60 seconds into rinse
							
							
							Cl_rinseretcode =  Cl_AlarmActivateAlarms(FLOW_NO_FLOW,true );
							Cl_rinseretcode =  Cl_AlarmActivateAlarms(FLOW_LOW_FLOWRATE,true );
							Cl_rinseretcode =  Cl_AlarmActivateAlarms(FLOW_HIGH_FLOWRATE,true );
							
							
						}
						
						if ((Cl_Rinsesecondscounter == 10) && (rinse1 == 1))
						{
							Cl_SendDatatoconsole(CON_TX_COMMAND_PRINTTEXT,"RINSE2",6);
							sv_cntrl_deactivate_valve(VALVE_ID4);
							sv_cntrl_deactivatepump(DCMOTOR1);
						//	sv_cntrl_deactivate_valve(VALVE_ID6);
						//	sv_cntrl_deactivate_valve(VALVE_ID8);
							sv_cntrl_activate_valve(VALVE_ID19);
							sv_cntrl_activate_valve(VALVE_ID20);
							rinse2=1;
						}
						if ((Cl_Rinsesecondscounter == 30) && (rinse1 == 1) && (rinse2== 1))
						{
							Cl_SendDatatoconsole(CON_TX_COMMAND_PRINTTEXT,"RINSE3",6);
							sv_cntrl_deactivatepump(DCMOTOR2);
							sv_cntrl_deactivate_valve(VALVE_ID19);
							sv_cntrl_deactivate_valve(VALVE_ID20);
							sv_cntrl_activatepump(DCMOTOR1);
							sv_cntrl_activatepump(DCMOTOR2);
							sv_cntrl_activate_valve(VALVE_ID6);
							sv_cntrl_activate_valve(VALVE_ID8);
							sv_cntrl_activate_valve(VALVE_ID17);
							rinse3=1;
						}
						if ((Cl_Rinsesecondscounter == 55) && (rinse1 == 1) && (rinse2 == 1)&& (rinse3 == 1))
						{
							Cl_SendDatatoconsole(CON_TX_COMMAND_PRINTTEXT,"RINSE4",6);
							sv_cntrl_activate_valve(VALVE_ID4);
							sv_cntrl_deactivate_valve(VALVE_ID17);
							rinse4=1;
						}
						if(Cl_Rinsesecondscounter == 60)
						{
								cl_uf_controller(CL_UF_EVENT_STOP,0);
											
							Cl_Rinsesecondscounter = 0;
							Cl_RinseMinutescounter++;
							if (Cl_RinseMinutescounter == 1)
							{
								Cl_SendDatatoconsole(CON_TX_COMMAND_PRINTTEXT,"RINSE1",6);
								sv_cntrl_activate_valve(VALVE_ID4);
								sv_cntrl_deactivate_valve(VALVE_ID17);
								rinse1=1;
							}
							if (Cl_RinseMinutescounter == CL_BC_OPENFILL_TIMEOUT )  // earlier 2 min
							{
								Cl_SendDatatoconsole(CON_TX_COMMAND_PRINTTEXT,"DONE",6);
								sv_cntrl_deactivate_valve(VALVE_ID4);
								sv_cntrl_activate_valve(VALVE_ID17);
								sv_prop_stopopenfill();
								sv_cntrl_deactivate_valve(VALVE_ID2);
								sv_cntrl_deactivate_valve(VALVE_ID3);
								
								rinse1=0;
								rinse2=0;
								rinse3=0;
								rinse4=0;
							}
							
							Cl_RinseTotalMinutescounter++;
							if(cl_rinsestate == CL_RINSE_STATE_RINSING)
							{
								Cl_rinseretcode = Cl_Rinse_UpdateTimeInfo();
							}
	
						}
						//	Cl_Rinse_ResetAlertsforReassertion();
							//Cl_rinseretcode = Cl_Rinse_ProcessAlarms();
					
						break;
						case EVENT_RINSE_TICK_MINUTE:
						//	Cl_Rinsesecondscounter = 0;
							minute++;
							if (minute > 30)
							{
								sv_cntrl_poweroffheater();
								sv_cntrl_disable_loopback();
								sv_cntrl_enable_bypass();
							}
						//	Cl_RinseTotalMinutescounter++;
							Cl_Rinse_ResetAlertsforReassertion();
														
							if(Cl_Rinse_CheckforRinseCompletion() == CL_OK)
							{
								sv_cntrl_setgreenalarm();
								//Cl_rinseretcode = sv_cntrl_poweroffheater();
// 								sv_cntrl_activatepump(DCMOTOR1 );
// 								sv_cntrl_activatepump(DCMOTOR2 );
								sv_cntrl_deactivatepump(DCMOTOR1 );
								sv_cntrl_deactivatepump(DCMOTOR2 );
							//	sv_prop_stopopenfill();
							//	sv_prop_startpropeo_aligning();
							//	sv_cntrl_deactivate_valve(VALVE_ID2);
							//	sv_cntrl_deactivate_valve(VALVE_ID3);
								Cl_rinseretcode = cl_rinse_notifydacandgotorinsestandby();

								
								
								Cl_rinseretcode =  Cl_AlarmActivateAlarms(FLOW_NO_FLOW,false );					
								Cl_rinseretcode =  Cl_AlarmActivateAlarms(FLOW_LOW_FLOWRATE,false );
								Cl_rinseretcode =  Cl_AlarmActivateAlarms(FLOW_HIGH_FLOWRATE,false );
								Cl_rinseretcode =  Cl_AlarmActivateAlarms(ACID_IN_OPEN,false );
								Cl_rinseretcode =  Cl_AlarmActivateAlarms(BICARB_IN_OPEN,false );
								Cl_rinseretcode =  Cl_AlarmActivateAlarms(HOLDER1STATUS_OPEN,false );
								Cl_rinseretcode =  Cl_AlarmActivateAlarms(HOLDER2STATUS_OPEN,false );
								cl_rinsestate = CL_RINSE_STATE_POST_RINSE_STANDBY;
								
								Cl_Uint16Type sensordata;
								Cl_SysStat_GetSensor_Status_Query(SENSOR_TEMP1STATUS, &sensordata);
// 												uint16_t cal_data  = (402 *100* sensordata)/(2*32768); 
// 												res_temp_lookuptable(cal_data);
// 												 uint16_t temp = res_temp_value /10;
// 												
// 					//							temp = temp - 25 +12 + 2.5;
										uint16_t temp = sensordata /10;
												
										cl_Datastreamtype  cl_tdata;
										cl_tdata.word = 0;
										cl_tdata.Twobyte = temp;
										cl_tdata.bytearray[2] = 11;
										Cl_SendDatatoconsole(CON_TX_COMMAND_COMMAND_SCRIPT_PRNIT,&cl_tdata,4);
								//	Cl_rinseretcode = (Cl_ReturnCodeType)sv_setflowpath(Default path ?);//set appropirate flow path configuration
	
							}
						break;
						case EVENT_RINSE_TICK_HOUR:
							Cl_Rinseshourscounter++;
							Cl_RinsesTotalhourscounter++;
						break;
						case EVENT_STOP_RINSE:
							Cl_Rinse_StopRinse();
						break;
						break;
						case EVENT_RINSE_ALARM:
							Cl_rinseretcode = Cl_Rinse_ProcessAlarms();
						break;
						case EVENT_RINSE_ALERT:
							Cl_Alarm_GetAlarmStatus(FPCURRENTSTATUS,&alarmstatus1);
							if(alarmstatus1)
							{
							//	Cl_rinseretcode = Cl_bc_controller(BC_EVENT_CS);
							//	sv_cntrl_activate_valve(VALVE_ID19);
								cl_50ms_timer  = 1;
							}
							Cl_Alarm_GetAlarmStatus(SENSOR_TEMP2STATUS,&alarmstatus3);
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
						case EVT_RINSE_START_PRIME:
								blood_flag = 1;
					//	if(cl_dprep_prime_state != CL_DPREP_PRIME_STATE_PRIMING)
						{
					//		Cl_SendDatatoconsole(CON_TX_COMMAND_PRINTTEXT,"PRIME1",6);
							cl_dprep_activate_prime_related_alarms();
							Cl_RinseAlarmTable[CL_RINSE_ALARM_BLOODDOOR_STATUS_OPEN].IsRaised = false;
							cl_dprep_primecontroller(CL_DPREP_PRIME_PRIME_START,0);
						}
						
						break;
						case EVT_RINSE_STOP_PRIME:
							blood_flag = 0;
						cl_dprep_primecontroller(CL_DPREP_PRIME_PRIME_STOP,0);
						break;
						case EVT_RINSE_RCIRC_START:
						if(cl_dprep_prime_state != CL_DPREP_PRIME_STATE_PRIMING)
						{
							Cl_SendDatatoconsole(CON_TX_COMMAND_PRINTTEXT,"RCIRC",6);
							cl_dprep_primecontroller(CL_DPREP_PRIME_RCIRC_START,0);
							Cl_RinseAlarmTable[CL_RINSE_ALARM_BLOODDOOR_STATUS_OPEN].IsRaised = false;
						}
						
						break;
						case EVT_RINSE_RCIRC_STOP:

							Cl_SendDatatoconsole(CON_TX_COMMAND_PRINTTEXT,"RCIRC_STOP",6);
							cl_dprep_primecontroller(CL_DPREP_PRIME_RCIRC_STOP,0);

						
						break;
						case EVT_RINSE_COMMAND_SET_BLDPMP_ON:
						cl_bp_controller(CL_BP_EVENT_START,0);
						Cl_RinseAlarmTable[CL_RINSE_ALARM_BLOODDOOR_STATUS_OPEN].IsActive = true;
						break;
						case EVT_RINSE_COMMAND_SET_BLDPMP_OFF:
						cl_bp_controller(CL_BP_EVENT_STOP,0);
							Cl_RinseAlarmTable[CL_RINSE_ALARM_BLOODDOOR_STATUS_OPEN].IsActive = false;
						break;
						case EVENT_MAC_RINSE_COMPLETED:
						if(cl_rinsestate != CL_RINSE_STATE_POST_RINSE_STANDBY)
						{
								Cl_rinseretcode =  Cl_AlarmActivateAlarms(FLOW_NO_FLOW,false );					
								Cl_rinseretcode =  Cl_AlarmActivateAlarms(FLOW_LOW_FLOWRATE,false );
								Cl_rinseretcode =  Cl_AlarmActivateAlarms(FLOW_HIGH_FLOWRATE,false );
								Cl_rinseretcode =  Cl_AlarmActivateAlarms(ACID_IN_OPEN,false );
								Cl_rinseretcode =  Cl_AlarmActivateAlarms(BICARB_IN_OPEN,false );
								Cl_rinseretcode =  Cl_AlarmActivateAlarms(HOLDER1STATUS_OPEN,false );
								Cl_rinseretcode =  Cl_AlarmActivateAlarms(HOLDER2STATUS_OPEN,false );
							cl_rinsestate = CL_RINSE_STATE_POST_RINSE_STANDBY;
						}
						break;
						case EVENT_RINSE_SET_BLDPMP_ON:
						cl_dprep_primecontroller(CL_DPREP_PRIME_BLOODPUMP_START,0);
						break;
						case EVENT_RINSE_SET_BLDPMP_OFF:
						cl_dprep_primecontroller(CL_DPREP_PRIME_BLOODPUMP_STOP,0);
						break;
						case EVENT_RINSE_SET_BLDPUMPRATE:
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
						case EVENT_RINSE_CLR_ALARM:
						
						command = CON_TX_COMMAND_ALARM_CLEARED;
						data = (Cl_Uint8Type)COMMAND_RESULT_SUCCESS;
						Cl_rinseretcode = Cl_SendDatatoconsole(command,&data,0);
						
										sv_cntrl_resetyellowalarm();
										sv_cntrl_resetredalarm();
										sv_cntrl_nobuzzer();
										Cl_Rinse_ResetAlertsforReassertion();
					/*		Cl_rinseretcode = Cl_Rinse_StartRinse();
								
								if(Cl_rinseretcode == CL_OK){
									
							command = CON_TX_COMMAND_RINSE_CONFIRM;
							data = (Cl_Uint8Type)COMMAND_RESULT_SUCCESS;
							Cl_rinseretcode = Cl_SendDatatoconsole(command,&data,1);
							command = CON_TX_COMMAND_RINSE_STARTED;
							Cl_rinseretcode = Cl_SendDatatoconsole(command,&data,0);
							cl_wait(20);
							Cl_rinseretcode = Cl_Rinse_UpdateTimeInfo();
										cl_rinsestate = CL_RINSE_STATE_RINSING;
							cl_rinsing_substate = CL_RINSE_RINSING_OPENFILL;  
								}		*/		
										
						break;
						case EVENT_RINSE_BC_OPENFILL_COMPLETED:
						//sv_prop_stopopenfill();
						cl_rinsing_substate = CL_RINSE_RINSING_BC_SWITCHING;
						break;
				}
		break;
		case CL_RINSE_STATE_CRITICAL_ALARM:
				switch (cl_rinseevent)
				{
					case EVENT_START_RINSE:
					Cl_Rinse_StartRinse();
					command = CON_TX_COMMAND_RINSE_CONFIRM;
					data = (Cl_Uint8Type)COMMAND_RESULT_SUCCESS;
					Cl_rinseretcode = Cl_SendDatatoconsole(command,&data,1);					
					command = CON_TX_COMMAND_RINSE_STARTED;
					Cl_rinseretcode = Cl_SendDatatoconsole(command,&data,0);
					Cl_rinseretcode = Cl_Rinse_UpdateTimeInfo();
						cl_rinsestate = CL_RINSE_STATE_RINSING;
						cl_rinsing_substate = CL_RINSE_RINSING_OPENFILL;
					break;
				case EVENT_START_PRE_DISINF_RINSE:
				disinfectionrinseFlag = 1;
				Cl_Rinse_StartRinse();
				
				cl_rinsestate = CL_RINSE_STATE_PRE_DISINFECT_RINSING;
				cl_rinsing_substate = CL_RINSE_RINSING_OPENFILL;
				break;
				case EVENT_START_POST_DISINF_RINSE:
				disinfectionrinseFlag = 1;
				Cl_Rinse_StartRinse();
				cl_rinsestate = CL_RINSE_STATE_POST_DISINFECT_RINSING;
				cl_rinsing_substate = CL_RINSE_RINSING_OPENFILL;
				break;
					case EVENT_STOP_RINSE:
					Cl_Rinse_StopRinse();
					break;
// 					case EVT_RINSE_GET_DATA:
// 					if(Cl_ConsoleRxMsg.msgready == true)
// 					{
// 					
// 						switch(Cl_ConsoleRxMsg.data.byte[0])
// 						{
// 							case CON_RX_PARAM_DATA_RINSE_STATUS:
// 							command = CON_TX_COMMAND_SYSDATA;
// 							dataarray[0] = CON_TX_PARAM_DATA_RINSE_STATUS;
// 							Cl_rinseretcode = (Cl_Uint8Type)sv_nvmgetdata(NV_NVM_RINSE_STATUS, &dataarray[1]);
// 							Cl_rinseretcode = Cl_SendDatatoconsole(command,&dataarray,2);
// 						
// 							break;
// 							case CON_RX_PARAM_DATA_DISINF_STATUS:
// 							command = CON_TX_COMMAND_SYSDATA;
// 							dataarray[0] = CON_TX_PARAM_DATA_DISINF_STATUS;
// 							Cl_rinseretcode = (Cl_Uint8Type)sv_nvmgetdata(NV_NVM_DISINF_STATUS, &dataarray[1]);
// 							Cl_rinseretcode = Cl_SendDatatoconsole(command,&dataarray,2);	
// 							break;
// 							default:
// 							break;
// 						}
// 					}
// 					break;
					case EVENT_RINSE_ALARM:
					Cl_rinseretcode = Cl_Rinse_ProcessAlarms();
					break;
					
					case EVENT_RINSE_TICK_SECOND:
						
							for (tempcount = 0 ; tempcount < CL_RINSE_ALRM_MAX ; tempcount++)
							{
								//Cl_SendDatatoconsole(CON_TX_COMMAND_PRINTTEXT,"ENTER",8);
								Cl_Alarm_GetAlarmStatus(Cl_RinseAlarmTable[tempcount].Cl_RinseAlarmId,&Cl_RinseAlarmTable[tempcount].IsActive);
								if (Cl_RinseAlarmTable[tempcount].IsActive)
								{
										Cl_SendDatatoconsole(CON_TX_COMMAND_PRINTTEXT,"ENTER_1",8);
										Cl_Rinse_StopRinse();
										cl_rinsestate = CL_RINSE_STATE_CRITICAL_ALARM;
										tempcount=0;
										break;	
								}
								else if (tempcount == CL_RINSE_ALRM_MAX - 1)
								{
									if(!Cl_RinseAlarmTable[tempcount].IsActive)
									{
										Cl_SendDatatoconsole(CON_TX_COMMAND_PRINTTEXT,"SAFE",8);
										
										//alarm_flag = 0;
										Cl_Rinse_ResetAlertsforReassertion();
										//Cl_Rinse_StartRinse();
									//	command = CON_TX_COMMAND_RINSE_STARTED;
										//Cl_rinseretcode = Cl_SendDatatoconsole(command,&data,0);
									//	cl_rinsestate = CL_RINSE_STATE_RINSING;
									}
								}
							}
						break;
					case EVENT_RINSE_CLR_ALARM:
					
							if(cl_rinse_powerFailureFlag == 1){
								
								Cl_SysStat_GetSensor_Status_Query(SENSOR_POWER_FAILURE,&powerFailSensorData);
					
								if(powerFailSensorData == 0){
									
									cl_rinse_powerFailureFlag = 0;
									command = CON_TX_COMMAND_ALARM_CLEARED;
									Cl_SendDatatoconsole(command,&data,0);
									sv_cntrl_resetyellowalarm();
									sv_cntrl_resetredalarm();
									sv_cntrl_nobuzzer();
									Cl_Rinse_ResetAlertsforReassertion();
								}
							}
							else{
					
							command = CON_TX_COMMAND_ALARM_CLEARED;
							Cl_rinseretcode = Cl_SendDatatoconsole(command,&data,0);
							
					
										sv_cntrl_resetyellowalarm();
										sv_cntrl_resetredalarm();
										sv_cntrl_nobuzzer();
										Cl_Rinse_ResetAlertsforReassertion();
										
								// manav02 added to clear saline prime alarm		
										if(prime_alarm_flag == 1){
											
										cl_dprep_primecontroller(CL_DPREP_PRIME_PRIME_START,0); 	
											blood_flag = 1;
											prime_alarm_flag = 0;
											cl_rinsestate = CL_RINSE_STATE_POST_RINSE_STANDBY;
										}
										else if(disinfectionrinseFlag == 1){
											
											
											
										}
										else{
											
								Cl_rinseretcode = Cl_Rinse_StartRinse();
								
								if(Cl_rinseretcode == CL_OK){
									
							command = CON_TX_COMMAND_RINSE_CONFIRM;
							data = (Cl_Uint8Type)COMMAND_RESULT_SUCCESS;
							Cl_rinseretcode = Cl_SendDatatoconsole(command,&data,1);
							command = CON_TX_COMMAND_RINSE_STARTED;
							Cl_rinseretcode = Cl_SendDatatoconsole(command,&data,0);
							cl_wait(20);
							Cl_rinseretcode = Cl_Rinse_UpdateTimeInfo();
										cl_rinsestate = CL_RINSE_STATE_RINSING;
							cl_rinsing_substate = CL_RINSE_RINSING_OPENFILL;  
								}
				}
							}
						//alarm_flag = 1;
						break;
					case EVENT_RINSE_TICK_MINUTE:
					//Cl_rinseretcode = Cl_Rinse_ResetAlertsforReassertion();
					//Cl_rinseretcode = Cl_Rinse_repeatalrm();
					break;
					case EVENT_RINSE_SET_BLDPUMPRATE:
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
					default:break;
				}
		break;
		case CL_RINSE_STATE_POST_RINSE_STANDBY:
				switch (cl_rinseevent)
				{
					case EVENT_RINSE_ALARM:
					Cl_rinseretcode = Cl_Rinse_ProcessAlarms();
					break;
					
					case EVENT_START_RINSE:
					
					if(Cl_ConsoleRxMsg.msgready == true)
					{
					cl_Datastreamtype cl_temp  ;
					cl_temp.bytearray[0] = Cl_ConsoleRxMsg.data.byte[0];
					if (cl_temp.bytearray[0] == 1)
					{
						
					}
					else if (cl_temp.bytearray[0] == 2)
					{
						hotrinse_flag = 1;
					}
					
					}
					Cl_Rinse_StartRinse();
					command = CON_TX_COMMAND_RINSE_CONFIRM;
					data = (Cl_Uint8Type)COMMAND_RESULT_SUCCESS;
					Cl_rinseretcode = Cl_SendDatatoconsole(command,&data,1);					
					command = CON_TX_COMMAND_RINSE_STARTED;
					Cl_rinseretcode = Cl_SendDatatoconsole(command,&data,0);
					Cl_rinseretcode = Cl_Rinse_UpdateTimeInfo();
						cl_rinsestate = CL_RINSE_STATE_RINSING;
						cl_rinsing_substate = CL_RINSE_RINSING_OPENFILL;
					break;
					case EVENT_START_PRE_DISINF_RINSE:
						Cl_Rinse_StartRinse();
						Cl_rinseretcode = Cl_Rinse_UpdateTimeInfo();
						cl_rinsestate = CL_RINSE_STATE_PRE_DISINFECT_RINSING;
						cl_rinsing_substate = CL_RINSE_RINSING_OPENFILL;
					break;
					case EVENT_START_POST_DISINF_RINSE:
							Cl_Rinse_StartRinse();
							Cl_rinseretcode = Cl_Rinse_UpdateTimeInfo();
							cl_rinsestate = CL_RINSE_STATE_POST_DISINFECT_RINSING;
							cl_rinsing_substate = CL_RINSE_RINSING_OPENFILL;
					break;
// 					case EVT_RINSE_GET_DATA:
// 					if(Cl_ConsoleRxMsg.msgready == true)
// 					{
// 						switch(Cl_ConsoleRxMsg.data.byte[0])
// 						{
// 							case CON_RX_PARAM_DATA_RINSE_STATUS:
// 							command = CON_TX_COMMAND_SYSDATA;
// 							dataarray[0] = CON_TX_PARAM_DATA_RINSE_STATUS;
// 							Cl_rinseretcode = (Cl_Uint8Type)sv_nvmgetdata(NV_NVM_RINSE_STATUS, &dataarray[1]);
// 							Cl_rinseretcode = Cl_SendDatatoconsole(command,&dataarray,2);
// 							break;
// 							case CON_RX_PARAM_DATA_PRIME_STATUS:
// 							command = CON_TX_COMMAND_SYSDATA;
// 							dataarray[0] = CON_TX_PARAM_DATA_PRIME_STATUS;
// 							Cl_rinseretcode = (Cl_Uint8Type)sv_nvmgetdata(NV_NVM_PRIME_STATUS, &dataarray[1]);
// 							Cl_rinseretcode = Cl_SendDatatoconsole(command,&dataarray,2);
// 							break;
// 							case CON_RX_PARAM_DATA_MIXING_STATUS:
// 							command = CON_TX_COMMAND_SYSDATA;
// 							dataarray[0] = CON_TX_PARAM_DATA_MIXING;
// 							//Cl_rinseretcode = (Cl_Uint8Type)sv_nvmgetdata(NV_NVM_RINSE_STATUS, &dataarray[1]);
// 							dataarray[1] = 0;
// 							Cl_rinseretcode = Cl_SendDatatoconsole(command,&dataarray,2);
// 							break;
// 							case CON_RX_PARAM_DATA_RECIRC_STATUS:
// 							command = CON_TX_COMMAND_SYSDATA;
// 							dataarray[0] = CON_TX_PARAM_DATA_RCIRC;
// 							//Cl_rinseretcode = (Cl_Uint8Type)sv_nvmgetdata(NV_NVM_RINSE_STATUS, &dataarray[1]);
// 							dataarray[1] = 0;
// 							Cl_rinseretcode = Cl_SendDatatoconsole(command,&dataarray,2);
// 							break;
// 							case CON_RX_PARAM_DATA_DISINF_STATUS:
// 							command = CON_TX_COMMAND_SYSDATA;
// 							dataarray[0] = CON_TX_PARAM_DATA_DISINF_STATUS;
// 							Cl_rinseretcode = (Cl_Uint8Type)sv_nvmgetdata(NV_NVM_DISINF_STATUS, &dataarray[1]);
// 							Cl_rinseretcode = Cl_SendDatatoconsole(command,&dataarray,2);	
// 							break;
// 							
// 							default:
// 							break;
// 						}
// 					}
// 					break;
					case EVT_RINSE_START_PRIME:
					
					sv_cntrl_resetgreenalarm();
						blood_flag = 1;
					if(cl_dprep_prime_state != CL_DPREP_PRIME_STATE_PRIMING)
					{
						Cl_SendDatatoconsole(CON_TX_COMMAND_PRINTTEXT,"PRIME1",6);
						cl_dprep_activate_prime_related_alarms();
						Cl_RinseAlarmTable[CL_RINSE_ALARM_BLOODDOOR_STATUS_OPEN].IsRaised = false;
						cl_dprep_primecontroller(CL_DPREP_PRIME_PRIME_START,0);
					}
						
					break;
					case EVT_RINSE_STOP_PRIME:
					//blood_flag = 0;
					cl_dprep_primecontroller(CL_DPREP_PRIME_PRIME_STOP,0);
					break;
					case EVT_RINSE_RCIRC_START:
					if(cl_dprep_prime_state != CL_DPREP_PRIME_STATE_PRIMING)
					{
						Cl_SendDatatoconsole(CON_TX_COMMAND_PRINTTEXT,"RCIRC",6);
						cl_dprep_primecontroller(CL_DPREP_PRIME_RCIRC_START,0);
						Cl_RinseAlarmTable[CL_RINSE_ALARM_BLOODDOOR_STATUS_OPEN].IsRaised = false;
					}
						
					break;
					case EVT_RINSE_RCIRC_STOP:

						Cl_SendDatatoconsole(CON_TX_COMMAND_PRINTTEXT,"RCIRC_STOP",6);
						cl_dprep_primecontroller(CL_DPREP_PRIME_RCIRC_STOP,0);

						
					break;
					case EVENT_MAC_RINSE_COMPLETED:
					return(CL_OK);
					break;
					
					case EVENT_RINSE_CLR_ALARM:
					
					command = CON_TX_COMMAND_ALARM_CLEARED;
					Cl_rinseretcode = Cl_SendDatatoconsole(command,&data,0);
					
					
					sv_cntrl_resetyellowalarm();
					sv_cntrl_resetredalarm();
					sv_cntrl_nobuzzer();
					Cl_Rinse_ResetAlertsforReassertion();
					
					break;
					
					case EVENT_RINSE_SET_BLDPUMPRATE:
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
					default: 
					
					return(CL_REJECTED);
					break;
				}
					
		break;

	
	default:
			switch(cl_rinseevent)
			{
				case EVENT_START_RINSE:
				break;
				case EVENT_STOP_RINSE:
				Cl_Rinse_StopRinse();
				break;
				case EVENT_RINSE_TIMEOUT:
				//printf("RTO\n");
				break;
				case EVENT_RINSE_ALARM:
				// update console 
				
					Cl_rinseretcode = Cl_Rinse_ProcessAlarms();
				
				break;
				case EVENT_MAC_RINSE_COMPLETED:
				if(cl_rinsestate != CL_RINSE_STATE_POST_RINSE_STANDBY)
				{
								Cl_rinseretcode =  Cl_AlarmActivateAlarms(FLOW_NO_FLOW,false );					
								Cl_rinseretcode =  Cl_AlarmActivateAlarms(FLOW_LOW_FLOWRATE,false );
								Cl_rinseretcode =  Cl_AlarmActivateAlarms(FLOW_HIGH_FLOWRATE,false );
								Cl_rinseretcode =  Cl_AlarmActivateAlarms(ACID_IN_OPEN,false );
								Cl_rinseretcode =  Cl_AlarmActivateAlarms(BICARB_IN_OPEN,false );
								Cl_rinseretcode =  Cl_AlarmActivateAlarms(HOLDER1STATUS_OPEN,false );
								Cl_rinseretcode =  Cl_AlarmActivateAlarms(HOLDER2STATUS_OPEN,false );
					cl_rinsestate = CL_RINSE_STATE_POST_RINSE_STANDBY;
				}
				break;
				case EVENT_RINSE_DIALYSIS_PREP:
				if (cl_rinsestate != CL_RINSE_STATE_POST_RINSE_STANDBY )
				{
					Cl_rinseretcode = CL_ERROR;
				}
				break;
				default:
				break;
			}
	break;
}


return Cl_rinseretcode;
}

Cl_ReturnCodeType  cl_rinse_translatemacevent(Cl_Mac_EventsType Cl_MacRinseEvt,Cl_Rinse_Eevents* cl_rinse_event)
{
	switch(Cl_MacRinseEvt)
	{
		case EVT_CONSOLE_COMMAND_HEPARIN_START:
		*cl_rinse_event= EVENT_RINSE_HEP_PMP_START;
		break;
		case EVT_CONSOLE_COMMAND_HEPARIN_STOP:
		*cl_rinse_event= EVENT_RINSE_HEP_PMP_STOP;
		break;
		case EVT_CONSOLE_COMMAND_HEPARIN_BOLUS:
		*cl_rinse_event= EVENT_RINSE_HEP_PMP_BLOUS;
		break;
		

		case EVT_CONSOLE_COMMAND_CLR_ALARM:
		*cl_rinse_event= EVENT_RINSE_CLR_ALARM;
		break;
		case  EVT_CONSOLE_COMMAND_RINSE_START:
		*cl_rinse_event= EVENT_START_RINSE;
		break;
		case  MACREQ_START_PRE_DISINF_RINSE:
		*cl_rinse_event= EVENT_START_PRE_DISINF_RINSE;
		break;
		case  MACREQ_START_POST_DISINF_RINSE:
		*cl_rinse_event= EVENT_START_POST_DISINF_RINSE;
		break;
		case EVT_CONSOLE_COMMAND_STOP_RINSE:
		*cl_rinse_event= EVENT_STOP_RINSE;
		break;
	
		case MACREQ_RINSE_COMPLETED:
		*cl_rinse_event = EVENT_MAC_RINSE_COMPLETED;
		break;
		case MACREQ_BC_OPENFILL_COMPLETED:
		*cl_rinse_event = EVENT_RINSE_BC_OPENFILL_COMPLETED;
		break;
		case EVT_CONSOLE_COMMAND_DIALYSIS_PREP:
		*cl_rinse_event = EVENT_RINSE_DIALYSIS_PREP;
		break; 
		
		case EVT_TICK_50M:
		*cl_rinse_event =   EVENT_RINSE_TICK_50MS;
		break;
 		break;
		case EVT_TICK_100M:
		*cl_rinse_event =   EVENT_RINSE_TICK_100MS;
		break;
		case EVT_TICK_500M:
		  		*cl_rinse_event =   EVENT_RINSE_TICK_500MS;
		break;
		case EVT_TICK_SEC:
		*cl_rinse_event =   EVENT_RINSE_TICK_SECOND;
		break;
		case EVT_TICK_MIN:
		*cl_rinse_event =   EVENT_RINSE_TICK_MINUTE;
		break;			 
		case EVT_TICK_HOUR:
		*cl_rinse_event =   EVENT_RINSE_TICK_HOUR;
		break;
		case EVT_ALARM_TRIGGERED:
		*cl_rinse_event =  EVENT_RINSE_ALARM;
		break;
		case  EVT_ALERT_TRIGGERED:
		*cl_rinse_event =  EVENT_RINSE_ALERT;
		break;
		
		case EVT_CONSOLE_COMMAND_SET_DATA:
		*cl_rinse_event = EVT_RINSE_SET_DATA;
		break;
		
		case EVT_CONSOLE_COMMAND_GET_DATA:
		*cl_rinse_event =  EVT_RINSE_GET_DATA;
		break;
		case EVT_CONSOLE_COMMAND_START_PRIME:
		*cl_rinse_event =  EVT_RINSE_START_PRIME;
		break;
		case EVT_CONSOLE_COMMAND_STOP_PRIME:
		*cl_rinse_event =  EVT_RINSE_STOP_PRIME;
		break;
		case EVT_CONSOLE_COMMAND_RCIRC_START:
		*cl_rinse_event = EVT_RINSE_RCIRC_START;
		break;
		case EVT_CONSOLE_COMMAND_RCIRC_STOP:
		*cl_rinse_event = EVT_RINSE_RCIRC_STOP;
		break;
		case EVT_CONSOLE_COMMAND_SET_BLDPMP_ON:
		*cl_rinse_event = EVT_RINSE_COMMAND_SET_BLDPMP_ON;
		break;
		case EVT_CONSOLE_COMMAND_SET_BLDPMP_OFF:
		*cl_rinse_event = EVT_RINSE_COMMAND_SET_BLDPMP_OFF;
		break;
		break;
		case EVT_CONSOLE_COMMAND_SET_BLDPUMPRATE:
		*cl_rinse_event = EVT_RINSE_COMMAND_SET_BLDPUMPRATE;
		break;
		case EVT_CONSOLE_COMMAND_SET_BLDPUMPINC_MANUAL:
		*cl_rinse_event = EVT_CONSOLE_COMMAND_SET_BLDPUMPINC_MANUAL;
		break;
		case EVT_CONSOLE_COMMAND_SET_BLDPUMPDEC_MANUAL:
		*cl_rinse_event = EVT_CONSOLE_COMMAND_SET_BLDPUMPDEC_MANUAL;
		break;
		case EVT_CONSOLE_COMMAND_MUTE_ALARM:
		*cl_rinse_event = EVT_RINSE_MUTE;
		break;
		
		default:
		break;
	}
		 return CL_OK;
}
 Cl_ReturnCodeType Cl_rinse_init(void)
 {
	 disinfectionrinseFlag = 0;
		tempcount = 0;
	   blood_flag=0;
	   var1=0;
	   rinse1=0;
	   rinse2=0;
	   rinse3=0;
	   rinse4=0;
	   prime_alarm_flag = 0;
	 cl_rinse_powerFailureFlag = 0;
	  cl_rinsestate = CL_RINSE_STATE_IDLE;
	  cl_rinsing_substate = CL_RINSE_RINSING_IDLE;
	   Cl_Rinsesecondscounter =0 ;Cl_RinseMinutescounter=0; Cl_Rinseshourscounter=0;Cl_RinseOpenFillsecondscounter=0;
	   Cl_RinseTotalMinutescounter=0; Cl_RinsesTotalhourscounter=0;
	   Cl_RinseOpenFillTimeOut = false;
	   cl_gfillinprogress = false;
	   wait_cnt = 0;TmpVal=0;
	   temp1;temp2;
	    cl_50ms_timer = 0;
	   prevrinsealarm_id =_NO_ALARM;
	   rinsealarm =0;
	  
	    hotrinse_flag=0;minute=0;
	
	 return CL_OK;
 }



 Cl_ReturnCodeType cl_rinse_processtimeoutevets(Cl_Rinse_Eevents cl_rinseevent)
 {
	 
	 switch(cl_rinseevent)
	 {
		 case EVENT_RINSE_TICK_SECOND:
		 // update console with time info /system info..?
		 break;
		 case EVENT_RINSE_TICK_MINUTE:
		 // check for state transition time outs
		 break;
		 case EVENT_RINSE_TICK_HOUR:
		 break;
		 default:
		 break;

	 }
	 
	 return CL_OK;
 }
 
 
 Cl_ReturnCodeType Cl_Rinse_UpdateTimeInfo(void)
 {
	 
	 Cl_ReturnCodeType Cl_rinseretcode = CL_ERROR;
	 Cl_ConsoleTxCommandtype command = CON_TX_COMMAND_COMMAND_MAX;
	 Cl_Uint8Type data[7] = {0,0,0,0,0 ,0,0};
	 
	command = CON_TX_COMMAND_REM_TIME;

	
		data[0]= (Cl_Uint8Type)CLEAN_DATA;
		data[1]= (Cl_Uint8Type)Cl_RinseTotalMinutescounter;

		
		Cl_rinseretcode = Cl_SendDatatoconsole(command,&data,2);
		
		
//	Cl_Rinse_SendRinseStateData();                                                          commented on 28.05.2014
	return CL_OK;
	
 }
 
  Cl_ReturnCodeType Cl_Rinse_UpdateSecTick(void)
  {
	  
	  Cl_ReturnCodeType Cl_rinseretcode = CL_ERROR;
	  Cl_ConsoleTxCommandtype command = CON_TX_COMMAND_COMMAND_MAX;
	  Cl_Uint8Type data[7] = {0,0,0,0,0 ,0,0};
	  
	  command = CON_TX_COMMAND_TICK_SEC;
	  

	  
	  Cl_rinseretcode = Cl_SendDatatoconsole(command,&data,0);
	  //Cl_rinseretcode = Cl_SendDatatoconsole(command,&data,1);
	  Cl_Rinse_SendRinseStateData();
	  return CL_OK;
	  
  }
 
 
 Cl_ReturnCodeType Cl_Rinse_CheckforRinseCompletion(void)
 {
	 Cl_ReturnCodeType Cl_rinseretcode = CL_ERROR;
	 if(gtest_short_rinse)
	 {
		if(Cl_RinseTotalMinutescounter > 5 ) 
		Cl_rinseretcode = CL_OK;
		return Cl_rinseretcode;
	 }
	if (hotrinse_flag == 1)
	{
		if (Cl_RinseTotalMinutescounter >= (CL_HOTRINSE_TIMEOUT_MIN ) )
		{
			sv_cntrl_poweroffheater();
			sv_cntrl_disable_loopback();											// added on 30.06.2017
				Cl_AlarmActivateAlarms(TEMP1_LOW_THRESHOLD,false );     // manav02 added no ts1 alarm even after hot rinse
				Cl_AlarmActivateAlarms(TEMP1_HIGH_THRESHOLD,false );

			SetHeaterState(CL_HEATER_STATE_OFF);
			Cl_bc_controller(BC_EVENT_STOP);
			Cl_bc_controller(BC_EVENT_START);
			Cl_RinseTotalMinutescounter = 0;	
			//Cl_rinseretcode = CL_OK;
			hotrinse_flag = 0;
			//return Cl_rinseretcode;					// // added on 19.08.2017
		}
// 		if (Cl_RinseTotalMinutescounter >= CL_HOTRINSE_TIMEOUT_MIN )
// 		{
// 			
// 			Cl_rinseretcode = CL_OK;
// 			hotrinse_flag = 0;
// 			return Cl_rinseretcode;
// 		}
	}
	else if (Cl_RinseTotalMinutescounter >= CL_RINSE_TIMEOUT_MIN)
	{
		sv_cntrl_setgreenalarm();
	
	//	sv_cntrl_buzzer();	
	
	Cl_AlarmActivateAlarms(TEMP1_LOW_THRESHOLD,true );     // manav02 added ts1 alarm even after rinse
	Cl_AlarmActivateAlarms(TEMP1_HIGH_THRESHOLD,true );

		Cl_rinseretcode = CL_OK;
	}

	return Cl_rinseretcode;
	 
 }
 
Cl_ReturnCodeType cl_rinse_notifydacandgotorinsestandby(void)
{
	Cl_ReturnCodeType 	Cl_rinseretcode2 = CL_OK;
	
	Cl_Uint8Type data =0;
	Cl_RinseStop();
	Cl_RinseFlowOff();
//	Cl_rinseretcode2 = sv_cntrl_poweroffheater();
	//inform DAC about rinse completed state.
	Cl_Rinsesecondscounter = 0;
	Cl_RinseMinutescounter= 0;
	Cl_Rinseshourscounter= 0;
	Cl_RinseTotalMinutescounter= 0;
	Cl_RinsesTotalhourscounter=0;
	
	data = 1;
	cl_wait(5);
	if((cl_rinsestate == CL_RINSE_STATE_RINSING)  )
	{
		Cl_rinseretcode2 = sv_nvmsetdata(NV_NVM_RINSE_STATUS,&data,1);
		Cl_rinseretcode2 = Cl_SendDatatoconsole(CON_TX_COMMAND_RINSE_COMPLETED,&data,0);
		Cl_rinseretcode2 = Cl_mac_apprequesthandler(MACREQ_RINSE_COMPLETED);
		
	}
	if(cl_rinsestate == CL_RINSE_STATE_PRE_DISINFECT_RINSING)
	{
		Cl_rinseretcode2 = Cl_mac_apprequesthandler(MACREQ_PRE_DISINFECT_RINSE_COMPLETED);
	}
	else if(cl_rinsestate == CL_RINSE_STATE_POST_DISINFECT_RINSING)
	{
		Cl_rinseretcode2 = Cl_mac_apprequesthandler(MACREQ_POST_DISINFECT_RINSE_COMPLETED);
	}
	
	cl_flow_switchcontroller(FLOW_SWITCH_CNTRLER_EVENT_STOP);

						
	return Cl_rinseretcode2;
}

Cl_ReturnCodeType Cl_Rinse_UpdateAlarmTable(ClRinseAlarmIdType* ClRinseAlarmId )
{
	Cl_ReturnCodeType 	Cl_rinseretcode = CL_OK;
	Cl_Uint8Type tempcount = 0;
	Cl_Uint8Type data[3] ={0, 0} ,*data1 = NULL;
	Cl_Uint8Type data3;
	Cl_NewAlarmIdType cl_alarmId;
	
	Cl_ConsoleTxCommandtype command = CON_TX_COMMAND_COMMAND_MAX;
	
	
	for (tempcount = 0 ; tempcount < CL_RINSE_ALRM_MAX ; tempcount++)
	{
		
		Cl_Alarm_GetAlarmStatus(Cl_RinseAlarmTable[tempcount].Cl_RinseAlarmId,&Cl_RinseAlarmTable[tempcount].IsActive);

		
			if (Cl_RinseAlarmTable[tempcount].IsActive)
			{
				if(!Cl_RinseAlarmTable[tempcount].IsRaised)
					{
	
						Cl_RinseAlarmTable[tempcount].IsRaised = true;
					
						Cl_rinseretcode = CL_RinseAlarmActon(Cl_RinseAlarmTable[tempcount].Cl_RinseAlarmId);
					
					}
			
			}
			else
			{
				// alarm was present before , but not active now.
				if(Cl_RinseAlarmTable[tempcount].IsRaised == true)
				{
					Cl_RinseAlarmTable[tempcount].IsRaised = false;
					command = CON_TX_COMMAND_ALARM;
					data[0] = (Cl_Uint8Type)(Cl_RinseAlarmTable[tempcount].Cl_RinseAlarmId);
					data[1] = (Cl_Uint8Type) CL_ALARM_CLEARED;
			//		Cl_rinseretcode = Cl_SendDatatoconsole(command,&data,2);
				}

			}
		//}

	}

return (Cl_rinseretcode );
}

Cl_ReturnCodeType Cl_Rinse_ProcessAlarms(void )
{
		Cl_ReturnCodeType 	Cl_rinseretcode = CL_OK;
		ClRinseAlarmIdType ClRinseAlarmId;
		Cl_Uint8Type data;
		Cl_ConsoleTxCommandtype command = CON_TX_COMMAND_COMMAND_MAX;
		Cl_NewAlarmIdType cl_rinsealarmid;
	
	   Cl_rinseretcode = Cl_Rinse_UpdateAlarmTable(&ClRinseAlarmId);

		   
		  
	   	return (Cl_rinseretcode);
}

Cl_ReturnCodeType Cl_Rinse_ResetAlertsforReassertion(void )
{
	Cl_ReturnCodeType 	Cl_rinseretcode = CL_OK;
	ClRinseAlarmIdType ClRinseAlarmId;
	Cl_Uint8Type data;
	Cl_ConsoleTxCommandtype command = CON_TX_COMMAND_COMMAND_MAX;
	Cl_NewAlarmIdType cl_rinsealarmid;
	
		
	Cl_RinseAlarmTable[CL_RINSE_ALARM_BLOODDOOR_STATUS_OPEN].IsRaised = false;
	//Cl_AlarmResetAlarm(BLOODDOOR_STATUS_OPEN);
	Cl_AlarmResetAlarm(BLOODDOOR_STATUS_OPEN);//Cl_RinseAlarmTable[CL_RINSE_ALARM_BLOODDOOR_STATUS_OPEN].Cl_RinseAlarmId);
	//Cl_RinseAlarmTable[CL_RINSE_ALARM_BYPASSDOOR_STATUS_OPEN].IsRaised = false;
	//Cl_AlarmResetAlarm(BYPASSDOOR_STATUS_OPEN);

	//Cl_RinseAlarmTable[CL_RINSE_ALARM_COND_STATUS_LOW].IsRaised = false;
	//Cl_AlarmResetAlarm(COND_STATUS_LOW);
	Cl_RinseAlarmTable[CL_RINSE_ALARM_ACID_IN].IsRaised = false;
	Cl_AlarmResetAlarm(ACID_IN_OPEN);
	Cl_RinseAlarmTable[CL_RINSE_ALARM_BICARB_IN].IsRaised = false;
	Cl_AlarmResetAlarm(BICARB_IN_OPEN);
	Cl_RinseAlarmTable[CL_RINSE_ALARM_COND_STATUS_HIGH].IsRaised = false;
	Cl_AlarmResetAlarm(COND_STATUS_HIGH);
	//Cl_RinseAlarmTable[CL_RINSE_ALARM_COND_DAC_OPEN].IsRaised = false;
	//Cl_RinseAlarmTable[CL_RINSE_ALARM_COND_DAC_RO].IsRaised = false;
	//Cl_RinseAlarmTable[CL_RINSE_ALARM_COND_DAC_HIGH].IsRaised = false;
	Cl_RinseAlarmTable[CL_RINSE_ALARM_PS1_HIGH_THRESHOLD].IsRaised = false;
	Cl_RinseAlarmTable[CL_RINSE_ALARM_PS1_LOW_THRESHOLD].IsRaised = false;
	Cl_AlarmResetAlarm(PS1_HIGH_THRESHOLD);
	Cl_AlarmResetAlarm(PS1_LOW_THRESHOLD);
	Cl_RinseAlarmTable[CL_RINSE_ALARM_PS2_HIGH_THRESHOLD].IsRaised = false;
	Cl_RinseAlarmTable[CL_RINSE_ALARM_PS2_LOW_THRESHOLD].IsRaised = false;
	Cl_AlarmResetAlarm(PS2_HIGH_THRESHOLD);
	Cl_AlarmResetAlarm(PS2_LOW_THRESHOLD);
	Cl_RinseAlarmTable[CL_RINSE_ALARM_PS3_HIGH_THRESHOLD].IsRaised = false;
	Cl_RinseAlarmTable[CL_RINSE_ALARM_PS3_LOW_THRESHOLD].IsRaised = false;
	Cl_AlarmResetAlarm(PS3_LOW_THRESHOLD);
	Cl_AlarmResetAlarm(PS3_HIGH_THRESHOLD);
	Cl_RinseAlarmTable[CL_RINSE_ALARM_TEMP2_HIGH_THRESHOLD].IsRaised = false;
	Cl_AlarmResetAlarm(TEMP2_LOW_THRESHOLD);
	Cl_AlarmResetAlarm(TEMP2_HIGH_THRESHOLD);
	Cl_RinseAlarmTable[CL_RINSE_ALARM_TEMP3_HIGH_THRESHOLD].IsRaised = false;
	Cl_RinseAlarmTable[CL_RINSE_ALARM_TEMP3_LOW_THRESHOLD].IsRaised = false;
	Cl_AlarmResetAlarm(TEMP3_HIGH_THRESHOLD);
	Cl_AlarmResetAlarm(TEMP3_LOW_THRESHOLD);
	Cl_RinseAlarmTable[CL_RINSE_ALARM_HOLDER1STATUS_OPEN].IsRaised = false;
	Cl_AlarmResetAlarm(HOLDER1STATUS_OPEN);
	//Cl_AlarmResetAlarm(HOLDER1STATUS_CLOSED);
	Cl_RinseAlarmTable[CL_RINSE_ALARM_HOLDER2STATUS_OPEN].IsRaised = false;
	Cl_AlarmResetAlarm(HOLDER2STATUS_OPEN);
	//Cl_AlarmResetAlarm(HOLDER2STATUS_CLOSED);
	Cl_RinseAlarmTable[CL_RINSE_ALARM_POWER_FAILURE].IsRaised = false;
	Cl_AlarmResetAlarm(POWER_FAILURE);
	
	Cl_RinseAlarmTable[CL_RINSE_ALARM_FLOW_NO_FLOW].IsRaised = false;
	Cl_RinseAlarmTable[CL_RINSE_ALARM_FLOW_HIGH_FLOWRATE].IsRaised = false;
	Cl_RinseAlarmTable[CL_RINSE_ALARM_FLOW_LOW_FLOWRATE].IsRaised = false;
	
	Cl_AlarmResetAlarm(FLOW_NO_FLOW);
	Cl_AlarmResetAlarm(FLOW_HIGH_FLOWRATE);
	Cl_AlarmResetAlarm(FLOW_LOW_FLOWRATE);
	
	//Cl_RinseAlarmTable[CL_RINSE_ALARM_LEVEL_SWITCH_LOW_TOGGLERATE].IsRaised = false;
	Cl_RinseAlarmTable[CL_RINSE_ALARM_TEMP1_LOW_THRESHOLD].IsRaised = false;
	Cl_AlarmResetAlarm(TEMP1_LOW_THRESHOLD);
	Cl_RinseAlarmTable[CL_RINSE_ALARM_TEMP1_HIGH_THRESHOLD].IsRaised = false;
	Cl_AlarmResetAlarm(TEMP1_HIGH_THRESHOLD);
	Cl_RinseAlarmTable[CL_RINSE_ALARM_BP_STOPPED].IsRaised = false;
	Cl_AlarmResetAlarm(BP_STOPPED);
	//Cl_AlarmResetAlarm(TEMP2_HIGH_THRESHOLD);
	
	
	return (Cl_rinseretcode);
}


Cl_ReturnCodeType  CL_RinseAlarmActon(Cl_NewAlarmIdType cl_rinsealarmid)
{
		Cl_ReturnCodeType 	Cl_rinseretcode = CL_OK;
			Cl_Uint8Type data[3] ={0, 0} ,*data1 = NULL;
			Cl_Uint8Type data3;
			Cl_ConsoleTxCommandtype command = CON_TX_COMMAND_COMMAND_MAX;
			Cl_Uint16Type levelswitchstatus = 0;	
			static Cl_Uint8Type fillseccounter=0;
			Cl_NewAlarmIdType NewAlarmId = _NO_ALARM;
		float temp;
		//Cl_Uint16Type var=0;

			

							
		switch(cl_rinsealarmid)
		{
			
			case POWER_FAILURE:
			
		//	sv_cntrl_setredalarm();
		cl_rinse_powerFailureFlag = 1 ;
			sv_cntrl_buzzer();
			Cl_Rinse_StopRinse();
			cl_rinsestate = CL_RINSE_STATE_CRITICAL_ALARM;
			break;
			
			case BP_STOPPED:
			// manav05 added flag for saline priming
			if(blood_flag == 1){
			
			prime_alarm_flag = 1;
			sv_cntrl_setredalarm();
			sv_cntrl_buzzer();
			cl_rinsestate = CL_RINSE_STATE_CRITICAL_ALARM;	
				
			}
			else{
				
				sv_cntrl_setredalarm();
				sv_cntrl_buzzer();
				cl_rinsestate = CL_RINSE_STATE_CRITICAL_ALARM;
			}
			
			break;
			
			case BLOODDOOR_STATUS_OPEN:
					sv_cntrl_setyellowalarm();
					sv_cntrl_buzzer();
					cl_bp_controller(CL_BP_EVENT_STOP,0);
					Cl_AlarmActivateAlarms(BLOODDOOR_STATUS_OPEN,false);
					if(blood_flag == 1){
						
						sv_cntrl_setredalarm();
						sv_cntrl_resetyellowalarm();
						prime_alarm_flag = 1;
						//blood_flag = 0;
						sv_cntrl_buzzer();
						cl_dprep_primecontroller(CL_DPREP_PRIME_PRIME_STOP,0);
						//Cl_AlarmActivateAlarms(BLOODDOOR_STATUS_OPEN,false);  // manav02 added for prime alarm
						cl_rinsestate = CL_RINSE_STATE_CRITICAL_ALARM;         // manav02 added
					}// manav03 added
					else{
						
						
						//cl_rinsestate = CL_RINSE_STATE_ALERT_ALARM;
					}
				//	cl_dprep_primecontroller(CL_DPREP_PRIME_PRIME_ALARM,0);
			//		Cl_Rinse_StopRinse();
			//		cl_rinsestate = CL_RINSE_STATE_CRITICAL_ALARM;
				
			break;
			
			case ACID_IN_OPEN:
			case BICARB_IN_OPEN:
			case HOLDER1STATUS_OPEN:
			case HOLDER2STATUS_OPEN:
				case TEMP1_HIGH_THRESHOLD:
				case TEMP1_LOW_THRESHOLD:
				case TEMP2_HIGH_THRESHOLD:
				case TEMP2_LOW_THRESHOLD:
				case TEMP3_HIGH_THRESHOLD:
					case PS1_HIGH_THRESHOLD:
					case PS1_LOW_THRESHOLD:
					case	PS2_HIGH_THRESHOLD:
					case    PS2_LOW_THRESHOLD:
					case	PS3_HIGH_THRESHOLD:
					case    PS3_LOW_THRESHOLD:
					case COND_STATUS_HIGH:
					case FLOW_NO_FLOW:
					case FLOW_LOW_FLOWRATE:
					case FLOW_HIGH_FLOWRATE:
					case 	BC_OVERRUN:
					case	BC_UNDERRUN:
					case	BC_FAILED:
					case 	UFP_OVERRUN:
					case	UFP_UNDERRUN:
					case BC_ALARM:
	//		sv_cntrl_setyellowalarm();
	
	if(disinfectionrinseFlag == 1){
	
				sv_cntrl_buzzer();
				sv_cntrl_setredalarm();
				Cl_Rinse_DisinfStop();
				cl_rinsestate = CL_RINSE_STATE_CRITICAL_ALARM;
						
	}
	else{
		
	
	
			sv_cntrl_buzzer();
			Cl_Rinse_StopRinse();
			cl_rinsestate = CL_RINSE_STATE_CRITICAL_ALARM;
	}
			break;
			
			/*case LEVELSWITCH_OFF_TO_ON:
				sv_cntrl_setredalarm();
				sv_cntrl_buzzer();
			Cl_SysStat_GetSensor_Status_Query(LEVELSWITCH_OFF_TO_ON,&levelswitchstatus);
			{
				if(levelswitchstatus == 1)
				{
	
					//fillseccounter++;

				//	Cl_RinseFlowOff();
					cl_gfillinprogress = false;		
					
				}
			}
			break;
			case LEVELSWITCH_ON_TO_OFF:
			// TURN ON WATER INLET
		//	NewAlarmId = _NO_ALARM;
			Cl_SysStat_GetSensor_Status_Query(LEVELSWITCH_ON_TO_OFF,&levelswitchstatus);
			{
				if(levelswitchstatus == 0)
				{

					cl_gfillinprogress = true;
					
				}
			}


			break;*/
			
			case WATCHDOG_TIMER: //27
		//		NewAlarmId = _WATCHDOG_TIMER;
			break;
			default:
			cl_rinsealarmid = _NO_ALARM;
			break;

		}
		
		 if(cl_rinsealarmid != _NO_ALARM)
		 {
			 

			
			data[0] = (Cl_Uint8Type)cl_rinsealarmid;
			data[1] = (Cl_Uint8Type) CL_ALARM_TRIGGERED;
			data[2] = (Cl_Uint8Type)CRITICAL;
			command = CON_TX_COMMAND_ALARM;
			
			Cl_rinseretcode = Cl_SendDatatoconsole(command,&data,3);
			cl_wait(10);
					

		 }
		 prevrinsealarm_id = cl_rinsealarmid;
		return CL_OK;	
}
Cl_ReturnCodeType  Cl_RinseSendAlarm(Cl_NewAlarmIdType cl_rinsealarmId )
{
	
			Cl_ReturnCodeType Cl_RetVal = CL_OK;
			 
			 
			Cl_Uint8Type data[3] ={0, 0} ,*data1 = NULL;
			Cl_Uint8Type data3;
			Cl_ConsoleTxCommandtype command = CON_TX_COMMAND_COMMAND_MAX;
			cl_rinsestate = CL_RINSE_STATE_CRITICAL_ALARM;
			sv_cntrl_setredalarm();
			sv_cntrl_buzzer();
			if(cl_rinsealarmId != _NO_ALARM)
			 {
				data[0] = (Cl_Uint8Type)cl_rinsealarmId;
				data[1] = (Cl_Uint8Type) CL_ALARM_TRIGGERED;
				data[2] = (Cl_Uint8Type)CRITICAL;
				command = CON_TX_COMMAND_ALARM;
			
				Cl_RetVal = Cl_SendDatatoconsole(command,&data,3);
				cl_wait(10);

			 }
}
Cl_ReturnCodeType  Cl_RinseFlowOn(void )
{
	Cl_ReturnCodeType Cl_RetVal = CL_OK;
	sv_cntrl_activate_valve(VALVE_ID1);
	cl_gfillinprogress = true;
	return Cl_RetVal;
	
}

Cl_ReturnCodeType  Cl_RinseFlowOff(void )
{
	Cl_ReturnCodeType Cl_RetVal = CL_OK;
	sv_cntrl_deactivate_valve(VALVE_ID1);
	cl_gfillinprogress = false;
	return Cl_RetVal;
	
}

Cl_ReturnCodeType  Cl_RinseStop(void)
{
	Cl_ReturnCodeType Cl_RetVal = CL_OK;
	//sv_cntrl_setflowpath(FLOW_PATH_IDLE_RINSE);
	Cl_bc_controller(BC_EVENT_STOP);
	
	sv_cntrl_poweroffheater();
	return Cl_RetVal;
	
}
Cl_ReturnCodeType Cl_Rinse_SendRinseStateData(void)
{
	static float cond = 0;
	Cl_ReturnCodeType  Cl_rinseretcode = CL_OK;
	StateDatatype data;
	Cl_Uint8Type count=0;
	Cl_Uint16Type ts2,temp=0,temp1=0,sensordata =0 , temp2 = 0;
	Cl_Uint8Type dataarray[12] =  {0,0,0,0,0,0,0,0,0,0,0,0};
	static timecount = 0;
	static float tmp3 = 0, tmp2 = 0;
	
	//Cl_Console_bulkdatatype 
	
	timecount++;

	if (timecount < 5)
	{
		return;
	}
	else
	{
		timecount = 0;
	}
	
	dataarray[0] = CLEAN_DATA;
	count++;
	//Cl_SysStat_GetSensor_Status_Query(SENSOR_TEMP3STATUS,&temp);


		Cl_SysStat_GetSensor_Status_Query(SENSOR_TEMP3STATUS,&sensordata);
		{
			tmp3 = (float)sensordata /10;
		}

		Cl_SysStat_GetSensor_Status_Query(SENSOR_COND_STATUS,&temp);
		{
			// manav03 changed
							data.fourbytedata = temp/100;
			 				dataarray[count++] = data.bytedata[0];
			 				dataarray[count++] = data.bytedata[1];
			 				dataarray[count++] = data.bytedata[2];
			 				dataarray[count++] = data.bytedata[3];
//  			
// 			float conductivity_CS3, avg_conductivity,avgcond;
// 			conductivity_CS3 = cl_utilities_CalConductivity(temp,tmp3);
// 			avg_conductivity = (avg_conductivity *9 + conductivity_CS3)/10;
// 			avgcond = avgcond + conductivity_CS3;
// 			
// 			if (count % 20)
// 			{
// 				avgcond = avgcond/20;
// 				
// 				avg_conductivity = avgcond;
// 
// 			}
// 			if (avg_conductivity >0)
// 			{
// 				data.fourbytedata = avg_conductivity;
// 				dataarray[count++] = data.bytedata[0];
// 				dataarray[count++] = data.bytedata[1];
// 				dataarray[count++] = data.bytedata[2];
// 				dataarray[count++] = data.bytedata[3];
// 				}
// 			else
// 			{
// 				data.fourbytedata = 0;
// 				dataarray[count++] = data.bytedata[0];
// 				dataarray[count++] = data.bytedata[1];
// 				dataarray[count++] = data.bytedata[2];
// 				dataarray[count++] = data.bytedata[3];
// 			}
			
		}
			Cl_SysStat_GetSensor_Status_Query(SENSOR_TEMP3STATUS,&sensordata);
			{
				float ftemp,ftemp1,temp1;
				tmp3 = sensordata /10;
				data.fourbytedata = (uint16_t)(tmp3 );
				dataarray[count++] = data.bytedata[0];
				dataarray[count++] = data.bytedata[1];
				dataarray[count++] = data.bytedata[2];
				dataarray[count++] = data.bytedata[3];
			}
		
		// manav02 added
		Cl_SysStat_GetSensor_Status_Query(SENSOR_TEMP2STATUS,&temp2);
		{
			
			data.fourbytedata = (uint16_t)temp2/10 ;
			dataarray[count++] = data.bytedata[0];
			dataarray[count++] = data.bytedata[1];
			dataarray[count++] = data.bytedata[2];
			dataarray[count++] = data.bytedata[3];
		}
		
		Cl_rinseretcode = Cl_SendDatatoconsole(CON_TX_COMMAND_CL_SYS_STATE_DATA,&dataarray,count);
		

}

void Cl_Rinse_100msTMP(void)
{
	
	int16_t Ps1,Ps2,Ps3,Atp,Vtp;
	int16_t sensordata=0;
	int16_t Pressuredata=0;
	int16_t sensordatamillivolts =0;
	int16_t tempdata =0;
	int16_t Pressuredatainmillibars =0,ps1Pressuredatainmillibars=0,ps2Pressuredatainmillibars=0,ps3Pressuredatainmillibars=0,aptPressuredatainmillibars=0,vptPressuredatainmillibars=0;
	Cl_Uint8Type* pdataarray;
	Cl_Uint8Type dataarray[12] =  {0,0,0,0,0,0,0,0,0,0,0,0};
	Datatype data;
	//Datatype temp;
	static Cl_Uint32Type timecount = 0;
	
	static int16_t counter,ps1avg,ps2avg,ps3avg,aptavg,vptavg;
	Cl_ReturnCodeType Cl_rinseretcode = CL_ERROR;
	
	
	timecount++;
//	#if 0
	if (timecount < 20)
	{
		return;
	}
	else
	{
		timecount = 0;
	}
	//#endif
	
	//cl_testgetsensordata(PS1STATUS_HIGH,&Ps1);
	Cl_SysStat_GetSensor_Status_Query(SENSOR_PS1STATUS,&Ps1);
// 	sensordatamillivolts = ((Ps1* 3300 /4096) ) ;
// 	Pressuredatainmillibars =  ((sensordatamillivolts - 860)/0.78 ) ;
// 	if (  sensordatamillivolts < 1360 )
// 	{
// 		Pressuredatainmillibars = Pressuredatainmillibars- 50 ;
// 	}
// 	else if(  sensordatamillivolts < 1650 )
// 	{
// 		Pressuredatainmillibars = Pressuredatainmillibars- 25 ;
// 	}
// 	
// 	ps1Pressuredatainmillibars = Pressuredatainmillibars; // - 1000;
// 	ps1avg = ((ps1avg* 9) + ps1Pressuredatainmillibars) /10;
// 	
// 	//cl_testgetsensordata(PS2STATUS_HIGH,&Ps2);
	Cl_SysStat_GetSensor_Status_Query(SENSOR_PS2STATUS, &Ps2);
// 	sensordatamillivolts = ((Ps2* 3300 /4096) ) ;
// 	Pressuredatainmillibars =  ((sensordatamillivolts - 860)/0.78 ) ;
// 	if (  sensordatamillivolts < 1360 )
// 	{
// 		Pressuredatainmillibars = Pressuredatainmillibars- 50 ;
// 	}
// 	else if(  sensordatamillivolts < 1650 )
// 	{
// 		Pressuredatainmillibars = Pressuredatainmillibars- 25 ;
// 	}
// 	ps2Pressuredatainmillibars = Pressuredatainmillibars;
// 	ps2avg = ((ps2avg* 9) + ps2Pressuredatainmillibars) /10;
	
	Cl_SysStat_GetSensor_Status_Query(SENSOR_PS3STATUS, &Ps3);
// 	sensordatamillivolts = ((Ps3* 3300 /4096) ) ;
// 	//Pressuredatainmillibars =  (sensordatamillivolts/1.485 ) ;
// 	Pressuredatainmillibars = (sensordatamillivolts - 1.1)/1.1;
// 	ps3Pressuredatainmillibars = Pressuredatainmillibars;// - 1000;
// 	
// 	ps3avg = ((ps3avg* 9) + ps3Pressuredatainmillibars) /10;
	//ps3avg = Ps3;
	
	//cl_testgetsensordata(APTSTATUS_HIGH,&Atp);
	Cl_SysStat_GetSensor_Status_Query(SENSOR_APTSTATUS, &Atp);
// 	sensordatamillivolts = ((Atp* 3300 /4096) ) ;
// 	Pressuredatainmillibars = (sensordatamillivolts - 1640)/0.6;
// 	aptPressuredatainmillibars = Pressuredatainmillibars;  // - 1000;
// 	//Atp = Pressuredatainmillibars;
// 	aptavg = (( aptavg *9) + aptPressuredatainmillibars) /10;
// 	//cl_testgetsensordata(VPTSTATUS_HIGH,&Vtp);
	Cl_SysStat_GetSensor_Status_Query(SENSOR_VPTSTATUS, &Vtp);
// 	sensordatamillivolts = ((Vtp* 3300 /4096) ) ;
// 	Pressuredatainmillibars = (sensordatamillivolts - 1640)/0.6;
// 	vptPressuredatainmillibars = Pressuredatainmillibars;  // - 1000;
// 	//Vtp = Pressuredatainmillibars;
// 	vptavg = (( vptavg *9) + vptPressuredatainmillibars) /10;
//  
	
		data.TMP = (vptavg + aptavg)/2 - (ps1avg+ps2avg)/2;
		
		dataarray[0] = CON_TX_PARAM_DATA_PRESSURE_DATA;
		dataarray[1] = data.byte[0];
		dataarray[2] = data.byte[1];
		data.twobyte = ps1Pressuredatainmillibars;
		dataarray[3] = data.byte[0];
		dataarray[4] = data.byte[1];
		data.twobyte = ps2Pressuredatainmillibars;
		dataarray[5] = data.byte[0];
		dataarray[6] = data.byte[1];
		data.twobyte = aptPressuredatainmillibars;
		dataarray[7] = data.byte[0];
		dataarray[8] = data.byte[1];
		data.twobyte = vptPressuredatainmillibars;
		dataarray[9] = data.byte[0];
		dataarray[10] = data.byte[1];
		data.twobyte = ps1avg;
		dataarray[11] = data.byte[0];
		dataarray[12] = data.byte[1];
		data.twobyte = ps2avg;
	//	data.TMP = ps3avg;
		dataarray[13] = data.byte[0];
		dataarray[14] = data.byte[1];
		data.twobyte = aptavg;
		dataarray[15] = data.byte[0];
		dataarray[16] = data.byte[1];
		data.twobyte = vptavg;
		dataarray[17] = data.byte[0];
		dataarray[18] = data.byte[1];
	Cl_rinseretcode = Cl_SendDatatoconsole(CON_TX_COMMAND_SYSDATA,&dataarray,19);
	

	
}
Cl_ReturnCodeType Cl_Rinse_StartRinse(void)
{
				Cl_ReturnCodeType Cl_rinseretcode = CL_OK;
				Cl_ConsoleTxCommandtype command = CON_TX_COMMAND_COMMAND_MAX;
				Cl_Uint8Type dataarray[4] = {0,0,0,0};
				
				Cl_Uint8Type data = 0;
				Cl_Uint16Type cl_rinse_acidstatus = 0, cl_rinse_bicarbstatus =0 , cl_rinse_holder1status = 0 , cl_rinse_holder2status = 0;
				Cl_BoolType Cl_Rinsereaduinessflag = true;
					
				//Check for  rinse readiness and alarm status
				
				//Cl_checkRinseReadyness();

                    Cl_rinseretcode =  Cl_AlarmActivateAlarms(FLOW_LOW_FLOWRATE,false );
					Cl_rinseretcode =  Cl_AlarmActivateAlarms(FLOW_NO_FLOW,false );					
 					Cl_rinseretcode =  Cl_AlarmActivateAlarms(FLOW_HIGH_FLOWRATE,false );   // manav02 changed from low to high


				SetHeaterState(CL_HEATER_STATE_OFF);
				cl_flow_switchcontroller(FLOW_SWITCH_CNTRLER_EVENT_START);
				
				Cl_SysStat_GetSensor_Status_Query(SENSOR_ACID_INLET,&cl_rinse_acidstatus);
				Cl_SysStat_GetSensor_Status_Query(SENSOR_BICARB_INLET,&cl_rinse_bicarbstatus);
				Cl_SysStat_GetSensor_Status_Query(SENSOR_HOLDER1STATUS,&cl_rinse_holder1status);
				Cl_SysStat_GetSensor_Status_Query(SENSOR_HOLDER2STATUS,&cl_rinse_holder2status);
				
				//DD_CAN_WRITE_HEMO(HEMO_START_BD_CALIBRATION, 0);
				
				// below commented for tuv testing holder alarms were present even after hoolders were connected
				 if(cl_rinse_acidstatus == ACID_INLET_OPEN )    // manav03 commented for testing
				 {
					 Cl_RinseSendAlarm(ACID_IN_OPEN);
					 Cl_Rinsereaduinessflag = false;
				 } 
				 if(cl_rinse_bicarbstatus == BICARB_INLET_OPEN )
				 {
					  Cl_RinseSendAlarm(BICARB_IN_OPEN);
					 Cl_Rinsereaduinessflag = false;
				 }	
				 if(cl_rinse_holder1status == CL_HOLDER1OPEN )  //CL_HOLDER1OPEN
				 {
					 Cl_RinseSendAlarm(HOLDER1STATUS_OPEN);
					 Cl_Rinsereaduinessflag = false;
				 }
				 if(cl_rinse_holder2status == CL_HOLDER2OPEN )// CL_HOLDER1OPEN
				 {
					 Cl_RinseSendAlarm(HOLDER2STATUS_OPEN);
					 Cl_Rinsereaduinessflag = false;
				 }				 
				 if(Cl_Rinsereaduinessflag == false)
				 {
					// Cl_Rinsereaduinessflag = true;
					 return CL_REJECTED;
				 }
				 
				sv_cntrl_resetgreenalarm();
// 				sv_cntrl_resetyellowalarm();
// 				sv_cntrl_resetredalarm();
 				sv_cntrl_nobuzzer();
// 				

	sv_cntrl_resetredalarm();
//	sv_cntrl_setgreenalarm();
				Cl_RinseFlowOn();
				//sv_prop_startpropeo_aligning();
				sv_cntrl_activate_valve(VALVE_ID2);
				sv_cntrl_activate_valve(VALVE_ID3);
				//	Cl_rinseretcode = sv_cntrl_setflowpath(BC_FLUSH_OPEN); // sanjeer BC_NEW
				
				Cl_rinseretcode = sv_cntrl_setpumpspeed(DCMOTOR2,1000);   // 900
				Cl_rinseretcode = sv_cntrl_setpumpspeed(DCMOTOR1,1100);
				Cl_rinseretcode =  sv_cntrl_activatepump(DCMOTOR1);
				Cl_rinseretcode =  sv_cntrl_activatepump(DCMOTOR2);

				Cl_Uint16Type potvalue = 0;
				potvalue = (900* 1024)/10000;                                 // 600
				sv_cs_setpotvalue(potvalue);
				
				Cl_Uint32Type temp = Treatdata[ID_dflow];
				flow_rate = temp;
				
				cl_uf_controller(CL_UF_EVENT_SET_UF_RATE,16000);
					cl_wait(100);
					cl_uf_controller(CL_UF_EVENT_START,0);
					cl_ufpumpFeedback_start();
					cl_wait(100);
					cl_uf_controller(CL_UF_EVENT_SET_UF_RATE,12000);
					cl_wait(100);
					cl_uf_controller(CL_UF_EVENT_SET_UF_RATE,8000);
					cl_wait(100);
					cl_uf_controller(CL_UF_EVENT_SET_UF_RATE,6000);
					cl_wait(100);
					cl_uf_controller(CL_UF_EVENT_SET_UF_RATE,2000);
					cl_wait(100);
					cl_uf_controller(CL_UF_EVENT_SET_UF_RATE,500);
					
// 				sv_cntrl_setpumpspeed(UFPUMP,4000);
// 				cl_wait(100);
// 				sv_cntrl_activatepump(UFPUMP);
// 				cl_ufpumpFeedback_start();
// 				cl_wait(100);
// 				sv_cntrl_setpumpspeed(UFPUMP,2000);
// 				cl_wait(100);
// 				sv_cntrl_setpumpspeed(UFPUMP,1000);
// 				cl_wait(100);
// 				sv_cntrl_setpumpspeed(UFPUMP,150);
// 				cl_wait(100);
// 				
				cl_uf_controller(CL_UF_EVENT_START,0);
				Cl_rinseretcode =  cl_deaeration_controller(CL_DEAER_EVENT_ACTIVATE);
				
				Cl_rinseretcode = Cl_bc_controller(BC_EVENT_START);
		
				sv_prop_startopenfill(); 
							//	sv_cntrl_deactivatepump(DCMOTOR1 );
							//	sv_cntrl_deactivatepump(DCMOTOR2 );
				if(Cl_rinseretcode == CL_OK)
				{
					

					//Reset the  OPENFILL time count
					Cl_RinseMinutescounter = 0;
					Cl_RinseTotalMinutescounter = 0;
					Cl_RinseOpenFillTimeOut = false;

					

					sv_cntrl_activatevenousclamp();
					Cl_rinseretcode =  cl_flow_switchcontroller(FLOW_SWITCH_CNTRLER_EVENT_START);
					//Cl_Rinse_ResetAlertsforReassertion();
					Cl_rinseretcode =  Cl_AlarmConfigureAlarmType(BLOODDOOR_STATUS_OPEN,LOGIC_HIGH,0,0,0);	
					Cl_rinseretcode =  Cl_AlarmConfigureAlarmType(ACID_IN_OPEN,LOGIC_LOW,0,0,0);	
					Cl_rinseretcode =  Cl_AlarmConfigureAlarmType(BICARB_IN_OPEN,LOGIC_LOW,0,0,0);			
					Cl_rinseretcode =  Cl_AlarmConfigureAlarmType(HOLDER1STATUS_OPEN,LOGIC_HIGH,0,0,0);
					Cl_rinseretcode =  Cl_AlarmConfigureAlarmType(HOLDER2STATUS_OPEN,LOGIC_HIGH,0,0,0);
					Cl_rinseretcode =  Cl_AlarmConfigureAlarmType(LEVELSWITCH_OFF_TO_ON,LOGIC_LOW,0,0,0);
					Cl_rinseretcode =  Cl_AlarmConfigureAlarmType(LEVELSWITCH_ON_TO_OFF,LOGIC_HIGH,0,0,0);
					Cl_rinseretcode =  Cl_AlarmConfigureAlarmType(BC_ALARM,0,0,0,0);
					
					Cl_rinseretcode =  Cl_AlarmConfigureAlarmType(POWER_FAILURE,LOGIC_HIGH,0,0,0);
					Cl_rinseretcode =  Cl_AlarmConfigureAlarmType(FLOW_LOW_FLOWRATE,0,0,0,0);
					Cl_rinseretcode =  Cl_AlarmConfigureAlarmType(FLOW_NO_FLOW,0,0,0,0);
					Cl_rinseretcode =  Cl_AlarmConfigureAlarmType(FLOW_HIGH_FLOWRATE,0,1500,0,0);
					Cl_rinseretcode =  Cl_AlarmConfigureAlarmType(TEMP1_HIGH_THRESHOLD,0,370,150,0);
					Cl_rinseretcode =  Cl_AlarmConfigureAlarmType(TEMP1_LOW_THRESHOLD,0,150,150,0);
					Cl_rinseretcode =  Cl_AlarmConfigureAlarmType(TEMP2_HIGH_THRESHOLD,0,850,150,0);
					Cl_rinseretcode =  Cl_AlarmConfigureAlarmType(TEMP2_LOW_THRESHOLD,0,150,150,0);
					//Cl_rinseretcode =  Cl_AlarmActivateAlarms(ACID_IN,true );
					//Cl_rinseretcode =  Cl_AlarmActivateAlarms(BICARB_IN,true );
					//Cl_rinseretcode =  Cl_AlarmActivateAlarms(BLOODDOOR_STATUS_OPEN,true);
					//Cl_rinseretcode =  Cl_AlarmActivateAlarms(HOLDER1STATUS_OPEN,true );
					//Cl_rinseretcode =  Cl_AlarmActivateAlarms(HOLDER2STATUS_OPEN,true );
				
				   Cl_rinseretcode =  Cl_AlarmActivateAlarms(POWER_FAILURE,true);          //manav03 disabled for testing
					Cl_rinseretcode =  Cl_AlarmActivateAlarms(LEVELSWITCH_OFF_TO_ON,true );
					Cl_rinseretcode =  Cl_AlarmActivateAlarms(LEVELSWITCH_ON_TO_OFF,true );					
				//	Cl_rinseretcode =  Cl_AlarmActivateAlarms(FLOW_LOW_FLOWRATE,true );
// 					Cl_rinseretcode =  Cl_AlarmActivateAlarms(FLOW_NO_FLOW,true );					
// 					Cl_rinseretcode =  Cl_AlarmActivateAlarms(FLOW_LOW_FLOWRATE,true );
// 					Cl_rinseretcode =  Cl_AlarmActivateAlarms(FLOW_HIGH_FLOWRATE,true );
 					Cl_rinseretcode =  Cl_AlarmActivateAlarms(ACID_IN_OPEN,true );       // manav03 alarm removed for testing
					Cl_rinseretcode =  Cl_AlarmActivateAlarms(BICARB_IN_OPEN,true ); 
					Cl_rinseretcode =  Cl_AlarmActivateAlarms(HOLDER1STATUS_OPEN,true ); // manav03 alarm removed for testing
					Cl_rinseretcode =  Cl_AlarmActivateAlarms(HOLDER2STATUS_OPEN,true ); // manav03 alarm removed for testing
					//Cl_rinseretcode =  Cl_AlarmActivateAlarms(BC_ALARM,true);
					
				//	Cl_rinseretcode =  Cl_AlarmActivateAlarms(COND_STATUS_HIGH,true );
					//Cl_rinseretcode =  Cl_AlarmActivateAlarms(COND_STATUS_LOW,true );
					//Cl_rinseretcode =  Cl_AlarmActivateAlarms(PS2_HIGH_THRESHOLD,true );
					//Cl_rinseretcode =  Cl_AlarmActivateAlarms(PS1_HIGH_THRESHOLD,true );
					//Cl_rinseretcode =  Cl_AlarmActivateAlarms(TEMP3_HIGH_THRESHOLD,true );
					//Cl_rinseretcode =  Cl_AlarmActivateAlarms(TEMP3_LOW_THRESHOLD,true );
					Cl_rinseretcode =  Cl_AlarmActivateAlarms(TEMP2_HIGH_THRESHOLD,true );
					Cl_rinseretcode =  Cl_AlarmActivateAlarms(TEMP2_LOW_THRESHOLD,true );
					Cl_rinseretcode =  Cl_AlarmActivateAlarms(TEMP1_LOW_THRESHOLD,true );
					Cl_rinseretcode =  Cl_AlarmActivateAlarms(TEMP1_HIGH_THRESHOLD,true );
					Cl_rinseretcode =  Cl_AlarmActivateAlarms(BP_STOPPED,true );
					
					//Cl_rinseretcode =  Cl_AlarmActivateAlarms(PS1STATUS_HIGH,true );
					//Cl_rinseretcode =  Cl_AlarmActivateAlarms(PS2STATUS_HIGH,true );
					
					//Cl_rinseretcode =  Cl_AlarmActivateAlarms(PS3STATUS_HIGH,true );
					//Cl_rinseretcode =  Cl_AlarmActivateAlarms(COND_STATUS_HIGH,true );
					//Cl_rinseretcode =  Cl_AlarmActivateAlarms(COND_STATUS_LOW,true );
					//Cl_rinseretcode =  Cl_AlarmActivateAlarms(COND_DAC_OPEN,true );
					//Cl_rinseretcode =  Cl_AlarmActivateAlarms(COND_DAC_RO,true );
					//Cl_rinseretcode =  Cl_AlarmActivateAlarms(COND_DAC_HIGH,true );
					

					
					if(Current_sense_trigger)
					{
						Cl_rinseretcode =  Cl_AlarmActivateAlarms( FPCURRENTSTATUS,true );
					}
					else
					{
						//			Cl_rinseretcode =  Cl_AlarmActivateAlarms( PS3STATUS_HIGH,true );
					}
											
					Cl_rinseretcode = Cl_Rinse_ResetAlertsforReassertion();
					//Cl_rinseretcode = SetHeaterState(CL_HEATER_STATE_START);
					

					
				}
					return CL_OK;
}

Cl_ReturnCodeType Cl_Rinse_DisinfStop(void){
	
	
	Cl_ReturnCodeType Cl_rinseretcode = CL_OK;
	Cl_Uint8Type data;
	
	sv_cntrl_setredalarm();
	
	Cl_rinseretcode =  Cl_AlarmActivateAlarms(BLOODDOOR_STATUS_OPEN,false );
	//Cl_rinseretcode =  Cl_AlarmActivateAlarms(HOLDER1STATUS_OPEN,true );
	//Cl_rinseretcode =  Cl_AlarmActivateAlarms(HOLDER2STATUS_OPEN,true );
	Cl_rinseretcode =  Cl_AlarmActivateAlarms(LEVELSWITCH_OFF_TO_ON,false );
	Cl_rinseretcode =  Cl_AlarmActivateAlarms(LEVELSWITCH_ON_TO_OFF,false );
	Cl_rinseretcode =  Cl_AlarmActivateAlarms(FLOW_LOW_FLOWRATE,false );
	Cl_rinseretcode =  Cl_AlarmActivateAlarms(FLOW_NO_FLOW,false );
	Cl_rinseretcode =  Cl_AlarmActivateAlarms(FLOW_LOW_FLOWRATE,false );
	// 					Cl_rinseretcode =  Cl_AlarmActivateAlarms(FLOW_HIGH_FLOWRATE,true );
	Cl_rinseretcode =  Cl_AlarmActivateAlarms(ACID_IN_OPEN,false );
	Cl_rinseretcode =  Cl_AlarmActivateAlarms(BICARB_IN_OPEN,false );
	Cl_rinseretcode =  Cl_AlarmActivateAlarms(HOLDER1STATUS_OPEN,false );
	Cl_rinseretcode =  Cl_AlarmActivateAlarms(HOLDER2STATUS_OPEN,false );
	Cl_rinseretcode =  Cl_AlarmActivateAlarms(BC_ALARM,false);
	
	Cl_Rinse_ResetAlertsforReassertion();
	
	if(!((cl_rinsestate == CL_RINSE_STATE_IDLE ) || (cl_rinsestate == CL_RINSE_STATE_STOPPED ) ||(cl_rinsestate == CL_RINSE_STATE_CRITICAL_ALARM )  ))
	{
		
		// 					if ((flag_hotcitric == 1) ||(flag_na == 1)||(flag_citric == 1)){
		//
		// 						sv_cntrl_activatevenousclamp();
		// 						Cl_Rinsesecondscounter = 0;
		// 						Cl_RinseMinutescounter = 0;
		// 						Cl_RinseTotalMinutescounter = 0;
		// 						Cl_RinsesTotalhourscounter = 0 ;
		// 						Cl_bc_controller(BC_EVENT_STOP);
		// 						Cl_bc_controller(BC_EVENT_STOP_RINSE);
		// 						cl_uf_controller(CL_UF_EVENT_STOP,0);
		// 						sv_prop_stopmixing();
		// 						cl_rinsestate = CL_RINSE_STATE_IDLE;
		// 						Cl_rinseretcode = Cl_SendDatatoconsole(CON_TX_COMMAND_DISINF_STOPPED,&data,0);
		//
		// 					}
		// 					else{
		//
		
		//	Cl_SendDatatoconsole(CON_TX_COMMAND_PRINTTEXT,"DOOR",4);
		sv_cntrl_activatevenousclamp();
		Cl_Rinsesecondscounter = 0;
		Cl_RinseMinutescounter = 0;
		Cl_RinseTotalMinutescounter = 0;
		Cl_RinsesTotalhourscounter = 0 ;
		//Cl_rinseretcode = sv_cntrl_poweroffheater();
		Cl_bc_controller(BC_EVENT_STOP);
		Cl_bc_controller(BC_EVENT_STOP_RINSE);
		cl_uf_controller(CL_UF_EVENT_STOP,0);
		sv_prop_stopmixing();
		//Cl_rinseretcode = sv_cntrl_setflowpath(FLOW_PATH_IDLE_RINSE);
		cl_rinsestate = CL_RINSE_STATE_IDLE;
		Cl_rinseretcode = Cl_SendDatatoconsole(CON_TX_COMMAND_DISINF_STOPPED,&data,0);
	
		}
}

Cl_ReturnCodeType Cl_Rinse_StopRinse(void)
{
				Cl_ReturnCodeType Cl_rinseretcode = CL_OK;
				Cl_Uint8Type data;
				
				sv_cntrl_setredalarm();
				
				Cl_rinseretcode =  Cl_AlarmActivateAlarms(BLOODDOOR_STATUS_OPEN,false );
				//Cl_rinseretcode =  Cl_AlarmActivateAlarms(HOLDER1STATUS_OPEN,true );
				//Cl_rinseretcode =  Cl_AlarmActivateAlarms(HOLDER2STATUS_OPEN,true );
				Cl_rinseretcode =  Cl_AlarmActivateAlarms(LEVELSWITCH_OFF_TO_ON,false );
				Cl_rinseretcode =  Cl_AlarmActivateAlarms(LEVELSWITCH_ON_TO_OFF,false );
					Cl_rinseretcode =  Cl_AlarmActivateAlarms(FLOW_LOW_FLOWRATE,false );
				 					Cl_rinseretcode =  Cl_AlarmActivateAlarms(FLOW_NO_FLOW,false );
				 					Cl_rinseretcode =  Cl_AlarmActivateAlarms(FLOW_LOW_FLOWRATE,false );
				// 					Cl_rinseretcode =  Cl_AlarmActivateAlarms(FLOW_HIGH_FLOWRATE,true );
				Cl_rinseretcode =  Cl_AlarmActivateAlarms(ACID_IN_OPEN,false );
				Cl_rinseretcode =  Cl_AlarmActivateAlarms(BICARB_IN_OPEN,false );
				Cl_rinseretcode =  Cl_AlarmActivateAlarms(HOLDER1STATUS_OPEN,false );
				Cl_rinseretcode =  Cl_AlarmActivateAlarms(HOLDER2STATUS_OPEN,false );
				Cl_rinseretcode =  Cl_AlarmActivateAlarms(BC_ALARM,false);
				
				Cl_Rinse_ResetAlertsforReassertion();
				
				if(!((cl_rinsestate == CL_RINSE_STATE_IDLE ) || (cl_rinsestate == CL_RINSE_STATE_STOPPED ) ||(cl_rinsestate == CL_RINSE_STATE_CRITICAL_ALARM )  ))
				{
					
// 					if ((flag_hotcitric == 1) ||(flag_na == 1)||(flag_citric == 1)){
// 						
// 						sv_cntrl_activatevenousclamp();
// 						Cl_Rinsesecondscounter = 0;
// 						Cl_RinseMinutescounter = 0;
// 						Cl_RinseTotalMinutescounter = 0;
// 						Cl_RinsesTotalhourscounter = 0 ;
// 						Cl_bc_controller(BC_EVENT_STOP);
// 						Cl_bc_controller(BC_EVENT_STOP_RINSE);
// 						cl_uf_controller(CL_UF_EVENT_STOP,0);
// 						sv_prop_stopmixing();
// 						cl_rinsestate = CL_RINSE_STATE_IDLE;
// 						Cl_rinseretcode = Cl_SendDatatoconsole(CON_TX_COMMAND_DISINF_STOPPED,&data,0);
// 						
// 					}
// 					else{
// 						
					
				//	Cl_SendDatatoconsole(CON_TX_COMMAND_PRINTTEXT,"DOOR",4);
				sv_cntrl_activatevenousclamp();
				Cl_Rinsesecondscounter = 0;
				Cl_RinseMinutescounter = 0;
				Cl_RinseTotalMinutescounter = 0;
				Cl_RinsesTotalhourscounter = 0 ;
				//Cl_rinseretcode = sv_cntrl_poweroffheater();
				Cl_bc_controller(BC_EVENT_STOP);
				Cl_bc_controller(BC_EVENT_STOP_RINSE);
				cl_uf_controller(CL_UF_EVENT_STOP,0);
				sv_prop_stopmixing();
				//Cl_rinseretcode = sv_cntrl_setflowpath(FLOW_PATH_IDLE_RINSE);
				cl_rinsestate = CL_RINSE_STATE_IDLE;
				Cl_rinseretcode = Cl_SendDatatoconsole(CON_TX_COMMAND_RINSE_STOPPED,&data,0);
//					}
					
				}
			
				return Cl_rinseretcode;
}


Cl_ReturnCodeType UpdateRinseMinuteTick(void)
{
					Cl_ReturnCodeType Cl_rinseretcode = CL_OK;
					Cl_Rinsesecondscounter = 0;
					Cl_RinseMinutescounter++;
					Cl_RinseTotalMinutescounter++;
					Cl_rinseretcode = Cl_Rinse_UpdateTimeInfo();
					return Cl_rinseretcode;
}
