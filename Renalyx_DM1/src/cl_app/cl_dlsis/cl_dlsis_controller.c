/*
 * cl_dlsis_controller.c
 *
 * Created: 12/27/2013 12:26:22 PM
 *  Author: user
 */ 

#include "cl_dlsiscontroller_include.h"
#include "cl_dlsiscontroller_proto.h"
#include "sv_interface_fun.h"

extern Cl_Uint32Type Treatdata[ID_MAX_TREAT_PARAM] ;
extern Cl_ReturnCodeType UpdateHeaterControls(void);

typedef union {
	Cl_Uint32Type fourbytedata;
	int16_t twobytedata;
	Cl_Uint8Type bytedata[4] ;
}StateDatatype;
//extern Cl_ReturnCodeType Cl_Heater_GetHeaterStableStatus(Cl_BoolType* );
//extern Cl_ReturnCodeType Cl_mac_apprequesthandler(Cl_Mac_EventsType);
extern void calibration_apt(int16_t sensordata);

extern Cl_AlarmThresholdType  Cl_alarmThresholdTable;
extern Cl_Sys_statusType cl_sys_statbuffer;
extern Cl_ConsoleMsgType Cl_ConsoleRxMsg;
extern Cl_BoolType BC_window; //test
extern Cl_BoolType g_testbcfreeze; //test
extern Cl_BoolType Current_sense_trigger; // test
extern float dummy3;
extern  Cl_Dlsis_States cl_dlsis_state_dummy;
Cl_Dlsis_States cl_dlsis_state = CL_DLSIS_STATE_IDLE;
volatile Cl_Uint8Type error_flag=0;
volatile Cl_Uint8Type flag_bc = 1;
 DlsisTimeType DlsisTime ;
 extern float apt;
extern float vpt;
extern float ufRemoved;
extern volatile Cl_Uint32Type hep_speed;
 int Cl_dlsissecondscounter_1 =0 ,Cl_dlsisMinutescounter_1=0, Cl_dlsishourscounter_1=0;
 int Cl_dlsisTotalMinutescounter_1=0, Cl_dlsisTotalhourscounter_1=0;
Cl_BoolType Cl_dlsisOpenFillTimeOut = false, heater_stable_status = false;
 Cl_Uint16Type dlis_wait_cnt;
Cl_NewAlarmIdType dlsisalarm;
Cl_NewAlarmIdType dlsisalarm1;
 Cl_Uint16Type wait_cnt_dls = 0,TmpVal_dls =0;
 int16_t temp1_dls ,temp2_dls;
 Cl_Uint8Type bypass_flag=0;
extern volatile float temprature_final_value_1,temprature_final_value_2,temprature_final_value_3;
extern volatile float pressure_final_apt,pressure_final_vpt,pressure_final_ps1,pressure_final_ps2,pressure_final_ps3;
extern volatile float cond_final_cs3;
extern float res_temp_value;

volatile Cl_Uint8Type bc_flag=0;
volatile Cl_Uint32Type flow_rate=0;
float dlsis_ps1 = 0, dlsis_ps2=0, dlsis_ps3=0,dlsis_apt = 0, dlsis_vpt=0;
float dlsis_temp1=0,dlsis_temp2=0,dlsis_temp3=0,dlsis_cond=0;


 extern Cl_ReturnCodeType  sv_nvmgetdata(Cl_Uint8Type,Cl_Uint8Type*);
 extern Cl_ReturnCodeType cl_wait(Cl_Uint32Type );
 extern Cl_ConsoleMsgType Cl_ConsoleRxMsg;
 extern Cl_ReturnCodeType  sv_nvmgetdata(Cl_Uint8Type,Cl_Uint8Type*);
 extern Cl_ReturnCodeType  sv_nvmsetdata(Cl_Uint8Type ,Cl_Uint8Type* ,Cl_Uint8Type datasize);

static float avgcond=0;
uint8_t averaging_count=0;
extern bool isouf;
extern float cl_utilities_CalConductivity(uint16_t ,float);
int cl_dlsis_powerfailure = 0;
Cl_Uint16Type dlsis_powerFailSensorData = 0;

int bpflagCounter = 0 , bpFlag = 0;

DlsisAlarmsType Cl_DlsisAlarmTable[CL_DLSIS_ALRM_MAX] = 
{
{HOLDER1STATUS_CLOSED,CL_ALARM_ALARM,false,false,false},
{HOLDER2STATUS_CLOSED,CL_ALARM_ALARM,false,false,false},		
{ACID_IN_CLOSED,CL_ALARM_ALARM,false,false,false},	
{BICARB_IN_CLOSED,CL_ALARM_ALARM,false,false,false},	
{FLOW_NO_FLOW,CL_ALARM_ALARM,false,false,false},	
{FLOW_LOW_FLOWRATE,CL_ALARM_ALARM,false,false,false},
{FLOW_HIGH_FLOWRATE,CL_ALARM_ALARM,false,false,false},          // manav02 added	
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
{POWER_FAILURE,CL_ALARM_ALARM,false,false,false},       // manav02 added
{BP_STOPPED,CL_ALARM_ALARM,false,false,false},
{TMP_HIGH,CL_ALARM_ALARM,false,false,false},
{TMP_LOW,CL_ALARM_ALARM,false,false,false},
{BLOOD_NOT_DETECTED,CL_ALARM_ALARM,false,false,false},
{BC_ALARM,CL_ALARM_ALARM,false,false,false},
//{BLOOD_NOT_DETECTED , CL_ALARM_ALARM,false,false,false },
{HP_UNDERRUN , CL_ALARM_ALARM,false,false,false },
{HP_OVERRUN , CL_ALARM_ALARM,false,false,false },			
{BP_UNDERRUN , CL_ALARM_ALARM,false,false,false },
{BP_OVERRUN , CL_ALARM_ALARM,false,false,false }	
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
volatile Cl_Uint8Type var=0;
 Cl_ReturnCodeType Cl_dlsis_init(void)
 {
  Cl_dlsissecondscounter_1 =0;Cl_dlsisMinutescounter_1=0; Cl_dlsishourscounter_1=0;
  Cl_dlsisTotalMinutescounter_1=0; Cl_dlsisTotalhourscounter_1=0;
 Cl_dlsisOpenFillTimeOut = false; heater_stable_status = false;
  dlis_wait_cnt =0;

  wait_cnt_dls = 0;TmpVal_dls =0;
  temp1_dls =0 ;temp2_dls =0;
  bypass_flag=0;
   bc_flag=1;
  flow_rate=0;
  cl_dlsis_powerfailure = 0;
 dlsis_ps1 = 0; dlsis_ps2=0; dlsis_ps3=0;dlsis_apt = 0; dlsis_vpt=0;
 dlsis_temp1=0;dlsis_temp2=0;dlsis_temp3=0;dlsis_cond=0;
 
	 return CL_OK;
 }
 
 
Cl_ReturnCodeType Cl_Dlsis_controller(Cl_Mac_EventsType Cl_MacDlsisEvent)
{
	
Cl_ReturnCodeType Cl_dlsisretcode;
Cl_Dlsis_Eevents cl_dlsis_event = EVENT_NULL_EVENT;
Cl_BoolType alarmstatus1 = false,alarmstatus2 = false,alarmstatus3 = false, flowstatus = false;
static Cl_BoolType bcs_wait = false;
static Cl_BoolType V2_toggle = false;
Cl_Uint16Type Tmp2Val = 0;
Cl_Uint8Type dataarray[8] =  {0,0,0,0};
extern Cl_BoolType BC_window; //test
extern Cl_BoolType g_testbcfreeze; //test
extern Cl_BoolType Current_sense_trigger; // test
Cl_ConsoleTxCommandtype command = CON_TX_COMMAND_COMMAND_MAX;
Cl_Uint8Type data = 0;
Cl_Uint16Type tempcount=0;
float temp;
//static float apt=0, vpt=0;
StateDatatype abfdata;
Cl_Uint8Type abfdataarray[5] =  {0,0,0,0,0};


Cl_dlsisretcode =  cl_dlsis_translatemacevent( Cl_MacDlsisEvent, &cl_dlsis_event);
	
	switch(cl_dlsis_event)
	{
		case EVENT_DLSIS_TICK_SECOND:
		Cl_dlsisretcode = Cl_bc_controller(BC_EVENT_SECOND);
		break;
		case EVENT_DLSIS_TICK_50MS:
		//Cl_dlsisretcode = Cl_bc_controller(BC_EVENT_50MS);
		break;
		
		
		case EVENT_DLSIS_HEAPARIN_BOLUS:
		cl_hep_controller(CL_HEP_EVENT_STARTBOLUS,0);
		break;
		
		case EVENT_DLSIS_MUTE:
		sv_cntrl_nobuzzer();
		break;
		
		case EVENT_DLSIS_HEP_RATE:
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
		case EVENT_DLSIS_HEP_ON:
		
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
									Cl_AlarmActivateAlarms(HP_OVERRUN,true);
									Cl_AlarmActivateAlarms(HP_UNDERRUN,true);
			}
			
		}
		
		
		break;
		case EVENT_DLSIS_HEP_OFF:
									Cl_AlarmActivateAlarms(HP_OVERRUN,false);
									Cl_AlarmActivateAlarms(HP_UNDERRUN,false);
			cl_hep_controller(CL_HEP_EVENT_STOP_NORMAL_DELIVERY,0);
		break;
		case EVENT_DLSIS_COMMAND_SET_BLDPMP_ON:
		cl_bp_controller(CL_BP_EVENT_START,0);
		Cl_dlsisretcode =  Cl_AlarmActivateAlarms(BLOODDOOR_STATUS_OPEN,true);
		bpFlag = 0 ;
		bpflagCounter = 0;
		break;
		case EVENT_DLSIS_COMMAND_SET_BLDPMP_OFF:
		cl_bp_controller(CL_BP_EVENT_STOP,0);
		
		if(cl_dlsis_state == CL_DLSIS_STATE_DIALYSIS){
			Cl_Alarm_TriggerAlarm(BP_STOPPED,1);
		
		}
		Cl_dlsisretcode =  Cl_AlarmActivateAlarms(BP_OVERRUN,false );
		Cl_dlsisretcode =  Cl_AlarmActivateAlarms(BP_UNDERRUN,false );
		bpFlag = 0 ;
		bpflagCounter = 0;
		
		Cl_dlsisretcode =  Cl_AlarmActivateAlarms(BLOODDOOR_STATUS_OPEN,false);
		break;
		case EVENT_DLSIS_COMMAND_SET_BLDPUMPRATE:
		if(Cl_ConsoleRxMsg.msgready == true)
		{
			
			cl_Datastreamtype cl_temp  ;
			cl_temp.bytearray[0] = Cl_ConsoleRxMsg.data.byte[0];
			cl_temp.bytearray[1] = Cl_ConsoleRxMsg.data.byte[1];
			cl_temp.bytearray[2] = Cl_ConsoleRxMsg.data.byte[2];
			cl_temp.bytearray[3] = Cl_ConsoleRxMsg.data.byte[3];
			cl_bp_controller(CL_BP_EVENT_SET_BP_RATE,cl_temp.Twobyte);
			Treatdata[ID_bloodratetreat] = cl_temp.Twobyte;
			Cl_dlsisretcode =  Cl_AlarmActivateAlarms(BP_OVERRUN,false );
			Cl_dlsisretcode =  Cl_AlarmActivateAlarms(BP_UNDERRUN,false );
		bpFlag = 0 ;
		bpflagCounter = 0;	
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
			
			cl_dlsis_event == EVENT_NULL_EVENT;
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
			
			cl_dlsis_event == EVENT_NULL_EVENT;
			break;
			
		
		
		default:break;
		
	}
	if(cl_dlsis_event == EVENT_DLSIS_TICK_SECOND)
	{

		//Cl_dlsisretcode =  Cl_AlarmResetAlarm( TEMP3_HIGH_THRESHOLD);
		//Cl_dlsisretcode =  Cl_AlarmResetAlarm( TEMP3_LOW_THRESHOLD);
		//Cl_dlsisretcode =  Cl_AlarmResetAlarm( TEMP2_HIGH_THRESHOLD);
		Cl_dlsisretcode =  Cl_AlarmResetAlarm( FLOW_NO_FLOW );
		Cl_dlsisretcode =  Cl_AlarmResetAlarm( FLOW_HIGH_FLOWRATE );
		Cl_dlsisretcode =  Cl_AlarmResetAlarm( FLOW_LOW_FLOWRATE );
		
		cl_dlsis_state_dummy = cl_dlsis_state;
	}


	switch(cl_dlsis_state)
	{

		case CL_DLSIS_STATE_INIT:
		break;
		case CL_DLSIS_STATE_IDLE:
		switch (cl_dlsis_event)
		{
/*
			case EVENT_DLSIS_GET_DATA:

				if(Cl_ConsoleRxMsg.msgready == true)
				{
					switch(Cl_ConsoleRxMsg.data.byte[0])
					{
						case	CON_RX_PARAM_DATA_DIALYSIS_STATUS:
						command = CON_TX_COMMAND_SYSDATA;
						dataarray[0] = CON_TX_PARAM_DATA_DIALYSIS_STATUS;
						Cl_dlsisretcode = (Cl_Uint8Type)sv_nvmgetdata(NV_NVM_DIALYSIS_STATUS, &dataarray[1]);
						Cl_dlsisretcode = Cl_SendDatatoconsole(command,&dataarray,2);
						break;

							
						default:
						break;
					}
				}
			break;
			case EVENT_DLSIS_SET_DATA:
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
			
			
			break;
			*/
			case EVENT_DLSIS_START_DIALYSIS:
					Cl_Dlsis_StartDialysis();
			break;
			case EVENT_DLSIS_ALARM:
					//Cl_dlsisretcode = Cl_Dlsis_ProcessAlarms();
			break;
			default:
			break;
		}
		break;
		case CL_DLSIS_STATE_WAIT_FOR_TEMP_COND_STABILITY:
		
			switch(cl_dlsis_event)
			{
				
				case EVENT_DLSIS_STOP_DIALYSIS:                   // manav02 dialysis stopped changed
				sv_cntrl_enable_bypass();
				Cl_Dlsis_StopDialysis();
				cl_dlsis_state = CL_DLSIS_STATE_STOPPED;
				break;
				
				case EVENT_DLSIS_TICK_500MS:
				
					
					Cl_Heater_GetHeaterStableStatus(&heater_stable_status);
					Cl_Uint16Type temp = 0;
					//Cl_SysStat_GetSensor_Status_Query(SENSOR_COND_STATUS , &temp);	
				//	if ((heater_stable_status) &&  (temp < 14) && (temp > 13.3))
					if( (temp == 0))
					{
						
					//	sv_cntrl_disable_bypass();
				bypass_flag = 0;
				cl_dlsis_state = CL_DLSIS_STATE_WAIT_FOR_BLOOD_DETECT;
				
					}
					else{
						Cl_Uint8Type data[3] ={0, 0};
						data[0] = (Cl_Uint8Type)1;
						data[1] = (Cl_Uint8Type) CL_ALARM_TRIGGERED;
						data[2] = (Cl_Uint8Type)CRITICAL;
						command = CON_TX_COMMAND_ALERT;
						 cl_wait(10);
						 Cl_dlsisretcode = Cl_SendDatatoconsole(command,&data,3);
						sv_cntrl_enable_bypass();
						cl_uf_controller(CL_UF_EVENT_STOP,0);
						
					}
				break;
				
				
				default:break;
			}

		break;
		case CL_DLSIS_STATE_WAIT_FOR_BLOOD_DETECT:
		
		switch(cl_dlsis_event)
		{
			
			case EVENT_DLSIS_STOP_DIALYSIS:                   // manav02 dialysis stopped changed
			sv_cntrl_enable_bypass();
			Cl_Dlsis_StopDialysis();
			cl_dlsis_state = CL_DLSIS_STATE_STOPPED;
			break;
			
			case EVENT_DLSIS_TICK_500MS:
					{
						Cl_Uint16Type temp = 0;
					
					Cl_SysStat_GetSensor_Status_Query(SENSOR_BDSTATUS , &temp);
					if( temp == 1)  // need to change as per the blood detector status // manav02 changed to 1
					{
						
						Cl_SendDatatoconsole(CON_TX_COMMAND_PRINTTEXT,"BLOOD_DL",8);

         					Cl_Uint32Type temp1 = Treatdata[ID_heprate];
							cl_hep_controller(CL_HEP_EVENT_START_NORMAL_DELIVERY,temp1);
						
						sv_cntrl_disable_bypass();
					//	Cl_Uint32Type temp = Treatdata[ID_ufgoal];
					//	Cl_Uint32Type temptime = Treatdata[ID_treattime]; // added to get the dialysis duration
			
						cl_uf_controller(CL_UF_EVENT_START,0);
						//cl_wait(100);
						//cl_uf_controller(CL_UF_EVENT_SET_UF_RATE,10000); // commented no need to set uf rate here
						//cl_ufpumpFeedback_start(); 
			
						// calculate rate based on dialysis duration and goal
				//		Cl_Uint32Type tempUfRate = temp/temptime; 
			
						// based on this uf rate calculate the PWM
			
			
			// below logic needs to be changed lookup table needs to be implemented between uf rate and pwm
// 						switch (temp)
// 						{
// 							case 0 : cl_uf_controller(CL_UF_EVENT_SET_UF_RATE,0); // added when uf goal is 0
// 							break;
// 							case 60:
// 								cl_uf_controller(CL_UF_EVENT_SET_UF_RATE,35000);  // tempUfRate and pwn need to be defined
// 							break;
// 							case 90:
// 								cl_uf_controller(CL_UF_EVENT_SET_UF_RATE,25000);
// 							break;
// 							case 120:
// 								cl_uf_controller(CL_UF_EVENT_SET_UF_RATE,20000);
// 							break;
// 							default:
// 								cl_uf_controller(CL_UF_EVENT_SET_UF_RATE,8000);
// 							break;
// 						}
// 						
						/*cl_wait(200);
						cl_uf_controller(CL_UF_EVENT_SET_UF_RATE,5000);
						cl_wait(600);
						cl_uf_controller(CL_UF_EVENT_SET_UF_RATE,3000);
						cl_wait(800);
						cl_uf_controller(CL_UF_EVENT_SET_UF_RATE,2500);*/
			
			Cl_dlsisretcode =  Cl_AlarmActivateAlarms(BLOODDOOR_STATUS_OPEN,true );
			Cl_dlsisretcode =  Cl_AlarmActivateAlarms(APTSTATUS_HIGH,true );
			Cl_dlsisretcode =  Cl_AlarmActivateAlarms(APTSTATUS_LOW,true );
			Cl_dlsisretcode =  Cl_AlarmActivateAlarms(VPTSTATUS_HIGH,true );
			Cl_dlsisretcode =  Cl_AlarmActivateAlarms(VPTSTATUS_LOW,true );
			Cl_dlsisretcode =  Cl_AlarmActivateAlarms(PS1_HIGH_THRESHOLD,true );
			Cl_dlsisretcode =  Cl_AlarmActivateAlarms(PS1_LOW_THRESHOLD,true );
			Cl_dlsisretcode =  Cl_AlarmActivateAlarms(PS2_HIGH_THRESHOLD,true );
			Cl_dlsisretcode =  Cl_AlarmActivateAlarms(PS2_LOW_THRESHOLD,true );
			Cl_dlsisretcode =  Cl_AlarmActivateAlarms(PS3_HIGH_THRESHOLD,true );
			Cl_dlsisretcode =  Cl_AlarmActivateAlarms(PS3_LOW_THRESHOLD,true );
			//	 Cl_dlsisretcode =  Cl_AlarmActivateAlarms(LEVELSWITCH_OFF_TO_ON,false );
			//	 Cl_dlsisretcode =  Cl_AlarmActivateAlarms(LEVELSWITCH_ON_TO_OFF ,false);
			Cl_dlsisretcode =  Cl_AlarmActivateAlarms(FLOW_NO_FLOW,true );
									Cl_dlsisretcode =  Cl_AlarmActivateAlarms(FLOW_LOW_FLOWRATE,true );
									Cl_dlsisretcode =  Cl_AlarmActivateAlarms(FLOW_HIGH_FLOWRATE,true ); 
			Cl_dlsisretcode =  Cl_AlarmActivateAlarms( HOLDER1STATUS_CLOSED,true );
			Cl_dlsisretcode =  Cl_AlarmActivateAlarms( HOLDER2STATUS_CLOSED,true );
			Cl_dlsisretcode =  Cl_AlarmActivateAlarms( HOLDER1STATUS_OPEN,false );
			Cl_dlsisretcode =  Cl_AlarmActivateAlarms( HOLDER2STATUS_OPEN,false );
			Cl_dlsisretcode =  Cl_AlarmActivateAlarms(COND_STATUS_LOW,true );
			Cl_dlsisretcode =  Cl_AlarmActivateAlarms(COND_STATUS_HIGH,true );
			//Cl_dlsisretcode =  Cl_AlarmActivateAlarms( ABD_EVENT,true );
			Cl_dlsisretcode =  Cl_AlarmActivateAlarms(ACID_IN_OPEN,false);
			Cl_dlsisretcode =  Cl_AlarmActivateAlarms(BICARB_IN_OPEN,false );
			Cl_dlsisretcode =  Cl_AlarmActivateAlarms(ACID_IN_CLOSED,true);
			Cl_dlsisretcode =  Cl_AlarmActivateAlarms(BICARB_IN_CLOSED,true );
			Cl_dlsisretcode =  Cl_AlarmActivateAlarms( BD_EVENT,false );
			//Cl_dlsisretcode =  Cl_AlarmActivateAlarms(BLD_EVENT,true );
			Cl_dlsisretcode =  Cl_AlarmActivateAlarms(TEMP3_HIGH_THRESHOLD,true );
			Cl_dlsisretcode =  Cl_AlarmActivateAlarms(TEMP3_LOW_THRESHOLD,true );
			
			
						cl_dlsis_state =CL_DLSIS_STATE_DIALYSIS; // added 
						//cl_dlsis_state = CL_DLSIS_STATE_UF_ACTIVATION; // commented as uf tick second event is under event dialysis tick seconds
					}
					else{
						
						sv_cntrl_enable_bypass();
						
						cl_uf_controller(CL_UF_EVENT_STOP,0);
						
						// trigger blood not detected alarm
						Cl_Alarm_TriggerAlarm(BLOOD_NOT_DETECTED, 1);               // manav02 added
					}
					
					}
			break;
			default:break;
		}
		break;
		case CL_DLSIS_STATE_UF_ACTIVATION:
		switch (cl_dlsis_event)
		{
			
			case EVENT_DLSIS_TICK_SECOND:
			Cl_Dlsis_ResetAlertsforReassertion();
			cl_uf_controller(CL_UF_EVENT_SET_UF_RATE,2500);
				cl_dlsis_state =CL_DLSIS_STATE_DIALYSIS;
			break;
			default:break;
		}
		
		break;
		
		case CL_DLSIS_STATE_DIALYSIS:
		switch (cl_dlsis_event)
		{

			case EVENT_DLSIS_DIALYSIS_ISOUF_ON:
			
			sv_cntrl_deactivate_valve(VALVE_ID13);
			sv_cntrl_activate_valve(VALVE_ID14);
			sv_cntrl_deactivate_valve(VALVE_ID15);
			cl_uf_controller(CL_UF_EVENT_START,0);
						
			isouf = true;
// 				Cl_dlsisretcode =  Cl_AlarmActivateAlarms(APTSTATUS_HIGH,true );
// 				Cl_dlsisretcode =  Cl_AlarmActivateAlarms(VPTSTATUS_HIGH ,true);
// 				Cl_dlsisretcode =  Cl_AlarmActivateAlarms(APTSTATUS_LOW,true );
// 				Cl_dlsisretcode =  Cl_AlarmActivateAlarms(VPTSTATUS_LOW,true);
// 				Cl_dlsisretcode =  Cl_AlarmActivateAlarms(PS2_HIGH_THRESHOLD,true );
				Cl_dlsisretcode =  Cl_AlarmActivateAlarms(PS2_LOW_THRESHOLD,false );
				Cl_dlsisretcode =  Cl_AlarmActivateAlarms(PS1_HIGH_THRESHOLD,false );
				Cl_dlsisretcode =  Cl_AlarmActivateAlarms(PS1_LOW_THRESHOLD,false );
				Cl_dlsisretcode =  Cl_AlarmActivateAlarms(PS3_HIGH_THRESHOLD ,false);
				Cl_dlsisretcode =  Cl_AlarmActivateAlarms(PS3_LOW_THRESHOLD ,false);
				Cl_dlsisretcode =  Cl_AlarmActivateAlarms(TMP_HIGH,false );
				Cl_dlsisretcode =  Cl_AlarmActivateAlarms(TMP_LOW,false );
			
			break;
			case EVENT_DLSIS_DIALYSIS_ISOUF_OFF:

			sv_cntrl_activate_valve(VALVE_ID13);
			sv_cntrl_activate_valve(VALVE_ID14);
			sv_cntrl_deactivate_valve(VALVE_ID15);
			
			if(ufRemoved >= Treatdata[ID_ufgoal]){
				cl_uf_controller(CL_UF_EVENT_STOP,0);
				}
			
			isouf = false;

				Cl_dlsisretcode =  Cl_AlarmActivateAlarms(PS2_LOW_THRESHOLD,true );
				Cl_dlsisretcode =  Cl_AlarmActivateAlarms(PS1_HIGH_THRESHOLD,true );
				Cl_dlsisretcode =  Cl_AlarmActivateAlarms(PS1_LOW_THRESHOLD,true );
				Cl_dlsisretcode =  Cl_AlarmActivateAlarms(PS3_HIGH_THRESHOLD ,true);
				Cl_dlsisretcode =  Cl_AlarmActivateAlarms(PS3_LOW_THRESHOLD ,true);
				Cl_dlsisretcode =  Cl_AlarmActivateAlarms(TMP_HIGH,true );
				Cl_dlsisretcode =  Cl_AlarmActivateAlarms(TMP_LOW,true );

			break;			

			case EVENT_DLSIS_COMMAND_SET_BLDPMP_OFF:			
			Cl_Dlsis_PauseDialysis();
			cl_dlsis_state = CL_DLSIS_STATE_PAUSED;
			break;
			case EVENT_DLSIS_GET_DATA:

				if(Cl_ConsoleRxMsg.msgready == true)
				{
					switch(Cl_ConsoleRxMsg.data.byte[0])
					{
						case	CON_RX_PARAM_DATA_DIALYSIS_STATUS:
						command = CON_TX_COMMAND_SYSDATA;
						dataarray[0] = CON_TX_PARAM_DATA_DIALYSIS_STATUS;
						Cl_dlsisretcode = (Cl_Uint8Type)sv_nvmgetdata(NV_NVM_DIALYSIS_STATUS, &dataarray[1]);
						Cl_dlsisretcode = Cl_SendDatatoconsole(command,&dataarray,2);
						break;

						default:
						break;
					}
				}
			break;
// 			case EVENT_DLSIS_SET_DATA:
// 				if(Cl_ConsoleRxMsg.msgready == true)
// 										{
// 													cl_Datastreamtype cl_temp  ;	
// 													cl_temp.bytearray[0] = Cl_ConsoleRxMsg.data.byte[1];
// 													cl_temp.bytearray[1] = Cl_ConsoleRxMsg.data.byte[2];
// 													cl_temp.bytearray[2] = Cl_ConsoleRxMsg.data.byte[3];
// 													cl_temp.bytearray[3] = Cl_ConsoleRxMsg.data.byte[4];
// 											
// 											switch(Cl_ConsoleRxMsg.data.byte[0])
// 											{
// 													case ID_dflow:
// 													break;
// 													case ID_settemp:
// 													break;
// 													case ID_heprate:
// 													break;
// 													case ID_setcond:
// 													break;
// 													case ID_ufrate:
// 													break;
// 													case ID_ufgoal:
// 													if(cl_temp.word == 0){
// 														cl_uf_controller(CL_UF_EVENT_STOP,0);
// 													}
// 													Treatdata[ID_ufgoal] = cl_temp.word;
// 													cl_uf_controller(CL_UF_EVENT_RESET,0);
// 													
// 													break;
// 													case ID_bolusvol:
// 													break;
// 													case ID_bloodrate:
// 													break;
// 													case ID_hepstoptime:
// 													break;
// 													case ID_syringetype:
// 													break;
// 													case ID_heparincheck:
// 													break;
// 													case ID_minufrate:
// 													break;
// 													case ID_treattime:
// 													Treatdata[ID_treattime] = cl_temp.word;
// 													cl_uf_controller(CL_UF_EVENT_RESET,0);
// 													
// 													break;
// 													case ID_bloodratereturn:
// 													break;
// 													case ID_bloodratetreat:
// 													break;
// 													case ID_tempulimit:
// 														Cl_alarmThresholdTable.temp3_high_threshold =  (cl_temp.word)/10;
// 													break;
// 													case ID_templlimit:
// 														Cl_alarmThresholdTable.temp3_low_threshold =  (cl_temp.word)/10;
// 													break;
// 													case ID_tmpllimit:
// 													Cl_alarmThresholdTable.tmp_low_threshold =  cl_temp.word;
// 													break;
// 													case ID_tmpulimit:
// 													Cl_alarmThresholdTable.tmp_high_threshold =  cl_temp.word;
// 													break;
// 													case ID_vptllimit:
// 														Cl_alarmThresholdTable.vpt_low_threshold =  cl_temp.word;
// 													break;
// 													case ID_vptulimit:
// 														Cl_alarmThresholdTable.vpt_high_threshold =  cl_temp.word;
// 													break;
// 													case ID_ufllimit:
// 													break;
// 													case ID_ufulimit:
// 													break;
// 													case ID_dflowllimit:
// 													break;
// 													case ID_dflowulimit:
// 													break;
// 													case ID_condllimit:
// 													Cl_alarmThresholdTable.cond_low_threshold =  cl_temp.word;
// 													break;
// 													case ID_condulimit:
// 													Cl_alarmThresholdTable.cond_high_threshold =  cl_temp.word;
// 													break;
// 													case ID_aptllimit:
// 														Cl_alarmThresholdTable.apt_low_threshold =  cl_temp.word;
// 													break;
// 													case ID_aptulimit:
// 													Cl_alarmThresholdTable.apt_high_threshold =  cl_temp.word;
// 													break;
// 
// 												default:break;
// 											}
// 										}
// 						
// 			break;
			case EVENT_DLSIS_TICK_50MS:
			
			break;
			
			case EVENT_DLSIS_TICK_500MS:
				//UpdateHeaterControls();
				// check for blood
				Cl_SysStat_GetSensor_Status_Query(SENSOR_BDSTATUS , &temp);
				
				if(temp == 0){
				// raise alarm blood not detected
				Cl_Alarm_TriggerAlarm(BLOOD_NOT_DETECTED,1);	
					
				}
					
				
			break;
			
			case EVENT_DLSIS_TICK_MINUTE:
				cl_uf_controller(CL_UF_EVENT_MINUTE,0);
			break;
			
			case EVENT_DLSIS_TICK_SECOND:
		//	Cl_Dlsis_ResetAlertsforReassertion();
			//	UpdateHeaterControls();
			bpflagCounter++;
			if(bpflagCounter > 120){
				
				bpFlag = 1;
				Cl_dlsisretcode =  Cl_AlarmActivateAlarms(BP_OVERRUN,true );
				Cl_dlsisretcode =  Cl_AlarmActivateAlarms(BP_UNDERRUN,true );
				
			}
			
			
				cl_hep_controller(CL_HEP_EVENT_SEC_TICK,0);
				Cl_Dlsis_SenddlsisData();
				DlsisTime.Cl_dlsissecondscounter++;
				if ((DlsisTime.Cl_dlsissecondscounter == 10))
				{
					//sv_cs_setpotvalue(1900);
					//flag_bc =0 ;
					Cl_Dlsis_ResetAlertsforReassertion();
				}
				if(DlsisTime.Cl_dlsissecondscounter == 60)
				{
					
					Cl_dlsisretcode = Cl_Dlsis_UpdateTimeInfo();
					DlsisTime.Cl_dlsissecondscounter = 0;
					DlsisTime.Cl_dlsisMinutescounter++;
					if (bypass_flag == 1)
						{
						/*Cl_dlsisretcode =  Cl_AlarmActivateAlarms(APTSTATUS_HIGH,true );
						Cl_dlsisretcode =  Cl_AlarmActivateAlarms(VPTSTATUS_HIGH ,true);
						Cl_dlsisretcode =  Cl_AlarmActivateAlarms(APTSTATUS_LOW,true );
						Cl_dlsisretcode =  Cl_AlarmActivateAlarms(VPTSTATUS_LOW,true);
						bypass_flag = 0;*/
					}
                          if (DlsisTime.Cl_dlsisMinutescounter == 1)
			                {
								/*Cl_dlsisretcode =  Cl_AlarmActivateAlarms(APTSTATUS_HIGH,true );
								Cl_dlsisretcode =  Cl_AlarmActivateAlarms(VPTSTATUS_HIGH ,true);
								Cl_dlsisretcode =  Cl_AlarmActivateAlarms(APTSTATUS_LOW,true );
								Cl_dlsisretcode =  Cl_AlarmActivateAlarms(VPTSTATUS_LOW,true);*/
						
									Cl_Uint16Type temp_ps=0, low_threshold=0, high_threshold=0;
									temp_ps=apt;
									Cl_SendDatatoconsole(CON_TX_COMMAND_PRINTTEXT,"APT",4);
									Cl_SendDatatoconsole(CON_TX_COMMAND_PRINTDATA,&temp_ps,2);
						
										//Cl_alarmThresholdTable.apt_high_threshold = apt + 50;
										//Cl_alarmThresholdTable.apt_low_threshold = apt - 50;
										//low_threshold = Cl_alarmThresholdTable.apt_low_threshold;
										//high_threshold = Cl_alarmThresholdTable.apt_high_threshold;
						
										Cl_SendDatatoconsole(CON_TX_COMMAND_PRINTDATA,&low_threshold,2);
										Cl_SendDatatoconsole(CON_TX_COMMAND_PRINTDATA,&high_threshold,2);
						
										//Cl_alarmThresholdTable.vpt_high_threshold = vpt + 50;
										//Cl_alarmThresholdTable.vpt_low_threshold = vpt - 50;
							}
					DlsisTime.Cl_dlsisTotalMinutescounter++;
					if (DlsisTime.Cl_dlsisTotalMinutescounter == CL_DLSIS_TIMEOUT_MIN);//Treatdata[ID_treattime])
					{
						//cl_testvirus();
					}
				}
				if(cl_Dlsis_checkforHeparinCompletion() == CL_OK){
					
					
					Cl_AlarmActivateAlarms(HP_OVERRUN,false);
					Cl_AlarmActivateAlarms(HP_UNDERRUN,false);
					cl_hep_controller(CL_HEP_EVENT_STOP_NORMAL_DELIVERY,0);
					
				}
				if(cl_Dlsis_CheckforDialysisCompletion() == CL_OK )
				{
					sv_cntrl_setgreenalarm();
				//	sv_cntrl_deactivatepump(BLOODPUMP);
					Cl_dlsisretcode = cl_Dlsis_notifydacandgotorinsestandby();
					sv_cntrl_poweroffheater();
					SetHeaterState(CL_HEATER_STATE_OFF);
					sv_cntrl_setflowpath(FLOW_PATH_IDLE_RINSE);
					Cl_DlsisFillingFlowOff();
					cl_uf_controller(CL_UF_EVENT_STOP,0);
					cl_hep_controller(CL_HEP_EVENT_STOP_NORMAL_DELIVERY,0);
					cl_dlsis_state = CL_DLSIS_STATE_POST_DIALYSIS_STANDBY;
				}
			break;
			case EVENT_DLSIS_STOP_DIALYSIS:
			Cl_dlsisretcode = sv_cntrl_enable_bypass();
			Cl_Dlsis_StopDialysis();
			cl_dlsis_state = CL_DLSIS_STATE_STOPPED;
			break;
			case EVENT_DLSIS_PAUSE_DIALYSIS:
			Cl_Dlsis_PauseDialysis();
			cl_dlsis_state = CL_DLSIS_STATE_PAUSED;
			break;
			case EVENT_DLSIS_BYPASS_ON:
			Cl_Dlsis_BypassDialysis();
			break;
			case EVENT_DLSIS_BYPASS_OFF:
			Cl_Dlsis_BypassOff();
			break;

			case EVENT_DLSIS__MINUF_ON:
			break;
			case EVENT_DLSIS__MINUF_OFF:
			break;
			case EVENT_DLSIS_ALARM:
					Cl_dlsisretcode = Cl_Dlsis_ProcessAlarms();
			break;
			case EVENT_DLSIS_ALERT:
					//Cl_Alarm_GetAlarmStatus(FPCURRENTSTATUS,&alarmstatus1);
					if(alarmstatus1)
					{
						//Cl_dlsisretcode = Cl_bc_controller(BC_EVENT_CS);
					}
					Cl_Alarm_GetAlarmStatus(TEMP2_HIGH_THRESHOLD,&alarmstatus3);
					if(alarmstatus3)
					{
						//	UpdateHeaterControls();
					}
			
					Cl_Alarm_GetAlarmStatus(FLOW_HIGH_FLOWRATE,&flowstatus);
					//	if(flowstatus)
					//	{
					//		Cl_rinseretcode = sv_cntrl_poweroffheater();
					//		Cl_SendDatatoconsole(CON_TX_COMMAND_PRINTTEXT,"FLOW_OFF",8);
					//	}
			break;


		}
		break;
		case CL_DLSIS_STATE_BYPASS:
									switch (cl_dlsis_event)
									{
										case EVENT_DLSIS_ALARM:
										Cl_dlsisretcode = Cl_Dlsis_ProcessAlarms();
										break;
										case EVENT_DLSIS_BYPASS_OFF:
										Cl_Dlsis_BypassOff();
										break;
										case EVENT_DLSIS_PAUSE_DIALYSIS:
										Cl_Dlsis_PauseDialysis();
										cl_dlsis_state = CL_DLSIS_STATE_PAUSED;
										break;
										case EVENT_DLSIS_STOP_DIALYSIS:                   // manav02 dialysis stopped changed
										sv_cntrl_enable_bypass();
										Cl_Dlsis_StopDialysis();
										cl_dlsis_state = CL_DLSIS_STATE_STOPPED;
										break;
										default:break;
									}

					
		break;
		
		
		case CL_DLSIS_STATE_PAUSED:
							switch (cl_dlsis_event)
							{
								case EVENT_DLSIS_START_DIALYSIS:
								Cl_Dlsis_ResumeDialysis();
								break;
								case EVENT_DLSIS_STOP_DIALYSIS:                   // manav02 dialysis stopped changed
								sv_cntrl_enable_bypass();
								Cl_Dlsis_StopDialysis();
								cl_dlsis_state = CL_DLSIS_STATE_STOPPED;
								break;
								case EVENT_DLSIS_BYPASS_OFF:
								Cl_Dlsis_BypassOff();
								break;
								case EVENT_DLSIS_BYPASS_ON:
								Cl_Dlsis_BypassDialysis();
								break;
								case EVENT_DLSIS_ALARM:
										Cl_dlsisretcode = Cl_Dlsis_ProcessAlarms();
										break;
								default:break;
							}
		
		break;
		case CL_DLSIS_STATE_STOPPED:
							switch (cl_dlsis_event)
							{
// 								case EVENT_DLSIS_START_DIALYSIS:
// 								Cl_Dlsis_StartDialysis();
// 								break;
								case EVENT_DLSIS_START_RINSE:
								Cl_mac_apprequesthandler(MACREQ_POST_DIALYSIS_RINSE_REQ);
								break;
								case EVENT_DLSIS_ISOUF_START:
								
								//Cl_MacState = MAC_ISOUF;
								Cl_mac_apprequesthandler(MACREQ_POST_DIALYSIS_ISOUF_REQ);
								//Cl_MacRetval = cl_isouf_controller(Cl_MacEvent);
								
								
								break;
								
									case EVENT_DLSIS_CLR_ALARM:
									
									if(cl_dlsis_powerfailure == 1){
										
										Cl_SysStat_GetSensor_Status_Query(SENSOR_POWER_FAILURE,&dlsis_powerFailSensorData);
										
										if(dlsis_powerFailSensorData == 0){
											
											cl_dlsis_powerfailure = 0;
											command = CON_TX_COMMAND_ALARM_CLEARED;
											Cl_SendDatatoconsole(command,&data,0);
											
											Cl_Dlsis_ResetAlertsforReassertion();
											sv_cntrl_resetredalarm();
											sv_cntrl_resetyellowalarm();
											sv_cntrl_nobuzzer();
										
										}
									}
									else{
										
										command = CON_TX_COMMAND_ALARM_CLEARED;
										Cl_SendDatatoconsole(command,&data,0);
										
										Cl_Dlsis_ResetAlertsforReassertion();
										sv_cntrl_resetredalarm();
										sv_cntrl_resetyellowalarm();
										sv_cntrl_nobuzzer();
										
										//cl_temp_recoveredstate();
									}
									break;
								
								default:break;
							}
							
		break;

		case CL_DLSIS_STATE_CRITICAL_ALARM:
					switch (cl_dlsis_event)
					{
						case EVENT_DLSIS_START_DIALYSIS:
						Cl_Dlsis_ResumeDialysis();
						break;
					//	case EVENT_DLSIS_ALARM:
					//	Cl_dlsisretcode = Cl_Dlsis_ProcessAlarms();
					//	break;
						case EVENT_DLSIS__START_RETURN:
						break;
						case EVENT_DLSIS__STOP_RETURN:
						break;
						//manav05 commented
// 						case EVENT_DLSIS_GET_DATA:
// 
// 							if(Cl_ConsoleRxMsg.msgready == true)
// 							{
// 								switch(Cl_ConsoleRxMsg.data.byte[0])
// 								{
// 									case	CON_RX_PARAM_DATA_DIALYSIS_STATUS:
// 									command = CON_TX_COMMAND_SYSDATA;
// 									dataarray[0] = CON_TX_PARAM_DATA_DIALYSIS_STATUS;
// 									Cl_dlsisretcode = (Cl_Uint8Type)sv_nvmgetdata(NV_NVM_DIALYSIS_STATUS, &dataarray[1]);
// 									Cl_dlsisretcode = Cl_SendDatatoconsole(command,&dataarray,2);
// 									break;
// 
// 									default:
// 									break;
// 								}
// 							}
// 						break;
						case EVENT_DLSIS_TICK_SECOND:
						
						Cl_Dlsis_SenddlsisData();
						/*
							for (tempcount = 0 ; tempcount < CL_DLSIS_ALRM_MAX ; tempcount++)
							{
								//Cl_SendDatatoconsole(CON_TX_COMMAND_PRINTTEXT,"ENTER",8);
								Cl_Alarm_GetAlarmStatus(Cl_DlsisAlarmTable[tempcount].Cl_DlsisAlarmId,&Cl_DlsisAlarmTable[tempcount].IsActive);
								if (Cl_DlsisAlarmTable[tempcount].IsActive)
								{
										Cl_SendDatatoconsole(CON_TX_COMMAND_PRINTTEXT,"ENTER_1",8);
										dlsisalarm = Cl_DlsisAlarmTable[tempcount].Cl_DlsisAlarmId;
										dlsis_check_alarm(dlsisalarm);
										dlsisalarm1 = dlsisalarm;
										var=1;
										cl_dlsis_state = CL_DLSIS_STATE_CRITICAL_ALARM;
										tempcount=0;
										break;	
								}
								if (var == 1)
								{
									Cl_SendDatatoconsole(CON_TX_COMMAND_PRINTTEXT,"RECOVERY",8);
									dlsis_check_alarm(dlsisalarm1);
									var = 0;
								}
								if (tempcount == CL_DLSIS_ALRM_MAX - 1)
								{
									if(!Cl_DlsisAlarmTable[tempcount].IsActive)
									{
										Cl_SendDatatoconsole(CON_TX_COMMAND_PRINTTEXT,"SAFE",8);
										//sv_cntrl_resetredalarm();
										//sv_cntrl_resetyellowalarm();
										//sv_cntrl_nobuzzer();
										//cl_bp_controller(CL_BP_EVENT_START,0);
										//sv_cntrl_activatevenousclamp();
										//Cl_Dlsis_StartDialysis();
										//cl_dlsis_state = CL_DLSIS_STATE_DIALYSIS;
									}
								}
							} */
						/*	if (dlsisalarm != _NO_ALARM)
							{
								Cl_SendDatatoconsole(CON_TX_COMMAND_PRINTTEXT,"ENTER_1",8);
								dlsis_check_alarm(dlsisalarm);
							}*/
						break;
						case EVENT_DLSIS_CLR_ALARM:
						
											if(cl_dlsis_powerfailure == 1){
												
											Cl_SysStat_GetSensor_Status_Query(SENSOR_POWER_FAILURE,&dlsis_powerFailSensorData);
					
								if(dlsis_powerFailSensorData == 0){
													
													cl_dlsis_powerfailure = 0;
													command = CON_TX_COMMAND_ALARM_CLEARED;
													Cl_SendDatatoconsole(command,&data,0);
													
													Cl_Dlsis_ResetAlertsforReassertion();
													sv_cntrl_resetredalarm();
													sv_cntrl_resetyellowalarm();
													sv_cntrl_nobuzzer();
													cl_bp_controller(CL_BP_EVENT_START,0);
													sv_cntrl_activatevenousclamp();
													Cl_Dlsis_BypassOff();
												}
											}
											else{
						
						command = CON_TX_COMMAND_ALARM_CLEARED;
						Cl_SendDatatoconsole(command,&data,0);
						
						Cl_Dlsis_ResetAlertsforReassertion();
										sv_cntrl_resetredalarm();
										sv_cntrl_resetyellowalarm();
										sv_cntrl_nobuzzer();
										cl_bp_controller(CL_BP_EVENT_START,0);
										sv_cntrl_activatevenousclamp();
										Cl_Dlsis_BypassOff();
										//cl_temp_recoveredstate();
											}
						break;
						case EVENT_DLSIS_ALARM:
						Cl_dlsisretcode = Cl_Dlsis_ProcessAlarms();
						break;
						case EVENT_DLSIS_BYPASS_ON:
						Cl_Dlsis_BypassDialysis();
						break;
						case EVENT_DLSIS_BYPASS_OFF:
						Cl_Dlsis_BypassOff();
						break;
						case EVENT_DLSIS_STOP_DIALYSIS:                   // manav02 dialysis stopped changed
						sv_cntrl_enable_bypass();
						Cl_Dlsis_StopDialysis();
						cl_dlsis_state = CL_DLSIS_STATE_STOPPED;
						break;
						
						default:break;
					}
		break;
		case CL_DLSIS_STATE_POST_DIALYSIS_STANDBY:
				switch(cl_dlsis_event)
					{
						case EVENT_DLSIS__START_RETURN:
						break;
						case EVENT_DLSIS__STOP_RETURN:
						break;
						
						case EVENT_DLSIS_GET_DATA:

	
						case EVENT_DLSIS_HEP_RATE:
							if(Cl_ConsoleRxMsg.msgready == true)
							{
				
								cl_Datastreamtype cl_temp  ;
								cl_temp.bytearray[0] = Cl_ConsoleRxMsg.data.byte[0];
								cl_temp.bytearray[1] = Cl_ConsoleRxMsg.data.byte[1];
								cl_temp.bytearray[2] = Cl_ConsoleRxMsg.data.byte[2];
								cl_temp.bytearray[3] = Cl_ConsoleRxMsg.data.byte[3];
								Treatdata[ID_heprate] = cl_temp.Twobyte;
								hep_calibration(Treatdata[ID_heprate]);
								cl_hep_controller(CL_HEP_EVENT_SET_NORMAL_DELIVERY_RATE,hep_speed);
							}
						break;
						case EVENT_DLSIS_HEP_ON:
							cl_hep_controller(CL_HEP_EVENT_START_NORMAL_DELIVERY,hep_speed);
						break;
						case EVENT_DLSIS_HEP_OFF:
							cl_hep_controller(CL_HEP_EVENT_STOPBOLUS,0);
						break;
						case EVENT_DLSIS_COMMAND_SET_BLDPMP_ON:
						cl_bp_controller(CL_BP_EVENT_START,0);
						break;
						case EVENT_DLSIS_COMMAND_SET_BLDPMP_OFF:
						cl_bp_controller(CL_BP_EVENT_STOP,0);
						break;
						case EVENT_DLSIS_COMMAND_SET_BLDPUMPRATE:
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
// 							case EVENT_DLSIS_SET_DATA:
// 									if(Cl_ConsoleRxMsg.msgready == true)
// 									{
// 										cl_Datastreamtype cl_temp  ;
// 										cl_temp.bytearray[0] = Cl_ConsoleRxMsg.data.byte[1];
// 										cl_temp.bytearray[1] = Cl_ConsoleRxMsg.data.byte[2];
// 										cl_temp.bytearray[2] = Cl_ConsoleRxMsg.data.byte[3];
// 										cl_temp.bytearray[3] = Cl_ConsoleRxMsg.data.byte[4];
// 				
// 										switch(Cl_ConsoleRxMsg.data.byte[0])
// 											{
// 											case ID_dflow:
// 											break;
// 											case ID_settemp:
// 											break;
// 											case ID_heprate:
// 											break;
// 											case ID_setcond:
// 											break;
// 											case ID_ufrate:
// 											break;
// 											case ID_ufgoal:
// 											break;
// 											case ID_bolusvol:
// 											break;
// 											case ID_bloodrate:
// 											break;
// 											case ID_hepstoptime:
// 											break;
// 											case ID_syringetype:
// 											break;
// 											case ID_heparincheck:
// 											break;
// 											case ID_minufrate:
// 											break;
// 											case ID_treattime:
// 											break;
// 											case ID_bloodratereturn:
// 											break;
// 											case ID_bloodratetreat:
// 											break;
// 											case ID_tempulimit:
// 											Cl_alarmThresholdTable.temp3_high_threshold =  (cl_temp.word)/10;
// 											break;
// 											case ID_templlimit:
// 											Cl_alarmThresholdTable.temp3_low_threshold =  (cl_temp.word)/10;
// 											break;
// 											case ID_tmpllimit:
// 											Cl_alarmThresholdTable.tmp_low_threshold =  cl_temp.word;
// 											break;
// 											case ID_tmpulimit:
// 											Cl_alarmThresholdTable.tmp_high_threshold =  cl_temp.word;
// 											break;
// 											case ID_vptllimit:
// 											Cl_alarmThresholdTable.vpt_low_threshold =  cl_temp.word;
// 											break;
// 											case ID_vptulimit:
// 											Cl_alarmThresholdTable.vpt_high_threshold =  cl_temp.word;
// 											break;
// 											case ID_ufllimit:
// 											break;
// 											case ID_ufulimit:
// 											break;
// 											case ID_dflowllimit:
// 											break;
// 											case ID_dflowulimit:
// 											break;
// 											case ID_condllimit:
// 											Cl_alarmThresholdTable.cond_low_threshold =  cl_temp.word;
// 											break;
// 											case ID_condulimit:
// 											Cl_alarmThresholdTable.cond_high_threshold =  cl_temp.word;
// 											break;
// 											case ID_aptllimit:
// 											Cl_alarmThresholdTable.apt_low_threshold =  cl_temp.word;
// 											break;
// 											case ID_aptulimit:
// 											Cl_alarmThresholdTable.apt_high_threshold =  cl_temp.word;
// 											
// 											break;
// 
// 											default:break;
// 											}
// 										
// 										}
// 			
// 						break;
						default:
						break;
					}
		break;
		default:break;
	}



return CL_OK;
}


Cl_ReturnCodeType dlsis_check_alarm(Cl_NewAlarmIdType dlsisalarm)
{
	float temp;
	Cl_Uint8Type temp1;
	Cl_Uint8Type dlsis;
	switch (dlsisalarm)
	{
		//Cl_SendDatatoconsole(CON_TX_COMMAND_PRINTTEXT,"ENTER_2",8);
		
		case BLOODDOOR_STATUS_OPEN:
				Cl_SysStat_GetSensor_Status_Query(SENSOR_BLOODDOOR_STATUS,&temp1);
				if (temp1 == 1)
				{
					Cl_Dlsis_safesate();
				}
				else
				{
					Cl_Dlsis_recoveredsate();
					cl_dlsis_state = CL_DLSIS_STATE_CRITICAL_ALARM;
				}	
		break;
		
		case HOLDER1STATUS_CLOSED:
		Cl_SendDatatoconsole(CON_TX_COMMAND_PRINTTEXT,"ENTER_2",8);
			Cl_SysStat_GetSensor_Status_Query(SENSOR_HOLDER1STATUS,&dlsis);
			if (dlsis == 0)
			{
				Cl_SendDatatoconsole(CON_TX_COMMAND_PRINTTEXT,"ENTER_3",8);
				cl_temp_safestate();
			}
			else
			{
				Cl_SendDatatoconsole(CON_TX_COMMAND_PRINTTEXT,"ENTER_4",8);
				cl_temp_recoveredstate();
				cl_dlsis_state = CL_DLSIS_STATE_CRITICAL_ALARM;
			}
		break;
		
		case HOLDER2STATUS_CLOSED:
		Cl_SysStat_GetSensor_Status_Query(SENSOR_HOLDER2STATUS,&dlsis);
		if (dlsis == 0)
		{
			cl_temp_safestate();
		}
		else
		{
			cl_temp_recoveredstate();
			cl_dlsis_state = CL_DLSIS_STATE_CRITICAL_ALARM;
		}
		break;
		
		case BD_EVENT:
			Cl_SendDatatoconsole(CON_TX_COMMAND_PRINTTEXT,"ENTER_3",8);
			Cl_SysStat_GetSensor_Status_Query(SENSOR_BDSTATUS,&temp1);
			if (temp1 == 0)
			{
				Cl_Dlsis_StartDialysis();
				cl_dlsis_state = CL_DLSIS_STATE_DIALYSIS;
			}
			else
			{
				Cl_Dlsis_StopDialysis();
				cl_dlsis_state = CL_DLSIS_STATE_CRITICAL_ALARM;
			}
		break;
		
		case ABD_EVENT:
			Cl_SendDatatoconsole(CON_TX_COMMAND_PRINTTEXT,"ENTER_3",8);
			Cl_SysStat_GetSensor_Status_Query(SENSOR_ABDSTATUS,&temp1);
			if (temp1 == 0)
			{
				Cl_Dlsis_safesate();
			}
			else
			{
				Cl_SendDatatoconsole(CON_TX_COMMAND_PRINTTEXT,"ENTER_4",8);
				Cl_Dlsis_recoveredsate();
				cl_dlsis_state = CL_DLSIS_STATE_CRITICAL_ALARM;
			}
		break;
		
		case BLD_EVENT:
			Cl_SendDatatoconsole(CON_TX_COMMAND_PRINTTEXT,"ENTER_3",8);
			Cl_SysStat_GetSensor_Status_Query(SENSOR_BLDSTATUS,&temp1);
			if (temp1 == 0)
			{
				Cl_Dlsis_safesate();
			}
			else
			{
				Cl_SendDatatoconsole(CON_TX_COMMAND_PRINTTEXT,"ENTER_4",8);
				Cl_Dlsis_recoveredsate();
				cl_dlsis_state = CL_DLSIS_STATE_CRITICAL_ALARM;
			}
		break;
		
		case COND_STATUS_HIGH:
		Cl_SendDatatoconsole(CON_TX_COMMAND_PRINTTEXT,"ENTER_3",8);
		temp = (cl_sys_statbuffer.cond_status * 3300/4096);
		calibration_cond(temp);
// 		temp = (cl_sys_statbuffer.Temp3status * 3300/4096);
// 		calibration_tmp(temp,TS3);
       
		dlsis_cond= cond_final_cs3/(1+(cl_sys_statbuffer.Temp3status-25.0)*0.021);         // changed the temperature reading style from previous to current
		dlsis_cond=dlsis_cond/10;
		Cl_Uint16Type var1;
		var1=dlsis_cond;
		Cl_SendDatatoconsole(CON_TX_COMMAND_PRINTDATA,&var1,2);
		if (dlsis_cond > Cl_alarmThresholdTable.cond_high_threshold)
		{
			cl_cond_safestate();
		}
		else
		{
			Cl_SendDatatoconsole(CON_TX_COMMAND_PRINTTEXT,"ENTER_4",8);
			cl_cond_recoveredstate();
			cl_dlsis_state = CL_DLSIS_STATE_CRITICAL_ALARM;
		}
		break;
		
		case COND_STATUS_LOW:
		Cl_SendDatatoconsole(CON_TX_COMMAND_PRINTTEXT,"ENTER_3",8);
		temp = (cl_sys_statbuffer.cond_status * 3300/4096);
		calibration_cond(temp);
// 		temp = (cl_sys_statbuffer.Temp3status * 3300/4096);
// 		calibration_tmp(temp,TS3);
		temprature_final_value_3 = cl_sys_statbuffer.Temp3status;
		dlsis_cond= cond_final_cs3/(1+(temprature_final_value_3-25.0)*0.021);
		dlsis_cond=dlsis_cond/10;
		if (dlsis_cond < Cl_alarmThresholdTable.cond_low_threshold)
		{
			Cl_Uint16Type var1;
			var1=dlsis_cond;
			Cl_SendDatatoconsole(CON_TX_COMMAND_PRINTDATA,&var1,2);
			cl_cond_safestate();
		}
		else
		{
			Cl_SendDatatoconsole(CON_TX_COMMAND_PRINTTEXT,"ENTER_4",8);
			cl_cond_recoveredstate();
			cl_dlsis_state = CL_DLSIS_STATE_CRITICAL_ALARM;
		}
		break;
		
		case PS1_HIGH_THRESHOLD:
		Cl_SendDatatoconsole(CON_TX_COMMAND_PRINTTEXT,"ENTER_3",8);
		temp = (cl_sys_statbuffer.ps1status * 3300/4096);
		calibration_ps1(temp);
		dlsis_ps1 = pressure_final_ps1;
		if (dlsis_ps1 > Cl_alarmThresholdTable.ps1_high_threshold)
		{
			Cl_Uint16Type var1;
			var1=dlsis_ps1;
			Cl_SendDatatoconsole(CON_TX_COMMAND_PRINTDATA,&var1,2);
			cl_temp_safestate();
		}
		else
		{
			Cl_SendDatatoconsole(CON_TX_COMMAND_PRINTTEXT,"ENTER_4",8);
			cl_temp_recoveredstate();
			cl_dlsis_state = CL_DLSIS_STATE_CRITICAL_ALARM;
		}
		break;
		
		case PS1_LOW_THRESHOLD:
		Cl_SendDatatoconsole(CON_TX_COMMAND_PRINTTEXT,"ENTER_3",8);
		temp = (cl_sys_statbuffer.ps1status * 3300/4096);
		calibration_ps1(temp);
		dlsis_ps1 = pressure_final_ps1;
		if (dlsis_ps1 < Cl_alarmThresholdTable.ps1_low_threshold)
		{
			Cl_Uint16Type var1;
			var1=dlsis_ps1;
			Cl_SendDatatoconsole(CON_TX_COMMAND_PRINTDATA,&var1,2);
			cl_temp_safestate();
		}
		else
		{
			Cl_SendDatatoconsole(CON_TX_COMMAND_PRINTTEXT,"ENTER_4",8);
			cl_temp_recoveredstate();
			cl_dlsis_state = CL_DLSIS_STATE_CRITICAL_ALARM;
		}
		break;
		
		case PS2_HIGH_THRESHOLD:
		Cl_SendDatatoconsole(CON_TX_COMMAND_PRINTTEXT,"ENTER_3",8);
		temp = (cl_sys_statbuffer.ps2status * 3300/4096);
		calibration_ps2(temp);
		dlsis_ps2 = pressure_final_ps2;
		if (dlsis_ps2 > Cl_alarmThresholdTable.ps2_high_threshold)
		{
			Cl_Uint16Type var1;
			var1=dlsis_ps2;
			Cl_SendDatatoconsole(CON_TX_COMMAND_PRINTDATA,&var1,2);
			cl_temp_safestate();
		}
		else
		{
			Cl_SendDatatoconsole(CON_TX_COMMAND_PRINTTEXT,"ENTER_4",8);
			cl_temp_recoveredstate();
			cl_dlsis_state = CL_DLSIS_STATE_CRITICAL_ALARM;
		}
		break;
		
		case PS2_LOW_THRESHOLD:
		Cl_SendDatatoconsole(CON_TX_COMMAND_PRINTTEXT,"ENTER_3",8);
		temp = (cl_sys_statbuffer.ps2status * 3300/4096);
		calibration_ps2(temp);
		dlsis_ps2 = pressure_final_ps2;
		if (dlsis_ps2 < Cl_alarmThresholdTable.ps2_low_threshold)
		{
			Cl_Uint16Type var1;
			var1=dlsis_ps2;
			Cl_SendDatatoconsole(CON_TX_COMMAND_PRINTDATA,&var1,2);
			cl_temp_safestate();
		}
		else
		{
			Cl_SendDatatoconsole(CON_TX_COMMAND_PRINTTEXT,"ENTER_4",8);
			cl_temp_recoveredstate();
			cl_dlsis_state = CL_DLSIS_STATE_CRITICAL_ALARM;
		}
		break;
		
		case PS3_HIGH_THRESHOLD:
		Cl_SendDatatoconsole(CON_TX_COMMAND_PRINTTEXT,"ENTER_3",8);
		temp = (cl_sys_statbuffer.ps3status * 3300/4096);
		calibration_ps3(temp);
		dlsis_ps3 = pressure_final_ps3;
		if (dlsis_ps3 > Cl_alarmThresholdTable.ps3_high_threshold)
		{
			Cl_Uint16Type var1;
			var1=dlsis_ps3;
			Cl_SendDatatoconsole(CON_TX_COMMAND_PRINTDATA,&var1,2);
			cl_temp_safestate();
		}
		else
		{
			Cl_SendDatatoconsole(CON_TX_COMMAND_PRINTTEXT,"ENTER_4",8);
			cl_temp_recoveredstate();
			cl_dlsis_state = CL_DLSIS_STATE_CRITICAL_ALARM;
		}
		break;
		
		case PS3_LOW_THRESHOLD:
		Cl_SendDatatoconsole(CON_TX_COMMAND_PRINTTEXT,"ENTER_3",8);
		temp = (cl_sys_statbuffer.ps3status * 3300/4096);
		calibration_ps3(temp);
		dlsis_ps3 = pressure_final_ps3;
		if (dlsis_ps3 < Cl_alarmThresholdTable.ps3_low_threshold)
		{
			Cl_Uint16Type var1;
			var1=dlsis_ps3;
			Cl_SendDatatoconsole(CON_TX_COMMAND_PRINTDATA,&var1,2);
			cl_temp_safestate();
		}
		else
		{
			Cl_SendDatatoconsole(CON_TX_COMMAND_PRINTTEXT,"ENTER_4",8);
			cl_temp_recoveredstate();
			cl_dlsis_state = CL_DLSIS_STATE_CRITICAL_ALARM;
		}
		break;
		case APTSTATUS_HIGH:
		Cl_SendDatatoconsole(CON_TX_COMMAND_PRINTTEXT,"ENTER_3",8);
		temp = (cl_sys_statbuffer.aptstatus * 3300/4096);
		calibration_apt(temp);
		dlsis_apt = pressure_final_apt;
		if (dlsis_apt > Cl_alarmThresholdTable.apt_high_threshold)
		{
			Cl_Uint16Type var1;
			var1=dlsis_apt;
			Cl_SendDatatoconsole(CON_TX_COMMAND_PRINTDATA,&var1,2);
			Cl_Dlsis_safesate();
		}
		else
		{
			Cl_SendDatatoconsole(CON_TX_COMMAND_PRINTTEXT,"ENTER_4",8);
			Cl_Dlsis_recoveredsate();
			cl_dlsis_state = CL_DLSIS_STATE_CRITICAL_ALARM;
		}
		break;
		
		case APTSTATUS_LOW:
		Cl_SendDatatoconsole(CON_TX_COMMAND_PRINTTEXT,"ENTER_3",8);
		temp = (cl_sys_statbuffer.aptstatus * 3300/4096);
		calibration_apt(temp);
		dlsis_apt = pressure_final_apt;
		if (dlsis_apt < Cl_alarmThresholdTable.apt_low_threshold)
		{
			Cl_Uint16Type var1;
			var1=dlsis_apt;
			Cl_SendDatatoconsole(CON_TX_COMMAND_PRINTDATA,&var1,2);
			Cl_Dlsis_safesate();
		}
		else
		{
			Cl_SendDatatoconsole(CON_TX_COMMAND_PRINTTEXT,"ENTER_4",8);
			Cl_Dlsis_recoveredsate();
			cl_dlsis_state = CL_DLSIS_STATE_CRITICAL_ALARM;
		}
		break;
		
		case VPTSTATUS_HIGH:
		Cl_SendDatatoconsole(CON_TX_COMMAND_PRINTTEXT,"ENTER_3",8);
		temp = (cl_sys_statbuffer.vptstatus * 3300/4096);
		calibration_vpt(temp);
		dlsis_vpt = pressure_final_vpt;
		if (dlsis_vpt > Cl_alarmThresholdTable.vpt_high_threshold)
		{
			Cl_Uint16Type var1;
			var1=dlsis_vpt;
			Cl_SendDatatoconsole(CON_TX_COMMAND_PRINTDATA,&var1,2);
			Cl_Dlsis_safesate();
		}
		else
		{
			Cl_SendDatatoconsole(CON_TX_COMMAND_PRINTTEXT,"ENTER_4",8);
			Cl_Dlsis_recoveredsate();
			cl_dlsis_state = CL_DLSIS_STATE_CRITICAL_ALARM;
		}
		break;
		
		case VPTSTATUS_LOW:
		Cl_SendDatatoconsole(CON_TX_COMMAND_PRINTTEXT,"ENTER_3",8);
		temp = (cl_sys_statbuffer.vptstatus * 3300/4096);
		calibration_vpt(temp);
		dlsis_vpt = pressure_final_vpt;
		if (dlsis_vpt < Cl_alarmThresholdTable.vpt_low_threshold)
		{
			Cl_Uint16Type var1;
			var1=dlsis_vpt;
			Cl_SendDatatoconsole(CON_TX_COMMAND_PRINTDATA,&var1,2);
			Cl_Dlsis_safesate();
		}
		else
		{
			Cl_SendDatatoconsole(CON_TX_COMMAND_PRINTTEXT,"ENTER_4",8);
			Cl_Dlsis_recoveredsate();
			cl_dlsis_state = CL_DLSIS_STATE_CRITICAL_ALARM;
		}
		break;
		case TEMP3_HIGH_THRESHOLD:
		Cl_SendDatatoconsole(CON_TX_COMMAND_PRINTTEXT,"ENTER_3",8);
//		temp = (cl_sys_statbuffer.Temp3status * 3300/4096);
//		calibration_tmp(temp,TS3);
		dlsis_temp3 = cl_sys_statbuffer.Temp3status;
		if (dlsis_temp3 > Cl_alarmThresholdTable.temp3_high_threshold)
		{
			Cl_Uint16Type var1;
			var1=dlsis_temp3;
			Cl_SendDatatoconsole(CON_TX_COMMAND_PRINTDATA,&var1,2);
			cl_temp_safestate();
		}
		else
		{
			Cl_SendDatatoconsole(CON_TX_COMMAND_PRINTTEXT,"ENTER_4",8);
			cl_temp_recoveredstate();
			cl_dlsis_state = CL_DLSIS_STATE_CRITICAL_ALARM;
		}
		break;
		case TEMP3_LOW_THRESHOLD:
		Cl_SendDatatoconsole(CON_TX_COMMAND_PRINTTEXT,"ENTER_3",8);
// 		temp = (cl_sys_statbuffer.Temp3status * 3300/4096);
// 		calibration_tmp(temp,TS3);
		
		dlsis_temp3 = cl_sys_statbuffer.Temp1status;
		if (dlsis_temp3 < Cl_alarmThresholdTable.temp3_low_threshold)
		{
			Cl_Uint16Type var1;
			var1=dlsis_temp3;
			Cl_SendDatatoconsole(CON_TX_COMMAND_PRINTDATA,&var1,2);
			cl_temp_safestate();
		}
		else
		{
			Cl_SendDatatoconsole(CON_TX_COMMAND_PRINTTEXT,"ENTER_4",8);
			cl_temp_recoveredstate();
			cl_dlsis_state = CL_DLSIS_STATE_CRITICAL_ALARM;
		}
		break;
		
		case TEMP2_HIGH_THRESHOLD:
		Cl_SendDatatoconsole(CON_TX_COMMAND_PRINTTEXT,"ENTER_3",8);
// 		temp = (cl_sys_statbuffer.Temp2status * 3300/4096);
// 		calibration_tmp(temp,TS2);
		dlsis_temp2 = cl_sys_statbuffer.Temp2status;
		if (dlsis_temp2 > Cl_alarmThresholdTable.temp2_high_threshold)
		{
			Cl_Uint16Type var1;
			var1=dlsis_temp2;
			Cl_SendDatatoconsole(CON_TX_COMMAND_PRINTDATA,&var1,2);
			cl_temp_safestate();
		}
		else
		{
			Cl_SendDatatoconsole(CON_TX_COMMAND_PRINTTEXT,"ENTER_4",8);
			cl_temp_recoveredstate();
			cl_dlsis_state = CL_DLSIS_STATE_CRITICAL_ALARM;
		}
		break;
		case TEMP2_LOW_THRESHOLD:
		Cl_SendDatatoconsole(CON_TX_COMMAND_PRINTTEXT,"ENTER_3",8);
// 		temp = (cl_sys_statbuffer.Temp2status * 3300/4096);
// 		calibration_tmp(temp,TS2);
		dlsis_temp2 = cl_sys_statbuffer.Temp2status;
		if (dlsis_temp2 < Cl_alarmThresholdTable.temp2_low_threshold)
		{
			Cl_Uint16Type var1;
			var1=dlsis_temp2;
			Cl_SendDatatoconsole(CON_TX_COMMAND_PRINTDATA,&var1,2);
			cl_temp_safestate();
		}
		else
		{
			Cl_SendDatatoconsole(CON_TX_COMMAND_PRINTTEXT,"ENTER_4",8);
			cl_temp_recoveredstate();
			cl_dlsis_state = CL_DLSIS_STATE_CRITICAL_ALARM;
		}
		break;
		case TEMP1_HIGH_THRESHOLD:
		Cl_SendDatatoconsole(CON_TX_COMMAND_PRINTTEXT,"ENTER_3",8);
// 		temp = (cl_sys_statbuffer.Temp1status * 3300/4096);
// 		calibration_tmp(temp,TS1);
		dlsis_temp1 = cl_sys_statbuffer.Temp1status;
		dlsis_temp1 = temprature_final_value_1;
		if (dlsis_temp1 > Cl_alarmThresholdTable.temp1_high_threshold)
		{
			Cl_Uint16Type var1;
			var1=dlsis_temp1;
			Cl_SendDatatoconsole(CON_TX_COMMAND_PRINTDATA,&var1,2);
			cl_temp_safestate();
		}
		else
		{
			Cl_SendDatatoconsole(CON_TX_COMMAND_PRINTTEXT,"ENTER_4",8);
			cl_temp_recoveredstate();
			cl_dlsis_state = CL_DLSIS_STATE_CRITICAL_ALARM;
		}
		break;
		case TEMP1_LOW_THRESHOLD:
		Cl_SendDatatoconsole(CON_TX_COMMAND_PRINTTEXT,"ENTER_3",8);
// 		temp = cl_sys_statbuffer.Temp1status * 0.805;
// 		calibration_tmp(temp,TS1);
		dlsis_temp1 = cl_sys_statbuffer.Temp1status;
		if (dlsis_temp1 < Cl_alarmThresholdTable.temp1_low_threshold)
		{
			Cl_Uint16Type var1;
			var1=dlsis_temp1;
			Cl_SendDatatoconsole(CON_TX_COMMAND_PRINTDATA,&var1,2);
			cl_temp_safestate();
		}
		else
		{
			Cl_SendDatatoconsole(CON_TX_COMMAND_PRINTTEXT,"ENTER_4",8);
			cl_temp_recoveredstate();
			cl_dlsis_state = CL_DLSIS_STATE_CRITICAL_ALARM;
		}
		break;
	}
return CL_OK;
}

Cl_ReturnCodeType  cl_dlsis_translatemacevent(Cl_Mac_EventsType Cl_MacRinseEvt,Cl_Dlsis_Eevents* cl_dlsis_event)
{
	switch(Cl_MacRinseEvt)
	{
		case EVT_CONSOLE_COMMAND_CLR_ALARM:
		*cl_dlsis_event = EVENT_DLSIS_CLR_ALARM;	
		break;
		case 	EVT_CONSOLE_COMMAND_SET_BLDPUMPRATE:
		*cl_dlsis_event = EVENT_DLSIS_COMMAND_SET_BLDPUMPRATE;
		break;
		case EVT_CONSOLE_COMMAND_SET_BLDPMP_ON:
		*cl_dlsis_event = EVENT_DLSIS_COMMAND_SET_BLDPMP_ON;
		break;
		case EVT_CONSOLE_COMMAND_SET_BLDPMP_OFF:
		*cl_dlsis_event = EVENT_DLSIS_COMMAND_SET_BLDPMP_OFF;
		break;
	
		case EVT_CONSOLE_COMMAND_GET_DATA:
		*cl_dlsis_event = EVENT_DLSIS_GET_DATA;
		break;
		case EVT_CONSOLE_COMMAND_SET_DATA:
		*cl_dlsis_event = EVENT_DLSIS_SET_DATA;
		break;
		case EVT_CONSOLE_COMMAND_RINSE_START:
		*cl_dlsis_event= EVENT_DLSIS_START_RINSE;
		break;
		case  EVT_CONSOLE_COMMAND_DIALYSIS_START:
		*cl_dlsis_event= EVENT_DLSIS_START_DIALYSIS;
		break;
		case EVT_CONSOLE_COMMAND_DIALYSIS_STOP:
		*cl_dlsis_event= EVENT_DLSIS_STOP_DIALYSIS;
		break;
		case EVT_CONSOLE_COMMAND_DIALYSIS_PAUSE:
		*cl_dlsis_event= EVENT_DLSIS_PAUSE_DIALYSIS;
		break;
		case EVT_CONSOLE_COMMAND_BYPASS_ON:
		*cl_dlsis_event= EVENT_DLSIS_BYPASS_ON;
		break;
		case EVT_CONSOLE_COMMAND_BYPASS_OFF:
		*cl_dlsis_event= EVENT_DLSIS_BYPASS_OFF;
		break;
		case  EVT_TIMER_EXPIRED:
		
		break;
		case EVT_TICK_50M:
		*cl_dlsis_event =   EVENT_DLSIS_TICK_50MS;
		break;
		case EVT_TICK_500M:
		*cl_dlsis_event = EVENT_DLSIS_TICK_500MS;
		break;
		case EVT_TICK_SEC:
		*cl_dlsis_event =   EVENT_DLSIS_TICK_SECOND;
		break;
		case EVT_TICK_MIN:
		*cl_dlsis_event =   EVENT_DLSIS_TICK_MINUTE;
		break;
		case EVT_TICK_HOUR:
		*cl_dlsis_event =   EVENT_DLSIS_TICK_HOUR;
		break;
		case EVT_ALARM_TRIGGERED:
		*cl_dlsis_event =   EVENT_DLSIS_ALARM;		
		break;
		case EVT_ALERT_TRIGGERED:
		*cl_dlsis_event =   EVENT_DLSIS_ALERT;
		break;
		case EVT_CONSOLE_COMMAND_PATIENT_READY:
		
			*cl_dlsis_event =   EVENT_DLSIS__START_RETURN;

		break;
		case EVT_CONSOLE_COMMAND_START_RETURN:
			*cl_dlsis_event =   EVENT_DLSIS__STOP_RETURN;
		break;
		case EVT_CONSOLE_COMMAND_STOP_RETURN:
			*cl_dlsis_event =   EVENT_DLSIS__STOP_RETURN;
		break;
		case EVT_CONSOLE_COMMAND_MINUF_ON:
			*cl_dlsis_event =   EVENT_DLSIS__MINUF_ON;
		break;
		case EVT_CONSOLE_COMMAND_MINUF_OFF:
			*cl_dlsis_event =   EVENT_DLSIS__MINUF_OFF;
		break;
		case EVT_CONSOLE_COMMAND_HEP_PMP_RATE:
		*cl_dlsis_event =   EVENT_DLSIS_HEP_RATE;
		break;
		case EVT_CONSOLE_COMMAND_HEPARIN_START:
		*cl_dlsis_event =   EVENT_DLSIS_HEP_ON;
		break;
		case EVT_CONSOLE_COMMAND_HEPARIN_STOP:
		*cl_dlsis_event =   EVENT_DLSIS_HEP_OFF;
		break;
		case EVT_CONSOLE_COMMAND_SET_BLDPUMPINC_MANUAL:
		*cl_dlsis_event = EVT_CONSOLE_COMMAND_SET_BLDPUMPINC_MANUAL;
		break;
		case EVT_CONSOLE_COMMAND_SET_BLDPUMPDEC_MANUAL:
		*cl_dlsis_event = EVT_CONSOLE_COMMAND_SET_BLDPUMPDEC_MANUAL;
		break;
		case 	EVT_CONSOLE_COMMAND_HEPARIN_BOLUS:
		*cl_dlsis_event = EVENT_DLSIS_HEAPARIN_BOLUS;
		break;
		case EVT_CONSOLE_COMMAND_ISOUF_START:
		*cl_dlsis_event = EVENT_DLSIS_ISOUF_START;
		break;
		case EVT_CONSOLE_COMMAND_MUTE_ALARM:
		*cl_dlsis_event = EVENT_DLSIS_MUTE;
		break;
		case EVT_CONSOLE_COMMAND_DIALYSIS_ISOUF_ON :
		*cl_dlsis_event = EVENT_DLSIS_DIALYSIS_ISOUF_ON;
		break;	
		case EVT_CONSOLE_COMMAND_DIALYSIS_ISOUF_OFF :
		*cl_dlsis_event = EVENT_DLSIS_DIALYSIS_ISOUF_OFF;
		break;	
		default:
		
		break;
	}
	return CL_OK;
}



Cl_ReturnCodeType Cl_Dlsis_UpdateTimeInfo(void)
{
	
	Cl_ReturnCodeType Cl_dlsisretcode = CL_ERROR;
	Cl_ConsoleTxCommandtype command = CON_TX_COMMAND_COMMAND_MAX;
	Cl_Uint8Type data[7] = {0,0,0,0 ,0,0};
	
	command = CON_TX_COMMAND_REM_TIME;
	data[0] = (Cl_Uint8Type) DIALYSIS_DATA;
	data[1]= (Cl_Uint8Type)DlsisTime.Cl_dlsisTotalMinutescounter;
	data[2]= (Cl_Uint8Type)DlsisTime.Cl_dlsisTotalhourscounter;
	data[3]= (Cl_Uint8Type)DlsisTime.Cl_dlsissecondscounter;
	data[4]= (Cl_Uint8Type) (CL_DLSIS_TIMEOUT_MIN - DlsisTime.Cl_dlsisTotalMinutescounter );
	data[5]= (Cl_Uint8Type) (CL_DLSIS_TIMEOUT_HRS - DlsisTime.Cl_dlsisTotalhourscounter );
	data[6]= (Cl_Uint8Type) (60 - DlsisTime.Cl_dlsissecondscounter );
	
	Cl_dlsisretcode = Cl_SendDatatoconsole(command,&data,7);
	
	return CL_OK;
	}
	
Cl_ReturnCodeType cl_Dlsis_checkforHeparinCompletion(void){
		
		
		 Cl_ReturnCodeType Cl_dlsisretcode = CL_ERROR;
		 
		 if(DlsisTime.Cl_dlsisTotalMinutescounter > (Treatdata[ID_treattime] - 60) )
		 {
			 
			 Cl_dlsisretcode = CL_OK;
		 }
		 
		 return (Cl_dlsisretcode);
		
	}
	
	
	 Cl_ReturnCodeType cl_Dlsis_CheckforDialysisCompletion(void)
	 {
		 Cl_ReturnCodeType Cl_dlsisretcode = CL_ERROR;
		 
		 if(DlsisTime.Cl_dlsisTotalMinutescounter > Treatdata[ID_treattime] )
		 {
			 
			 Cl_dlsisretcode = CL_OK;
		 }
		 
		 return (Cl_dlsisretcode);
		 
	 }
	
	Cl_ReturnCodeType cl_Dlsis_notifydacandgotorinsestandby(void)
	{
		Cl_ReturnCodeType 	Cl_dlsisretcode = CL_OK;
		
		Cl_Uint8Type data =0;
		//inform DAC about rinse completed state.
		DlsisTime.Cl_dlsissecondscounter = 0;
		DlsisTime.Cl_dlsisMinutescounter= 0;
		DlsisTime.Cl_dlsishourscounter= 0;
		DlsisTime.Cl_dlsisTotalMinutescounter= 0;
		DlsisTime.Cl_dlsisTotalhourscounter=0;
		
		
		Cl_dlsisretcode = Cl_SendDatatoconsole(CON_TX_COMMAND_DIALYS_COMPLTED,&data,0);
		Cl_dlsisretcode = Cl_mac_apprequesthandler(MACREQ_DIALYSIS_COMPLETED);
		
		return Cl_dlsisretcode;
	}
	
	
Cl_ReturnCodeType Cl_Dlsis_UpdateAlarmTable(ClDlsisAlarmIdType * ClRinseAlarmId )
{
	Cl_ReturnCodeType 	Cl_dlsisretcode = CL_OK;
			Cl_NewAlarmIdType cl_alarmId;
	Cl_Uint8Type tempcount = 0;
			Cl_Uint8Type data[2];
			Cl_ConsoleTxCommandtype command = CON_TX_COMMAND_COMMAND_MAX;
	
	for (tempcount = 0 ; tempcount < CL_DLSIS_ALRM_MAX ; tempcount++)
	{
		
		Cl_Alarm_GetAlarmStatus(Cl_DlsisAlarmTable[tempcount].Cl_DlsisAlarmId,&Cl_DlsisAlarmTable[tempcount].IsActive);
		if (Cl_DlsisAlarmTable[tempcount].IsActive)
		{
			if(!Cl_DlsisAlarmTable[tempcount].IsRaised)
				{
				//	cl_alarmId = (Cl_Uint8Type)Cl_DlsisAlarmTable[tempcount].Cl_DlsisAlarmId;

				//	data[0] = (Cl_Uint8Type)Cl_DlsisAlarmTable[tempcount].Cl_DlsisAlarmId;
				//	data[1] = (Cl_Uint8Type) CL_ALARM_TRIGGERED;
				//	data[1] = (Cl_Uint8Type) CRITICAL;
				//	command = CON_TX_COMMAND_ALARM;
				//	Cl_dlsisretcode = Cl_SendDatatoconsole(command,&data,3);
					Cl_DlsisAlarmTable[tempcount].IsRaised = true;
					Cl_dlsisretcode = CL_DlsisAlarmActon(Cl_DlsisAlarmTable[tempcount].Cl_DlsisAlarmId);
					
				
				}
			
		}
		else
			{
				// alarm was present before , but not active now.
				if(Cl_DlsisAlarmTable[tempcount].IsRaised == true)
				{
					Cl_DlsisAlarmTable[tempcount].IsRaised = false;
					command = CON_TX_COMMAND_ALARM;
					data[0] = (Cl_Uint8Type)(Cl_DlsisAlarmTable[tempcount].Cl_DlsisAlarmId);
					data[1] = (Cl_Uint8Type) 0;
					Cl_dlsisretcode = Cl_SendDatatoconsole(command,&data,2);
				}

			}
		
	
		
		

	}
	

	

return (Cl_dlsisretcode );
}


Cl_ReturnCodeType  CL_DlsisAlarmActon(Cl_NewAlarmIdType cl_dlsisalarmid)
{
		Cl_ReturnCodeType 	Cl_dlsisretcode = CL_OK;
			Cl_Uint8Type data[3] ={0, 0} ,*data1 = NULL;
			Cl_Uint8Type data3;
			Cl_ConsoleTxCommandtype command = CON_TX_COMMAND_COMMAND_MAX;
			Cl_Uint16Type levelswitchstatus = 0;	
			static Cl_Uint8Type fillseccounter = 0;
			Cl_NewAlarmIdType NewAlarmId = _NO_ALARM;
		

			

							
		switch(cl_dlsisalarmid)
		{
			//case BP_STOPPED:
			
			case POWER_FAILURE: 
			
			cl_dlsis_powerfailure = 1;
			
			sv_cntrl_setredalarm();
			sv_cntrl_buzzer();
			Cl_Dlsis_StopDialysis();
			cl_dlsis_state = CL_DLSIS_STATE_CRITICAL_ALARM;
			
			break;
			
			
			case BLOODDOOR_STATUS_OPEN:
			case APTSTATUS_LOW:
			case APTSTATUS_HIGH:
			case VPTSTATUS_LOW:
			case VPTSTATUS_HIGH:
			case ABD_EVENT:
			case BLD_EVENT:
			case BP_STOPPED:
			case TMP_HIGH:
			case TMP_LOW:
			case BLOOD_NOT_DETECTED:
			case HP_OVERRUN:
			case HP_UNDERRUN:
			case BP_OVERRUN:
			case BP_UNDERRUN:
			//case BD_EVENT:                  // manav02 added
							
				sv_cntrl_setredalarm();
				sv_cntrl_buzzer();
				Cl_Dlsis_safesate();	
			break;
			
		//	case POWER_FAILURE:            // manav02 added
			case ACID_IN_CLOSED:
			case BICARB_IN_CLOSED:
			case HOLDER1STATUS_CLOSED:
			case HOLDER2STATUS_CLOSED:
			case TEMP2_HIGH_THRESHOLD:
			case TEMP2_LOW_THRESHOLD:
			case TEMP3_HIGH_THRESHOLD:
			case TEMP3_LOW_THRESHOLD:
				sv_cntrl_setredalarm();
				sv_cntrl_buzzer();
				cl_temp_safestate();
				
			break;
			// manav05 action from safestate to stop dialysis
			case 	PS1_HIGH_THRESHOLD:
			case    PS1_LOW_THRESHOLD:
			case 	PS2_HIGH_THRESHOLD:
			case    PS2_LOW_THRESHOLD:
			case 	PS3_HIGH_THRESHOLD:
			case    PS3_LOW_THRESHOLD:
			
			sv_cntrl_setredalarm();
			sv_cntrl_buzzer();
			Cl_Dlsis_StopDialysis();
			cl_dlsis_state = CL_DLSIS_STATE_STOPPED;//CL_DLSIS_STATE_CRITICAL_ALARM;   // manav05 changed
			
			
			break;
				
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
				sv_cntrl_setredalarm();
				sv_cntrl_buzzer();
				Cl_Dlsis_StopDialysis();
				cl_dlsis_state = CL_DLSIS_STATE_STOPPED;//CL_DLSIS_STATE_CRITICAL_ALARM;   // manav05 changed
		    	//cl_dlsis_state = CL_DLSIS_STATE_CRITICAL_ALARM;
			break;
			
			case COND_STATUS_LOW:
			case COND_STATUS_HIGH:
				sv_cntrl_setredalarm();           //manav02 changed from yellow to red
				sv_cntrl_buzzer();
				cl_temp_safestate();
				//cl_cond_safestate();
			break;
			
		
			case DE_CHAMBER_LOW: //23
			break;
			case SYSTEM_NOT_READY: //24
			break;
			

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
		 if(cl_dlsisalarmid != _NO_ALARM)
		 {
			 dlsisalarm = cl_dlsisalarmid;
			 dlsisalarm1 = dlsisalarm;
			data[0] = (Cl_Uint8Type)cl_dlsisalarmid;
			data[1] = (Cl_Uint8Type) CL_ALARM_TRIGGERED;
			data[2] = (Cl_Uint8Type)CRITICAL;
			command = CON_TX_COMMAND_ALARM;
			cl_wait(10);
			Cl_dlsisretcode = Cl_SendDatatoconsole(command,&data,3);
			
					
			command = CON_TX_COMMAND_CL_SYS_STATE;
			data[0] = (Cl_Uint8Type)CLEANING;
			data[1] = (Cl_Uint8Type)CLEAN_CLEAN_ALARM_STOPPED;
			Cl_dlsisretcode = Cl_SendDatatoconsole(command,&data,2);
		 }
return CL_OK;			
}

Cl_ReturnCodeType Cl_Dlsis_deactivateallalarms(){
	
	Cl_ReturnCodeType Cl_dlsisretcode = CL_OK;
	
	Cl_Dlsis_ResetAlertsforReassertion();
	
	Cl_dlsisretcode =  Cl_AlarmActivateAlarms(LEVELSWITCH_OFF_TO_ON,true );
									Cl_dlsisretcode =  Cl_AlarmActivateAlarms(LEVELSWITCH_ON_TO_OFF ,false);
									Cl_dlsisretcode =  Cl_AlarmActivateAlarms( HOLDER1STATUS_CLOSED,false );
									Cl_dlsisretcode =  Cl_AlarmActivateAlarms( HOLDER2STATUS_CLOSED,false );
									Cl_dlsisretcode =  Cl_AlarmActivateAlarms( HOLDER1STATUS_OPEN,false );
									Cl_dlsisretcode =  Cl_AlarmActivateAlarms( HOLDER2STATUS_OPEN,false );
									Cl_dlsisretcode =  Cl_AlarmActivateAlarms(COND_STATUS_LOW,false );
									Cl_dlsisretcode =  Cl_AlarmActivateAlarms(COND_STATUS_HIGH,false );
								//	Cl_dlsisretcode =  Cl_AlarmActivateAlarms(COND_DAC_OPEN,true );
								//	Cl_dlsisretcode =  Cl_AlarmActivateAlarms(COND_DAC_RO,true );
								//	Cl_dlsisretcode =  Cl_AlarmActivateAlarms(COND_DAC_HIGH,true );
									Cl_dlsisretcode =  Cl_AlarmActivateAlarms( ABD_EVENT,false );
									Cl_dlsisretcode =  Cl_AlarmActivateAlarms( BD_EVENT,false );
									Cl_dlsisretcode =  Cl_AlarmActivateAlarms(APTSTATUS_HIGH,false );
									Cl_dlsisretcode =  Cl_AlarmActivateAlarms(VPTSTATUS_HIGH ,false);
									Cl_dlsisretcode =  Cl_AlarmActivateAlarms(APTSTATUS_LOW,false );
									Cl_dlsisretcode =  Cl_AlarmActivateAlarms(VPTSTATUS_LOW,false);
									Cl_dlsisretcode =  Cl_AlarmActivateAlarms( BLD_EVENT,true );
									Cl_dlsisretcode =  Cl_AlarmActivateAlarms(PS2_HIGH_THRESHOLD,false );
									Cl_dlsisretcode =  Cl_AlarmActivateAlarms(PS2_LOW_THRESHOLD,false );
									Cl_dlsisretcode =  Cl_AlarmActivateAlarms(PS1_HIGH_THRESHOLD,false );
									Cl_dlsisretcode =  Cl_AlarmActivateAlarms(PS1_LOW_THRESHOLD,false );
									Cl_dlsisretcode =  Cl_AlarmActivateAlarms(PS3_HIGH_THRESHOLD ,false);
									Cl_dlsisretcode =  Cl_AlarmActivateAlarms(PS3_LOW_THRESHOLD ,false);
									
									Cl_dlsisretcode =  Cl_AlarmActivateAlarms(SENSOR_TEMP3STATUS,false );
									Cl_dlsisretcode =  Cl_AlarmActivateAlarms(SENSOR_TEMP2STATUS,false );
									
									Cl_dlsisretcode =  Cl_AlarmActivateAlarms(FLOW_NO_FLOW,false );
									Cl_dlsisretcode =  Cl_AlarmActivateAlarms(FLOW_LOW_FLOWRATE,false );
									Cl_dlsisretcode =  Cl_AlarmActivateAlarms(FLOW_HIGH_FLOWRATE,false );
									
									Cl_dlsisretcode =  Cl_AlarmActivateAlarms(TEMP1_HIGH_THRESHOLD,false );
									Cl_dlsisretcode =  Cl_AlarmActivateAlarms(TEMP1_LOW_THRESHOLD,false );
									Cl_dlsisretcode =  Cl_AlarmActivateAlarms(TEMP2_HIGH_THRESHOLD,false );
									Cl_dlsisretcode =  Cl_AlarmActivateAlarms(TEMP2_LOW_THRESHOLD,false );
									Cl_dlsisretcode =  Cl_AlarmActivateAlarms(TEMP3_HIGH_THRESHOLD,false );
									Cl_dlsisretcode =  Cl_AlarmActivateAlarms(TEMP3_LOW_THRESHOLD,false );
									Cl_dlsisretcode =  Cl_AlarmActivateAlarms(BP_STOPPED,false );
									Cl_dlsisretcode =  Cl_AlarmActivateAlarms(TMP_HIGH,false );
									Cl_dlsisretcode =  Cl_AlarmActivateAlarms(TMP_LOW,false );
									Cl_dlsisretcode =  Cl_AlarmActivateAlarms(BC_ALARM,false );
									
	
	
	return CL_OK;
}

Cl_ReturnCodeType Cl_Dlsis_StopDialysis(void)
{
	Cl_Uint8Type data;
	Cl_ReturnCodeType Cl_dlsisretcode = CL_OK;
	Cl_Dlsis_deactivateallalarms();
				if(!((cl_dlsis_state == CL_DLSIS_STATE_IDLE ) || (cl_dlsis_state == CL_DLSIS_STATE_STOPPED )|| (cl_dlsis_state == CL_DLSIS_STATE_PAUSED ) ||(cl_dlsis_state == CL_DLSIS_STATE_CRITICAL_ALARM )  ))
				{

					
					 	DlsisTime.Cl_dlsissecondscounter = 0;
					 	DlsisTime.Cl_dlsisMinutescounter= 0;
					 	DlsisTime.Cl_dlsishourscounter= 0;
					 	DlsisTime.Cl_dlsisTotalMinutescounter= 0;
					 	DlsisTime.Cl_dlsisTotalhourscounter=0;

				Cl_dlsisretcode =  sv_cntrl_deactivatevenousclamp();
				Cl_dlsisretcode =  sv_cntrl_deactivatepump(UFPUMP);
				Cl_dlsisretcode = sv_cntrl_setflowpath(FLOW_PATH_IDLE_RINSE);
				Cl_dlsisretcode = sv_cntrl_poweroffheater();
				Cl_dlsisretcode = SetHeaterState(CL_HEATER_STATE_OFF);
				cl_bp_controller(CL_BP_EVENT_STOP,0);
				Cl_dlsisretcode =  Cl_AlarmActivateAlarms(BP_OVERRUN,false );
				Cl_dlsisretcode =  Cl_AlarmActivateAlarms(BP_UNDERRUN,false );
				bpFlag = 0 ;
				bpflagCounter = 0;
				Cl_dlsisretcode = sv_cntrl_setflowpath(FLOW_PATH_IDLE_RINSE);
				cl_dlsis_state = CL_DLSIS_STATE_IDLE;
				Cl_dlsisretcode = Cl_SendDatatoconsole(CON_TX_COMMAND_DLSIS_STOPPED,&data,0);
				Cl_bc_controller(BC_EVENT_STOP);
				bc_flag = 0;
					
				}
				
				// deactivate all alarms
				
				return CL_OK;
}

Cl_ReturnCodeType Cl_Dlsis_BypassDialysis(void)
{
			Cl_Uint8Type data;
			Cl_ReturnCodeType Cl_dlsisretcode = CL_OK;
			Cl_dlsisretcode = sv_cntrl_enable_bypass();
				cl_uf_controller(CL_UF_EVENT_STOP,0);
				isouf = false;
			//Cl_dlsisretcode =  sv_cntrl_deactivatepump(UFPUMP);
			
			Cl_dlsissecondscounter_1 = DlsisTime.Cl_dlsissecondscounter;
			Cl_dlsishourscounter_1 = DlsisTime.Cl_dlsishourscounter;
			Cl_dlsisMinutescounter_1 = DlsisTime.Cl_dlsisMinutescounter;
			Cl_dlsisTotalMinutescounter_1 = DlsisTime.Cl_dlsisTotalMinutescounter;
			Cl_dlsisTotalhourscounter_1 = DlsisTime.Cl_dlsisTotalhourscounter;
// 			DlsisTime.Cl_dlsissecondscounter = 0;
// 			DlsisTime.Cl_dlsishourscounter = 0;
// 			DlsisTime.Cl_dlsisMinutescounter = 0;
// 			DlsisTime.Cl_dlsisTotalhourscounter = 0;
// 			 DlsisTime.Cl_dlsisTotalMinutescounter = 0;
		//	 Cl_dlsisretcode =  Cl_AlarmActivateAlarms(BLOODDOOR_STATUS_OPEN,false );
			 Cl_dlsisretcode =  Cl_AlarmActivateAlarms(APTSTATUS_HIGH,false );
			 Cl_dlsisretcode =  Cl_AlarmActivateAlarms(APTSTATUS_LOW,false );
		Cl_dlsisretcode =  Cl_AlarmActivateAlarms(VPTSTATUS_HIGH,false );
		Cl_dlsisretcode =  Cl_AlarmActivateAlarms(VPTSTATUS_LOW,false );
	Cl_dlsisretcode =  Cl_AlarmActivateAlarms(PS1_HIGH_THRESHOLD,false );
	Cl_dlsisretcode =  Cl_AlarmActivateAlarms(PS1_LOW_THRESHOLD,false );
	Cl_dlsisretcode =  Cl_AlarmActivateAlarms(PS2_HIGH_THRESHOLD,false );
	Cl_dlsisretcode =  Cl_AlarmActivateAlarms(PS2_LOW_THRESHOLD,false );
	Cl_dlsisretcode =  Cl_AlarmActivateAlarms(PS3_HIGH_THRESHOLD,false );
	Cl_dlsisretcode =  Cl_AlarmActivateAlarms(PS3_LOW_THRESHOLD,false );
		//	 Cl_dlsisretcode =  Cl_AlarmActivateAlarms(LEVELSWITCH_OFF_TO_ON,false );
		//	 Cl_dlsisretcode =  Cl_AlarmActivateAlarms(LEVELSWITCH_ON_TO_OFF ,false);
			 Cl_dlsisretcode =  Cl_AlarmActivateAlarms( HOLDER1STATUS_CLOSED,false );
			 Cl_dlsisretcode =  Cl_AlarmActivateAlarms( HOLDER2STATUS_CLOSED,false );
			 Cl_dlsisretcode =  Cl_AlarmActivateAlarms( HOLDER1STATUS_OPEN,false );
			 Cl_dlsisretcode =  Cl_AlarmActivateAlarms( HOLDER2STATUS_OPEN,false );
			 Cl_dlsisretcode =  Cl_AlarmActivateAlarms(COND_STATUS_LOW,false );
			 Cl_dlsisretcode =  Cl_AlarmActivateAlarms(COND_STATUS_HIGH,false );
			 //Cl_dlsisretcode =  Cl_AlarmActivateAlarms( ABD_EVENT,true );
			 Cl_dlsisretcode =  Cl_AlarmActivateAlarms(ACID_IN_OPEN,false);
			 Cl_dlsisretcode =  Cl_AlarmActivateAlarms(BICARB_IN_OPEN,false );
			 Cl_dlsisretcode =  Cl_AlarmActivateAlarms( BD_EVENT,false );
			 //Cl_dlsisretcode =  Cl_AlarmActivateAlarms(BLD_EVENT,true );
			 Cl_dlsisretcode =  Cl_AlarmActivateAlarms(TEMP3_HIGH_THRESHOLD,false );
			 Cl_dlsisretcode =  Cl_AlarmActivateAlarms(TEMP3_LOW_THRESHOLD,false );
			 Cl_dlsisretcode =  Cl_AlarmActivateAlarms(BP_STOPPED,false); 
			 Cl_dlsisretcode =  Cl_AlarmActivateAlarms(TMP_HIGH,false );
	  Cl_dlsisretcode =  Cl_AlarmActivateAlarms(TMP_LOW,false );
			 
			cl_dlsis_state = CL_DLSIS_STATE_BYPASS;
						
			Cl_dlsisretcode = Cl_SendDatatoconsole(CON_TX_COMMAND_DLSIS_BYPASS_ON,&data,0);
	cl_wait(20);   // manav03 added
			Cl_dlsisretcode = Cl_SendDatatoconsole(CON_TX_COMMAND_DLSIS_PAUSED,&data,0);
return CL_OK;	
}

Cl_ReturnCodeType Cl_Dlsis_BypassOff(void)
{
	Cl_Uint8Type data;
	Cl_ReturnCodeType Cl_dlsisretcode = CL_OK;
	Cl_dlsisretcode = sv_cntrl_disable_bypass();
	
	 DlsisTime.Cl_dlsissecondscounter = Cl_dlsissecondscounter_1 ;
	 DlsisTime.Cl_dlsishourscounter = Cl_dlsishourscounter_1 ;
	 DlsisTime.Cl_dlsisMinutescounter = Cl_dlsisMinutescounter_1 ;
	DlsisTime.Cl_dlsisTotalMinutescounter =  Cl_dlsisTotalMinutescounter_1;
	  DlsisTime.Cl_dlsisTotalhourscounter = Cl_dlsisTotalhourscounter_1 ;
	  Cl_dlsisretcode =  Cl_AlarmActivateAlarms(BLOODDOOR_STATUS_OPEN,true );
	  Cl_dlsisretcode =  Cl_AlarmActivateAlarms(LEVELSWITCH_OFF_TO_ON,true );
	  Cl_dlsisretcode =  Cl_AlarmActivateAlarms(LEVELSWITCH_ON_TO_OFF ,true);
	  Cl_dlsisretcode =  Cl_AlarmActivateAlarms( HOLDER1STATUS_CLOSED,true );
	  Cl_dlsisretcode =  Cl_AlarmActivateAlarms( HOLDER2STATUS_CLOSED,true );
	  Cl_dlsisretcode =  Cl_AlarmActivateAlarms( HOLDER1STATUS_OPEN,false );
	  Cl_dlsisretcode =  Cl_AlarmActivateAlarms( HOLDER2STATUS_OPEN,false );
	  Cl_dlsisretcode =  Cl_AlarmActivateAlarms(COND_STATUS_LOW,true );
	  Cl_dlsisretcode =  Cl_AlarmActivateAlarms(COND_STATUS_HIGH,true );
	  //Cl_dlsisretcode =  Cl_AlarmActivateAlarms( ABD_EVENT,true );
	  Cl_dlsisretcode =  Cl_AlarmActivateAlarms(ACID_IN_CLOSED,true );
	  Cl_dlsisretcode =  Cl_AlarmActivateAlarms(BICARB_IN_CLOSED,true );
	  Cl_dlsisretcode =  Cl_AlarmActivateAlarms(ACID_IN_OPEN,false );
	  Cl_dlsisretcode =  Cl_AlarmActivateAlarms(BICARB_IN_OPEN,false );
	  Cl_dlsisretcode =  Cl_AlarmActivateAlarms( BD_EVENT,false );
	  //Cl_dlsisretcode =  Cl_AlarmActivateAlarms(BLD_EVENT,true );
	  Cl_dlsisretcode =  Cl_AlarmActivateAlarms(TEMP3_HIGH_THRESHOLD,true );
	  Cl_dlsisretcode =  Cl_AlarmActivateAlarms(TEMP3_LOW_THRESHOLD,true );
	  
	  Cl_dlsisretcode =  Cl_AlarmActivateAlarms(PS1_HIGH_THRESHOLD,true );
	  Cl_dlsisretcode =  Cl_AlarmActivateAlarms(PS1_LOW_THRESHOLD,true );
	  Cl_dlsisretcode =  Cl_AlarmActivateAlarms(PS2_HIGH_THRESHOLD,true );
	  Cl_dlsisretcode =  Cl_AlarmActivateAlarms(PS2_LOW_THRESHOLD,true );
	  Cl_dlsisretcode =  Cl_AlarmActivateAlarms(PS3_HIGH_THRESHOLD,true );
	  Cl_dlsisretcode =  Cl_AlarmActivateAlarms(PS3_LOW_THRESHOLD,true );
	  Cl_dlsisretcode =  Cl_AlarmActivateAlarms(BP_STOPPED,true );
	  Cl_dlsisretcode =  Cl_AlarmActivateAlarms(TMP_HIGH,true );
	  Cl_dlsisretcode =  Cl_AlarmActivateAlarms(TMP_LOW,true );
	  Cl_dlsisretcode =  Cl_AlarmActivateAlarms(BC_ALARM,true );
	  
	  
	//Cl_dlsisretcode =  sv_cntrl_activatepump(UFPUMP);
	cl_dlsis_state = CL_DLSIS_STATE_WAIT_FOR_TEMP_COND_STABILITY;//CL_DLSIS_STATE_DIALYSIS;
 	Cl_dlsisretcode = Cl_SendDatatoconsole(CON_TX_COMMAND_DLSIS_BYPASS_OFF,&data,0);
	 cl_wait(20);
 	Cl_dlsisretcode = Cl_SendDatatoconsole(CON_TX_COMMAND_DLSIS_RESUMED,&data,0);
 		Cl_Dlsis_UpdateTimeInfo();
 	bypass_flag = 1;
// 	
 	cl_uf_controller(CL_UF_EVENT_START,0);
// 	cl_ufpumpFeedback_start();
// 	
return CL_OK;
}


Cl_ReturnCodeType Cl_Dlsis_PauseDialysis(void)
{
		Cl_Uint8Type data;
		Cl_ReturnCodeType Cl_dlsisretcode = CL_OK;
		
		if(!(cl_dlsis_state == CL_DLSIS_STATE_IDLE ) )
		
		switch (cl_dlsis_state)
		{
			case CL_DLSIS_STATE_WAIT_FOR_TEMP_COND_STABILITY:
			case CL_DLSIS_STATE_WAIT_FOR_BLOOD_DETECT:
			case CL_DLSIS_STATE_DIALYSIS:
			case CL_DLSIS_STATE_UF_ACTIVATION:
			
	
			Cl_dlsisretcode = sv_cntrl_enable_bypass();
					
					//	Cl_dlsisretcode =  sv_cntrl_deactivatevenousclamp();
					//	Cl_dlsisretcode = sv_cntrl_setflowpath(FLOW_PATH_IDLE_RINSE);
					//	Cl_dlsisretcode = sv_cntrl_poweroffheater();
					//	Cl_dlsisretcode = SetHeaterState(CL_HEATER_STATE_OFF);
					//	cl_bp_controller(CL_BP_EVENT_STOP,0);
					//	Cl_dlsisretcode = sv_cntrl_setflowpath(FLOW_PATH_IDLE_RINSE);
						cl_dlsis_state = CL_DLSIS_STATE_PAUSED;
						Cl_dlsisretcode = Cl_SendDatatoconsole(CON_TX_COMMAND_DLSIS_PAUSED,&data,0);
			
			break;
			default:break;
		}
return CL_OK;
	
}
Cl_ReturnCodeType Cl_Dlsis_ProcessAlarms( )
{
		Cl_ReturnCodeType 	Cl_dlsisretcode = CL_OK;
		ClDlsisAlarmIdType  ClDlsisAlarmId;
		Cl_Uint8Type data;
		Cl_ConsoleTxCommandtype command = CON_TX_COMMAND_COMMAND_MAX;
		Cl_NewAlarmIdType cl_alarmid;
		
	   Cl_dlsisretcode = Cl_Dlsis_UpdateAlarmTable(&ClDlsisAlarmId);
	//   	Cl_rinseretcode =	Cl_Alarm_GetLastAlarm(&cl_rinsealarmid);
	//   	data = (Cl_Uint8Type)cl_rinsealarmid;
	//   	command = CON_TX_COMMAND_ALARM;
	//   	Cl_rinseretcode = Cl_SendDatatoconsole(command,&data,0);
		   
		   return (Cl_dlsisretcode );
	   	
}


Cl_ReturnCodeType  Cl_DlsisFillingFlowOn(void )
{
	Cl_ReturnCodeType Cl_RetVal = CL_OK;
	sv_cntrl_activate_valve(VALVE_ID1);
	//cl_gfillinprogress = true;
	return Cl_RetVal;
	
}

Cl_ReturnCodeType  Cl_DlsisFillingFlowOff(void )
{
	Cl_ReturnCodeType Cl_RetVal = CL_OK;
	sv_cntrl_deactivate_valve(VALVE_ID1);
	//cl_gfillinprogress = false;
	return Cl_RetVal;
	
}
Cl_ReturnCodeType Cl_Dlsis_ResetAlertsforReassertion(void )
{
	Cl_ReturnCodeType 	Cl_dlsisretcode = CL_OK;
	ClDlsisAlarmIdType ClDlsisAlarmId;
	Cl_Uint8Type data;
	Cl_ConsoleTxCommandtype command = CON_TX_COMMAND_COMMAND_MAX;
	Cl_NewAlarmIdType cl_dlsisalarmid;
	
	Cl_DlsisAlarmTable[CL_DLSIS_ALARM_HOLDER1STATUS_CLOSE].IsRaised = false;
	Cl_DlsisAlarmTable[CL_DLSIS_ALARM_HOLDER2STATUS_CLOSE].IsRaised = false;
	Cl_AlarmResetAlarm(HOLDER1STATUS_CLOSED);
	Cl_AlarmResetAlarm(HOLDER2STATUS_CLOSED);
	Cl_DlsisAlarmTable[CL_DLSIS_ALARM_BLOODDOOR_STATUS_OPEN].IsRaised = false;
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
	Cl_DlsisAlarmTable[CL_Dlsis_ALARM_PS1_HIGH_THRESHOLD].IsRaised = false;
	Cl_DlsisAlarmTable[CL_Dlsis_ALARM_PS1_LOW_THRESHOLD].IsRaised = false;
	Cl_AlarmResetAlarm(PS1_HIGH_THRESHOLD);
	Cl_AlarmResetAlarm(PS1_LOW_THRESHOLD);
	Cl_DlsisAlarmTable[CL_Dlsis_ALARM_PS2_HIGH_THRESHOLD].IsRaised = false;
	Cl_DlsisAlarmTable[CL_Dlsis_ALARM_PS2_LOW_THRESHOLD].IsRaised = false;
	Cl_AlarmResetAlarm(PS2_HIGH_THRESHOLD);
	Cl_AlarmResetAlarm(PS2_LOW_THRESHOLD);
	Cl_DlsisAlarmTable[CL_Dlsis_ALARM_PS3_HIGH_THRESHOLD].IsRaised = false;
	Cl_DlsisAlarmTable[CL_Dlsis_ALARM_PS3_LOW_THRESHOLD].IsRaised = false;
	Cl_AlarmResetAlarm(PS3_HIGH_THRESHOLD);
	Cl_AlarmResetAlarm(PS3_LOW_THRESHOLD);
	Cl_DlsisAlarmTable[CL_DLSIS_BDSTATUS].IsRaised = false;
	Cl_AlarmResetAlarm(BD_EVENT);
	Cl_DlsisAlarmTable[CL_DLSIS_ABDSTATUS].IsRaised = false;
	Cl_AlarmResetAlarm(ABD_EVENT);
	Cl_DlsisAlarmTable[CL_DLSIS_BLDSTATUS].IsRaised = false;
	Cl_AlarmResetAlarm(BLD_EVENT);
	Cl_DlsisAlarmTable[CL_DLSIS_APTSTATUS_HIGH].IsRaised = false;
	Cl_DlsisAlarmTable[CL_DLSIS_APTSTATUS_LOW].IsRaised = false;
	Cl_AlarmResetAlarm(APTSTATUS_HIGH);
	Cl_AlarmResetAlarm(APTSTATUS_LOW);
	Cl_DlsisAlarmTable[CL_DLSIS_VPTSTATUS_HIGH].IsRaised = false;
	Cl_DlsisAlarmTable[CL_DLSIS_VPTSTATUS_LOW].IsRaised = false;
	Cl_AlarmResetAlarm(VPTSTATUS_HIGH);
	Cl_AlarmResetAlarm(VPTSTATUS_LOW);
	Cl_DlsisAlarmTable[CL_DLSIS_COND_STATUS_LOW].IsRaised = false;
	Cl_DlsisAlarmTable[CL_DLSIS_COND_STATUS_HIGH].IsRaised = false;
	Cl_AlarmResetAlarm(COND_STATUS_LOW);
	Cl_AlarmResetAlarm(COND_STATUS_HIGH);
	Cl_DlsisAlarmTable[CL_Dlsis_ALARM_TEMP3_HIGH_THRESHOLD].IsRaised = false;
	Cl_DlsisAlarmTable[CL_Dlsis_ALARM_TEMP3_LOW_THRESHOLD].IsRaised = false;
	Cl_AlarmResetAlarm(TEMP3_HIGH_THRESHOLD);
	Cl_AlarmResetAlarm(TEMP3_LOW_THRESHOLD);
	Cl_DlsisAlarmTable[CL_Dlsis_ALARM_TEMP2_HIGH_THRESHOLD].IsRaised = false;
	Cl_DlsisAlarmTable[CL_Dlsis_ALARM_TEMP2_LOW_THRESHOLD].IsRaised = false;
	Cl_AlarmResetAlarm(TEMP2_HIGH_THRESHOLD);
	Cl_AlarmResetAlarm(TEMP2_LOW_THRESHOLD);
	Cl_DlsisAlarmTable[CL_Dlsis_ALARM_TEMP1_HIGH_THRESHOLD].IsRaised = false;
	Cl_DlsisAlarmTable[CL_Dlsis_ALARM_TEMP1_LOW_THRESHOLD].IsRaised = false;
	Cl_AlarmResetAlarm(TEMP1_HIGH_THRESHOLD);
	Cl_AlarmResetAlarm(TEMP1_LOW_THRESHOLD);
	Cl_DlsisAlarmTable[CL_Dlsis_ALARM_BP_STOPPED].IsRaised = false;
	Cl_AlarmResetAlarm(BP_STOPPED);
	Cl_DlsisAlarmTable[CL_Dlsis_ALARM_POWER_FAILURE].IsRaised = false;
	Cl_AlarmResetAlarm(POWER_FAILURE);
	
	Cl_DlsisAlarmTable[CL_DLSIS_ALARM_ACID_IN].IsRaised = false;
	Cl_AlarmResetAlarm(ACID_IN_CLOSED);
	
	Cl_DlsisAlarmTable[CL_DLSIS_ALARM_BICAR_IN].IsRaised = false;
	Cl_AlarmResetAlarm(BICARB_IN_CLOSED);
	
	Cl_DlsisAlarmTable[CL_Dlsis_ALARM_TMP_HIGH].IsRaised = false;
	Cl_AlarmResetAlarm(TMP_HIGH);
	Cl_DlsisAlarmTable[CL_Dlsis_ALARM_TMP_LOW].IsRaised = false;
	Cl_AlarmResetAlarm(TMP_LOW);
	Cl_DlsisAlarmTable[CL_Dlsis_ALARM_BCALARM].IsRaised = false;
	Cl_AlarmResetAlarm(BC_ALARM);
	Cl_DlsisAlarmTable[CL_Dlsis_ALARM_BLOODNOTDETECTED].IsRaised = false;
	Cl_AlarmResetAlarm(BLOOD_NOT_DETECTED);
	
	Cl_DlsisAlarmTable[CL_Dlsis_ALARM_HPOVERRUN].IsRaised = false;
	Cl_AlarmResetAlarm(HP_OVERRUN);
	Cl_DlsisAlarmTable[CL_Dlsis_ALARM_HPUNDERRUN].IsRaised = false;
	Cl_AlarmResetAlarm(HP_UNDERRUN);
	
	
	Cl_DlsisAlarmTable[CL_Dlsis_ALARM_BPOVERRUN].IsRaised = false;
	Cl_AlarmResetAlarm(BP_OVERRUN);
	Cl_DlsisAlarmTable[CL_Dlsis_ALARM_BPUNDERRUN].IsRaised = false;
	Cl_AlarmResetAlarm(BP_UNDERRUN);
	
	/*Cl_RinseAlarmTable[CL_RINSE_ALARM_TEMP2_HIGH_THRESHOLD].IsRaised = false;
	Cl_AlarmResetAlarm(TEMP2_LOW_THRESHOLD);
	Cl_AlarmResetAlarm(TEMP2_HIGH_THRESHOLD);
	Cl_RinseAlarmTable[CL_RINSE_ALARM_TEMP3_HIGH_THRESHOLD].IsRaised = false;
	Cl_RinseAlarmTable[CL_RINSE_ALARM_TEMP3_LOW_THRESHOLD].IsRaised = false;
	Cl_AlarmResetAlarm(TEMP2_HIGH_THRESHOLD);
	Cl_AlarmResetAlarm(TEMP2_LOW_THRESHOLD);
	*/
// 	Cl_DlsisAlarmTable[CL_DLSIS_ALARM_HOLDER1STATUS_OPEN].IsRaised = false;
// 	Cl_AlarmResetAlarm(HOLDER1STATUS_OPEN);
// //	Cl_AlarmResetAlarm(HOLDER1STATUS_CLOSED);
// 	Cl_DlsisAlarmTable[CL_DLSIS_ALARM_HOLDER2STATUS_OPEN].IsRaised = false;
// 	Cl_AlarmResetAlarm(HOLDER2STATUS_OPEN);
//	Cl_AlarmResetAlarm(HOLDER2STATUS_CLOSED);
	Cl_DlsisAlarmTable[CL_DLSIS_ALARM_FLOW_NO_FLOW].IsRaised = false;
	Cl_DlsisAlarmTable[CL_DLSIS_ALARM_FLOW_LOW_FLOWRATE].IsRaised = false;
	Cl_DlsisAlarmTable[CL_DLSIS_ALARM_FLOW_HIGH_FLOWRATE].IsRaised = false;
	//Cl_RinseAlarmTable[CL_RINSE_ALARM_LEVEL_SWITCH_LOW_TOGGLERATE].IsRaised = false;
	Cl_AlarmResetAlarm(FLOW_NO_FLOW);
	Cl_AlarmResetAlarm(FLOW_HIGH_FLOWRATE);
	Cl_AlarmResetAlarm(FLOW_LOW_FLOWRATE);
	
	
	
	return (Cl_dlsisretcode);
}

Cl_ReturnCodeType Cl_Dlsis_SenddlsisData(void)
{
	static float cond = 0;
	Cl_Uint8Type systemdataarray[44] =  {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
	cl_DlsisDatatype tempdata;
	Cl_Uint16Type temp=0, temp1=0 ,apt=0,vpt=0, temp2ui = 0 , ps1ui = 0 , ps2ui = 0 , ps1tmp = 0 , ps2tmp = 0;
	static float avgtmp3=0,ps1=0,ps2=0 , ts2 = 0  ;
	static Cl_Uint32Type timecount = 0;
	Cl_ReturnCodeType Cl_dlsisretcode = CL_OK;
	Cl_Uint8Type count = 0;
	Cl_ConsoleTxCommandtype command = CON_TX_COMMAND_COMMAND_MAX;
	Cl_Uint16Type sensordatamillivolts;
	command = CON_TX_COMMAND_CL_SYS_STATE_DATA ;
	systemdataarray[0] = DIALYSIS_DATA;
	count++;
	averaging_count++;
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
// 							float sensordata,ftemp1,temp1;
// 							sensordata = temp;
// 							uint16_t cal_data  = (402 *100* sensordata)/(2*32768);
// 							res_temp_lookuptable(cal_data);
// 							//	tmp3 =	(tmp3*5 + res_temp_value)/6;
// 							temp1 = res_temp_value/10;
// //							temp1 = temp1 - 3.1+1.4;                                  // removed offset on 02.07.17
// 							temp1 = temp1 - 0.4;                                  // removed offset on 02.07.17
							temp1 = (float)temp/10;
						//	avgtmp3 =(avgtmp3*5 + temp1)/6;           // manav03 commented
							
		}
		
		uint16_t sensordata = 0;
		Cl_SysStat_GetSensor_Status_Query(SENSOR_COND_STATUS,&sensordata);
		{
			
			
					tempdata.word = sensordata/100;
					systemdataarray[count++] = tempdata.bytearray[0];
					systemdataarray[count++] = tempdata.bytearray[1];
					systemdataarray[count++] = tempdata.bytearray[2];
					systemdataarray[count++] = tempdata.bytearray[3];
		}
		{

			tempdata.word = (temp1);  // manav03 changed avgtemp3
			systemdataarray[count++] = tempdata.bytearray[0];
			systemdataarray[count++] = tempdata.bytearray[1];
			systemdataarray[count++] = tempdata.bytearray[2];
			systemdataarray[count++] = tempdata.bytearray[3];
		}
		Cl_SysStat_GetSensor_Status_Query(SENSOR_APTSTATUS,&apt); // manav03 changed from  tempdata.Twobyte to apt
		{
			
			
			tempdata.word	= apt;
			systemdataarray[count++] = tempdata.bytearray[0] ;
			systemdataarray[count++] = tempdata.bytearray[1] ;
			systemdataarray[count++] = tempdata.bytearray[2] ;
			systemdataarray[count++] = tempdata.bytearray[3] ;
		}
		Cl_SysStat_GetSensor_Status_Query(SENSOR_VPTSTATUS,&vpt);   // manav03 changed from  tempdata.Twobyte to vpt
		{
			
			tempdata.word	 = vpt;
			systemdataarray[count++] = tempdata.bytearray[0] ;
			systemdataarray[count++] = tempdata.bytearray[1] ;
			systemdataarray[count++] = tempdata.bytearray[2] ;
			systemdataarray[count++] = tempdata.bytearray[3] ;
		}
			Cl_SysStat_GetSensor_Status_Query(SENSOR_PS1STATUS,&ps1tmp);
// 			if (tempdata.Twobyte >32268)
// 			{
// 				ps1 = (65536 - tempdata.Twobyte) *(-1);
// 			}
// 			else
// 			ps1 = (float)tempdata.Twobyte;
// 			
// 			ps1 = (0.091009 * ps1) + 491.794;
// 			ps1=(ps1*5 + pressure_final_ps1)/6;
	
			Cl_SysStat_GetSensor_Status_Query(SENSOR_PS2STATUS,&ps2tmp);
// 			if (tempdata.Twobyte >32268)
// 			{
// 				ps2 = (65536 - tempdata.Twobyte) *(-1);
// 			}
// 			else
// 			ps2 = (float)tempdata.Twobyte;
// 			
// 			ps2 = (0.091009 * ps2) + 491.794;
// 			ps2=(ps2*5 + pressure_final_ps2)/6;
			
			tempdata.word	 = (((apt + vpt) - (ps1tmp+ps2tmp))/2)-25;


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
			
			// manav02 added pressure and ts2 values for ui
			Cl_SysStat_GetSensor_Status_Query(SENSOR_PS1STATUS,&ps1ui); // manav03 changed the variable
			{
				
				tempdata.word = ps1ui ;
				systemdataarray[count++] = tempdata.bytearray[0] ;
				systemdataarray[count++] = tempdata.bytearray[1] ;
				systemdataarray[count++] = tempdata.bytearray[2] ;
				systemdataarray[count++] = tempdata.bytearray[3] ;
			}
			
			Cl_SysStat_GetSensor_Status_Query(SENSOR_PS2STATUS,&ps2ui);     // manav03 changed from ps1 to ps2
			{
				
				tempdata.word = ps2ui ;
				systemdataarray[count++] = tempdata.bytearray[0] ;
				systemdataarray[count++] = tempdata.bytearray[1] ;
				systemdataarray[count++] = tempdata.bytearray[2] ;
				systemdataarray[count++] = tempdata.bytearray[3] ;
			}
			
			Cl_SysStat_GetSensor_Status_Query(SENSOR_TEMP2STATUS,&temp2ui);    // manav02 ts2 changed to temp2ui
			{
				
				tempdata.word = temp2ui/10 ; 
				systemdataarray[count++] = tempdata.bytearray[0] ;
				systemdataarray[count++] = tempdata.bytearray[1] ;
				systemdataarray[count++] = tempdata.bytearray[2] ;
				systemdataarray[count++] = tempdata.bytearray[3] ;
			}
			
			
Cl_dlsisretcode = Cl_SendDatatoconsole(CON_TX_COMMAND_CL_SYS_STATE_DATA,&systemdataarray,count);
	//Cl_dlsisretcode = Cl_SendDatatoconsole(command,&systemdataarray,count);
	
	return CL_OK;
}




Cl_ReturnCodeType Cl_Dlsis_Get_data(Cl_ConsoleRxDataType DataId, Cl_Uint8Type size)
{
	Cl_ReturnCodeType Cl_dlsisretcode = CL_OK;
	Cl_ConsoleTxCommandtype command = CON_TX_COMMAND_COMMAND_MAX;
	Cl_Uint8Type dataarray[8] =  {0,0,0,0};
	Cl_Uint8Type  datasize = 0;
	
	if(DataId == CON_RX_PARAM_DATA_TREATMENT_DATA )
	{
		//Cl_Dprep_SendtreatementData();
	}
	else
	{
		switch (DataId)
		{

			case	CON_RX_PARAM_DATA_RINSE_STATUS:
			//cl_wait(10);
			
			command = CON_TX_COMMAND_SYSDATA;

			dataarray[0] = CON_TX_PARAM_DATA_RINSE_STATUS;
			Cl_dlsisretcode = (Cl_Uint8Type)sv_nvmgetdata(NV_NVM_RINSE_STATUS, &dataarray[1]);
			datasize = 2;

			break;
			case	CON_RX_PARAM_DATA_PRIME_STATUS:
			//	cl_wait(10);
			
			command = CON_TX_COMMAND_SYSDATA;

			dataarray[0] = CON_TX_PARAM_DATA_PRIME_STATUS;
			Cl_dlsisretcode = (Cl_Uint8Type)sv_nvmgetdata(NV_NVM_PRIME_STATUS, &dataarray[1]);
			datasize = 2;

			break;
			
			case	CON_RX_PARAM_DATA_DIALYSIS_STATUS:
			//cl_wait(10);
			
			command = CON_TX_COMMAND_SYSDATA;

			dataarray[0] = CON_TX_PARAM_DATA_DIALYSIS_STATUS;
			Cl_dlsisretcode = (Cl_Uint8Type)sv_nvmgetdata(NV_NVM_DIALYSIS_STATUS, &dataarray[1]);
			datasize = 2;

			break;
			case	CON_RX_PARAM_DATA_DIALYSIS_FLOW:
			//cl_wait(10);
			
			command = CON_TX_COMMAND_SYSDATA;

			dataarray[0] = CON_TX_PARAM_DATA_DIALYSIS_FLOW;
			Cl_dlsisretcode = (Cl_Uint8Type)sv_nvmgetdata(NV_DIALYSATE_FLOW, &dataarray[1]);
			datasize = 2;
			//Reset the  OPENFILL time count
			//Cl_dlsisretcode = Cl_SendDatatoconsole(command,&dataarray,3);
			//cl_wait(10);
			break;
			case	CON_RX_PARAM_DATA_DIALYSIS_TEMP:
			//cl_wait(10);
			
			command = CON_TX_COMMAND_SYSDATA;
			//cl_stby_retval = (Cl_Uint8Type)sv_nvmgetdata(NV_RINSE_NVM_RINSE_STATUS, &data);
			dataarray[0] = CON_TX_PARAM_DATA_DIALYSIS_TEMP;
			Cl_dlsisretcode = (Cl_Uint8Type)sv_nvmgetdata(NV_DIALYSATE_TEMP, &dataarray[1]);
			datasize = 2;
			//Reset the  OPENFILL time count
			//Cl_dlsisretcode = Cl_SendDatatoconsole(command,&dataarray,1);
			//cl_wait(10);
			break;
			case	CON_RX_PARAM_DATA_HEPARIN_RATE:
			//cl_wait(10);
			
			command = CON_TX_COMMAND_SYSDATA;
			//cl_stby_retval = (Cl_Uint8Type)sv_nvmgetdata(NV_RINSE_NVM_RINSE_STATUS, &data);
			dataarray[0] = CON_TX_PARAM_DATA_HEPARIN_RATE;
			//				Cl_dlsisretcode = (Cl_Uint8Type)sv_nvmgetdata(NV_HEPARIN_RATE, &dataarray[1]);
			
			datasize = 2;
			break;
			case	CON_RX_PARAM_DATA_APT_VALUE:
			//cl_wait(10);
			
			command = CON_TX_COMMAND_SYSDATA;
			//cl_stby_retval = (Cl_Uint8Type)sv_nvmgetdata(NV_RINSE_NVM_RINSE_STATUS, &data);
			dataarray[0] = CON_TX_PARAM_DATA_APT_VALUE;
			//			Cl_dlsisretcode = (Cl_Uint8Type)sv_nvmgetdata(NV_APT_VALUE, &dataarray[1]);
			
			//Reset the  OPENFILL time count
			//Cl_dlsisretcode = Cl_SendDatatoconsole(command,&dataarray,1);
			//cl_wait(10);
			datasize = 2;
			break;
			case	CON_RX_PARAM_DATA_VPT_VALUE:
			//cl_wait(10);
			
			command = CON_TX_COMMAND_SYSDATA;
			//cl_stby_retval = (Cl_Uint8Type)sv_nvmgetdata(NV_RINSE_NVM_RINSE_STATUS, &data);
			dataarray[0] = CON_TX_PARAM_DATA_VPT_VALUE;
			//	Cl_dlsisretcode = (Cl_Uint8Type)sv_nvmgetdata(NV_VPT_VALUE, &dataarray[1]);
			
			//Reset the  OPENFILL time count
			//Cl_dlsisretcode = Cl_SendDatatoconsole(command,&dataarray,1);
			//cl_wait(10);
			datasize = 2;
			break;
			case	CON_RX_PARAM_DATA_TMP_VALUE:
			//cl_wait(10);
			
			command = CON_TX_COMMAND_SYSDATA;
			//cl_stby_retval = (Cl_Uint8Type)sv_nvmgetdata(NV_RINSE_NVM_RINSE_STATUS, &data);
			dataarray[0] = CON_TX_PARAM_DATA_TMP_VALUE;
			//			Cl_dlsisretcode = (Cl_Uint8Type)sv_nvmgetdata(NV_RINSE_NVM_RINSE_STATUS, &dataarray[1]);
			
			//Reset the  OPENFILL time count
			//Cl_dlsisretcode = Cl_SendDatatoconsole(command,&dataarray,1);
			//cl_wait(10);
			datasize = 2;
			break;
			case	CON_RX_PARAM_DATA_COND_VALUE:
			//cl_wait(10);
			
			command = CON_TX_COMMAND_SYSDATA;
			//cl_stby_retval = (Cl_Uint8Type)sv_nvmgetdata(NV_RINSE_NVM_RINSE_STATUS, &data);
			dataarray[0] = CON_TX_PARAM_DATA_COND_VALUE;
			//				Cl_dlsisretcode = (Cl_Uint8Type)sv_nvmgetdata(NV_RINSE_NVM_RINSE_STATUS, &dataarray[1]);
			
			//Reset the  OPENFILL time count
			//Cl_dlsisretcode = Cl_SendDatatoconsole(command,&dataarray,1);
			//cl_wait(10);
			datasize = 2;
			break;
			case	CON_RX_PARAM_DATA_UF_RATE:
			//	cl_wait(10);
			
			command = CON_TX_COMMAND_SYSDATA;
			//cl_stby_retval = (Cl_Uint8Type)sv_nvmgetdata(NV_RINSE_NVM_RINSE_STATUS, &data);
			dataarray[0] = CON_TX_PARAM_DATA_UF_RATE;
			Cl_dlsisretcode = (Cl_Uint8Type)sv_nvmgetdata(NV_UF_REMOVAL_RATE, &dataarray[1]);
			
			//Reset the  OPENFILL time count
			//Cl_dlsisretcode = Cl_SendDatatoconsole(command,&dataarray,1);
			//	cl_wait(10);
			datasize = 2;
			break;
			case	CON_RX_PARAM_DATA_ABF_RATE:
			//cl_wait(10);
			
			command = CON_TX_COMMAND_SYSDATA;
			//cl_stby_retval = (Cl_Uint8Type)sv_nvmgetdata(NV_RINSE_NVM_RINSE_STATUS, &data);
			dataarray[0] = CON_TX_PARAM_DATA_ABF_RATE;
			Cl_dlsisretcode = (Cl_Uint8Type)sv_nvmgetdata(NV_ARTERIAL_BLOODFLOW_RATE, &dataarray[1]);
			
			//Reset the  OPENFILL time count
			//Cl_dlsisretcode = Cl_SendDatatoconsole(command,&dataarray,1);
			//cl_wait(10);
			datasize = 2;
			break;
			#if 0
			case	CON_RX_PARAM_DATA_ABD_THRESHOLD:
			//cl_wait(10);
			
			command = CON_TX_COMMAND_SYSDATA;
			//cl_stby_retval = (Cl_Uint8Type)sv_nvmgetdata(NV_RINSE_NVM_RINSE_STATUS, &data);
			dataarray[0] = CON_TX_PARAM_DATA_ABD_THRESHOLD;
			Cl_dlsisretcode = (Cl_Uint8Type)sv_nvmgetdata(NV_ABD_THRESHOLD, &dataarray[1]);
			
			//Reset the  OPENFILL time count
			//Cl_dlsisretcode = Cl_SendDatatoconsole(command,&dataarray,1);
			//	cl_wait(10);
			break;
			case	CON_RX_PARAM_DATA_BLD_THRESHOLD:
			//cl_wait(10);
			
			command = CON_TX_COMMAND_SYSDATA;
			//cl_stby_retval = (Cl_Uint8Type)sv_nvmgetdata(NV_RINSE_NVM_RINSE_STATUS, &data);
			dataarray[0] = CON_TX_PARAM_DATA_BLD_THRESHOLD;
			Cl_dlsisretcode = (Cl_Uint8Type)sv_nvmgetdata(NV_BLD_THRESHOLD, &dataarray[1]);
			
			//Reset the  OPENFILL time count
			//Cl_dlsisretcode = Cl_SendDatatoconsole(command,&dataarray,1);
			//cl_wait(10);
			datasize = 2;
			break;
			#endif
			
			
			case CON_RX_PARAM_DATA_DIALYSIS_FLOW_CONFIG: //0x13
			
			
			
			
			dataarray[0] = CON_TX_PARAM_DATA_DIALYSIS_FLOW_CONFIG;
			Cl_dlsisretcode = (Cl_Uint8Type)sv_nvmgetdata(NV_DIALYSATE_FLOW, &dataarray[1]);
			Cl_dlsisretcode = (Cl_Uint8Type)sv_nvmgetdata(NV_DIALYSIS_FLOW_CONFIG_UPPER, &dataarray[3]);
			Cl_dlsisretcode = (Cl_Uint8Type)sv_nvmgetdata(NV_DIALYSIS_FLOW_CONFIG_LOWER, &dataarray[5]);
			
			datasize = 	7;
			
			break;
			case CON_RX_PARAM_DATA_DIALYSIS_TEMP_CONFIG://0x14
			

			dataarray[0] = CON_TX_PARAM_DATA_DIALYSIS_TEMP_CONFIG;
			Cl_dlsisretcode = (Cl_Uint8Type)sv_nvmgetdata(NV_DIALYSATE_TEMP, &dataarray[1]);
			Cl_dlsisretcode = (Cl_Uint8Type)sv_nvmgetdata(NV_DIALYSIS_TEMP_CONFIG_UPPER, &dataarray[3]);
			Cl_dlsisretcode = (Cl_Uint8Type)sv_nvmgetdata(NV_DIALYSIS_TEMP_CONFIG_LOWER, &dataarray[5]);
			datasize = 	7;
			break;
			case CON_RX_PARAM_DATA_HEPARIN_RATE_CONFIG://0x15

			dataarray[0] = CON_TX_PARAM_DATA_HEPARIN_RATE_CONFIG;
			Cl_dlsisretcode = (Cl_Uint8Type)sv_nvmgetdata(NV_HEPARIN_FLOW_RATE, &dataarray[1]);
			Cl_dlsisretcode = (Cl_Uint8Type)sv_nvmgetdata(NV_HEPARIN_RATE_CONFIG_UPPER, &dataarray[3]);
			Cl_dlsisretcode = (Cl_Uint8Type)sv_nvmgetdata(NV_HEPARIN_RATE_CONFIG_LOWER, &dataarray[5]);
			datasize = 	7;
			break;
			case CON_RX_PARAM_DATA_APT_VALUE_CONFIG://0x16
			
			dataarray[0] = CON_TX_PARAM_DATA_APT_VALUE_CONFIG;
			Cl_dlsisretcode = (Cl_Uint8Type)sv_nvmgetdata(NV_APT_VALUE, &dataarray[1]);
			Cl_dlsisretcode = (Cl_Uint8Type)sv_nvmgetdata(NV_APT_VALUE_CONFIG_UPPER, &dataarray[3]);
			Cl_dlsisretcode = (Cl_Uint8Type)sv_nvmgetdata(NV_APT_VALUE_CONFIG_LOWER, &dataarray[5]);
			datasize = 	7;
			
			break;
			case CON_RX_PARAM_DATA_VPT_VALUE_CONFIG://0x17

			dataarray[0] = CON_TX_PARAM_DATA_VPT_VALUE_CONFIG;
			Cl_dlsisretcode = (Cl_Uint8Type)sv_nvmgetdata(NV_VPT_VALUE, &dataarray[1]);
			Cl_dlsisretcode = (Cl_Uint8Type)sv_nvmgetdata(NV_VPT_VALUE_CONFIG_UPPER, &dataarray[3]);
			Cl_dlsisretcode = (Cl_Uint8Type)sv_nvmgetdata(NV_VPT_VALUE_CONFIG_LOWER, &dataarray[5]);
			datasize = 	7;
			break;
			case CON_RX_PARAM_DATA_TMP_VALUE_CONFIG://0x18

			dataarray[0] = CON_TX_PARAM_DATA_TMP_VALUE_CONFIG;
			Cl_dlsisretcode = (Cl_Uint8Type)sv_nvmgetdata(NV_TMP, &dataarray[1]);
			Cl_dlsisretcode = (Cl_Uint8Type)sv_nvmgetdata(NV_TMP_VALUE_CONFIG_UPPER, &dataarray[3]);
			Cl_dlsisretcode = (Cl_Uint8Type)sv_nvmgetdata(NV_TMP_VALUE_CONFIG_LOWER, &dataarray[5]);
			datasize = 	7;
			break;
			case CON_RX_PARAM_DATA_COND_VALUE_CONFIG://0x19

			dataarray[0] = CON_TX_PARAM_DATA_COND_VALUE_CONFIG;
			Cl_dlsisretcode = (Cl_Uint8Type)sv_nvmgetdata(NV_CONDUCTIVITY, &dataarray[1]);
			Cl_dlsisretcode = (Cl_Uint8Type)sv_nvmgetdata(NV_COND_VALUE_CONFIG_UPPER, &dataarray[3]);
			Cl_dlsisretcode = (Cl_Uint8Type)sv_nvmgetdata(NV_COND_VALUE_CONFIG_LOWER, &dataarray[5]);
			datasize = 	7;
			break;
			case CON_RX_PARAM_DATA_UF_RATE_CONFIG://0x1A

			dataarray[0] = CON_TX_PARAM_DATA_UF_RATE_CONFIG;
			Cl_dlsisretcode = (Cl_Uint8Type)sv_nvmgetdata(NV_UF_REMOVAL_RATE, &dataarray[1]);
			Cl_dlsisretcode = (Cl_Uint8Type)sv_nvmgetdata(NV_UF_RATE_CONFIG_UPPER, &dataarray[3]);
			Cl_dlsisretcode = (Cl_Uint8Type)sv_nvmgetdata(NV_UF_RATE_CONFIG_LOWER, &dataarray[5]);
			datasize = 	7;
			break;
			case CON_RX_PARAM_DATA_UF_GOAL_CONFIG://0x1B,

			dataarray[0] = CON_TX_PARAM_DATA_UF_GOAL_CONFIG;
			Cl_dlsisretcode = (Cl_Uint8Type)sv_nvmgetdata(NV_UF_GOAL, &dataarray[1]);
			Cl_dlsisretcode = (Cl_Uint8Type)sv_nvmgetdata(NV_UF_GOAL_CONFIG_UPPER, &dataarray[3]);
			Cl_dlsisretcode = (Cl_Uint8Type)sv_nvmgetdata(NV_UF_GOAL_CONFIG_LOWER, &dataarray[5]);
			datasize = 	7;
			break;
			case CON_RX_PARAM_DATA_BOLUS_VOLUME_CONFIG://1C

			dataarray[0] = CON_TX_PARAM_DATA_BOLUS_VOLUME_CONFIG;
			Cl_dlsisretcode = (Cl_Uint8Type)sv_nvmgetdata(NV_HEPARIN_BOLUS, &dataarray[1]);
			Cl_dlsisretcode = (Cl_Uint8Type)sv_nvmgetdata(NV_BOLUS_VOLUME_CONFIG_UPPER, &dataarray[3]);
			Cl_dlsisretcode = (Cl_Uint8Type)sv_nvmgetdata(NV_BOLUS_VOLUME_CONFIG_LOWER, &dataarray[5]);
			datasize = 	7;
			break;
			case CON_RX_PARAM_DATA_ABF_RATE_CONFIG://0X1D

			dataarray[0] = CON_TX_PARAM_DATA_ABF_RATE_CONFIG;
			Cl_dlsisretcode = (Cl_Uint8Type)sv_nvmgetdata(NV_HEPARIN_BOLUS, &dataarray[1]);
			Cl_dlsisretcode = (Cl_Uint8Type)sv_nvmgetdata(NV_ABF_RATE_CONFIG_UPPER, &dataarray[3]);
			Cl_dlsisretcode = (Cl_Uint8Type)sv_nvmgetdata(NV_ABF_RATE_CONFIG_LOWER, &dataarray[5]);
			datasize = 	7;
			break;
			default:
			break;
		}
		command = CON_TX_COMMAND_SYSDATA;
		Cl_dlsisretcode = Cl_SendDatatoconsole(command,&dataarray,datasize);
		
	}
return CL_OK;
}



Cl_ReturnCodeType	Cl_Dlsis_StartDialysis(void)
{
	
	sv_cntrl_resetgreenalarm();
	Cl_ReturnCodeType Cl_dlsisretcode = CL_OK;
	Cl_ConsoleTxCommandtype command = CON_TX_COMMAND_COMMAND_MAX;
	Cl_Uint8Type data = 0;
	
	ufRemoved = 0;
	

	//Reset the  OPENFILL time count
	DlsisTime.Cl_dlsisMinutescounter = 0;
	DlsisTime.Cl_dlsisTotalMinutescounter = 0;
	Cl_dlsisOpenFillTimeOut = false;
	
	// manav05 commented
// 		command = CON_TX_COMMAND_DLSIS_CNFRM;
// 		data = (Cl_Uint8Type)COMMAND_RESULT_SUCCESS;
// 		Cl_dlsisretcode = Cl_SendDatatoconsole(command,&data,1);
// 		
//		cl_wait(20); // manav03 added
		command = CON_TX_COMMAND_DLSIS_STARTED;
		data = (Cl_Uint8Type)COMMAND_RESULT_SUCCESS;
		Cl_dlsisretcode = Cl_SendDatatoconsole(command,&data,1);
	Cl_Dlsis_ResumeDialysis();
	
	return 0;

}

Cl_ReturnCodeType	Cl_Dlsis_ResumeDialysis(void)
{
	
		Cl_ReturnCodeType Cl_dlsisretcode = CL_OK;
		Cl_ConsoleTxCommandtype command = CON_TX_COMMAND_COMMAND_MAX;
		Cl_Uint8Type data = 0;
									
									
									Cl_dlsisretcode =  Cl_AlarmConfigureAlarmType(BLOODDOOR_STATUS_OPEN,LOGIC_HIGH,0,0,0);
									Cl_dlsisretcode =  Cl_AlarmConfigureAlarmType(ACID_IN_OPEN,LOGIC_LOW,0,0,0);
									Cl_dlsisretcode =  Cl_AlarmConfigureAlarmType(BICARB_IN_OPEN,LOGIC_LOW,0,0,0);
									Cl_dlsisretcode =  Cl_AlarmConfigureAlarmType(TMP_HIGH,0,1500,0,0);
									Cl_dlsisretcode =  Cl_AlarmConfigureAlarmType(TMP_LOW,0,600,0,0);
									
									
									Cl_dlsisretcode =  Cl_AlarmActivateAlarms(BLOODDOOR_STATUS_OPEN,true );
									Cl_dlsisretcode =  Cl_AlarmActivateAlarms(LEVELSWITCH_OFF_TO_ON,true );
									Cl_dlsisretcode =  Cl_AlarmActivateAlarms(LEVELSWITCH_ON_TO_OFF ,true);
									Cl_dlsisretcode =  Cl_AlarmActivateAlarms( HOLDER1STATUS_CLOSED,true );
									Cl_dlsisretcode =  Cl_AlarmActivateAlarms( HOLDER2STATUS_CLOSED,true );
									Cl_dlsisretcode =  Cl_AlarmActivateAlarms( HOLDER1STATUS_OPEN,false );
									Cl_dlsisretcode =  Cl_AlarmActivateAlarms( HOLDER2STATUS_OPEN,false );
									Cl_dlsisretcode =  Cl_AlarmActivateAlarms(COND_STATUS_LOW,true );
									Cl_dlsisretcode =  Cl_AlarmActivateAlarms(COND_STATUS_HIGH,true );
								//	Cl_dlsisretcode =  Cl_AlarmActivateAlarms(COND_DAC_OPEN,true );
								//	Cl_dlsisretcode =  Cl_AlarmActivateAlarms(COND_DAC_RO,true );
								//	Cl_dlsisretcode =  Cl_AlarmActivateAlarms(COND_DAC_HIGH,true );
									Cl_dlsisretcode =  Cl_AlarmActivateAlarms( ABD_EVENT,true );
									Cl_dlsisretcode =  Cl_AlarmActivateAlarms( BD_EVENT,false );
									Cl_dlsisretcode =  Cl_AlarmActivateAlarms(APTSTATUS_HIGH,true );
									Cl_dlsisretcode =  Cl_AlarmActivateAlarms(VPTSTATUS_HIGH ,true);
									Cl_dlsisretcode =  Cl_AlarmActivateAlarms(APTSTATUS_LOW,true );
									Cl_dlsisretcode =  Cl_AlarmActivateAlarms(VPTSTATUS_LOW,true);
									Cl_dlsisretcode =  Cl_AlarmActivateAlarms( BLD_EVENT,true );
									Cl_dlsisretcode =  Cl_AlarmActivateAlarms(PS2_HIGH_THRESHOLD,true );
									Cl_dlsisretcode =  Cl_AlarmActivateAlarms(PS2_LOW_THRESHOLD,true );
									Cl_dlsisretcode =  Cl_AlarmActivateAlarms(PS1_HIGH_THRESHOLD,true );
									Cl_dlsisretcode =  Cl_AlarmActivateAlarms(PS1_LOW_THRESHOLD,true );
									Cl_dlsisretcode =  Cl_AlarmActivateAlarms(PS3_HIGH_THRESHOLD ,true);
									Cl_dlsisretcode =  Cl_AlarmActivateAlarms(PS3_LOW_THRESHOLD ,true);
									
// 									Cl_dlsisretcode =  Cl_AlarmActivateAlarms(SENSOR_TEMP3STATUS,false );
// 									Cl_dlsisretcode =  Cl_AlarmActivateAlarms(SENSOR_TEMP2STATUS,false );

//
									Cl_dlsisretcode =  Cl_AlarmActivateAlarms(FLOW_NO_FLOW,true );
									Cl_dlsisretcode =  Cl_AlarmActivateAlarms(FLOW_LOW_FLOWRATE,true );
									Cl_dlsisretcode =  Cl_AlarmActivateAlarms(FLOW_HIGH_FLOWRATE,true ); 									
									Cl_dlsisretcode =  Cl_AlarmActivateAlarms(TEMP1_HIGH_THRESHOLD,true );
									Cl_dlsisretcode =  Cl_AlarmActivateAlarms(TEMP1_LOW_THRESHOLD,true );
									Cl_dlsisretcode =  Cl_AlarmActivateAlarms(TEMP2_HIGH_THRESHOLD,true );
									Cl_dlsisretcode =  Cl_AlarmActivateAlarms(TEMP2_LOW_THRESHOLD,true );
									Cl_dlsisretcode =  Cl_AlarmActivateAlarms(TEMP3_HIGH_THRESHOLD,true );
									Cl_dlsisretcode =  Cl_AlarmActivateAlarms(TEMP3_LOW_THRESHOLD,true );
									Cl_dlsisretcode =  Cl_AlarmActivateAlarms(BP_STOPPED,true );
									Cl_dlsisretcode =  Cl_AlarmActivateAlarms(TMP_HIGH,true );
									Cl_dlsisretcode =  Cl_AlarmActivateAlarms(TMP_LOW,true );
									Cl_dlsisretcode =  Cl_AlarmActivateAlarms(BLOOD_NOT_DETECTED,false );  // manav05 enabled 
									Cl_dlsisretcode =  Cl_AlarmActivateAlarms(BC_ALARM,true);
									
									Cl_dlsisretcode =  Cl_AlarmActivateAlarms(HP_OVERRUN,true );  // manav05 enabled 
									Cl_dlsisretcode =  Cl_AlarmActivateAlarms(HP_UNDERRUN,true);
								
									if(Current_sense_trigger)
									{
										Cl_dlsisretcode =  Cl_AlarmActivateAlarms( FPCURRENTSTATUS,true );
									}
									else
									{
										//			Cl_rinseretcode =  Cl_AlarmActivateAlarms( PS3STATUS_HIGH,true );
									}
									sv_cntrl_disable_loopback();
									//Cl_Dlsis_BypassOff();
									sv_cntrl_activatevenousclamp();
									Cl_dlsisretcode =  sv_cntrl_activatepump(DCMOTOR1);
									Cl_dlsisretcode =  sv_cntrl_activatepump(DCMOTOR2);
								//	Cl_dlsisretcode = Cl_bc_controller(BC_EVENT_RESUME);               // manav03 removed
									Cl_dlsisretcode = SetHeaterState(CL_HEATER_STATE_DIALYSIS_ON);
									cl_bp_controller(CL_BP_EVENT_START,0);
									cl_wait(100);
									Cl_Uint32Type temp1 = Treatdata[ID_bloodratetreat];
									cl_bp_controller(CL_BP_EVENT_SET_BP_RATE,temp1);
									cl_wait(100);
									//cl_wait(100);
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
										break;
										
										case 500:
										Cl_SendDatatoconsole(CON_TX_COMMAND_PRINTTEXT,"500",3);
										sv_cntrl_setpumpspeed(DCMOTOR2,1000);
										sv_cntrl_setpumpspeed(DCMOTOR1,1000);   //960
										potvalue = (600 * 1024)/10000;
										sv_cs_setpotvalue(potvalue);
										break;
										
									
										case 300:
										sv_cntrl_setpumpspeed(DCMOTOR2,1000);
										sv_cntrl_setpumpspeed(DCMOTOR1,600);
										potvalue = (650 * 1024)/10000;
										sv_cs_setpotvalue(potvalue);
										break;
										
										default:
										sv_cntrl_setpumpspeed(DCMOTOR2,1000);
										sv_cntrl_setpumpspeed(DCMOTOR1,1100);   //960
										potvalue = (670 * 1024)/10000;
										sv_cs_setpotvalue(potvalue);
										break;
									}
									bc_flag = 1;
									//Cl_Dlsis_BypassDialysis();
									Cl_dlsisretcode = sv_cntrl_disable_bypass();
									Cl_dlsisretcode = Cl_Dlsis_UpdateTimeInfo();
									cl_dlsis_state = CL_DLSIS_STATE_WAIT_FOR_TEMP_COND_STABILITY;
									//Cl_Dlsis_BypassOff();
									return 0;	
}

Cl_ReturnCodeType	Cl_Dlsis_safesate(void)
{
	
	sv_cntrl_enable_bypass();
	cl_uf_controller(CL_UF_EVENT_STOP,0);
	cl_bp_controller(CL_BP_EVENT_STOP,0);
	Cl_AlarmActivateAlarms(BP_OVERRUN,false );
	Cl_AlarmActivateAlarms(BP_UNDERRUN,false );
	bpFlag = 0 ;
	bpflagCounter = 0;
	cl_hep_controller(CL_HEP_EVENT_STOP_NORMAL_DELIVERY,0);
	sv_cntrl_deactivatevenousclamp();

	Cl_Dlsis_BypassDialysis();

	//isouf = false;
	// send dialysis isouf off
// 	command = CON_TX_COMMAND_DIALYSIS_ISOUF_OFF;
// 	dataarray[0] = CON_TX_PARAM_DATA_DIALYSIS_STATUS;
// 	Cl_dlsisretcode = Cl_SendDatatoconsole(command,&dataarray,0);
	cl_dlsis_state = CL_DLSIS_STATE_CRITICAL_ALARM;
	
	return 0;
}
Cl_ReturnCodeType	Cl_Dlsis_recoveredsate(void)
{
	
	sv_cntrl_disable_bypass();
	cl_uf_controller(CL_UF_EVENT_START,0);
	cl_ufpumpFeedback_start();
// 	cl_uf_controller(CL_UF_EVENT_SET_UF_RATE,10000);
// 	cl_wait(100);
// 	cl_uf_controller(CL_UF_EVENT_SET_UF_RATE,5000);
// 	cl_wait(100);
// 	cl_uf_controller(CL_UF_EVENT_SET_UF_RATE,3500);
// 	cl_wait(100);
// 	cl_uf_controller(CL_UF_EVENT_SET_UF_RATE,2500);
	//sv_cntrl_resetredalarm();
	//sv_cntrl_nobuzzer();
	
	return 0;
}
Cl_ReturnCodeType cl_temp_safestate(void)
{
	Cl_SendDatatoconsole(CON_TX_COMMAND_PRINTTEXT,"BYPASS",6);
	sv_cntrl_enable_bypass();
	cl_uf_controller(CL_UF_EVENT_STOP,0);
	//sv_cntrl_deactivatevenousclamp();
	Cl_Dlsis_BypassDialysis();
	cl_dlsis_state = CL_DLSIS_STATE_CRITICAL_ALARM;
	return 0;
}
Cl_ReturnCodeType cl_temp_recoveredstate(void)
{
	sv_cntrl_disable_bypass();
	
	cl_uf_controller(CL_UF_EVENT_START,0);
	cl_ufpumpFeedback_start();
	
// 	cl_uf_controller(CL_UF_EVENT_SET_UF_RATE,10000);
// 	cl_wait(100);
// 	cl_uf_controller(CL_UF_EVENT_SET_UF_RATE,5000);
// 	cl_wait(100);
// 	cl_uf_controller(CL_UF_EVENT_SET_UF_RATE,3500);
// 	cl_wait(100);
// 	cl_uf_controller(CL_UF_EVENT_SET_UF_RATE,2500);
	
	
	
	return 0;
}

Cl_ReturnCodeType cl_cond_safestate(void)
{
	//sv_cntrl_enable_bypass();
	Cl_Dlsis_BypassDialysis();
	//sv_cntrl_deactivate_valve(VALVE_ID13);
	//sv_cntrl_deactivatepump(DCMOTOR1);
	//sv_cntrl_deactivatepump(DCMOTOR2);
	
	//sv_cntrl_setflowpath(FLOW_PATH_BC_ALARM);
	//SetHeaterState(CL_HEATER_STATE_CLOSED_HEATING);
	cl_dlsis_state = CL_DLSIS_STATE_CRITICAL_ALARM;
	return 0;
}

Cl_ReturnCodeType cl_cond_recoveredstate(void)
{
	//sv_cntrl_enable_bypass();
	sv_cntrl_activate_valve(VALVE_ID13);
	sv_cntrl_activatepump(DCMOTOR1);
	sv_cntrl_activatepump(DCMOTOR2);
	Cl_bc_controller(BC_EVENT_RESUME);
	SetHeaterState(CL_HEATER_STATE_CLOSED_HEATING);
	return 0;
}
