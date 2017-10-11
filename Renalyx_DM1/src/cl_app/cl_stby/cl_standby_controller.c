

#ifdef WIN32
#include<stdio.h>
#endif

#include "cl_standby_controller_include.h"
#include "cl_standby_controller_proto.h"


extern Cl_ConsoleMsgType Cl_ConsoleRxMsg;
extern Cl_ReturnCodeType cl_wait(Cl_Uint32Type ul_dly_ticks);
extern Cl_ReturnCodeType   Cl_SendDatatoconsole(Cl_ConsoleTxCommandtype , Cl_Uint8Type* ,Cl_Uint8Type );
extern Cl_ReturnCodeType Cl_AlarmActivateAlarms(Cl_NewAlarmIdType , Cl_BoolType);
extern Cl_ReturnCodeType sv_nvmgetdata(Cl_Uint8Type , Cl_Uint8Type* );
extern Cl_ReturnCodeType  sv_nvmsetdata(Cl_Uint8Type ,Cl_Uint8Type*,Cl_Uint8Type datasize );
extern Cl_ReturnCodeType Cl_Alarm_GetAlarmStatus(Cl_NewAlarmIdType  , Cl_BoolType* );
extern Cl_Uint8Type  sv_cntrl_deactivatepump(sv_pumptype);
//extern sv_cntrl_poweroffheater(void);
extern Cl_Uint8Type sv_cntrl_deactivate_valve(sv_valvetype );
extern Cl_Uint8Type sv_cs_setpotvalue(Cl_Uint32Type resistance) ;
extern Cl_Uint8Type sv_cntrl_deactivate_valve(sv_valvetype ); 
extern Cl_Uint8Type sv_cs_setcondpotvalue(Cl_Uint16Type resistance);
extern Cl_ReturnCodeType  Cl_Conductivity_Sensor_Controller(Cl_CondSensor_EventType Cl_cond_Event);
extern Cl_Uint16Type prev_bpstate;

typedef union {
	Cl_Uint32Type fourbytedata;
	int16_t twobytedata;
	Cl_Uint8Type bytedata[4] ;
}STBStateDatatype;

StandbyAlarmsType Cl_StandbyAlarmTable[CL_STBY_ALRM_MAX] = {{LEVELSWITCH_OFF_TO_ON,false,false,false},{LEVELSWITCH_ON_TO_OFF,false,false,false},{HOLDER1STATUS_OPEN,false,false,false},{HOLDER2STATUS_OPEN,false,false,false}};
	
Cl_Stby_States cl_stbystate = CL_STBY_STATE_IDLE;

 int Cl_StbRinsesecondscounter =0 ,Cl_StbRinseMinutescounter=0, Cl_StbRinseshourscounter=0;
 int Cl_StbRinseTotalMinutescounter=0, Cl_StbRinseTotalhourscounter=0;
 Cl_BoolType Cl_StbRinseOpenFillTimeOut = false;
 
 extern Cl_AlarmThresholdType  Cl_alarmThresholdTable;
extern Cl_Uint32Type Treatdata[ID_MAX_TREAT_PARAM] ;

Cl_ReturnCodeType Cl_Standby_Controller(Cl_Mac_EventsType Cl_MacStandbyEvent)
{
		Cl_ReturnCodeType cl_stby_retval;
		Cl_Stby_Events cl_stdby_event;
		Cl_ConsoleTxCommandtype con_command = CON_TX_COMMAND_COMMAND_MAX;
		Cl_Uint8Type data =0;
		Cl_Uint8Type dataarray[4];
		ClStbyAlarmIdType ClstbyAlarmId;
						STBStateDatatype abfdata;
						Cl_Uint8Type abfdataarray[5] =  {0,0,0,0,0};

		// dummy = Cl_MacStandbyEvent;

	//	cl_stby_translatemacevent( dummy, &cl_stdby_event);
		cl_stby_retval = cl_stby_translatemacevent( Cl_MacStandbyEvent, &cl_stdby_event);
		switch(cl_stbystate)
		{
			case CL_STBY_STATE_INIT:
		//	printf("K\n");
			break;
			case CL_STBY_STATE_IDLE:
				switch (cl_stdby_event)
				{
					
					
				case EVENT_STBY_CLR_ALARM :
				
				con_command = CON_TX_COMMAND_ALARM_CLEARED;
						data = (Cl_Uint8Type)COMMAND_RESULT_SUCCESS;
						Cl_SendDatatoconsole(con_command,&data,0);
						
										sv_cntrl_resetyellowalarm();
										sv_cntrl_resetredalarm();
										sv_cntrl_nobuzzer();
										
				
				
				break;	
				case EVENT_STBY_ENTER_STANDBY:
				
			//	cl_stby_retval =  Cl_AlarmActivateAlarms(LEVELSWITCH_OFF_TO_ON , true );
			//	cl_stby_retval =  Cl_AlarmActivateAlarms(LEVELSWITCH2 , true );
												
				con_command = CON_TX_COMMAND_SYSDATA;
				//cl_stby_retval = (Cl_Uint8Type)sv_nvmgetdata(NV_RINSE_NVM_RINSE_STATUS, &data);
				dataarray[0] = CON_TX_PARAM_SYS_SOFT_VERSION;
			//	cl_stby_retval = (Cl_Uint8Type)sv_nvmgetdata(NV_RINSE_NVM_RINSE_STATUS, &dataarray[1]);
			//	cl_stby_retval = cl_testgetversion(&dataarray[1]);								
				//Reset the  OPENFILL time count
				
				dataarray[1] = SYS_VER;
				cl_stby_retval = Cl_SendDatatoconsole(con_command,&dataarray,2);
				
		//		cl_stbystate = CL_STBY_STATE_V1;
				break;
				case EVENT_STBY_SET_BLDPMP_ON:
				prev_bpstate = 1;
					cl_bp_controller(CL_BP_EVENT_START,0);
				break;
				case EVENT_STBY_SET_BLDPMP_OFF:
				prev_bpstate = 0;
					cl_bp_controller(CL_BP_EVENT_STOP,0);
				break;
				case EVENT_STBY__SET_BP_RATE:
					if(Cl_ConsoleRxMsg.msgready == true)
					{
						
						
			
						cl_Datastreamtype cl_temp  ;
						cl_temp.bytearray[0] = Cl_ConsoleRxMsg.data.byte[0];
						cl_temp.bytearray[1] = Cl_ConsoleRxMsg.data.byte[1];
						cl_temp.bytearray[2] = Cl_ConsoleRxMsg.data.byte[2];
						cl_temp.bytearray[3] = Cl_ConsoleRxMsg.data.byte[3];
						
						Treatdata[ID_bloodrate] = cl_temp.Twobyte;
						
						cl_bp_controller(CL_BP_EVENT_SET_BP_RATE,cl_temp.Twobyte);
						
						
					}
					cl_stdby_event == EVENT_STBY_NULL;
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
				
				cl_stdby_event == EVENT_STBY_NULL;
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
				
				cl_stdby_event == EVENT_STBY_NULL;
				break;
				
				default:
				break;	
				}


				break;
			break;
			case CL_STBY_STATE_STANDBY_RINSE:
			//printf("J\n");
			switch (cl_stdby_event)
			{
					case EVENT_STBY_START_RINSE:
					//check for alarms and then notify if needed
						cl_stbystate = CL_STBY_STATE_IDLE;
					break;
					case EVENT_STBY_DIALYSIS_PREP:
					// check for alarms and change hardware status as required before going to dialysis prepration
					break;
					case EVENT_STBY_GET_DATA:
					if(Cl_ConsoleRxMsg.msgready == true)
					{
					
						switch(Cl_ConsoleRxMsg.data.byte[0])
						{
							case CON_RX_PARAM_DATA_RINSE_STATUS:
								//	cl_wait(10);
													
									con_command = CON_TX_COMMAND_SYSDATA;
									//cl_stby_retval = (Cl_Uint8Type)sv_nvmgetdata(NV_RINSE_NVM_RINSE_STATUS, &data);
									dataarray[0] = CON_TX_PARAM_DATA_RINSE_STATUS;
									cl_stby_retval = (Cl_Uint8Type)sv_nvmgetdata(NV_NVM_RINSE_STATUS, &dataarray[1]);
									cl_stby_retval = Cl_SendDatatoconsole(con_command,&dataarray,2);
									//cl_wait(100);
							break;
						{
							case CON_RX_PARAM_DATA_DISINF_STATUS:
								//	cl_wait(10);
													
									con_command = CON_TX_COMMAND_SYSDATA;									
									dataarray[0] = CON_TX_PARAM_DATA_DISINF_STATUS;	
									cl_stby_retval = (Cl_Uint8Type)sv_nvmgetdata(NV_NVM_RINSE_STATUS, &dataarray[1]);																		
									cl_stby_retval = (Cl_Uint8Type)sv_nvmgetdata(NV_NVM_DISINF_STATUS, &dataarray[1]);
									cl_stby_retval = Cl_SendDatatoconsole(con_command,&dataarray,2);

							break;
							default:
							break;
						}

					}
				}
				break;
				
				case EVENT_STBY_SET_DATA:
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
						break;
						case ID_dflowllimit:
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
				
				case EVENT_STBY_ENTER_STANDBY:
				
				//cl_stby_retval =  Cl_AlarmActivateAlarms(LEVELSWITCH_OFF_TO_ON , true );
				//cl_stby_retval =  Cl_AlarmActivateAlarms(LEVELSWITCH2 , true );
				
				cl_stbystate = CL_STBY_STATE_IDLE;

				break;
				case EVENT_STBY_ALARM:
				//printf("\n");
				//sv_setflowpath(V1);
				//Activate V1 time count
				//change state to V1
				// send update to console
				//Cl_SendDatatoconsole();
				//command = CON_TX_HELLO;
				con_command = CON_TX_COMMAND_ALARM;
				//cl_stby_retval =  Cl_AlarmActivateAlarms(LEVELSWITCH_OFF_TO_ON , false );
				//cl_stby_retval =  Cl_AlarmActivateAlarms(LEVELSWITCH2 , false );
				cl_stby_retval = Cl_SendDatatoconsole(con_command,&data,0);

				break;
				case	EVENT_STBY_TICK_SECOND:
				Cl_StbRinsesecondscounter++;
				cl_stby_retval = Cl_StabyRinse_UpdateTimeInfo();
				break;
				case EVENT_STBY_TICK_MINUTES:
				Cl_StbRinsesecondscounter =0;
				cl_stbystate = CL_STBY_STATE_IDLE;
			//	printf("M");
				break;
				case EVENT_STBY_TICK_HOURS:
			//	printf("H");
				break;

				
				default:
				break;
			}
			break;

			case CL_STBY_STATE_V1:
			
			switch (cl_stdby_event)
			{
				case EVENT_STBY_DIALYSIS_PREP:
				// check for alarms and change hardware status as required before going to dialysis prepration
				break;
				case EVENT_STBY_GET_DATA:
				
								if(Cl_ConsoleRxMsg.msgready == true)
								{
									
									switch(Cl_ConsoleRxMsg.data.byte[0])
									{
										case CON_RX_PARAM_DATA_RINSE_STATUS:
									//	cl_wait(10);
										
										con_command = CON_TX_COMMAND_SYSDATA;
										//cl_stby_retval = (Cl_Uint8Type)sv_nvmgetdata(NV_RINSE_NVM_RINSE_STATUS, &data);
										dataarray[0] = CON_TX_PARAM_DATA_RINSE_STATUS;
										cl_stby_retval = (Cl_Uint8Type)sv_nvmgetdata(NV_NVM_RINSE_STATUS, &dataarray[1]);
										
										//Reset the  OPENFILL time count
										cl_stby_retval = Cl_SendDatatoconsole(con_command,&dataarray,(Cl_Uint8Type)2);
									//	cl_wait(
										break;
										case CON_RX_PARAM_DATA_DISINF_STATUS:
											//	cl_wait(10);
													
												con_command = CON_TX_COMMAND_SYSDATA;
												//cl_stby_retval = (Cl_Uint8Type)sv_nvmgetdata(NV_RINSE_NVM_RINSE_STATUS, &data);
												dataarray[0] = CON_TX_PARAM_DATA_DISINF_STATUS;	
												cl_stby_retval = (Cl_Uint8Type)sv_nvmgetdata(NV_NVM_RINSE_STATUS, &dataarray[1]);
												cl_stby_retval = Cl_SendDatatoconsole(con_command,&dataarray,2);
												//cl_wait(100);
						break;
										default:
										break;
									}
								}

				break;
				case EVENT_STBY_SET_DATA:
				if(Cl_ConsoleRxMsg.msgready == true)
				{
					
					switch(Cl_ConsoleRxMsg.data.byte[0])
					{
						case CON_RX_PARAM_DATA_RINSE_STATUS:
					//	cl_wait(10);
						
						con_command = CON_TX_COMMAND_SYSDATA;
						//cl_stby_retval = (Cl_Uint8Type)sv_nvmgetdata(NV_RINSE_NVM_RINSE_STATUS, &data);
						dataarray[0] = CON_TX_PARAM_DATA_RINSE_STATUS;
					//	cl_stby_retval =sv_nvmsetdata(Cl_ConsoleRxMsg.data.byte[0],&Cl_ConsoleRxMsg.data.byte[1]);
						cl_stby_retval = (Cl_Uint8Type)sv_nvmgetdata(NV_NVM_RINSE_STATUS, &dataarray[1]);
						
						//Reset the  OPENFILL time count
						cl_stby_retval = Cl_SendDatatoconsole(con_command,&dataarray,2);
						//cl_wait(10);
						break;
						default:
						break;
					}
					
				}
				break;
				case EVENT_STBY_ALARM:
				//printf("\n");
				//sv_setflowpath(V1);
				//Activate V1 time count
				//change state to V1
				// send update to console
				//Cl_SendDatatoconsole();
				//command = CON_TX_HELLO;
				con_command = CON_TX_COMMAND_ALARM;
				//cl_stby_retval =  Cl_AlarmActivateAlarms(LEVELSWITCH_OFF_TO_ON , false );
				//cl_stby_retval =  Cl_AlarmActivateAlarms(LEVELSWITCH2 , false );
				//cl_stby_retval = Cl_SendDatatoconsole(con_command,&data,0);
				cl_stby_retval = Cl_standby_ProcessAlarms();

				break;
				case	EVENT_STBY_TICK_SECOND:
				Cl_StbRinsesecondscounter++;
				//cl_stby_retval = Cl_StabyRinse_UpdateTimeInfo();
				//cl_stby_retval = Cl_standby_ProcessAlarms();
				cl_stby_retval = Cl_standby_UpdateAlarmTable(&ClstbyAlarmId );
				
				
				break;
				case EVENT_STBY_TICK_MINUTES:
				Cl_StbRinsesecondscounter =0;
			//	printf("M");
				break;
				case EVENT_STBY_TICK_HOURS:
			//	printf("H");
				break;

				
				default:
				break;
			}
			break;
			case CL_STBY_STATE_V2:
			switch (cl_stdby_event)
			{
				case EVENT_STBY_DIALYSIS_PREP:
				// check for alarms and change hardware status as required before going to dialysis prepration
				break;
				case EVENT_STBY_GET_DATA:
								if(Cl_ConsoleRxMsg.msgready == true)
								{
									
									switch(Cl_ConsoleRxMsg.data.byte[0])
									{
										case CON_RX_PARAM_DATA_RINSE_STATUS:
										//	cl_wait(10);
										
										con_command = CON_TX_COMMAND_SYSDATA;
										//cl_stby_retval = (Cl_Uint8Type)sv_nvmgetdata(NV_RINSE_NVM_RINSE_STATUS, &data);
										dataarray[0] = CON_TX_PARAM_DATA_RINSE_STATUS;
										cl_stby_retval = (Cl_Uint8Type)sv_nvmgetdata(NV_NVM_RINSE_STATUS, &dataarray[1]);
										
										//Reset the  OPENFILL time count
										cl_stby_retval = Cl_SendDatatoconsole(con_command,&dataarray,2);
										//cl_wait(10);
										break;
										case CON_RX_PARAM_DATA_DISINF_STATUS:
											//	cl_wait(10);
													
												con_command = CON_TX_COMMAND_SYSDATA;
												//cl_stby_retval = (Cl_Uint8Type)sv_nvmgetdata(NV_RINSE_NVM_RINSE_STATUS, &data);
												dataarray[0] = CON_TX_PARAM_DATA_DISINF_STATUS;								
												cl_stby_retval = (Cl_Uint8Type)sv_nvmgetdata(NV_NVM_RINSE_STATUS, &dataarray[1]);
													
												//Reset the  OPENFILL time count
												cl_stby_retval = Cl_SendDatatoconsole(con_command,&dataarray,2);
												//cl_wait(100);
										break;
										default:
										break;
									}
								}
				break;
				case EVENT_STBY_ALARM:
				//printf("\n");
				//sv_setflowpath(V1);
				//Activate V1 time count
				//change state to V1
				// send update to console
				//Cl_SendDatatoconsole();
				//command = CON_TX_HELLO;
				con_command = CON_TX_COMMAND_ALARM;
				//cl_stby_retval =  Cl_AlarmActivateAlarms(LEVELSWITCH_OFF_TO_ON , false );
				//cl_stby_retval =  Cl_AlarmActivateAlarms(LEVELSWITCH2 , false );
				cl_stby_retval = Cl_SendDatatoconsole(con_command,&data,0);

				break;
				case	EVENT_STBY_TICK_SECOND:
				Cl_StbRinsesecondscounter++;
				cl_stby_retval = Cl_StabyRinse_UpdateTimeInfo();
				break;
				case EVENT_STBY_TICK_MINUTES:
				Cl_StbRinsesecondscounter =0;
			//	printf("M");
				break;
				case EVENT_STBY_TICK_HOURS:
			//	printf("H");
				break;

				
				default:
				break;
			}
			break;
			case CL_STBY_STATE_BO1_V4:
						switch (cl_stdby_event)
						{
							case EVENT_STBY_GET_DATA:
										if(Cl_ConsoleRxMsg.msgready == true)
										{
											
											switch(Cl_ConsoleRxMsg.data.byte[0])
											{
												case CON_RX_PARAM_DATA_RINSE_STATUS:
												//	cl_wait(10);
												
												con_command = CON_TX_COMMAND_SYSDATA;
												//cl_stby_retval = (Cl_Uint8Type)sv_nvmgetdata(NV_RINSE_NVM_RINSE_STATUS, &data);
												dataarray[0] = CON_TX_PARAM_DATA_RINSE_STATUS;
												cl_stby_retval = (Cl_Uint8Type)sv_nvmgetdata(NV_NVM_RINSE_STATUS, &dataarray[1]);
	
												cl_stby_retval = Cl_SendDatatoconsole(con_command,&dataarray,2);
												//cl_wait(10);
												break;
												case CON_RX_PARAM_DATA_DISINF_STATUS:
													//	cl_wait(10);
													
														con_command = CON_TX_COMMAND_SYSDATA;
														//cl_stby_retval = (Cl_Uint8Type)sv_nvmgetdata(NV_RINSE_NVM_RINSE_STATUS, &data);
														dataarray[0] = CON_TX_PARAM_DATA_DISINF_STATUS;								
														cl_stby_retval = (Cl_Uint8Type)sv_nvmgetdata(NV_NVM_RINSE_STATUS, &dataarray[1]);
													
														//Reset the  OPENFILL time count
														cl_stby_retval = Cl_SendDatatoconsole(con_command,&dataarray,2);
														//cl_wait(100);
												break;
												default:
												break;
											}
										}
							break;

							case EVENT_STBY_DIALYSIS_PREP:
							// check for alarms and change hardware status as required before going to dialysis prepration
							break;
							default:
							break;
						}
			break;
			case CL_STBY_STATE_BO2_V4:
						switch (cl_stdby_event)
						{
							case EVENT_STBY_GET_DATA:
								if(Cl_ConsoleRxMsg.msgready == true)
								{
									
									switch(Cl_ConsoleRxMsg.data.byte[0])
									{
										case CON_RX_PARAM_DATA_RINSE_STATUS:
										//	cl_wait(10);
										
										con_command = CON_TX_COMMAND_SYSDATA;
										//cl_stby_retval = (Cl_Uint8Type)sv_nvmgetdata(NV_RINSE_NVM_RINSE_STATUS, &data);
										dataarray[0] = CON_TX_PARAM_DATA_RINSE_STATUS;
										cl_stby_retval = (Cl_Uint8Type)sv_nvmgetdata(NV_NVM_RINSE_STATUS, &dataarray[1]);
										
										//Reset the  OPENFILL time count
										cl_stby_retval = Cl_SendDatatoconsole(con_command,&dataarray,2);
										//cl_wait(10);
										break;
										case CON_RX_PARAM_DATA_DISINF_STATUS:
											//	cl_wait(10);
													
												con_command = CON_TX_COMMAND_SYSDATA;
												//cl_stby_retval = (Cl_Uint8Type)sv_nvmgetdata(NV_RINSE_NVM_RINSE_STATUS, &data);
												dataarray[0] = CON_TX_PARAM_DATA_DISINF_STATUS;								
												cl_stby_retval = (Cl_Uint8Type)sv_nvmgetdata(NV_NVM_RINSE_STATUS, &dataarray[1]);
													
												//Reset the  OPENFILL time count
												cl_stby_retval = Cl_SendDatatoconsole(con_command,&dataarray,2);
												//cl_wait(100);
										break;
										default:
										break;
									}
								}
							break;

							case EVENT_STBY_DIALYSIS_PREP:
							// check for alarms and change hardware status as required before going to dialysis prepration
							break;
							default:
							break;
						}
			case CL_STBY_STATE_BO1_V13V14:
						switch (cl_stdby_event)
						{
							case EVENT_STBY_GET_DATA:
								if(Cl_ConsoleRxMsg.msgready == true)
								{
									
									switch(Cl_ConsoleRxMsg.data.byte[0])
									{
										case CON_RX_PARAM_DATA_RINSE_STATUS:
										//	cl_wait(10);
										
										con_command = CON_TX_COMMAND_SYSDATA;
										//cl_stby_retval = (Cl_Uint8Type)sv_nvmgetdata(NV_RINSE_NVM_RINSE_STATUS, &data);
										dataarray[0] = CON_TX_PARAM_DATA_RINSE_STATUS;
										cl_stby_retval = (Cl_Uint8Type)sv_nvmgetdata(NV_NVM_RINSE_STATUS, &dataarray[1]);
										
										//Reset the  OPENFILL time count
										cl_stby_retval = Cl_SendDatatoconsole(con_command,&dataarray,2);
										//cl_wait(10);
										break;
										case CON_RX_PARAM_DATA_DISINF_STATUS:
											//	cl_wait(10);
													
												con_command = CON_TX_COMMAND_SYSDATA;
												//cl_stby_retval = (Cl_Uint8Type)sv_nvmgetdata(NV_RINSE_NVM_RINSE_STATUS, &data);
												dataarray[0] = CON_TX_PARAM_DATA_DISINF_STATUS;								
												cl_stby_retval = (Cl_Uint8Type)sv_nvmgetdata(NV_NVM_RINSE_STATUS, &dataarray[1]);
													
												//Reset the  OPENFILL time count
												cl_stby_retval = Cl_SendDatatoconsole(con_command,&dataarray,2);
												//cl_wait(100);
										break;
										default:
										break;
									}
								}
							break;

							case EVENT_STBY_DIALYSIS_PREP:
							// check for alarms and change hardware status as required before going to dialysis prepration
							break;
							default:
							break;
						}
			break;
			case CL_STBY_STATE_BO2_V13V14:
						switch (cl_stdby_event)
						{
							case EVENT_STBY_GET_DATA:
								if(Cl_ConsoleRxMsg.msgready == true)
								{
									
									switch(Cl_ConsoleRxMsg.data.byte[0])
									{
										case CON_RX_PARAM_DATA_RINSE_STATUS:
										//	cl_wait(10);
										
										con_command = CON_TX_COMMAND_SYSDATA;
										//cl_stby_retval = (Cl_Uint8Type)sv_nvmgetdata(NV_RINSE_NVM_RINSE_STATUS, &data);
										dataarray[0] = CON_TX_PARAM_DATA_RINSE_STATUS;
										cl_stby_retval = (Cl_Uint8Type)sv_nvmgetdata(NV_NVM_RINSE_STATUS, &dataarray[1]);
										
										//Reset the  OPENFILL time count
										cl_stby_retval = Cl_SendDatatoconsole(con_command,&dataarray,2);
										//cl_wait(10);
										break;
										case CON_RX_PARAM_DATA_DISINF_STATUS:
											//	cl_wait(10);
													
												con_command = CON_TX_COMMAND_SYSDATA;
												//cl_stby_retval = (Cl_Uint8Type)sv_nvmgetdata(NV_RINSE_NVM_RINSE_STATUS, &data);
												dataarray[0] = CON_TX_PARAM_DATA_DISINF_STATUS;								
												cl_stby_retval = (Cl_Uint8Type)sv_nvmgetdata(NV_NVM_RINSE_STATUS, &dataarray[1]);
													
												//Reset the  OPENFILL time count
												cl_stby_retval = Cl_SendDatatoconsole(con_command,&dataarray,2);
												//cl_wait(100);
										break;
										default:
										break;
									}
								}
							break;

							case EVENT_STBY_DIALYSIS_PREP:
							// check for alarms and change hardware status as required before going to dialysis prepration
							break;
							default:
							break;
						}
			break;
			case CL_STBY_STATE_PAUSE:
			break;
			case CL_STBY_STATE_STOP:
			break;

			default:
			break;
		}

	
return CL_OK;
}



Cl_ReturnCodeType  cl_stby_translatemacevent(Cl_Mac_EventsType Cl_MacStbyEvent,Cl_Stby_Events* cl_stdby_event)
{
	//dummy1  =  Cl_MacStbyEvent;
	
	//switch(dummy1)
	
		*cl_stdby_event =	EVENT_STBY_NULL;
		
	switch(Cl_MacStbyEvent)
	{
		case  EVT_COMMAND_ENTER_STANDBY:
		*cl_stdby_event = EVENT_STBY_ENTER_STANDBY;
		break;
		case EVT_TICK_100M:
		*cl_stdby_event = EVENT_STBY_TICK_100MS;
		break;
		case EVT_TICK_SEC:
		*cl_stdby_event = EVENT_STBY_TICK_SECOND;
		break;
		case EVT_TICK_MIN:
		*cl_stdby_event = EVENT_STBY_TICK_MINUTES;
		break;
		case EVT_TICK_HOUR:
		break;
		case  EVT_ALARM_TRIGGERED:
		*cl_stdby_event = EVENT_STBY_ALARM;
		break;
		case EVT_CONSOLE_COMMAND_GET_DATA:
		*cl_stdby_event = EVENT_STBY_GET_DATA;
		break;
		case  EVT_CONSOLE_COMMAND_RINSE_START:
		*cl_stdby_event= EVENT_STBY_START_RINSE;
		break;
		case EVT_CONSOLE_COMMAND_DIALYSIS_PREP:
		*cl_stdby_event = EVENT_STBY_DIALYSIS_PREP;
		break; 
		case EVT_CONSOLE_COMMAND_SET_BLDPMP_ON:
			*cl_stdby_event = EVENT_STBY_SET_BLDPMP_ON;
		break;
		case EVT_CONSOLE_COMMAND_SET_BLDPMP_OFF:
			*cl_stdby_event = EVENT_STBY_SET_BLDPMP_OFF;
		break;
		case EVT_CONSOLE_COMMAND_SET_BLDPUMPRATE:
		*cl_stdby_event = EVENT_STBY__SET_BP_RATE;
		break;
		
		case EVENT_STBY__SET_BP_RATE:
			*cl_stdby_event = EVENT_STBY__SET_BP_RATE;
		break;
		case EVT_CONSOLE_COMMAND_CLR_ALARM:
			*cl_stdby_event = EVENT_STBY_CLR_ALARM ;
		break;
		case EVT_CONSOLE_COMMAND_SET_BLDPUMPINC_MANUAL:
		*cl_stdby_event = EVT_CONSOLE_COMMAND_SET_BLDPUMPINC_MANUAL;
		break;
		case EVT_CONSOLE_COMMAND_SET_BLDPUMPDEC_MANUAL:
		*cl_stdby_event = EVT_CONSOLE_COMMAND_SET_BLDPUMPDEC_MANUAL;
		break;
		default:
		break;
	}
	return CL_OK;
}

 Cl_ReturnCodeType Cl_stby_init(void)
 {
	 Cl_Uint16Type temp;
	 	 Cl_ReturnCodeType cl_stby_retval = CL_ERROR;
		  
		 
		  	 for(temp = 1;temp<=20; temp++)
		 {
				sv_cntrl_deactivate_valve((sv_valvetype)temp);
				cl_wait(10);
		 }
		  
	 	cl_stby_retval =  sv_cntrl_deactivatepump(DCMOTOR1);
	 	cl_stby_retval =  sv_cntrl_deactivatepump(DCMOTOR2);
	 //	cl_stby_retval =  sv_cntrl_deactivatepump(BLOODPUMP);
	 	cl_stby_retval =  sv_cntrl_deactivatepump(UFPUMP);
	 	cl_stby_retval =  sv_cntrl_deactivatepump(HEPARINPUMP);
		cl_stby_retval =  sv_cntrl_deactivatepump(VLA_PUMP);
	//	cl_stby_retval =  sv_cntrl_poweroffheater();

		
	
			cl_stby_retval =  sv_cs_setpotvalue((2200 * 1024)/10000);	
			//cl_stby_retval =  sv_cs_setcondpotvalue((3200 * 1024)/10000);
			cl_stby_retval =  Cl_Conductivity_Sensor_Controller(COND_EVENT_INIT);
		
		

				
	 return CL_OK;
 }
 
 Cl_ReturnCodeType Cl_StabyRinse_UpdateTimeInfo(void)
 {
	 
	 Cl_ReturnCodeType cl_stby_retval = CL_ERROR;
	 Cl_ConsoleTxCommandtype command = CON_TX_COMMAND_COMMAND_MAX;
	 Cl_Uint8Type data[7] = {0,0,0,0,0 ,0,0};
	 
	command = CON_TX_COMMAND_REM_TIME;
	
	data[0]= (Cl_Uint8Type)STANDBY_DATA;
	data[1]= (Cl_Uint8Type)Cl_StbRinseTotalhourscounter; 
	data[2]= (Cl_Uint8Type)Cl_StbRinseTotalMinutescounter;
	data[3]= (Cl_Uint8Type)Cl_StbRinsesecondscounter;
	data[4]= (Cl_Uint8Type) (CL_STBYRINSE_TIMEOUT_HRS - Cl_StbRinseTotalhourscounter );
	data[5]= (Cl_Uint8Type) (CL_STBYRINSE_TIMEOUT_MIN - Cl_StbRinseTotalMinutescounter );
	data[6]= (Cl_Uint8Type) (60 - Cl_StbRinsesecondscounter );
	
	cl_stby_retval = Cl_SendDatatoconsole(command,&data,7);
	//Cl_rinseretcode = Cl_SendDatatoconsole(command,&data,1);
	
	return CL_OK;
	
 }
 
 Cl_ReturnCodeType Cl_standby_ProcessAlarms( void)
{

	Cl_ReturnCodeType 	cl_stby_retval = CL_OK;
	Cl_Uint8Type tempcount = 0;
	Cl_Uint8Type data[2];
	Cl_ConsoleTxCommandtype command = CON_TX_COMMAND_COMMAND_MAX;
	
	//cl_wait(10);
	
	for (tempcount = 0 ; tempcount < CL_STBY_ALRM_MAX ; tempcount++)
	{
		
		Cl_Alarm_GetAlarmStatus(Cl_StandbyAlarmTable[tempcount].Cl_RinseAlarmId,&Cl_StandbyAlarmTable[tempcount].IsActive);
		if (Cl_StandbyAlarmTable[tempcount].IsActive)
		{
			if(!Cl_StandbyAlarmTable[tempcount].IsRaised)
				{
					

				//	data[0] = (Cl_Uint8Type)Cl_StandbyAlarmTable[tempcount].Cl_RinseAlarmId;
				//	data[1] = 1;
			
				//	command = CON_TX_COMMAND_ALARM;
				//	cl_stby_retval = Cl_SendDatatoconsole(command,&data,2);

					
					
			
				//	Cl_StandbyAlarmTable[tempcount].IsRaised = true;
				}
				else
				{
				}
			
		}

	}

return (cl_stby_retval );
	   	
}

Cl_ReturnCodeType Cl_standby_UpdateAlarmTable(ClStbyAlarmIdType* ClstbyAlarmId )
{
	Cl_ReturnCodeType 	cl_stby_retval = CL_OK;
	Cl_Uint8Type tempcount = 0;
	Cl_Uint8Type data[2];
	
	Cl_ConsoleTxCommandtype command = CON_TX_COMMAND_COMMAND_MAX;
	
	for (tempcount = 0 ; tempcount < CL_STBY_ALRM_MAX ; tempcount++)
	{
		
		Cl_Alarm_GetAlarmStatus(Cl_StandbyAlarmTable[tempcount].Cl_RinseAlarmId,&Cl_StandbyAlarmTable[tempcount].IsActive);
		if (Cl_StandbyAlarmTable[tempcount].IsActive)
		{
			if(!Cl_StandbyAlarmTable[tempcount].IsRaised)
				{
					data[0] = (Cl_Uint8Type)Cl_StandbyAlarmTable[tempcount].Cl_RinseAlarmId;
					data[1] = 1;
					command = CON_TX_COMMAND_ALARM;
					cl_stby_retval = Cl_SendDatatoconsole(command,&data,2);
					Cl_StandbyAlarmTable[tempcount].IsRaised = true;
					
			
				}
				else
				{
					Cl_StandbyAlarmTable[tempcount].alarmseccount++;
					Cl_StandbyAlarmTable[tempcount].alarm5seccount++;
					
					if (Cl_StandbyAlarmTable[tempcount].alarmseccount > 5 )
					{
						// do  5 sec action
						Cl_StandbyAlarmTable[tempcount].alarm5seccount = 0;
					}
					if (Cl_StandbyAlarmTable[tempcount].alarmseccount >= 60 )
					{
						// do 1 min action
						Cl_StandbyAlarmTable[tempcount].alarmseccount = 0;
					}
					if (Cl_StandbyAlarmTable[tempcount].alarmmincount > 5 )
					{
						 // do five min action
					}
				}
			
		}
		else
		{
			// alarm was present before , but not active now.
			if(Cl_StandbyAlarmTable[tempcount].IsRaised)
			{
				
				Cl_StandbyAlarmTable[tempcount].IsRaised = false;
				command = CON_TX_COMMAND_ALARM;
				data[0] = (Cl_Uint8Type)(Cl_StandbyAlarmTable[tempcount].Cl_RinseAlarmId);
				data[1] = (Cl_Uint8Type) 0;
				cl_stby_retval = Cl_SendDatatoconsole(command,&data,2);
			}

		}	

	}

return (cl_stby_retval );
}
