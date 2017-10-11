/*
 * cl_disinf_controller.c
 *
 * Created: 12/27/2013 12:27:27 PM
 *  Author: user
 */ 

#include "cl_disinf_controller_include.h"
#include "cl_disinf_controller_proto.h"
#include "sv_interface_fun.h"

extern float avgtemp2_dummy;
volatile Cl_Uint8Type alarm_flag=0,alarm_flag1 = 0;

extern volatile float temprature_final_value_1,temprature_final_value_2,temprature_final_value_3;
extern volatile float pressure_final_apt,pressure_final_vpt,pressure_final_ps1,pressure_final_ps2,pressure_final_ps3;
extern volatile float cond_final_cs3;

extern Cl_ReturnCodeType  sv_nvmgetdata(Cl_Uint8Type,Cl_Uint8Type*);
extern Cl_ReturnCodeType cl_wait(Cl_Uint32Type );
extern int CSeventcount;
extern float res_temp_value;
extern Cl_Uint8Type hotrinse_flag;
Cl_ReturnCodeType  CL_DisinfAlarmActon(Cl_NewAlarmIdType cl_disinfalarmid);
Cl_disinf_StatesType cl_disinfstate = CL_DISINF_STATE_IDLE;
Cl_disinf_SubStatesType Cl_disinf_SubState = CL_DISINF_RINSING_IDLE;
int Cl_disinfsecondscounter =0 ,Cl_disinfMinutescounter=0, Cl_disinfshourscounter=0,Cl_disinfOpenFillsecondscounter=0;
int Cl_disinfsecondscounter_1 =0 ,Cl_disinfMinutescounter_1=0, Cl_disinfshourscounter_1=0,Cl_disinfOpenFillsecondscounter_1=0;
int Cl_disinfTotalMinutescounter=0, Cl_disinfsTotalhourscounter=0;
int Cl_disinfTotalMinutescounter_1=0, Cl_disinfsTotalhourscounter_1=0;
Cl_BoolType Cl_disinfOpenFillTimeOut = true;
extern Cl_Uint8Type flag_hotcitric, flag_hotrinse,flag_na , flag_citric;
Cl_Uint16Type sec_count=0;
volatile Cl_Uint8Type openfill_flag=0;
extern Cl_ConsoleMsgType Cl_ConsoleRxMsg;
Cl_Uint16Type tempcount;
cl_DlsInfDatatype heatcutdata;
static float heatcutavgtmp2 = 0;
cl_Datastreamtype cl_temp_diinftype  ;

Cl_BoolType IntakeReadyFlag = false;

  /*
 * \brief  Cl_disinf_init  - The initialization of the disninfection  module
 * \param  
 * \ 
 * \return CL_OK on success. 
 * \Globals used : 
 */

 Cl_ReturnCodeType Cl_disinf_init(void)
 {
	 Cl_ReturnCodeType cl_disinf_retval;
	
	 return CL_OK;
 }
 
  /*
 * \brief  Cl_disinf_start  - The starting request handler for the disninfection  module
 * \param  
 * \ 
 * \return CL_OK on success. 
 * \Globals used : 
 */
  Cl_ReturnCodeType Cl_disinf_start(void)
  {
	  
	  return CL_OK;
  }
  
  
  DisinfAlarmsType Cl_DisinfAlarmTable[CL_DISINF_ALRM_MAX] =
  {
	  {ACID_IN_CLOSED,CL_ALARM_ALARM,false,false,false},
	  {BICARB_IN_CLOSED,CL_ALARM_ALARM,false,false,false},
	  {HOLDER1STATUS_OPEN,CL_ALARM_ALARM,false,false,false},
	  {HOLDER2STATUS_OPEN,CL_ALARM_ALARM,false,false,false},
	  {TEMP3_HIGH_THRESHOLD,CL_ALARM_ALARM,false,false,false},
	  {PS1_HIGH_THRESHOLD,CL_ALARM_ALARM,false,false,false},
	  {PS1_LOW_THRESHOLD,CL_ALARM_ALARM,false,false,false},
	  {PS2_HIGH_THRESHOLD,CL_ALARM_ALARM,false,false,false},
	  {PS2_LOW_THRESHOLD,CL_ALARM_ALARM,false,false,false},
	  {PS3_HIGH_THRESHOLD,CL_ALARM_ALARM,false,false,false},
	  {PS3_LOW_THRESHOLD,CL_ALARM_ALARM,false,false,false},
	  {COND_STATUS_HIGH,CL_ALARM_ALARM,false,false,false},
  };



  /*
 * \brief  Cl_disinf_controller  - The disninfection  module events and actions are handled in this function
 * \param  
 * \ 
 * \return CL_OK on success. 
 * \Globals used : cl_disinfstate,Cl_ConsoleRxMsg
 */
Cl_ReturnCodeType Cl_disinf_controller(Cl_Mac_EventsType Cl_MacDisinfEvent)
{
		Cl_ReturnCodeType  Cl_disinfretcode = CL_OK;
		Cl_disinf_Eevents cl_disinfevent = EVENT_NULL_EVENT;
		Cl_NewAlarmIdType cl_disinfalarmid;
		CldisinfAlarmIdType CldisinfAlarmId;

		
		Cl_ConsoleTxCommandtype command = CON_TX_COMMAND_COMMAND_MAX;
		Cl_Uint8Type dataarray[4] = {0,0,0,0};
		Cl_Uint8Type data = 0;
		Cl_BoolType alarmstatus1 = false,alarmstatus2 = false,alarmstatus3 = false, flowstatus = false;


		Cl_disinfretcode = cl_disinf_translatemacevent(Cl_MacDisinfEvent,&cl_disinfevent);
		
		
		
		switch(cl_disinfstate)
		{

			case CL_DISINF_STATE_INIT:
			break;
			case CL_DISINF_STATE_IDLE:
			switch (cl_disinfevent)
			{
				case EVT_DISINF_GET_DATA:
				if(Cl_ConsoleRxMsg.msgready == true)
				{
					
					switch(Cl_ConsoleRxMsg.data.byte[0])
					{
						case CON_RX_PARAM_DATA_DISINF_STATUS:
						command = CON_TX_COMMAND_SYSDATA;
						dataarray[0] = CON_TX_PARAM_DATA_DISINF_STATUS;
						Cl_disinfretcode = (Cl_Uint8Type)sv_nvmgetdata(NV_NVM_DISINF_STATUS, &dataarray[1]);
						Cl_disinfretcode = Cl_SendDatatoconsole(command,&dataarray,2);
						
						break;
						case CON_RX_PARAM_DATA_PRIME_STATUS:
						command = CON_TX_COMMAND_SYSDATA;
						dataarray[0] = CON_TX_PARAM_DATA_PRIME_STATUS;
						Cl_disinfretcode = (Cl_Uint8Type)sv_nvmgetdata(NV_NVM_PRIME_STATUS, &dataarray[1]);
						Cl_disinfretcode = Cl_SendDatatoconsole(command,&dataarray,2);
						default:
						break;
					}
				}
				break;
				
				case EVENT_STOP_DISINF:
				Cl_Disinf_StopDisinf();
				break;
				
				case EVENT_START_DISINF:
				
				//cl_wait(20);
				// manav05 commented as already read in mac
// 				if(Cl_ConsoleRxMsg.msgready == true)
// 				{
// 					
//  					cl_temp_diinftype.bytearray[0] = Cl_ConsoleRxMsg.data.byte[0];
//  					if (cl_temp_diinftype.bytearray[0] == 4)
//  					{
//  						flag_hotcitric = 1;
//  					}
//  					else if (cl_temp_diinftype.bytearray[0] == 3)
//  					{
//  						flag_na = 1;
//  					}
//  					else if(cl_temp_diinftype.bytearray[0] == 5){
//  						
//  						flag_citric = 1;
//  					}
//  					
// 				}
// 				
				
				command = CON_TX_COMMAND_DISINF_STARTED;
				data = (Cl_Uint8Type)COMMAND_RESULT_SUCCESS;
				Cl_disinfretcode = Cl_SendDatatoconsole(command,&data,1);
				Cl_Disinf_UpdateTimeInfo();
				Cl_disinfretcode = Cl_mac_apprequesthandler(MACREQ_START_PRE_DISINF_RINSE);
				cl_disinfstate = CL_DISINF_STATE_FIRST_RO_RINSE;
				break;
				
				default:
				break;
				
				
			}
			break;
			case CL_DISINF_STATE_FIRST_RO_RINSE:
				switch ( cl_disinfevent)
				{
					
					case EVENT_STOP_DISINF:
					Cl_Disinf_StopDisinf();
					cl_disinfstate = CL_DISINF_STATE_IDLE;
					break;
					
					
					case EVENT_DISINF_ALARM:
					
				//	Cl_disinfretcode =  Cl_disinf_ProcessAlarms();
					break;
					
					case EVENT_PRE_DISINFECT_RINSE_COMPLETED:
					
					
					
					
					//Send Pause
					CSeventcount = 0;
					 IntakeReadyFlag = true;
					 sv_cntrl_deactivate_valve(VALVE_ID1);
					 sv_cntrl_setpumpspeed(DCMOTOR2,1000);
					 sv_cntrl_setpumpspeed(DCMOTOR1,1000);
					 sv_cntrl_activatepump(DCMOTOR1);
					 sv_cntrl_activatepump(DCMOTOR2);
					 
					 Cl_bc_controller(BC_EVENT_DISINF_INTAKE);
					 
					cl_disinfstate = CL_DISINF_STATE_FIRST_RO_RINSCOMPLETED;
					
					break;
					case EVENT_DISINF_TICK_SECOND:
									
					Cl_disinfsecondscounter++;
					Cl_Disinf_SendDisinfStateData();
					if(Cl_disinfsecondscounter == 60)
					{
	
					//	UpdateDisinfMinuteTick();
						
							
					}

					break;
					
					default:break;
				}
				
			break;
			
			case CL_DISINF_STATE_FIRST_RO_RINSCOMPLETED:
				switch ( cl_disinfevent)
				{ 
					case EVENT_DISINF_TICK_500MS :
					if(CSeventcount > 2){
						
						Cl_bc_controller(BC_EVENT_STOP);
						command = CON_TX_COMMAND_DISINF_STAGE_COMPLETED;
						data = (Cl_Uint8Type)1;
						Cl_disinfretcode = Cl_SendDatatoconsole(command,&data,1);
						
						cl_wait(10);

						command = CON_TX_COMMAND_DISINF_PAUSED;
						data = (Cl_Uint8Type)0;
						Cl_disinfretcode = Cl_SendDatatoconsole(command,&data,0);
						
						//cl_disinfstate = CL_DISINF_STATE_FIRST_RO_RINSCOMPLETED;
						CSeventcount = 0;
						
					}
					
					break;
					
					case EVENT_STOP_DISINF:
					Cl_Disinf_StopDisinf();
					cl_disinfstate = CL_DISINF_STATE_IDLE;
					break;
					
									
					case EVT_RESUME_DISINF_STAGE:
					 Cl_Disinf_StartDisinf();
					 if (flag_hotcitric == 1 || flag_citric == 1)
					 {
						 Cl_disinfretcode =  Cl_AlarmConfigureAlarmType(BICARB_IN_OPEN,LOGIC_HIGH,0,0,0);
						 Cl_disinfretcode =  Cl_AlarmActivateAlarms(BICARB_IN_OPEN,true );
					 }
					 else if (flag_na == 1)
					 {
						 Cl_disinfretcode =  Cl_AlarmConfigureAlarmType(ACID_IN_OPEN,LOGIC_HIGH,0,0,0);
						  Cl_disinfretcode =  Cl_AlarmActivateAlarms(ACID_IN_OPEN,true );
					 }
					 
					 // run bc for 2 switches and disconnect v1 from level switch . keep v1 off till the intake is completed
				//	CSeventcount = 0;
					
					 
					 
					 
					// cl_disinfstate = CL_DISINF_STATE_INTER;
			//		 command = CON_TX_COMMAND_DISINF_PAUSED;
			//		  data = (Cl_Uint8Type)0;
			//		  Cl_disinfretcode = Cl_SendDatatoconsole(command,&data,0);
					
 					 command = CON_TX_COMMAND_DISINF_RESUMED;
 					 data = (Cl_Uint8Type)0;
					 Cl_disinfretcode = Cl_SendDatatoconsole(command,&data,0);
 					cl_disinfstate = CL_DISINF_STATE_DISINF_OPENFILL;
 					sv_cntrl_activate_valve(VALVE_ID16);
 					sv_cntrl_deactivate_valve(VALVE_ID18);

					 cl_disinfstate = CL_DISINF_STATE_DISINF_OPENFILL;
// 					 command = CON_TX_COMMAND_DISINF_RESUMED;
// 					 data = (Cl_Uint8Type)0;
// 					 Cl_disinfretcode = Cl_SendDatatoconsole(command,&data,0);
					break;
					
					case EVENT_DISINF_ALARM:
					Cl_disinfretcode = Cl_disinf_ProcessAlarms();
					break;
					
					
					default:break;
					
							
				}
			
			break;
			case CL_DISINF_STATE_INTER:
				switch (cl_disinfevent)
				{
					case EVENT_STOP_DISINF:
					Cl_Disinf_StopDisinf();
					cl_disinfstate = CL_DISINF_STATE_IDLE;
					break;
					
					case EVENT_DISINF_ALARM:
					Cl_disinfretcode =  Cl_disinf_ProcessAlarms();
					break;
					
					case EVENT_DISINF_CLR_ALARM:
					sv_cntrl_resetredalarm();
					sv_cntrl_nobuzzer();
					Cl_Disinf_resumeDisinf();
					if (alarm_flag == 1)
					{
					 cl_disinfstate = CL_DISINF_STATE_DISINF_OPENFILL;
						alarm_flag = 0;
					}
					break;
					
					case EVENT_DISINF_TICK_SECOND:
						for (tempcount = 0 ; tempcount < CL_DISINF_ALRM_MAX ; tempcount++)
						{
							//Cl_SendDatatoconsole(CON_TX_COMMAND_PRINTTEXT,"ENTER",8);
							Cl_Alarm_GetAlarmStatus(Cl_DisinfAlarmTable[tempcount].CldisinfAlarmId,&Cl_DisinfAlarmTable[tempcount].IsActive);
					 
							if (Cl_DisinfAlarmTable[tempcount].IsActive)
							{
								sv_cntrl_setredalarm();
								sv_cntrl_buzzer();
								Cl_SendDatatoconsole(CON_TX_COMMAND_PRINTTEXT,"ENTER_1",8);
								Cl_Disinf_StopDisinf();
								cl_disinfstate = CL_DISINF_STATE_INTER;
								alarm_flag1 = 1;
								tempcount=0;
								break;
							}
					 
							else if (tempcount == CL_DISINF_ALRM_MAX - 1)
							{
								if(!Cl_DisinfAlarmTable[tempcount].IsActive)
								{
									sv_cntrl_resetredalarm();
									sv_cntrl_nobuzzer();
									alarm_flag = 1;
									if (alarm_flag1 == 0)
									{
					 command = CON_TX_COMMAND_DISINF_RESUMED;
					 data = (Cl_Uint8Type)0;
					 Cl_disinfretcode = Cl_SendDatatoconsole(command,&data,0);
										cl_disinfstate = CL_DISINF_STATE_DISINF_OPENFILL;
									}
									//command = CON_TX_COMMAND_DISINF_RESUMED;
									//Cl_Disinf_resumeDisinf();
									//data = (Cl_Uint8Type)0;
									//Cl_disinfretcode = Cl_SendDatatoconsole(command,&data,0);
									//Cl_Disinf_UpdateTimeInfo();
								}
							}
						}
					break;
							
				}
			break;
			case CL_DISINF_STATE_DISINF_OPENFILL:
					
						switch ( cl_disinfevent)
						{

							case EVENT_STOP_DISINF:
							Cl_Disinf_StopDisinf();
							cl_disinfstate = CL_DISINF_STATE_IDLE;
							break;
							
							case EVENT_DISINF_TICK_50MS:
							break;
							case EVENT_DISINF_TICK_100MS:
							break;
							case EVENT_DISINF_TICK_500MS:
							break;
							case EVENT_DISINF_TICK_SECOND:
							sec_count++;
							if(sec_count > 3)
							{
								Cl_disinfOpenFillTimeOut = true;
							}
							//wait for 1 min openfill before intake;
							if(Cl_disinfOpenFillTimeOut)
							{
								if (flag_hotcitric == 1 || flag_citric == 1 )
								{
								Cl_disinfretcode =  sv_cntrl_deactivatepump(DCMOTOR1);
								Cl_disinfretcode =  sv_cntrl_deactivatepump(DCMOTOR2);
								//Cl_bc_controller(BC_EVENT_STOP);
								Cl_SendDatatoconsole(CON_TX_COMMAND_PRINTTEXT,"INT",3);
								sv_prop_start_citricdisinfect_intake();
								cl_disinfstate = CL_DISINF_STATE_DISINF_INTAKE;
								sec_count =0;
								}
								else
								{
								Cl_disinfretcode =  sv_cntrl_deactivatepump(DCMOTOR1);
								Cl_disinfretcode =  sv_cntrl_deactivatepump(DCMOTOR2);
								//Cl_bc_controller(BC_EVENT_STOP);
								Cl_SendDatatoconsole(CON_TX_COMMAND_PRINTTEXT,"INT",3);
								sv_prop_start_disinfect_intake();
								cl_disinfstate = CL_DISINF_STATE_DISINF_INTAKE;
								sec_count =0;
							}
								
							}

					
							
							break;
							case EVENT_DISINF_ALARM:
							Cl_disinfretcode = Cl_disinf_ProcessAlarms();
							break;
							default:break;
						}
			break;
			
			case CL_DISINF_STATE_DISINF_INTAKE:
						switch ( cl_disinfevent)
						{
							
							case EVENT_STOP_DISINF:
							Cl_Disinf_StopDisinf();
							cl_disinfstate = CL_DISINF_STATE_IDLE;
							break;
							
							
							case EVENT_DISINF_TICK_50MS:
							break;
							case EVENT_DISINF_TICK_100MS:
							break;
							case EVENT_DISINF_TICK_500MS:
							break;
							case EVENT_DISINF_TICK_SECOND:
						//	Cl_disinfsecondscounter++;
							Cl_Disinf_SendDisinfStateData();
							if(Cl_disinfsecondscounter == 60)
							{
						
					//			UpdateDisinfMinuteTick();
						
						
							}
							sec_count ++;
							if (flag_hotcitric == 1 )
							{
								if(sec_count > 90)
								{
									Cl_SendDatatoconsole(CON_TX_COMMAND_PRINTTEXT,"ove",3);
									sv_prop_stop_citricdisinfect_intake();
									Cl_disinfretcode =  sv_cntrl_deactivatepump(DCMOTOR2);
									Cl_disinfretcode =  sv_cntrl_deactivatepump(DCMOTOR1);
									Cl_bc_controller(BC_EVENT_STOP);
									
									command = CON_TX_COMMAND_DISINF_STAGE_COMPLETED;
									data = (Cl_Uint8Type)2;
									Cl_disinfretcode = Cl_SendDatatoconsole(command,&data,1);
									
									IntakeReadyFlag = false;
									
									cl_wait(20);

									command = CON_TX_COMMAND_DISINF_PAUSED;
									data = (Cl_Uint8Type)0;
						//			cl_disinfstate = CL_DISINF_STATE_DISINF_INTAKE_COMPLETED;
							
									Cl_disinfretcode = Cl_SendDatatoconsole(command,&data,0);
									cl_disinfstate = CL_DISINF_STATE_DISINF_INTAKE_COMPLETED;

								}
							}
							
							if (flag_citric == 1 )
							{
								if(sec_count > 90)
								{
									Cl_SendDatatoconsole(CON_TX_COMMAND_PRINTTEXT,"ove",3);
									sv_prop_stop_citricdisinfect_intake();
									Cl_disinfretcode =  sv_cntrl_deactivatepump(DCMOTOR2);
									Cl_disinfretcode =  sv_cntrl_deactivatepump(DCMOTOR1);
									Cl_bc_controller(BC_EVENT_STOP);
								
								IntakeReadyFlag = false;
									
									command = CON_TX_COMMAND_DISINF_STAGE_COMPLETED;
									data = (Cl_Uint8Type)2;
									Cl_disinfretcode = Cl_SendDatatoconsole(command,&data,1);
									
									cl_wait(20);

									command = CON_TX_COMMAND_DISINF_PAUSED;
									data = (Cl_Uint8Type)0;
									Cl_disinfretcode = Cl_SendDatatoconsole(command,&data,0);
									cl_disinfstate = CL_DISINF_STATE_DISINF_INTAKE_COMPLETED;
									//cl_disinfstate = CL_DISINF_STATE_DISINF_INTAKE_COMPLETED;
									
								

								}
							}
							
							else if(sec_count > 80)
							{
								Cl_SendDatatoconsole(CON_TX_COMMAND_PRINTTEXT,"ove",3);
								sv_prop_stop_disinfect_intake();
								sv_prop_stop_citricdisinfect_intake();
								Cl_disinfretcode =  sv_cntrl_deactivatepump(DCMOTOR2);
								Cl_disinfretcode =  sv_cntrl_deactivatepump(DCMOTOR1);
								Cl_bc_controller(BC_EVENT_STOP);
								//flag_hotrinse = 0;
								//Cl_AlarmActivateAlarms(ACID_IN,false);
								//Cl_AlarmActivateAlarms(BICARB_IN,false);
								command = CON_TX_COMMAND_DISINF_STAGE_COMPLETED;
								data = (Cl_Uint8Type)2;
								Cl_disinfretcode = Cl_SendDatatoconsole(command,&data,1);
								
								IntakeReadyFlag = false;
								
								cl_wait(20);

								command = CON_TX_COMMAND_DISINF_PAUSED;
								data = (Cl_Uint8Type)0;
							//	cl_disinfstate = CL_DISINF_STATE_DISINF_INTAKE_COMPLETED;
								
								Cl_disinfretcode = Cl_SendDatatoconsole(command,&data,0);
								cl_disinfstate = CL_DISINF_STATE_DISINF_INTAKE_COMPLETED;

							}
				
							break;
							case EVENT_DISINF_ALARM:
							Cl_disinfretcode = Cl_disinf_ProcessAlarms();
							break;
							default:break;
						}
			
			break;
			case CL_DISINF_STATE_DISINF_INTAKE_COMPLETED:
			
						switch (cl_disinfevent)
						{
							
							case EVENT_STOP_DISINF:
							Cl_Disinf_StopDisinf();
							cl_disinfstate = CL_DISINF_STATE_IDLE;
							break;
							
								case EVT_RESUME_DISINF_STAGE:
								
								 Cl_disinfretcode =  sv_cntrl_activatepump(DCMOTOR2);
								Cl_disinfretcode =  sv_cntrl_activatepump(DCMOTOR1);
								//Cl_disinfretcode =  sv_cntrl_activatepump(DCMOTOR2);
								Cl_disinfretcode = sv_cntrl_setpumpspeed(DCMOTOR2,1000);
								Cl_disinfretcode = sv_cntrl_setpumpspeed(DCMOTOR1,1000);
								openfill_flag = 1;
								Cl_Uint16Type potvalue = 0;
								potvalue = (850 * 1024)/10000;                                 // 600
								sv_cs_setpotvalue(potvalue);
								if (flag_hotcitric == 1)
 								{
 								hotrinse_flag = 1;
 								}
								
								Cl_bc_controller(BC_EVENT_START);
								Cl_AlarmActivateAlarms(TEMP1_LOW_THRESHOLD,false );     // manav02 added no ts1 alarm during loopback
								Cl_AlarmActivateAlarms(TEMP1_HIGH_THRESHOLD,false );

								sv_cntrl_enable_loopback();
						//		sv_cntrl_enable_bypass();
// 								if (flag_hotcitric == 1)
// 								{
// 									sv_cntrl_poweronheater();
// 									sv_cntrl_incheater(3500);
// 								}
								sv_cntrl_activate_valve(VALVE_ID2);
								sv_cntrl_activate_valve(VALVE_ID3);
							//	sv_prop_startmixing();     // manav03 commented
								sv_prop_startopenfill();   // manav02 added to run piston pumps during disinfect open fill
								sv_cntrl_activatepump(UFPUMP);
								sec_count =0;
								command = CON_TX_COMMAND_DISINF_RESUMED;
 									Cl_disinfretcode = Cl_SendDatatoconsole(command,&data,0);
									 Cl_disinfsecondscounter = 0;
 									cl_disinfstate = CL_DISINF_STATE_DISINF_RINSING;
							//	cl_disinfstate = CL_DISINF_STATE_INTER_1;
								 Cl_disinfretcode =  Cl_AlarmConfigureAlarmType(ACID_IN_OPEN,LOGIC_LOW,0,0,0);
								 Cl_disinfretcode =  Cl_AlarmConfigureAlarmType(BICARB_IN_OPEN,LOGIC_LOW,0,0,0);
								Cl_AlarmActivateAlarms(ACID_IN_OPEN,true);
								Cl_AlarmActivateAlarms(BICARB_IN_OPEN,true);
								/*command = CON_TX_COMMAND_DISINF_PAUSED;
								data = (Cl_Uint8Type)0;
								Cl_disinfretcode = Cl_SendDatatoconsole(command,&data,0);*/
								 //Send resumed
								/*if (flag_hotcitric == 1)
								{
									 command = CON_TX_COMMAND_DISINF_RESUMED;
									 data = (Cl_Uint8Type)4;
									 Cl_disinfretcode = Cl_SendDatatoconsole(command,&data,0);
								}
								else
								{
									command = CON_TX_COMMAND_DISINF_RESUMED;
									data = (Cl_Uint8Type)3;
									Cl_disinfretcode = Cl_SendDatatoconsole(command,&data,0);
								}*/
								

								break;
								case EVENT_DISINF_ALARM:
								Cl_disinfretcode = Cl_disinf_ProcessAlarms();
								break;
								default:break;
						}
			
			break;
			case CL_DISINF_STATE_INTER_1:
				switch (cl_disinfevent)
				{
					
					case EVENT_STOP_DISINF:
					Cl_Disinf_StopDisinf();
					cl_disinfstate = CL_DISINF_STATE_IDLE;
					break;
					
					
					case EVENT_DISINF_ALARM:
					Cl_disinfretcode =  Cl_disinf_ProcessAlarms();
					break;
					
					case EVENT_DISINF_CLR_ALARM:
					sv_cntrl_resetredalarm();
					sv_cntrl_nobuzzer();
					Cl_Disinf_resumeDisinf();
					if (alarm_flag == 1)
					{
								cl_disinfstate = CL_DISINF_STATE_DISINF_RINSING;
						alarm_flag = 0;
					}
					break;
								 
					case EVENT_DISINF_TICK_SECOND:
					
					 command = CON_TX_COMMAND_DISINF_RESUMED;
 									Cl_disinfretcode = Cl_SendDatatoconsole(command,&data,0);
 									cl_disinfstate = CL_DISINF_STATE_DISINF_RINSING;
//					for (tempcount = 0 ; tempcount < CL_DISINF_ALRM_MAX ; tempcount++)
//					{
						//Cl_SendDatatoconsole(CON_TX_COMMAND_PRINTTEXT,"ENTER",8);
// 						Cl_Alarm_GetAlarmStatus(Cl_DisinfAlarmTable[tempcount].CldisinfAlarmId,&Cl_DisinfAlarmTable[tempcount].IsActive);
// 					 
// 						if (Cl_DisinfAlarmTable[tempcount].IsActive)
// 						{
// 							sv_cntrl_setredalarm();
// 							sv_cntrl_buzzer();
// 							Cl_SendDatatoconsole(CON_TX_COMMAND_PRINTTEXT,"ENTER_1",8);
// 							Cl_Disinf_StopDisinf();
// 							cl_disinfstate = CL_DISINF_STATE_INTER_1;
// 							alarm_flag1 = 1;
// 							tempcount=0;
// 							break;
// 						}
// 						
// 						else if (tempcount == CL_DISINF_ALRM_MAX - 1)
// 						{
// 							if(!Cl_DisinfAlarmTable[tempcount].IsActive)
// 							{
// 					 
// 								sv_cntrl_resetredalarm();
// 								sv_cntrl_nobuzzer();
// 								alarm_flag = 1;
// 								if (alarm_flag1 == 0)
// 								{
// 								 command = CON_TX_COMMAND_DISINF_RESUMED;
// 									Cl_disinfretcode = Cl_SendDatatoconsole(command,&data,0);
// 									cl_disinfstate = CL_DISINF_STATE_DISINF_RINSING;
// 									
// 								}
// 							/*	if (flag_hotcitric == 1)
// 								{
// 									//command = CON_TX_COMMAND_DISINF_RESUMED;
// 									data = (Cl_Uint8Type)4;
// 									Cl_disinfretcode = Cl_SendDatatoconsole(command,&data,0);
// 								}
// 								else if (flag_na == 1)
// 								{
// 									//command = CON_TX_COMMAND_DISINF_RESUMED;
// 								 data = (Cl_Uint8Type)3;
// 								 Cl_disinfretcode = Cl_SendDatatoconsole(command,&data,0);
// 								}*/
// 								//Cl_Disinf_UpdateTimeInfo();
// 								
// 							}
// 						}
//					}
								break;
					
						}
			break;
			case CL_DISINF_STATE_DISINF_RINSING:
						switch ( cl_disinfevent)
						{
							
							case EVENT_STOP_DISINF:
							Cl_Disinf_StopDisinf();
							cl_disinfstate = CL_DISINF_STATE_IDLE;
							break;
							
							
							case EVENT_DISINF_TICK_50MS:
							break;
							case EVENT_DISINF_TICK_100MS:
							break;
							case EVENT_DISINF_TICK_500MS:
							
							
							
							
								if (avgtemp2_dummy > 75)
								{
				//					
								}
							break;
							case EVENT_DISINF_TICK_SECOND:
							Cl_disinf_ResetAlertsforReassertion();
							Cl_Uint16Type temp;
							Cl_SysStat_GetSensor_Status_Query(SENSOR_TEMP2STATUS,&temp);
	
									{
		
											float ftemp,temp1;
										
											res_temp_lookuptable((402 *100* temp)/(2*32768));									// temperature from look up table in 4 digits
											temp1 = res_temp_value/100;									// Temp3 value in XX.yy format
											temp1 = temp1 - 3.1 + 1.4 + 1.2;										// sensor offset
											heatcutavgtmp2 =(heatcutavgtmp2*5 + temp1)/6;
									}
							
// 							if(heatcutavgtmp2 > 85){
// 								sv_cntrl_poweroffheater();
// 								
// 								SetHeaterState(CL_HEATER_STATE_OFF);
// 							}
							
							Cl_disinfsecondscounter++;
							Cl_Disinf_SendDisinfStateData();
							if(Cl_disinfsecondscounter == 60)
							{
								UpdateDisinfMinuteTick();	
							}
							if (Cl_disinfMinutescounter == 1)
							{
								//Cl_AlarmActivateAlarms(PS1_LOW_THRESHOLD,true);
								//Cl_AlarmActivateAlarms(PS1_HIGH_THRESHOLD,true);
								//Cl_AlarmActivateAlarms(PS2_LOW_THRESHOLD,true);
								//Cl_AlarmActivateAlarms(PS2_HIGH_THRESHOLD,true);
							}
							if(Cl_disinfMinutescounter > CL_DISINF_TIMEOUT )
							{
								sv_cntrl_setpumpspeed(DCMOTOR2,1000);
								Cl_Disinf_StopDisinf();
								sv_cntrl_deactivate_valve(VALVE_ID2);
								sv_cntrl_deactivate_valve(VALVE_ID3);
								sv_cntrl_poweroffheater();
								sv_prop_stopmixing();
								sv_cntrl_deactivatepump(UFPUMP);
								flag_hotrinse = 0;
								Cl_disinfretcode =  sv_cntrl_deactivatepump(DCMOTOR2);
								Cl_disinfretcode =  sv_cntrl_deactivatepump(DCMOTOR1);
								Cl_bc_controller(BC_EVENT_STOP);
								if (flag_hotcitric == 1)
								{
								command = CON_TX_COMMAND_DISINF_STAGE_COMPLETED;
								data = (Cl_Uint8Type)4;
								Cl_disinfretcode = Cl_SendDatatoconsole(command,&data,1);
								}
								else if (flag_citric == 1)
								{
									command = CON_TX_COMMAND_DISINF_STAGE_COMPLETED;
									data = (Cl_Uint8Type)5;
									Cl_disinfretcode = Cl_SendDatatoconsole(command,&data,1);
								}
								else if (flag_na == 1)
								{
									command = CON_TX_COMMAND_DISINF_STAGE_COMPLETED;
									data = (Cl_Uint8Type)3;
									Cl_disinfretcode = Cl_SendDatatoconsole(command,&data,1);
								}
								flag_hotcitric = 0;
								flag_citric = 0;
								flag_na = 0;
								openfill_flag = 0;
								
								

							//	command = CON_TX_COMMAND_DISINF_PAUSED;
							//	data = (Cl_Uint8Type)0;
							//	Cl_disinfretcode = Cl_SendDatatoconsole(command,&data,0);
							//	cl_disinfstate = CL_DISINF_STATE_DISINF_RINSING_COMPLETED;
							
	
							
							//	command = CON_TX_COMMAND_DISINF_RESUMED;
							//	data = (Cl_Uint8Type)0;
							//	Cl_disinfretcode = Cl_SendDatatoconsole(command,&data,0);
								 Cl_disinfretcode = Cl_mac_apprequesthandler(MACREQ_START_POST_DISINF_RINSE);
								 cl_disinfstate = CL_DISINF_STATE_SECOND_RO_RINSE;
					 
							}
							break;
							case EVENT_DISINF_TICK_MINUTE:
								//Cl_disinf_ResetAlertsforReassertion();
							break;
							case EVENT_DISINF_ALARM:
							
							Cl_disinfretcode =  Cl_disinf_ProcessAlarms();
							break;
							command = CON_TX_COMMAND_DISINF_PAUSED;
							data = (Cl_Uint8Type)0;
							Cl_disinfretcode = Cl_SendDatatoconsole(command,&data,0);
							break;
							default:break;
						}
						
				break;
				case CL_DISINF_STATE_DISINF_RINSING_COMPLETED:
						switch ( cl_disinfevent)
						{ 
							
							case EVENT_STOP_DISINF:
							Cl_Disinf_StopDisinf();
							cl_disinfstate = CL_DISINF_STATE_IDLE;
							break;
							
							
							case EVT_RESUME_DISINF_STAGE:
							// Cl_Disinf_StartDisinf();
							 Cl_disinfretcode = Cl_mac_apprequesthandler(MACREQ_START_POST_DISINF_RINSE);
							 
							command = CON_TX_COMMAND_DISINF_RESUMED;
							data = (Cl_Uint8Type)0;
							Cl_disinfretcode = Cl_SendDatatoconsole(command,&data,0);
							
							 cl_disinfstate = CL_DISINF_STATE_SECOND_RO_RINSE;
					 
							 //Send resumed
					 

							break;
							default:break;
			
							
						}
			
				break;		
				case CL_DISINF_STATE_SECOND_RO_RINSE:
								switch ( cl_disinfevent)
								{
									
									case EVENT_STOP_DISINF:
									Cl_Disinf_StopDisinf();
									cl_disinfstate = CL_DISINF_STATE_IDLE;
									break;
									
									
									case EVENT_POST_DISINFECT_RINSE_COMPLETED:
									
									//Send Complete
									 command = CON_TX_COMMAND_DISINF_COMPLETED;
									 data = (Cl_Uint8Type)0;
									 Cl_disinfretcode = Cl_SendDatatoconsole(command,&data,0);
									  Cl_disinfretcode = Cl_mac_apprequesthandler(MACREQ_DISINF_COMPLETED);
									 cl_disinfstate = CL_DISINF_STATE_POST_DISINF_STANDBY;
									 
									break;
									case EVENT_DISINF_TICK_SECOND:
									Cl_disinfsecondscounter++;
									Cl_Disinf_SendDisinfStateData();
									if(Cl_disinfsecondscounter == 60)
									{
									//	UpdateDisinfMinuteTick();
									}
									break;
									case EVENT_DISINF_ALARM:
									Cl_disinfretcode = Cl_disinf_ProcessAlarms();
									break;
									default:break;
								}
				
				
				break;
				case CL_DISINF_STATE_POST_DISINF_STANDBY:
					switch ( cl_disinfevent)
					{
						case EVENT_DISINF_START_RINSE:
							Cl_disinfretcode = Cl_mac_apprequesthandler(MACREQ_START_POST_DISINF_RINSE);
						break;
						case EVENT_START_DISINF:
							Cl_disinfretcode = Cl_mac_apprequesthandler(MACREQ_START_PRE_DISINF_RINSE);
							if(Cl_ConsoleRxMsg.msgready == true)
							{
								cl_Datastreamtype cl_temp  ;
								cl_temp.bytearray[0] = Cl_ConsoleRxMsg.data.byte[0];
								if (cl_temp.bytearray[0] == 4)
								{
										flag_hotcitric = 1;
								}
								else if (cl_temp.bytearray[0] == 3)
								{
										flag_na = 1;
								}
								else if (cl_temp.bytearray[0] == 5)
								{
									flag_citric = 1;
								}
				
							}
							command = CON_TX_COMMAND_DISINF_STARTED;
							data = (Cl_Uint8Type)COMMAND_RESULT_SUCCESS;
							Cl_disinfretcode = Cl_SendDatatoconsole(command,&data,1);			
							cl_disinfstate = CL_DISINF_STATE_FIRST_RO_RINSE;
						break;
					}
				break;
			break;
			case CL_DISINF_STATE_CRITICAL_ALARM:
			switch (cl_disinfevent)
			{
				case EVENT_START_DISINF:
				Cl_disinfretcode = Cl_mac_apprequesthandler(MACREQ_START_PRE_DISINF_RINSE);
				if(Cl_ConsoleRxMsg.msgready == true)
				{
					cl_Datastreamtype cl_temp  ;
					cl_temp.bytearray[0] = Cl_ConsoleRxMsg.data.byte[0];
					if (cl_temp.bytearray[0] == 4)
					{
						flag_hotcitric = 1;
					}
					else if (cl_temp.bytearray[0] == 3)
					{
						flag_na = 1;
					}
					else if (cl_temp.bytearray[0] == 5)
					{
						flag_citric = 1;
					}
				}
				command = CON_TX_COMMAND_DISINF_STARTED;
				data = (Cl_Uint8Type)COMMAND_RESULT_SUCCESS;
				Cl_disinfretcode = Cl_SendDatatoconsole(command,&data,1);
				cl_disinfstate = CL_DISINF_STATE_FIRST_RO_RINSE;
				break;
				case EVENT_DISINF_ALARM:
				Cl_disinfretcode = Cl_disinf_ProcessAlarms();
				break; 
				case EVENT_DISINF_TICK_SECOND:
				
				for (tempcount = 0 ; tempcount < CL_DISINF_ALRM_MAX ; tempcount++)
				{
					//Cl_SendDatatoconsole(CON_TX_COMMAND_PRINTTEXT,"ENTER",8);
					Cl_Alarm_GetAlarmStatus(Cl_DisinfAlarmTable[tempcount].CldisinfAlarmId,&Cl_DisinfAlarmTable[tempcount].IsActive);
					
					if (Cl_DisinfAlarmTable[tempcount].IsActive)
					{
						Cl_SendDatatoconsole(CON_TX_COMMAND_PRINTTEXT,"ENTER_1",8);
						Cl_Disinf_StopDisinf();
						
							
						
						cl_disinfstate = CL_DISINF_STATE_CRITICAL_ALARM;
						//alarm_flag = 1;
						tempcount=0;
						break;
					}
					
					else if (tempcount == CL_DISINF_ALRM_MAX - 1)
					{
						if(!Cl_DisinfAlarmTable[tempcount].IsActive)
						{
							sv_cntrl_resetredalarm();
							sv_cntrl_resetyellowalarm();
							sv_cntrl_nobuzzer();
							//Cl_Disinf_resumeDisinf();
							Cl_SendDatatoconsole(CON_TX_COMMAND_PRINTTEXT,"SAFE",8);
							Cl_disinf_ResetAlertsforReassertion();
						}
					}
				}
				break;
				
				
				case EVENT_DISINF_CLR_ALARM:
					sv_cntrl_resetredalarm();
					sv_cntrl_resetyellowalarm();
					sv_cntrl_nobuzzer();
					Cl_Disinf_resumeDisinf();
					cl_disinfstate = CL_DISINF_STATE_DISINF_RINSING;
				break;
				case EVENT_DISINF_TICK_MINUTE:
				Cl_disinfretcode = Cl_disinf_ResetAlertsforReassertion();
				break;
				
			}
			break;
			default:break;
			
		}
		

	return CL_OK;
}
 
  /*
 * \brief  cl_disinf_translatemacevent  - Translate   disinfection related MAC events to disinf module events 
 
 * \param  Cl_MacdisinfEvt, *cl_disinf_event
 * \ 
 * \return CL_OK on success. 
 * \Globals used :
 */   

Cl_ReturnCodeType  cl_disinf_translatemacevent(Cl_Mac_EventsType Cl_MacdisinfEvt,Cl_disinf_Eevents* cl_disinf_event)
{
	switch(Cl_MacdisinfEvt)
	{

		
		case  EVT_CONSOLE_COMMAND_DISINF_START:
		*cl_disinf_event= EVENT_START_DISINF;
		break;
		
		case EVT_CONSOLE_COMMAND_CLR_ALARM:
		*cl_disinf_event= EVENT_DISINF_CLR_ALARM;
		break;
		case EVT_CONSOLE_RESUME_DISINF_STAGE:
		*cl_disinf_event= EVT_RESUME_DISINF_STAGE;
		break;
		case EVT_CONSOLE_COMMAND_STOP_DISINF:
		*cl_disinf_event= EVENT_STOP_DISINF;
		break;

		case MACREQ_PRE_DISINFECT_RINSE_COMPLETED:
		*cl_disinf_event= EVENT_PRE_DISINFECT_RINSE_COMPLETED;
		break;


		case MACREQ_POST_DISINFECT_RINSE_COMPLETED:
		*cl_disinf_event = EVENT_POST_DISINFECT_RINSE_COMPLETED ;
		break;
		case MACREQ_DISINF_COMPLETED:
		*cl_disinf_event = EVENT_MAC_DISINF_COMPLETED;
		break;
		case MACREQ_BC_OPENFILL_COMPLETED:
		*cl_disinf_event = EVENT_DISINF_BC_OPENFILL_COMPLETED;
		break;

		
		case EVT_TICK_50M:
		*cl_disinf_event =   EVENT_DISINF_TICK_50MS;
		break;
		break;
		case EVT_TICK_100M:
		*cl_disinf_event =   EVENT_DISINF_TICK_100MS;
		break;
		case EVT_TICK_500M:
		*cl_disinf_event =   EVENT_DISINF_TICK_500MS;
		break;
		case EVT_TICK_SEC:
		*cl_disinf_event =   EVENT_DISINF_TICK_SECOND;
		break;
		case EVT_TICK_MIN:
		*cl_disinf_event =   EVENT_DISINF_TICK_MINUTE;
		break;
		case EVT_TICK_HOUR:
		*cl_disinf_event =   EVENT_DISINF_TICK_HOUR;
		break;
		case EVT_ALARM_TRIGGERED:
		*cl_disinf_event =  EVENT_DISINF_ALARM;
		break;
		case  EVT_ALERT_TRIGGERED:
		*cl_disinf_event =  EVENT_DISINF_ALERT;
		break;
		
		case EVT_CONSOLE_COMMAND_SET_DATA:
		*cl_disinf_event = EVT_DISINF_SET_DATA;
		break;
		
		case  EVT_CONSOLE_COMMAND_RINSE_START:
		*cl_disinf_event= EVENT_DISINF_START_RINSE;
		break;
		
		case EVT_CONSOLE_COMMAND_GET_DATA:
		*cl_disinf_event =  EVT_DISINF_GET_DATA;
		break;
		case EVT_CONSOLE_COMMAND_START_PRIME:
		*cl_disinf_event =  EVT_DISINF_START_PRIME;
		break;
		case EVT_CONSOLE_COMMAND_STOP_PRIME:
		*cl_disinf_event =  EVT_DISINF_STOP_PRIME;
		break;
		case EVT_CONSOLE_COMMAND_RCIRC_START:
		*cl_disinf_event = EVT_DISINF_RCIRC_START;
		break;
		case EVT_CONSOLE_COMMAND_RCIRC_STOP:
		*cl_disinf_event = EVT_DISINF_RCIRC_STOP;
		break;
		case EVT_CONSOLE_COMMAND_SET_BLDPMP_ON:
		*cl_disinf_event = EVT_DISINF_COMMAND_SET_BLDPMP_ON;
		break;
		case EVT_CONSOLE_COMMAND_SET_BLDPMP_OFF:
		*cl_disinf_event = EVT_DISINF_COMMAND_SET_BLDPMP_OFF;
		break;
		break;
		case EVT_CONSOLE_COMMAND_SET_BLDPUMPRATE:
		*cl_disinf_event = EVT_DISINF_COMMAND_SET_BLDPUMPRATE;
		break;
		
		default:
		break;
	}
	return CL_OK;
}

  /*
 * \brief  Cl_Disinf_StartDisinf  - Activate actuators and disinfection FSM
 
 * \param  
 * \ 
 * \return CL_OK on success. 
 * \Globals used :
 */   

Cl_ReturnCodeType Cl_Disinf_StartDisinf(void)
{
	Cl_ReturnCodeType Cl_disinfretcode = CL_OK;
	Cl_ConsoleTxCommandtype command = CON_TX_COMMAND_COMMAND_MAX;
	Cl_Uint8Type dataarray[4] = {0,0,0,0};
	Cl_Uint8Type data = 0;
	
	//Check for  rinse readiness and alarm status
	
	sv_cntrl_deactivate_valve(VALVE_ID1);
	sv_cs_setpotvalue(600);
	
	Cl_disinfretcode =  sv_cntrl_activatepump(DCMOTOR1);
	Cl_disinfretcode = sv_cntrl_setpumpspeed(DCMOTOR1,600);
	
	Cl_disinfretcode = Cl_bc_controller(BC_EVENT_START);
	
	if(Cl_disinfretcode == CL_OK)
	{
		Cl_disinfOpenFillTimeOut = false;
		Cl_disinfretcode = Cl_Disinf_UpdateTimeInfo();
		Cl_disinfretcode =  cl_flow_switchcontroller(FLOW_SWITCH_CNTRLER_EVENT_START);
		Cl_disinf_ResetAlertsforReassertion();
		Cl_disinfretcode =  Cl_AlarmConfigureAlarmType(ACID_IN_OPEN,LOGIC_LOW,0,0,0);
		Cl_disinfretcode =  Cl_AlarmConfigureAlarmType(BICARB_IN_OPEN,LOGIC_LOW,0,0,0);
		Cl_disinfretcode =  Cl_AlarmConfigureAlarmType(HOLDER1STATUS_OPEN,LOGIC_HIGH,0,0,0);
		Cl_disinfretcode =  Cl_AlarmConfigureAlarmType(HOLDER2STATUS_OPEN,LOGIC_HIGH,0,0,0);
		Cl_disinfretcode =  Cl_AlarmConfigureAlarmType(LEVELSWITCH_OFF_TO_ON,LOGIC_LOW,0,0,0);
		Cl_disinfretcode =  Cl_AlarmConfigureAlarmType(LEVELSWITCH_ON_TO_OFF,LOGIC_HIGH,0,0,0);
		
		/*Cl_disinfretcode =  Cl_AlarmActivateAlarms(ACID_IN,true );
		Cl_disinfretcode =  Cl_AlarmActivateAlarms(BICARB_IN,true );
		Cl_disinfretcode =  Cl_AlarmActivateAlarms(HOLDER1STATUS_OPEN,true);
		Cl_disinfretcode =  Cl_AlarmActivateAlarms(HOLDER2STATUS_OPEN,true);*/
		Cl_disinfretcode =  Cl_AlarmActivateAlarms(LEVELSWITCH_OFF_TO_ON,true);
		Cl_disinfretcode =  Cl_AlarmActivateAlarms(LEVELSWITCH_ON_TO_OFF,true);
		/*Cl_disinfretcode =  Cl_AlarmActivateAlarms(COND_STATUS_HIGH,true);
		Cl_disinfretcode =  Cl_AlarmActivateAlarms(TEMP3_HIGH_THRESHOLD,true);*/
		
		Cl_disinfretcode = Cl_disinf_ResetAlertsforReassertion();	
	}
return CL_OK;
}

Cl_ReturnCodeType Cl_Disinf_StopDisinf(void)
{
	Cl_Uint8Type data;
	//Cl_ReturnCodeType Cl_disinf_retval = CL_OK;
	sv_prop_stopmixing();
 	sv_cntrl_deactivatepump(UFPUMP);              // manav02 added
	flag_hotrinse = 0;							// manav02 added	
	sv_cntrl_deactivatepump(DCMOTOR2);			// manav02 added	
	sv_cntrl_deactivatepump(DCMOTOR1);			// manav02 added
	Cl_bc_controller(BC_EVENT_STOP);			// manav02 added
	Cl_AlarmActivateAlarms(TEMP1_LOW_THRESHOLD,true );     // manav02 added ts1 alarm during loopback off
	Cl_AlarmActivateAlarms(TEMP1_HIGH_THRESHOLD,true );

	sv_cntrl_disable_loopback();
	sv_cntrl_poweroffheater();
	cl_disinfstate = CL_DISINF_STATE_CRITICAL_ALARM;
	Cl_disinfsecondscounter_1 = Cl_disinfsecondscounter;
	Cl_disinfshourscounter_1 = Cl_disinfshourscounter;
	Cl_disinfMinutescounter_1 = Cl_disinfMinutescounter;
	Cl_disinfTotalMinutescounter_1 = Cl_disinfTotalMinutescounter;
	Cl_disinfsTotalhourscounter_1 =Cl_disinfsTotalhourscounter;
	Cl_disinfsecondscounter=0;
	Cl_disinfshourscounter = 0;
	Cl_disinfMinutescounter = 0;
	Cl_disinfTotalMinutescounter = 0;
	Cl_disinfsTotalhourscounter = 0;
	hotrinse_flag = 0;
	Cl_SendDatatoconsole(CON_TX_COMMAND_DISINF_STOPPED,&data,0);
	//Cl_disinf_retval = Cl_SendDatatoconsole(CON_TX_COMMAND_DISINF_PAUSED,&data,0);
	return CL_OK;
}

Cl_ReturnCodeType Cl_Disinf_resumeDisinf(void)
{
	Cl_Uint8Type data;
	Cl_ReturnCodeType Cl_disinf_retval = CL_OK;
	sv_cntrl_enable_loopback();
	Cl_AlarmActivateAlarms(TEMP1_LOW_THRESHOLD,false );     // manav02 added no ts1 alarm during loopback
	Cl_AlarmActivateAlarms(TEMP1_HIGH_THRESHOLD,false );

	cl_disinfstate = CL_DISINF_STATE_DISINF_RINSING;
	if (flag_hotcitric == 1)
	{
		sv_cntrl_poweronheater();
	}
	Cl_disinf_retval = Cl_SendDatatoconsole(CON_TX_COMMAND_DISINF_RESUMED,&data,0);
	Cl_disinfsecondscounter = Cl_disinfsecondscounter_1;
	Cl_disinfshourscounter = Cl_disinfshourscounter_1;
	Cl_disinfMinutescounter = Cl_disinfMinutescounter_1;
	Cl_disinfTotalMinutescounter = Cl_disinfTotalMinutescounter_1;
	Cl_disinfsTotalhourscounter = Cl_disinfsTotalhourscounter_1;
	//Cl_Disinf_UpdateTimeInfo();
	
	return CL_OK;
}

  /*
 * \brief  Cl_Disinf_UpdateTimeInfo  - Update Time and Date to Console
 
 * \param  
 * \ 
 * \return CL_OK on success. 
 * \Globals used :
 */ 
Cl_ReturnCodeType Cl_Disinf_UpdateTimeInfo(void)
{
	
	Cl_ReturnCodeType Cl_disinfretcode = CL_ERROR;
	Cl_ConsoleTxCommandtype command = CON_TX_COMMAND_COMMAND_MAX;
	Cl_Uint8Type data[7] = {0,0,0,0,0 ,0,0};
	
	command = CON_TX_COMMAND_REM_TIME;

	
	data[0]= (Cl_Uint8Type)DISINF_STATE;
	data[1]= (Cl_Uint8Type)Cl_disinfTotalMinutescounter;
	
	Cl_disinfretcode = Cl_SendDatatoconsole(command,&data,2);
	
	return CL_OK;
	
}
Cl_ReturnCodeType Cl_Disinf_UpdateAlarmTable(CldisinfAlarmIdType* CldisinfAlarmId )
{
	Cl_ReturnCodeType 	Cl_disinfretcode = CL_OK;
	Cl_Uint8Type tempcount = 0;
	Cl_Uint8Type data[3] ={0, 0} ,*data1 = NULL;
	Cl_NewAlarmIdType cl_alarmId;
	
	Cl_ConsoleTxCommandtype command = CON_TX_COMMAND_COMMAND_MAX;
	
	
	for (tempcount = 0 ; tempcount < CL_DISINF_ALRM_MAX ; tempcount++)
	{
		
		Cl_Alarm_GetAlarmStatus(Cl_DisinfAlarmTable[tempcount].CldisinfAlarmId,&Cl_DisinfAlarmTable[tempcount].IsActive);

	
			if (Cl_DisinfAlarmTable[tempcount].IsActive)
			{
				if(!Cl_DisinfAlarmTable[tempcount].IsRaised)
					{
	
						Cl_DisinfAlarmTable[tempcount].IsRaised = true;
	
						Cl_disinfretcode = CL_DisinfAlarmActon(Cl_DisinfAlarmTable[tempcount].CldisinfAlarmId);
	
					}
				/*	else
					{
						alarm_flag = 1;
					}*/
	
			}
			else
			{
				// alarm was present before , but not active now.
				if(Cl_DisinfAlarmTable[tempcount].IsRaised == true)
				{
					Cl_DisinfAlarmTable[tempcount].IsRaised = false;
					command = CON_TX_COMMAND_ALARM;
					data[0] = (Cl_Uint8Type)(Cl_DisinfAlarmTable[tempcount].CldisinfAlarmId);
					data[1] = (Cl_Uint8Type) CL_ALARM_CLEARED;
			//		Cl_rinseretcode = Cl_SendDatatoconsole(command,&data,2);
				}

			}
		//}

	}

return (Cl_disinfretcode );
}
  /*
 * \brief  Cl_disinf_ProcessAlarms  - Alarm handling during disinfection
 
 * \param  
 * \ 
 * \return CL_OK on success. 
 * \Globals used :
 */ 
Cl_ReturnCodeType  Cl_disinf_ProcessAlarms(void)
{
	Cl_ReturnCodeType 	Cl_disinfretcode = CL_OK;
	CldisinfAlarmIdType CldisinfAlarmId;
	Cl_ConsoleTxCommandtype command = CON_TX_COMMAND_COMMAND_MAX;
	Cl_NewAlarmIdType cl_disinfalarmid;
	
	Cl_disinfretcode = Cl_Disinf_UpdateAlarmTable(&CldisinfAlarmId);
	return (Cl_disinfretcode);
}

Cl_ReturnCodeType  CL_DisinfAlarmActon(Cl_NewAlarmIdType cl_disinfalarmid)
{
		Cl_ReturnCodeType 	Cl_disinfretcode = CL_OK;
			Cl_Uint8Type data[3] ={0, 0} ,*data1 = NULL;
			Cl_ConsoleTxCommandtype command = CON_TX_COMMAND_COMMAND_MAX;
			Cl_NewAlarmIdType NewAlarmId = _NO_ALARM;
		float temp;
		//Cl_Uint16Type var=0
							
		switch(cl_disinfalarmid)
		{
			case ACID_IN_OPEN:
			if ((cl_disinfstate == CL_DISINF_STATE_INTER))
			{
				sv_cntrl_setredalarm();
				sv_cntrl_buzzer();
				if (cl_disinfstate == CL_DISINF_STATE_INTER)
				{
					cl_disinfstate = CL_DISINF_STATE_INTER;
				}
			}
			else if ((cl_disinfstate == CL_DISINF_STATE_INTER_1))
			{
				sv_cntrl_setredalarm();
				sv_cntrl_buzzer();
				cl_disinfstate = CL_DISINF_STATE_INTER_1;
			}
			else
			{
				sv_cntrl_setredalarm();
				sv_cntrl_buzzer();
				Cl_Disinf_StopDisinf();
				cl_disinfstate = CL_DISINF_STATE_CRITICAL_ALARM;
			}
			
			break;
			
			case BICARB_IN_OPEN:
			sv_cntrl_setredalarm();
			sv_cntrl_buzzer();
			Cl_Disinf_StopDisinf();
			cl_disinfstate = CL_DISINF_STATE_CRITICAL_ALARM;
			break;
			
			case HOLDER1STATUS_OPEN:
				sv_cntrl_setredalarm();
				sv_cntrl_buzzer();
				Cl_Disinf_StopDisinf();
				cl_disinfstate = CL_DISINF_STATE_CRITICAL_ALARM;
			break;
			
			case HOLDER2STATUS_OPEN:
				sv_cntrl_setredalarm();
				sv_cntrl_buzzer();
				Cl_Disinf_StopDisinf();
				cl_disinfstate = CL_DISINF_STATE_CRITICAL_ALARM;
			break;
			
			case TEMP3_HIGH_THRESHOLD:
					sv_cntrl_setyellowalarm();
					sv_cntrl_buzzer();
					Cl_Disinf_StopDisinf();
					cl_disinfstate = CL_DISINF_STATE_CRITICAL_ALARM;
			break;
			
			case TEMP3_LOW_THRESHOLD:
			sv_cntrl_setyellowalarm();
			sv_cntrl_buzzer();
			Cl_Disinf_StopDisinf();
			cl_disinfstate = CL_DISINF_STATE_CRITICAL_ALARM;
			break;
			
			case PS1_HIGH_THRESHOLD:
			case PS1_LOW_THRESHOLD:
				sv_cntrl_setyellowalarm();
				sv_cntrl_buzzer();
				Cl_Disinf_StopDisinf();
				cl_disinfstate = CL_DISINF_STATE_CRITICAL_ALARM;
			break;
			case	POWER_FAILURE:            // manav02 added
			case	PS2_HIGH_THRESHOLD:
			case    PS2_LOW_THRESHOLD:
				sv_cntrl_setredalarm();
				sv_cntrl_buzzer();
				Cl_Disinf_StopDisinf();
				cl_disinfstate = CL_DISINF_STATE_CRITICAL_ALARM;
			break;
			
			case	PS3_HIGH_THRESHOLD:
			case    PS3_LOW_THRESHOLD:
				sv_cntrl_setyellowalarm();
				sv_cntrl_buzzer();
				Cl_Disinf_StopDisinf();
				cl_disinfstate = CL_DISINF_STATE_CRITICAL_ALARM;
			break;
			
			case COND_STATUS_HIGH:
				sv_cntrl_setyellowalarm();
				sv_cntrl_buzzer();
				Cl_Disinf_StopDisinf();
				cl_disinfstate = CL_DISINF_STATE_CRITICAL_ALARM;
			break;
			
			default:
			break;

		}
		 if(cl_disinfalarmid != _NO_ALARM)
		 {
 
				  data[0] = (Cl_Uint8Type)cl_disinfalarmid;
				 data[1] = (Cl_Uint8Type) CL_ALARM_TRIGGERED;
				 data[2] = (Cl_Uint8Type)CRITICAL;
				 command = CON_TX_COMMAND_ALARM;
			 			
			Cl_disinfretcode = Cl_SendDatatoconsole(command,&data,3);
			cl_wait(10);
		 }
	return CL_OK;		
}
Cl_ReturnCodeType Cl_disinf_ResetAlertsforReassertion(void )
{
	Cl_ReturnCodeType 	Cl_disinfretcode = CL_OK;
	CldisinfAlarmIdType CldisinfAlarmId;
	Cl_Uint8Type data;
	Cl_ConsoleTxCommandtype command = CON_TX_COMMAND_COMMAND_MAX;
	Cl_NewAlarmIdType cl_disinfalarmid;
	
	
	Cl_DisinfAlarmTable[CL_DISINF_ALARM_ACIDIN].IsRaised = false;
	Cl_DisinfAlarmTable[CL_DISINF_ALARM_BICARBIN].IsRaised = false;
	Cl_AlarmResetAlarm(ACID_IN_OPEN);
	Cl_AlarmResetAlarm(BICARB_IN_OPEN);
	Cl_DisinfAlarmTable[CL_DISINF_ALARM_COND_STATUS_HIGH].IsRaised = false;
	Cl_AlarmResetAlarm(COND_STATUS_HIGH);
	Cl_DisinfAlarmTable[CL_DISINF_ALARM_PS1_HIGH_THRESHOLD].IsRaised = false;
	Cl_DisinfAlarmTable[CL_DISINF_ALARM_PS1_LOW_THRESHOLD].IsRaised = false;
	Cl_AlarmResetAlarm(PS1_HIGH_THRESHOLD);
	Cl_AlarmResetAlarm(PS1_LOW_THRESHOLD);
	Cl_DisinfAlarmTable[CL_DISINF_ALARM_PS2_HIGH_THRESHOLD].IsRaised = false;
	Cl_DisinfAlarmTable[CL_DISINF_ALARM_PS2_LOW_THRESHOLD].IsRaised = false;
	Cl_AlarmResetAlarm(PS2_HIGH_THRESHOLD);
	Cl_AlarmResetAlarm(PS2_LOW_THRESHOLD);
	Cl_DisinfAlarmTable[CL_DISINF_ALARM_PS3_HIGH_THRESHOLD].IsRaised = false;
	Cl_DisinfAlarmTable[CL_DISINF_ALARM_PS3_LOW_THRESHOLD].IsRaised = false;
	Cl_AlarmResetAlarm(PS3_LOW_THRESHOLD);
	Cl_AlarmResetAlarm(PS3_HIGH_THRESHOLD);
	Cl_DisinfAlarmTable[CL_DISINF_ALARM_TEMP3_HIGH_THRESHOLD].IsRaised = false;
	Cl_AlarmResetAlarm(TEMP2_HIGH_THRESHOLD);
	Cl_DisinfAlarmTable[CL_DISINF_ALARM_HOLDER1STATUS_OPEN].IsRaised = false;
	Cl_AlarmResetAlarm(HOLDER1STATUS_OPEN);
	Cl_AlarmResetAlarm(HOLDER1STATUS_CLOSED);
	Cl_DisinfAlarmTable[CL_DISINF_ALARM_HOLDER2STATUS_OPEN].IsRaised = false;
	Cl_AlarmResetAlarm(HOLDER2STATUS_OPEN);
	Cl_AlarmResetAlarm(HOLDER2STATUS_CLOSED);
	
	
	return (Cl_disinfretcode);
}

  /*
 * \brief  Cl_Disinf_SendDisinfStateData  - Send disinfection state data to console
 
 * \param  
 * \ 
 * \return CL_OK on success. 
 * \Globals used :
 */ 
Cl_ReturnCodeType Cl_Disinf_SendDisinfStateData(void)
{
	static float cond = 0;
	Cl_ReturnCodeType  Cl_disinfretcode = CL_OK;
	cl_DlsInfDatatype data;
	Cl_Uint8Type count=0;
	Cl_Uint16Type ts2,temp,temp1 ,temp2;
	Cl_Uint8Type dataarray[12] =  {0,0,0,0,0,0,0,0,0,0,0,0};
	static Cl_Uint32Type timecount = 0;
	static float avgtmp3 = 0;
	
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
	
	dataarray[0] = DISINF_STATE;
	count++;

	
	Cl_SysStat_GetSensor_Status_Query(SENSOR_TEMP3STATUS,&temp);
	
	{
		
		float ftemp,temp1;
		data.word = 0;
// 						// resistance of PT100
// 			res_temp_lookuptable((402 *100* temp)/(2*32768));									// temperature from look up table in 4 digits
// 			temp1 = res_temp_value/100;									// Temp3 value in XX.yy format
// //			temp1 = temp1 - 3.1 + 1.4;										// sensor offset
// //			temp1 = temp1 - 3.1 -2 ;										// sensor offset
// //			avgtmp3 =(avgtmp3*5 + temp1)/6;
// 			avgtmp3 = (res_temp_value- 0.4)/10;
			temp1 = (float)temp/100;
			avgtmp3 = temp1;
	}
	
	Cl_SysStat_GetSensor_Status_Query(SENSOR_COND_STATUS,&temp);
	{
		
		float cond_comp=0;
		float temp2=0;
		temp2 = temp * 0.8036;
		if (temp2 > 600)
					{
			calibration_cond(temp2);
			cond =(cond*5 + cond_final_cs3)/6;
			cond_comp= cond/(1+(avgtmp3-25.0)*0.020);
			data.Twobyte = (cond_comp/10);
			dataarray[count++] = data.bytearray[0];
			dataarray[count++] = data.bytearray[1];
			dataarray[count++] = data.bytearray[2];
			dataarray[count++] = data.bytearray[3];
					}
		else
					{
			data.Twobyte = 0;
			dataarray[count++] = data.bytearray[0];
			dataarray[count++] = data.bytearray[1];
			dataarray[count++] = data.bytearray[2];
			dataarray[count++] = data.bytearray[3];
						}
						
		
					}
					{
		data.Twobyte = (uint16_t)(avgtmp3 * 10);
		dataarray[count++] = data.bytearray[0];
		dataarray[count++] = data.bytearray[1];
		dataarray[count++] = data.bytearray[2];
		dataarray[count++] = data.bytearray[3];
	}
	// manav02 added
	Cl_SysStat_GetSensor_Status_Query(SENSOR_TEMP2STATUS,&temp2);
	{
		
		data.Twobyte = (uint16_t)temp2/10 ;
		dataarray[count++] = data.bytearray[0];
		dataarray[count++] = data.bytearray[1];
		dataarray[count++] = data.bytearray[2];
		dataarray[count++] = data.bytearray[3];
	}
	

	
	Cl_disinfretcode = Cl_SendDatatoconsole(CON_TX_COMMAND_CL_SYS_STATE_DATA,&dataarray,count);
	
	
	}
	
	  /*
 * \brief  UpdateDisinfMinuteTick  - Minute tick update to console
 
 * \param  
 * \ 
 * \return CL_OK on success. 
 * \Globals used :
 */ 
	  
	Cl_ReturnCodeType UpdateDisinfMinuteTick(void)
	{
		Cl_ReturnCodeType Cl_disinfretcode = CL_OK;
		Cl_disinfsecondscounter = 0;
		Cl_disinfMinutescounter++;
		Cl_disinfTotalMinutescounter++;
		Cl_disinfretcode = Cl_Disinf_UpdateTimeInfo();
		return Cl_disinfretcode;
	}
	
	
	#if 0

 Cl_ReturnCodeType Cl_Disinf_UpdateTimeInfo(void)
 {
	 
	 Cl_ReturnCodeType Cl_disinfretcode = CL_ERROR;
	 Cl_ConsoleTxCommandtype command = CON_TX_COMMAND_COMMAND_MAX;
	 Cl_Uint8Type data[7] = {0,0,0,0,0 ,0,0};
	 
	 command = CON_TX_COMMAND_REM_TIME;

	 
	 data[0]= (Cl_Uint8Type)DISINF_STATE;
	 data[1]= (Cl_Uint8Type)Cl_disinfTotalMinutescounter;

	 
	 Cl_disinfretcode = Cl_SendDatatoconsole(command,&data,2);
	 
	 
	 //	Cl_Rinse_SendRinseStateData();                                                          commented on 28.05.2014
	 return CL_OK;
	 
 }


#endif