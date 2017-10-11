/*
 * Cl_balancing_chamber_controller.c
 *
 * Created: 11/2/2014 4:13:35 PM
 *  Author: user
 */ 
#include "cl_bc_contrl_include.h"
#include "cl_bc_cntril_proto.h"
#include "sv_interface_fun.h"


extern Cl_ReturnCodeType cl_wait(Cl_Uint32Type );
extern Cl_Uint8Type flag_hotcitric,flag_na,openfill_flag,hotrinse_flag,flag_citric;
static Cl_BC_States bc_state = CL_BC_STATE_IDLE;
static Cl_BC_States bc_laststate = CL_BC_STATE_IDLE;
static Cl_BoolType bc_wait = false;
static int16_t wait_counter = 0;
static int16_t openfillcounter = 0;
int16_t disinfopenfillcounter = 0;
Cl_Uint16Type potvalue = 0;
volatile Cl_Uint16Type switch_time1=0,switch_time2=0;
extern Cl_BoolType current_sense ;
extern Cl_Uint8Type bc_flag;
extern Cl_Uint32Type flow_rate;
int switchb01alrcnt = 0;
int switchb02alrcnt = 0;
int CSeventcount = 0;
uint16_t wait_4_propo_off =0;
//Cl_Uint16Type potvalue = 0;
sv_flowpathtype cl_flowpathtable[CL_BC_STATE_MAXSTATE] =
{
	FLOW_PATH_NULL,		//CL_BC_STATE_IDLE,
	FLOW_PATH_NULL,		//CL_BC_STATE_NC,
	BC_FLUSH_OPEN,		//CL_BC_STATE_OPENFILL_TRANSITION,
	BC_FLUSH_OPEN,		//CL_BC_STATE_OPENFILL,
	V15_FLUSH,			//CL_BC_STATE_V15OPEN_TRANSITION,
	V15_FLUSH,			//CL_BC_STATE_V15OPEN,
	V2_FLUSH,			//CL_BC_STATE_V2OPEN_TRANSITION,
	V2_FLUSH,			//CL_BC_STATE_V2OPEN,
	BC_FLUSH_1_V15,		//CL_BC_STATE_BO1_V4_TRANSITION,
	BC_FLUSH_1_V15,		//CL_BC_STATE_BO1_V4,
	BC_FLUSH_2_V15,		//CL_BC_STATE_BO2_V4_TRANSITION,
	BC_FLUSH_2_V15,		//CL_BC_STATE_BO2_V4,
	BC_FLUSH_1_V13_14,	//CL_BC_STATE_BO1_V13V14_TRANSITION,
	BC_FLUSH_1_V13_14,	//CL_BC_STATE_BO1_V13V14,
	BC_FLUSH_2_V13_14,	//CL_BC_STATE_BO2_V13V14_TRANSITION,
	BC_FLUSH_2_V13_14,  //CL_BC_STATE_BO2_V13V14,

};

  /*
 * \brief  Cl_stop_bc_controller  - STOP the BC controller module
 * \param  
 * \ 
 * \return CL_OK on success. 
 * \Globals used : 
 */

Cl_ReturnCodeType  Cl_stop_bc_controller(void)
{
	
	return CL_OK;
}
 /*
 * \brief  Cl_bc_controller  -  BC controller  event handler
 * \param  
 * \ 
 * \return CL_OK on success. 
 * \Globals used : bc_state
 */
 
 

Cl_ReturnCodeType  Cl_bc_controller(Cl_BC_EventType cl_bc_event)
{
	Cl_ReturnCodeType cl_bc_returncode = CL_OK;
	static int16_t fill_time = 0;
	static cl_Datastreamtype cl_tdata,cl_tdata_temp;
	
	switch(cl_bc_event)
	{
		case	BC_EVENT_START:
		
		
				cl_bc_returncode = sv_cntrl_setflowpath(BC_FLUSH_OPEN);
				sv_cntrl_setpumpspeed(DCMOTOR2,1100);
				sv_cntrl_setpumpspeed(DCMOTOR1,900);
				Cl_AlarmActivateAlarms(BC_ALARM,true);  // manav02 activated
			
				
				if(openfill_flag == 1){
					
					cl_bc_returncode = sv_cntrl_setflowpath(BC_FLUSH_OPEN_DIS);
					
					if(flag_hotcitric == 1){
					
					sv_cntrl_poweronheater();
					sv_cntrl_incheater(3500);
					SetHeaterState(CL_HEATER_STATE_ON);
					Cl_Heater_Event_Handler(CL_HEATER_EVENT_TARGET_TEMP , 75);	
					}
					
				}
					bc_laststate = bc_state;
					bc_state = CL_BC_STATE_OPENFILL_TRANSITION;
				
		
		break;
		case BC_EVENT_RESUME:
		if(bc_state == CL_BC_STATE_IDLE)
				{
				cl_bc_returncode = sv_cntrl_setflowpath(BC_FLUSH_OPEN);
				cl_bc_returncode = sv_cntrl_setflowpath(cl_flowpathtable[bc_laststate]);
				//bc_laststate = bc_state;
				bc_state = bc_laststate;                                                       // added on 20.06.17 hot rinse switching was not happening
				}
		break;
		case	BC_EVENT_STOP:
		CSeventcount = 0;
				Cl_AlarmActivateAlarms(BC_ALARM,false);
				sv_cntrl_setflowpath(FLOW_PATH_IDLE_RINSE);
				bc_state = CL_BC_STATE_IDLE;
			
		break;
		case BC_EVENT_STOP_RINSE:
				openfillcounter = 0;
				bc_state = CL_BC_STATE_IDLE;
		break;
		case	BC_EVENT_PAUSE:
		break;
		case	BC_EVENT_RESET:
		break;
		case BC_EVENT_TEST_SWITCH:
		
		cl_bc_returncode = (Cl_ReturnCodeType)sv_cntrl_setflowpath(BC_FLUSH_1_V13_14);
		sv_cntrl_setpumpspeed(DCMOTOR2,1000);
		sv_cntrl_setpumpspeed(DCMOTOR1,1000);
		
		potvalue = (950 * 1024)/10000;                                 // 600
			sv_cs_setpotvalue(potvalue);
			
		bc_state = CL_BC_STATE_BO1_V13V14_TRANSITION;
		break;
		
		case BC_EVENT_DISINF_INTAKE:
		
			
			potvalue = (650 * 1024)/10000;                                 // 600
			sv_cs_setpotvalue(potvalue);
			
		bc_state = CL_BC_STATE_BO1_V13V14_TRANSITION;
		break;
	}
	switch (bc_state)
	{
			case CL_BC_STATE_IDLE:
			// wait for commands
			break;
			case CL_BC_STATE_NC:
			break;
			case CL_BC_STATE_OPENFILL_TRANSITION:
							switch(cl_bc_event)
							{							
								case BC_EVENT_500MS:				
									bc_wait = true;				
								break;
								case BC_EVENT_SECOND:
								if(bc_wait == true)
								{
									bc_laststate = bc_state;
									bc_state = CL_BC_STATE_OPENFILL;
									bc_wait = false;
								}
								break;
								case BC_EVENT_STOP_FOR_BC1:
								openfillcounter = 0;
								cl_bc_returncode = 
								(Cl_ReturnCodeType)sv_cntrl_setflowpath(BC_FLUSH_1_V13_14);
								if(cl_bc_returncode == CL_OK)
								{
									bc_laststate = bc_state;
									bc_state = CL_BC_STATE_STOPPED_FOR_BC1;
								}
								break;
								case BC_EVENT_STOP_FOR_BC2:
								openfillcounter = 0;
								cl_bc_returncode = (Cl_ReturnCodeType)sv_cntrl_setflowpath(BC_FLUSH_2_V13_14);
								if(cl_bc_returncode == CL_OK)
								{
									bc_laststate = bc_state;
									bc_state = CL_BC_STATE_STOPPED_FOR_BC2;
								}
								break;
								default:break;
							}
			break;
			case CL_BC_STATE_OPENFILL:
							switch(cl_bc_event)
							{
								case 	BC_EVENT_500MS:
								break;
								case	BC_EVENT_SECOND:
								
								if(openfill_flag == 1){
									
									Cl_Uint16Type potvalue = 0;
									potvalue = (650 * 1024)/10000;                                 // 600
									sv_cs_setpotvalue(potvalue);
										sv_cntrl_setpumpspeed(DCMOTOR2,500);
										sv_cntrl_setpumpspeed(DCMOTOR1,1000);
										
										if(disinfopenfillcounter++ > 900){ // 20 mins
											
											openfillcounter = 0;
											disinfopenfillcounter = 0;
											sv_prop_stopopenfill(); // manav02 added
								//			sv_prop_startmixing();
											cl_bc_returncode = (Cl_ReturnCodeType)sv_cntrl_setflowpath(BC_FLUSH_1_V13_14);
											cl_bc_returncode = sv_cntrl_deactivate_valve(VALVE_ID19);
											cl_bc_returncode = Cl_mac_apprequesthandler(MACREQ_BC_OPENFILL_COMPLETED);
											if(cl_bc_returncode == CL_OK)
											{
												Cl_AlarmActivateAlarms(BC_ALARM,false);           // manav02 deactivated
												bc_laststate = bc_state;
												bc_state = CL_BC_STATE_BO1_V13V14_TRANSITION;
												bc_laststate = bc_state;
											}
		
											
											
										}
									Cl_AlarmActivateAlarms(BC_ALARM,false);
									
								}
								
								else{
								
								
								if(openfillcounter++ > CL_BC_OPENFILL_TIMEOUT * 60)
								{
									
									
									if (openfill_flag == 1)
									{
										//sv_prop_stopopenfill();
										openfillcounter = 0;
										
										sv_cntrl_setpumpspeed(DCMOTOR2,500);
										sv_cntrl_setpumpspeed(DCMOTOR1,1000);
										
									}
									else
									{
									sv_prop_stopopenfill();
									sv_prop_startpropeo_aligning();
							
									openfillcounter = 0;
										
										sv_cntrl_setpumpspeed(DCMOTOR2,1000);
										sv_cntrl_setpumpspeed(DCMOTOR1,1100);
										potvalue = (670 * 1024)/10000;                                 // added on 30.06.2017
										sv_cs_setpotvalue(potvalue);
										if (hotrinse_flag == 1)
										{
											 
											 sv_cntrl_setpumpspeed(DCMOTOR2,600);
											 sv_cntrl_setpumpspeed(DCMOTOR1,1100);
												sv_cntrl_enable_loopback();
										//		sv_cntrl_enable_bypass();									// commented on 30.06.2017
										    
											Cl_AlarmActivateAlarms(BC_ALARM,false);           // manav02 deactivated added
											potvalue = (690 * 1024)/10000;                    //manav02 changed from 800             // added on 30.06.2017
											sv_cs_setpotvalue(potvalue);
											
												sv_cntrl_poweronheater();
												sv_cntrl_incheater(4000);
												SetHeaterState(CL_HEATER_STATE_ON);
												Cl_Heater_Event_Handler(CL_HEATER_EVENT_TARGET_TEMP , 80);
											
											sv_cntrl_activate_valve(VALVE_ID2);
											sv_cntrl_activate_valve(VALVE_ID3);
											sv_prop_startmixing();
											cl_uf_controller(CL_UF_EVENT_START,0);
											cl_ufpumpFeedback_start();
											cl_wait(100);
											cl_uf_controller(CL_UF_EVENT_SET_UF_RATE,16000);
											cl_wait(200);
											cl_uf_controller(CL_UF_EVENT_SET_UF_RATE,8000);
											cl_wait(600);
											cl_uf_controller(CL_UF_EVENT_SET_UF_RATE,5000);
											cl_wait(800);
											cl_uf_controller(CL_UF_EVENT_SET_UF_RATE,2500);
										}
										
									}
								//	sv_prop_startmixing();
								//	SetHeaterState(CL_HEATER_STATE_START);
//check	
									potvalue = (600 * 1024)/10000;                                 // added on 30.06.2017
									sv_cs_setpotvalue(potvalue);

									sv_cntrl_setpumpspeed(DCMOTOR2,1000);
									sv_cntrl_setpumpspeed(DCMOTOR1,1000);  //960
									
									
									cl_bc_returncode = (Cl_ReturnCodeType)sv_cntrl_setflowpath(BC_FLUSH_1_V13_14);
									cl_bc_returncode = sv_cntrl_deactivate_valve(VALVE_ID19);
									sv_cntrl_deactivate_valve(VALVE_ID4);
									cl_bc_returncode = Cl_mac_apprequesthandler(MACREQ_BC_OPENFILL_COMPLETED);
									if(cl_bc_returncode == CL_OK)
									{
										Cl_AlarmActivateAlarms(BC_ALARM,true); // manav02 activated
										bc_laststate = bc_state;
										bc_state = CL_BC_STATE_BO1_V13V14_TRANSITION;
										bc_laststate = bc_state;
									}
								}
								}
								break;
								case	BC_EVENT_CS:
								break;
								case BC_EVENT_STOP_FOR_BC1:
								openfillcounter = 0;
								cl_bc_returncode = (Cl_ReturnCodeType)sv_cntrl_setflowpath(BC_FLUSH_1_V13_14);
								if(cl_bc_returncode == CL_OK)
								{
									bc_laststate = bc_state;
									bc_state = CL_BC_STATE_STOPPED_FOR_BC1;
								}
								break;
								case BC_EVENT_STOP_FOR_BC2:
								openfillcounter = 0;
								cl_bc_returncode = (Cl_ReturnCodeType)sv_cntrl_setflowpath(BC_FLUSH_2_V13_14);
								if(cl_bc_returncode == CL_OK)
								{
									bc_laststate = bc_state;
									bc_state = CL_BC_STATE_STOPPED_FOR_BC2;
								}
								break;
								default:break;
							}
			break;
			case CL_BC_STATE_V15OPEN_TRANSITION:
							switch(cl_bc_event)
							{
								
								case BC_EVENT_500MS:
									bc_wait = true;
								
								break;
								case BC_EVENT_SECOND:
								if(bc_wait == true)
								{
									bc_laststate = bc_state;
									bc_state = CL_BC_STATE_V15OPEN;
									bc_wait = false;
								}
								break;
								default:break;
							}
			break;
			case CL_BC_STATE_V15OPEN:
							switch(cl_bc_event)
							{
								case 	BC_EVENT_500MS:
								break;
								case	BC_EVENT_SECOND:
										
								break;
								case	BC_EVENT_CS:
								break;
								default:break;
							}
			break;
			case CL_BC_STATE_V2OPEN_TRANSITION:
							switch(cl_bc_event)
							{
								
								case BC_EVENT_500MS:
								bc_wait = true;
								
								break;
								case BC_EVENT_SECOND:
								if(bc_wait == true)
								{
									bc_laststate = bc_state;
									bc_state = CL_BC_STATE_V2OPEN;
									bc_wait = false;
								}
								break;
								default:break;
							}
			break;
			case CL_BC_STATE_V2OPEN:
							switch(cl_bc_event)
							{
								case 	BC_EVENT_500MS:
								break;
								case	BC_EVENT_SECOND:
								break;
								case	BC_EVENT_CS:
								break;
								default:break;
							}
			break;
			case CL_BC_STATE_BO1_V4_TRANSITION:
							switch(cl_bc_event)
							{
								
								case BC_EVENT_500MS:
									bc_laststate = bc_state;
									bc_state = CL_BC_STATE_BO1_V4;
									bc_wait = false;
								break;
								case BC_EVENT_SECOND:
							//	if(bc_wait == true)
								{
							//		bc_laststate = bc_state;
							//		bc_state = CL_BC_STATE_BO1_V4;
							//		bc_wait = false;
								}
								break;
								default:break;
							}
			break;
			case CL_BC_STATE_BO1_V4:
							switch(cl_bc_event)
							{
								case 	BC_EVENT_500MS:
								break;
								case	BC_EVENT_SECOND:
								break;
								case	BC_EVENT_CS:
								break;
								default:break;
							}
			break;
			case CL_BC_STATE_BO2_V4_TRANSITION:
							switch(cl_bc_event)
							{
								
								case BC_EVENT_500MS:
								bc_wait = true;
								
								break;
								case BC_EVENT_SECOND:
								if(bc_wait == true)
								{
									bc_laststate = bc_state;
									bc_state = CL_BC_STATE_BO2_V4;
									bc_wait = false;
								}
								break;
								default:break;
							}
			break;
			case CL_BC_STATE_BO2_V4:
							switch(cl_bc_event)
							{
								case 	BC_EVENT_500MS:
								break;
								case	BC_EVENT_SECOND:
								break;
								case	BC_EVENT_CS:
								break;
								default:break;
							}
			break;
			case CL_BC_STATE_BO1_V13V14_TRANSITION:
							switch(cl_bc_event)
							{
								case BC_EVENT_50MS:
								fill_time++;
								wait_4_propo_off ++;
// 								if(wait_counter < 20)
// 								{
// 									if(wait_counter == 2)
// 									{
// 								//		sv_cntrl_deactivate_valve(VALVE_ID20);
// 										sv_cntrl_deactivate_valve(VALVE_ID19);
// 								//		sv_cntrl_deactivate_valve(VALVE_ID4);
// 									}
// 									wait_counter ++;	// 1 sec wait
// 									
// 								}
// 								else
								{
									wait_counter = 0;
									bc_laststate = bc_state;
									bc_state = CL_BC_STATE_BO1_V13V14;
									
								}
								break;
								case BC_EVENT_SECOND:
						
								break;
				
							}
			break;
			case CL_BC_STATE_BO1_V13V14:
							switch(cl_bc_event)
							{
								case BC_EVENT_50MS:
								fill_time++;
								wait_4_propo_off ++;
									if (wait_4_propo_off == 42)
									{
										sv_cntrl_activate_valve(VALVE_ID19);
									//	sv_cntrl_activate_valve(VALVE_ID20);
									//	sv_cntrl_activate_valve(VALVE_ID4);
									}
									else if (wait_4_propo_off == 46)
									{
									//	sv_cntrl_deactivate_valve(VALVE_ID4);
									//	sv_cntrl_deactivate_valve(VALVE_ID20);
										sv_cntrl_deactivate_valve(VALVE_ID19);
										
									}
								break;
								case 	BC_EVENT_500MS:
								break;
								case	BC_EVENT_SECOND:
								if(fill_time > 150){
									
								//sv_cntrl_setyellowalarm();
// 								sv_cntrl_activate_valve( VALVE_ID4);
// 								cl_wait(50);
// 								sv_cntrl_deactivate_valve(VALVE_ID4);
// 								cl_wait(50);
//								Cl_Alarm_TriggerAlarm(BC_ALARM, 1);
													
								}
								
								break;
								case	BC_EVENT_CS:
									//switch_time1 = fill_time;
									current_sense = true;
									cl_tdata.word =0;
									cl_tdata.Twobyte = fill_time;
									cl_tdata.bytearray[2] = 22;
									cl_tdata_temp.Twobyte = fill_time;
									if (bc_flag == 1)
									{
										switch (flow_rate)
										{
											case 800:
												if ((fill_time > 55) || (fill_time < 45))
												{	
													
													switchb01alrcnt++ ;

													if(switchb01alrcnt >3){
														
// 													sv_cntrl_setyellowalarm();
// 													sv_cntrl_activate_valve( VALVE_ID4);
// 													cl_wait(50);
// 													sv_cntrl_deactivate_valve(VALVE_ID4);
// 													cl_wait(50);
													 Cl_Alarm_TriggerAlarm(BC_ALARM, 1);
													}
												}
												else
												{
													switchb01alrcnt = 0 ;
										//			sv_cntrl_resetyellowalarm();
												}
											break;
											
											case 500:
												if ((fill_time > 75) || (fill_time < 63))
												{
													switchb01alrcnt++ ;

													if(switchb01alrcnt >3){
														
																									
// 													sv_cntrl_setyellowalarm();
// 													sv_cntrl_activate_valve( VALVE_ID4);
// 													cl_wait(50);
// 													sv_cntrl_deactivate_valve(VALVE_ID4);
// 													cl_wait(50);
													Cl_Alarm_TriggerAlarm(BC_ALARM, 1);
													}
												}
												else
												{
													switchb01alrcnt = 0 ;
									//				sv_cntrl_resetyellowalarm();
												}
											break;
											
											case 400:
											break;
											
											case 300:
												if ((fill_time > 122) || (fill_time < 110))
												{
													
													switchb01alrcnt++ ;

													if(switchb01alrcnt >3){
														
													
// 													sv_cntrl_setyellowalarm();
// 													sv_cntrl_activate_valve( VALVE_ID4);
// 													cl_wait(50);
// 													sv_cntrl_deactivate_valve(VALVE_ID4);
// 													cl_wait(50);
													Cl_Alarm_TriggerAlarm(BC_ALARM, 1);
													}
												}
												else
												{
													switchb01alrcnt = 0 ;
										//			sv_cntrl_resetyellowalarm();
												}
											break;
											
											case 200:
												if (fill_time > 190)
												{
// 													sv_cntrl_setyellowalarm();
// 													sv_cntrl_activate_valve( VALVE_ID4);
// 													cl_wait(50);
// 													sv_cntrl_deactivate_valve(VALVE_ID4);
// 													cl_wait(50);
													Cl_Alarm_TriggerAlarm(BC_ALARM, 1);
												}
												else
												{
										//			sv_cntrl_resetyellowalarm();
												}
											break;
										}
									}

								cl_bc_returncode = (Cl_ReturnCodeType)sv_cntrl_setflowpath(BC_FLUSH_2_V13_14);
								
								wait_4_propo_off = 0;
							//	sv_cntrl_activate_valve(VALVE_ID20);
							//	sv_cntrl_activate_valve(VALVE_ID19);
							//	sv_cntrl_activate_valve(VALVE_ID4);
								Cl_SendDatatoconsole(CON_TX_COMMAND_COMMAND_SCRIPT_PRNIT,&cl_tdata,4);
								
//************************************************************************** 22082017*********************
// 								BC_count++;
// 								Current_BC = fill_time;
// //								if (BC_count >10)
// 								{
// 													
// 								
// 								
// 								if ((Current_BC - 120) >5 )
// 								{
// 									POT_VALUE = POT_VALUE - 5;
// 									potvalue = (POT_VALUE * 1024)/10000;
// 									sv_cs_setpotvalue(potvalue);
// 									if(POT_VALUE < 570)
// 										POT_VALUE = 570;
// 								}
// 								
// 								if ((120-Current_BC) >5 )
// 								{
// 									POT_VALUE = POT_VALUE + 5;									
// 									potvalue = (POT_VALUE * 1024)/10000;
// 									sv_cs_setpotvalue(potvalue);
// 									if(POT_VALUE > 800)
// 									POT_VALUE = 700;
// 								}
// 								
// 								BC_count =0;
// 								}
//************************************************************************** 22082017*********************								
								Cl_SendDatatoconsole(CON_TX_COMMAND_COMMAND_BC_STATUS,&cl_tdata_temp,2);
								CSeventcount++;
								if(cl_bc_returncode == CL_OK)
								{
									bc_laststate = bc_state;
									bc_state = CL_BC_STATE_BO2_V13V14_TRANSITION;	
									fill_time = 0;															
								}
								break;
								default:break;								
							}
			break;
			case CL_BC_STATE_BO2_V13V14_TRANSITION:

							switch(cl_bc_event)
							{
								case BC_EVENT_50MS:
								fill_time++;
// 								if(wait_counter < 20)
// 								{
// 									if(wait_counter == 2)
// 									{
// 									//	sv_cntrl_deactivate_valve(VALVE_ID20);
// 								//		sv_cntrl_deactivate_valve(VALVE_ID19);
// 									//	sv_cntrl_deactivate_valve(VALVE_ID4);
// 									}
// 									
// 									wait_counter ++;	// 1 sec wait
// 									
// 								}
// 								else
								{
									wait_counter = 0;
									bc_laststate = bc_state;
									bc_state = CL_BC_STATE_BO2_V13V14;
								}
								break;
								case BC_EVENT_SECOND:
								
								break;
								
							}
			break;
			case CL_BC_STATE_BO2_V13V14:
							switch(cl_bc_event)
							{
								case BC_EVENT_50MS:
								fill_time++;
								wait_4_propo_off ++;
									if (wait_4_propo_off == 42)
									{
										sv_cntrl_activate_valve(VALVE_ID19);
									//	sv_cntrl_activate_valve(VALVE_ID20);
									//	sv_cntrl_activate_valve(VALVE_ID4);
									}
									else if (wait_4_propo_off == 46)
									{
									//	sv_cntrl_deactivate_valve(VALVE_ID4);
									//	sv_cntrl_deactivate_valve(VALVE_ID20);
										sv_cntrl_deactivate_valve(VALVE_ID19);
										
									}
								break;
								case 	BC_EVENT_500MS:
								break;
								case	BC_EVENT_SECOND:
									if(fill_time > 150){
										
					//					sv_cntrl_setyellowalarm();
// 										sv_cntrl_activate_valve( VALVE_ID4);
// 										cl_wait(50);
// 										sv_cntrl_deactivate_valve(VALVE_ID4);
// 										cl_wait(50);
//										Cl_Alarm_TriggerAlarm(BC_ALARM, 1);
										
									}
								
								break;
								case	BC_EVENT_CS:
								//switch_time2 = fill_time;
								
								 current_sense = true;
									cl_tdata.word =0;
									cl_tdata.Twobyte = fill_time;
									cl_tdata.bytearray[2] = 21;
									
								cl_tdata_temp.Twobyte = fill_time ;
							//	cl_tdata_temp.word = cl_tdata_temp.word << 16;
									if (bc_flag == 1)
									{
										switch (flow_rate)
										{
											case 800:
												if ((fill_time > 55) || (fill_time < 45))
												{
													
													switchb02alrcnt++ ;

													if(switchb02alrcnt >3){
// 													sv_cntrl_setyellowalarm();
// 													sv_cntrl_activate_valve( VALVE_ID4);
// 													cl_wait(50);
// 													sv_cntrl_deactivate_valve(VALVE_ID4);
// 													cl_wait(50);
													Cl_Alarm_TriggerAlarm(BC_ALARM, 1);
													}
												}
												else
												{
													switchb02alrcnt = 0;
								//					sv_cntrl_resetyellowalarm();
												}
											break;
											
											case 500:
												if ((fill_time > 75) || (fill_time < 63))
												{
													
													switchb02alrcnt++ ;

													if(switchb02alrcnt >3){
									//				sv_cntrl_setyellowalarm();
// 													sv_cntrl_activate_valve( VALVE_ID4);
// 													cl_wait(50);
// 													sv_cntrl_deactivate_valve(VALVE_ID4);
// 													cl_wait(50);
													Cl_Alarm_TriggerAlarm(BC_ALARM, 1);
													}
												}
												else
												{
													switchb02alrcnt = 0;
										//			sv_cntrl_resetyellowalarm();
												}
											break;
											
											case 400:
											break;
											
											case 300:
											if ((fill_time > 122) || (fill_time < 110))
											{
												
												switchb02alrcnt++ ;

												if(switchb02alrcnt >3){
							//					sv_cntrl_setyellowalarm();
// 												sv_cntrl_activate_valve( VALVE_ID4);
// 												cl_wait(50);
// 												sv_cntrl_deactivate_valve(VALVE_ID4);
// 												cl_wait(50);
												Cl_Alarm_TriggerAlarm(BC_ALARM, 1);
												}
											}
											else
											{
												switchb02alrcnt = 0;
									//			sv_cntrl_resetyellowalarm();
											}
											break;
											
											case 200:
												if (fill_time > 190)
												{
													sv_cntrl_setyellowalarm();
													sv_cntrl_activate_valve( VALVE_ID4);
													cl_wait(50);
													sv_cntrl_deactivate_valve(VALVE_ID4);
													cl_wait(50);
													Cl_Alarm_TriggerAlarm(BC_ALARM, 1);
												}
												else
												{
								//					sv_cntrl_resetyellowalarm();
												}
											break;
										}
									}
			//				
								cl_bc_returncode = (Cl_ReturnCodeType)sv_cntrl_setflowpath(BC_FLUSH_1_V13_14);
								wait_4_propo_off =0;
						//		sv_cntrl_activate_valve(VALVE_ID19);
						//		sv_cntrl_activate_valve(VALVE_ID20);
						//		sv_cntrl_activate_valve(VALVE_ID4);
								Cl_SendDatatoconsole(CON_TX_COMMAND_COMMAND_SCRIPT_PRNIT,&cl_tdata,4);
								
//************************************************************************** 22082017*********************								
// 								BC_count++;
// 								Current_BC = fill_time;
// //								if (BC_count >10)
// 								{
// 								
// 									if ((Current_BC - 120) >5 )
// 									{
// 										POT_VALUE = POT_VALUE - 5;
// 										potvalue = (POT_VALUE * 1024)/10000;
// 										sv_cs_setpotvalue(potvalue);
// 										if(POT_VALUE < 570)
// 										POT_VALUE = 570;
// 									}
// 									
// 									if ((120-Current_BC) >5 )
// 									{
// 										POT_VALUE = POT_VALUE + 5;
// 										potvalue = (POT_VALUE * 1024)/10000;
// 										sv_cs_setpotvalue(potvalue);
// 										if(POT_VALUE > 800)
// 										POT_VALUE = 700;
// 									}
// 									
// 									BC_count =0;
// 								}
								
//************************************************************************** 22082017*********************							
								Cl_SendDatatoconsole(CON_TX_COMMAND_COMMAND_BC_STATUS,&cl_tdata_temp,2);
								CSeventcount++;
								if(cl_bc_returncode == CL_OK)
								{
									bc_laststate = bc_state;
									fill_time = 0;
									bc_state = CL_BC_STATE_BO1_V13V14_TRANSITION;
								}
								break;
								default:break;
							}
			break;
			case CL_BC_STATE_STOPPED_FOR_BC1:
							switch(cl_bc_event)
							{
								case BC_EVENT_START_BC1:
								cl_bc_returncode = (Cl_ReturnCodeType)sv_cntrl_setflowpath(BC_FLUSH_1_V13_14);
								bc_state = CL_BC_STATE_BC1_TRANSITION;
								break;
								case BC_EVENT_START:
								cl_bc_returncode = sv_cntrl_setflowpath(BC_FLUSH_OPEN);
								bc_laststate = bc_state;
								bc_state = CL_BC_STATE_OPENFILL_TRANSITION;
								cl_bc_returncode = sv_cntrl_activate_valve(VALVE_ID19);
								break;
								default:break;
						
							}
						
			break;

			case CL_BC_STATE_STOPPED_FOR_BC2:
							switch(cl_bc_event)
							{
								case BC_EVENT_START_BC2:
								cl_bc_returncode = (Cl_ReturnCodeType)sv_cntrl_setflowpath(BC_FLUSH_2_V13_14);
								bc_state = CL_BC_STATE_BC2_TRANSITION;
								break;
								case BC_EVENT_START:
								cl_bc_returncode = sv_cntrl_setflowpath(BC_FLUSH_OPEN);
								bc_laststate = bc_state;
								cl_bc_returncode = sv_cntrl_activate_valve(VALVE_ID19);
								bc_state = CL_BC_STATE_OPENFILL_TRANSITION;
								break;
								default:break;
								
							}
			break;
			case CL_BC_STATE_COMPLETED_BC1:
			break;
			case CL_BC_STATE_COMPLETED_BC2:
			break;
			default:break;
	}

	
	return 0;
}
