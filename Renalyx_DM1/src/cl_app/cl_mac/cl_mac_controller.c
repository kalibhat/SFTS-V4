/*
 * cl_mac_controller.c
 *
 * Created: 12/27/2013 12:17:08 PM
 *  Author: sanjeer_k
 */
 
#include "asf.h"
#include "cl_mac_include.h"
#include "Platform/Service/sv_interface.h"
Cl_ReturnCodeType Cl_Handle_Error(void);



/**********************************/
#define NVRAM_BUS_ADDR       0x77        //!< TWI slave bus address (digital pot)
#define NVRAM_MEM_ADDR       0x0002        //!< TWI slave memory address

#define DIPOT_BUS_ADDR       0x2C        //!< TWI slave bus address (digital pot)
#define TWI_SPEED            1000000       //!< TWI data transfer rate(100Khz)
#define TWI_EXAMPLE TWI0
#define PATTERN_TEST_LENGTH     sizeof(testpattern)
#define PATTERN_TEST_LENGTH     sizeof(testpattern)


/****************************************/

#define FOREVER 1

Cl_Mac_StatesType Cl_MacState = MAC_START;
Cl_Mac_StatesType Cl_SelfTest_prevMacState = MAC_STANDBY;
Cl_Mac_EventsType Cl_MacEvent = EVT_NULL;
Cl_Sys_StatesType cl_sys_state = CL_SYS_STATE_IDLE;
Cl_BoolType Cl_MacNewEvent = false;
Cl_BoolType ConsoleSync = true;
Cl_Mac_EventQueueType Cl_MacEventQueue1 ;
Cl_Mac_EventQueueType Cl_MacEventQueue2 ;
Cl_Mac_EventQueueType Cl_MacEventQueue3 ;
Cl_Mac_EventQueueType Cl_MacEventQueue4 ; 
Cl_Mac_EventQueueType Cl_MacEventQueueBc ;                                                                                                 ;
cl_console_bufferType Cl_Console_buffer,Cl_Console_buffer1,Cl_Console_buffer2;
cl_console_msgbufferType Cl_Console_msgbuffer;
Cl_ConsoleMsgType Cl_ConsoleRxMsg;
Cl_Sys_statusType sv_sys_statbuffer;
Cl_Uint8Type Cl_ConsoleTxMsg[40];
Cl_Uint8Type Cl_IIC_ConsoleTxMsg[40];
Cl_Uint8Type testbool = false;
Cl_Sys_statusType  cl_sys_statbuffer;

Cl_Uint32Type ul_cur_ticks, delayticks; // testing
	
extern Cl_Mac_StatesType Cl_Mac_StateDummy;
extern volatile Cl_Uint32Type g_ul_ms_ticks ;

extern Cl_ReturnCodeType cl_wait(Cl_Uint32Type ul_dly_ticks);

extern volatile Cl_Uint32Type g_ul_recv_status;
extern can_mb_conf_t can0_mailbox;
extern Cl_Uint32Type can_flag;

extern uint8_t sv_cntrl_setoutput(uint32_t p_io, uint32_t peri);
extern uint8_t sv_cntrl_resetoutput(uint32_t p_io, uint32_t peri);
Sys_statusType temp_sensor_struct;

extern Cl_ConsoleTxCommandtype ackCommand;

extern Cl_ConsoleTxCommandtype cur_command;
extern Cl_Uint8Type cur_data ;
extern Cl_Uint8Type cur_datasize;
extern Cl_Uint8Type flag_hotcitric, flag_hotrinse,flag_na , flag_citric;
Cl_Uint8Type queue_sequence_counter = 0;
int main(void)

{
	   
	Cl_ReturnCodeType 	 Cl_MacRetval = CL_ERROR;
	Cl_Mac_EventsType Cl_MacstbEvent = EVT_NULL;
	Cl_Uint8Type Cl_MacEventData = 0;
	static uint loopcountm = 0, loopcountl = 0 , loopcountinsec = 0;
	Cl_Uint16Type temp100;
	   cl_Datastreamtype cl_temp  ;
	board_init();
	sysclk_init();	
	SV_SYSTEM_INIT();
	                                                                                           
//	udc_start();
	// Insert application code here, after the board has been initialized.
	//timerThread(  ) ;
//	Cl_MacRetval = sv_system_init(); 
pmc_enable_periph_clk(ID_PIOA);
pmc_enable_periph_clk(ID_TWI0);

	sv_cntrl_activatepump(DCMOTOR1);
	sv_cntrl_activatepump(DCMOTOR2);
	sv_cntrl_activatepump(UFPUMP);
	sv_cntrl_activatepump(HEPARINPUMP);

	sv_cntrl_deactivatepump(DCMOTOR1);
	sv_cntrl_deactivatepump(DCMOTOR2);
	sv_cntrl_deactivatepump(UFPUMP);
	sv_cntrl_deactivatepump(HEPARINPUMP);
	
// 	sv_cntrl_setpumpspeed(DCMOTOR2,1400);
// 	sv_cntrl_activatepump(DCMOTOR2);
   
	Cl_MacRetval = Cl_MacInit(); 	
	for(temp100 = 1;temp100<=20; temp100++)
	{
		sv_cntrl_deactivate_valve((sv_valvetype)temp100);
		cl_wait(10);
	}
//	SV_CS_SET_POT(3800);				  
//	sv_cs_setpotvalue(3800);


	
	sv_cntrl_resetoutput(29,PA);                         //  V1 Hardware control disabled
	sv_cntrl_setoutput(10,PB);                            //  V1 on Software control    
	sv_cntrl_resetoutput(27,PC);
	sv_cntrl_setoutput(14,PB);                            // Blood Pump Power Control    
	
	sv_cntrl_setoutput(2,PD);
	
	
	
	
	sv_cntrl_buzzer();
	sv_cntrl_nobuzzer();
	sv_cntrl_setyellowalarm();
	sv_cntrl_resetyellowalarm();
	sv_cntrl_setredalarm();
	sv_cntrl_resetredalarm();
	sv_cntrl_setgreenalarm();
	sv_cntrl_resetgreenalarm();
//sv_cntrl_setoutput(27,PC);

// while(1)
// {
// 	sv_status_getsystemstatus(&temp_sensor_struct);
// 	;
// 	
// }
	
	sv_cntrl_activate_valve( VALVE_ID17);
	
	Cl_MacRetval = Cl_App_init(); 
	 ASSERT_ERROR(Cl_MacRetval);
	
	if( Cl_MacRetval != CL_OK)
	{
		// Handle error
	}
	Cl_MacRetval = Cl_Mac_Configure();
	 ASSERT_ERROR(Cl_MacRetval);
		if ( Cl_MacRetval == CL_ACCEPTED)
		{
			
		// Configure the system for a default configuration
		}
		else
		{
		// Handle error
		}		
	//	 if ( Cl_MacRetval == CL_ACCEPTED)
		 {
		 // System is ready for sending and receiving the data from console . A timer will read the data to be fetched by the MAC main loop for further processing
			
			Cl_MacState = MAC_CONSOLE_SYNC;
			Cl_MacRetval = CL_ERROR;
			Cl_MacRetval = Cl_Console_Console_Sync();
			ASSERT_ERROR(Cl_MacRetval);
			if(Cl_MacRetval == CL_OK)
			{
				cl_sys_state = CL_SYS_STATE_WAIT_FOR_MODE;
				Cl_MacState =   MAC_INIT;
			}

			
		 }
		 	
 #if 0
		 if( cl_sys_state == CL_SYS_STATE_WAIT_FOR_MODE)
		 {
			 while (FOREVER)
			 {
				 

				 Cl_MacRetval = Cl_MacCheckforConsoleMessage();
				 
					while(Cl_MacEventQueue1.Cl_MacNewEventcount > 0)

					{
						Cl_MacRetval = Cl_MacCreate_Macevent();
											
						if(Cl_MacNewEvent == EVT_COMMAND_ENTER_INIT)
						{
							Cl_MacRetval = Cl_Init_Controller(Cl_MacEvent);
							
							Cl_MacState =   MAC_STANDBY;
						}
						else
						{
						
							if(Cl_MacEvent == EVT_CONSOLE_COMMAND_SERVICEMODE_ENTER)
							{
								// call calibrate entry and loop there; else break and continue following flow
							}
							else if (Cl_MacEvent == EVT_CONSOLE_COMMAND_USER_MODE_ENTER)
							{
								break;
							
							}
						}
					}
					
							 if (Cl_MacEvent == EVT_CONSOLE_COMMAND_USER_MODE_ENTER)
							{
								break;
							}
			 }
		 }
		 
	
		 
		 
		 

#endif
	while (FOREVER)  // LOOP MAIN
	{

//sv_cntrl_setyellowalarm();
//sv_cntrl_deactivate_valve( VALVE_ID18);
	Cl_Mac_StateDummy = Cl_MacState;
	ul_cur_ticks = g_ul_ms_ticks;
	
		Cl_MacRetval = Cl_Mac_GetSystem_Status_Query();// check if there is any system updates . ALSO may be this can be moved to a timer thread
		Cl_MacRetval = Cl_Mac_CheckforBCEvent();
		Cl_MacRetval = Cl_Mac_CheckforAlarms();
		ASSERT_ERROR(Cl_MacRetval);
			if( Cl_MacRetval != CL_OK)
			{
				// Handle error
			}
		Cl_MacRetval = Cl_MacCheckforConsoleMessage();// check if there is any updates from console .. to be shifted to a polling timer..???
		ASSERT_ERROR(Cl_MacRetval);
			if( Cl_MacRetval != CL_OK)
			{
				// Handle error
			}
		// TODO :  check for ret val
		delayticks = g_ul_ms_ticks - ul_cur_ticks; // testing-debugging
		
		Cl_MacRetval = cl_event_handler(EVT_TICK_INSTANT);
		ASSERT_ERROR(Cl_MacRetval);
			if( Cl_MacRetval != CL_OK)
			{
				// Handle error
			}
		// TODO :  check for ret val
		
		while((Cl_MacEventQueueBc.Cl_MacNewEventcount > 0) || (Cl_MacEventQueue1.Cl_MacNewEventcount > 0) || (Cl_MacEventQueue2.Cl_MacNewEventcount > 0) || (Cl_MacEventQueue3.Cl_MacNewEventcount > 0) || (Cl_MacEventQueue4.Cl_MacNewEventcount > 0))  // LOOP EVENT DISPATCH
		
		{
		Cl_MacEvent = EVT_NULL;
		Cl_MacRetval = Cl_MacCreate_Macevent();
		ASSERT_ERROR(Cl_MacRetval);
			if( Cl_MacRetval != CL_OK)
			{
				// Handle error
			}

		if(Cl_MacNewEvent)
		
		{
			Cl_MacRetval = cl_event_handler(Cl_MacEvent);
			ASSERT_ERROR(Cl_MacRetval);
			if( Cl_MacRetval != CL_OK)
			{
				// Handle error
			}

			if(Cl_MacEvent == EVT_CONSOLE_COMMAND_ACK)
			{
				//send sync
				// save the data command 
				if(ackCommand == Cl_ConsoleRxMsg.data.byte[0]){
					
				}
				else{
					// again send 
			//		Cl_SendDatatoconsole(cur_command, cur_data,cur_datasize);
				}
				ConsoleSync = true;
			}
			
			if(Cl_MacEvent == EVT_CONSOLE_COMMAND_SYNC)
			{
				//send sync
				Cl_Uint8Type data = CON_RX_COMMAND_SYNC;
				ConsoleSync = true;
				Cl_MacRetval = Cl_SendDatatoconsole(CON_TX_COMMAND_ACK,&data,1);
				ASSERT_ERROR(Cl_MacRetval);
				if( Cl_MacRetval != CL_OK)
				{
				// Handle error
				}
				
			}			
			
			
				switch (Cl_MacState)
				{			
					case MAC_START:
					break;
					case MAC_IDLE:
					break;
					case MAC_INIT:
				
					switch ( Cl_MacEvent )
					{
						case EVT_COMMAND_ENTER_INIT:
						Cl_MacRetval = Cl_Init_Controller(Cl_MacEvent);
						ASSERT_ERROR(Cl_MacRetval);
								if( Cl_MacRetval != CL_OK)
								{
								// Handle error
								}
			
							Cl_MacEvent = EVT_NULL;
						break;

						case MACREQ_ENTER_STANDBY:
						Cl_MacRetval = Cl_Init_Controller(EVT_COMMAND_ENTER_STANDBY);
						ASSERT_ERROR(Cl_MacRetval);
								if( Cl_MacRetval == CL_OK)
								{
									Cl_MacRetval = Cl_Standby_Controller(EVT_COMMAND_ENTER_STANDBY);
									ASSERT_ERROR(Cl_MacRetval);
								}
								
								if( Cl_MacRetval == CL_OK)
								{
									Cl_MacEvent = EVT_NULL;
									Cl_MacState =   MAC_STANDBY;
									//Cl_MacRetval = Cl_SendDatatoconsole(CON_TX_COMMAND_COMMAND_SYSTEM_STATE,(void*)MAC_STANDBY,1);
									ASSERT_ERROR(Cl_MacRetval);
								}
								

								if( Cl_MacRetval != CL_OK)
								{
									// Handle error
								}
						break;
						case EVT_TICK_100M:	
						case EVT_TICK_SEC:
						case EVT_TICK_MIN:
						case EVT_TICK_HOUR:
						Cl_MacRetval = Cl_Init_Controller(Cl_MacEvent);
								ASSERT_ERROR(Cl_MacRetval);
								if( Cl_MacRetval != CL_OK)
								{
									// Handle error
								}
							Cl_MacEvent = EVT_NULL;

						break;
						default: break;
					}
					break;
					
					case MAC_CONSOLE_SYNC:
				
					break;
					case MAC_STANDBY:
				
					switch ( Cl_MacEvent )
				
					{
						
						case EVT_CONSOLE_COMMAND_CLR_ALARM:
						
							Cl_MacRetval = Cl_Standby_Controller(Cl_MacEvent);
							ASSERT_ERROR(Cl_MacRetval);
										if( Cl_MacRetval != CL_OK)
										{
											// Handle error
										}
							Cl_MacEvent = EVT_NULL;
							break;
						case EVT_COMMAND_ENTER_STANDBY:
						case EVT_CONSOLE_COMMAND_MUTE_ALARM:
						case EVT_TICK_100M:
						case EVT_TICK_SEC:
						case EVT_TICK_MIN:
						case EVT_TICK_HOUR:			
						case EVT_ALERT_TRIGGERED:	
						case EVT_ALARM_TRIGGERED:
						case EVT_CONSOLE_COMMAND_SET_BLDPMP_ON:
						case EVT_CONSOLE_COMMAND_SET_BLDPMP_OFF:
						case EVT_CONSOLE_COMMAND_SET_BLDPUMPRATE:
						case EVT_CONSOLE_COMMAND_SET_BLDPUMPINC_MANUAL:
						case EVT_CONSOLE_COMMAND_SET_BLDPUMPDEC_MANUAL:
						
							Cl_MacRetval = Cl_Standby_Controller(Cl_MacEvent);
							ASSERT_ERROR(Cl_MacRetval);
										if( Cl_MacRetval != CL_OK)
										{
											// Handle error
										}
							Cl_MacEvent = EVT_NULL;
							break;

					
						case EVT_CONSOLE_COMMAND_RINSE_START:
							loopcountinsec = 0;
							Cl_MacRetval = Cl_Standby_Controller(Cl_MacEvent);
							ASSERT_ERROR(Cl_MacRetval);

							if(Cl_MacRetval == CL_OK)
							{
								Cl_MacRetval = Cl_Rinse_Controller(Cl_MacEvent);
								if(Cl_MacRetval != CL_REJECTED)
								{
									ASSERT_ERROR(Cl_MacRetval);	
								}
								
							}							
							if( Cl_MacRetval == CL_OK)
							{
								Cl_MacState = MAC_RINSE;
								//Cl_MacRetval = Cl_SendDatatoconsole(CON_TX_COMMAND_COMMAND_SYSTEM_STATE,(void*)MAC_RINSE,1);
								ASSERT_ERROR(Cl_MacRetval);			
							}
							if( Cl_MacRetval != CL_OK)
							{
								// Handle error
							}
							Cl_MacEvent = EVT_NULL;
						break;
						case EVT_CONSOLE_COMMAND_START_PRIME:
							Cl_MacRetval = Cl_Rinse_Controller(Cl_MacEvent);
							ASSERT_ERROR(Cl_MacRetval);
							if( Cl_MacRetval == CL_OK)
							{
								Cl_MacState = MAC_POST_RINSE_STANDBY;
							}
							else
							{
									// Handle error
							}
							
						break;
						
						case EVT_CONSOLE_COMMAND_SELFTEST_START:
					
						{
							
				//	Cl_MacRetval = Cl_selftest_init();
						Cl_MacRetval = self_test(Cl_MacEvent);
						
						Cl_SelfTest_prevMacState = Cl_MacState;
						Cl_MacState = MAC_START_SELFTEST;
// 						command = CON_TX_COMMAND_RINSE_STARTED;
// 						Cl_rinseretcode = Cl_SendDatatoconsole(command,&data,0);
// 						
						}
						break;
						
						case EVT_CONSOLE_COMMAND_DISINF_START:
						
						//	Cl_MacRetval = Cl_Dlsis_controller(Cl_MacEvent);
						//	ASSERT_ERROR(Cl_MacRetval);
						//	if( Cl_MacRetval == CL_OK)
						//	{
								
								
								cl_temp.bytearray[0] = Cl_ConsoleRxMsg.data.byte[0];
								if (cl_temp.bytearray[0] == 4)
								{
									flag_hotcitric = 1;
								}
								else if (cl_temp.bytearray[0] == 3)
								{
									flag_na = 1;
								}
								else if(cl_temp.bytearray[0] == 5){
									
									flag_citric = 1;
								}
								
								Cl_MacRetval = Cl_disinf_controller(Cl_MacEvent);
								ASSERT_ERROR(Cl_MacRetval);
								Cl_MacState = MAC_DISINFECT;
							//	Cl_SendDatatoconsole(CON_TX_COMMAND_COMMAND_SYSTEM_STATE,(void*)MAC_DISINFECT,1);
								ASSERT_ERROR(Cl_MacRetval);
							//}
// 							else
// 							{
// 								// Handle error
// 							}
							

						break;
						
						case EVT_CONSOLE_COMMAND_DIALYSIS_PREP:
										
							Cl_MacRetval = Cl_Standby_Controller(Cl_MacEvent);
							ASSERT_ERROR(Cl_MacRetval);
							if(Cl_MacRetval == CL_ACCEPTED)
							{
								Cl_MacRetval = Cl_Rinse_Controller(Cl_MacEvent);
								ASSERT_ERROR(Cl_MacRetval);
							}
							
							if(Cl_MacRetval == CL_ACCEPTED)
							{
								Cl_MacRetval = Cl_dprep_controller(Cl_MacEvent);
								ASSERT_ERROR(Cl_MacRetval);
							}
							
							if( Cl_MacRetval == CL_OK)
							{
								Cl_MacState = MAC_DPREP;
							//	Cl_SendDatatoconsole(CON_TX_COMMAND_COMMAND_SYSTEM_STATE,(void*)MAC_DPREP,1);
								ASSERT_ERROR(Cl_MacRetval);
							}
							if( Cl_MacRetval != CL_OK)
							{
									// handle error
							}
							
							
							Cl_MacEvent = EVT_NULL;
							
							
						break;

						
						default: break;
					}
					break;
						case MAC_START_SELFTEST:                                                                   //  *************************************
						
						switch(Cl_MacEvent)
						{
							case EVT_TICK_50M:
							case EVT_TICK_100M:
							case EVT_TICK_500M:
							case EVT_TICK_SEC:
							case EVT_TICK_MIN:
							case EVT_CONSOLE_COMMAND_SELFTEST_START:
							//case EVT_CONSOLE_COMMAND_GET_DATA:
							//case EVT_CONSOLE_COMMAND_SET_DATA:
							case EVT_COSOLE_COMMAND_SELFTEST_ALARM_PAAS:
							case EVT_COSOLE_COMMAND_SELFTEST_ALARM_FAIL:
							case EVT_CONSOLE_COMMAND_SYNCCONFIRMED:
							Cl_MacRetval =  self_test(Cl_MacEvent);
							ASSERT_ERROR(Cl_MacRetval);
							if(Cl_MacRetval != CL_OK)
							{
								// handle error
							}
							
							Cl_MacEvent = EVT_NULL;
							break;
							
							case EVT_CONSOLE_COMMAND_SELFTEST_STOP:
							
			       				Cl_MacRetval = self_test(Cl_MacEvent);
						 		ASSERT_ERROR(Cl_MacRetval);
							    if( Cl_MacRetval == CL_OK)
								{
									
									
									Cl_MacRetval =Cl_selftest_init();
									
									Cl_MacRetval = Cl_Standby_Controller(EVT_COMMAND_ENTER_STANDBY);
									ASSERT_ERROR(Cl_MacRetval);
									//Cl_MacState =   MAC_STANDBY;
								 	Cl_MacState = Cl_SelfTest_prevMacState;//  MAC_STANDBY;	
									Cl_MacEvent = EVT_NULL;
									break;

								}
								else
								{
									Cl_MacEvent = EVT_NULL;    //handle error
								}
							
							break;
							
							
							
							case MACREQ_SELFTEST_COMPLETED:
							
							Cl_MacRetval = self_test(Cl_MacEvent);
							ASSERT_ERROR(Cl_MacRetval);
							if( Cl_MacRetval == CL_OK)
							{
								
								
 							Cl_MacRetval =Cl_selftest_init();
								 	
									 	Cl_MacRetval = Cl_Standby_Controller(EVT_COMMAND_ENTER_STANDBY);
									 	ASSERT_ERROR(Cl_MacRetval);
										 
									 	Cl_MacState = Cl_SelfTest_prevMacState;//  MAC_STANDBY;
								 	
								 	Cl_MacEvent = EVT_NULL;
								 	break;

							}
							else
							{
								//handle error
							}
							Cl_MacEvent = EVT_NULL;
							break;
							
// 							case EVT_CONSOLE_COMMAND_SELFTEST_STOP:
// 							Cl_MacRetval = self_test(Cl_MacEvent);
// 							ASSERT_ERROR(Cl_MacRetval);
// 							break;
// 							
						
						
						default: break;                                                              //        **********************************************
						
						}
					
					case MAC_DISINFECT:
					
					switch(Cl_MacEvent)
					{
						
						case EVT_TICK_50M:
						case EVT_TICK_100M:
						case EVT_TICK_500M:
						case EVT_TICK_SEC:
						case EVT_TICK_MIN:
						Cl_MacRetval = Cl_disinf_controller(Cl_MacEvent);
						ASSERT_ERROR(Cl_MacRetval);
							if( Cl_MacRetval == CL_OK)
							{
							Cl_MacRetval = Cl_Rinse_Controller(Cl_MacEvent);
							ASSERT_ERROR(Cl_MacRetval);
							}
							if( Cl_MacRetval != CL_OK)
							{
								// handle error
							}
						break;
						
						case EVT_CONSOLE_COMMAND_CLR_ALARM:
						case EVT_ALARM_TRIGGERED:
						
						Cl_MacRetval = Cl_disinf_controller(Cl_MacEvent);
						
						break;
						
						case EVT_CONSOLE_COMMAND_SET_BLDPMP_ON:
						case EVT_CONSOLE_COMMAND_SET_BLDPMP_OFF:
						case EVT_CONSOLE_COMMAND_SET_BLDPUMPRATE:
						case EVT_CONSOLE_COMMAND_DISINF_START:
						case EVT_CONSOLE_COMMAND_STOP_DISINF:
						case EVT_CONSOLE_RESUME_DISINF_STAGE:
						case EVT_CONSOLE_COMMAND_RINSE_START:
						case MACREQ_BC_OPENFILL_COMPLETED:
						case EVT_CONSOLE_COMMAND_SET_BLDPUMPINC_MANUAL:
						case EVT_CONSOLE_COMMAND_SET_BLDPUMPDEC_MANUAL:
						
						Cl_MacRetval = Cl_disinf_controller(Cl_MacEvent);
						ASSERT_ERROR(Cl_MacRetval);
							if( Cl_MacRetval != CL_OK)
							{
								// handle error
							}
						break;
						case MACREQ_START_PRE_DISINF_RINSE:
						case MACREQ_START_POST_DISINF_RINSE:
						Cl_MacRetval = Cl_Rinse_Controller(Cl_MacEvent);
						ASSERT_ERROR(Cl_MacRetval);
							if( Cl_MacRetval != CL_OK)
							{
								// handle error
							}
						break;
						case MACREQ_PRE_DISINFECT_RINSE_COMPLETED:
						case MACREQ_POST_DISINFECT_RINSE_COMPLETED:
						Cl_MacRetval = Cl_disinf_controller(Cl_MacEvent);
						ASSERT_ERROR(Cl_MacRetval);
							if( Cl_MacRetval != CL_OK)
							{
								// handle error
							}
						break;
						case MACREQ_DISINF_COMPLETED:
					//	Cl_MacRetval = Cl_Init_Controller(EVT_COMMAND_ENTER_STANDBY);
					//	ASSERT_ERROR(Cl_MacRetval);
					//		if( Cl_MacRetval == CL_OK)
							{
								Cl_MacRetval = Cl_Standby_Controller(EVT_COMMAND_ENTER_STANDBY);
								ASSERT_ERROR(Cl_MacRetval);
								Cl_MacState =   MAC_STANDBY;
							}
						Cl_MacEvent = EVT_NULL;
						break;
						default:break;
					}
	
					Cl_MacEvent = EVT_NULL;
					break;
					case MAC_SAFE_STANDBY:
					switch ( Cl_MacEvent )
				
					{
						case EVT_CONSOLE_COMMAND_RINSE_START:
						
						break;
						case EVT_CONSOLE_COMMAND_DIALYSIS_PREP:
						break;
						case EVT_ALARM_TRIGGERED:
						break;
						case EVT_ALERT_TRIGGERED:

						break;
						default: break;
					}
					break;
					case MAC_RINSE:

					switch ( Cl_MacEvent )
				
					{
						case EVT_CONSOLE_COMMAND_RINSE_START:
							loopcountinsec =0;
							Cl_MacRetval = Cl_Standby_Controller(Cl_MacEvent);
							ASSERT_ERROR(Cl_MacRetval);
							if(Cl_MacRetval == CL_OK)
							{
								Cl_MacRetval = Cl_Rinse_Controller(Cl_MacEvent);
								ASSERT_ERROR(Cl_MacRetval);	
							}
							else if(Cl_MacRetval == CL_OK)
							{
								Cl_MacRetval = Cl_Rinse_Controller(Cl_MacEvent);
								ASSERT_ERROR(Cl_MacRetval);
							}
							
							
							Cl_MacEvent = EVT_NULL;
						break;

						case EVT_TIMER_EXPIRED:
							Cl_MacRetval = Cl_Rinse_Controller(Cl_MacEvent);
							ASSERT_ERROR(Cl_MacRetval);
							if(Cl_MacRetval != CL_OK)
							{
								 // handle error
							}
							Cl_MacEvent = EVT_NULL;
						break;
						case EVT_CONSOLE_COMMAND_CLR_ALARM:
						case EVT_CONSOLE_COMMAND_DIALYSIS_PREP:
						case EVT_TICK_50M:
						case EVT_TICK_100M:
						case EVT_TICK_500M:
						case EVT_TICK_SEC:
						case EVT_TICK_MIN:
						case EVT_TICK_HOUR:
						case EVT_ALARM_TRIGGERED:
						case EVT_ALERT_TRIGGERED:
						case EVT_CONSOLE_COMMAND_MUTE_ALARM:
					//	case EVT_CONSOLE_COMMAND_GET_DATA:
					//	case EVT_CONSOLE_COMMAND_SET_DATA:
						case MACREQ_BC_OPENFILL_COMPLETED:
						case EVT_CONSOLE_COMMAND_SET_BLDPUMPINC_MANUAL:
						case EVT_CONSOLE_COMMAND_SET_BLDPUMPDEC_MANUAL:

							Cl_MacRetval = Cl_Rinse_Controller(Cl_MacEvent);
							ASSERT_ERROR(Cl_MacRetval);
							if(Cl_MacRetval != CL_OK)
							{
								// handle error
							}
							
							Cl_MacEvent = EVT_NULL;
						break;
						case MACREQ_RINSE_COMPLETED:
							Cl_MacRetval = Cl_Rinse_Controller(Cl_MacEvent);
							ASSERT_ERROR(Cl_MacRetval);
							if( Cl_MacRetval == CL_OK)
							{
								Cl_MacState = MAC_POST_RINSE_STANDBY;
								Cl_Uint8Type temp = MAC_POST_RINSE_STANDBY;
								//	Cl_SendDatatoconsole(CON_TX_COMMAND_COMMAND_SYSTEM_STATE,(Cl_Uint8Type*)temp,1);
									ASSERT_ERROR(Cl_MacRetval);
							}
							if(Cl_MacRetval != CL_OK)
							{
								// handle error
							}
							
							Cl_MacRetval = Cl_dlsis_init();
							
							Cl_MacEvent = EVT_NULL;
						break;
						case EVT_CONSOLE_COMMAND_STOP_RINSE:
							Cl_MacRetval = Cl_Rinse_Controller(Cl_MacEvent);
							ASSERT_ERROR(Cl_MacRetval);
							if( Cl_MacRetval == CL_OK)
							{
							//	Cl_MacState = MAC_POST_RINSE_STANDBY;
							}
							Cl_MacEvent = EVT_NULL;
						break;

						case EVT_CONSOLE_COMMAND_START_PRIME:
						case EVT_CONSOLE_COMMAND_STOP_PRIME:
						case EVT_CONSOLE_COMMAND_RCIRC_START:
						case EVT_CONSOLE_COMMAND_RCIRC_STOP:
						Cl_MacRetval = Cl_Rinse_Controller(Cl_MacEvent);
						ASSERT_ERROR(Cl_MacRetval);
							if(Cl_MacRetval != CL_OK)
							{
								// handle error
							}
						//Cl_MacRetval = Cl_dprep_controller(Cl_MacEvent);
						break;
						case EVT_CONSOLE_COMMAND_SET_BLDPMP_ON:
						case EVT_CONSOLE_COMMAND_SET_BLDPMP_OFF:
						case EVT_CONSOLE_COMMAND_SET_BLDPUMPRATE:
						case EVT_CONSOLE_COMMAND_HEPARIN_BOLUS:
						case EVT_CONSOLE_COMMAND_HEPARIN_START:
						case EVT_CONSOLE_COMMAND_HEPARIN_STOP:
						Cl_MacRetval = Cl_Rinse_Controller(Cl_MacEvent);
						ASSERT_ERROR(Cl_MacRetval);
							if(Cl_MacRetval != CL_OK)
							{
								// handle error
							}						
						break;
						default: break;
					}
					break;
					case MAC_POST_RINSE_STANDBY:
				switch ( Cl_MacEvent )
				
					{
						case EVT_CONSOLE_COMMAND_SELFTEST_START:
						
						{
							
							//	Cl_MacRetval = Cl_selftest_init();
							Cl_MacRetval = self_test(Cl_MacEvent);
							
							Cl_SelfTest_prevMacState = Cl_MacState;
							Cl_MacState = MAC_START_SELFTEST;
							// 						command = CON_TX_COMMAND_RINSE_STARTED;
							// 						Cl_rinseretcode = Cl_SendDatatoconsole(command,&data,0);
							//
						}
						break;
						
						
						case EVT_ALARM_TRIGGERED:
						case EVT_ALERT_TRIGGERED:
						case EVT_CONSOLE_COMMAND_MUTE_ALARM:
						Cl_MacRetval = Cl_Rinse_Controller(Cl_MacEvent);
						break;
						// manav05 commented as disinfect start event was generated internally  . need to debug
/*
						case EVT_CONSOLE_COMMAND_DISINF_START:

						Cl_MacRetval = Cl_Dlsis_controller(Cl_MacEvent);
						ASSERT_ERROR(Cl_MacRetval);
							if(Cl_MacRetval == CL_OK)
							{
							Cl_MacRetval = Cl_disinf_controller(Cl_MacEvent);
							ASSERT_ERROR(Cl_MacRetval);
							}
							if(Cl_MacRetval == CL_OK)
							{
								Cl_MacState = MAC_DISINFECT;
							}
							else
							{
								// handdle error
							}
						
						break;
*/						
						case EVT_CONSOLE_COMMAND_RINSE_START:
						
								Cl_MacRetval = Cl_Standby_Controller(Cl_MacEvent);
								ASSERT_ERROR(Cl_MacRetval);
								if(Cl_MacRetval == CL_OK)
								{
									Cl_MacRetval = Cl_Rinse_Controller(Cl_MacEvent);
									ASSERT_ERROR(Cl_MacRetval);
								}
													
								if( Cl_MacRetval == CL_OK)
								{
									Cl_MacState = MAC_RINSE;
									//Cl_MacState = MAC_POST_RINSE_STANDBY;						
								}
								if( Cl_MacRetval == CL_OK)
								{
								//	Cl_MacRetval = Cl_SendDatatoconsole(CON_TX_COMMAND_COMMAND_SYSTEM_STATE,(void*)MAC_RINSE,1);
									ASSERT_ERROR(Cl_MacRetval);
								}
								if( Cl_MacRetval != CL_OK)
								{
									// handle error
								}
								
								Cl_MacEvent = EVT_NULL;
						break;
						case EVT_CONSOLE_COMMAND_DIALYSIS_PREP:
																				
							Cl_MacRetval = Cl_Standby_Controller(Cl_MacEvent);
							ASSERT_ERROR(Cl_MacRetval);
							if(Cl_MacRetval == CL_OK)
							{
								Cl_MacRetval = Cl_Rinse_Controller(Cl_MacEvent);
								ASSERT_ERROR(Cl_MacRetval);
							}
							if(Cl_MacRetval == CL_OK)
							{
								Cl_MacRetval = Cl_dprep_controller(Cl_MacEvent);
								ASSERT_ERROR(Cl_MacRetval);
							}
							if(Cl_MacRetval == CL_OK)
							{
									Cl_MacState = MAC_DPREP;
							}
							if(Cl_MacRetval == CL_OK)
							{
							//Cl_MacRetval = Cl_SendDatatoconsole(CON_TX_COMMAND_COMMAND_SYSTEM_STATE,(void*)MAC_DPREP,1);
							ASSERT_ERROR(Cl_MacRetval);
							}
							else
							{
								// handle error
							}
						break;
						case EVT_SEND_MIXING_PREP_START://0x21
						Cl_MacRetval = Cl_dprep_controller(Cl_MacEvent);
						ASSERT_ERROR(Cl_MacRetval);
							if(Cl_MacRetval == CL_OK)
							{
									Cl_MacState = MAC_DPREP;
							}
							else
							{
								// handle error
							}
						
						break;
						case EVT_SEND_MIXING_PREP_STOP://0x22
						break;

						case EVT_CONSOLE_COMMAND_RCIRC_START:
						case EVT_CONSOLE_COMMAND_RCIRC_STOP:
						Cl_MacRetval = Cl_Rinse_Controller(Cl_MacEvent);
						ASSERT_ERROR(Cl_MacRetval);
							if(Cl_MacRetval == CL_OK)
							{
								Cl_MacState = MAC_DPREP;
							}
							else
							{
								// handle error
							}
													
						break;
						case EVT_CONSOLE_COMMAND_START_PRIME:
						case EVT_CONSOLE_COMMAND_STOP_PRIME:
						Cl_MacRetval = Cl_Rinse_Controller(Cl_MacEvent);
						ASSERT_ERROR(Cl_MacRetval);
							if(Cl_MacRetval == CL_OK)
							{
						//		Cl_MacState = MAC_DPREP;
							}
							else
							{
								// handle error
							}
													
						break;
						case EVT_CONSOLE_COMMAND_CLR_ALARM:
						case EVT_CONSOLE_COMMAND_SET_BLDPMP_ON:
						case EVT_CONSOLE_COMMAND_SET_BLDPMP_OFF:
						case EVT_CONSOLE_COMMAND_SET_BLDPUMPRATE:
						case EVT_CONSOLE_COMMAND_HEPARIN_BOLUS:
						case EVT_CONSOLE_COMMAND_HEPARIN_START:
						case EVT_CONSOLE_COMMAND_HEPARIN_STOP:
						case EVT_CONSOLE_COMMAND_SET_BLDPUMPINC_MANUAL:
						case EVT_CONSOLE_COMMAND_SET_BLDPUMPDEC_MANUAL:
						Cl_MacRetval = Cl_Rinse_Controller(Cl_MacEvent);	
						ASSERT_ERROR(Cl_MacRetval);
							if(Cl_MacRetval == CL_OK)
							{
							//	Cl_MacState = MAC_DPREP;
							}
							else
							{
								// handle error
							}
																		
						break;
						default: break;
					}
					break;
					case MAC_DPREP:
					switch ( Cl_MacEvent )
						{
							
							case EVT_CONSOLE_COMMAND_ISOUF_START:
							
							Cl_MacState = MAC_ISOUF;
							Cl_MacRetval = cl_isouf_controller(Cl_MacEvent);
							
							
							break;
					//	case EVT_CONSOLE_COMMAND_SET_DATA:
					//	case EVT_CONSOLE_COMMAND_GET_DATA:
						case EVT_SEND_MIXING_PREP_START:
						case EVT_CONSOLE_COMMAND_CLR_ALARM:
						case EVT_CONSOLE_COMMAND_HEPARIN_START:
						case EVT_CONSOLE_COMMAND_HEPARIN_STOP: 
						case EVT_CONSOLE_COMMAND_HEP_PMP_RATE:
						case EVT_CONSOLE_COMMAND_SET_BLDPMP_ON:
						case EVT_CONSOLE_COMMAND_SET_BLDPMP_OFF:
						case EVT_CONSOLE_COMMAND_SET_BLDPUMPRATE:
						case EVT_CONSOLE_COMMAND_DIALYSIS_PREP:
						case EVT_CONSOLE_COMMAND_START_DIALISER_PRIME:
						case EVT_CONSOLE_COMMAND_STOP_DIALISER_PRIME:
						case EVT_CONSOLE_COMMAND_HEPARIN_BOLUS:
						case EVT_TICK_50M:
						case EVT_TICK_100M:
						case EVT_TICK_500M:
						case EVT_TICK_SEC:
						case EVT_TICK_MIN:
						case EVT_TICK_HOUR:
						case EVT_CONSOLE_COMMAND_START_DIALYSATE_FILLING:
						case EVT_ALERT_TRIGGERED:
						case EVT_ALARM_TRIGGERED:
						case EVT_CONSOLE_COMMAND_MUTE_ALARM:
						case EVT_CONSOLE_COMMAND_STOP_PRIME:
						case EVT_SEND_MIXING_PREP_STOP://0x22
						case EVT_CONSOLE_COMMAND_SET_BLDPUMPINC_MANUAL:
						case EVT_CONSOLE_COMMAND_SET_BLDPUMPDEC_MANUAL:
	
						
						Cl_MacRetval = Cl_dprep_controller(Cl_MacEvent);
						ASSERT_ERROR(Cl_MacRetval);
							if(Cl_MacRetval == CL_OK)
							{
								Cl_MacState = MAC_DPREP;
							}
							else
							{
								// handle error
							}
							
						break;
						case EVT_CONSOLE_COMMAND_START_PRIME:
						Cl_MacRetval = Cl_dprep_controller(Cl_MacEvent);
						ASSERT_ERROR(Cl_MacRetval);
							if(Cl_MacRetval == CL_OK)
							{
								Cl_MacState = MAC_DPREP;
							}
							else
							{
								// handle error
							}
						break;
						case EVT_CONSOLE_COMMAND_RCIRC_START:
						Cl_MacRetval = Cl_dprep_controller(Cl_MacEvent);
						ASSERT_ERROR(Cl_MacRetval);
							if(Cl_MacRetval == CL_OK)
							{
								Cl_MacState = MAC_DPREP;
							}
							else
							{
								// handle error
							}
						break;
						case EVT_CONSOLE_COMMAND_MAN_PREP_COMPLETED:
						Cl_MacRetval = Cl_dprep_controller(Cl_MacEvent);
						ASSERT_ERROR(Cl_MacRetval);
							if(Cl_MacRetval == CL_OK)
							{
								Cl_MacState = MAC_DPREP;
							}
							else
							{
								// handle error
							}
	
						//	Cl_MacState = MAC_POST_DPREP_STANDBY; // for now the prime is set after manual prwepartion is completed
							Cl_MacState = MAC_DPREP_PRIME;
						//	Cl_SendDatatoconsole(CON_TX_COMMAND_COMMAND_SYSTEM_STATE,(void*)MAC_DPREP_PRIME,1);
							ASSERT_ERROR(Cl_MacRetval);
						
						break;
						
						
						//Cl_MacRetval = Cl_dprep_controller(Cl_MacEvent);
						break;
						case MACREQ_DIALYSER_PRIME_COMPLETED:
						Cl_MacRetval = Cl_dprep_controller(Cl_MacEvent);
						ASSERT_ERROR(Cl_MacRetval);
						if(Cl_MacRetval == CL_OK)
						{
							Cl_MacState = MAC_POST_DPREP_STANDBY;
						//	Cl_SendDatatoconsole(CON_TX_COMMAND_COMMAND_SYSTEM_STATE,(void*)MAC_POST_DPREP_STANDBY,1);
							ASSERT_ERROR(Cl_MacRetval);
						}
						else
						{
							// handle error
						}
						break;
						
						case EVT_CONSOLE_COMMAND_DILYSER_CONNECTED:
						Cl_MacRetval = Cl_dprep_controller(Cl_MacEvent);
						ASSERT_ERROR(Cl_MacRetval);



						break;

						default: break;
					}
					break;
					case MAC_DPREP_PRIME:
					switch ( Cl_MacEvent )
				
					{
					//	case EVT_CONSOLE_COMMAND_SET_DATA:
					//	case EVT_CONSOLE_COMMAND_GET_DATA:
						case EVT_CONSOLE_COMMAND_HEPARIN_START:
						case EVT_CONSOLE_COMMAND_HEPARIN_STOP:
						case EVT_CONSOLE_COMMAND_HEPARIN_BOLUS:
						case EVT_CONSOLE_COMMAND_START_DIALYSATE_FILLING:
						case EVT_CONSOLE_COMMAND_DILYSER_CONNECTED:
						case EVT_CONSOLE_COMMAND_START_DIALISER_PRIME:
						case EVT_CONSOLE_COMMAND_STOP_DIALISER_PRIME:
						case EVT_CONSOLE_COMMAND_MAN_PREP_COMPLETED:
						case EVT_CONSOLE_COMMAND_SET_BLDPMP_ON:
						case EVT_CONSOLE_COMMAND_SET_BLDPMP_OFF:
						case EVT_CONSOLE_COMMAND_SET_BLDPUMPRATE:
						case EVT_TICK_500M:
						case EVT_TICK_SEC:
						case EVT_TICK_MIN:
						case EVT_TICK_HOUR:
						case EVT_ALERT_TRIGGERED:
						case EVT_ALARM_TRIGGERED:
						case EVT_CONSOLE_COMMAND_MUTE_ALARM:
						case EVT_CONSOLE_COMMAND_SET_BLDPUMPINC_MANUAL:
						case EVT_CONSOLE_COMMAND_SET_BLDPUMPDEC_MANUAL:
						Cl_MacRetval = Cl_dprep_controller(Cl_MacEvent);
						ASSERT_ERROR(Cl_MacRetval);
						if(Cl_MacRetval != CL_OK)
						{
							// handle error
						}
						break;
						case EVT_CONSOLE_COMMAND_START_PRIME:
						Cl_MacRetval = Cl_dprep_controller(Cl_MacEvent);
						ASSERT_ERROR(Cl_MacRetval);
							if(Cl_MacRetval == CL_OK)
							{
								Cl_MacState = 	MAC_DPREP_PRIME;
							//	Cl_SendDatatoconsole(CON_TX_COMMAND_COMMAND_SYSTEM_STATE,(void*)MAC_DPREP_PRIME,1);
								ASSERT_ERROR(Cl_MacRetval);
							}
							else
							{
								// handle error
							}

						break;

						

						case MACREQ_PRIME_COMPLETED:
						Cl_MacRetval = Cl_dprep_controller(Cl_MacEvent);
						ASSERT_ERROR(Cl_MacRetval);
							if(Cl_MacRetval != CL_OK)
							{
								// handle error
							}

						break;
						case MACREQ_DIALYSER_PRIME_COMPLETED:
						Cl_MacRetval = Cl_dprep_controller(Cl_MacEvent);
						ASSERT_ERROR(Cl_MacRetval);
							if(Cl_MacRetval == CL_OK)
							{
							Cl_MacState = MAC_POST_DPREP_STANDBY;
						//	Cl_SendDatatoconsole(CON_TX_COMMAND_COMMAND_SYSTEM_STATE,(void*)MAC_POST_DPREP_STANDBY,1);
							ASSERT_ERROR(Cl_MacRetval);
							}
							else
							{
								// handle error
							}

						break;
					

						default: break;
					}
					break;
					case MAC_SAFE_DPREP:
					switch ( Cl_MacEvent )
				
					{
						case EVT_CONSOLE_COMMAND_RINSE_START:
						break;
						case EVT_CONSOLE_COMMAND_DIALYSIS_PREP:
						break;
						default: break;
					}
					break;
					case MAC_POST_DPREP_STANDBY:
					switch ( Cl_MacEvent )
				
					{
						case EVT_CONSOLE_COMMAND_HEPARIN_START:
						case EVT_CONSOLE_COMMAND_HEPARIN_STOP:
						case EVT_CONSOLE_COMMAND_HEP_PMP_RATE:
						case EVT_CONSOLE_COMMAND_SET_BLDPMP_ON:
						case EVT_CONSOLE_COMMAND_SET_BLDPMP_OFF:
						case EVT_CONSOLE_COMMAND_SET_BLDPUMPRATE:
						case MACREQ_HEPARIN_BOLUS_COMPLETED:
						case EVT_ALERT_TRIGGERED:
						case EVT_ALARM_TRIGGERED:
						case EVT_CONSOLE_COMMAND_MUTE_ALARM:
						case EVT_TICK_SEC:
						case EVT_TICK_50M:
						case EVT_TICK_500M:
						case EVT_CONSOLE_COMMAND_PATIENT_CONNECTED:
						case EVT_CONSOLE_COMMAND_PATIENT_READY:
						case EVT_CONSOLE_COMMAND_BYPASS_ON:
						case EVT_CONSOLE_COMMAND_BYPASS_OFF:
						case EVT_CONSOLE_COMMAND_SET_BLDPUMPINC_MANUAL:
						case EVT_CONSOLE_COMMAND_SET_BLDPUMPDEC_MANUAL:
						Cl_MacRetval = Cl_dprep_controller(Cl_MacEvent);
						ASSERT_ERROR(Cl_MacRetval);
							if(Cl_MacRetval != CL_OK)
							{
								// handle error
							}
							//Cl_MacEvent = EVT_NULL;
						break;
						
						//case EVT_CONSOLE_COMMAND_SET_DATA:
						//case EVT_CONSOLE_COMMAND_GET_DATA:
					//	Cl_MacRetval = Cl_dprep_controller(Cl_MacEvent);
					//	ASSERT_ERROR(Cl_MacRetval);
					//	break;
						case EVT_CONSOLE_COMMAND_DIALYSIS_START:
// 							Cl_MacRetval = Cl_Standby_Controller(Cl_MacEvent);
// 							ASSERT_ERROR(Cl_MacRetval);
// 							if((Cl_MacRetval == CL_OK)||(Cl_MacRetval == CL_REJECTED))
// 							{
// 								Cl_MacRetval = Cl_Rinse_Controller(Cl_MacEvent);
// 								ASSERT_ERROR(Cl_MacRetval);
// 							}
// 							else
// 							{
// 								//handle error
// 							}
// 							if((Cl_MacRetval == CL_OK)||(Cl_MacRetval == CL_REJECTED))
// 							{
// 								Cl_MacRetval = Cl_dprep_controller(Cl_MacEvent);
// 								ASSERT_ERROR(Cl_MacRetval);
// 							}
// 							else
// 							{
// 								//handle error
// 							}
// 							if((Cl_MacRetval == CL_OK)||(Cl_MacRetval == CL_REJECTED))
//							{
								Cl_MacRetval = Cl_Dlsis_controller(Cl_MacEvent);
								ASSERT_ERROR(Cl_MacRetval);
//							}
//							else
//							{
								//handle error
//							}

						if(Cl_MacRetval == CL_OK)
						{
							Cl_MacState = MAC_DIALYSIS;
						//	Cl_SendDatatoconsole(CON_TX_COMMAND_COMMAND_SYSTEM_STATE,(void*)MAC_DIALYSIS,1);
							ASSERT_ERROR(Cl_MacRetval);
						}
						break;
						
						// add case for isouf start event
						case EVT_CONSOLE_COMMAND_ISOUF_START:
						
						Cl_MacState = MAC_ISOUF;
						Cl_MacRetval = cl_isouf_controller(Cl_MacEvent);
						
						
						break;
						
						default: break;
					}
					break;
					
					case MAC_ISOUF:
					
					switch(Cl_MacEvent){
					
					case EVT_CONSOLE_COMMAND_ISOUF_START:
							
							//Cl_MacState = MAC_ISOUF;
							Cl_MacRetval = cl_isouf_controller(Cl_MacEvent);
							
							
							break;
					
						case EVT_CONSOLE_COMMAND_CLR_ALARM:
						case EVT_CONSOLE_COMMAND_SET_BLDPMP_ON:
						case EVT_CONSOLE_COMMAND_SET_BLDPMP_OFF:
						case EVT_CONSOLE_COMMAND_SET_BLDPUMPRATE:
						case EVT_CONSOLE_COMMAND_HEPARIN_START:
						case EVT_CONSOLE_COMMAND_HEPARIN_STOP:
						case EVT_CONSOLE_COMMAND_HEP_PMP_RATE:
						case EVT_TICK_50M:
						case EVT_TICK_500M:
						case EVT_TICK_SEC:
						case EVT_TICK_MIN:
						case EVT_TICK_HOUR:
						case EVT_ALARM_TRIGGERED:
						case EVT_ALERT_TRIGGERED:
						case EVT_CONSOLE_COMMAND_MUTE_ALARM:
						case EVT_CONSOLE_COMMAND_BYPASS_ON:
						case EVT_CONSOLE_COMMAND_BYPASS_OFF:
						case EVT_CONSOLE_COMMAND_SET_BLDPUMPINC_MANUAL:
						case EVT_CONSOLE_COMMAND_SET_BLDPUMPDEC_MANUAL:
						//case EVT_CONSOLE_COMMAND_GET_DATA:
						//case EVT_CONSOLE_COMMAND_SET_DATA:
						Cl_MacRetval = cl_isouf_controller(Cl_MacEvent);
						ASSERT_ERROR(Cl_MacRetval);
						if(Cl_MacRetval != CL_OK)
						{
							// handle error
						}
						
						Cl_MacEvent = EVT_NULL;
						break;
						
						case MACREQ_ISOUF_COMPLETED:
						
						Cl_MacRetval = cl_isouf_controller(Cl_MacEvent);
						ASSERT_ERROR(Cl_MacRetval);
						if( Cl_MacRetval == CL_OK)
						{
							
						//	Cl_MacRetval = Cl_rinse_init();
						//	Cl_MacRetval = Cl_stby_init();
						//	Cl_MacRetval = Cl_dprep_init();
							Cl_MacRetval =Cl_isouf_init();
							
							Cl_MacState = MAC_POST_ISOUF_STANDBY;
						//	Cl_SendDatatoconsole(CON_TX_COMMAND_COMMAND_SYSTEM_STATE,(void*)Cl_MacState,1);
							ASSERT_ERROR(Cl_MacRetval);
						}
						else
						{
							//handle error
						}
						Cl_MacEvent = EVT_NULL;
						break;
					
						case EVT_CONSOLE_COMMAND_ISOUF_STOP:
						Cl_MacRetval = cl_isouf_controller(Cl_MacEvent);
						ASSERT_ERROR(Cl_MacRetval);
						break;
						
						case EVT_CONSOLE_COMMAND_DIALYSIS_START:
// 						Cl_MacRetval = Cl_Standby_Controller(Cl_MacEvent);
// 						ASSERT_ERROR(Cl_MacRetval);
// 						if((Cl_MacRetval == CL_OK)||(Cl_MacRetval == CL_REJECTED))
// 						{
// 							Cl_MacRetval = Cl_Rinse_Controller(Cl_MacEvent);
// 							ASSERT_ERROR(Cl_MacRetval);
// 						}
// 						else
// 						{
// 							//handle error
// 						}
// 						if((Cl_MacRetval == CL_OK)||(Cl_MacRetval == CL_REJECTED))
// 						{
// 							Cl_MacRetval = Cl_dprep_controller(Cl_MacEvent);
// 							ASSERT_ERROR(Cl_MacRetval);
// 						}
// 						else
// 						{
// 							//handle error
// 						}
// 						if((Cl_MacRetval == CL_OK)||(Cl_MacRetval == CL_REJECTED))
// 						{
							Cl_MacRetval = Cl_Dlsis_controller(Cl_MacEvent);
							ASSERT_ERROR(Cl_MacRetval);
//						}
//						else
						{
							//handle error
						}

						if(Cl_MacRetval == CL_OK)
						{
							Cl_MacState = MAC_DIALYSIS;
						//	Cl_SendDatatoconsole(CON_TX_COMMAND_COMMAND_SYSTEM_STATE,(void*)MAC_DIALYSIS,1);
							ASSERT_ERROR(Cl_MacRetval);
						}
						break;
						
						case EVT_CONSOLE_COMMAND_RINSE_START:
						loopcountinsec = 0;
		//				Cl_MacRetval = Cl_Standby_Controller(Cl_MacEvent);
		//				ASSERT_ERROR(Cl_MacRetval);

		//				if(Cl_MacRetval == CL_OK)
		//				{
							
							Cl_MacRetval = Cl_Rinse_Controller(Cl_MacEvent);
							if(Cl_MacRetval != CL_REJECTED)
							{
								ASSERT_ERROR(Cl_MacRetval);
							}
							
		//				}
						if( Cl_MacRetval == CL_OK)
						{
							Cl_MacState = MAC_RINSE;
						//	Cl_MacRetval = Cl_SendDatatoconsole(CON_TX_COMMAND_COMMAND_SYSTEM_STATE,(void*)MAC_RINSE,1);
							ASSERT_ERROR(Cl_MacRetval);
						}
						if( Cl_MacRetval != CL_OK)
						{
							// Handle error
						}
						Cl_MacEvent = EVT_NULL;
						break;

						
					}
					
					break;
					
					case MAC_POST_ISOUF_STANDBY:
					
					switch(Cl_MacEvent){
						
						case EVT_CONSOLE_COMMAND_CLR_ALARM:
						case EVT_CONSOLE_COMMAND_SET_BLDPMP_ON:
						case EVT_CONSOLE_COMMAND_SET_BLDPMP_OFF:
						case EVT_CONSOLE_COMMAND_SET_BLDPUMPRATE:
						case EVT_CONSOLE_COMMAND_HEPARIN_START:
						case EVT_CONSOLE_COMMAND_HEPARIN_STOP:
						case EVT_CONSOLE_COMMAND_HEP_PMP_RATE:
						case EVT_TICK_50M:
						case EVT_TICK_500M:
						case EVT_TICK_SEC:
						case EVT_TICK_MIN:
						case EVT_TICK_HOUR:
						case EVT_ALARM_TRIGGERED:
						case EVT_ALERT_TRIGGERED:
						case EVT_CONSOLE_COMMAND_MUTE_ALARM:
						case EVT_CONSOLE_COMMAND_SET_BLDPUMPINC_MANUAL:
						case EVT_CONSOLE_COMMAND_SET_BLDPUMPDEC_MANUAL:
						//case EVT_CONSOLE_COMMAND_GET_DATA:
						//case EVT_CONSOLE_COMMAND_SET_DATA:
						Cl_MacRetval = cl_isouf_controller(Cl_MacEvent);
						ASSERT_ERROR(Cl_MacRetval);
						if(Cl_MacRetval != CL_OK)
						{
							// handle error
						}
						
						Cl_MacEvent = EVT_NULL;
						break;
						
						case EVT_CONSOLE_COMMAND_DIALYSIS_START:
						/*
						Cl_MacRetval = Cl_Standby_Controller(Cl_MacEvent);
						ASSERT_ERROR(Cl_MacRetval);
						if((Cl_MacRetval == CL_OK)||(Cl_MacRetval == CL_REJECTED))
						{
							Cl_MacRetval = Cl_Rinse_Controller(Cl_MacEvent);
							ASSERT_ERROR(Cl_MacRetval);
						}
						else
						{
							//handle error
						}
						if((Cl_MacRetval == CL_OK)||(Cl_MacRetval == CL_REJECTED))
						{
							Cl_MacRetval = Cl_dprep_controller(Cl_MacEvent);
							ASSERT_ERROR(Cl_MacRetval);
						}
						else
						{
							//handle error
						}      */
					//	if((Cl_MacRetval == CL_OK)||(Cl_MacRetval == CL_REJECTED))
					//	{
							Cl_MacRetval = Cl_Dlsis_controller(Cl_MacEvent);
						//	ASSERT_ERROR(Cl_MacRetval);
					//	}
					//	else
					//	{
							//handle error
					//	}

						if(Cl_MacRetval == CL_OK)
						{
							Cl_MacState = MAC_DIALYSIS;
						//	Cl_SendDatatoconsole(CON_TX_COMMAND_COMMAND_SYSTEM_STATE,(void*)MAC_DIALYSIS,1);
							ASSERT_ERROR(Cl_MacRetval);
						}
						break;
						
						case EVT_CONSOLE_COMMAND_RINSE_START:
						//Cl_MacRetval = Cl_Standby_Controller(Cl_MacEvent);
						//ASSERT_ERROR(Cl_MacRetval);
						//if(Cl_MacRetval == CL_OK)
						//{
							Cl_rinse_init();
							Cl_MacRetval = Cl_Rinse_Controller(Cl_MacEvent);
							ASSERT_ERROR(Cl_MacRetval);
						//}
						//else
						//{
							//handle error
						//}
						
						if( Cl_MacRetval == CL_OK)
						{
							Cl_MacState = MAC_RINSE;
						//	Cl_SendDatatoconsole(CON_TX_COMMAND_COMMAND_SYSTEM_STATE,(void*)MAC_RINSE,1);
							ASSERT_ERROR(Cl_MacRetval);
							//Cl_MacState = MAC_POST_RINSE_STANDBY;
						}else
						{
							//handle error
						}
						break;
						
						case EVT_CONSOLE_COMMAND_ISOUF_START:
						
						Cl_MacState = MAC_ISOUF;
						Cl_MacRetval = cl_isouf_controller(Cl_MacEvent);
						
						
						break;
						
						
						default: break;
					}
					break;
					
					case MAC_DIALYSIS:
					switch ( Cl_MacEvent )
				
					{
						case EVT_CONSOLE_COMMAND_CLR_ALARM:
						case EVT_CONSOLE_COMMAND_SET_BLDPMP_ON:
						case EVT_CONSOLE_COMMAND_SET_BLDPMP_OFF:
						case EVT_CONSOLE_COMMAND_SET_BLDPUMPRATE:
						case EVT_CONSOLE_COMMAND_HEPARIN_START:
						case EVT_CONSOLE_COMMAND_HEPARIN_STOP:
						case EVT_CONSOLE_COMMAND_HEP_PMP_RATE:
						case EVT_CONSOLE_COMMAND_HEPARIN_BOLUS:
						case EVT_TICK_50M:
						case EVT_TICK_500M:
						case EVT_TICK_SEC:
						case EVT_TICK_MIN:
						case EVT_TICK_HOUR:
						case EVT_ALARM_TRIGGERED:
						case EVT_ALERT_TRIGGERED:
						case EVT_CONSOLE_COMMAND_MUTE_ALARM:
						case EVT_CONSOLE_COMMAND_DIALYSIS_PAUSE:
						case EVT_CONSOLE_COMMAND_BYPASS_ON:
						case EVT_CONSOLE_COMMAND_BYPASS_OFF:
						case EVT_CONSOLE_COMMAND_SET_BLDPUMPINC_MANUAL:
						case EVT_CONSOLE_COMMAND_SET_BLDPUMPDEC_MANUAL:
						case EVT_CONSOLE_COMMAND_DIALYSIS_ISOUF_ON:
						case EVT_CONSOLE_COMMAND_DIALYSIS_ISOUF_OFF:
					//	case EVT_CONSOLE_COMMAND_GET_DATA:
					//	case EVT_CONSOLE_COMMAND_SET_DATA:
							Cl_MacRetval = Cl_Dlsis_controller(Cl_MacEvent);
							ASSERT_ERROR(Cl_MacRetval);
								if(Cl_MacRetval != CL_OK)
								{
									// handle error
								}
							
							Cl_MacEvent = EVT_NULL;
						break;
						//case EVT_CONSOLE_COMMAND_SET_BLDPMP_ON:
						//case EVT_CONSOLE_COMMAND_SET_BLDPMP_OFF:
						//case EVT_CONSOLE_COMMAND_SET_BLDPUMPRATE:
						
// 						case EVT_CONSOLE_COMMAND_DIALYSIS_PAUSE:
// 						case EVT_CONSOLE_COMMAND_BYPASS_ON:
// 						case EVT_CONSOLE_COMMAND_BYPASS_OFF:
// 						Cl_MacRetval = Cl_Dlsis_controller(Cl_MacEvent);
// 						ASSERT_ERROR(Cl_MacRetval);
// 							if( Cl_MacRetval == CL_OK)
// 							{
// 								Cl_MacState = MAC_SAFE_DIALYSIS;
// 								Cl_SendDatatoconsole(CON_TX_COMMAND_COMMAND_SYSTEM_STATE,(void*)MAC_SAFE_DIALYSIS,1);
// 								ASSERT_ERROR(Cl_MacRetval);
// 							}
// 							else
// 							{
// 								// handle error
// 							}
// 						break;
						case EVT_CONSOLE_COMMAND_DIALYSIS_STOP:
						Cl_MacRetval = Cl_Dlsis_controller(Cl_MacEvent);
						ASSERT_ERROR(Cl_MacRetval);
						break;
						#if 0
						case EVT_CONSOLE_COMMAND_DIALYSIS_START:
							Cl_MacRetval = Cl_Standby_Controller(Cl_MacEvent);
							ASSERT_ERROR(Cl_MacRetval);
							if((Cl_MacRetval == CL_OK)||(Cl_MacRetval == CL_REJECTED))
							{
								Cl_MacRetval = Cl_Rinse_Controller(Cl_MacEvent);
								ASSERT_ERROR(Cl_MacRetval);
							}
							else
							{
								// handle error
							}
							if((Cl_MacRetval == CL_OK)||(Cl_MacRetval == CL_REJECTED))
							{
								Cl_MacRetval = Cl_dprep_controller(Cl_MacEvent);
								ASSERT_ERROR(Cl_MacRetval);
							}
							else
							{
								// handle error
							}
							if((Cl_MacRetval == CL_OK)||(Cl_MacRetval == CL_REJECTED))
							{
								Cl_MacRetval = Cl_Dlsis_controller(Cl_MacEvent);
								ASSERT_ERROR(Cl_MacRetval);
							}
							else
							{
								// handle error
							}

						if(Cl_MacRetval == CL_OK)
						{
							Cl_MacState = MAC_DIALYSIS;
								Cl_SendDatatoconsole(CON_TX_COMMAND_COMMAND_SYSTEM_STATE,(void*)Cl_MacState,1);
								ASSERT_ERROR(Cl_MacRetval);
						}else
						{
							// handle error
						}
						break;
						#endif
						
						case EVT_CONSOLE_COMMAND_ISOUF_START:
						
						Cl_MacState = MAC_ISOUF;
						Cl_MacRetval = cl_isouf_controller(Cl_MacEvent);
						
						
						break;
						
						
						case MACREQ_POST_DIALYSIS_ISOUF_REQ :
						Cl_MacState = MAC_ISOUF;
						Cl_MacRetval = cl_isouf_controller(Cl_MacEvent);
						
						break;
					/*	case EVT_CONSOLE_COMMAND_RINSE_START:
							Cl_MacRetval = Cl_Standby_Controller(Cl_MacEvent);
							ASSERT_ERROR(Cl_MacRetval);
							if(Cl_MacRetval == CL_OK)
							{
								Cl_MacRetval = Cl_Rinse_Controller(Cl_MacEvent);
								ASSERT_ERROR(Cl_MacRetval);
							}
							else
							{
								//handle error
							}
							
							if( Cl_MacRetval == CL_OK)
							{
								Cl_MacState = MAC_RINSE;
							//	Cl_SendDatatoconsole(CON_TX_COMMAND_COMMAND_SYSTEM_STATE,(void*)MAC_RINSE,1);
								ASSERT_ERROR(Cl_MacRetval);
								//Cl_MacState = MAC_POST_RINSE_STANDBY;
							}else
							{
								//handle error
							}
						break;
		*/
						case MACREQ_DIALYSIS_COMPLETED:
						
							Cl_MacRetval = Cl_Dlsis_controller(Cl_MacEvent);
							ASSERT_ERROR(Cl_MacRetval);
							if( Cl_MacRetval == CL_OK)
							{
							
								Cl_MacRetval = Cl_rinse_init();
								Cl_MacRetval = Cl_stby_init();
								Cl_MacRetval = Cl_dprep_init();
								
								
								Cl_MacState = MAC_POST_DIALYSIS_STANDBY;
							//		Cl_SendDatatoconsole(CON_TX_COMMAND_COMMAND_SYSTEM_STATE,(void*)Cl_MacState,1);
									ASSERT_ERROR(Cl_MacRetval);
							}
							else
							{
								//handle error
							}
							Cl_MacEvent = EVT_NULL;
						break;
						default: break;
					}
					break;
					case MAC_SAFE_DIALYSIS:
					switch ( Cl_MacEvent )
				
					{	
						case EVT_CONSOLE_COMMAND_SET_BLDPMP_ON:
						case EVT_CONSOLE_COMMAND_SET_BLDPMP_OFF:
						case EVT_CONSOLE_COMMAND_SET_BLDPUMPRATE:
						case EVT_CONSOLE_COMMAND_HEPARIN_START:
						case EVT_CONSOLE_COMMAND_HEPARIN_STOP:
						case EVT_CONSOLE_COMMAND_SET_BLDPUMPINC_MANUAL:
						case EVT_CONSOLE_COMMAND_SET_BLDPUMPDEC_MANUAL:
						Cl_MacRetval = Cl_Standby_Controller(Cl_MacEvent);
						break;
						case EVT_CONSOLE_COMMAND_RINSE_START:
							Cl_MacRetval = Cl_Standby_Controller(Cl_MacEvent);
							ASSERT_ERROR(Cl_MacRetval);
							if(Cl_MacRetval == CL_OK)
							{
								Cl_MacRetval = Cl_Rinse_Controller(Cl_MacEvent);	
								ASSERT_ERROR(Cl_MacRetval);
							}
							else
							{
								//handle error
							}
							if( Cl_MacRetval == CL_OK)
							{
								Cl_MacState = MAC_RINSE;
								//	Cl_SendDatatoconsole(CON_TX_COMMAND_COMMAND_SYSTEM_STATE,(void*)Cl_MacState,1);
									ASSERT_ERROR(Cl_MacRetval);
								//Cl_MacState = MAC_POST_RINSE_STANDBY;
							}
							else
							{
								//handle error
							}
							Cl_MacEvent = EVT_NULL;
						break;
					//	case EVT_CONSOLE_COMMAND_GET_DATA:
					//	case EVT_CONSOLE_COMMAND_SET_DATA:
					//		Cl_MacRetval = Cl_Dlsis_controller(Cl_MacEvent);
						//	Cl_MacEvent = EVT_NULL;
						//	break;
						case EVT_CONSOLE_COMMAND_BYPASS_ON:
						case EVT_CONSOLE_COMMAND_BYPASS_OFF:
						Cl_MacRetval = Cl_Dlsis_controller(Cl_MacEvent);
						ASSERT_ERROR(Cl_MacRetval);
								if( Cl_MacRetval == CL_OK)
								{
									Cl_MacState = MAC_DIALYSIS;
								}
								else
								{
									//handle error
								}
						
						break;
						#if 0
						case EVT_CONSOLE_COMMAND_DIALYSIS_START:
							Cl_MacRetval = Cl_Standby_Controller(Cl_MacEvent);
							ASSERT_ERROR(Cl_MacRetval);
							if((Cl_MacRetval == CL_OK)||(Cl_MacRetval == CL_REJECTED))
							{
								Cl_MacRetval = Cl_Rinse_Controller(Cl_MacEvent);
								ASSERT_ERROR(Cl_MacRetval);
							}
							else
							{
								//handle error
							}
							if((Cl_MacRetval == CL_OK)||(Cl_MacRetval == CL_REJECTED))
							{
								Cl_MacRetval = Cl_dprep_controller(Cl_MacEvent);
								ASSERT_ERROR(Cl_MacRetval);
							}
							else
							{
								//handle error
							}
							if((Cl_MacRetval == CL_OK)||(Cl_MacRetval == CL_REJECTED))
							{
								Cl_MacRetval = Cl_Dlsis_controller(Cl_MacEvent);
								ASSERT_ERROR(Cl_MacRetval);
							}
							else
							{
								//handle error
							}

						if(Cl_MacRetval == CL_OK)
						{
							Cl_MacState = MAC_DIALYSIS;
								Cl_SendDatatoconsole(CON_TX_COMMAND_COMMAND_SYSTEM_STATE,(void*)MAC_DIALYSIS,1);
								ASSERT_ERROR(Cl_MacRetval);
						}else
						{
							//handle error
						}
						break;
						#endif
						default: break;
					}
					break;
					case MAC_POST_DIALYSIS_STANDBY:
					switch ( Cl_MacEvent )
				
					{
						case EVT_CONSOLE_COMMAND_SET_BLDPMP_ON:
						case EVT_CONSOLE_COMMAND_SET_BLDPMP_OFF:
						case EVT_CONSOLE_COMMAND_SET_BLDPUMPRATE:
						case EVT_CONSOLE_COMMAND_HEPARIN_START:
						case EVT_CONSOLE_COMMAND_HEPARIN_STOP:
						case EVT_CONSOLE_COMMAND_SET_BLDPUMPINC_MANUAL:
						case EVT_CONSOLE_COMMAND_SET_BLDPUMPDEC_MANUAL:
						case EVT_CONSOLE_COMMAND_BYPASS_ON:
						case EVT_CONSOLE_COMMAND_BYPASS_OFF:
						Cl_MacRetval = Cl_Dlsis_controller(Cl_MacEvent);
						ASSERT_ERROR(Cl_MacRetval);
							if( Cl_MacRetval != CL_OK)
							{
								//handle error
							}
						break;
						case EVT_CONSOLE_COMMAND_DISINF_START:
						
						Cl_MacRetval = Cl_Dlsis_controller(Cl_MacEvent);
						ASSERT_ERROR(Cl_MacRetval);
							if( Cl_MacRetval != CL_OK)
							{
								//handle error
							}
						Cl_MacRetval = Cl_disinf_controller(Cl_MacEvent);
						ASSERT_ERROR(Cl_MacRetval);
							if( Cl_MacRetval != CL_OK)
							{
								//handle error
							}
						
						Cl_MacState = MAC_DISINFECT;
						
						break;
					//	case EVT_CONSOLE_COMMAND_GET_DATA:
					//	case EVT_CONSOLE_COMMAND_SET_DATA:
					//		Cl_MacRetval = Cl_Dlsis_controller(Cl_MacEvent);
					//		Cl_MacEvent = EVT_NULL;
					//		break;
						case EVT_CONSOLE_COMMAND_RINSE_START:
							Cl_MacRetval = Cl_Standby_Controller(Cl_MacEvent);
							ASSERT_ERROR(Cl_MacRetval);
							if(Cl_MacRetval == CL_OK)
							{
								Cl_MacRetval = Cl_Rinse_Controller(Cl_MacEvent);
								ASSERT_ERROR(Cl_MacRetval);	
							}
							else
							{
								//handle error
							}
							
							if( Cl_MacRetval == CL_OK)
							{
								Cl_MacState = MAC_RINSE;
								//	Cl_SendDatatoconsole(CON_TX_COMMAND_COMMAND_SYSTEM_STATE,(void*)MAC_RINSE,1);
									ASSERT_ERROR(Cl_MacRetval);
								//Cl_MacState = MAC_POST_RINSE_STANDBY;
							}else
							{
								//handle error
							}
							Cl_MacEvent = EVT_NULL;
						break;
						

						default: break;
					}
					break;
					case MAC_SERVICE:
					switch ( Cl_MacEvent )
				
					{

						default: break;
					}
					break;
					case MAC_STATE_MAX:
					switch ( Cl_MacEvent )
				
					{

						default: break;
					}
					break;
					default :
					break;
				}
				

			
		
				Cl_MacRetval = Cl_Mac_ClearEvents();
				ASSERT_ERROR(Cl_MacRetval);
					if( Cl_MacRetval != CL_OK)
					{
						//handle error
					}
		}
		else
		{
			
		}
		
		}
		
		//sv_cntrl_resetyellowalarm();

	}

//#endif
}


/*
 * \brief Initialize theM MAC controller data structures and queues
 * \void function 
 * \return CL_OK on success.
 *
 * \Globals used : Cl_MacEventQueue1
 */
Cl_ReturnCodeType 	Cl_MacInit(void) 
{
		Cl_ReturnCodeType Cl_MacRetval = CL_OK;
		
		queue_sequence_counter = 0;
		
		Cl_MacEventQueue1.Cl_MacEvent[0] =0;
		Cl_MacEventQueue1.Cl_MacNewEvent = false;
		Cl_MacEventQueue1.Cl_MacNewEventcount = 0;
		
		Cl_MacEventQueue4.Cl_MacEvent[0] =0;
		Cl_MacEventQueue4.Cl_MacNewEvent = false;
		Cl_MacEventQueue4.Cl_MacNewEventcount = 0;
		
		Cl_MacEventQueueBc.Cl_MacEvent[0] =0;
		Cl_MacEventQueueBc.Cl_MacNewEvent = false;
		Cl_MacEventQueueBc.Cl_MacNewEventcount = 0;
		
		
		Cl_Console_buffer.BufferSize = CONSOLE_BUFFER_SIZE;
		Cl_Console_msgbuffer.BufferSize = CONSOLE_MSG_BUFFER_SIZE;
		Cl_Console_buffer1.BufferEmpty = true;
		Cl_Console_buffer1.ConsoleDatabufferprocessed= false;
		Cl_Console_buffer1.ConsoleDatabufferReady=false;
		Cl_Console_buffer1.CurreltyActive=true;
		Cl_Console_buffer1.StartSequenceDetected=false;

		Cl_Console_buffer2.BufferEmpty = true;
		Cl_Console_buffer2.ConsoleDatabufferprocessed= false;
		Cl_Console_buffer2.ConsoleDatabufferReady=false;
		Cl_Console_buffer2.CurreltyActive=false;
		Cl_Console_buffer2.StartSequenceDetected=false;
		
		Cl_MacState =	MAC_INIT;
		
		if( Cl_MacRetval != CL_OK)
		{
			//handle error
		}
		ASSERT_ERROR(Cl_MacRetval);
		return Cl_MacRetval;
}
/*
 * \brief Initialize theM Application Modules  
 * \void function 
 * \return CL_OK on success.
 * \Globals used : 
 */
Cl_ReturnCodeType  Cl_App_init(void)
{
	Cl_ReturnCodeType  Cl_RetValue = CL_ERROR;
	
	Cl_RetValue = Cl_Init_Alarms();
	ASSERT_ERROR(Cl_RetValue);
			if( Cl_RetValue != CL_OK)
			{
				//handle error
			}
	Cl_RetValue = Cl_Console_init();
	ASSERT_ERROR(Cl_RetValue);
			if( Cl_RetValue != CL_OK)
			{
				//handle error
			}
	Cl_RetValue = Cl_disinf_init();
	ASSERT_ERROR(Cl_RetValue);
			if( Cl_RetValue != CL_OK)
			{
				//handle error
			}
	Cl_RetValue = Cl_dlsis_init();
	ASSERT_ERROR(Cl_RetValue);
			if( Cl_RetValue != CL_OK)
			{
				//handle error
			}
	Cl_RetValue = Cl_dprep_init(	);
	ASSERT_ERROR(Cl_RetValue);
			if( Cl_RetValue != CL_OK)
			{
				//handle error
			}
	Cl_RetValue = Cl_rinse_init();
	ASSERT_ERROR(Cl_RetValue);
			if( Cl_RetValue != CL_OK)
			{
				//handle error
			}
	Cl_RetValue = Cl_stby_init();
	ASSERT_ERROR(Cl_RetValue);
			if( Cl_RetValue != CL_OK)
			{
				//handle error
			}
	
	Cl_RetValue = cl_datamanager_init();
	ASSERT_ERROR(Cl_RetValue);
			if( Cl_RetValue != CL_OK)
			{
				//handle error
			}
	
	return CL_OK;
	
}
/*
 * \brief Configure MAC Module  
 * \void function 
 * \return CL_OK on success.
 * \Globals used : 
 */
Cl_ReturnCodeType 	Cl_Mac_Configure(void)

{ 
	Cl_ReturnCodeType Cl_Mac_retval = CL_ERROR;
	
	/*printf("System Configuring Controls\n");*/
	ASSERT_ERROR(Cl_Mac_retval);
			if( Cl_Mac_retval != CL_OK)
			{
				//handle error
			}
	
	return CL_OK;
}
/*
 * \brief Check if timer is expired  
 * \void function 
 * \return CL_OK on success.
 * \Globals used : 
 */
Cl_ReturnCodeType 	Cl_MacCheck_TimerExpired(void)
{
	Cl_ReturnCodeType Cl_Mac_retval = CL_ERROR;
			if( Cl_Mac_retval != CL_OK)
			{
				//handle error
			}
			ASSERT_ERROR(Cl_Mac_retval);
	return CL_OK;
}


/*
 * \brief  query for  the system status 
 * \void function 
 * \return CL_OK on success.
 * \Globals used : 
 */

Cl_ReturnCodeType 	Cl_Mac_GetSystem_Status_Query(void)
{
	Cl_ReturnCodeType Cl_MacRetval= CL_ERROR;
	Cl_MacRetval =	Cl_SysStat_System_Status_Query();
	ASSERT_ERROR(Cl_MacRetval);
	
				if( Cl_MacRetval != CL_OK)
				{
					//handle error
				}
	
	return CL_OK;
}
/*
 * \brief  check console message has arrived or not 
			checks the ready status and construct the message and add to the queue in buffer
 * \void function 
 * \return CL_OK on success.
 * \Globals used : 
 */

Cl_ReturnCodeType Cl_MacCall_Newappreq_Query(void)
{
		Cl_ReturnCodeType Cl_MacRetval= CL_ERROR;
		Cl_MacRetval =	Cl_SysStat_System_Status_Query();
		ASSERT_ERROR(Cl_MacRetval);
						if( Cl_MacRetval != CL_OK)
						{
							//handle error
						}
		return CL_OK;
	
}
Cl_ReturnCodeType 	Cl_MacCheckforConsoleMessage(void)
{
			Cl_ReturnCodeType Cl_MacRetval1;
			
			Cl_MacRetval1 = Cl_Console_Read_ConsoleMessage_Status(); // Reads the status of the data buffer populated by the timer console read interrupt.
			ASSERT_ERROR(Cl_MacRetval1);
			if(Cl_MacRetval1 == CL_OK)
			{
		
				Cl_MacRetval1 = Cl_Console_Read_Message_Console();
				ASSERT_ERROR(Cl_MacRetval1);
				
			} else 
			
			{ // LOG ERROR
			}
			if(Cl_MacRetval1 == CL_OK)
			{
			Cl_MacRetval1 = Cl_Console_Read_Message_Ready_Status(); // Reads the status of the data buffer populated by the timer console read interrupt.
			ASSERT_ERROR(Cl_MacRetval1);
			} 
			else { //LOG ERROR
			}
			
			if(Cl_MacRetval1== CL_OK)
			{
		
				Cl_MacRetval1 = Cl_Mac_Create_ConsoleEvent(); // Create tyhe event and put it in the queue
				ASSERT_ERROR(Cl_MacRetval1);
				
			} 
			else {// LOG ERROR
			}
			
			return CL_OK; 
}
/*
 * \brief  check if any pending event is posted to MAC or not
			if so  read it from the buffer and shift the buffer
 * \void function 
 * \return CL_OK on success.
 * \Globals used : 
 */


Cl_ReturnCodeType	Cl_MacCreate_Macevent(void)
{
	Cl_ReturnCodeType Cl_Mac_retVal = CL_OK;
	Cl_Uint8Type tempcount;
	
	Cl_MacNewEvent  = false;
	Cl_MacEvent = EVT_NULL;
	
	queue_sequence_counter++;
	if(queue_sequence_counter > 5){
		
		queue_sequence_counter = 1;
	}
	
	switch(queue_sequence_counter){
		
		case 1 : // bc switching
		
		if ((Cl_MacEventQueueBc.Cl_MacNewEvent) && (Cl_MacEventQueueBc.Cl_MacNewEventcount > 0))
		{
			

			Cl_MacNewEvent = true;
			Cl_MacEvent = Cl_MacEventQueueBc.Cl_MacEvent[0];
			Cl_MacEventQueueBc.Cl_MacNewEventcount--;
			
			for (tempcount =0 ;tempcount < Cl_MacEventQueueBc.Cl_MacNewEventcount;tempcount++ )
			{
				if(tempcount < MAC_EVENT_COUNT_MAX)
				{
					Cl_MacEventQueueBc.Cl_MacEvent[tempcount] = Cl_MacEventQueueBc.Cl_MacEvent[tempcount+1];
					

					
				}
				else
				{
					if(tempcount >= MAC_EVENT_COUNT_MAX)
					{
						Cl_MacEventQueueBc.Cl_MacEvent[tempcount] = EVT_NULL;
					}
					// handle error, raise alarm. system stops
					break; //handle exception
				}
			}
			
			
		}
		else{
			Cl_MacNewEvent = false;
			Cl_MacEvent = EVT_NULL;
		}
		
		break;
		
		case 2 : // serial data
		
		if ((Cl_MacEventQueue3.Cl_MacNewEvent) && (Cl_MacEventQueue3.Cl_MacNewEventcount > 0))
		{
	

			Cl_MacNewEvent = true;
			Cl_MacEvent = Cl_MacEventQueue3.Cl_MacEvent[0];
			Cl_MacEventQueue3.Cl_MacNewEventcount--;
	
			for (tempcount =0 ;tempcount < Cl_MacEventQueue3.Cl_MacNewEventcount;tempcount++ )
			{
				if(tempcount < MAC_EVENT_COUNT_MAX)
				{
					Cl_MacEventQueue3.Cl_MacEvent[tempcount] = Cl_MacEventQueue3.Cl_MacEvent[tempcount+1];
			

			
				}
				else
				{
					if(tempcount >= MAC_EVENT_COUNT_MAX)
					{
						Cl_MacEventQueue3.Cl_MacEvent[tempcount] = EVT_NULL;
					}
					// handle error, raise alarm. system stops
					break; //handle exception
				}
			}
	
	
		}
	else{
		Cl_MacNewEvent = false;
			Cl_MacEvent = EVT_NULL;
	}
		break;
		case 3: // alarm
		
		 if ((Cl_MacEventQueue4.Cl_MacNewEvent) && (Cl_MacEventQueue4.Cl_MacNewEventcount > 0))
		{
			

			Cl_MacNewEvent = true;
			Cl_MacEvent = Cl_MacEventQueue4.Cl_MacEvent[0];
			Cl_MacEventQueue4.Cl_MacNewEventcount--;
			
			for (tempcount =0 ;tempcount < Cl_MacEventQueue4.Cl_MacNewEventcount;tempcount++ )
			{
				if(tempcount < MAC_EVENT_COUNT_MAX)
				{
					Cl_MacEventQueue4.Cl_MacEvent[tempcount] = Cl_MacEventQueue4.Cl_MacEvent[tempcount+1];
					

					
				}
				else
				{
					if(tempcount >= MAC_EVENT_COUNT_MAX)
					{
						Cl_MacEventQueue4.Cl_MacEvent[tempcount] = EVT_NULL;
					}
					// handle error, raise alarm. system stops
					break; //handle exception
				}
			}
			
			
		}
		else{
		Cl_MacNewEvent = false;
			Cl_MacEvent = EVT_NULL;
	}
		
		break;
		case 4 : // internal state change
		 if ((Cl_MacEventQueue1.Cl_MacNewEvent) && (Cl_MacEventQueue1.Cl_MacNewEventcount > 0))
			{
				

				Cl_MacNewEvent = true;
				Cl_MacEvent = Cl_MacEventQueue1.Cl_MacEvent[0];
				Cl_MacEventQueue1.Cl_MacNewEventcount--;
				
				for (tempcount =0 ;tempcount < Cl_MacEventQueue1.Cl_MacNewEventcount;tempcount++ )
				{
					if(tempcount < MAC_EVENT_COUNT_MAX)
					{
						Cl_MacEventQueue1.Cl_MacEvent[tempcount] = Cl_MacEventQueue1.Cl_MacEvent[tempcount+1];
						

						
					}
					else
					{
						if(tempcount >= MAC_EVENT_COUNT_MAX)
						{
							Cl_MacEventQueue1.Cl_MacEvent[tempcount] = EVT_NULL;
						}
						// handle error, raise alarm. system stops
						break; //handle exception
					}
				}
				
				
			}
			else{
		Cl_MacNewEvent = false;
			Cl_MacEvent = EVT_NULL;
	}
			
		break;
		case 5 : // timer events
		if ((Cl_MacEventQueue2.Cl_MacNewEvent) && (Cl_MacEventQueue2.Cl_MacNewEventcount > 0))
		{
			

			Cl_MacNewEvent = true;
			Cl_MacEvent = Cl_MacEventQueue2.Cl_MacEvent[0];
			Cl_MacEventQueue2.Cl_MacNewEventcount--;
			
			for (tempcount =0 ;tempcount < Cl_MacEventQueue2.Cl_MacNewEventcount;tempcount++ )
			{
				if(tempcount < MAC_EVENT_COUNT_MAX)
				{
					Cl_MacEventQueue2.Cl_MacEvent[tempcount] = Cl_MacEventQueue2.Cl_MacEvent[tempcount+1];
					

					
				}
				else
				{
					if(tempcount >= MAC_EVENT_COUNT_MAX)
					{
						Cl_MacEventQueue2.Cl_MacEvent[tempcount] = EVT_NULL;
					}
					// handle error, raise alarm. system stops
					break; //handle exception
				}
			}
			Cl_MacEventQueue2.Cl_MacEvent[Cl_MacEventQueue2.Cl_MacNewEventcount] = 0;
			
			
		}
		else
		{
			Cl_MacNewEvent = false;
			Cl_MacEvent = EVT_NULL;
			
		}
		break;
		
		
		
	}
	
	
	
	
/*	
	
	if ((Cl_MacEventQueueBc.Cl_MacNewEvent) && (Cl_MacEventQueueBc.Cl_MacNewEventcount > 0))
	{
		

		Cl_MacNewEvent = true;
		Cl_MacEvent = Cl_MacEventQueueBc.Cl_MacEvent[0];
		Cl_MacEventQueueBc.Cl_MacNewEventcount--;
		
		for (tempcount =0 ;tempcount < Cl_MacEventQueueBc.Cl_MacNewEventcount;tempcount++ )
		{
			if(tempcount < MAC_EVENT_COUNT_MAX)
			{
				Cl_MacEventQueueBc.Cl_MacEvent[tempcount] = Cl_MacEventQueueBc.Cl_MacEvent[tempcount+1];
				

				
			}
			else
			{
				if(tempcount >= MAC_EVENT_COUNT_MAX)
				{
					Cl_MacEventQueueBc.Cl_MacEvent[tempcount] = EVT_NULL;
				}
				// handle error, raise alarm. system stops
				break; //handle exception
			}
		}
		
		
	}
	
	
	
		else if ((Cl_MacEventQueue3.Cl_MacNewEvent) && (Cl_MacEventQueue3.Cl_MacNewEventcount > 0))
		{
	

			Cl_MacNewEvent = true;
			Cl_MacEvent = Cl_MacEventQueue3.Cl_MacEvent[0];
			Cl_MacEventQueue3.Cl_MacNewEventcount--;
	
			for (tempcount =0 ;tempcount < Cl_MacEventQueue3.Cl_MacNewEventcount;tempcount++ )
			{
				if(tempcount < MAC_EVENT_COUNT_MAX)
				{
					Cl_MacEventQueue3.Cl_MacEvent[tempcount] = Cl_MacEventQueue3.Cl_MacEvent[tempcount+1];
			

			
				}
				else
				{
					if(tempcount >= MAC_EVENT_COUNT_MAX)
					{
						Cl_MacEventQueue3.Cl_MacEvent[tempcount] = EVT_NULL;
					}
					// handle error, raise alarm. system stops
					break; //handle exception
				}
			}
	
	
		}
		
			else if ((Cl_MacEventQueue1.Cl_MacNewEvent) && (Cl_MacEventQueue1.Cl_MacNewEventcount > 0))
			{
				

				Cl_MacNewEvent = true;
				Cl_MacEvent = Cl_MacEventQueue1.Cl_MacEvent[0];
				Cl_MacEventQueue1.Cl_MacNewEventcount--;
				
				for (tempcount =0 ;tempcount < Cl_MacEventQueue1.Cl_MacNewEventcount;tempcount++ )
				{
					if(tempcount < MAC_EVENT_COUNT_MAX)
					{
						Cl_MacEventQueue1.Cl_MacEvent[tempcount] = Cl_MacEventQueue1.Cl_MacEvent[tempcount+1];
						

						
					}
					else
					{
						if(tempcount >= MAC_EVENT_COUNT_MAX)
						{
							Cl_MacEventQueue1.Cl_MacEvent[tempcount] = EVT_NULL;
						}
						// handle error, raise alarm. system stops
						break; //handle exception
					}
				}
				
				
			}
		else if ((Cl_MacEventQueue4.Cl_MacNewEvent) && (Cl_MacEventQueue4.Cl_MacNewEventcount > 0))
		{
			

			Cl_MacNewEvent = true;
			Cl_MacEvent = Cl_MacEventQueue4.Cl_MacEvent[0];
			Cl_MacEventQueue4.Cl_MacNewEventcount--;
			
			for (tempcount =0 ;tempcount < Cl_MacEventQueue4.Cl_MacNewEventcount;tempcount++ )
			{
				if(tempcount < MAC_EVENT_COUNT_MAX)
				{
					Cl_MacEventQueue4.Cl_MacEvent[tempcount] = Cl_MacEventQueue4.Cl_MacEvent[tempcount+1];
					

					
				}
				else
				{
					if(tempcount >= MAC_EVENT_COUNT_MAX)
					{
						Cl_MacEventQueue4.Cl_MacEvent[tempcount] = EVT_NULL;
					}
					// handle error, raise alarm. system stops
					break; //handle exception
				}
			}
			
			
		}
		
		else if ((Cl_MacEventQueue2.Cl_MacNewEvent) && (Cl_MacEventQueue2.Cl_MacNewEventcount > 0))
		{
			

			Cl_MacNewEvent = true;
			Cl_MacEvent = Cl_MacEventQueue2.Cl_MacEvent[0];
			Cl_MacEventQueue2.Cl_MacNewEventcount--;
			
			for (tempcount =0 ;tempcount < Cl_MacEventQueue2.Cl_MacNewEventcount;tempcount++ )
			{
				if(tempcount < MAC_EVENT_COUNT_MAX)
				{
					Cl_MacEventQueue2.Cl_MacEvent[tempcount] = Cl_MacEventQueue2.Cl_MacEvent[tempcount+1];
					

					
				}
				else
				{
					if(tempcount >= MAC_EVENT_COUNT_MAX)
					{
						Cl_MacEventQueue2.Cl_MacEvent[tempcount] = EVT_NULL;
					}
					// handle error, raise alarm. system stops
					break; //handle exception
				}
			}
			Cl_MacEventQueue2.Cl_MacEvent[Cl_MacEventQueue2.Cl_MacNewEventcount] = 0;
			
			
		}

	
		
		else
		{
			Cl_MacNewEvent = false;
			Cl_MacEvent = EVT_NULL;
			
			
			
		}
	*/

	return CL_OK;
}


/*
 * \brief  clear all events if  already handled
			
 * \void function 
 * \return CL_OK on success.
 * \Globals used : 
 */

Cl_ReturnCodeType Cl_Mac_ClearEvents(void)
{
	
	return CL_OK;
	
}


/*
 * \brief  check for a a new alarm present
		   call the alarm scaning routine
 * \void function 
 * \return CL_OK on success.
 * \Globals used : 
 */



Cl_ReturnCodeType Cl_Mac_CheckforAlarms(void)
{
	Cl_ReturnCodeType Cl_mac_retval;
	
	Cl_mac_retval = Cl_alarms_scanforAlarms();
	ASSERT_ERROR(Cl_mac_retval);
	
	if(Cl_mac_retval != CL_OK)
	{
		// handle error
	}
	
		return CL_OK;
}

Cl_ReturnCodeType Cl_Mac_CheckforBCEvent(void)
{
	Cl_ReturnCodeType Cl_mac_retval;
	
	Cl_mac_retval = Cl_CheckBcEvent();
	ASSERT_ERROR(Cl_mac_retval);
	
	if(Cl_mac_retval != CL_OK)
	{
		// handle error
	}
	
	return CL_OK;
}


/*
 * \brief  enter into calibration mode
		   
 * \void function 
 * \return CL_OK on success.
 * \Globals used : 
 */



Cl_ReturnCodeType Cl_Enter_CalibrateMode(void)
{
	
	Cl_ReturnCodeType  Cl_calibrate_Retcode = CL_OK;
	
	while(FOREVER)
	{
		
		Cl_calibrate_Retcode = Cl_Mac_GetSystem_Status_Query();// check if there is any system updates . ALSO may be this can be moved to a timer thread
		
		//
	}
	
	
}

Cl_ReturnCodeType cl_mac_checkforsync(void)
{
	Cl_ReturnCodeType  Cl_Retcode = CL_OK;
	if (ConsoleSync == true)
	{
		Cl_Retcode = Cl_SendDatatoconsole( CON_TX_COMMAND_SYNC,NULL ,0);
		ASSERT_ERROR(Cl_Retcode);
		if(Cl_Retcode != CL_OK)
		{
			// handle error
		}
		ConsoleSync = false;
	}
	else
	{
		// raise alarm
	//	Cl_SendDatatoconsole(CON_TX_COMMAND_PRINTTEXT,"LOST",4);
	}
	return CL_OK;
}

