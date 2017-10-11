/*
 * cl_event_manager.c
 *
 * Created: 3/9/2015 11:18:06 AM
 *  Author: user
 */ 

#include "cl_event_manager_include.h"
#include "cl_event_manager_proto.h"
#include "sv_interface_fun.h"

// extern Cl_ReturnCodeType cl_ufpumpFeedback_timer(void);
// extern Cl_ReturnCodeType cl_hep_pumpFeedback_timer(void) ;
// extern Cl_ReturnCodeType cl_level_switchcontroller(levelswitchcntrl_event_type cntrl_event );
// extern Cl_ReturnCodeType cl_flow_switchcontroller(flowswitchcntrl_event_type cntrl_event);
//extern Cl_ReturnCodeType cl_level_switchcontroller(levelswitchcntrl_event_type cntrl_event );
//extern Cl_ReturnCodeType cl_dprep_primecontroller(Cl_Dprep_PrimeEvents prime_event , int16_t data);
//extern Cl_ReturnCodeType cl_deaeration_controller(cl_deaeration_event_type cntrl_event);
// extern void Cl_SysStat_mean_status_update(void);
// extern  Cl_ReturnCodeType UpdateHeaterControls(void);
// extern Cl_ReturnCodeType  Cl_bc_controller(Cl_BC_EventType cl_bc_event);
// extern Cl_ReturnCodeType Cl_Heater_Event_Handler(HeaterEventsType HeaterEvent);
// extern Cl_ReturnCodeType  Cl_Temperature_Controller_Handler(Cl_Temp_EventType Cl_Tc_Event);
// extern Cl_ReturnCodeType  Cl_Pressure_Controller_Handler(Cl_PressureSensor_EventType Cl_Ps_Event);
// extern Cl_ReturnCodeType cl_bp_pumpFeedback_timer(void);
// extern Cl_ReturnCodeType Cl_Mac_CheckforAlarms(void);// check if there is any alarms , if there is one generate an event. ALSO This can me moved to respective sub  state handlers to be acted up on a much smaller time tick (on a 10- 100 ms event..???)
// extern Cl_Uint8Type  sv_cntrl_activatepump(sv_pumptype);
// extern Cl_Uint8Type  sv_cntrl_deactivatepump(sv_pumptype);
extern Cl_ReturnCodeType cl_wait(uint32_t);
//extern Cl_Uint8Type sv_cs_setpotvalue(uint16_t);
// extern void sv_prop_stopmixing(void);
// extern void sv_prop_startmixing(void);
// extern Cl_ReturnCodeType  Cl_Conductivity_Sensor_Controller(Cl_CondSensor_EventType Cl_cond_Event);
// extern Cl_ReturnCodeType  Cl_SendDatatoconsole(Cl_ConsoleTxCommandtype , Cl_Uint8Type* ,Cl_Uint8Type );
extern void DD_RETRY_IIC(void);
//extern Cl_ReturnCodeType cl_datamamanager(Cl_Mac_EventsType cl_data_event);

extern volatile int func_timer ;
extern Cl_Uint32Type write_count , read_count , success_count ,fail_count, clock_count  ,reset_count ;
extern Cl_Uint32Type bp_write_count , bp_read_count , bp_success_count ,bp_fail_count;
extern volatile Cl_Uint32Type g_ul_ms_ticks ;
extern void sv_prop_propo_CONDUCTIVITY(Cl_Uint16Type data);
extern void sv_prop_set_propo_ACIDrc(uint16_t);
bool transitionFlag = false;

//extern Cl_ReturnCodeType cl_bp_event_handler(cl_bp_events bp_event );
extern Cl_Mac_EventQueueType  Cl_MacEventQueue2;
Cl_Uint16Type dtemp = 0;
Cl_Uint16Type itemp = 0;
Cl_Uint16Type bptemp = 0;
Cl_Uint16Type prev_bpstate = 0;
Cl_Uint16Type prevdtemp = 0;
Cl_Uint16Type previtemp = 0;
Cl_Uint16Type prevtran = 0;
Cl_Uint16Type bpstatus = 0;
extern Cl_Uint32Type Treatdata[ID_MAX_TREAT_PARAM] ;
Cl_Int16Type testFPcurrentstatus_prev=0;
static Cl_Uint32Type test_counter =0;
extern Cl_Sys_statusType cl_sys_statbuffer;

uint16_t cond_4_Propo,Correction_Count=0,Set_conductivity =0, Set_RC=0;
uint16_t update_RC=0, prev_cond=0,direction_count=0, dir_cond =0 ;
extern heat_directiontype cond_dir;
void Get_direction(uint16_t);


Cl_ReturnCodeType cl_event_handler(Cl_Mac_EventsType cl_event)
{
	
	static volatile Cl_BoolType toggle = false;
	static Cl_BoolType toggle_500 = false;
	
	switch(cl_event)
	{
		case EVT_TICK_INSTANT:
		cl_level_switchcontroller(LEVEL_SWITCH_CNTRLER_EVENT_INSTANT);
		cl_flow_switchcontroller(FLOW_SWITCH_CNTRLER_EVENT_INSTANT);

		//	sv_cs_setpotvalue(250);
		
		
		
		
		break;
		

		
		case EVT_TICK_5M:

		
		break;
		case EVT_TICK_20M:
		
		 cl_bp_pumpFeedback_timer();
		
	/*	if(cl_sys_statbuffer.FPcurrentstatus == 1 && testFPcurrentstatus_prev == 0 ){
			
			Cl_bc_controller(BC_EVENT_CS);
			
		}
		
		testFPcurrentstatus_prev = cl_sys_statbuffer.FPcurrentstatus;
		*/
				//cl_ufpumpFeedback_timer(); // commented for testing
				//cl_hep_pumpFeedback_timer();
		//  cl_ufpumpFeedback_timer();
		//  cl_bp_pumpFeedback_timer();
		  break;
				
		#if 0
				if(func_timer > 10)
				{
					//sv_prop_startmixing();
				}
				//sv_cs_setpotvalue(225);
				if( toggle == false)
				{
					toggle = true;
					sv_cntrl_activatepump(BLOODPUMP);
					sv_prop_startmixing();
				//	sv_cs_setpotvalue(225);
				}
				else
				{
					sv_cntrl_deactivatepump(BLOODPUMP);
					sv_prop_stopmixing();
				//	sv_cs_setpotvalue(225);
					toggle = false;
				}
			
		#endif
	//	#if 0

			
		//	#endif
		//Cl_SysStat_mean_status_update();

//		if( toggle == false)
{
	//	sv_cntrl_activatepump(BLOODPUMP);
	//	sv_cs_setpotvalue(250);
//	sv_cntrl_activatepump(BLOODPUMP);
//	sv_cs_setpotvalue(225);
//	sv_cntrl_deactivatepump(BLOODPUMP);
//	sv_prop_stopmixing();
//	sv_cs_setpotvalue(450);
//	sv_cs_setpotvalue(550);
//	sv_cs_setpotvalue(250);
	;


//	toggle = true;
}
//else
{

//		sv_cntrl_activatepump(BLOODPUMP);
	//	sv_cs_setpotvalue(250);
	//	sv_cntrl_activatepump(BLOODPUMP);
//	sv_cs_setpotvalue(225);
//	sv_cntrl_deactivatepump(BLOODPUMP);
//	sv_prop_stopmixing();
//	sv_cs_setpotvalue(450);
//	sv_cs_setpotvalue(550);
//	sv_cs_setpotvalue(250);


//	toggle = false;
}
		
	
	
		break;		
		case EVT_TICK_50M:
		
	//	{
		#if 0

		Cl_Uint32Type init_ticks =0, spent_ticks=0;
		init_ticks = g_ul_ms_ticks;

				if( toggle == false)
				{
					toggle = true;
			
					sv_cntrl_activatepump(BLOODPUMP);
					sv_prop_startmixing();
					
					
					
				}
				else
				{
				
					sv_cntrl_deactivatepump(BLOODPUMP);
					sv_prop_stopmixing();
					
					toggle = false;
				}
					
					spent_ticks = 	g_ul_ms_ticks - init_ticks;
			#endif
					
		 Cl_bc_controller(BC_EVENT_50MS);	
	
		 
// 		 if(cl_sys_statbuffer.FPcurrentstatus == 1 && testFPcurrentstatus_prev == 0 ){
// 			
// 			Cl_bc_controller(BC_EVENT_CS);
// 			
// 		}
// 		
// 		testFPcurrentstatus_prev = cl_sys_statbuffer.FPcurrentstatus;
// 		 
// 		}
		break;
		
		case EVT_TICK_150M:
	
		
	break;	
		case EVT_TICK_100M:
	//	Cl_Mac_CheckforAlarms();// check if there is any alarms , if there is one generate an event. ALSO This can me moved to respective sub  state handlers to be acted up on a much smaller time tick (on a 10- 100 ms event..???)
	cl_hep_pumpFeedback_timer();
	cl_ufpumpFeedback_timer();
	
	Cl_SysStat_GetSensor_Status_Query(SENSOR_BP_DEC_M , &dtemp);
	if(dtemp == 1){

		if(prevdtemp == 0){             // manav02 added transition logic
			
		prevdtemp = 1;
		
		dtemp = 0;
		if( Cl_MacEventQueue2.Cl_MacNewEventcount <= MAC_EVENT_COUNT_MAX )
		{
			Cl_MacEventQueue2.Cl_MacEvent[Cl_MacEventQueue2.Cl_MacNewEventcount] = EVT_CONSOLE_COMMAND_SET_BLDPUMPDEC_MANUAL ;
			Cl_MacEventQueue2.Cl_MacNewEvent = true;
			Cl_MacEventQueue2.Cl_MacNewEventcount++;
			
		}
		}
		//Treatdata[ID_bloodrate] - 10 ;
	}
	else{
		prevdtemp = 0 ;
	}
	
	
	Cl_SysStat_GetSensor_Status_Query(SENSOR_BP_INC_M , &itemp);
	if(itemp == 1){

		if(previtemp == 0){               // manav02 added transition logic
			
		previtemp = 1 ; 

		itemp = 0;
		if( Cl_MacEventQueue2.Cl_MacNewEventcount <= MAC_EVENT_COUNT_MAX )
		{
			Cl_MacEventQueue2.Cl_MacEvent[Cl_MacEventQueue2.Cl_MacNewEventcount] = EVT_CONSOLE_COMMAND_SET_BLDPUMPINC_MANUAL ;
			Cl_MacEventQueue2.Cl_MacNewEvent = true;
			Cl_MacEventQueue2.Cl_MacNewEventcount++;
			
		}
		
		}
		//Treatdata[ID_bloodrate] + 10 ;
		
	}
	else{
		previtemp = 0 ;
	}
	
	
	Cl_SysStat_GetSensor_Status_Query(SENSOR_BP_ON_OFF_M , &bptemp);
	
	
	if(bptemp == 0){
		
		prevtran = 0;
	}
	else{
		
		if(prevtran == 0){
			
		prevtran = 1;

		if(bpstatus == 0){
	
			//bp on
			if( Cl_MacEventQueue2.Cl_MacNewEventcount <= MAC_EVENT_COUNT_MAX )
			{
				Cl_MacEventQueue2.Cl_MacEvent[Cl_MacEventQueue2.Cl_MacNewEventcount] = EVT_CONSOLE_COMMAND_SET_BLDPMP_ON ;
				Cl_MacEventQueue2.Cl_MacNewEvent = true;
				Cl_MacEventQueue2.Cl_MacNewEventcount++;
				
			}
	
			bpstatus = 1;
		}
		else{
			// bp off	
			if( Cl_MacEventQueue2.Cl_MacNewEventcount <= MAC_EVENT_COUNT_MAX )
			{
				Cl_MacEventQueue2.Cl_MacEvent[Cl_MacEventQueue2.Cl_MacNewEventcount] = EVT_CONSOLE_COMMAND_SET_BLDPMP_OFF ;
				Cl_MacEventQueue2.Cl_MacNewEvent = true;
				Cl_MacEventQueue2.Cl_MacNewEventcount++;
				
			}
			bpstatus = 0;
		}
			
			
		}
		
	}
	
	
	
/*	
	if(bptemp == 1 ){
		
		if(transitionFlag){
		
		if(prev_bpstate == 0 ){
			
			prev_bpstate = 1;
			bptemp = 0;
			transitionFlag =false;
			if( Cl_MacEventQueue2.Cl_MacNewEventcount <= MAC_EVENT_COUNT_MAX )
			{
				Cl_MacEventQueue2.Cl_MacEvent[Cl_MacEventQueue2.Cl_MacNewEventcount] = EVT_CONSOLE_COMMAND_SET_BLDPMP_ON ;
				Cl_MacEventQueue2.Cl_MacNewEvent = true;
				Cl_MacEventQueue2.Cl_MacNewEventcount++;
				
			}
			
			
		}
		else {
			
			prev_bpstate = 0;
			bptemp =0;
			transitionFlag = false;
			if( Cl_MacEventQueue2.Cl_MacNewEventcount <= MAC_EVENT_COUNT_MAX )
			{
				Cl_MacEventQueue2.Cl_MacEvent[Cl_MacEventQueue2.Cl_MacNewEventcount] = EVT_CONSOLE_COMMAND_SET_BLDPMP_OFF ;
				Cl_MacEventQueue2.Cl_MacNewEvent = true;
				Cl_MacEventQueue2.Cl_MacNewEventcount++;
				
			}
			
			
		}
		}
	}
	else{
		
		bptemp = 0;
		transitionFlag = true;
	}
	
	*/
	
	
	
		Cl_Temperature_Controller_Handler(TS_EVENT_100MS);    // commented on 17082017, we dont need this controller handler any more
		Cl_Pressure_Controller_Handler(PS_EVENT_100MS);
//		cl_bp_pumpFeedback_timer();
		cl_hep_pump_enddetction_timer();
	//	Cl_SysStat_mean_status_update();
	
	
	
	
		
		#if 0
		Cl_Uint32Type init_ticks =0, spent_ticks=0;
		init_ticks = g_ul_ms_ticks;


				if( toggle == false)
				{
					toggle = true;
			
					sv_cntrl_activatepump(BLOODPUMP);
					sv_prop_startmixing();
					
					
				}
				else
				{
				
					sv_cntrl_deactivatepump(BLOODPUMP);
					sv_prop_stopmixing();
					
					toggle = false;
				}
				spent_ticks = 	g_ul_ms_ticks - init_ticks;	
			#endif
			
		#if 0
		//DD_RETRY_IIC();
		static bool toggle = false;
				static Cl_Uint8Type temp_count=0;
				temp_count++;

				
		if(!toggle)
		{
		SV_CAN_READ_HEMO();
		sv_cntrl_activatepump(BLOODPUMP);
		
			toggle = true;
				if (temp_count == 5)
				{
			//		sv_cntrl_activatepump(BLOODPUMP);
				}

		}
		else
		{
		SV_CAN_REQUEST_SENSORDATA();
		sv_cntrl_deactivatepump(BLOODPUMP);
			toggle = false;
				if (temp_count == 10)
				{
			//		sv_cntrl_deactivatepump(BLOODPUMP);
					temp_count = 0;
				}
		}
		
		#endif
		
		
		break;
		case EVT_TICK_500M:
		//UpdateHeaterControls();
		Cl_Heater_Event_Handler(CL_HEATER_EVENT_500MS,0);
		Cl_bc_controller(BC_EVENT_500MS);
		Cl_SysStat_GetSensor_Status_Query(SENSOR_COND_STATUS,&cond_4_Propo);
		
		if (Set_RC == 0)
		{
		
		switch (Treatdata[ID_setcond])
		{
		case 135:
			Set_conductivity = 1370;
			Set_RC = 200;
		break;
		case 136:
			Set_conductivity = 1381;
			Set_RC = 198;
		break;
		case 137:
			Set_conductivity = 1392;
			Set_RC = 196;
		break;
		case 138:
			Set_conductivity = 1403;
			Set_RC = 194;
		break;
		case 139:
			Set_conductivity = 1400;  // 1414
			Set_RC = 192;
		break;
		case 140:
			Set_conductivity = 1425;
			Set_RC = 190;
		break;
		case 141:
			Set_conductivity = 1410;  // 1436
			Set_RC = 183;
			break;
		case 142:
		Set_conductivity = 1447;
		Set_RC = 186;
		break;
		case 143:
		Set_conductivity = 1458;
		Set_RC = 184;
		break;
		
		case 144:
		Set_conductivity = 1469;
		Set_RC = 182;
		break;
		
		case 145:
		Set_conductivity = 1480;
		Set_RC = 180;
		break;
		
		}
		update_RC = Set_RC;
		}
		
		if (cond_4_Propo > (Set_conductivity -150))
		{
			Get_direction(cond_4_Propo);
			update_RC = calculate_RC(cond_4_Propo);
		}
		sv_prop_set_propo_ACIDrc(update_RC);
//		sv_prop_propo_CONDUCTIVITY(cond_4_Propo);  // Gopal: added on 13092017 sending conductivity to proportioning for RC correction
		if(  toggle_500)
		{
			toggle_500 = false;
		//	sv_cntrl_activatevenousclamp();
		}

		else
		{
			toggle_500 = true;
		//	sv_cntrl_deactivatevenousclamp();
		}
		
		
		


							
		break;
		case EVT_TICK_SEC:
		{

		cl_level_switchcontroller(LEVEL_SWITCH_CNTRLER_EVENT_SEC);
		cl_flow_switchcontroller(FLOW_SWITCH_CNTRLER_EVENT_SEC);
		cl_dprep_primecontroller(CL_DPREP_PRIME_PRIME_TICK_SEC,0);
		cl_hep_controller(CL_HEP_EVENT_SEC_TICK,0);
		Cl_bc_controller(BC_EVENT_SECOND);
		Cl_Heater_Event_Handler(CL_HEATER_EVENT_1SEC,0);
		Cl_Conductivity_Sensor_Controller(COND_EVENT_1SEC);
		Cl_SysStat_mean_status_update();
		cl_bp_feedback_hemo();
//	#if 0
		//DD_RETRY_IIC();
		static bool toggle = false;
				static Cl_Uint8Type temp_count=0;
				temp_count++;

				
		if(!toggle)
		{
		SV_CAN_READ_HEMO(); // disabled for testing
		
			toggle = true;
				if (temp_count == 5)
				{
			//		sv_cntrl_activatepump(BLOODPUMP);
				}

		}
		else
		{
		SV_CAN_REQUEST_SENSORDATA();
			toggle = false;
				if (temp_count == 10)
				{
			//		sv_cntrl_deactivatepump(BLOODPUMP);
					temp_count = 0;
				}
		}
		
//		#endif

		test_counter++;
		}
		
		break;
		case EVT_TICK_MIN:
		{
			//Cl_SysStat_mean_status_update();
		Cl_Temperature_Controller_Handler(TS_EVENT_MIN);
		cl_Datastreamtype cl_tdata;
		Cl_Uint16Type t_wr_cnt,t_succ_cnt,t_fail_cnt,t_bpfail_cnt;
		
// 		if (write_count > 65535)
// 		{
// 			t_wr_cnt = write_count % 65535;
// 			
// 		}
// 		else {t_wr_cnt = write_count ;}
// 			
// 		
// 		if (success_count > 65535)
// 		{
// 			t_succ_cnt = success_count % 65535;
// 			
// 		}	else {t_succ_cnt = success_count;}	
// 			
// 		if (fail_count > 65535)
// 		{
// 			t_fail_cnt = fail_count % 65535;
// 			
// 		}	else {t_fail_cnt = fail_count;}
// 			
// 		if (bp_fail_count > 65535)
// 		{
// 			t_bpfail_cnt = bp_fail_count % 65535;
// 			
// 		} else {t_bpfail_cnt = bp_fail_count;}
// 			
// 		cl_tdata.word =0;
// 		cl_tdata.Twobyte = t_wr_cnt;
// 		cl_tdata.bytearray[2] = 25;
// 		Cl_SendDatatoconsole(CON_TX_COMMAND_COMMAND_SCRIPT_PRNIT,&cl_tdata,4);
// 		cl_tdata.word =0;
// 		cl_tdata.Twobyte = t_succ_cnt;
// 		cl_tdata.bytearray[2] = 26;
// 		Cl_SendDatatoconsole(CON_TX_COMMAND_COMMAND_SCRIPT_PRNIT,&cl_tdata,4);
// 		cl_tdata.word =0;
// 		cl_tdata.Twobyte = t_fail_cnt;
// 		cl_tdata.bytearray[2] = 27;
// 		Cl_SendDatatoconsole(CON_TX_COMMAND_COMMAND_SCRIPT_PRNIT,&cl_tdata,4);
// 		cl_tdata.word =0;
// 		cl_tdata.Twobyte = t_bpfail_cnt;
// 		cl_tdata.bytearray[2] = 28;
// 		Cl_SendDatatoconsole(CON_TX_COMMAND_COMMAND_SCRIPT_PRNIT,&cl_tdata,4);
// 		
	
		}
		break;
		case EVT_ALERT_CS:
		Cl_bc_controller(BC_EVENT_CS);
		break;
		
		case EVT_CONSOLE_COMMAND_GET_DATA:
		case EVT_CONSOLE_COMMAND_SET_DATA:
		cl_datamamanager(cl_event);
		break;
		default:
		break;
	}
	
	return 0;
}


// uint16_t calculate_RC(uint16_t System_Conductivity)
// {
// 					Correction_Count++;
// 
// //				if(Set_conductivity > (System_Conductivity -10))
// 				if(1200 < System_Conductivity  < 1700)
// 				{
// 				if(Correction_Count >= 60)
// 				{
// 	
// 					if((System_Conductivity > (Set_conductivity +5)) || (System_Conductivity < (Set_conductivity -5) ) )
// 					{
// 					
// 			
// 						if (System_Conductivity < Set_conductivity)
// 						{
// 						//	if (System_Conductivity < (Set_conductivity -10) ) 
// 							{
// 					
// 			//					if ((Set_RC - rc) < 3 || (rc - Set_RC) < 3 )
// 								{
// 									update_RC = update_RC-1;
// // 									if (update_RC < (Set_RC -4))
// // 									{
// // 										update_RC = Set_RC;
// // 										
// // 									}
// 									Correction_Count =0;
// 								}
// 							// 					else
// 							// 						rc = Set_RC;
// 							}	
// 						}
// 						else if((System_Conductivity > Set_conductivity))
// 						{
// 		//					if ((Set_RC - rc) < 3 || (rc - Set_RC) < 3 )
// 					
// 							{
// 								update_RC = update_RC+1;
// // 								if (update_RC > (Set_RC + 4))
// // 								{
// // 									update_RC = Set_RC;
// // 									
// // 								}
// 								Correction_Count =0;
// 							}
// 							// 					else
// 							// 					rc = Set_RC;
// 						}
// 					}
// 					
// 					if (update_RC ==0)
// 					{
// 						update_RC = Set_RC;
// 					}
// 					sv_prop_set_propo_ACIDrc(update_RC);
// 				}
// 				}
// 
// 	return update_RC;
// }

uint16_t calculate_RC(uint16_t System_Conductivity)
{
		Correction_Count++;
		
		if (Correction_Count ==60)
		{
		
			Correction_Count =0;		
			if (dir_cond == UP)
			{
				if (System_Conductivity > (Set_conductivity +3))
				{
					update_RC = update_RC +1;
				}
// 				else if (System_Conductivity < (Set_conductivity -5))
// 				{
// 					update_RC = update_RC -1;
// 				}
			}
			else if (dir_cond == DOWN)
			{
				if (System_Conductivity < (Set_conductivity -3))
				{
					update_RC = update_RC -1;
				}
			
// 				else if (System_Conductivity > (Set_conductivity +5))
// 				{
// 					update_RC = update_RC +1;
// 				}
			}
// 		else if (dir_cond == NUETRAL)
// 		{
// 		}
					
		}
					
		if (update_RC ==0)
		{
			update_RC = Set_RC;
		}
//		sv_prop_set_propo_ACIDrc(update_RC);
				
				

	return update_RC;
}



void Get_direction(uint16_t cond)
{
	direction_count ++;
	
	if (prev_cond > cond)
		{
			dir_cond = UP;
		}
	else if( prev_cond < cond)
		{
			dir_cond = DOWN;
		}
	else
		dir_cond = NUETRAL;
		
	if (direction_count >= 10)
	{
		direction_count =0;
		prev_cond = cond;
	}
	
	return 0;
}