/*
 * Cl_TimerManager.c
 *
 * Created: 1/14/2014 12:58:28 PM
 *  Author: user
 */ 
#include "Cl_TimerManager_include.h"
#include "Cl_TimerManager_proto.h"

extern Cl_Mac_EventsType Cl_MacEvent;
extern Cl_BoolType Cl_MacNewEvent;

extern Cl_Mac_EventQueueType  Cl_MacEventQueue2;
extern volatile int FUNC_ENTRY;
extern Cl_BoolType syncdone;

extern Cl_tg_prick_100ms(void);
extern Cl_tg_prick_50ms(void);
extern Cl_ReturnCodeType Cl_Alarm_timmer_50millisec(void);
extern void pinclock(void);
extern void pinclock1(void);
extern Cl_ReturnCodeType cl_ufpumpFeedback_timer(void);
extern void testsectclock(void);
extern void tg_timer_minute(void);

extern void Cl_tg_prick_1ms(void);

//Cl_Int16Type testFPcurrentstatus_prev=0;
//extern Cl_Sys_statusType cl_sys_statbuffer;

Cl_ReturnCodeType Cl_RetValue;
int gmstick = 0;
volatile int func_timer =0;
Cl_ReturnCodeType Cl_TimerTickCallback()
{
	static int Cl_ticks =0 , Cl_Seconds =0 , Cl_Minutes = 0 , Cl_Hours = 0;
	
	Cl_ticks ++;
	
	 if (Cl_ticks >= TICKS_PER_SEC)
	 {
		 Cl_ticks = 0;
		 Cl_Seconds++;
		// Cl_RetValue = Cl_Timer_Create_TickSec_Event();
	 }
	 
	// if(Cl_Seconds >= SIXTY)
	 if(Cl_Seconds >= SIXTY)
	 {
		 Cl_Seconds = 0;
		  Cl_Minutes++;
		// Cl_RetValue = Cl_Timer_Create_TickMin_Event();
	 }
	 
	if(Cl_Minutes >= SIXTY)
//	if(Cl_Minutes >= 10)
	{
		Cl_Minutes = 0;
		Cl_Hours++;
		Cl_RetValue = Cl_Timer_Create_TickHour_Event();
	}
	
	if(Cl_Hours >= 24)
	{
		Cl_Hours =0;
	}
	
	return CL_OK;
}





Cl_ReturnCodeType Cl_MilliTickcallBack(void)
{
	Cl_ReturnCodeType Cl_TimerRetValue;
	static int Cl_msticks =0;
//	static int Cl_ms50ticks = 0;
	static int Cl_ms100ticks = 0;
	static int Cl_ms150ticks = 0;
	static int Cl_ms500ticks = 0;
	static int Cl_ms1000ticks = 0;
	
//Cl_tg_prick_1ms();



if (FUNC_ENTRY == 1)
{
	func_timer++;
	if(func_timer > 1000)
	{
		// alarm
			
			if(syncdone)
			{
		//		Cl_SendDatatoconsole(CON_TX_COMMAND_COMMAND_IIC_HANGUP,(void*)0,0);
			}
		func_timer = 0;
	}
}else
{
	func_timer = 0;
}
			gmstick++;
			 if(gmstick > 50)
			 {
				 gmstick = 0;
			 }
			 
		Cl_msticks++;
		if(!(Cl_msticks % 20))
		{
		//	pinclock(); 
		Cl_TimerRetValue = Cl_Timer_Create_Tick20Ms_Event();
			
	
		//Cl_tg_prick_100ms();
		


		}
		if(!(Cl_msticks % 2))
		{
			pinclock(); 
			//pinclock();
			//pinclock1();
			Cl_TimerRetValue = Cl_Timer_Create_Tick2Ms_Event();
			
			//	cl_ufpumpFeedback_timer();
			//	cl_hep_pumpFeedback_timer();
		}
		if(!(Cl_msticks % 5))
		{
			
			//pinclock();
			//pinclock1(); 
			Cl_TimerRetValue = Cl_Timer_Create_Tick5Ms_Event();
	
		//	cl_ufpumpFeedback_timer();
		//	cl_hep_pumpFeedback_timer();
		}
		if(!(Cl_msticks % 10))
		{
			Cl_TimerRetValue = Cl_Timer_Create_Tick10Ms_Event();
			//pinclock();
			//pinclock1(); 
			
			

		}
		if(!(Cl_msticks % 50))
		{
			Cl_TimerRetValue = Cl_Timer_Create_Tick50Ms_Event();
		//	Cl_tg_prick_50ms();
			//pinclock1();

		}

		
		
		if(Cl_msticks == 100)
		{
			//Cl_msticks = 0;
			Cl_ms100ticks++;
			Cl_TimerRetValue = Cl_Timer_Create_Tick100Ms_Event();

				//cl_hep_pump_enddetction_timer();


		}

if(Cl_msticks == 150)
		{
			Cl_msticks = 0;
			Cl_ms150ticks++;
			Cl_TimerRetValue = Cl_Timer_Create_Tick150Ms_Event();

				//cl_hep_pump_enddetction_timer();


		}


		if(Cl_ms100ticks == 3 )
		{
			Cl_ms100ticks = 0;
			Cl_ms500ticks++;
			if(Cl_ms500ticks == 2 )
			{
				Cl_ms500ticks = 0;
				Cl_TimerRetValue  = Cl_Timer_Create_TickSec_Event();
				Cl_ms1000ticks++;
				testsectclock();
				
			}
			Cl_TimerRetValue = Cl_Timer_Create_Tick500Ms_Event();
			
		}
		if(Cl_ms1000ticks == 2)
		{
			Cl_ms1000ticks =0;
		//	Cl_Alarm_TriggerAlarm(FPCURRENTSTATUS,1);
		}
		return CL_OK;
}


 Cl_ReturnCodeType  Cl_Timer_Create_Tick2Ms_Event(void)
 {
	 if( Cl_MacEventQueue2.Cl_MacNewEventcount < MAC_EVENT_COUNT_MAX )
	 
	 
	 {
	//	 Cl_MacEventQueue2.Cl_MacEvent[Cl_MacEventQueue2.Cl_MacNewEventcount] = EVT_TICK_2M;
	//	 Cl_MacEventQueue2.Cl_MacNewEvent = true;
	//	 Cl_MacEventQueue2.Cl_MacNewEventcount++;
	 }
	 return CL_OK;
	 
 }

 Cl_ReturnCodeType  Cl_Timer_Create_Tick5Ms_Event(void)
 {
	 if( Cl_MacEventQueue2.Cl_MacNewEventcount < MAC_EVENT_COUNT_MAX )
	 
	 
	 {
	//	 Cl_MacEventQueue2.Cl_MacEvent[Cl_MacEventQueue2.Cl_MacNewEventcount] = EVT_TICK_5M;
	//	 Cl_MacEventQueue2.Cl_MacNewEvent = true;
	//	 Cl_MacEventQueue2.Cl_MacNewEventcount++;
	 }
	 return CL_OK;
	 
 }

 Cl_ReturnCodeType  Cl_Timer_Create_Tick10Ms_Event(void)
 {
	 if( Cl_MacEventQueue2.Cl_MacNewEventcount < MAC_EVENT_COUNT_MAX )
	 
	 
	 {
	//	 Cl_MacEventQueue2.Cl_MacEvent[Cl_MacEventQueue2.Cl_MacNewEventcount] = EVT_TICK_10M;
	//	 Cl_MacEventQueue2.Cl_MacNewEvent = true;
	//	 Cl_MacEventQueue2.Cl_MacNewEventcount++;
	 }
	 return CL_OK;
	 
 }

 Cl_ReturnCodeType  Cl_Timer_Create_Tick20Ms_Event(void)
 {
	 	 if( Cl_MacEventQueue2.Cl_MacNewEventcount < MAC_EVENT_COUNT_MAX )
	 	 
	 	 
	 	 {
			  
// 			  if(cl_sys_statbuffer.FPcurrentstatus == 1 && testFPcurrentstatus_prev == 0 ){
// 				  
// 				  Cl_bc_controller(BC_EVENT_CS);
// 				  
// 			  }
// 			  
// 			  testFPcurrentstatus_prev = cl_sys_statbuffer.FPcurrentstatus;
// 			  
		 	 Cl_MacEventQueue2.Cl_MacEvent[Cl_MacEventQueue2.Cl_MacNewEventcount] = EVT_TICK_20M;
		 	 Cl_MacEventQueue2.Cl_MacNewEvent = true;
		 	 Cl_MacEventQueue2.Cl_MacNewEventcount++;
	 	 }
		 else
		 {
//			 Cl_SendDatatoconsole(CON_TX_COMMAND_COMMAND_SCRIPT_PRNIT ,"OF",2);       //12-04
		 }
	 	 return CL_OK;
	 	 
 }

 Cl_ReturnCodeType  Cl_Timer_Create_Tick50Ms_Event(void)
 {
	 // Cl_MacEvent = EVT_TICK_SEC;
	 // Cl_MacNewEvent = true;
	 Cl_Alarm_timmer_50millisec();// testing
	 if( Cl_MacEventQueue2.Cl_MacNewEventcount < MAC_EVENT_COUNT_MAX )
	 
	 
	 {
		 Cl_MacEventQueue2.Cl_MacEvent[Cl_MacEventQueue2.Cl_MacNewEventcount] = EVT_TICK_50M;
		 Cl_MacEventQueue2.Cl_MacNewEvent = true;
		 Cl_MacEventQueue2.Cl_MacNewEventcount++;
	 }
		 else
		 {
//			 Cl_SendDatatoconsole(CON_TX_COMMAND_COMMAND_SCRIPT_PRNIT ,"OF",2);   //12-04
		 }
	 return CL_OK;
	 
	 
 }
 

	 Cl_ReturnCodeType  Cl_Timer_Create_Tick100Ms_Event(void)
	 {
		 // Cl_MacEvent = EVT_TICK_SEC;
		 // Cl_MacNewEvent = true;
		 
		 if( Cl_MacEventQueue2.Cl_MacNewEventcount < MAC_EVENT_COUNT_MAX )
		 		 
		 {	
			 Cl_MacEventQueue2.Cl_MacEvent[Cl_MacEventQueue2.Cl_MacNewEventcount] = EVT_TICK_100M ;
			 Cl_MacEventQueue2.Cl_MacNewEvent = true;
			 Cl_MacEventQueue2.Cl_MacNewEventcount++;
		 }
		 else
		 {
//			 Cl_SendDatatoconsole(CON_TX_COMMAND_COMMAND_SCRIPT_PRNIT ,"OF",2);   ////12-04
		 }
		 return CL_OK;
		 
		 
	 }



	 Cl_ReturnCodeType  Cl_Timer_Create_Tick150Ms_Event(void)
	 {
		 // Cl_MacEvent = EVT_TICK_SEC;
		 // Cl_MacNewEvent = true;
		 
		 if( Cl_MacEventQueue2.Cl_MacNewEventcount < MAC_EVENT_COUNT_MAX )
		 		 
		 {	
			 Cl_MacEventQueue2.Cl_MacEvent[Cl_MacEventQueue2.Cl_MacNewEventcount] = EVT_TICK_150M ;
			 Cl_MacEventQueue2.Cl_MacNewEvent = true;
			 Cl_MacEventQueue2.Cl_MacNewEventcount++;
		 }
		 else
		 {
//			 Cl_SendDatatoconsole(CON_TX_COMMAND_COMMAND_SCRIPT_PRNIT ,"OF",2);   ////12-04
		 }
		 return CL_OK;
		 
		 
	 }
	 
	 
	 Cl_ReturnCodeType  Cl_Timer_Create_Tick500Ms_Event(void)
	 {
		 // Cl_MacEvent = EVT_TICK_SEC;
		 // Cl_MacNewEvent = true;
		 
		 if( Cl_MacEventQueue2.Cl_MacNewEventcount < MAC_EVENT_COUNT_MAX )
		 
		 
		 {
			 Cl_MacEventQueue2.Cl_MacEvent[Cl_MacEventQueue2.Cl_MacNewEventcount] = EVT_TICK_500M ;
			 Cl_MacEventQueue2.Cl_MacNewEvent = true;
			 Cl_MacEventQueue2.Cl_MacNewEventcount++;
		 }
		 else
		 {
//			 Cl_SendDatatoconsole(CON_TX_COMMAND_COMMAND_SCRIPT_PRNIT ,"OF",2);   //12-04
		 }
		 return CL_OK;
		 
		 
	 }



	 Cl_ReturnCodeType  Cl_Timer_Create_TickSec_Event(void)
	 {
		// Cl_MacEvent = EVT_TICK_SEC;
		// Cl_MacNewEvent = true;
		static Cl_Uint8Type Cl_Secondscnt = 0;
		
		Cl_Secondscnt++;
		// Cl_Mac_GetSystem_Status_Query();
		 if( Cl_MacEventQueue2.Cl_MacNewEventcount < MAC_EVENT_COUNT_MAX )
		
		
		 {
			 Cl_MacEventQueue2.Cl_MacEvent[Cl_MacEventQueue2.Cl_MacNewEventcount] = EVT_TICK_SEC ;
			 Cl_MacEventQueue2.Cl_MacNewEvent = true;
			 Cl_MacEventQueue2.Cl_MacNewEventcount++;
		 }
		 else
		 {
//			 Cl_SendDatatoconsole(CON_TX_COMMAND_COMMAND_SCRIPT_PRNIT ,"OF",2);            //12-04
		 }		 
		if(Cl_Secondscnt >= 60)
		 {
			 Cl_Secondscnt = 0;
			//  Cl_Minutes++;
			 Cl_RetValue = Cl_Timer_Create_TickMin_Event();
			// tg_timer_minute();
		 }
			 return CL_OK;
		 
		
		 }

	 Cl_ReturnCodeType  Cl_Timer_Create_TickMin_Event(void)
	 {
		 
		 if( Cl_MacEventQueue2.Cl_MacNewEventcount < MAC_EVENT_COUNT_MAX )	
		 {
			 Cl_MacEventQueue2.Cl_MacEvent[Cl_MacEventQueue2.Cl_MacNewEventcount] = EVT_TICK_MIN ;
			 Cl_MacEventQueue2.Cl_MacNewEvent = true;
			 Cl_MacEventQueue2.Cl_MacNewEventcount++;
			
		 }
		 else
		 {
//			 Cl_SendDatatoconsole(CON_TX_COMMAND_COMMAND_SCRIPT_PRNIT ,"OF",2);             //12-04
		 }
		 return CL_OK;
		 
		 // Cl_Console_ConvertTimeoutEvent_toMacEvent(Cl_ConsoleRxEventsType command ,Cl_Mac_EventsType* MacEvent)
	 }

	 Cl_ReturnCodeType  Cl_Timer_Create_TickHour_Event(void)
	 {

		 if( Cl_MacEventQueue2.Cl_MacNewEventcount <= MAC_EVENT_COUNT_MAX )	
		 {
			 Cl_MacEventQueue2.Cl_MacEvent[Cl_MacEventQueue2.Cl_MacNewEventcount] = EVT_TICK_HOUR ;
			 Cl_MacEventQueue2.Cl_MacNewEvent = true;
			 Cl_MacEventQueue2.Cl_MacNewEventcount++;
			 
		 }
		 return CL_OK;
		 
		 // Cl_Console_ConvertTimeoutEvent_toMacEvent(Cl_ConsoleRxEventsType command ,Cl_Mac_EventsType* MacEvent)
	 }

