/*
 * Cl_MacAppRequestHandler.c
 *
 * Created: 1/16/2014 3:21:40 PM
 *  Author: user
 */ 

#include "cl_types.h"
#include "cl_mac_controller.h"

extern Cl_Mac_EventsType Cl_MacEvent;
extern Cl_BoolType Cl_MacNewEvent;
extern Cl_Mac_EventQueueType Cl_MacEventQueue1;


Cl_ReturnCodeType Cl_mac_apprequesthandler(Cl_Mac_EventsType);

Cl_ReturnCodeType Cl_mac_apprequesthandler(Cl_Mac_EventsType cl_macirq)
{
	Cl_ReturnCodeType Cl_Mac_RetVal = CL_OK;
	
	//Cl_MacEvent = cl_macirq;
	//Cl_MacNewEvent = true;
	
	if( Cl_MacEventQueue1.Cl_MacNewEventcount < MAC_EVENT_COUNT_MAX )
	{
		Cl_MacEventQueue1.Cl_MacEvent[Cl_MacEventQueue1.Cl_MacNewEventcount] = cl_macirq ;
		Cl_MacEventQueue1.Cl_MacNewEvent = true;
		Cl_MacEventQueue1.Cl_MacNewEventcount++;
	}
	return Cl_Mac_RetVal;
}