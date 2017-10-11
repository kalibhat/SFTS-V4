/*
 * cl_mac_proto.h
 *
 * Created: 4/25/2017 12:05:32 PM
 *  Author: user
 */ 


#ifndef CL_MAC_PROTO_H_
#define CL_MAC_PROTO_H_

#include "cl_types.h"

Cl_ReturnCodeType	Cl_MacInit(void);
Cl_ReturnCodeType	Cl_Mac_Configure(void);
Cl_ReturnCodeType	Cl_Mac_GetSystem_Status_Query(void);
Cl_ReturnCodeType	Cl_MacCheck_TimerExpired(void);
Cl_ReturnCodeType 	Cl_MacCheckforConsoleMessage(void);
Cl_ReturnCodeType  Cl_App_init(void);
Cl_ReturnCodeType  Cl_Mac_ClearEvents(void);
Cl_ReturnCodeType  Cl_MacCall_Newappreq_Query(void);
Cl_ReturnCodeType  Cl_Mac_CheckforAlarms(void);
Cl_ReturnCodeType  Cl_Mac_CheckforErrorCorrections(void);
Cl_ReturnCodeType  Cl_alarms_scanforAlarms(void);
Cl_ReturnCodeType  Cl_Enter_CalibrateMode(void);
Cl_ReturnCodeType  cl_mac_checkforsync(void);
Cl_ReturnCodeType	Cl_MacCreate_Macevent(void);
Cl_ReturnCodeType  Cl_Mac_Create_ConsoleEvent(void);
Cl_ReturnCodeType  Cl_Mac_CheckforBCEvent(void);
Cl_ReturnCodeType  Cl_CheckBcEvent(void);


#endif /* CL_MAC_PROTO_H_ */