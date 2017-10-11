/*
 * cl_consolecontroller_ext.h
 *
 * Created: 4/25/2017 10:58:34 AM
 *  Author: user
 */ 


#ifndef CL_CONSOLECONTROLLER_EXT_H_
#define CL_CONSOLECONTROLLER_EXT_H_

#include "cl_mac_controller.h"

extern Cl_ReturnCodeType		Cl_Console_Read_ConsoleMessage_Status(void);
extern Cl_ReturnCodeType		 Cl_Console_Read_Message_Console(void);
extern Cl_ReturnCodeType        Cl_SendDatatoconsole(Cl_ConsoleTxCommandtype , Cl_Uint8Type* ,Cl_Uint8Type );
extern Cl_ReturnCodeType		 Cl_MacWrite_Message_Console(void);
extern Cl_ReturnCodeType	 Cl_Mac_Create_ConsoleEvent(void);
extern Cl_ReturnCodeType  Cl_ScanForMsgStart(void);
extern Cl_ReturnCodeType Cl_Cosnsole_MsgAcknlowledge(void);
extern Cl_ReturnCodeType  cl_console_translatemacevent(Cl_Mac_EventsType ,Cl_ConsoleRxEventsType );
extern Cl_ReturnCodeType	 Cl_ConsoleStateHandler(Cl_Mac_EventsType );
extern Cl_ReturnCodeType 	Cl_Console_Console_Sync(void);
extern Cl_ReturnCodeType Cl_Console_init(void);
extern Cl_ReturnCodeType Cl_Console_SyncGetMessage(void);
extern Cl_Uint8Type Cl_increment_msgbufferindex(void);
extern Cl_Uint8Type Cl_increment_consolebufferindex(void);
extern Cl_ReturnCodeType  Cl_Console_Read_Message_Ready_Status(void);
extern Cl_ReturnCodeType Cl_SendDatatoconsoleDummycommand(Cl_ConsoleTxCommandtype command, Cl_Uint8Type* data,Cl_Uint8Type datasize);


#endif /* CL_CONSOLECONTROLLER_EXT_H_ */