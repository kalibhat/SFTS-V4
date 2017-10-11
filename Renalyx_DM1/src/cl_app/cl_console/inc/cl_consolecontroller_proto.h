/*
 * cl_consolecontroller_proto.h
 *
 * Created: 4/25/2017 12:53:43 PM
 *  Author: user
 */ 


#ifndef CL_CONSOLECONTROLLER_PROTO_H_
#define CL_CONSOLECONTROLLER_PROTO_H_

Cl_ReturnCodeType		Cl_Console_Read_ConsoleMessage_Status(void);
Cl_ReturnCodeType		 Cl_Console_Read_Message_Console(void);
Cl_ReturnCodeType        Cl_SendDatatoconsole(Cl_ConsoleTxCommandtype , Cl_Uint8Type* ,Cl_Uint8Type );
Cl_ReturnCodeType		 Cl_MacWrite_Message_Console(void);
Cl_ReturnCodeType	 Cl_Mac_Create_ConsoleEvent(void);
Cl_ReturnCodeType  Cl_ScanForMsgStart(void);
Cl_ReturnCodeType Cl_Cosnsole_MsgAcknlowledge(void);
Cl_ReturnCodeType  cl_console_translatemacevent(Cl_Mac_EventsType ,Cl_ConsoleRxEventsType );
Cl_ReturnCodeType	 Cl_ConsoleStateHandler(Cl_Mac_EventsType );
Cl_ReturnCodeType 	Cl_Console_Console_Sync(void);
Cl_ReturnCodeType Cl_Console_init(void);
Cl_ReturnCodeType Cl_Console_SyncGetMessage(void);
Cl_Uint8Type Cl_increment_msgbufferindex(void);
Cl_Uint8Type Cl_increment_consolebufferindex(void);
Cl_ReturnCodeType  Cl_Console_Read_Message_Ready_Status(void);
Cl_ReturnCodeType Cl_SendDatatoconsoleDummycommand(Cl_ConsoleTxCommandtype command, Cl_Uint8Type* data,Cl_Uint8Type datasize);

Cl_Uint8Type memcopy(Cl_Uint8Type* dest, Cl_Uint8Type* source, Cl_Uint32Type size);



#endif /* CL_CONSOLECONTROLLER_PROTO_H_ */