/*
 * cl_debug.c
 *
 * Created: 2/5/2014 9:28:07 AM
 *  Author: user
 */ 

#include "cl_types.h"
#include "cl_consolecontroller.h"

Cl_ReturnCodeType Cl_Printf(Cl_Uint8Type* pstring, Cl_Uint8Type size);
extern Cl_ReturnCodeType  Cl_SendDatatoconsole(Cl_ConsoleTxCommandtype , Cl_Uint8Type* ,Cl_Uint8Type );


Cl_ReturnCodeType Cl_Printf(Cl_Uint8Type* pstring, Cl_Uint8Type size)
{
	
	Cl_ReturnCodeType Cl_RetVal;
	
	Cl_RetVal = Cl_SendDatatoconsole(CON_TX_COMMAND_PRINTTEXT,pstring,8);
	
	return Cl_RetVal;
}