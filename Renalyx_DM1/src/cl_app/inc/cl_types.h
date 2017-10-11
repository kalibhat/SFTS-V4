/*
 * Cl_types.h
 *
 * Created: 12/27/2013 5:32:24 PM
 *  Author: sanjeer_k
 */ 



#ifndef CL_TYPES_H_
#define CL_TYPES_H_
#include "asf.h"
#define CONSOLE_BUFFER_SIZE 20 
#define CONSOLE_MSG_BUFFER_SIZE 40

#define Cl_BoolType bool
#define  Cl_Uint8Type uint8_t
#define  Cl_Uint32Type uint32_t
#define  Cl_Uint16Type uint16_t
#define  Cl_Int16Type int16_t


typedef enum 
{
	CL_OK,
	CL_WAIT,
	CL_ACCEPTED,
	CL_REJECTED,
	CL_ERROR
	
}Cl_ReturnCodeType;

typedef struct {
	Cl_Uint8Type	Console_databuffer[CONSOLE_BUFFER_SIZE];
	Cl_Uint8Type	Console_databuffer_write_idx;
	Cl_Uint8Type	Console_databuffer_read_idx;
	Cl_Uint8Type	BufferSize;
	Cl_BoolType		ConsoleNewData;
	Cl_BoolType		ConsoleDatabufferReady;
	Cl_BoolType		ConsoleDatabufferprocessed;
	Cl_BoolType		CurreltyActive;
	Cl_BoolType		StartSequenceDetected;
	Cl_BoolType		BufferEmpty;
	Cl_BoolType		Data_incomplete;
	
	
	} cl_console_bufferType;
typedef struct {
	Cl_Uint8Type	Console_databuffer[CONSOLE_MSG_BUFFER_SIZE];
	Cl_Uint8Type	Console_databuffer_write_idx;
	Cl_Uint8Type	Console_databuffer_read_idx;
	Cl_Uint8Type	Cmdptr;
	Cl_Uint8Type	BufferSize ;
	Cl_BoolType		ConsoleNewMessageStartReady;
	Cl_BoolType		ConsoleNewMessageReady;
	Cl_BoolType		ConsoleMsgRead;

	
} cl_console_msgbufferType;

typedef union
	{
		Cl_Uint8Type	byte[20] ;
		Cl_Uint16Type	dataTwobyte  ;
		Cl_Uint32Type	dataword ;
	
	}Cl_ConsoleDataType;
	
typedef struct{
	Cl_Uint8Type start1;
	Cl_Uint8Type start2;
	Cl_Uint8Type command;
	Cl_Uint8Type datasize;
	Cl_ConsoleDataType data;
	Cl_BoolType msgready;
	}Cl_ConsoleMsgType;


typedef struct{
	Cl_BoolType bloodpumpdoor;
	Cl_Uint16Type BPstatus;
	Cl_Uint16Type BP_INC;
	Cl_Uint16Type BP_DEC;
	Cl_Uint16Type BP_ON_OFF;
	Cl_Uint16Type Heparin_rotation_marker;
	Cl_Uint16Type Heparin_full_marker;
	Cl_Uint16Type Heparin_empty_marker;
	Cl_Uint16Type aptstatus;
	Cl_Uint16Type vptstatus;
	Cl_Uint16Type VC_status;
	Cl_Uint16Type VC_door_status;
	Cl_Uint16Type abdstatus;
	Cl_Uint16Type bdstatus;
	Cl_Uint16Type FPcurrentstatus;
	Cl_Uint16Type DGPstatus;
	Cl_Uint16Type UFPstatus;

	Cl_Uint16Type ps1status;
	Cl_Uint16Type ps2status;
	Cl_Uint16Type ps3status;	
	Cl_Uint16Type Temp1status;
	Cl_Uint16Type Temp2status;
	Cl_Uint16Type Temp3status;	
	Cl_Uint16Type Temp4status;
	Cl_Uint16Type CS3_Tempstatus;
	Cl_Uint16Type CS2_Tempstatus;
	Cl_Uint16Type CS1_Tempstatus;
		

	Cl_Uint16Type cond_status;
	Cl_Uint16Type cond_status_dac;
	Cl_Uint16Type cond_wien_status;
	Cl_Uint16Type cond_acid;
	Cl_Uint16Type cond_bicarb;

	Cl_BoolType levelswitch1;
	Cl_BoolType levelswitch2;
	Cl_Uint16Type Flowstatus;
	Cl_Uint16Type bldstatus;	

	Cl_BoolType holder1status;
	Cl_BoolType holder2status;
	Cl_BoolType acid_in;
	Cl_BoolType bicarb_in;
	Cl_Uint8Type calibration;
	Cl_Uint8Type acid_inlet_status;
	Cl_Uint8Type bicarb_inlet_status;
	Cl_Uint8Type CPU2_interupt;
	Cl_Uint8Type CPU3_interrupt;
	Cl_Uint8Type Redundancy_IRQ;
	

		
	Cl_Uint16Type cs1status;
	Cl_Uint16Type cs2status;


	Cl_Uint16Type ps4status;



	Cl_Uint16Type Flowcounter;
	Cl_Uint16Type PowerFailure;
	Cl_Uint16Type acid_fd_back;
    Cl_Uint16Type bicarb_fd_back;
	 Cl_Uint16Type v_c_F_B;
//	Cl_Uint16Type BloodPump_rotation_marker;

}Cl_Sys_statusType;
	
#endif /* CL_TYPES_H_ */
