/*
 * cl_consolehandler.c
 *
 * Created: 12/27/2013 5:21:35 PM
 *  Author: user
 */ 

#include "cl_consolecontroller_include.h"
#include "cl_consolecontroller_proto.h"
#include "sv_stubs/inc/sv_stub.h"


#define  IIC_CONSOLE_ADDR 0x55
extern Cl_Mac_EventQueueType Cl_MacEventQueue3;
extern Cl_BoolType Cl_MacNewEvent;
extern  Cl_Uint8Type testCl_BoolType ;
extern cl_console_bufferType Cl_Console_buffer,Cl_Console_buffer1,Cl_Console_buffer2;
extern cl_console_msgbufferType Cl_Console_msgbuffer;
extern Cl_BoolType  check_endofsequence_is_received(Cl_Uint8Type  );
extern Cl_BoolType  check_startofsequence_is_received(Cl_Uint8Type  );
extern Cl_ReturnCodeType cl_testharnesscommandhandler(Cl_ConsoleMsgType*);
extern void DD_IIC_CONSOLE_SEND(Cl_Uint8Type iic_address, Cl_Uint8Type* data,Cl_Uint8Type length);
Cl_Uint8Type sv_console_bytedatawrite(Cl_Uint8Type sv_consoleTxdatabyte);
extern Cl_Mac_EventsType Cl_MacEvent;
Cl_ReturnCodeType Cl_Console_ConvertConsoleEvent_toMacEvent(Cl_ConsoleRxEventsType , Cl_Mac_EventsType*);
Cl_Cl_ConsoleStateType Cl_ConsoleState = CON_STATE_INIT;
Cl_BoolType syncdone = false;

static Cl_Uint16Type test_counter = 0;
extern Cl_ConsoleMsgType Cl_ConsoleRxMsg;
extern Cl_Uint8Type Cl_ConsoleTxMsg[40];
extern Cl_Uint8Type Cl_IIC_ConsoleTxMsg[40];
extern array_command_type cmd_backup;
//extern  DD_SET_PIO(pio_type peripheral,Cl_Uint32Type pio);

static Cl_ConsoleMsg_readStateType Cl_ConsoleMsg_readState = MSG_RX_IDLE;
static Cl_BoolType ConsoleSync = false;


int8_t test_count = 0 ;
extern Cl_ReturnCodeType cl_wait(Cl_Uint32Type ul_dly_ticks);

bool cl_console_sync = false;


Cl_ConsoleTxCommandtype ackCommand;


Cl_Uint8Type loopackflag = false;

Cl_ConsoleTxCommandtype cur_command;
Cl_Uint8Type cur_data ;
Cl_Uint8Type cur_datasize;

/*
 * \brief  check a new console message has arrived in the consoledatabuffer
		   
 * \void function 
 * \return CL_OK on success. CL_WAIT if no message yet
 * \Globals used : Cl_Console_buffer1,Cl_Console_buffer2
 */

Cl_ReturnCodeType Cl_Console_Read_ConsoleMessage_Status(void)
{
	
	//Cl_ReturnCodeType Cl_ConsoleRetval;
	Cl_ReturnCodeType Cl_ConsoleRetval;
	Cl_ConsoleRetval = CL_ERROR;
	
	

	if(Cl_Console_buffer1.ConsoleDatabufferReady)
	{
			Cl_ConsoleRetval = CL_OK;
	}
	else if(Cl_Console_buffer2.ConsoleDatabufferReady)
	{
		Cl_ConsoleRetval = CL_OK;
	}
	else
	{
			Cl_ConsoleRetval =  CL_WAIT;
			//Cl_ConsoleRetval =  10;
			
	}
	return Cl_ConsoleRetval;
}
/*
 * \brief  check a new console message has arrived in the consolemessageabuffer
		   
 * \void function 
 * \return CL_OK on success. CL_WAIT if no message yet
 * \Globals used : Cl_Console_msgbuffer
 */
Cl_ReturnCodeType  Cl_Console_Read_Message_Ready_Status(void)
{
	
	if(Cl_Console_msgbuffer.ConsoleNewMessageReady == true)
	{

//printf("RDY\n");
	//for (tempcount=0;tempcount<Cl_Console_msgbuffer.Console_databuffer_write_idx;tempcount++)

		return CL_OK;
	}
	else
	{
//		printf("WT\n");
		return CL_WAIT;
	}
	
}
/*
 * \brief  copy message from databuffer to messagebuffer 
		   
 * \void function 
 * \return CL_OK on success. 
 * \Globals used : Cl_Console_msgbuffer,Cl_Console_buffer1,Cl_Console_buffer2
 */

Cl_ReturnCodeType		Cl_Console_Read_Message_Console()
{

	static Cl_Uint8Type tempcount=0;
	Cl_Uint8Type temp;

if((Cl_Console_buffer1.ConsoleDatabufferReady == true) && (Cl_Console_buffer1.ConsoleDatabufferprocessed == false))
{
						
	for( tempcount=0,temp=0;temp < Cl_Console_buffer1.Console_databuffer_write_idx;tempcount ++,temp++ )
	{
		Cl_Console_msgbuffer.Console_databuffer[tempcount]=Cl_Console_buffer1.Console_databuffer[temp];
							

	}
	if(!Cl_Console_buffer1.Data_incomplete)
	{
			Cl_Console_msgbuffer.ConsoleNewMessageReady = true;
	}
			
			Cl_Console_buffer1.ConsoleDatabufferprocessed = true;
			Cl_Console_buffer1.ConsoleDatabufferReady=false;
			Cl_Console_buffer1.StartSequenceDetected = false;
			Cl_Console_buffer1.Console_databuffer_write_idx =0;
			Cl_Console_buffer1.BufferEmpty = true;
	
}
if((Cl_Console_buffer2.ConsoleDatabufferReady == true) && (Cl_Console_buffer2.ConsoleDatabufferprocessed == false))
{
	if(!Cl_Console_buffer1.Data_incomplete)
	{
		tempcount=0;
	}
	else
	{
		// carry on appending to prevdata.
	}
	for( temp=0;temp < Cl_Console_buffer2.Console_databuffer_write_idx;tempcount ++,temp++ )
	{
		Cl_Console_msgbuffer.Console_databuffer[tempcount]=Cl_Console_buffer2.Console_databuffer[temp];

	}
		if(!Cl_Console_buffer2.Data_incomplete)
		{
			Cl_Console_msgbuffer.ConsoleNewMessageReady = true;
		}
	
		
			Cl_Console_buffer2.ConsoleDatabufferprocessed = true;
			Cl_Console_buffer2.ConsoleDatabufferReady=false;
			Cl_Console_buffer2.Console_databuffer_write_idx =0;
			Cl_Console_buffer2.StartSequenceDetected = false;
			Cl_Console_buffer2.BufferEmpty = true;
			if(Cl_Console_buffer1.Data_incomplete)
			{
				Cl_Console_buffer1.ConsoleDatabufferprocessed = true;
				Cl_Console_buffer1.ConsoleDatabufferReady=false;
				Cl_Console_buffer1.StartSequenceDetected = false;
				Cl_Console_buffer1.Data_incomplete = false;
				Cl_Console_buffer2.Data_incomplete = false;
				Cl_Console_buffer1.Console_databuffer_write_idx =0;
				Cl_Console_buffer1.BufferEmpty = true;
			}
}


	return CL_OK;
}

/*
 * \brief  copy message from databuffer to messagebuffer 
		   
 * \void function 
 * \return CL_OK on success. 
 * \Globals used : Cl_Console_msgbuffer,Cl_Console_buffer1,Cl_Console_buffer2
 */

Cl_ReturnCodeType	 Cl_Mac_Create_ConsoleEvent(void)
{
//	Cl_Uint8Type start1,start2,command,datasize,data1,data2,end1,end2;
	//Cl_ConsoleMsgType Cl_ConsoleMsg;
	static Cl_Uint8Type* Cl_cmdptr;
	Cl_Uint8Type tempcount,packet_size =0, *dataptr= NULL, *msgdataptr = NULL;
	Cl_Mac_EventsType MacEvent;
	//printf("#/n");
	
	
			
			Cl_cmdptr = (Cl_Uint8Type*)&Cl_Console_msgbuffer.Console_databuffer;
				
			Cl_ConsoleRxMsg.command = Cl_Console_msgbuffer.Console_databuffer[0];
			Cl_ConsoleRxMsg.datasize = Cl_Console_msgbuffer.Console_databuffer[1];
			msgdataptr = &Cl_Console_msgbuffer.Console_databuffer[2];
			dataptr = &Cl_ConsoleRxMsg.data.byte[0];
			for(tempcount=0;tempcount<Cl_ConsoleRxMsg.datasize;tempcount++)
			{
				 *dataptr = *msgdataptr ;
				 dataptr++;
				 msgdataptr++;
			
			}
			

						

			if(Cl_ConsoleRxMsg.command != CON_RX_COMMAND_ACK)
			{
			//	test_count--;
			//	if(test_count  != 0)
			//	{
			//		PIOC->PIO_SODR = 1 << 28; //test
			//	}
				Cl_Cosnsole_MsgAcknlowledge();
				test_counter++;
				if (test_counter >= 1000)
				{

				//	DD_SET_PIO(PD,7); // test
				//	PIOB->PIO_SODR = 1 << 25; //test
				//	PIOC->PIO_CODR = 1 << 29; //test
					test_counter = 0;
				}
				else
				if(test_counter > 2)
				{
					//DD_RESET_PIO(PD,7); // test
				//	PIOC->PIO_SODR = 1 << 29; //test
				}
			}
			else
			{
				// do nothing
				//	return CL_OK;
			}

			 if(Cl_ConsoleRxMsg.command == CON_RX_COMMAND_TEST_HARNESS)
			 		{
					
					cl_testharnesscommandhandler(&Cl_ConsoleRxMsg);	 
					}
			else
			{
			
			Cl_ConsoleRxMsg.msgready = true;
			Cl_Console_msgbuffer.ConsoleNewMessageReady = false;
		
			Cl_Console_ConvertConsoleEvent_toMacEvent(Cl_ConsoleRxMsg.command,&MacEvent);
			Cl_MacEvent = MacEvent;
			Cl_MacNewEvent = true;
			if( Cl_MacEventQueue3.Cl_MacNewEventcount <= MAC_EVENT_COUNT_MAX )	
			 {
				 Cl_MacEventQueue3.Cl_MacEvent[Cl_MacEventQueue3.Cl_MacNewEventcount] = MacEvent ;
				 Cl_MacEventQueue3.Cl_MacNewEvent = true;
				 Cl_MacEventQueue3.Cl_MacNewEventcount++;
				 
				msgdataptr = &Cl_Console_msgbuffer.Console_databuffer[2];
				dataptr = &Cl_MacEventQueue3.Cl_MacEventData[Cl_MacEventQueue3.Cl_MacNewEventcount][0];
				for(tempcount=0;tempcount<Cl_ConsoleRxMsg.datasize;tempcount++)
				{
					*dataptr = *msgdataptr ;
					dataptr++;
					msgdataptr++;
					 			
				}
				 			
			 }
			 
				
			}
			
	if(syncdone)
	{
		
	#if 0
	
			Cl_IIC_ConsoleTxMsg[0] = 0x99 ;
	
			Cl_IIC_ConsoleTxMsg[1] = 0xCC ;
			Cl_IIC_ConsoleTxMsg[2] = 0xDD ;
			msgdataptr = &Cl_Console_msgbuffer.Console_databuffer[0];
			for(tempcount = 0 ; tempcount< Cl_ConsoleRxMsg.datasize+3;tempcount++)
			{
				Cl_IIC_ConsoleTxMsg[tempcount+3] = *msgdataptr++;
		
			}
			//Cl_IIC_ConsoleTxMsg[tempcount+3] = 0xEE;
			Cl_IIC_ConsoleTxMsg[tempcount+3] = 0xBB;
			packet_size = Cl_ConsoleRxMsg.datasize + 7;
			{
			//	if ( (command != CON_TX_COMMAND_COMMAND_SCRIPT_BULK_PRINT) && (command != CON_TX_COMMAND_COMMAND_SCRIPT_PRNIT) &&(command != CON_TX_COMMAND_PRINTDATA) && (command != CON_TX_COMMAND_PRINTTEXT))
				{
			//		DD_IIC_CONSOLE_SEND(0x77, &Cl_IIC_ConsoleTxMsg, packet_size);
				}
		
			}
	#endif
			 		
	}
		
		return CL_OK;
}

/*
 * \brief  copy date from source to destination 
 * \param  dest - destination buffer , source - source buffer , size - num of bytes to be copied	   
 * \void function 
 * \return CL_OK on success. 
 * \Globals used : 
 */


Cl_ReturnCodeType memcopy(Cl_Uint8Type* dest, Cl_Uint8Type* source, Cl_Uint32Type size)
{
	Cl_Uint32Type count = 0;
	for (count = 0;count< size; count++ )
	{
		dest[count]= source[count];
	}
	
	return 1;
}

Cl_ReturnCodeType Cl_increment_msgbufferindex()
{
	
	if(Cl_Console_msgbuffer.Console_databuffer_write_idx < CONSOLE_BUFFER_SIZE)
	Cl_Console_msgbuffer.Console_databuffer_write_idx++;
	else
	Cl_Console_msgbuffer.Console_databuffer_write_idx = 0;
	return 1;
}

Cl_ReturnCodeType Cl_increment_consolebufferindex()
{
	
	if(Cl_Console_buffer.Console_databuffer_read_idx < CONSOLE_BUFFER_SIZE)
	Cl_Console_buffer.Console_databuffer_read_idx++;
	else
	Cl_Console_buffer.Console_databuffer_read_idx = 0;
		return 1;
}



/*
 * \brief  send acknowledgment  to console 
 * \param  	   
 * \ 
 * \return CL_OK on success. 
 * \Globals used : 
 */




Cl_ReturnCodeType Cl_Cosnsole_MsgAcknlowledge()
   
	{
		Cl_Uint8Type temp;

		cl_wait(1);
		Cl_ConsoleTxMsg[0] =	CL_CONSOLEMSG_STARTBYTE1;
		Cl_ConsoleTxMsg[1] =	CL_CONSOLEMSG_STARTBYTE2;
		Cl_ConsoleTxMsg[2]=		CON_TX_COMMAND_ACK;
		Cl_ConsoleTxMsg[3] = 1; //size of data
		Cl_ConsoleTxMsg[4]= Cl_ConsoleRxMsg.command;
		Cl_ConsoleTxMsg[5] =	CL_CONSOLEMSG_STOPBYTE1;
		Cl_ConsoleTxMsg[6] =	CL_CONSOLEMSG_STOPBYTE2;
		for (temp=0;temp<7;)
		{
				cl_console_sync = 1;
			if(! uart_write(CONF_UART,Cl_ConsoleTxMsg[temp]))
			{
				temp++;				
			}
		}
	return CL_OK;
}

/*
 * \brief  send data  to console 
 * \param  	   
 * \ 
 * \return CL_OK on success. 
 * \Globals used : 
 */

Cl_ReturnCodeType Cl_SendDatatoconsole(Cl_ConsoleTxCommandtype command, Cl_Uint8Type* data,Cl_Uint8Type datasize)
{
	Cl_Uint8Type tempcount=0 , temp =4 ;
	Cl_Uint8Type packet_size = 0;
	Cl_Uint8Type *dataptr=NULL, *msgdataptr =NULL;
	static Cl_Uint8Type testcnt =0;
	


		
			if(( CON_TX_COMMAND_COMMAND_SCRIPT_PRNIT == command)|| ( command == CON_TX_COMMAND_PRINTDATA) ||(command == CON_TX_COMMAND_PRINTTEXT) )// || (command == CON_TX_COMMAND_COMMAND_SCRIPT_BULK_PRINT))
		//	if(( CON_TX_COMMAND_COMMAND_SCRIPT_PRNIT == command)|| ( command == CON_TX_COMMAND_PRINTDATA) ||(command == CON_TX_COMMAND_PRINTTEXT) ||(command == CON_TX_COMMAND_COMMAND_SYSTEM_STATE )  || (command == CON_TX_COMMAND_COMMAND_SCRIPT_BULK_PRINT) || (command == CON_TX_COMMAND_ALARM))
		
		//	if(( CON_TX_COMMAND_COMMAND_SCRIPT_PRNIT == command)|| ( command == CON_TX_COMMAND_PRINTDATA) ||(command == CON_TX_COMMAND_PRINTTEXT) ) //|| (command == CON_TX_COMMAND_ALARM)
			{
				return;  /**************  To be returned for proper working with tablet ******/////  comment return to work with script
			}
		
	if( ( command != CON_TX_COMMAND_PRINTDATA) && (command != CON_TX_COMMAND_PRINTTEXT) )
	{
	//	cmd_backup.cmdtype[cmd_backup.array_commands_index].commands_inouttype = 1;
	//	cmd_backup.cmdtype[cmd_backup.array_commands_index].command =  command;
	//	cmd_backup.array_commands_index++;
	//	if(cmd_backup.array_commands_index == 100)
		{
	//		cmd_backup.array_commands_index = 0;
		}
		
		
	}
	
	cur_command = command;
	cur_data = data;
	cur_datasize = datasize;
	
	
	if (CONT_TX_COMMAND_BLOODPUMP_OFF == command)
	{
		syncdone = true;
	}

	if(datasize>0)
	{
		if(data != NULL)
		{
			dataptr = data;	
		}
			
	}
	
	packet_size = datasize + 6;
	msgdataptr = &Cl_ConsoleTxMsg[4];
	Cl_ConsoleTxMsg[0] = CL_CONSOLEMSG_STARTBYTE1 ;//(Cl_Uint8Type) 'X';
	Cl_ConsoleTxMsg[1] = CL_CONSOLEMSG_STARTBYTE2 ;// (Cl_Uint8Type)'Q';
	Cl_ConsoleTxMsg[2]= command;// command
	Cl_ConsoleTxMsg[3] = datasize; //size of data
	for(tempcount = 0 ; tempcount< datasize;tempcount++)
	{
		Cl_ConsoleTxMsg[temp] = data[tempcount];
		temp++;
	}
	
	Cl_ConsoleTxMsg[temp++] = CL_CONSOLEMSG_STOPBYTE1; // (Cl_Uint8Type)'D';
	Cl_ConsoleTxMsg[temp++] = CL_CONSOLEMSG_STOPBYTE2 ;// (Cl_Uint8Type)'F';
	
		if(syncdone)
		{
			
			
			Cl_IIC_ConsoleTxMsg[0] = 0xAA ;
			for(tempcount = 0 ; tempcount< packet_size;tempcount++)
			{
				Cl_IIC_ConsoleTxMsg[tempcount+1] = Cl_ConsoleTxMsg[tempcount];
				
			}
			
			{
				//if ( (command != CON_TX_COMMAND_COMMAND_SCRIPT_BULK_PRINT) && (command != CON_TX_COMMAND_COMMAND_SCRIPT_PRNIT) &&(command != CON_TX_COMMAND_PRINTDATA) && (command != CON_TX_COMMAND_PRINTTEXT))
				//if ( (command != CON_TX_COMMAND_COMMAND_SCRIPT_BULK_PRINT) && (command != CON_TX_COMMAND_COMMAND_SCRIPT_PRNIT) &&(command != CON_TX_COMMAND_PRINTDATA) && (command != CON_TX_COMMAND_PRINTTEXT))
				if ( (command != CON_TX_COMMAND_COMMAND_IIC_HANGUP) )
				{
				//	DD_IIC_CONSOLE_SEND(0x77, &Cl_IIC_ConsoleTxMsg, packet_size+1);
				}

			}
			
			
		}
		
		
	for (tempcount=0;tempcount<packet_size;)
	{
	//	if(tempcount == 0)
	//	{
	//			PIOC->PIO_SODR = 1 << 29; //test
	//	}
	//	else if (tempcount == ( packet_size - 1 ))
	//	{
	//			PIOC->PIO_CODR = 1 << 29; //test
	//	}
		
	cl_console_sync = 1;
	//	if(! uart_write(CONF_UART,Cl_ConsoleTxMsg[tempcount]))
	if(! sv_console_bytedatawrite(Cl_ConsoleTxMsg[tempcount]))
	
		{
			tempcount++;
			
		}
		
		
		//	printf("%s",Cl_ConsoleTxMsg);
		//printf("\n");
		
	}
	
ackCommand = command;	

//	printf("\n");
//cl_wait(5);                     // check if needed 21082017
	return CL_OK;
}

/*
 * \brief  send data  to console 
 * \param  	   
 * \ 
 * \return CL_OK on success. 
 * \Globals used : 
 */

Cl_ReturnCodeType Cl_SendDatatoconsoleDummycommand(Cl_ConsoleTxCommandtype command, Cl_Uint8Type* data,Cl_Uint8Type datasize)
{
	Cl_Uint8Type tempcount=0 , temp =4 ;
	Cl_Uint8Type packet_size = 0;
	Cl_Uint8Type *dataptr=NULL, *msgdataptr =NULL;
	
	dataptr = data;
	packet_size = 8;
	msgdataptr = &Cl_ConsoleTxMsg[4];
	Cl_ConsoleTxMsg[0] = command ;//(Cl_Uint8Type) 'X';
	Cl_ConsoleTxMsg[1] = CL_CONSOLEMSG_STARTBYTE1 ;//(Cl_Uint8Type) 'X';
	Cl_ConsoleTxMsg[2] = CL_CONSOLEMSG_STARTBYTE2 ;// (Cl_Uint8Type)'Q';
	if( command == 0xAA)
	{
		Cl_ConsoleTxMsg[3]=  0x66;
	}
	else
	{
		Cl_ConsoleTxMsg[3]=  0x77;
	}
	
	Cl_ConsoleTxMsg[4] = 0x01; //size of data
	Cl_ConsoleTxMsg[5] = command + 0x01;
	Cl_ConsoleTxMsg[6] = CL_CONSOLEMSG_STOPBYTE1; // (Cl_Uint8Type)'D';
	Cl_ConsoleTxMsg[7] = CL_CONSOLEMSG_STOPBYTE2 ;// (Cl_Uint8Type)'F';
	for (tempcount=0;tempcount<packet_size;)
	{
		if(! uart_write(CONF_UART,Cl_ConsoleTxMsg[tempcount]))
		{
			tempcount++;
			
		}
		
		
		//	printf("%s",Cl_ConsoleTxMsg);
		//printf("\n");
		
	}
	//printf("\n");
	return CL_OK;
}

/*
 * \brief  ConvertConsoleEvent_toMacEvent
 * \param  command - console event to be converted , MacEvent - MAC event generated	   
 * \ 
 * \return CL_OK on success. 
 * \Globals used : 
 */
Cl_ReturnCodeType Cl_Console_ConvertConsoleEvent_toMacEvent(Cl_ConsoleRxEventsType command ,Cl_Mac_EventsType* MacEvent)
{

	switch(command)
	{
		case CON_RX_COMMAND_SYNC:
		//printf("J/n");
		    *MacEvent = EVT_CONSOLE_COMMAND_SYNC;
		break;
		case CON_RX_COMMAND_CLR_ALARM:
		*MacEvent = EVT_CONSOLE_COMMAND_CLR_ALARM;
		break;
		case CON_RX_COMMAND_ACK:
		  *MacEvent = EVT_CONSOLE_COMMAND_ACK;
		break;
		case CON_RX_COMMAND_RINSE_START:
//		printf("J/n");
			*MacEvent = EVT_CONSOLE_COMMAND_RINSE_START;
		break;
		case CON_RX_EVT_COMMAND_START_DISINF:
		
		*MacEvent = EVT_CONSOLE_COMMAND_DISINF_START;
		break;
		case CON_RX_EVT_COMMAND_RESUME_DISINF_STAGE:
		*MacEvent = EVT_CONSOLE_RESUME_DISINF_STAGE;
		break;
		case CON_RX_COMMAND_SET_BLDPMP_ON:
		*MacEvent = EVT_CONSOLE_COMMAND_SET_BLDPMP_ON;
		break;
		case CON_RX_COMMAND_SET_BLDPMP_OFF:
		*MacEvent = EVT_CONSOLE_COMMAND_SET_BLDPMP_OFF;
		break;
		case CON_RX_COMMAND_SET_BLDPUMPRATE:
		*MacEvent = EVT_CONSOLE_COMMAND_SET_BLDPUMPRATE;
		break;
		case CON_RX_COMMAND_START_DIALYSATE_FILL:
		*MacEvent = EVT_CONSOLE_COMMAND_START_DIALYSATE_FILLING;
		break;
		case CON_RX_COMMAND_SET_DATA:
		*MacEvent = EVT_CONSOLE_COMMAND_SET_DATA;
		break;
		case CON_RX_COMMAND_GET_DATA:
		*MacEvent = EVT_CONSOLE_COMMAND_GET_DATA;
		break;
		case CON_RX_COMMAND_START_PRIME:
		*MacEvent = EVT_CONSOLE_COMMAND_START_PRIME;
		break;
		
		case CON_RX_COMMAND_STOP_PRIME:
		*MacEvent = EVT_CONSOLE_COMMAND_STOP_PRIME;
		break;
		
		case CON_RX_COMMAND_START_DIALISER_PRIME:
		*MacEvent = EVT_CONSOLE_COMMAND_START_DIALISER_PRIME;
		break;
		
		case CON_RX_EVT_COMMAND_STOP_DIALYSER_PRIME:
		*MacEvent = EVT_CONSOLE_COMMAND_STOP_DIALISER_PRIME;
		break;
		case CON_RX_COMMAND_HEP_PMP_RATE:
		*MacEvent = EVT_CONSOLE_COMMAND_HEP_PMP_RATE;
		break;
		case CON_RX_COMMAND_HEPARIN_BOLUS:
		*MacEvent = EVT_CONSOLE_COMMAND_HEPARIN_BOLUS;
		break;
		case CON_RX_COMMAND_HEPARIN_ON:
		*MacEvent = EVT_CONSOLE_COMMAND_HEPARIN_START;
		break;
		case CON_RX_COMMAND_HEPARIN_OFF:
		*MacEvent = EVT_CONSOLE_COMMAND_HEPARIN_STOP;
		break;
		case CON_RX_COMMAND_DIALYSIS_PREP:
			*MacEvent = EVT_CONSOLE_COMMAND_DIALYSIS_PREP;
		break;
		
		case CON_RX_COMMAND_DIALYSER_CONNECTED:
		*MacEvent = EVT_CONSOLE_COMMAND_DILYSER_CONNECTED;
		break;
		case CON_RX_COMMAND_MAN_PREP_COMPLETED:
			*MacEvent = EVT_CONSOLE_COMMAND_MAN_PREP_COMPLETED;
		break;
		case CON_RX_COMMAND_DIALYSIS_START:
			*MacEvent = EVT_CONSOLE_COMMAND_DIALYSIS_START;
		break;
		
		case CON_RX_COMMAND_SELFTEST_START:        //for selftest
		     *MacEvent = EVT_CONSOLE_COMMAND_SELFTEST_START;
			 break;
			 
			case CON_RX_COMMAND_SELFTEST_ALARM_OK:
			*MacEvent = EVT_COSOLE_COMMAND_SELFTEST_ALARM_PAAS;
			 break;
			 	case CON_RX_COMMAND_SELFTEST_ALARM_FAIL:
			 	*MacEvent = EVT_COSOLE_COMMAND_SELFTEST_ALARM_FAIL;
			 	break;
		case CON_RX_COMMAND_SELFTEST_STOP:        //for selftest
		*MacEvent = EVT_CONSOLE_COMMAND_SELFTEST_STOP;
		break;
		case CON_RX_COMMAND_SYNCCONFIMED:
		*MacEvent = EVT_CONSOLE_COMMAND_SYNCCONFIRMED;
		
		break;
		case CON_RX_COMMAND_DIALYSIS_STOP:
			*MacEvent = EVT_CONSOLE_COMMAND_DIALYSIS_STOP;
		break;
		case CON_RX_COMMAND_DIALYSIS_PAUSE:
			*MacEvent = EVT_CONSOLE_COMMAND_DIALYSIS_PAUSE;
		break;
		
		case CON_RX_EVT_COMMAND_BYPASS_ON:
			*MacEvent = EVT_CONSOLE_COMMAND_BYPASS_ON;
		break;
		case CON_RX_EVT_COMMAND_BYPASS_OFF:
		*MacEvent = EVT_CONSOLE_COMMAND_BYPASS_OFF;
		break;
		case CON_RX_COMMAND_RINSE_STOP:
			*MacEvent = EVT_CONSOLE_COMMAND_STOP_RINSE;
		break;
		
		case		CON_RX_COMMAND_USER_MODE_ENTER://1A
					*MacEvent =  EVT_CONSOLE_COMMAND_USER_MODE_ENTER;
		break;
		case		CON_RX_COMMAND_USER_MODE_EXIT://1B
					*MacEvent =  EVT_CONSOLE_COMMAND_USER_MODE_EXIT;
		break;
		case		CON_RX_COMMAND_SERVICEMODE_ENTER://1C
					*MacEvent =  EVT_CONSOLE_COMMAND_SERVICEMODE_ENTER ;
		break;
		case		CON_RX_COMMAND_SERVICEMODE_EXIT://1D
					*MacEvent = EVT_CONSOLE_COMMAND_SERVICEMODE_EXIT;
		break;
					
		case    CON_RX_EVT_COMMAND_PATIENT_CONNECTED: //1E
					*MacEvent = EVT_CONSOLE_COMMAND_PATIENT_CONNECTED;
		break;
		case 	CON_RX_EVT_COMMAND_MIXING_PREP_START://0x21
					*MacEvent = EVT_SEND_MIXING_PREP_START;
		break;
		case	CON_RX_EVT_COMMAND_MIXING_PREP_STOP://0x22
			*MacEvent = EVT_SEND_MIXING_PREP_STOP;
		break;

		case CON_RX_EVT_COMMAND_RCIRC_START: 
					*MacEvent = EVT_CONSOLE_COMMAND_RCIRC_START;
		break;
		case CON_RX_EVT_COMMAND_RCIRC_STOP: 
					*MacEvent = EVT_CONSOLE_COMMAND_RCIRC_STOP;
		break;
		case CON_RX_EVT_COMMAND_PATIENT_READY:
		*MacEvent = EVT_CONSOLE_COMMAND_PATIENT_READY;
		break;
		
		case CON_RX_EVT_COMMAND_START_RETURN:
		*MacEvent = EVT_CONSOLE_COMMAND_START_RETURN;
		break;
		
		case CON_RX_EVT_COMMAND_STOP_RETURN:
		*MacEvent = EVT_CONSOLE_COMMAND_STOP_RETURN;
		break;			
		case CON_RX_EVT_COMMAND_MINUF_ON:
		*MacEvent = EVT_CONSOLE_COMMAND_MINUF_ON;
		break;
		case CON_RX_EVT_COMMAND_MINUF_OFF:
		*MacEvent = EVT_CONSOLE_COMMAND_MINUF_OFF;
		break;
		case CON_RX_COMMAND_ISOUF_START:
		*MacEvent = EVT_CONSOLE_COMMAND_ISOUF_START;
		break;
		case CON_RX_COMMAND_ISOUF_STOP:
		*MacEvent = EVT_CONSOLE_COMMAND_ISOUF_STOP;
		break;
		case CON_RX_COMMAND_MUTE:
		*MacEvent = EVT_CONSOLE_COMMAND_MUTE_ALARM;
		break;
		case CON_RX_EVT_COMMAND_DLSIS_ISOUF_ON :
		*MacEvent =EVT_CONSOLE_COMMAND_DIALYSIS_ISOUF_ON;
		break;
		case CON_RX_EVT_COMMAND_DLSIS_ISOUF_OFF:
		*MacEvent =EVT_CONSOLE_COMMAND_DIALYSIS_ISOUF_OFF;
		break;
		default:
		break;
	}
	return CL_OK;
}

/*
 * \brief  State handler for console events
 * \param  command - cMacEvent -  for corresponding console events	   
 * \ 
 * \return CL_OK on success. 
 * \Globals used : 
 */
Cl_ReturnCodeType	 Cl_ConsoleStateHandler(Cl_Mac_EventsType Cl_MacConsoleEvent)
{
	Cl_ConsoleTxCommandtype command = CON_TX_COMMAND_COMMAND_MAX;
	Cl_ConsoleRxEventsType cl_console_event = CON_RX_COMMANDMAX;
	Cl_Uint8Type data =0;
	cl_console_translatemacevent( Cl_MacConsoleEvent, cl_console_event);
	switch (Cl_ConsoleState)
	{
		case CON_STATE_INIT:
		switch(Cl_MacConsoleEvent)
		{
			case CON_RX_COMMAND_SYNC:
			command = CON_TX_COMMAND_SYNC;
			 Cl_SendDatatoconsole(command,&data,0);
			break;
			default:
			break;
		}
		break;
		case CON_STATE_IDLE:
		break;
		case CON_STATE_ACTIVE:
			case CON_RX_COMMAND_SYNC:
			command = CON_TX_COMMAND_SYNC;
			Cl_SendDatatoconsole(command,&data,0);
			break;
		default:
		break;
	}
	
	return CL_OK;
}

/*
 * \brief  Translate MAC Events to console events
 * \param  Cl_MacRinseEvent - cMacEvent ; cl_consoleeevent for storing converted corresponding MAC events	   
 * \ 
 * \return CL_OK on success. 
 * \Globals used : 
 */
Cl_ReturnCodeType  cl_console_translatemacevent(Cl_Mac_EventsType Cl_MacRinseEvent,Cl_ConsoleRxEventsType cl_consoleeevent)
{
	switch(Cl_MacRinseEvent)
	{
		case EVT_CONSOLE_COMMAND_SYNC:
				cl_consoleeevent= CON_RX_COMMAND_SYNC;
		break;
		case EVT_CONSOLE_COMMAND_SET_DATA:
				cl_consoleeevent=CON_RX_COMMAND_SET_DATA;
		break;
		case EVT_CONSOLE_COMMAND_GET_DATA:
				cl_consoleeevent=CON_RX_COMMAND_GET_DATA;
		break;
		case EVT_CONSOLE_COMMAND_RINSE_START:
				cl_consoleeevent=CON_RX_COMMAND_RINSE_START;
		break;
		case EVT_CONSOLE_COMMAND_DIALYSIS_PREP:
				cl_consoleeevent=CON_RX_COMMAND_DIALYSIS_PREP;
		break;

		case EVT_CONSOLE_COMMAND_DIALYSIS_START:
				cl_consoleeevent=CON_RX_COMMAND_DIALYSIS_START;
		break;
		
		case EVT_CONSOLE_COMMAND_CLR_ALARM:
		cl_consoleeevent=CON_RX_COMMAND_CLR_ALARM;
		break;

		
		default:
		break;
	}
	return CL_OK;
}
/*
 * \brief  State MAchine Implementaion for Sync process
 * \param  
 * \ 
 * \return CL_OK on success. 
 * \Globals used : 
 */
Cl_ReturnCodeType 	Cl_Console_Console_Sync(void)
{
	Cl_ReturnCodeType Cl_MacRetval = CL_ERROR;
	Cl_ConsoleSyncStatesType Cl_SyncState = CON_SYNC_STATE_WAIT_FOR_SYNC;
	
	Cl_Uint8Type CmdData =0;
	

	while(	Cl_ConsoleState == CON_STATE_INIT )
	{
		switch(Cl_SyncState)
		{
			case CON_SYNC_STATE_WAIT_FOR_SYNC:
			
			Cl_MacRetval = Cl_Console_SyncGetMessage();

						if(Cl_MacRetval== CL_OK)
						{							
							Cl_ConsoleRxMsg.command = Cl_Console_msgbuffer.Console_databuffer[0];
							//Cl_ConsoleRxMsg.command = CON_HELLO;// for testing
							if(Cl_ConsoleRxMsg.command == CON_RX_COMMAND_SYNC )
							{
								
									Cl_Cosnsole_MsgAcknlowledge();
							//		loopackflag = true;
								
								
							//	count = 0;

								Cl_ConsoleRxMsg.msgready = true;
								Cl_Console_msgbuffer.ConsoleNewMessageReady = false;
								Cl_SyncState = CON_SYNC_STATE_SYNC_RECVD;
								//delay and send sync
								
							
							}
						}		
			Cl_MacRetval = CL_WAIT;
								
			break;
			case CON_SYNC_STATE_SYNC_RECVD:
				cl_wait(10);
				Cl_SendDatatoconsole( CON_TX_COMMAND_SYNC,&CmdData ,0);
				Cl_SyncState = CON_SYNC_STATE_WAIT_FOR_SYNC_ACK;
			Cl_MacRetval = CL_WAIT;
				break;
				
			case CON_SYNC_STATE_WAIT_FOR_SYNC_ACK:
			Cl_MacRetval = Cl_Console_SyncGetMessage();
				if(Cl_MacRetval== CL_OK)
				{
					Cl_ConsoleRxMsg.command = Cl_Console_msgbuffer.Console_databuffer[0];
					Cl_ConsoleRxMsg.datasize = Cl_Console_msgbuffer.Console_databuffer[1];
					Cl_ConsoleRxMsg.data.byte[0] = Cl_Console_msgbuffer.Console_databuffer[2];
					if(Cl_ConsoleRxMsg.command == CON_RX_COMMAND_ACK )
					{
						if(Cl_ConsoleRxMsg.data.byte[0] == CON_TX_COMMAND_SYNC )
						{
							Cl_SyncState = CON_SYNC_STATE_SYNC_ACK_RECVD;

							if( Cl_MacEventQueue3.Cl_MacNewEventcount <= MAC_EVENT_COUNT_MAX )	
							 {
								// Cl_MacEventQueue3.Cl_MacEvent[Cl_MacEventQueue3.Cl_MacNewEventcount] = EVT_COMMAND_ENTER_STANDBY ;
								Cl_MacEventQueue3.Cl_MacEvent[Cl_MacEventQueue3.Cl_MacNewEventcount] = EVT_COMMAND_ENTER_INIT ;
								
								 Cl_MacEventQueue3.Cl_MacNewEvent = true;
								 Cl_MacEventQueue3.Cl_MacNewEventcount++;
							 }
							 
							 syncdone = true; //testing
							Cl_ConsoleState = CON_STATE_ACTIVE;
							Cl_MacRetval = CL_OK;
							
							test_count = 0; //test
														
						}
					}
					else
					{
							Cl_MacRetval = CL_WAIT;
					}
				}
			break;
			case CON_SYNC_STATE_SYNC_ACK_RECVD:
			
				Cl_ConsoleState = CON_STATE_ACTIVE;
				Cl_MacRetval = CL_OK;
			break;
			
			default:
			break;
		}
	}
	/******************************************************************/

	return Cl_MacRetval;
 }
 /*
 * \brief  Initialise console controller
 * \param   
 * \ 
 * \return CL_OK on success. 
 * \Globals used : 
 */
 
 Cl_ReturnCodeType Cl_Console_init(void)
 {
	 Cl_ConsoleState = CON_STATE_INIT;
	 
	 return CL_OK;
 }
 
  /*
 * \brief  Get Message from console ( called by sync state handler)
 * \param   
 * \ 
 * \return CL_OK on success. 
 * \Globals used : 
 */
  Cl_ReturnCodeType Cl_Console_SyncGetMessage()
  {
	 Cl_ReturnCodeType  Cl_ConsoleRetval1 = CL_WAIT;
	 
 				Cl_ConsoleRetval1 = Cl_Console_Read_ConsoleMessage_Status(); // Reads the status of the data buffer populated by the timer console read interrupt.
 				if(Cl_ConsoleRetval1 == CL_OK)
 				{
	 				Cl_ConsoleRetval1 = Cl_Console_Read_Message_Console();
	 				if(Cl_ConsoleRetval1 == CL_OK)
	 				{
		 				Cl_ConsoleRetval1 = Cl_Console_Read_Message_Ready_Status(); // Reads the status of the data buffer populated by the timer console read interrupt.
					}
				 }
		return Cl_ConsoleRetval1;
				 
  }
  
/****************************************************************************/
  /*
 * \brief  Handler for periodic sync from console
 * \param   
 * \ 
 * \return CL_OK on success. 
 * \Globals used : 
 */
Cl_ReturnCodeType Cl_Console_SyncManager(Cl_Mac_EventsType Cl_consoleMacEvent )
{
			if(Cl_consoleMacEvent == EVT_CONSOLE_COMMAND_ACK)
			{
				//send sync
				ConsoleSync = true;
			}
			
			if(Cl_consoleMacEvent == EVT_CONSOLE_COMMAND_SYNC)
			{
				//send sync
				Cl_Uint8Type data = CON_RX_COMMAND_SYNC;
				ConsoleSync = true;
				Cl_SendDatatoconsole(CON_TX_COMMAND_ACK,&data,1);
				
			}
			
			return CL_OK;
}