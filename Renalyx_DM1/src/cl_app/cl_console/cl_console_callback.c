/*
 * cl_console_callback.c
 *
 * Created: 12/30/2013 3:21:02 PM
 *  Author: sanjeer_k
 */ 
#include "cl_types.h"
#include "cl_consolecontroller.h"


extern cl_console_bufferType Cl_Console_buffer,Cl_Console_buffer1,Cl_Console_buffer2;
extern  Cl_Uint8Type testCl_BoolType ;
extern int8_t test_count;

Cl_ReturnCodeType  cl_console_receivedbyte(Cl_Uint8Type console_databyte);

  /*
 * \brief  cl_console_receivedbyte  - Call back from UART handler in the interrupt context
 * \param  
 * \ 
 * \return CL_OK on success. 
 * \Globals used : Cl_Console_buffer1,Cl_Console_buffer2
 */
Cl_ReturnCodeType  cl_console_receivedbyte(Cl_Uint8Type console_databyte)
{
	static Cl_Uint8Type prev_databyte =0 ;
	static unsigned int temp = 0;

	if(Cl_Console_buffer1.BufferEmpty)
	{
		if(Cl_Console_buffer2.BufferEmpty)
			{
				Cl_Console_buffer1.CurreltyActive = true;
				Cl_Console_buffer2.CurreltyActive = false;					
			}
		
	}
	if(Cl_Console_buffer1.CurreltyActive == true )
	{				
				if(Cl_Console_buffer1.StartSequenceDetected == false)
				{
					if((prev_databyte == CL_CONSOLEMSG_STARTBYTE1) && (console_databyte == CL_CONSOLEMSG_STARTBYTE2))
						{
							Cl_Console_buffer1.StartSequenceDetected = true;
						}
				}
				else
				{
					Cl_Console_buffer1.Console_databuffer[Cl_Console_buffer1.Console_databuffer_write_idx] = console_databyte;
					Cl_Console_buffer1.ConsoleNewData = true;
					Cl_Console_buffer1.BufferEmpty = false;
					if((prev_databyte == CL_CONSOLEMSG_STOPBYTE1) && (console_databyte == CL_CONSOLEMSG_STOPBYTE2))
						{
							Cl_Console_buffer1.ConsoleDatabufferReady = true;
							Cl_Console_buffer1.ConsoleDatabufferprocessed = false;
							Cl_Console_buffer1.Data_incomplete = false;
							Cl_Console_buffer1.CurreltyActive = false;
							Cl_Console_buffer2.CurreltyActive =true;
						}
					
					Cl_Console_buffer1.Console_databuffer_write_idx++;
				}
				
				if(Cl_Console_buffer1.Console_databuffer_write_idx >= CONSOLE_BUFFER_SIZE)
					{
						Cl_Console_buffer1.ConsoleDatabufferReady = true ;
						Cl_Console_buffer1.ConsoleDatabufferprocessed = false;
						Cl_Console_buffer1.CurreltyActive = false;
						Cl_Console_buffer1.Data_incomplete = true;
						
					}

			prev_databyte = console_databyte; // we may have to compare start and stop sequences
			
				return CL_ACCEPTED; // if we have just filled 
	}
	
	if(Cl_Console_buffer2.CurreltyActive == true )
	{
		if(Cl_Console_buffer1.Data_incomplete == true)
		{
			Cl_Console_buffer2.StartSequenceDetected = true;
		}
				
		// Write to the buffer only when buffer is currently active and not full or packet ready
		
		if(Cl_Console_buffer2.StartSequenceDetected == false)
		{
			
			if((prev_databyte == CL_CONSOLEMSG_STARTBYTE1) && (console_databyte == CL_CONSOLEMSG_STARTBYTE2))
			{
				Cl_Console_buffer2.StartSequenceDetected = true;
				Cl_Console_buffer2.Console_databuffer_write_idx = 0;
			}
		}
		else
		{
			Cl_Console_buffer2.Console_databuffer[Cl_Console_buffer2.Console_databuffer_write_idx] = console_databyte;
			Cl_Console_buffer2.ConsoleNewData = true;
			Cl_Console_buffer2.BufferEmpty = false;
		if((prev_databyte == CL_CONSOLEMSG_STOPBYTE1) && (console_databyte == CL_CONSOLEMSG_STOPBYTE2))
			{
				Cl_Console_buffer2.ConsoleDatabufferReady = true;
				Cl_Console_buffer2.ConsoleDatabufferprocessed = false;
				Cl_Console_buffer1.CurreltyActive = true;
				Cl_Console_buffer2.CurreltyActive = false;
			}
			Cl_Console_buffer2.Console_databuffer_write_idx++;
				
		}
					
		if(Cl_Console_buffer2.Console_databuffer_write_idx > CONSOLE_BUFFER_SIZE)
		{
			Cl_Console_buffer2.ConsoleDatabufferReady = true ;
			Cl_Console_buffer2.CurreltyActive = false;
			Cl_Console_buffer2.Data_incomplete = true;
			Cl_Console_buffer1.Data_incomplete = false;
		}

	if (Cl_Console_buffer2.Data_incomplete && Cl_Console_buffer1.Data_incomplete )
	{
		//printf("buffer overflow\n");
	}

			prev_databyte = console_databyte; // we may have to compare start and stop sequences
	}
	
	return CL_ACCEPTED;
}
