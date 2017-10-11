/*
 * cl_dprep_primecontroller.c
 *
 * Created: 11/3/2014 1:03:14 PM
 *  Author: user
 */ 
#include "cl_dprep_primecontroller_include.h"
#include "cl_dprep_primecontroller_proto.h"
#include "sv_interface_fun.h"

Cl_Dprep_PrimeStates cl_dprep_prime_state = CL_DPREP_PRIME_STATE_IDLE;
Cl_Dprep_PrimeEvents cl_dprep_prime_event = CL_DPREP_PRIME_PRIME_NULL;
static int8_t cl_prime_sec = 0;
static int8_t cl_prime_min = 0;
static int8_t cl_dialyser_prime_sec = 0;
static int8_t cl_dialyser_prime_min = 0;

extern Cl_Dprep_PrimeStates cl_dprep_prime_stateDummy;

Cl_ReturnCodeType cl_dprep_primeInit(void)
{
	 cl_dprep_prime_state = CL_DPREP_PRIME_STATE_IDLE;
	 cl_dprep_prime_event = CL_DPREP_PRIME_PRIME_NULL;
	  cl_prime_sec = 0;
	  cl_prime_min = 0;
	  cl_dialyser_prime_sec = 0;
	  cl_dialyser_prime_min = 0;
	
}


Cl_ReturnCodeType cl_dprep_primecontroller(Cl_Dprep_PrimeEvents prime_event , int16_t data)
{
	Cl_ReturnCodeType 	 Cl_dprep_primeretcode = CL_ERROR;
	
	if(prime_event == CL_DPREP_PRIME_PRIME_TICK_SEC)
	{
		cl_dprep_prime_stateDummy = cl_dprep_prime_state;
	}
		switch(cl_dprep_prime_state)
		{
			case CL_DPREP_PRIME_STATE_IDLE:
			//case CL_DPREP_PRIME_STATE_DIALYSER_PRIMING_STOPPED:
				switch(prime_event)
				{
					case CL_DPREP_PRIME_BLOODPUMP_SETRATE:
					cl_bp_controller(CL_BP_EVENT_SET_BP_RATE,data);
					break;
					case CL_DPREP_PRIME_PRIME_START:
					DD_CAN_WRITE_HEMO(HEMO_START_BD_CALIBRATION, 0);
					Cl_SendDatatoconsole(CON_TX_COMMAND_PRINTTEXT,"PRIME2",6);
					sv_cntrl_disable_bypass();
					cl_bp_controller(CL_BP_EVENT_SET_BP_RATE,100);          // manav01 added
					cl_bp_controller(CL_BP_EVENT_START,0);
						Cl_AlarmActivateAlarms(BLOODDOOR_STATUS_OPEN,true); // manav03 added
						Cl_dprep_primeretcode = Cl_SendDatatoconsole(CON_TX_COMMAND_START_PRIME_CNFRM,&data,0);
						Cl_dprep_primeretcode = Cl_SendDatatoconsole(CON_TX_COMMAND_PRIME_STARTED,&data,0);
						Cl_dprep_primeretcode = Cl_Dprep_primeUpdatePrimeTimeInfo();
								
						cl_dprep_prime_state = CL_DPREP_PRIME_STATE_PRIMING;
					break;
					default:break;
				}
			break;
			case CL_DPREP_PRIME_STATE_PRIMING:
				switch(prime_event)
				{
					case CL_DPREP_PRIME_PRIME_ALARM:
					cl_dprep_prime_state = CL_DPREP_PRIME_STATE_PRIME_STOPPED;
					break;
					case CL_DPREP_PRIME_PRIME_TICK_SEC:
					Cl_dprep_primeretcode = Cl_Dprep_SendPrepStateData(PRIMING_DATA);
					if(cl_prime_sec++ >= 60)
					{
						cl_prime_sec = 0;
						cl_prime_min++;
						Cl_dprep_primeretcode = Cl_Dprep_primeUpdatePrimeTimeInfo();
						
					}
					if(cl_prime_min >= CL_DPREP_PRIME_TIMEOUT_MIN )
					{
						cl_bp_controller(CL_BP_EVENT_STOP,0);
						
						Cl_dprep_primeretcode = Cl_SendDatatoconsole(CON_TX_COMMAND_PRIME_COMPLETED,&data,0);
						cl_dprep_prime_state = CL_DPREP_PRIME_STATE_PRIME_COMPLETED;
					//	cl_dprep_prime_state = CL_DPREP_PRIME_STATE_IDLE;
					}

					break;
					case CL_DPREP_PRIME_PRIME_SET_RATE:
					break;
					case CL_DPREP_PRIME_PRIME_STOP:
					cl_bp_controller(CL_BP_EVENT_STOP,0);
					Cl_dprep_primeretcode = Cl_SendDatatoconsole(CON_TX_COMMAND_PRIME_STOPPED,&data,0);
					cl_dprep_prime_state = CL_DPREP_PRIME_STATE_PRIME_STOPPED;
					break;
					case CL_DPREP_PRIME_PRIME_PAUSE:
					cl_bp_controller(CL_BP_EVENT_STOP,0);
					cl_dprep_prime_state = CL_DPREP_PRIME_STATE_PRIME_PAUSED;
					break;
					case 	CL_DPREP_PRIME_BLOODPUMP_START:
					 cl_bp_controller(CL_BP_EVENT_START,0);
					break;
					case CL_DPREP_PRIME_BLOODPUMP_STOP:
					 cl_bp_controller(CL_BP_EVENT_STOP,0);
					break;
					case CL_DPREP_PRIME_BLOODPUMP_SETRATE:
					cl_bp_controller(CL_BP_EVENT_SET_BP_RATE,data);
					break;
					default:break;
				}

			
			break;
			case CL_DPREP_PRIME_STATE_PRIME_STOPPED:
						switch(prime_event)
						{
							case CL_DPREP_PRIME_PRIME_START:
							DD_CAN_WRITE_HEMO(HEMO_START_BD_CALIBRATION, 0);
								cl_bp_controller(CL_BP_EVENT_START,0);
								cl_prime_min = 0;
								cl_prime_sec = 0;
								Cl_dprep_primeretcode = Cl_SendDatatoconsole(CON_TX_COMMAND_START_PRIME_CNFRM,&data,0);
								Cl_dprep_primeretcode = Cl_SendDatatoconsole(CON_TX_COMMAND_PRIME_STARTED,&data,0);
								Cl_dprep_primeretcode = Cl_Dprep_primeUpdatePrimeTimeInfo();
								
								cl_dprep_prime_state = CL_DPREP_PRIME_STATE_PRIMING;
							break;
							case 	CL_DPREP_PRIME_BLOODPUMP_START:
							cl_bp_controller(CL_BP_EVENT_START,0);
							break;
							case CL_DPREP_PRIME_BLOODPUMP_STOP:
							cl_bp_controller(CL_BP_EVENT_STOP,0);
							break;
							case CL_DPREP_PRIME_BLOODPUMP_SETRATE:
							cl_bp_controller(CL_BP_EVENT_SET_BP_RATE,data);
							break;
						}
			break;
			case CL_DPREP_PRIME_STATE_PRIME_PAUSED:
						switch(prime_event)
						{
							case CL_DPREP_PRIME_PRIME_START:
								cl_bp_controller(CL_BP_EVENT_START,0);
								Cl_dprep_primeretcode = Cl_SendDatatoconsole(CON_TX_COMMAND_START_PRIME_CNFRM,&data,0);
								Cl_dprep_primeretcode = Cl_SendDatatoconsole(CON_TX_COMMAND_PRIME_STARTED,&data,0);
								Cl_dprep_primeretcode = Cl_Dprep_primeUpdatePrimeTimeInfo();
								
								cl_dprep_prime_state = CL_DPREP_PRIME_STATE_PRIMING;
							break;
							case CL_DPREP_PRIME_PRIME_STOP:
							cl_bp_controller(CL_BP_EVENT_STOP,0);
							Cl_dprep_primeretcode = Cl_SendDatatoconsole(CON_TX_COMMAND_PRIME_STOPPED,&data,0);
							cl_dprep_prime_state = CL_DPREP_PRIME_STATE_PRIME_STOPPED;
							break;
							case 	CL_DPREP_PRIME_BLOODPUMP_START:
							cl_bp_controller(CL_BP_EVENT_START,0);
							break;
							case CL_DPREP_PRIME_BLOODPUMP_STOP:
							cl_bp_controller(CL_BP_EVENT_STOP,0);
							break;
							case CL_DPREP_PRIME_BLOODPUMP_SETRATE:
							cl_bp_controller(CL_BP_EVENT_SET_BP_RATE,data);
							break;
						}
			break;
			case CL_DPREP_PRIME_STATE_PRIME_COMPLETED:
						switch(prime_event)
						{
							case CL_DPREP_PRIME_RCIRC_START:
							Cl_SendDatatoconsole(CON_TX_COMMAND_PRINTTEXT,"RCIRC2",6);
							cl_prime_min =  0;
							cl_prime_sec = 0;
					cl_bp_controller(CL_BP_EVENT_SET_BP_RATE,100); 
								cl_bp_controller(CL_BP_EVENT_START,0);
								//Cl_dprep_primeretcode = Cl_SendDatatoconsole(CON_TX_COMMAND_START_PRIME_CNFRM,&data,0);
								Cl_dprep_primeretcode = Cl_SendDatatoconsole(CONT_TX_COMMAND_SYS_RCIRC_STARTED,&data,0);
								Cl_dprep_primeretcode = Cl_Dprep_primeUpdatePrimeTimeInfo();
								
								cl_dprep_prime_state = CL_DPREP_PRIME_STATE_PRIME_RCIRC_STARTED;
							break;
							case CL_DPREP_PRIME_RCIRC_STOP:
							break;
							case CL_DPREP_PRIME_PRIME_START:
							Cl_SendDatatoconsole(CON_TX_COMMAND_PRINTTEXT,"PRIME2",6);
							cl_prime_min =  0;
							cl_prime_sec = 0;
					
								cl_bp_controller(CL_BP_EVENT_START,0);
								Cl_dprep_primeretcode = Cl_SendDatatoconsole(CON_TX_COMMAND_START_PRIME_CNFRM,&data,0);
								Cl_dprep_primeretcode = Cl_SendDatatoconsole(CON_TX_COMMAND_PRIME_STARTED,&data,0);
								Cl_dprep_primeretcode = Cl_Dprep_primeUpdatePrimeTimeInfo();
								
								cl_dprep_prime_state = CL_DPREP_PRIME_STATE_PRIMING;
							break;

							case CL_DPREP_PRIME_PRIME_START_DIALYSER_PRIMING:
							cl_bp_controller(CL_BP_EVENT_START,0);
						//	Cl_dprep_primeretcode = Cl_SendDatatoconsole(CON_TX_COMMAND_START_DIALISER_PRIME_CNFRM,&data,0);
						//	Cl_dprep_primeretcode = Cl_SendDatatoconsole(CON_TX_COMMAND_DIALYSER_PRIME_STARTED,&data,0);
							Cl_dprep_primeretcode = Cl_Dprep_UpdateDialyserPrimeTimeInfo();
							cl_dprep_prime_state = CL_DPREP_PRIME_STATE_DIALYSER_PRIMING;
							break;
							case 	CL_DPREP_PRIME_BLOODPUMP_START:
							cl_bp_controller(CL_BP_EVENT_START,0);
							break;
							case CL_DPREP_PRIME_BLOODPUMP_STOP:
							cl_bp_controller(CL_BP_EVENT_STOP,0);
							break;
							case CL_DPREP_PRIME_BLOODPUMP_SETRATE:
							cl_bp_controller(CL_BP_EVENT_SET_BP_RATE,data);
							break;
							
						}
						
			break;
			case CL_DPREP_PRIME_STATE_PRIME_RCIRC_STARTED:
		
					switch(prime_event)
					{
						case CL_DPREP_PRIME_PRIME_ALARM:
						cl_dprep_prime_state = CL_DPREP_PRIME_STATE_PRIME_RCIRC_STOPPED;
						break;
						case CL_DPREP_PRIME_PRIME_TICK_SEC:
						if(cl_prime_sec++ >= 60)
						{
							cl_prime_sec = 0;
							cl_prime_min++;
							Cl_dprep_primeretcode = Cl_Dprep_primeUpdatePrimeTimeInfo();
						}
						if(cl_prime_min >= CL_DPREP_PRIME_TIMEOUT_MIN )
						{
							cl_bp_controller(CL_BP_EVENT_STOP,0);
						
							Cl_dprep_primeretcode = Cl_SendDatatoconsole(CONT_TX_COMMAND_SYS_RCIRC_COMPLETED,&data,0);
							cl_dprep_prime_state = CL_DPREP_PRIME_STATE_PRIME_RCIRC_COMPLETED;
						
						}

						break;
						case CL_DPREP_PRIME_PRIME_SET_RATE:
						break;
						case CL_DPREP_PRIME_PRIME_STOP:
						cl_bp_controller(CL_BP_EVENT_STOP,0);
						Cl_dprep_primeretcode = Cl_SendDatatoconsole(CON_TX_COMMAND_PRIME_STOPPED,&data,0);
						cl_dprep_prime_state = CL_DPREP_PRIME_STATE_PRIME_STOPPED;
						break;
						case CL_DPREP_PRIME_PRIME_PAUSE:
						cl_bp_controller(CL_BP_EVENT_STOP,0);
						cl_dprep_prime_state = CL_DPREP_PRIME_STATE_PRIME_PAUSED;
						break;
						case 	CL_DPREP_PRIME_BLOODPUMP_START:
						 cl_bp_controller(CL_BP_EVENT_START,0);
						break;
						case CL_DPREP_PRIME_BLOODPUMP_STOP:
						 cl_bp_controller(CL_BP_EVENT_STOP,0);
						break;
						case CL_DPREP_PRIME_BLOODPUMP_SETRATE:
						cl_bp_controller(CL_BP_EVENT_SET_BP_RATE,data);
						break;
						default:break;
					}

			
			break;
			case CL_DPREP_PRIME_STATE_PRIME_RCIRC_STOPPED:
							switch(prime_event)
									{
							case CL_DPREP_PRIME_RCIRC_START:
									Cl_SendDatatoconsole(CON_TX_COMMAND_PRINTTEXT,"RCIRC2",6);
									cl_prime_min =  0;
									cl_prime_sec = 0;
					
									cl_bp_controller(CL_BP_EVENT_START,0);
									//Cl_dprep_primeretcode = Cl_SendDatatoconsole(CON_TX_COMMAND_START_PRIME_CNFRM,&data,0);
									Cl_dprep_primeretcode = Cl_SendDatatoconsole(CONT_TX_COMMAND_SYS_RCIRC_STARTED,&data,0);
									Cl_dprep_primeretcode = Cl_Dprep_primeUpdatePrimeTimeInfo();
								
									cl_dprep_prime_state = CL_DPREP_PRIME_STATE_PRIME_RCIRC_STARTED;
								break;
								default:break;
									}
			break;
			case CL_DPREP_PRIME_STATE_PRIME_RCIRC_COMPLETED:
						switch(prime_event)
						{
							case CL_DPREP_PRIME_RCIRC_START:
							Cl_SendDatatoconsole(CON_TX_COMMAND_PRINTTEXT,"RCIRC2",6);
							cl_prime_min =  0;
							cl_prime_sec = 0;
					
								cl_bp_controller(CL_BP_EVENT_START,0);
								//Cl_dprep_primeretcode = Cl_SendDatatoconsole(CON_TX_COMMAND_START_PRIME_CNFRM,&data,0);
								Cl_dprep_primeretcode = Cl_SendDatatoconsole(CONT_TX_COMMAND_SYS_RCIRC_STARTED,&data,0);
								Cl_dprep_primeretcode = Cl_Dprep_primeUpdatePrimeTimeInfo();
								
								cl_dprep_prime_state = CL_DPREP_PRIME_STATE_PRIME_RCIRC_STARTED;
							break;
							case CL_DPREP_PRIME_RCIRC_STOP:
							break;
							case CL_DPREP_PRIME_PRIME_START:
							Cl_SendDatatoconsole(CON_TX_COMMAND_PRINTTEXT,"PRIME2",6);
							cl_prime_min =  0;
							cl_prime_sec = 0;
					
								cl_bp_controller(CL_BP_EVENT_START,0);
								Cl_dprep_primeretcode = Cl_SendDatatoconsole(CON_TX_COMMAND_START_PRIME_CNFRM,&data,0);
								Cl_dprep_primeretcode = Cl_SendDatatoconsole(CON_TX_COMMAND_PRIME_STARTED,&data,0);
								Cl_dprep_primeretcode = Cl_Dprep_primeUpdatePrimeTimeInfo();
								
								cl_dprep_prime_state = CL_DPREP_PRIME_STATE_PRIMING;
							break;

							case CL_DPREP_PRIME_PRIME_START_DIALYSER_PRIMING:
							cl_bp_controller(CL_BP_EVENT_START,0);
						//	Cl_dprep_primeretcode = Cl_SendDatatoconsole(CON_TX_COMMAND_START_DIALISER_PRIME_CNFRM,&data,0);
							Cl_dprep_primeretcode = Cl_SendDatatoconsole(CON_TX_COMMAND_DIALYSER_PRIME_STARTED,&data,0);
							Cl_dprep_primeretcode = Cl_Dprep_UpdateDialyserPrimeTimeInfo();
							cl_dprep_prime_state = CL_DPREP_PRIME_STATE_DIALYSER_PRIMING;
							break;
							case 	CL_DPREP_PRIME_BLOODPUMP_START:
							cl_bp_controller(CL_BP_EVENT_START,0);
							break;
							case CL_DPREP_PRIME_BLOODPUMP_STOP:
							cl_bp_controller(CL_BP_EVENT_STOP,0);
							break;
							case CL_DPREP_PRIME_BLOODPUMP_SETRATE:
							cl_bp_controller(CL_BP_EVENT_SET_BP_RATE,data);
							break;
							
						}
			break;
			
			case CL_DPREP_PRIME_STATE_DIALYSER_PRIMING:
						switch(prime_event)
						{
							case CL_DPREP_PRIME_PRIME_ALARM:
							cl_dprep_prime_state = CL_DPREP_PRIME_STATE_DIALYSER_PRIMING_STOPPED;
							break;
							case CL_DPREP_PRIME_PRIME_TICK_SEC:
							if(cl_dialyser_prime_sec++ >= 60)
							{
								cl_dialyser_prime_sec = 0;
								cl_dialyser_prime_min++;
							}
// 							if(cl_dialyser_prime_min >= CL_DPREP_DIALISER_PRIME_TIMEOUT_MIN )
// 							{
// 								cl_bp_controller(CL_BP_EVENT_STOP,0);
// 								Cl_dprep_primeretcode = Cl_SendDatatoconsole(CON_TX_COMMAND_DIALISYS_PRIME_COMPLETED,&data,0);
// 								cl_dprep_prime_state = CL_DPREP_PRIME_STATE_DIALYSER_PRIMING_COMPLETED;
// 						
// 							}
							break;
							case CL_DPREP_PRIME_PRIME_SET_RATE:
							break;
							case CL_DPREP_PRIME_PRIME_STOP_DIALYSER_PRIMING:
							case CL_DPREP_PRIME_PRIME_STOP:
							
								// //lets keep bloodpump running
								if(data == 1)
								{
									//Cl_dprep_primeretcode = Cl_SendDatatoconsole(CON_TX_COMMAND_DIALISYS_PRIME_COMPLETED,&data,0);
									cl_bp_controller(CL_BP_EVENT_STOP,0);
									cl_dprep_prime_state = CL_DPREP_PRIME_STATE_DIALYSER_PRIMING_COMPLETED;
								}else
								{
									sv_cntrl_enable_bypass();
									cl_bp_controller(CL_BP_EVENT_STOP,0);
									cl_dialyser_prime_sec = 0;
									cl_dialyser_prime_min = 0;
									cl_dprep_prime_state = CL_DPREP_PRIME_STATE_DIALYSER_PRIMING_STOPPED;
									Cl_SendDatatoconsole(CON_TX_COMMAND_DIALYSER_PRIME_STOPPED,NULL,0);
								}

							//	Cl_SendDatatoconsole(CON_TX_COMMAND_DIALYSER_PRIME_STOPPED,NULL,0);
								
							break;
							case CL_DPREP_PRIME_PRIME_PAUSE_DIALYSER_PRIMING:
							
							cl_dprep_prime_state = CL_DPREP_PRIME_STATE_DIALYSER_PRIMING_PAUSED;
							break;
							case 	CL_DPREP_PRIME_BLOODPUMP_START:
							cl_bp_controller(CL_BP_EVENT_START,0);
							break;
							case CL_DPREP_PRIME_BLOODPUMP_STOP:
							cl_bp_controller(CL_BP_EVENT_STOP,0);
							break;
							case CL_DPREP_PRIME_BLOODPUMP_SETRATE:
							cl_bp_controller(CL_BP_EVENT_SET_BP_RATE,data);
							break;
							default:break;
						}
			break;
			case CL_DPREP_PRIME_STATE_DIALYSER_PRIMING_STOPPED:
						switch(prime_event)
						{
							case CL_DPREP_PRIME_PRIME_START_DIALYSER_PRIMING:
							sv_cntrl_disable_bypass();
							cl_bp_controller(CL_BP_EVENT_START,0);
					//		Cl_dprep_primeretcode = Cl_SendDatatoconsole(CON_TX_COMMAND_START_DIALISER_PRIME_CNFRM,&data,0);
							Cl_dprep_primeretcode = Cl_SendDatatoconsole(CON_TX_COMMAND_DIALYSER_PRIME_STARTED,&data,0);
							Cl_dprep_primeretcode = Cl_Dprep_UpdateDialyserPrimeTimeInfo();
							cl_dprep_prime_state = CL_DPREP_PRIME_STATE_DIALYSER_PRIMING;
							break;
							case 	CL_DPREP_PRIME_BLOODPUMP_START:
							cl_bp_controller(CL_BP_EVENT_START,0);
							break;
							case CL_DPREP_PRIME_BLOODPUMP_STOP:
							cl_bp_controller(CL_BP_EVENT_STOP,0);
							break;
							case CL_DPREP_PRIME_BLOODPUMP_SETRATE:
							cl_bp_controller(CL_BP_EVENT_SET_BP_RATE,data);
							break;
						}
			break;
			case CL_DPREP_PRIME_STATE_DIALYSER_PRIMING_PAUSED:
						switch(prime_event)
						{
							case CL_DPREP_PRIME_PRIME_START_DIALYSER_PRIMING:
							cl_bp_controller(CL_BP_EVENT_START,0);
						//	Cl_dprep_primeretcode = Cl_SendDatatoconsole(CON_TX_COMMAND_START_DIALISER_PRIME_CNFRM,&data,0);
							Cl_dprep_primeretcode = Cl_SendDatatoconsole(CON_TX_COMMAND_DIALYSER_PRIME_STARTED,&data,0);
							Cl_dprep_primeretcode = Cl_Dprep_UpdateDialyserPrimeTimeInfo();
							cl_dprep_prime_state = CL_DPREP_PRIME_STATE_DIALYSER_PRIMING;
							break;
							case CL_DPREP_PRIME_PRIME_STOP_DIALYSER_PRIMING:
							cl_dprep_prime_state = CL_DPREP_PRIME_STATE_DIALYSER_PRIMING_STOPPED;
							break;
							case 	CL_DPREP_PRIME_BLOODPUMP_START:
							cl_bp_controller(CL_BP_EVENT_START,0);
							break;
							case CL_DPREP_PRIME_BLOODPUMP_STOP:
							cl_bp_controller(CL_BP_EVENT_STOP,0);
							break;
							case CL_DPREP_PRIME_BLOODPUMP_SETRATE:
							cl_bp_controller(CL_BP_EVENT_SET_BP_RATE,data);
							break;

						}
			break;
			case CL_DPREP_PRIME_STATE_DIALYSER_PRIMING_COMPLETED:
			break;


		}
	

	
	return 0;
}


Cl_ReturnCodeType Cl_Dprep_primeUpdatePrimeTimeInfo(void)
{
	
	Cl_ReturnCodeType  Cl_dprep_primeretcode = CL_ERROR;
	Cl_ConsoleTxCommandtype command = CON_TX_COMMAND_COMMAND_MAX;
	Cl_Uint8Type data[7] = {0,0,0,0,0,0,0};
	
	command = CON_TX_COMMAND_REM_TIME;

	data[0] = (Cl_Uint8Type) PRIMING_DATA;

	data[1]= (Cl_Uint8Type)cl_prime_min;
	data[2]= (Cl_Uint8Type)0;
	data[3]= (Cl_Uint8Type)cl_prime_sec;
	data[4]= (Cl_Uint8Type) (CL_DPREP_PRIME_TIMEOUT_MIN - cl_prime_min );
	data[5]= (Cl_Uint8Type) 0;
	data[6]= (Cl_Uint8Type) 0;
	
	Cl_dprep_primeretcode = Cl_SendDatatoconsole(command,&data,7);
	
	return CL_OK;
}

Cl_ReturnCodeType Cl_Dprep_UpdateDialyserPrimeTimeInfo(void)
{
	
	Cl_ReturnCodeType  Cl_dprep_primeretcode = CL_ERROR;
	Cl_ConsoleTxCommandtype command = CON_TX_COMMAND_COMMAND_MAX;
	Cl_Uint8Type data[7] = {0,0,0,0,0,0,0};
	
	command = CON_TX_COMMAND_REM_TIME;

	data[0] = (Cl_Uint8Type) DIALYSER_PRIME_DATA;

	data[1]= (Cl_Uint8Type)cl_dialyser_prime_min;
	data[2]= (Cl_Uint8Type)0;
	data[3]= (Cl_Uint8Type)cl_dialyser_prime_sec;
	data[4]= (Cl_Uint8Type) (CL_DPREP_DIALISER_PRIME_TIMEOUT_MIN - cl_prime_min );
	data[5]= (Cl_Uint8Type) 0;
	data[6]= (Cl_Uint8Type) 0;
	
	Cl_dprep_primeretcode = Cl_SendDatatoconsole(command,&data,7);
	
	return CL_OK;

}
		
Cl_ReturnCodeType cl_dprep_activate_prime_related_alarms(void)
{
			Cl_ReturnCodeType  Cl_dprep_primeretcode = CL_ERROR;
			//		Cl_dprepretcode =  Cl_AlarmActivateAlarms(APTSTATUS_HIGH,true );
			//		Cl_dprepretcode =  Cl_AlarmActivateAlarms(VPTSTATUS_HIGH,true );
	//		Cl_dprep_primeretcode =  Cl_AlarmConfigureAlarmType(BLOODDOOR_STATUS_OPEN,LOGIC_LOW,0,0,0);
	//		Cl_dprep_primeretcode =  Cl_AlarmActivateAlarms(BLOODDOOR_STATUS_OPEN,true );
			
	return CL_OK;
}
