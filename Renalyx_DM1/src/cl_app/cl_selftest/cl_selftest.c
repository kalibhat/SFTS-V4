/*0
 * cl_selftest.c
 *
 * Created: 8/7/2017 5:27:29 PM
 *  Author: VarunK
 */ 
#include "cl_selftest.h"
#include "cl_types.h"

//Cl_ReturnCodeType self_test(Cl_Mac_EventsType);
extern Cl_ReturnCodeType cl_bp_selftest_start(void);
extern Cl_ReturnCodeType cl_bp_selftest_stop(void);
//extern volatile float pressure_final_apt,pressure_final_vpt,pressure_final_ps1,pressure_final_ps2,pressure_final_ps3;
 float res_temp_value;
uint16_t uf_periodvalue =0;
uint16_t hep_periodvalue =0;
uint16_t bp_periodvalue =0;
Cl_ReturnCodeType cl_selftest_translatemacevent(Cl_Mac_EventsType,cl_selftest_event* );
Cl_selftest_StatesType cl_selfteststate = CL_SELFTEST_STATE_IDLE;
Cl_Uint8Type sv_cntrl_activatepump(sv_pumptype );
 //int16_t apt=0,vpt=0,ps1=0,ps2=0,res_temp_value;
//int16_t res_temp_value=0;
int16_t sensordata=0;
int16_t Pressuredata=0;
int16_t sensordatamillivolts =0;
 uint16_t time_count = 0;
cl_DlsisDatatype TEMP_DATA;
int16_t Pressuredatainmillibars =0;
//static Ps1,Ps2,Ps3,Atp,Vtp,TMP = 0;
bool startpropFlag = false;
int durcount = 0;
 Cl_ReturnCodeType cl_wait(Cl_Uint32Type );  
int16_t test_time = 0 ,
selftest_second_counter = 0;

//   float Tmp1Value=0,Tmp2Value = 0,Tmp3Value=0 ;
   Cl_ReturnCodeType Cl_selftest_init(void);
   Cl_selftest_Component_Type Cl_selftest_Component;


  Cl_selftest_status_Type Cl_selftest_comp;

   
   Cl_Uint8Type status[CL_SELFTEST_RESULT_MAXSTATE];

  
//	cl_selftest_transelateevent(Cl_Mac_EventsType, Cl_selftest_event*)
Cl_ReturnCodeType self_test(Cl_Mac_EventsType Cl_MacselftestEvent)
	{
	//	Cl_selftest_StatesType cl_selfteststate = CL_SELFTEST_STATE_IDLE;
		Cl_ReturnCodeType  Cl_selftestretcode = CL_OK;
		cl_ufpump_states uf_pumpstate ;       //= CL_UFP_STATE_IDLE;
	Cl_ConsoleTxCommandtype command = CON_TX_COMMAND_COMMAND_MAX;
	Cl_Uint8Type dataarray[4] = {0,0,0,0};
	Cl_Uint8Type data = 0;
		 cl_selftest_event cl_selftest_event_type;
		Cl_Uint32Type uf_test_value=0, hp_test_value = 0, bp_test_value =0 ,bicarb_test_value = 0;
		Cl_selftestretcode= cl_selftest_translatemacevent(Cl_MacselftestEvent, &cl_selftest_event_type);
		
		  int16_t bf_b=0;
		  int16_t af_b=0;
		 
		int16_t apt_val= 0,vpt_val=0;
		int16_t Ts1=0,Ts2=0,Ts3=0;
	int16_t Ps_1 =0,Ps_2 =0,Ps_3= 0;
	int16_t cs1 =0,cs2 =0,cs3	= 0;
	  uint16_t v_C=0;
switch(cl_selftest_event_type)
{
	case EVENT_selftest_ALARM_PAAS:	
	{
		status[CL_SELFTEST_RESULT_ALARM] = TEST_PAAS;
		
			sv_cntrl_nobuzzer();
			sv_cntrl_resetredalarm();
			sv_cntrl_resetyellowalarm();
			sv_cntrl_resetgreenalarm();
// 			command = CON_TX_COMMAND_SELFTEST_START_ALARM;
// 			Cl_selftestretcode = Cl_SendDatatoconsole(command,&data,0);
			break;
	}
	
	case EVENT_selftest_ALARM_FAIL:	
	{
		status[CL_SELFTEST_RESULT_ALARM] = TEST_FAIL;
		sv_cntrl_nobuzzer();
		sv_cntrl_resetredalarm();
		sv_cntrl_resetyellowalarm();
		sv_cntrl_resetgreenalarm();
		break;
	}
	case EVENT_STOP_selftest:
	{
			
				 sv_cntrl_deactivatepump(UFPUMP);
				 sv_cntrl_deactivatepump(HEPARINPUMP);
				 sv_cntrl_deactivatepump(BLOODPUMP);
				sv_prop_bicarb_selftest_f_b();
				status[CL_SELFTEST_RESULT_ALARM] = TEST_FAIL;
				sv_prop_stopopenfill();
		sv_cntrl_nobuzzer();
		sv_cntrl_resetredalarm();
		sv_cntrl_resetyellowalarm();
		sv_cntrl_resetgreenalarm();
				
				command = CON_TX_COMMAND_SELFTEST_STOPPED;
				Cl_selftestretcode = Cl_SendDatatoconsole(command,&data,0);
				break;
			
	}
	case EVENT_selftest_SYNC_ACK:
	status[CL_SELFTEST_RESULT_SYNC] = 1;
	break;
	
	default:
	
	break;
}
	
	
	switch(cl_selfteststate)
	{
		case CL_SELFTEST_STATE_IDLE:
		switch(cl_selftest_event_type)
		{
			case EVENT_start_selftest:
			{
				
				data = 0;
				startpropFlag = false;
				durcount = 0;
			
			command = CON_TX_COMMAND_SELFTEST_STARTED;
			Cl_selftestretcode = Cl_SendDatatoconsole(command,&data,0);
			
		
			         cl_selfteststate = CL_SELFTEST_STATE_sync;
					//cl_selfteststate = CL_SELFTEST_STATE_START_ALARMS_BUZZERS;
			break;
			}
			
// 			case EVENT_STOP_selftest:
// 			{
// 				
// 				command = CON_TX_COMMAND_SELFTEST_STOPPED;
// 				Cl_selftestretcode = Cl_SendDatatoconsole(command,&data,0);
// 				break;
// 			}
// 			
		
 			default:
 			break;
		}
		break;
		case CL_SELFTEST_STATE_sync:
			switch (cl_selftest_event_type)
			
			{
					case EVENT_selftest_TICK_100MS:
						command = CON_TX_COMMAND_SELFTEST_SYNC_CHECK;
			Cl_selftestretcode = Cl_SendDatatoconsole(command,&data,0);
				cl_selfteststate = CL_SELFTEST_STATE_START_ALARMS_BUZZERS;
				break;
				default:
				break;
			}
			break;
		case CL_SELFTEST_STATE_START_ALARMS_BUZZERS:
	
			switch(cl_selftest_event_type)
			{
				case EVENT_selftest_TICK_100MS:
				if(status[CL_SELFTEST_RESULT_ALARM] == TEST_PAAS)
				{
				cl_selfteststate = CL_SELFTEST_STATE_START_UFPUMP;
				break;
				}
				else	
				{
					sv_cntrl_buzzer();
				sv_cntrl_setredalarm();
				sv_cntrl_setyellowalarm();
				sv_cntrl_setgreenalarm();
				
				command = CON_TX_COMMAND_SELFTEST_START_ALARM;
				Cl_selftestretcode = Cl_SendDatatoconsole(command,&data,0);
				cl_selfteststate = CL_SELFTEST_STATE_START_UFPUMP;
				break;
				}
				break;
			default:
			break;
		}
		break;
// 		case CL_SELFTEST_STATE_STOP_ALARMS_BUZZERS:
// 		
// 			switch(cl_selftest_event_type)
// 			{
// 				case EVENT_selftest_TICK_SECOND:
// 				test_time++;
// 				if(test_time>=5)
// 				{
// 					sv_cntrl_nobuzzer();
// 				sv_cntrl_resetredalarm();
// 				sv_cntrl_resetyellowalarm();
// 				sv_cntrl_resetgreenalarm();
// 				test_time = 0;
// 				cl_selfteststate = CL_SELFTEST_STATE_START_UFPUMP;
// 				break;
// 				}
// 				break;
// 				default:
// 				break;
// 			}
// 			break;
		
		
		
		case CL_SELFTEST_STATE_START_UFPUMP:
		switch(cl_selftest_event_type)
		{ 
			
			case EVENT_selftest_TICK_100MS:
			if(status[CL_SELFTEST_RESULT_UFPUMP] == TEST_PAAS)   
			{
				cl_selfteststate = CL_SELFTEST_STATE_START_HPPUMP;
				break;
			}
			
			else
			{
				//sv_cntrl_setpumpspeed(UFPUMP,1000);
				
			Cl_selftestretcode =  sv_cntrl_activatepump(UFPUMP);
			sv_cntrl_setpumpspeed(UFPUMP,12000);
			cl_wait(100);
			sv_cntrl_setpumpspeed(UFPUMP,8000);
			cl_wait(100);
			sv_cntrl_setpumpspeed(UFPUMP,6000);
			cl_wait(100);
			sv_cntrl_setpumpspeed(UFPUMP,1000);
			cl_wait(100);
			sv_cntrl_setpumpspeed(UFPUMP,500);
			cl_wait(100);
//			Cl_selftestretcode =  sv_cntrl_activatepump(UFPUMP);
             cl_selftestFeedback_start();
			cl_selfteststate = CL_SELFTEST_STATE_UFPUMP_STATUS;
			break;
			}
	//		break;
// 			case EVENT_STOP_selftest:
// 			//{
// 				status[CL_SELFTEST_RESULT_UFPUMP] = TEST_FAIL;
// 				command = CON_TX_COMMAND_SELFTEST_STOPPED;
// 				Cl_selftestretcode = Cl_SendDatatoconsole(command,&data,0);
// 				break;
// 			//}
// 			
			default:
			break;
		}
		break;	 
		
		case CL_SELFTEST_STATE_UFPUMP_STATUS:
		switch(cl_selftest_event_type)
		{
			case EVENT_selftest_TICK_SECOND:
			
		//	 int16_t uf_test_value = 0;
			 
				uf_test_value = uf_periodvalue;
				 
				 if ((uf_test_value >= Expected_uf_test_value) && (uf_test_value <= Expected_uf_test_value1))
				 {
					 status[CL_SELFTEST_RESULT_UFPUMP] = TEST_PAAS; //test[0] = 1;
					
					 sv_cntrl_deactivatepump(UFPUMP);
					 test_time = 0;
					 cl_selftestFeedback_stop();
					/* uf_test_value = 0;*/
					  
					 cl_selfteststate = CL_SELFTEST_STATE_START_HPPUMP;
				
					 break;
				 }
				 else
				 {
					  test_time++;
					 if (test_time > 3)
					 {
						 sv_cntrl_deactivatepump(UFPUMP);
						 cl_selftestFeedback_stop();
						   status[CL_SELFTEST_RESULT_UFPUMP] = TEST_FAIL  ;      // test[0] = 0;
						  test_time = 0;
						   cl_selftestFeedback_stop();
						 cl_selfteststate = CL_SELFTEST_STATE_START_HPPUMP;
						 
						 
						  break;
						 
					 }
				 }
				 break;
				 case EVENT_STOP_selftest:
				// {
					 status[CL_SELFTEST_RESULT_UFPUMP] = TEST_FAIL;
					 break;
				// }
			
			default:break;
		}
		
		break;

		case CL_SELFTEST_STATE_START_HPPUMP:
		switch(cl_selftest_event_type)
		{
			case EVENT_selftest_TICK_50MS:
			if(status[CL_SELFTEST_RESULT_HPPUMP] == TEST_PAAS)
		{
			 cl_selfteststate = CL_SELFTEST_STATE_START_BDPUMP;
			 break;
		}
		else
			{
				
			sv_cntrl_setHepa_dir();

			sv_cntrl_activatepump(HEPARINPUMP);
	        cl_wait(100);
			sv_cntrl_setpumpspeed(HEPARINPUMP,2);
			cl_hep_selftest_start();
			cl_selfteststate = CL_SELFTEST_STATE_HPPUMP_STATUS;
			break;
			}
			break;
			 case EVENT_STOP_selftest:
			 //{
				 status[CL_SELFTEST_RESULT_HPPUMP] = TEST_FAIL;
				 
				 break;
			// }
			default:break;
			
		}
		break;
		
		case CL_SELFTEST_STATE_HPPUMP_STATUS:
		switch(cl_selftest_event_type)
		{
			case EVENT_selftest_TICK_SECOND:
		//	 uint16_t hp_test_value = 0;
			
		 hp_test_value = hep_periodvalue;
			
			if( (hp_test_value >= Expected_hp_test_value1)&&(hp_test_value <= Expected_hp_test_value2))
			{
				status[CL_SELFTEST_RESULT_HPPUMP] = TEST_PAAS;
				sv_cntrl_deactivatepump(HEPARINPUMP);
				 cl_hep_selftest_stop();
				cl_selfteststate = CL_SELFTEST_STATE_START_BDPUMP;
		
				test_time = 0;
				break;
			}
			else
			{
				test_time++;
				if (test_time++ > 5)
				{
					sv_cntrl_deactivatepump(HEPARINPUMP);
					status[CL_SELFTEST_RESULT_HPPUMP] = TEST_FAIL;
					cl_hep_selftest_stop();
					cl_selfteststate = CL_SELFTEST_STATE_START_BDPUMP;
					
					test_time = 0;
					break;
					
				}
			}
			break;
			 case EVENT_STOP_selftest:
			// {
				 status[CL_SELFTEST_RESULT_HPPUMP] = TEST_FAIL;
				 break;
			// }
			
			default:break;
		}
		break;	
	
				case CL_SELFTEST_STATE_START_BDPUMP:
				switch(cl_selftest_event_type)
				{
					case EVENT_selftest_TICK_50MS:
					
                      if(status[CL_SELFTEST_RESULT_BDPUMP] == TEST_PAAS)
					  {
						  cl_selfteststate = CL_SELFTEST_STATE_START_PISTON_PUMP;
						  break;
					  }
					  else
					  {
						  
					  Cl_selftestretcode = sv_cntrl_activatepump(BLOODPUMP);
					  cl_wait(100);
					sv_cntrl_setpumpspeed(BLOODPUMP,200);

				
					cl_bp_selftest_start();
					cl_selfteststate = CL_SELFTEST_STATE_BDPUMP_STATUS;
					break;
					  }
					  break;
// 					  case EVENT_STOP_selftest:
// 					 // {
// 						  status[CL_SELFTEST_RESULT_BDPUMP] = TEST_FAIL;
// 						  break;
// 					//  }
					default:break;
					
				}
				break;
				
				case CL_SELFTEST_STATE_BDPUMP_STATUS:
				switch(cl_selftest_event_type)
				{
					case EVENT_selftest_TICK_SECOND:
			//		 uint16_t hp_test_value = 0;
					bp_test_value = bp_periodvalue;
					
					if ((bp_test_value >= Expected_bp_test_value1)&&(bp_test_value <= Expected_bp_test_value2))
					{
						status[CL_SELFTEST_RESULT_BDPUMP] = TEST_PAAS;
						sv_cntrl_deactivatepump(BLOODPUMP);
						cl_bp_selftest_stop();
						cl_selfteststate = CL_SELFTEST_STATE_START_PISTON_PUMP;
						test_time = 0;
						break;
					}
					else
					{
						test_time++;
						if (test_time > 3)
						{
							sv_cntrl_deactivatepump(BLOODPUMP);
							status[CL_SELFTEST_RESULT_BDPUMP] = TEST_FAIL;
							cl_selfteststate = CL_SELFTEST_STATE_START_PISTON_PUMP;
							cl_bp_selftest_stop();
							test_time = 0;
							break;
							
						}
					}
					break;
					 case EVENT_STOP_selftest:
					// {
						 status[CL_SELFTEST_RESULT_BDPUMP] = TEST_FAIL;
						 break;
					// }
					
					default:break;
				}
				break;
				
			
// 			case CL_SELFTEST_STATE_START_ACIDPUMP:
// 			switch(cl_selftest_event_type)
// 			{
// 				case EVENT_selftest_TICK_50MS:
//				sv_prop_startacidpump();
// 
// 				
// 				cl_selfteststate = CL_SELFTEST_STATE_ACIDPUMP_STATUS;
// 				break;
// 				default:break;
// 				
// 			}
// 			
// 			case CL_SELFTEST_STATE_ACIDPUMP_STATUS:
// 			switch(cl_selftest_event_type)
// 			{
// 				case EVENT_selftest_TICK_SECOND:
// 				
// 				
// 				
// 				if (bp_test_value == Expected_bp_test_value)
// 				{
// 					test[3] = 1;
// 					sv_cntrl_deactivatepump(UFPUMP);
// 					cl_bp_selftest_stop();
// 					test_time = 0;
// 					cl_selfteststate = CL_SELFTEST_STATE_START_BICARBPUMP;
// 					
// 					break;
// 				}
// 				else
// 				{
// 					test_time++;
// 					if (test_time++ > 3)
// 					{
// 						sv_cntrl_deactivatepump(UFPUMP);
// 						test[3] = 0;
// 						cl_selfteststate = CL_SELFTEST_STATE_START_BICARBPUMP;
// 						cl_bp_selftest_stop();
// 						test_time = 0;
// 						break;
// 						
// 					}
// 				}
// 				
// 				default:break;
// 			
// 			}
		case CL_SELFTEST_STATE_START_PISTON_PUMP:
		switch(cl_selftest_event_type)
		{
			
			
			case EVENT_selftest_TICK_500MS:
			if((status[CL_SELFTEST_RESULT_BICARBPUMP] == TEST_PAAS) && (status[CL_SELFTEST_RESULT_ACIDPUMP] == TEST_PAAS))
			{
				cl_selfteststate = CL_SELFTEST_STATE_APT_STATUS;
				break;
			}
			
			else
			{
				
			time_count++;
			if(time_count==2)
			{
            sv_prop_startpropeo_aligning();
			break;
			}
			else if (time_count==15)
			{
           
				 //sv_prop_bicarb_selftest();
				 sv_prop_startopenfill(); 
				 startpropFlag = true;
			
			break;
			}
			else if(startpropFlag){
				
			durcount++;
			if(durcount >=2){
				startpropFlag = false;	
				sv_prop_stopopenfill();
				time_count = 0;
			cl_selfteststate = CL_SELFTEST_STATE_STOP_PISTON_PUMP;
			
			}
			
			}
			}
			break;
			default:
			break;
			
		}
		break;
		
		case CL_SELFTEST_STATE_STOP_PISTON_PUMP:
		switch(cl_selftest_event_type)
		{
			case EVENT_selftest_TICK_SECOND:
		test_time++;
		if (test_time==1)
		{
				sv_prop_bicarb_selftest_f_b();
				
		}
		if (test_time==5)
		
			{	
				sv_prop_startpropeo_aligning();
				cl_selfteststate = CL_SELFTEST_STATE_ACIDPUMP_STATUS;
				time_count = 0;
				test_time =0;
			}
				break;
				
		
// 			else if(time_count>=5)
// 			
// 			{
// //				static int16_t af_b;
//                static int16_t bf_b;
// //				Cl_SysStat_GetSensor_Status_Query(SENSOR_ACID_F_B,&af_b);
// 				Cl_SysStat_GetSensor_Status_Query(SENSOR_ACID_F_B,&bf_b);
// 				
// // 				if ((af_b == Expected_bicarb_test_value1) || (af_b == Expected_bicarb_test_value2))
// // 				{
// // 					status[CL_SELFTEST_RESULT_ACIDPUMP] = TEST_PAAS;
// // 						cl_selfteststate =  CL_SELFTEST_STATE_APT_STATUS;
// // 						test_time = 0;
// // 						time_count = 0;
// 				    
// 		//		}
// 			if ((bf_b == Expected_bicarb_test_value1) || (bf_b == Expected_bicarb_test_value2))
// 			{
// 				status[CL_SELFTEST_RESULT_BICARBPUMP] = TEST_PAAS;
// 			//	SV_CAN_WRITE_PROPO( PROP_STOP_MIXING,0);
// 			
// 				cl_selfteststate =  CL_SELFTEST_STATE_APT_STATUS;
// 				test_time = 0;
// 				time_count = 0;
// 				break;
// 			}
// 				
// 		
// 			else
// 			{
// // 				test_time++;
// // 				if (test_time++ > 3)
// // 				{
// // 					sv_cntrl_deactivatepump(UFPUMP);
// // 					test[4] = 0;
// // 					cl_selfteststate =  CL_SELFTEST_STATE_APT_STATUS;
// // 					cl_bp_selftest_stop();
// // 					test_time = 0;
// // 					break;
// // 					
// // 				}
//              status[CL_SELFTEST_RESULT_BICARBPUMP] = TEST_FAIL;
// 			 test_time = 0;
// 			 cl_selfteststate =  CL_SELFTEST_STATE_APT_STATUS;
// 			 break;
// 			}
// 			break;
// 			}
			  case EVENT_STOP_selftest:
			 // {
				  status[CL_SELFTEST_RESULT_BICARBPUMP] = TEST_FAIL;
				  break;
			 // }
			
			default:break;
		}
		break;
		case CL_SELFTEST_STATE_ACIDPUMP_STATUS:
	    switch(cl_selftest_event_type)
		{
			case EVENT_selftest_TICK_SECOND:
				time_count++;
				if(time_count==2)
				{
				Cl_SysStat_GetSensor_Status_Query(SENSOR_ACID_F_B,&af_b);
				if ((af_b >= Expected_acid_test_value1) && (af_b <= Expected_acid_test_value2))
				 				{
				 					status[CL_SELFTEST_RESULT_ACIDPUMP] = TEST_PAAS;
				 						cl_selfteststate =  CL_SELFTEST_STATE_BICARBPUMP_STATUS;
										 time_count=0;
										 af_b=0;
										 break;
			
		                         }
		else 
		{
			status[CL_SELFTEST_RESULT_ACIDPUMP] = TEST_FAIL;
			cl_selfteststate =  CL_SELFTEST_STATE_BICARBPUMP_STATUS;
			af_b=0;
			time_count=0;
			break;
		}
			
				}
		break;
		default:
		break;
		
		}
		
		break;
		case CL_SELFTEST_STATE_BICARBPUMP_STATUS:
		  switch(cl_selftest_event_type)
		  {
			  case EVENT_selftest_TICK_500MS:
			  
			
			  Cl_SysStat_GetSensor_Status_Query(SENSOR_ACID_F_B,&bf_b);
			  if ((bf_b >= Expected_bicarb_test_value1) && (bf_b <= Expected_bicarb_test_value2))
			  {
				  status[CL_SELFTEST_RESULT_BICARBPUMP] = TEST_PAAS;
				  cl_selfteststate = CL_SELFTEST_STATE_APT_STATUS;
				  bf_b=0;
				  break;
				  
			  }
			  else
			  {
				  status[CL_SELFTEST_RESULT_BICARBPUMP] = TEST_FAIL;
				  cl_selfteststate =  CL_SELFTEST_STATE_APT_STATUS;
				  bf_b=0;
				  break;
			  }
			  break;
			  default:
			  break;
			  
		  }
		
		
		break;
		
		
		
		
		
			case CL_SELFTEST_STATE_APT_STATUS:
			switch(cl_selftest_event_type)
			{
				
				case EVENT_selftest_TICK_SECOND:
				if (status[CL_SELFTEST_RESULT_APT_STATUS] == TEST_PAAS)
				{
					cl_selfteststate = CL_SELFTEST_STATE_VPT_STATUS;
					break;
				}
				else{
				
				test_time++;
				
					Cl_SysStat_GetSensor_Status_Query(SENSOR_APTSTATUS,&apt_val);
				
						apt_val=apt_val-1000;
						if((apt_val >= expected_lowerapt_value)&&(apt_val <= expected_higherapt_value))
							{
								status[CL_SELFTEST_RESULT_APT_STATUS] = TEST_PAAS;
								test_time = 0;
								cl_selfteststate =  CL_SELFTEST_STATE_VPT_STATUS;
								break;
							}
							
							else
							{
								status[CL_SELFTEST_RESULT_APT_STATUS] = TEST_FAIL;
								test_time = 0;
								cl_selfteststate =  CL_SELFTEST_STATE_VPT_STATUS;
								break;
							}
				}
				break;
							default:
							break;
							
						
					
			}
			break;
					case CL_SELFTEST_STATE_VPT_STATUS:
					switch(cl_selftest_event_type)
					{
						
						case EVENT_selftest_TICK_SECOND:
						if (status[CL_SELFTEST_RESULT_VPT_STATUS] == TEST_PAAS)
						{
							cl_selfteststate = CL_SELFTEST_STATE_SENSOR_TEMPSTATUS;
							break;
						}
						else{
						test_time++;
						
					//	uint16_t vpt_val = 0;
						
						Cl_SysStat_GetSensor_Status_Query(SENSOR_VPTSTATUS,&vpt_val);
					//	vpt_val=vpt_val/10;
							vpt_val=(vpt_val-1000);
								if((vpt_val >= expected_lowerapt_value)&&(vpt_val <= expected_higherapt_value) )
								{
									status[CL_SELFTEST_RESULT_VPT_STATUS] = TEST_PAAS;
									test_time = 0;
									cl_selfteststate =  CL_SELFTEST_STATE_SENSOR_TEMPSTATUS;
									break;
								}
								
								else
								{
									status[CL_SELFTEST_RESULT_VPT_STATUS] = TEST_FAIL ;
									test_time = 0;
									cl_selfteststate =  CL_SELFTEST_STATE_SENSOR_TEMPSTATUS;
									break;
								}
						}
						break;
								default:break;
							}
					break;
				
				case CL_SELFTEST_STATE_SENSOR_TEMPSTATUS:
				switch(cl_selftest_event_type)
				{
					
					case  EVENT_selftest_TICK_500MS:
					if ((status[CL_SELFTEST_RESULT_TEMPSTATUS1] == TEST_PAAS) && (status[CL_SELFTEST_RESULT_TEMPSTATUS2] == TEST_PAAS) && (status[CL_SELFTEST_RESULT_TEMPSTATUS3] == TEST_PAAS))
					{
						cl_selfteststate = CL_SELFTEST_STATE_SENSOR_PSTATUS;   //	Cl_SysStat_GetSensor_Status_Query(SENSOR_TEMP1STATUS,&TEMP_DATA.Twobyte);
						break;                                                      //Cl_SysStat_GetSensor_Status_Query(SENSOR_TEMP2STATUS,&TEMP_DATA.Twobyte)
					}
					else
					{
						
							
						
						Cl_SysStat_GetSensor_Status_Query(SENSOR_TEMP1STATUS, &Ts1);
						Ts1=Ts1/100;
					
						Cl_SysStat_GetSensor_Status_Query(SENSOR_TEMP2STATUS, &Ts2);
						Ts2=Ts2/100;
						Cl_SysStat_GetSensor_Status_Query(SENSOR_TEMP3STATUS, &Ts3);
						Ts3=Ts3/100;
						
					test_time++;
						if(test_time == 1)
						{
							if((Ts1 >= expected_lowertemp1_value)&&(Ts1 <= expected_highertemp1_value) )
							
							{
								status[CL_SELFTEST_RESULT_TEMPSTATUS1] = TEST_PAAS;
								break;
							}
							
							
							else
							{
								status[CL_SELFTEST_RESULT_TEMPSTATUS1] = TEST_FAIL;
								break;
							}
						}
						
						if((test_time == 2))
						{
							if((Ts2 >= expected_lowertemp2_value)&&(Ts2 <= expected_highertemp2_value) )
							
							{
								status[CL_SELFTEST_RESULT_TEMPSTATUS2] = TEST_PAAS;
								break;
							}
							
							
							else
							{
								status[CL_SELFTEST_RESULT_TEMPSTATUS2] = TEST_FAIL;
								break;
							}
						}
						
							
							if((test_time >= 3))
							{
								if((Ts3 >= expected_lowertemp3_value)&&(Ts3 <= expected_highertemp3_value) )
								
								{
									status[CL_SELFTEST_RESULT_TEMPSTATUS3] = TEST_PAAS;
									cl_selfteststate = CL_SELFTEST_STATE_SENSOR_PSTATUS;
									test_time = 0;
									break;
								}
								
								
								else
								{
									status[CL_SELFTEST_RESULT_TEMPSTATUS3] = TEST_FAIL;
									cl_selfteststate = CL_SELFTEST_STATE_SENSOR_PSTATUS;
									test_time = 0;
									break;
								}
							}
					}
					break;
					default:
					break;
		}
		
				break;
				
				case CL_SELFTEST_STATE_SENSOR_PSTATUS:
				switch(cl_selftest_event_type)
				{
					case  EVENT_selftest_TICK_500MS:
					if ((status[CL_SELFTEST_RESULT_PSTATUS1] == TEST_PAAS) && (status[CL_SELFTEST_RESULT_PSTATUS2] == TEST_PAAS) && (status[CL_SELFTEST_RESULT_PSTATUS3] == TEST_PAAS))
					{
						cl_selfteststate = CL_SELFTEST_STATE_SENSOR_CONDSTATUS;
						break;
					}
				else	
				{
					
			Cl_SysStat_GetSensor_Status_Query(SENSOR_PS1STATUS, &Ps_1);
		Ps_1=(Ps_1-1000);
			Cl_SysStat_GetSensor_Status_Query(SENSOR_PS2STATUS, &Ps_2);
			Ps_2=(Ps_2-1000);
			Cl_SysStat_GetSensor_Status_Query(SENSOR_PS3STATUS, &Ps_3);
			Ps_3=(Ps_3-1000);
			test_time++;
			if(test_time == 1)
			{
				if((Ps_1 >= expected_lowerPS1_value)&&(Ps_1<=expected_higherPS1_value) )
			
					{
						status[CL_SELFTEST_RESULT_PSTATUS1] = TEST_PAAS;
						break;
					}
					
					
					else
					{
						status[CL_SELFTEST_RESULT_PSTATUS1] = TEST_FAIL;
						break;
					}
			}
					
					if((test_time == 2) ) 
					{
						if((Ps_2 >= expected_lowerPS2_value)&&(Ps_2 <=expected_higherPS2_value) )
						
						{
							status[CL_SELFTEST_RESULT_PSTATUS2] = TEST_PAAS;
							break;
						}
						
						
						else
						{
							status[CL_SELFTEST_RESULT_PSTATUS2] = TEST_FAIL;
							break;
						}
					}
						if((test_time==3) )
							{
								if((Ps_3 >= expected_lowerPS3_value)&& (Ps_3<=expected_higherPS3_value))
								
								{
									status[CL_SELFTEST_RESULT_PSTATUS3] = TEST_PAAS;
									test_time=0;
									cl_selfteststate =  CL_SELFTEST_STATE_SENSOR_CONDSTATUS;
									break;
								}
								
								
								else
								{
									status[CL_SELFTEST_RESULT_PSTATUS3] = TEST_FAIL;
									test_time=0;
									cl_selfteststate =  CL_SELFTEST_STATE_SENSOR_CONDSTATUS;
									break;
								}
							}
				}
				break;
						default:break;
					
				}
				break;
// 				
// 					case CL_SELFTEST_STATE_SENSOR_VENOUSCLAMP:
// 					switch(cl_selftest_event_type)
// 					{
// 						case EVENT_selftest_TICK_50MS:
// 					if (status[CL_SELFTEST_RESULT_VENOUSCLAMP] == TEST_PAAS)
// 						{
// 							cl_selfteststate = CL_SELFTEST_STATE_COMPLETED;
// 							break;
// 						}
//                      else{
//                         Cl_SysStat_GetSensor_Status_Query(SENSOR_VC_F_B,&v_C );
// 					sv_cntrl_activatevenousclamp();
// 					
// 
// 						
// 					
// 						cl_selfteststate = CL_SELFTEST_STATE_VENOUSCLAMP_STATUS;
// 						break;
// 					 }
// 					 break;
// 						default:break;
// 						
// 					}
// 					break;
					
							case CL_SELFTEST_STATE_SENSOR_CONDSTATUS:
							switch(cl_selftest_event_type)
							{
								
								case  EVENT_selftest_TICK_500MS:
								if ((status[CL_SELFTEST_RESULT_CONDSTATUS1] == TEST_PAAS) && (status[CL_SELFTEST_RESULT_CONDSTATUS2] == TEST_PAAS) && (status[CL_SELFTEST_RESULT_CONDSTATUS3] == TEST_PAAS))
								{
									cl_selfteststate = CL_SELFTEST_STATE_SENSOR_VENOUSCLAMP;   //	Cl_SysStat_GetSensor_Status_Query(SENSOR_TEMP1STATUS,&TEMP_DATA.Twobyte);
									break;                                                      //Cl_SysStat_GetSensor_Status_Query(SENSOR_TEMP2STATUS,&TEMP_DATA.Twobyte)
								}
								else
								{
									
									Cl_SysStat_GetSensor_Status_Query(SENSOR_ACID_COND, &cs1);//SENSOR_COND_STATUS
									cs1 =cs1/100;
									Cl_SysStat_GetSensor_Status_Query(SENSOR_BICARB_COND, &cs2);
									cs2=cs2/100;
									Cl_SysStat_GetSensor_Status_Query(SENSOR_COND_STATUS, &cs3);
									cs3=cs3/100;
									
									test_time++;
									if(test_time == 1)
									{
										if((cs1 >= expected_lowercond1_value)&&(cs1 <= expected_highercond1_value) )
										
										{
											status[CL_SELFTEST_RESULT_CONDSTATUS1] = TEST_PAAS;
											break;
										}
										
										
										else
										{
											status[CL_SELFTEST_RESULT_CONDSTATUS1] = TEST_FAIL;
											break;
										}
									}
									
									if((test_time == 2))
									{
										if((cs2 >= expected_lowercond2_value)&&(cs2 <= expected_highercond2_value) )
										
										{
											status[CL_SELFTEST_RESULT_CONDSTATUS2] = TEST_PAAS;
											break;
										}
										
										
										else
										{
											status[CL_SELFTEST_RESULT_CONDSTATUS2] = TEST_FAIL;
											break;
										}
									}
									
									
									if((test_time >= 3))
									{
										if((cs3 >= expected_lowercond3_value)&&(cs3 <= expected_highercond3_value) )
										
										{
											status[CL_SELFTEST_RESULT_CONDSTATUS3] = TEST_PAAS;
											cl_selfteststate = CL_SELFTEST_STATE_SENSOR_VENOUSCLAMP;
											test_time = 0;
											break;
										}
										
										
										else
										{
											status[CL_SELFTEST_RESULT_CONDSTATUS3] = TEST_FAIL;
											cl_selfteststate = CL_SELFTEST_STATE_SENSOR_VENOUSCLAMP;
											test_time = 0;
											break;
										}
									}
								}
								break;
								default:
								break;
							}
							
							break;
							
							case CL_SELFTEST_STATE_SENSOR_VENOUSCLAMP:
							switch(cl_selftest_event_type)
							{
								case EVENT_selftest_TICK_50MS:
								if (status[CL_SELFTEST_RESULT_VENOUSCLAMP] == TEST_PAAS)
								{
									cl_selfteststate = CL_SELFTEST_STATE_COMPLETED;
									 Cl_mac_apprequesthandler(MACREQ_SELFTEST_COMPLETED);
									break;
								}
								else{
								/*	Cl_SysStat_GetSensor_Status_Query(SENSOR_VC_F_B,&v_C );*/
									sv_cntrl_activatevenousclamp();
									v_C =0;

									
									
									cl_selfteststate = CL_SELFTEST_STATE_VENOUSCLAMP_STATUS;
									break;
								}
								break;
								default:break;
								
							}
							break;
					
		           case CL_SELFTEST_STATE_VENOUSCLAMP_STATUS:
		           switch(cl_selftest_event_type)
		           {
			           case EVENT_selftest_TICK_SECOND:
                    
					   test_time++;
					   if(test_time=2)

			          Cl_SysStat_GetSensor_Status_Query(SENSOR_VC_F_B,&v_C );    //for venous
					   if(v_C==1)
					   {
						  status[CL_SELFTEST_RESULT_VENOUSCLAMP] = TEST_PAAS; 
						     sv_cntrl_deactivatevenousclamp();
 						   Cl_mac_apprequesthandler(MACREQ_SELFTEST_COMPLETED);
						
						   cl_selfteststate = CL_SELFTEST_STATE_COMPLETED;
						    test_time =0;
						  break;
						  
					   }
					   else
					   {
						   status[CL_SELFTEST_RESULT_VENOUSCLAMP] = TEST_FAIL;
						   sv_cntrl_deactivatevenousclamp(); 
						    Cl_mac_apprequesthandler(MACREQ_SELFTEST_COMPLETED);
							
							   cl_selfteststate =CL_SELFTEST_STATE_COMPLETED;
							    test_time =0;
						   break;
						 
					   }

			          
			           
			          
			           break;
			           default:break;
			           
		           }
				   break;
		   
		 case CL_SELFTEST_STATE_COMPLETED:
		 switch(cl_selftest_event_type)
		 {
			 case EVENT_selftest_TICK_100MS:
			 break;
			 case EVENT_selftest_COMPLETED:
			// {
				 command = CON_TX_COMMAND_SELFTEST_REPORT;
				 Cl_selftestretcode = Cl_SendDatatoconsole(command,&status,19);
				 
				 for(int16_t i=0;i<CL_SELFTEST_RESULT_MAXSTATE;i++)
				 {
					 if (status[i]== 0)
					 {
					 	command = CON_TX_COMMAND_SELFTEST_FAILED;
					 	Cl_selftestretcode = Cl_SendDatatoconsole(command,&data,0);
						 break;
					 }
					 
					 else
					 {
						 command = CON_TX_COMMAND_SELFTEST_PASSED;
						 Cl_selftestretcode = Cl_SendDatatoconsole(command,&data,0);
					 }
				 }
				 
               break;
			// }
			 default:break;
			 
		 }
		
      default:break;
		
		
		
		
		
		
					
	
	}	
	return CL_OK;
		
	}
		
		
		
		
		
		
		
		
		
		
		
		
		
		
		
		
		
		
		
		
		
		
		
		
		
		
		
		
		
		
	
	Cl_ReturnCodeType cl_selftest_translatemacevent(Cl_Mac_EventsType Cl_MacselftestEvent,cl_selftest_event* cl_selftest_event_type) 
	{
		switch(Cl_MacselftestEvent)
		{

			
			case  EVT_CONSOLE_COMMAND_SELFTEST_START:
			*cl_selftest_event_type = EVENT_start_selftest;
			break;
			
			case EVT_COSOLE_COMMAND_SELFTEST_ALARM_PAAS:
			*cl_selftest_event_type =EVENT_selftest_ALARM_PAAS;
			break;
			case EVT_COSOLE_COMMAND_SELFTEST_ALARM_FAIL:
		    *cl_selftest_event_type =EVENT_selftest_ALARM_FAIL;
		    break;
			
			case EVT_CONSOLE_COMMAND_SYNCCONFIRMED:
			 *cl_selftest_event_type =EVENT_selftest_SYNC_ACK;
			 break;
			case EVT_CONSOLE_COMMAND_SELFTEST_STOP:
			*cl_selftest_event_type = EVENT_STOP_selftest;
			break;
			case MACREQ_SELFTEST_COMPLETED:
			*cl_selftest_event_type = EVENT_selftest_COMPLETED;
			break;
			
			
			case EVT_TICK_50M:
			*cl_selftest_event_type =   EVENT_selftest_TICK_50MS;
			break;
			break;
			case EVT_TICK_100M:
			*cl_selftest_event_type =  EVENT_selftest_TICK_100MS;
			break;
			case EVT_TICK_500M:
			*cl_selftest_event_type =   EVENT_selftest_TICK_500MS;
			break;
			case EVT_TICK_SEC:
			*cl_selftest_event_type =  EVENT_selftest_TICK_SECOND;
			break;
			case EVT_TICK_MIN:
			*cl_selftest_event_type =   EVENT_selftest_TICK_MINUTE;
			break;
			default:
			break;
		}
		return CL_OK;
	}	
	
	
	
	
	Cl_ReturnCodeType Cl_selftest_init(void)
	{
		
		cl_selfteststate = CL_SELFTEST_STATE_IDLE;

		//cl_isouf_event = EVENT_NULL_EVENT;
		
		selftest_second_counter=0 ;
		return CL_OK;
	}

	















 















  
 





	








































// 
// Cl_ReturnCodeType  cl_test_get_sensordata(Cl_Uint8Type sensorId, uint16_t* sensordata)
// {
// 
// 	
// 	Cl_SysStat_GetSensor_Status_Query(sensorId, sensordata);
// 	
// 	return CL_OK;
// }









