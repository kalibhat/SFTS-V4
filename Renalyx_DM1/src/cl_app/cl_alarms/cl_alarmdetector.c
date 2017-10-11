/*
 * cl_alarmdetector.c
 *
 * Created: 1/22/2014 10:02:29 AM
 *  Author: user
 */ 


#include "cl_alarm_include.h"
#include "cl_alarm_proto.h"

extern float avgtemp2_dummy,avgtemp3_dummy,cond_dummy;
//extern Cl_Mac_EventQueueType Cl_MacEventQueue1;
extern Cl_Mac_EventQueueType Cl_MacEventQueue4;     // Alarms Que
extern Cl_Mac_EventQueueType Cl_MacEventQueueBc ;   // Balancing Chamber Que
extern Cl_Sys_statusType cl_sys_statbuffer;
extern Cl_BoolType gflow_en;
extern Cl_BoolType TEST_STUB_ENABLEGATE;

extern Cl_Levelswitch_msgType Levelswitch_msg;
extern Cl_Flowswitch_msgType Flowswitch_msg;
extern Cl_testsensorType test_sensor_status[SENSOR_MAX_INPUT];

static Cl_Uint32Type prev_flowstatus = 0;
static Cl_Uint32Type flow_watchdog = 0,flow_count=0,flow_count_rate = 0;
Cl_AlarmStructType Cl_alarms_alarms[_ALARM_MAX_ID];
Cl_NewAlarmIdType  cl_lastalarmid;
static Cl_Uint8Type levelswitchprevstatus = 1;
extern  Cl_Uint32Type flowstatus_off_alarm_count;// testing
Cl_NewAlarmIdType cond_alarm = CL_COND_ALARM_NO_ALARM;
Cl_NewAlarmIdType dac_cond_alarm = CL_DAC_COND_ALARM_NO_ALARM;
Cl_BoolType cl_alarm_triggered = false;
Cl_Int16Type ps3_threshold = 4096;
Cl_Int16Type FPcurrentstatus_prev=0;
Cl_AlarmThresholdType  Cl_alarmThresholdTable;

float apt , prev_aptval = 0;
float tmpVal = 0;
float vpt = 0 , prev_vptval = 0;
float ps1 = 0 , prev_ps1val = 0;
static float ps2 = 0;
float ps3 = 0, prev_ps3val = 0;
float temp1 =0,prev_temp1val = 0;
float temp2 = 0,prev_temp2val = 0 ;
float temp3 = 0 , prev_temp3val = 0;

float cond1 = 0, cond_final=0;

int powerfailurecount = 0;
extern Cl_Uint8Type DD_SET_VALVE(sv_valvetype );// sanjeer -testing
extern Cl_Uint8Type DD_RESET_VALVE(sv_valvetype );

extern volatile float temprature_final_value_1,temprature_final_value_2,temprature_final_value_3,cond_final_cs3;
extern volatile float pressure_final_apt,pressure_final_vpt,pressure_final_ps1,pressure_final_ps2,pressure_final_ps3;
extern float res_temp_value;
//extern Cl_SysStat_GetSensor_Status_Query(Cl_SensorDeviceIdType dataID, uint16_t* pdata);

  /*
 * \brief  Cl_Init_Alarms(void) - Initialization of alarm detection module 
 * \param   
 * \ 
 * \return CL_OK on success. 
 * \Globals used : 
 */
  
Cl_ReturnCodeType Cl_Init_Alarms(void)
{
	powerfailurecount = 0;
	Cl_ReturnCodeType cl_Alarm_retVal = CL_ERROR;
	cl_Alarm_retVal = Cl_LoadAlarmThresholdTable();
	ASSERT_ERROR(cl_Alarm_retVal)
	cl_Alarm_retVal = Cl_Temperature_Controller_Init();
	ASSERT_ERROR(cl_Alarm_retVal)
	cl_Alarm_retVal = Cl_Pressure_Controller_Init();
	ASSERT_ERROR(cl_Alarm_retVal)
	return CL_OK;
	
}
  /*
 * \brief  Cl_LoadAlarmThresholdTable(void) - Initialization of alarm threshold table with default values. 
 * \param   
 * \ 
 * \return CL_OK on success. 
 * \Globals used : 
 */
  
Cl_ReturnCodeType Cl_LoadAlarmThresholdTable(void)
{
		Cl_alarmThresholdTable.cond_low_threshold = 20;   // manav03
		Cl_alarmThresholdTable.cond_high_threshold = 145;  // manav03
		Cl_alarmThresholdTable.cond_dac_high_threshold = 0;
		Cl_alarmThresholdTable.cond_dac_low_threshold = 10000;
		Cl_alarmThresholdTable.apt_high_threshold = 1500; // manav03
		Cl_alarmThresholdTable. apt_low_threshold = 700; // manav03
		Cl_alarmThresholdTable.vpt_high_threshold = 1500;
		Cl_alarmThresholdTable.vpt_low_threshold = 700;  // manav03
		Cl_alarmThresholdTable.ps1_high_threshold = 400;
		Cl_alarmThresholdTable.ps1_low_threshold = -200;
		Cl_alarmThresholdTable.ps2_high_threshold = 400;
		Cl_alarmThresholdTable.ps2_low_threshold = -300;
		Cl_alarmThresholdTable.ps3_high_threshold = 1000;
		Cl_alarmThresholdTable.ps3_low_threshold = -300;
		Cl_alarmThresholdTable.temp1_high_threshold = 37;
		Cl_alarmThresholdTable.temp1_low_threshold = 25;
		Cl_alarmThresholdTable.temp2_high_threshold = 80;
		Cl_alarmThresholdTable.temp2_low_threshold = 25;
		Cl_alarmThresholdTable.temp3_high_threshold = 375;  // manav03
		Cl_alarmThresholdTable.temp3_low_threshold = 250;   // manav03
		Cl_alarmThresholdTable.heater_high_temp_threshold = 45;
		Cl_alarmThresholdTable.hp_high_speed_threshold = 10000;
		Cl_alarmThresholdTable.hp_low_speed_threshold = 0;
		Cl_alarmThresholdTable.bp_high_speed_threshold = 10000;
		Cl_alarmThresholdTable.bp_low_speed_threshold = 0;
		Cl_alarmThresholdTable.uf_high_speed_threshold = 10000;
		Cl_alarmThresholdTable.uf_low_speed_threshold = 0;
		Cl_alarmThresholdTable.flow_low_threshold = 5;
		Cl_alarmThresholdTable.flow_high_threshold = 20;
		Cl_alarmThresholdTable.ls_toggle_high_threshold = 30;
		Cl_alarmThresholdTable.ls_toggle_low_threshold = 5;
		Cl_alarmThresholdTable.bc_toggle_high_threshold = 30;
		Cl_alarmThresholdTable.bc_toggle_low_threshold = 5;
		Cl_alarmThresholdTable.tmp_low_threshold  = -400;
		Cl_alarmThresholdTable.tmp_high_threshold  = 400;
		//Cl_alarmThresholdTable.tmp_low_threshold  = 1500;
		return CL_OK ;
}

  /*
 * \brief  Cl_Alarm_translate_sensor_values(void) -  Read sensor values and average to filter out noise .
			Also, if test flag is enabled used test values instead of sensor values
 * \param   
 * \ 
 * \return CL_OK on success. 
 * \Globals used : test_sensor_status,cl_sys_statbuffer
 */
// manav02 function added
Cl_ReturnCodeType Cl_Alarm_translate_sensor_values(void){
	
	Cl_ReturnCodeType cl_alarm_retval = CL_ERROR;
	float temp;
	Cl_Uint16Type sensordata = 0 , ps1data  = 0 , ps2data  = 0 , ps3data  = 0 ;
	
		Cl_SysStat_GetSensor_Status_Query(SENSOR_TEMP1STATUS,&sensordata);
		{
			temp1 = (float)sensordata/10;
		}
		Cl_SysStat_GetSensor_Status_Query(SENSOR_TEMP2STATUS,&sensordata);
		{
			temp2 = (float)sensordata/10;
		}
		Cl_SysStat_GetSensor_Status_Query(SENSOR_TEMP3STATUS,&sensordata);
		{
			temp3 = (float)sensordata/10;
		}
	
		Cl_SysStat_GetSensor_Status_Query(SENSOR_COND_STATUS,&sensordata);
		{
			cond_final = (float)sensordata/100;
		}
		Cl_SysStat_GetSensor_Status_Query(SENSOR_APTSTATUS,&sensordata);
		{
			apt = (float)sensordata - 1000;
		}
		Cl_SysStat_GetSensor_Status_Query(SENSOR_VPTSTATUS,&sensordata);
		{
			vpt = (float)sensordata - 1000;
		}

		Cl_SysStat_GetSensor_Status_Query(SENSOR_PS1STATUS,&ps1data);
		{
			ps1 = 	ps1data - 1000;
		}
		Cl_SysStat_GetSensor_Status_Query(SENSOR_PS2STATUS,&ps2data);
		{
			ps2 = 	ps2data - 1000;
		}
		Cl_SysStat_GetSensor_Status_Query(SENSOR_PS3STATUS,&ps3data);
		{
			ps3 = 	ps3data - 1000;
		}

		tmpVal = 	(((apt + vpt) - (ps1+ps2))/2)-25;
		
		

}
/*   manav02 changed the function body
Cl_ReturnCodeType Cl_Alarm_translate_sensor_values(void)
{							
	
	Cl_ReturnCodeType cl_alarm_retval = CL_ERROR;
	float temp;
	Cl_Uint16Type sensordata = 0;
	uint16_t Ps1,Ps2,Ps3;
	#if 0
	if ((temp1 -  prev_temp1val ) > 1)
	{
		temp1 = prev_temp1val;
	}
	if ((  prev_temp1val - temp1 ) > 1)
	{
		temp1 = prev_temp1val;
	}
	
	
	if ((temp2 -  prev_temp2val ) > 1)
	{
		temp2 = prev_temp2val;
	}
	if ((  prev_temp2val - temp2 ) > 1)
	{
		temp2 = prev_temp2val;
	}
	if ((  prev_temp3val - temp3 ) > 1)
	{
		temp3 = prev_temp3val;
	}
	if ((  prev_aptval - apt ) > 1)
	{
		apt = prev_aptval;
	}
	
	if ((  prev_vptval - vpt ) > 200)
	{
		vpt = prev_vptval;
	}
	if ((  prev_ps1val - ps1 ) > 200)
	{
		ps1 = prev_ps1val;
	}
	if ((  prev_ps2val - ps2 ) > 200)
	{
		ps2 = prev_ps2val;
	}
	if ((  prev_ps3val - ps3 ) > 200)
	{
		ps3 = prev_ps3val;
	}
	#endif
	Cl_SysStat_GetSensor_Status_Query(SENSOR_PS3STATUS,&Ps1);
	Cl_SysStat_GetSensor_Status_Query(SENSOR_PS3STATUS,&Ps2);
	Cl_SysStat_GetSensor_Status_Query(SENSOR_PS3STATUS,&Ps3);
	
	
				
	temp = ((cl_sys_statbuffer.aptstatus * 3300 /4096) ) ;
	calibration_apt(temp);
	if(test_sensor_status[SENSOR_APTSTATUS].test_flag == true)
	{
		Cl_SendDatatoconsole(CON_TX_COMMAND_PRINTTEXT,"TESTAPT",5);
		apt =  test_sensor_status[SENSOR_APTSTATUS].test_sensord_data;
	}
	else
	{
		apt = ((apt* 49) + pressure_final_apt)/50;
	}

	temp = ((cl_sys_statbuffer.vptstatus * 3300 /4096) ) ;
	calibration_vpt(temp);
	if(test_sensor_status[SENSOR_VPTSTATUS].test_flag == true)
	{
		Cl_SendDatatoconsole(CON_TX_COMMAND_PRINTTEXT,"TESTAPT",5);
		vpt =  test_sensor_status[SENSOR_VPTSTATUS].test_sensord_data;
	}
	else
	{
		vpt = ((vpt* 49) + pressure_final_vpt)/50;
	}

// 	temp = cl_sys_statbuffer.Temp1status * 0.805;
// 	calibration_tmp(temp,TS1);
	if(test_sensor_status[SENSOR_TEMP1STATUS].test_flag == true)
	{
		Cl_SendDatatoconsole(CON_TX_COMMAND_PRINTTEXT,"TESTT1",7);
		temp1 =  test_sensor_status[SENSOR_TEMP1STATUS].test_sensord_data;
	}
	else
	{
		
// 		uint16_t cal_data  = (402 *100* cl_sys_statbuffer.Temp1status)/(2*32768);
// 		res_temp_lookuptable(cal_data);
// 		
// 		uint16_t temp = res_temp_value/100;
// 		temp = temp - 2.5- 0.25;
// 								
// 		temp1 = ((temp1* 4) +temp)/5;
// 		//temp1=temp1*10; 
			Cl_SysStat_GetSensor_Status_Query(SENSOR_TEMP1STATUS,&sensordata);
			{
				
				temp1 = (float)sensordata/10;

				
			}
	}
	

	if(test_sensor_status[SENSOR_TEMP2STATUS].test_flag == true)
	{
		Cl_SendDatatoconsole(CON_TX_COMMAND_PRINTTEXT,"TESTT2",7);
		temp2 =  test_sensor_status[SENSOR_TEMP2STATUS].test_sensord_data;
	}
	else
	{
// 		/*		uint16_t cal_data  = (402 *100* cl_sys_statbuffer.Temp2status)/(2*32768);
// 				res_temp_lookuptable(cal_data);
// 				
// 				uint16_t temp = res_temp_value/100;
// 
// 		
// 					temp = temp - 25+0.81;
// 	
// 				
// 				
// 				temp2 = ((temp2* 4) +temp)/5;
// 		//temp2=temp2*10;
		
			Cl_SysStat_GetSensor_Status_Query(SENSOR_TEMP2STATUS,&sensordata);
			{
				
				temp2 = (float)sensordata/10;

				
			}
	}

	if(test_sensor_status[SENSOR_TEMP3STATUS].test_flag == true)
	{
		//Cl_SendDatatoconsole(CON_TX_COMMAND_PRINTTEXT,"TESTT3",7);
		temp3 =  test_sensor_status[SENSOR_TEMP3STATUS].test_sensord_data;
	}
	else
	{
// 				uint16_t cal_data  = (402 *100* cl_sys_statbuffer.Temp3status)/(2*32768);
// 				res_temp_lookuptable(cal_data);
// 				
// 				uint16_t temp = res_temp_value/100;
// 				temp = temp - 0.4;
// 				
// 				temp3 = ((temp3* 4) +temp)/5;
// 		//temp3=temp3*10; 
// 			
				Cl_SysStat_GetSensor_Status_Query(SENSOR_TEMP3STATUS,&sensordata);
				{
					
					temp3 = (float)sensordata/10;

					
				}
			
	}
	

	
	if(test_sensor_status[SENSOR_COND_STATUS].test_flag == true)
	{
		Cl_SendDatatoconsole(CON_TX_COMMAND_PRINTTEXT,"TESTCOND",5);
		cond_final =  test_sensor_status[SENSOR_COND_STATUS].test_sensord_data;
		
	}
	else
	{
		
// 		cond1 =  -0.0001 * cl_sys_statbuffer.cond_status * cl_sys_statbuffer.cond_status   + 0.36 * cl_sys_statbuffer.cond_status  - 38.39 ;
// 		cond_final= (cond1/(1+(temp3-25.0)*0.020)/10);

	Cl_SysStat_GetSensor_Status_Query(SENSOR_COND_STATUS,&sensordata);
	{
		
		float cond_comp=0;
		float temp2=0;
		float	 cond1,	cond_final, cond1x100, cond_final_X10000;
		sensordata = sensordata/10;
		if (sensordata > 50)
		{
			
			cond1 = sensordata/100;
			cond_final = 0.0001*cond1*cond1 + 0.032*cond1 +0.91 + 0.4;
			cond1x100 = cond_final*100; ;
			cond_final_X10000= (cond1x100/(1+(temp3/10  -25.0)*0.021));
			cond_final_X10000 =	cond_final_X10000 * 5.3 ;
			
			
			//	cond =  -0.0001 * temp* temp  + 0.36 * temp  - 38.39 ;
			//	cond_comp= cond/(1+(tmp3/100-25.0)*0.020);
			cond_final = (cond_final_X10000/10);
			
		}
		else
		{
			cond_final = 0;
		}

		
		
	}										

	}
	
}
*/
  /*
 * \brief  Cl_alarms_scanforAlarms(void) -  compare the various sensor values against the alarm threshold table and raise alarm event. 
 * \param   
 * \ 
 * \return CL_OK on success. 
 * \Globals used : Cl_alarms_alarms[]
 */
  
Cl_ReturnCodeType Cl_alarms_scanforAlarms(void)
{
	Cl_ReturnCodeType Cl_alarms_retcode;
	
	 Cl_alarms_retcode = Cl_Alarm_translate_sensor_values();
	 ASSERT_ERROR(Cl_alarms_retcode);
	
	if(Cl_alarms_alarms[POWER_FAILURE].cl_is_enabled){
		
		if(cl_sys_statbuffer.PowerFailure)
		{
			powerfailurecount++;

//if(powerfailurecount > 5){
			powerfailurecount = 0 ;
			Cl_alarms_alarms[POWER_FAILURE].cl_alarmstate = CL_ALARM_DETECTED;
			Cl_MacEventQueue4.Cl_MacEventData[Cl_MacEventQueue4.Cl_MacNewEventcount][0]=POWER_FAILURE; 
			Cl_alarms_alarms[POWER_FAILURE].cl_detected_count++;
			cl_alarm_triggered = true;
			cl_lastalarmid = POWER_FAILURE;
			Cl_MacEventQueue4.Cl_MacEvent[Cl_MacEventQueue4.Cl_MacNewEventcount] = EVT_ALARM_TRIGGERED ;
			Cl_alarms_retcode =  Cl_UpdateMacAlarmEventTable();
			ASSERT_ERROR(Cl_alarms_retcode);
//}
		}
		else
		{
			Cl_alarms_alarms[POWER_FAILURE].cl_alarmstate = CL_ALARM_INACTIVE;
		}
		
		
	}
	
	
	
	if (Cl_alarms_alarms[ACID_IN_OPEN].cl_is_enabled)
	{
		
		if(((Cl_alarms_alarms[ACID_IN_OPEN].cl_triggertype == LOGIC_HIGH) && (cl_sys_statbuffer.acid_inlet_status )) || ((Cl_alarms_alarms[ACID_IN_OPEN].cl_triggertype == LOGIC_LOW) && (!cl_sys_statbuffer.acid_inlet_status )) )
		{
			if(Cl_alarms_alarms[ACID_IN_OPEN].cl_alarmstate != CL_ALARM_DETECTED)
			{
			Cl_alarms_alarms[ACID_IN_OPEN].cl_alarmstate = CL_ALARM_DETECTED;
			Cl_MacEventQueue4.Cl_MacEventData[Cl_MacEventQueue4.Cl_MacNewEventcount][0]=ACID_IN_OPEN;
			Cl_alarms_alarms[ACID_IN_OPEN].cl_detected_count++;
			cl_alarm_triggered = true;
			cl_lastalarmid = ACID_IN_OPEN;
			Cl_MacEventQueue4.Cl_MacEvent[Cl_MacEventQueue4.Cl_MacNewEventcount] = EVT_ALARM_TRIGGERED ;
			Cl_alarms_retcode =  Cl_UpdateMacAlarmEventTable();
			ASSERT_ERROR(Cl_alarms_retcode);
			}
		}
		else
		{
			Cl_alarms_alarms[ACID_IN_OPEN].cl_alarmstate = CL_ALARM_INACTIVE;
		}	
	}
	if (Cl_alarms_alarms[ACID_IN_CLOSED].cl_is_enabled)
	{
		
		if(((Cl_alarms_alarms[ACID_IN_CLOSED].cl_triggertype == LOGIC_HIGH) && (cl_sys_statbuffer.acid_inlet_status )) || ((Cl_alarms_alarms[ACID_IN_CLOSED].cl_triggertype == LOGIC_LOW) && (!cl_sys_statbuffer.acid_inlet_status )) )
		{
			if(Cl_alarms_alarms[ACID_IN_CLOSED].cl_alarmstate != CL_ALARM_DETECTED)
			{
				Cl_alarms_alarms[ACID_IN_CLOSED].cl_alarmstate = CL_ALARM_DETECTED;
				Cl_MacEventQueue4.Cl_MacEventData[Cl_MacEventQueue4.Cl_MacNewEventcount][0]=ACID_IN_OPEN;
				Cl_alarms_alarms[ACID_IN_CLOSED].cl_detected_count++;
				cl_alarm_triggered = true;
				cl_lastalarmid = ACID_IN_CLOSED;
				Cl_MacEventQueue4.Cl_MacEvent[Cl_MacEventQueue4.Cl_MacNewEventcount] = EVT_ALARM_TRIGGERED ;
				Cl_alarms_retcode =  Cl_UpdateMacAlarmEventTable();
				ASSERT_ERROR(Cl_alarms_retcode);
			}
		}
		else
		{
			Cl_alarms_alarms[ACID_IN_OPEN].cl_alarmstate = CL_ALARM_INACTIVE;
		}
	}
	
	if (Cl_alarms_alarms[BICARB_IN_OPEN].cl_is_enabled)
	{
		
		if(((Cl_alarms_alarms[BICARB_IN_OPEN].cl_triggertype == LOGIC_HIGH) && (cl_sys_statbuffer.bicarb_inlet_status )) || ((Cl_alarms_alarms[BICARB_IN_OPEN].cl_triggertype == LOGIC_LOW) && (!cl_sys_statbuffer.bicarb_inlet_status )) )
		{
			if(Cl_alarms_alarms[BICARB_IN_OPEN].cl_alarmstate != CL_ALARM_DETECTED)
			{
			Cl_alarms_alarms[BICARB_IN_OPEN].cl_alarmstate = CL_ALARM_DETECTED;
			Cl_MacEventQueue4.Cl_MacEventData[Cl_MacEventQueue4.Cl_MacNewEventcount][0]=BICARB_IN_OPEN;
			Cl_alarms_alarms[BICARB_IN_OPEN].cl_detected_count++;
			cl_alarm_triggered = true;
			cl_lastalarmid = BICARB_IN_OPEN;
			Cl_MacEventQueue4.Cl_MacEvent[Cl_MacEventQueue4.Cl_MacNewEventcount] = EVT_ALARM_TRIGGERED ;
			Cl_alarms_retcode =  Cl_UpdateMacAlarmEventTable();
			ASSERT_ERROR(Cl_alarms_retcode);
			}
		}
		else
		{
			Cl_alarms_alarms[BICARB_IN_OPEN].cl_alarmstate = CL_ALARM_INACTIVE;
		}
	}
	if (Cl_alarms_alarms[BICARB_IN_CLOSED].cl_is_enabled)
	{
		
		if(((Cl_alarms_alarms[BICARB_IN_CLOSED].cl_triggertype == LOGIC_HIGH) && (cl_sys_statbuffer.bicarb_inlet_status )) || ((Cl_alarms_alarms[BICARB_IN_CLOSED].cl_triggertype == LOGIC_LOW) && (!cl_sys_statbuffer.bicarb_inlet_status )) )
		{
			if(Cl_alarms_alarms[BICARB_IN_CLOSED].cl_alarmstate != CL_ALARM_DETECTED)
			{
				Cl_alarms_alarms[BICARB_IN_CLOSED].cl_alarmstate = CL_ALARM_DETECTED;
				Cl_MacEventQueue4.Cl_MacEventData[Cl_MacEventQueue4.Cl_MacNewEventcount][0]=BICARB_IN_CLOSED;
				Cl_alarms_alarms[BICARB_IN_CLOSED].cl_detected_count++;
				cl_alarm_triggered = true;
				cl_lastalarmid = BICARB_IN_CLOSED;
				Cl_MacEventQueue4.Cl_MacEvent[Cl_MacEventQueue4.Cl_MacNewEventcount] = EVT_ALARM_TRIGGERED ;
				Cl_alarms_retcode =  Cl_UpdateMacAlarmEventTable();
				ASSERT_ERROR(Cl_alarms_retcode);
			}
		}
		else
		{
			Cl_alarms_alarms[BICARB_IN_CLOSED].cl_alarmstate = CL_ALARM_INACTIVE;
		}
	}
	
	if(Cl_alarms_alarms[ABD_EVENT].cl_is_enabled)
	{
		
		if(cl_sys_statbuffer.abdstatus)
		{

			Cl_alarms_alarms[ABD_EVENT].cl_alarmstate = CL_ALARM_DETECTED;
			Cl_MacEventQueue4.Cl_MacEventData[Cl_MacEventQueue4.Cl_MacNewEventcount][0]=ABD_EVENT; 
			Cl_alarms_alarms[ABD_EVENT].cl_detected_count++;
			cl_alarm_triggered = true;
			cl_lastalarmid = ABD_EVENT;
			Cl_MacEventQueue4.Cl_MacEvent[Cl_MacEventQueue4.Cl_MacNewEventcount] = EVT_ALARM_TRIGGERED ;
			Cl_alarms_retcode =  Cl_UpdateMacAlarmEventTable();
			ASSERT_ERROR(Cl_alarms_retcode);
		}
		else
		{
			Cl_alarms_alarms[ABD_EVENT].cl_alarmstate = CL_ALARM_INACTIVE;
		}
	
	}
	if(Cl_alarms_alarms[BD_EVENT].cl_is_enabled)
	{
		
		if(cl_sys_statbuffer.bdstatus)
		{

			Cl_alarms_alarms[BD_EVENT].cl_alarmstate = CL_ALARM_DETECTED;
			Cl_MacEventQueue4.Cl_MacEventData[Cl_MacEventQueue4.Cl_MacNewEventcount][0]=BD_EVENT; 
			Cl_alarms_alarms[BD_EVENT].cl_detected_count++;
			cl_alarm_triggered = true;
			cl_lastalarmid = BD_EVENT;
			Cl_MacEventQueue4.Cl_MacEvent[Cl_MacEventQueue4.Cl_MacNewEventcount] = EVT_ALARM_TRIGGERED ;
			Cl_alarms_retcode =  Cl_UpdateMacAlarmEventTable();
			ASSERT_ERROR(Cl_alarms_retcode);
		}
		else
		{
			Cl_alarms_alarms[BD_EVENT].cl_alarmstate = CL_ALARM_INACTIVE;
		}
	}
	if(Cl_alarms_alarms[BLD_EVENT].cl_is_enabled)
	{
				if(cl_sys_statbuffer.bldstatus)
				{
					Cl_alarms_alarms[BLD_EVENT].cl_alarmstate = CL_ALARM_DETECTED;
					Cl_MacEventQueue4.Cl_MacEventData[Cl_MacEventQueue4.Cl_MacNewEventcount][0]=BLD_EVENT; 
					Cl_alarms_alarms[BLD_EVENT].cl_detected_count++;
					cl_alarm_triggered = true;
					cl_lastalarmid = BLD_EVENT;
					Cl_MacEventQueue4.Cl_MacEvent[Cl_MacEventQueue4.Cl_MacNewEventcount] = EVT_ALARM_TRIGGERED ;
					Cl_alarms_retcode =  Cl_UpdateMacAlarmEventTable();
					ASSERT_ERROR(Cl_alarms_retcode);
				}
				else
				{

					Cl_alarms_alarms[BLD_EVENT].cl_alarmstate = CL_ALARM_INACTIVE;
					
				}
	}
		if(Cl_alarms_alarms[TMP_HIGH].cl_is_enabled)
		{
			
			if (Cl_alarms_alarms[TMP_HIGH].cl_alarmstate != CL_ALARM_DETECTED)
			{
				if(tmpVal > Cl_alarmThresholdTable.tmp_high_threshold -1000) //Cl_alarms_alarms[TMP_HIGH].cl_upper - 1000)
				{
					//Cl_Uint16Type temp_ps;
					//temp_ps=ps1;

					//Cl_SendDatatoconsole(CON_TX_COMMAND_PRINTTEXT,"PS1=",4);
					//Cl_SendDatatoconsole(CON_TX_COMMAND_PRINTDATA,&temp_ps,2);
					Cl_alarms_alarms[TMP_HIGH].cl_alarmstate = CL_ALARM_DETECTED;
					Cl_MacEventQueue4.Cl_MacEventData[Cl_MacEventQueue4.Cl_MacNewEventcount][0]=TMP_HIGH;
					Cl_alarms_alarms[TMP_HIGH].cl_detected_count++;
					cl_alarm_triggered = true;
					cl_lastalarmid = TMP_HIGH;
					Cl_MacEventQueue4.Cl_MacEvent[Cl_MacEventQueue4.Cl_MacNewEventcount] = EVT_ALARM_TRIGGERED ;
					Cl_alarms_retcode =  Cl_UpdateMacAlarmEventTable();
					ASSERT_ERROR(Cl_alarms_retcode);
				}
				
			}
			else if (Cl_alarms_alarms[TMP_HIGH].cl_alarmstate == CL_ALARM_DETECTED)
			{
				Cl_alarms_alarms[TMP_HIGH].cl_alarmstate = CL_ALARM_INACTIVE;
			}
		}
	
		if(Cl_alarms_alarms[TMP_LOW].cl_is_enabled)
		{
			
			if (Cl_alarms_alarms[TMP_LOW].cl_alarmstate != CL_ALARM_DETECTED)
			{
				if(tmpVal < Cl_alarmThresholdTable.tmp_low_threshold -1000)// Cl_alarms_alarms[TMP_LOW].cl_upper - 1000)
				{
					//Cl_Uint16Type temp_ps;
					//temp_ps=ps1;

					//Cl_SendDatatoconsole(CON_TX_COMMAND_PRINTTEXT,"PS1=",4);
					//Cl_SendDatatoconsole(CON_TX_COMMAND_PRINTDATA,&temp_ps,2);
					Cl_alarms_alarms[TMP_LOW].cl_alarmstate = CL_ALARM_DETECTED;
					Cl_MacEventQueue4.Cl_MacEventData[Cl_MacEventQueue4.Cl_MacNewEventcount][0]=TMP_LOW;
					Cl_alarms_alarms[TMP_LOW].cl_detected_count++;
					cl_alarm_triggered = true;
					cl_lastalarmid = TMP_LOW;
					Cl_MacEventQueue4.Cl_MacEvent[Cl_MacEventQueue4.Cl_MacNewEventcount] = EVT_ALARM_TRIGGERED ;
					Cl_alarms_retcode =  Cl_UpdateMacAlarmEventTable();
					ASSERT_ERROR(Cl_alarms_retcode);
				}
				
			}
			else if (Cl_alarms_alarms[TMP_LOW].cl_alarmstate == CL_ALARM_DETECTED)
			{
				Cl_alarms_alarms[TMP_LOW].cl_alarmstate = CL_ALARM_INACTIVE;
			}
		}
		
	
	if(Cl_alarms_alarms[APTSTATUS_HIGH].cl_is_enabled)
	{
		
				if(apt > (Cl_alarmThresholdTable.apt_high_threshold - 1000))  // manav03 added -1000
				{

					Cl_alarms_alarms[APTSTATUS_HIGH].cl_alarmstate = CL_ALARM_DETECTED;
					Cl_MacEventQueue4.Cl_MacEventData[Cl_MacEventQueue4.Cl_MacNewEventcount][0]=APTSTATUS_HIGH; 
					Cl_alarms_alarms[APTSTATUS_HIGH].cl_detected_count++;
					cl_alarm_triggered = true;
					cl_lastalarmid = APTSTATUS_HIGH;
					Cl_MacEventQueue4.Cl_MacEvent[Cl_MacEventQueue4.Cl_MacNewEventcount] = EVT_ALARM_TRIGGERED ;
					Cl_alarms_retcode =  Cl_UpdateMacAlarmEventTable();
					ASSERT_ERROR(Cl_alarms_retcode);
					
				}
				else if (Cl_alarms_alarms[APTSTATUS_HIGH].cl_alarmstate == CL_ALARM_DETECTED)
				{
					Cl_alarms_alarms[APTSTATUS_HIGH].cl_alarmstate = CL_ALARM_INACTIVE;
				}
	}
	if(Cl_alarms_alarms[APTSTATUS_LOW].cl_is_enabled)
	{
		//Cl_SendDatatoconsole(CON_TX_COMMAND_PRINTTEXT,"DETECT_LOW",11);
		if(apt < (Cl_alarmThresholdTable.apt_low_threshold -1000)) // manav03 added -1000
		{

			Cl_alarms_alarms[APTSTATUS_LOW].cl_alarmstate = CL_ALARM_DETECTED;
			Cl_MacEventQueue4.Cl_MacEventData[Cl_MacEventQueue4.Cl_MacNewEventcount][0]=APTSTATUS_LOW;
			Cl_alarms_alarms[APTSTATUS_LOW].cl_detected_count++;
			cl_alarm_triggered = true;
			cl_lastalarmid = APTSTATUS_LOW;
			Cl_MacEventQueue4.Cl_MacEvent[Cl_MacEventQueue4.Cl_MacNewEventcount] = EVT_ALARM_TRIGGERED ;
			Cl_alarms_retcode =  Cl_UpdateMacAlarmEventTable();
			ASSERT_ERROR(Cl_alarms_retcode);
			
		}
		else if (Cl_alarms_alarms[APTSTATUS_LOW].cl_alarmstate == CL_ALARM_DETECTED)
		{
			Cl_alarms_alarms[APTSTATUS_LOW].cl_alarmstate = CL_ALARM_INACTIVE;
		}
	}
	if(Cl_alarms_alarms[VPTSTATUS_HIGH].cl_is_enabled)
	{
		Cl_SendDatatoconsole(CON_TX_COMMAND_PRINTTEXT,"VPTHIGH",7);
		ASSERT_ERROR(Cl_alarms_retcode);
				if(vpt > (Cl_alarmThresholdTable.vpt_high_threshold - 1000 ))   // manav03 added -1000
				{

						Cl_alarms_alarms[VPTSTATUS_HIGH].cl_alarmstate = CL_ALARM_DETECTED;
						Cl_MacEventQueue4.Cl_MacEventData[Cl_MacEventQueue4.Cl_MacNewEventcount][0]=VPTSTATUS_HIGH; 
						Cl_alarms_alarms[VPTSTATUS_HIGH].cl_detected_count++;
						cl_alarm_triggered = true;
						cl_lastalarmid = VPTSTATUS_HIGH;
						Cl_MacEventQueue4.Cl_MacEvent[Cl_MacEventQueue4.Cl_MacNewEventcount] = EVT_ALARM_TRIGGERED ;
						Cl_alarms_retcode =  Cl_UpdateMacAlarmEventTable();
						ASSERT_ERROR(Cl_alarms_retcode);
					
				}
				else if (Cl_alarms_alarms[VPTSTATUS_HIGH].cl_alarmstate == CL_ALARM_DETECTED)
				{
					Cl_alarms_alarms[VPTSTATUS_HIGH].cl_alarmstate = CL_ALARM_INACTIVE;
				}
	}

	if(Cl_alarms_alarms[VPTSTATUS_LOW].cl_is_enabled)
	{
		
		if(vpt < (Cl_alarmThresholdTable.vpt_low_threshold - 1000)) // manav03 added -1000
		{
			Cl_alarms_alarms[VPTSTATUS_LOW].cl_alarmstate = CL_ALARM_DETECTED;
			Cl_MacEventQueue4.Cl_MacEventData[Cl_MacEventQueue4.Cl_MacNewEventcount][0]=VPTSTATUS_LOW;
			Cl_alarms_alarms[VPTSTATUS_LOW].cl_detected_count++;
			cl_alarm_triggered = true;
			cl_lastalarmid = VPTSTATUS_HIGH;
			Cl_MacEventQueue4.Cl_MacEvent[Cl_MacEventQueue4.Cl_MacNewEventcount] = EVT_ALARM_TRIGGERED ;
			Cl_alarms_retcode =  Cl_UpdateMacAlarmEventTable();
			ASSERT_ERROR(Cl_alarms_retcode);
			
		}
		else if (Cl_alarms_alarms[VPTSTATUS_LOW].cl_alarmstate == CL_ALARM_DETECTED)
		{
			Cl_alarms_alarms[VPTSTATUS_LOW].cl_alarmstate = CL_ALARM_INACTIVE;
		}
	}
	

	if(Cl_alarms_alarms[BLOODDOOR_STATUS_OPEN].cl_is_enabled)
	{
		
		
				if(((Cl_alarms_alarms[BLOODDOOR_STATUS_OPEN].cl_triggertype == LOGIC_HIGH) && (cl_sys_statbuffer.bloodpumpdoor )) || ((Cl_alarms_alarms[BLOODDOOR_STATUS_OPEN].cl_triggertype == LOGIC_LOW) && (!cl_sys_statbuffer.bloodpumpdoor )) )
				{
					if(Cl_alarms_alarms[BLOODDOOR_STATUS_OPEN].cl_alarmstate != CL_ALARM_DETECTED)
					{
						
						
					Cl_alarms_alarms[BLOODDOOR_STATUS_OPEN].cl_alarmstate = CL_ALARM_DETECTED;
					Cl_MacEventQueue4.Cl_MacEventData[Cl_MacEventQueue4.Cl_MacNewEventcount][0]=BLOODDOOR_STATUS_OPEN; 
					Cl_alarms_alarms[BLOODDOOR_STATUS_OPEN].cl_detected_count++;
					cl_alarm_triggered = true;
					cl_lastalarmid = BLOODDOOR_STATUS_OPEN;
					Cl_MacEventQueue4.Cl_MacEvent[Cl_MacEventQueue4.Cl_MacNewEventcount] = EVT_ALARM_TRIGGERED ;
					Cl_alarms_retcode =  Cl_UpdateMacAlarmEventTable();
					ASSERT_ERROR(Cl_alarms_retcode);
					
					}

				}
				else
				{

					Cl_alarms_alarms[BLOODDOOR_STATUS_OPEN].cl_alarmstate = CL_ALARM_INACTIVE;

					
		
				}
				
		
	
	}
	if(Cl_alarms_alarms[HOLDER1STATUS_OPEN].cl_is_enabled)
	{
				if(((Cl_alarms_alarms[HOLDER1STATUS_OPEN].cl_triggertype == LOGIC_HIGH) && (cl_sys_statbuffer.holder1status )) || ((Cl_alarms_alarms[HOLDER1STATUS_OPEN].cl_triggertype == LOGIC_LOW) && (!cl_sys_statbuffer.holder1status )) )
				{
					Cl_alarms_alarms[HOLDER1STATUS_OPEN].cl_alarmstate = CL_ALARM_DETECTED;
					Cl_MacEventQueue4.Cl_MacEventData[Cl_MacEventQueue4.Cl_MacNewEventcount][0]=HOLDER1STATUS_OPEN; 
					Cl_alarms_alarms[HOLDER1STATUS_OPEN].cl_detected_count++;
					cl_alarm_triggered = true;
					cl_lastalarmid = HOLDER1STATUS_OPEN;
					Cl_MacEventQueue4.Cl_MacEvent[Cl_MacEventQueue4.Cl_MacNewEventcount] = EVT_ALARM_TRIGGERED ;
					Cl_alarms_retcode =  Cl_UpdateMacAlarmEventTable();
					ASSERT_ERROR(Cl_alarms_retcode);
				}
				else
				{

					Cl_alarms_alarms[HOLDER1STATUS_OPEN].cl_alarmstate = CL_ALARM_INACTIVE;				
		
				}
					
	}
	
	if(Cl_alarms_alarms[HOLDER1STATUS_CLOSED].cl_is_enabled)
	{
		
		
		if(((Cl_alarms_alarms[HOLDER1STATUS_CLOSED].cl_triggertype == LOGIC_HIGH) && (cl_sys_statbuffer.holder1status )) || ((Cl_alarms_alarms[HOLDER1STATUS_CLOSED].cl_triggertype == LOGIC_LOW) && (!cl_sys_statbuffer.holder1status )) )
		{
			Cl_alarms_alarms[HOLDER1STATUS_CLOSED].cl_alarmstate = CL_ALARM_DETECTED;
			Cl_MacEventQueue4.Cl_MacEventData[Cl_MacEventQueue4.Cl_MacNewEventcount][0]=HOLDER1STATUS_OPEN;
			Cl_alarms_alarms[HOLDER1STATUS_CLOSED].cl_detected_count++;
			cl_alarm_triggered = true;
			cl_lastalarmid = HOLDER1STATUS_CLOSED;
			Cl_MacEventQueue4.Cl_MacEvent[Cl_MacEventQueue4.Cl_MacNewEventcount] = EVT_ALARM_TRIGGERED ;
			Cl_alarms_retcode =  Cl_UpdateMacAlarmEventTable();
			ASSERT_ERROR(Cl_alarms_retcode);
		}
		else
		{

			Cl_alarms_alarms[HOLDER1STATUS_CLOSED].cl_alarmstate = CL_ALARM_INACTIVE;
			
		}
		
	}
	
	
if(Cl_alarms_alarms[HOLDER2STATUS_OPEN].cl_is_enabled)
{
			
		if(((Cl_alarms_alarms[HOLDER2STATUS_OPEN].cl_triggertype == LOGIC_HIGH) && (cl_sys_statbuffer.holder2status )) || ((Cl_alarms_alarms[HOLDER2STATUS_OPEN].cl_triggertype == LOGIC_LOW) && (!cl_sys_statbuffer.holder2status )) )
				
			{
				Cl_alarms_alarms[HOLDER2STATUS_OPEN].cl_alarmstate = CL_ALARM_DETECTED;
				Cl_MacEventQueue4.Cl_MacEventData[Cl_MacEventQueue4.Cl_MacNewEventcount][0]=HOLDER2STATUS_OPEN;
				Cl_alarms_alarms[HOLDER2STATUS_OPEN].cl_detected_count++;
				cl_alarm_triggered = true;
				cl_lastalarmid = HOLDER2STATUS_OPEN;
				Cl_MacEventQueue4.Cl_MacEvent[Cl_MacEventQueue4.Cl_MacNewEventcount] = EVT_ALARM_TRIGGERED ;
				Cl_alarms_retcode =  Cl_UpdateMacAlarmEventTable();
				ASSERT_ERROR(Cl_alarms_retcode);
			}
			else
			{

				Cl_alarms_alarms[HOLDER2STATUS_OPEN].cl_alarmstate = CL_ALARM_INACTIVE;

			}
					
		

}

	
	if(Cl_alarms_alarms[HOLDER2STATUS_CLOSED].cl_is_enabled)
	{
		
		if(((Cl_alarms_alarms[HOLDER2STATUS_CLOSED].cl_triggertype == LOGIC_HIGH) && (cl_sys_statbuffer.holder2status )) || ((Cl_alarms_alarms[HOLDER2STATUS_CLOSED].cl_triggertype == LOGIC_LOW) && (!cl_sys_statbuffer.holder2status )) )
		
		{
			Cl_alarms_alarms[HOLDER2STATUS_CLOSED].cl_alarmstate = CL_ALARM_DETECTED;
			Cl_MacEventQueue4.Cl_MacEventData[Cl_MacEventQueue4.Cl_MacNewEventcount][0]=HOLDER2STATUS_CLOSED;
			Cl_alarms_alarms[HOLDER2STATUS_CLOSED].cl_detected_count++;
			cl_alarm_triggered = true;
			cl_lastalarmid = HOLDER2STATUS_CLOSED;
			Cl_MacEventQueue4.Cl_MacEvent[Cl_MacEventQueue4.Cl_MacNewEventcount] = EVT_ALARM_TRIGGERED ;
			Cl_alarms_retcode =  Cl_UpdateMacAlarmEventTable();
			ASSERT_ERROR(Cl_alarms_retcode);
		}
		else
		{

			Cl_alarms_alarms[HOLDER2STATUS_CLOSED].cl_alarmstate = CL_ALARM_INACTIVE;

		}
		
		

	}
	

	/*
	if(Cl_alarms_alarms[DGPstatus].cl_is_enabled)
	{
				if(cl_sys_statbuffer.DGPstatus == 0)
				{
				
					Cl_alarms_alarms[DGPstatus].cl_alarmstate = CL_ALARM_DETECTED;
					Cl_MacEventQueue1.Cl_MacEventData[Cl_MacEventQueue1.Cl_MacNewEventcount][0]=DGPstatus; 
					Cl_alarms_alarms[DGPstatus].cl_detected_count++;
					cl_alarm_triggered = true;	
					cl_lastalarmid = DGPstatus;
					Cl_MacEventQueue1.Cl_MacEvent[Cl_MacEventQueue1.Cl_MacNewEventcount] = EVT_ALERT_TRIGGERED ;
					Cl_UpdateMacAlarmEventTable();
					
				}
				else
				
				{
					Cl_alarms_alarms[DGPstatus].cl_alarmstate = CL_ALARM_INACTIVE;
					Cl_alarms_alarms[DGPstatus].cl_detected_count = 0;

					
				}
	}
	if(Cl_alarms_alarms[UF_ROTATION_MARKER].cl_is_enabled)
	{
				if(cl_sys_statbuffer.UFPstatus == 1)
				{
					Cl_alarms_alarms[UF_ROTATION_MARKER].cl_alarmstate = CL_ALARM_DETECTED;	
					Cl_MacEventQueue1.Cl_MacEventData[Cl_MacEventQueue1.Cl_MacNewEventcount][0]=UF_ROTATION_MARKER; 
					Cl_alarms_alarms[UF_ROTATION_MARKER].cl_detected_count++;
					cl_alarm_triggered = true;
					cl_lastalarmid = UF_ROTATION_MARKER;
					Cl_MacEventQueue1.Cl_MacEvent[Cl_MacEventQueue1.Cl_MacNewEventcount] = EVT_ALERT_TRIGGERED ;
					Cl_UpdateMacAlarmEventTable();
					
				}
				else
					
					{
						Cl_alarms_alarms[UF_ROTATION_MARKER].cl_alarmstate = CL_ALARM_INACTIVE;
						Cl_alarms_alarms[UF_ROTATION_MARKER].cl_detected_count = 0;

									
					}
	}
*/
/*
	if(Cl_alarms_alarms[HP_START].cl_is_enabled)
	{

	}
	if(Cl_alarms_alarms[HP_END].cl_is_enabled)
	{

	}


*/
		//manav03 commented and moved to flowswitch controller	
// 	//if(Cl_alarms_alarms[FLOWSTATUS_FLOWON].cl_is_enabled)
// 	{
// 				
// 		if(cl_sys_statbuffer.Flowstatus) 
// 		{	
// 				Flowswitch_msg.flowswitch_event = FLOW_SWITCH_EVENT_LEVEL_HIGH;						
// 		}
// 	}
// 
// 
// 	//if(Cl_alarms_alarms[FLOWSTATUS_FLOWOFF].cl_is_enabled)
// 	{
// 				
// 		if(!cl_sys_statbuffer.Flowstatus)
// 		{
// 				Flowswitch_msg.flowswitch_event = FLOW_SWITCH_EVENT_LEVEL_LOW;
// 		}
// 	}


			
	if(Cl_alarms_alarms[LEVELSWITCH_OFF_TO_ON].cl_is_enabled)
	{
	
		if(((cl_sys_statbuffer.levelswitch1) && (Cl_alarms_alarms[LEVELSWITCH_OFF_TO_ON].cl_triggertype == LOGIC_HIGH)) || ((!cl_sys_statbuffer.levelswitch1) && (Cl_alarms_alarms[LEVELSWITCH_OFF_TO_ON].cl_triggertype == LOGIC_LOW)))// when water level goes above it is a logic zero
		{
				
					//if(levelswitchprevstatus != 0)
					if (Cl_alarms_alarms[LEVELSWITCH_OFF_TO_ON].cl_alarmstate != CL_ALARM_DETECTED)
					{
									

								Cl_alarms_alarms[LEVELSWITCH_ON_TO_OFF].cl_alarmstate = CL_ALARM_ACTIVE;	
								Cl_alarms_alarms[LEVELSWITCH_OFF_TO_ON].cl_alarmstate = CL_ALARM_DETECTED;
								Cl_MacEventQueue4.Cl_MacEventData[Cl_MacEventQueue4.Cl_MacNewEventcount][0]=LEVELSWITCH_OFF_TO_ON; 
								//Cl_alarms_alarms[LEVELSWITCH_OFF_TO_ON].cl_detected_count++;
								cl_alarm_triggered = true;
								cl_lastalarmid = LEVELSWITCH_OFF_TO_ON;
								Cl_MacEventQueue4.Cl_MacEvent[Cl_MacEventQueue4.Cl_MacNewEventcount] = EVT_ALARM_TRIGGERED ;
								//Cl_UpdateMacAlarmEventTable();
								Levelswitch_msg.levelswitch_event = LEVEL_SWITCH_EVENT_ON_TO_OFF;
					
					}
					else
					{
						//		Cl_alarms_alarms[LEVELSWITCH_OFF_TO_ON].cl_alarmstate = CL_ALARM_INACTIVE;
								Cl_alarms_alarms[LEVELSWITCH_OFF_TO_ON].cl_detected_count = 0;
								cl_alarm_triggered = false;
					
					}
				
					levelswitchprevstatus = 0;
		}
	}


	if(Cl_alarms_alarms[LEVELSWITCH_ON_TO_OFF].cl_is_enabled)
	{
		if(((cl_sys_statbuffer.levelswitch1) && (Cl_alarms_alarms[LEVELSWITCH_ON_TO_OFF].cl_triggertype == LOGIC_HIGH)) || ((!cl_sys_statbuffer.levelswitch1) && (Cl_alarms_alarms[LEVELSWITCH_ON_TO_OFF].cl_triggertype == LOGIC_LOW)))	
		//if((!cl_sys_statbuffer.levelswitch1) && (Cl_alarms_alarms[LEVELSWITCH_ON_TO_OFF].cl_triggertype == LOGIC_LOW))
		{				
				//	if(levelswitchprevstatus != 1)
				if (Cl_alarms_alarms[LEVELSWITCH_ON_TO_OFF].cl_alarmstate != CL_ALARM_DETECTED)
					{
					
				
						if(Cl_alarms_alarms[LEVELSWITCH_ON_TO_OFF].cl_is_enabled)
						{
								Cl_alarms_alarms[LEVELSWITCH_OFF_TO_ON].cl_alarmstate = CL_ALARM_ACTIVE;	
									Cl_alarms_alarms[LEVELSWITCH_ON_TO_OFF].cl_alarmstate = CL_ALARM_DETECTED;	
							//	Cl_MacEventQueue1.Cl_MacEventData[Cl_MacEventQueue1.Cl_MacNewEventcount][0]=LEVELSWITCH_ON_TO_OFF; 
							//	Cl_alarms_alarms[LEVELSWITCH_ON_TO_OFF].cl_detected_count++;
								cl_alarm_triggered = true;
								cl_lastalarmid = LEVELSWITCH_ON_TO_OFF;
								Cl_MacEventQueue4.Cl_MacEvent[Cl_MacEventQueue4.Cl_MacNewEventcount] = EVT_ALARM_TRIGGERED ;
								//Cl_UpdateMacAlarmEventTable();
								Levelswitch_msg.levelswitch_event = LEVEL_SWITCH_EVENT_OFF_TO_ON;
						}
					}
					else
					{
					//	Cl_alarms_alarms[LEVELSWITCH_ON_TO_OFF].cl_alarmstate = CL_ALARM_INACTIVE;
						Cl_alarms_alarms[LEVELSWITCH_ON_TO_OFF].cl_detected_count = 0;
						cl_alarm_triggered = false;
					
					}
				
					levelswitchprevstatus = 1;
		
		}
	}

	
	if(Cl_alarms_alarms[PS1_HIGH_THRESHOLD].cl_is_enabled)
	{
		
		if (Cl_alarms_alarms[PS1_HIGH_THRESHOLD].cl_alarmstate != CL_ALARM_DETECTED)
		{
				if(ps1 > Cl_alarms_alarms[PS1_HIGH_THRESHOLD].cl_upper - 1000) 
				{
					//Cl_Uint16Type temp_ps;
					//temp_ps=ps1;

					//Cl_SendDatatoconsole(CON_TX_COMMAND_PRINTTEXT,"PS1=",4);
					//Cl_SendDatatoconsole(CON_TX_COMMAND_PRINTDATA,&temp_ps,2);
					Cl_alarms_alarms[PS1_HIGH_THRESHOLD].cl_alarmstate = CL_ALARM_DETECTED;	
					Cl_MacEventQueue4.Cl_MacEventData[Cl_MacEventQueue4.Cl_MacNewEventcount][0]=PS1_HIGH_THRESHOLD;
					Cl_alarms_alarms[PS1_HIGH_THRESHOLD].cl_detected_count++;
					cl_alarm_triggered = true;
					cl_lastalarmid = PS1_HIGH_THRESHOLD;
					Cl_MacEventQueue4.Cl_MacEvent[Cl_MacEventQueue4.Cl_MacNewEventcount] = EVT_ALARM_TRIGGERED ;
					Cl_alarms_retcode =  Cl_UpdateMacAlarmEventTable();
					ASSERT_ERROR(Cl_alarms_retcode);
				}
				
		}
		else if (Cl_alarms_alarms[PS1_HIGH_THRESHOLD].cl_alarmstate == CL_ALARM_DETECTED) 
		{
			Cl_alarms_alarms[PS1_HIGH_THRESHOLD].cl_alarmstate = CL_ALARM_INACTIVE;
		}
	}

if(Cl_alarms_alarms[PS1_LOW_THRESHOLD].cl_is_enabled)
{
	 
	if (Cl_alarms_alarms[PS1_LOW_THRESHOLD].cl_alarmstate != CL_ALARM_DETECTED)
	{
		if(ps1 < Cl_alarms_alarms[PS1_LOW_THRESHOLD].cl_lower - 1000) 
		{
			//Cl_Uint16Type temp_ps;
			//temp_ps=ps1;

			//Cl_SendDatatoconsole(CON_TX_COMMAND_PRINTTEXT,"PS1=",4);
			//Cl_SendDatatoconsole(CON_TX_COMMAND_PRINTDATA,&temp_ps,2);
			Cl_alarms_alarms[PS1_LOW_THRESHOLD].cl_alarmstate = CL_ALARM_DETECTED;
			Cl_MacEventQueue4.Cl_MacEventData[Cl_MacEventQueue4.Cl_MacNewEventcount][0]=PS1_LOW_THRESHOLD;
			Cl_alarms_alarms[PS1_LOW_THRESHOLD].cl_detected_count++;
			cl_alarm_triggered = true;
			cl_lastalarmid = PS1_LOW_THRESHOLD;
			Cl_MacEventQueue4.Cl_MacEvent[Cl_MacEventQueue4.Cl_MacNewEventcount] = EVT_ALARM_TRIGGERED ;
			Cl_alarms_retcode =  Cl_UpdateMacAlarmEventTable();
			ASSERT_ERROR(Cl_alarms_retcode);
		}
	}
	else if (Cl_alarms_alarms[PS1_LOW_THRESHOLD].cl_alarmstate == CL_ALARM_DETECTED)
	{
		Cl_alarms_alarms[PS1_LOW_THRESHOLD].cl_alarmstate = CL_ALARM_INACTIVE;
	}
}

	if(Cl_alarms_alarms[PS2_HIGH_THRESHOLD].cl_is_enabled)
	{
		
				if (Cl_alarms_alarms[PS2_HIGH_THRESHOLD].cl_alarmstate != CL_ALARM_DETECTED)
				{
				if(ps2 > Cl_alarms_alarms[PS2_HIGH_THRESHOLD].cl_upper - 1000) 
				{
						//Cl_Uint16Type temp_ps;
						//temp_ps=ps2;

							// Cl_SendDatatoconsole(CON_TX_COMMAND_PRINTTEXT,"PS2=",4);
							// Cl_SendDatatoconsole(CON_TX_COMMAND_PRINTDATA,&temp_ps,2);
												
					Cl_alarms_alarms[PS2_HIGH_THRESHOLD].cl_alarmstate = CL_ALARM_DETECTED;	
					Cl_MacEventQueue4.Cl_MacEventData[Cl_MacEventQueue4.Cl_MacNewEventcount][0]=PS2_HIGH_THRESHOLD;
					Cl_alarms_alarms[PS2_HIGH_THRESHOLD].cl_detected_count++;
					cl_alarm_triggered = true;
					cl_lastalarmid = PS2_HIGH_THRESHOLD;
					Cl_MacEventQueue4.Cl_MacEvent[Cl_MacEventQueue4.Cl_MacNewEventcount] = EVT_ALARM_TRIGGERED ;
					Cl_alarms_retcode =  Cl_UpdateMacAlarmEventTable();
					ASSERT_ERROR(Cl_alarms_retcode);
					
				}
				}
				else if (Cl_alarms_alarms[PS2_HIGH_THRESHOLD].cl_alarmstate == CL_ALARM_DETECTED)
				{
					Cl_alarms_alarms[PS2_HIGH_THRESHOLD].cl_alarmstate = CL_ALARM_INACTIVE;
				}
	}
		if(Cl_alarms_alarms[PS2_LOW_THRESHOLD].cl_is_enabled)
		{
			//Cl_SendDatatoconsole(CON_TX_COMMAND_PRINTTEXT,"DETECT",6);
			if (Cl_alarms_alarms[PS2_LOW_THRESHOLD].cl_alarmstate != CL_ALARM_DETECTED)
			{
				if(ps2 < Cl_alarms_alarms[PS2_LOW_THRESHOLD].cl_lower - 1000)  
				{
					//Cl_Uint16Type temp_ps;
					//temp_ps=ps2;

					//Cl_SendDatatoconsole(CON_TX_COMMAND_PRINTTEXT,"PS2=",4);
					//Cl_SendDatatoconsole(CON_TX_COMMAND_PRINTDATA,&temp_ps,2);
					Cl_alarms_alarms[PS2_LOW_THRESHOLD].cl_alarmstate = CL_ALARM_DETECTED;
					Cl_MacEventQueue4.Cl_MacEventData[Cl_MacEventQueue4.Cl_MacNewEventcount][0]=PS2_LOW_THRESHOLD;
					Cl_alarms_alarms[PS2_LOW_THRESHOLD].cl_detected_count++;
					cl_alarm_triggered = true;
					cl_lastalarmid = PS2_LOW_THRESHOLD;
					Cl_MacEventQueue4.Cl_MacEvent[Cl_MacEventQueue4.Cl_MacNewEventcount] = EVT_ALARM_TRIGGERED ;
					Cl_alarms_retcode =  Cl_UpdateMacAlarmEventTable();
					ASSERT_ERROR(Cl_alarms_retcode);
					
				}
			}
			else if (Cl_alarms_alarms[PS2_LOW_THRESHOLD].cl_alarmstate == CL_ALARM_DETECTED)
			{
				Cl_alarms_alarms[PS2_LOW_THRESHOLD].cl_alarmstate = CL_ALARM_INACTIVE;
			}
		}
		

	if(Cl_alarms_alarms[PS3_HIGH_THRESHOLD].cl_is_enabled)
		{
			// Cl_SendDatatoconsole(CON_TX_COMMAND_PRINTTEXT,"DETECT_HIGH",11);
			//	if((cl_sys_statbuffer.ps3status > 1474) && (Cl_alarms_alarms[PS3STATUS_HIGH].cl_alarmstate != CL_ALARM_DETECTED)) // eqvt to 0.8 bar
				if((ps3 > Cl_alarms_alarms[PS3_HIGH_THRESHOLD].cl_upper - 1000 ) && (Cl_alarms_alarms[PS3_HIGH_THRESHOLD].cl_alarmstate != CL_ALARM_DETECTED)) // eqvt to 0.8 bar 
				{
					//Cl_Uint16Type temp_ps;
					//temp_ps=ps3;

					//Cl_SendDatatoconsole(CON_TX_COMMAND_PRINTTEXT,"PS3=",4);
					//Cl_SendDatatoconsole(CON_TX_COMMAND_PRINTDATA,&temp_ps,2);
				Cl_alarms_alarms[PS3_HIGH_THRESHOLD].cl_alarmstate = CL_ALARM_DETECTED;
				Cl_MacEventQueue4.Cl_MacEventData[Cl_MacEventQueue4.Cl_MacNewEventcount][0]=PS3_HIGH_THRESHOLD;
				Cl_alarms_alarms[PS3_HIGH_THRESHOLD].cl_detected_count++;
				cl_alarm_triggered = true;
				cl_lastalarmid = PS3_HIGH_THRESHOLD;
				//Cl_MacEventQueue1.Cl_MacEvent[Cl_MacEventQueue1.Cl_MacNewEventcount] = EVT_ALERT_TRIGGERED ;
				Cl_MacEventQueue4.Cl_MacEvent[Cl_MacEventQueue4.Cl_MacNewEventcount] = EVT_ALARM_TRIGGERED ;
						Cl_alarms_retcode =  Cl_UpdateMacAlarmEventTable();
						ASSERT_ERROR(Cl_alarms_retcode);
				}
				
				else if (Cl_alarms_alarms[PS3_HIGH_THRESHOLD].cl_alarmstate == CL_ALARM_DETECTED)
				{
					Cl_alarms_alarms[PS3_HIGH_THRESHOLD].cl_alarmstate = CL_ALARM_INACTIVE;
				}
		}
	
	if(Cl_alarms_alarms[PS3_LOW_THRESHOLD].cl_is_enabled)
	{
		
		//	if((cl_sys_statbuffer.ps3status > 1474) && (Cl_alarms_alarms[PS3STATUS_HIGH].cl_alarmstate != CL_ALARM_DETECTED)) // eqvt to 0.8 bar
		
		float temp,temp_ps;
		//temp = 
		
		if((ps3 < Cl_alarmThresholdTable.ps3_low_threshold - 1000) && (Cl_alarms_alarms[PS3_LOW_THRESHOLD].cl_alarmstate != CL_ALARM_DETECTED)) // eqvt to 0.8 bar
		{
			//Cl_SendDatatoconsole(CON_TX_COMMAND_PRINTTEXT,"PS3=",4);
			// manav02 commented
// 			if(ps3 < 0)
// 			{
// 				ps3 = ps3+1000;
// 				//Cl_SendDatatoconsole(CON_TX_COMMAND_PRINTNDATA,&ps3,2);
// 			}
// 			else
// 			{
// 				//Cl_Uint16Type temp =0,temp1=0;
// 				//temp = ps3;
// 				//temp1= Cl_alarmThresholdTable.ps3_low_threshold;
// 				//Cl_SendDatatoconsole(CON_TX_COMMAND_PRINTDATA,&temp,2);
// 				//Cl_SendDatatoconsole(CON_TX_COMMAND_PRINTDATA,&temp1,2);
// 			}
// 			
			Cl_alarms_alarms[PS3_LOW_THRESHOLD].cl_alarmstate = CL_ALARM_DETECTED;
			Cl_MacEventQueue4.Cl_MacEventData[Cl_MacEventQueue4.Cl_MacNewEventcount][0]=PS3_LOW_THRESHOLD;
			Cl_alarms_alarms[PS3_LOW_THRESHOLD].cl_detected_count++;
			cl_alarm_triggered = true;
			cl_lastalarmid = PS3_LOW_THRESHOLD;
			//Cl_MacEventQueue1.Cl_MacEvent[Cl_MacEventQueue1.Cl_MacNewEventcount] = EVT_ALERT_TRIGGERED ;
			Cl_MacEventQueue4.Cl_MacEvent[Cl_MacEventQueue4.Cl_MacNewEventcount] = EVT_ALARM_TRIGGERED ;
			Cl_alarms_retcode =  Cl_UpdateMacAlarmEventTable();
			ASSERT_ERROR(Cl_alarms_retcode);
		}
		
		else if (Cl_alarms_alarms[PS3_LOW_THRESHOLD].cl_alarmstate == CL_ALARM_DETECTED)
		{
			Cl_alarms_alarms[PS3_LOW_THRESHOLD].cl_alarmstate = CL_ALARM_INACTIVE;
		}
	}
	
	
	if(Cl_alarms_alarms[TEMP1_HIGH_THRESHOLD].cl_is_enabled)
	{

		//	if(temp1 > Cl_alarmThresholdTable.temp1_high_threshold)

			if(temp1 >  Cl_alarms_alarms[TEMP1_HIGH_THRESHOLD].cl_upper)
			{
				//Cl_Uint16Type temp_ps;
				//temp_ps=temp1;

				//Cl_SendDatatoconsole(CON_TX_COMMAND_PRINTTEXT,"T1=",3);
				//Cl_SendDatatoconsole(CON_TX_COMMAND_PRINTDATA,&temp_ps,2);
				Cl_alarms_alarms[TEMP1_HIGH_THRESHOLD].cl_alarmstate = CL_ALARM_DETECTED;
				Cl_MacEventQueue4.Cl_MacEventData[Cl_MacEventQueue4.Cl_MacNewEventcount][0]=TEMP1_HIGH_THRESHOLD;
				Cl_alarms_alarms[TEMP1_HIGH_THRESHOLD].cl_detected_count++;
				cl_alarm_triggered = true;
				cl_lastalarmid = TEMP1_HIGH_THRESHOLD;
				Cl_MacEventQueue4.Cl_MacEvent[Cl_MacEventQueue4.Cl_MacNewEventcount] = EVT_ALARM_TRIGGERED ;
				Cl_alarms_retcode =  Cl_UpdateMacAlarmEventTable();
				ASSERT_ERROR(Cl_alarms_retcode);	
			}
			else
			{
				if(Cl_alarms_alarms[TEMP1_HIGH_THRESHOLD].cl_alarmstate == CL_ALARM_DETECTED)
				{
					Cl_alarms_alarms[TEMP1_HIGH_THRESHOLD].cl_alarmstate = CL_ALARM_INACTIVE;
					Cl_alarms_alarms[TEMP1_HIGH_THRESHOLD].cl_detected_count = 0;
					cl_alarm_triggered = false;
				}
			}
			
	}
	
	
	if(Cl_alarms_alarms[TEMP1_LOW_THRESHOLD].cl_is_enabled)
	{
			//	Cl_Int16Type temp1,temp2;
			//	temp1 = (0.805 * cl_sys_statbuffer.Temp1status) - 1004 ;
			//	temp2 = 3000 + (temp1 * 1000)/382;
			//Cl_Uint16Type temp2;
			//temp2=Cl_alarmThresholdTable.temp1_low_threshold;
			//Cl_SendDatatoconsole(CON_TX_COMMAND_PRINTTEXT,"DETECT_LOW",11);
			//Cl_SendDatatoconsole(CON_TX_COMMAND_PRINTDATA,&temp2,2);
			//Cl_SendDatatoconsole(CON_TX_COMMAND_PRINTDATA,&temp1,2);
			//if(cl_sys_statbuffer.Temp1status > 4096)
		//	if(temp1 < Cl_alarmThresholdTable.temp1_low_threshold)
			if(temp1 < Cl_alarms_alarms[TEMP1_LOW_THRESHOLD].cl_lower)
			{
				
				//Cl_Uint16Type temp_ps;
				//temp_ps=temp1;

				//Cl_SendDatatoconsole(CON_TX_COMMAND_PRINTTEXT,"T1=",3);
				//Cl_SendDatatoconsole(CON_TX_COMMAND_PRINTDATA,&temp_ps,2);
				if(Cl_alarms_alarms[TEMP1_LOW_THRESHOLD].cl_alarmstate != CL_ALARM_DETECTED)
				{
				Cl_alarms_alarms[TEMP1_LOW_THRESHOLD].cl_alarmstate = CL_ALARM_DETECTED;
				Cl_MacEventQueue4.Cl_MacEventData[Cl_MacEventQueue4.Cl_MacNewEventcount][0]=TEMP1_LOW_THRESHOLD;
				Cl_alarms_alarms[TEMP1_LOW_THRESHOLD].cl_detected_count++;
				cl_alarm_triggered = true;
				cl_lastalarmid = TEMP1_LOW_THRESHOLD;
				Cl_MacEventQueue4.Cl_MacEvent[Cl_MacEventQueue4.Cl_MacNewEventcount] = EVT_ALARM_TRIGGERED ;
				Cl_alarms_retcode =  Cl_UpdateMacAlarmEventTable();
				ASSERT_ERROR(Cl_alarms_retcode);
				}
						
			}
			else
			{	
				if(Cl_alarms_alarms[TEMP1_LOW_THRESHOLD].cl_alarmstate == CL_ALARM_DETECTED)
				{												
					Cl_alarms_alarms[TEMP1_LOW_THRESHOLD].cl_alarmstate = CL_ALARM_INACTIVE;
					Cl_alarms_alarms[TEMP1_LOW_THRESHOLD].cl_detected_count = 0;
					cl_alarm_triggered = false;	
				}						
			}
	}
	
	
	if(Cl_alarms_alarms[TEMP3_HIGH_THRESHOLD].cl_is_enabled)
		{
		//	Cl_SendDatatoconsole(CON_TX_COMMAND_PRINTTEXT,"DETECT_HIGH",11);
			//	Cl_Int16Type temp1,temp2;
			//	temp1 = (0.805 * cl_sys_statbuffer.Temp3status) - 1004 ;
			//	temp2 = 3000 + (temp1 * 1000)/382;
			if(temp3 > Cl_alarmThresholdTable.temp3_high_threshold)
		//	if(temp2 > Cl_alarms_alarms[SENSOR_TEMP3STATUS].cl_upper)
		//	if(cl_sys_statbuffer.Temp3status > 405)
			{
			//	Cl_Uint16Type temp_ps;
				//temp_ps=temp3;

				///Cl_SendDatatoconsole(CON_TX_COMMAND_PRINTTEXT,"T3=",3);
				//Cl_SendDatatoconsole(CON_TX_COMMAND_PRINTDATA,&temp_ps,2);
				if(Cl_alarms_alarms[TEMP3_HIGH_THRESHOLD].cl_alarmstate != CL_ALARM_DETECTED)
				{
					Cl_alarms_alarms[TEMP3_HIGH_THRESHOLD].cl_alarmstate = CL_ALARM_DETECTED;
					Cl_MacEventQueue4.Cl_MacEventData[Cl_MacEventQueue4.Cl_MacNewEventcount][0]=TEMP3_HIGH_THRESHOLD;
					Cl_alarms_alarms[TEMP3_HIGH_THRESHOLD].cl_detected_count++;
					cl_alarm_triggered = true;
					
					cl_lastalarmid = TEMP3_HIGH_THRESHOLD;
					Cl_MacEventQueue4.Cl_MacEvent[Cl_MacEventQueue4.Cl_MacNewEventcount] = EVT_ALARM_TRIGGERED ;
					Cl_alarms_retcode =  Cl_UpdateMacAlarmEventTable();
					ASSERT_ERROR(Cl_alarms_retcode);
				}						
			}
	
				else 
			//	else if (cl_sys_statbuffer.Temp3status > 400)
				{	
					if(Cl_alarms_alarms[TEMP3_HIGH_THRESHOLD].cl_alarmstate == CL_ALARM_DETECTED)
					{												
						Cl_alarms_alarms[TEMP3_HIGH_THRESHOLD].cl_alarmstate = CL_ALARM_INACTIVE;
						Cl_alarms_alarms[TEMP3_HIGH_THRESHOLD].cl_detected_count = 0;
						cl_alarm_triggered = false;	
					}						
				}
				}
	if(Cl_alarms_alarms[TEMP3_LOW_THRESHOLD].cl_is_enabled)
	{
		//Cl_SendDatatoconsole(CON_TX_COMMAND_PRINTTEXT,"DETECT_LOW",11);
			 if (temp3 < Cl_alarmThresholdTable.temp3_low_threshold)
		//	else if (cl_sys_statbuffer.Temp3status < 395)
				{
				//Cl_Uint16Type temp_ps;
				//temp_ps=temp3;

				//Cl_SendDatatoconsole(CON_TX_COMMAND_PRINTTEXT,"T3=",3);
				//Cl_SendDatatoconsole(CON_TX_COMMAND_PRINTDATA,&temp_ps,2);
							if(Cl_alarms_alarms[TEMP3_LOW_THRESHOLD].cl_alarmstate != CL_ALARM_DETECTED)
							{
								Cl_alarms_alarms[TEMP3_LOW_THRESHOLD].cl_alarmstate = CL_ALARM_DETECTED;
								Cl_MacEventQueue4.Cl_MacEventData[Cl_MacEventQueue4.Cl_MacNewEventcount][0]=TEMP3_LOW_THRESHOLD;
								Cl_alarms_alarms[TEMP3_LOW_THRESHOLD].cl_detected_count++;
								cl_alarm_triggered = true;
						
								cl_lastalarmid = TEMP3_LOW_THRESHOLD;
								Cl_MacEventQueue4.Cl_MacEvent[Cl_MacEventQueue4.Cl_MacNewEventcount] = EVT_ALARM_TRIGGERED ;
								Cl_alarms_retcode =  Cl_UpdateMacAlarmEventTable();
								ASSERT_ERROR(Cl_alarms_retcode);
						
							}
				}
				else
				{
									
							if(Cl_alarms_alarms[TEMP3_LOW_THRESHOLD].cl_alarmstate == CL_ALARM_DETECTED)
							{
								Cl_alarms_alarms[TEMP3_LOW_THRESHOLD].cl_alarmstate = CL_ALARM_INACTIVE;
								Cl_alarms_alarms[TEMP3_LOW_THRESHOLD].cl_detected_count = 0;
								cl_alarm_triggered = false;
							}
									
					
				}

		}


if(Cl_alarms_alarms[TEMP2_HIGH_THRESHOLD].cl_is_enabled)
{
	//Cl_SendDatatoconsole(CON_TX_COMMAND_PRINTTEXT,"DETECT_HIGH",11);
	//		Cl_Int16Type temp1,temp2;
	//		temp1 = (0.805 * cl_sys_statbuffer.Temp2status) - 1004 ;
	//		temp2 = 3000 + (temp1 * 1000)/382;
			if(temp2 > Cl_alarms_alarms[TEMP2_HIGH_THRESHOLD].cl_upper)

//	if(cl_sys_statbuffer.Temp2status > 840)
	{
	//Cl_Uint16Type temp_ps;
	//temp_ps=temp2;

	//Cl_SendDatatoconsole(CON_TX_COMMAND_PRINTTEXT,"T2=",3);
	//Cl_SendDatatoconsole(CON_TX_COMMAND_PRINTDATA,&temp_ps,2);
		if(Cl_alarms_alarms[TEMP2_HIGH_THRESHOLD].cl_alarmstate != CL_ALARM_DETECTED)
		{
			Cl_alarms_alarms[TEMP2_HIGH_THRESHOLD].cl_alarmstate = CL_ALARM_DETECTED;
			Cl_MacEventQueue4.Cl_MacEventData[Cl_MacEventQueue4.Cl_MacNewEventcount][0]=TEMP2_HIGH_THRESHOLD;
			Cl_alarms_alarms[TEMP2_HIGH_THRESHOLD].cl_detected_count++;
			cl_alarm_triggered = true;
			
			cl_lastalarmid = TEMP2_HIGH_THRESHOLD;
			Cl_MacEventQueue4.Cl_MacEvent[Cl_MacEventQueue4.Cl_MacNewEventcount] = EVT_ALARM_TRIGGERED ;
			Cl_alarms_retcode =  Cl_UpdateMacAlarmEventTable();
			ASSERT_ERROR(Cl_alarms_retcode);
			
		}

	}
	else 
	//	else if (cl_sys_statbuffer.Temp3status > 400)
	{

			if(Cl_alarms_alarms[TEMP2_HIGH_THRESHOLD].cl_alarmstate == CL_ALARM_DETECTED)
			{
				Cl_alarms_alarms[TEMP2_HIGH_THRESHOLD].cl_alarmstate = CL_ALARM_INACTIVE;
				Cl_alarms_alarms[TEMP2_HIGH_THRESHOLD].cl_detected_count = 0;
				cl_alarm_triggered = false;
			}

		
		
	}
}
if(Cl_alarms_alarms[TEMP2_LOW_THRESHOLD].cl_is_enabled)
{
	//Cl_SendDatatoconsole(CON_TX_COMMAND_PRINTTEXT,"DETECT_LOW",11);
	 if (temp2 < Cl_alarms_alarms[TEMP2_LOW_THRESHOLD].cl_lower)//)Cl_alarmThresholdTable.temp2_low_threshold)
		{
			//Cl_Uint16Type temp_ps;
			//temp_ps=temp2;

			//Cl_SendDatatoconsole(CON_TX_COMMAND_PRINTTEXT,"T2=",3);
			//Cl_SendDatatoconsole(CON_TX_COMMAND_PRINTDATA,&temp_ps,2);
		
		if(Cl_alarms_alarms[TEMP2_LOW_THRESHOLD].cl_alarmstate != CL_ALARM_DETECTED)
		{
			Cl_alarms_alarms[TEMP2_LOW_THRESHOLD].cl_alarmstate = CL_ALARM_DETECTED;
			Cl_MacEventQueue4.Cl_MacEventData[Cl_MacEventQueue4.Cl_MacNewEventcount][0]=TEMP2_LOW_THRESHOLD;
			Cl_alarms_alarms[TEMP2_LOW_THRESHOLD].cl_detected_count++;
			cl_alarm_triggered = true;
			
			cl_lastalarmid = TEMP2_LOW_THRESHOLD;
			Cl_MacEventQueue4.Cl_MacEvent[Cl_MacEventQueue4.Cl_MacNewEventcount] = EVT_ALARM_TRIGGERED ;
			Cl_alarms_retcode =  Cl_UpdateMacAlarmEventTable();
			ASSERT_ERROR(Cl_alarms_retcode);
			
		}
		}
	else
	//	else if (cl_sys_statbuffer.Temp3status > 400)
	{
		if(Cl_alarms_alarms[TEMP2_LOW_THRESHOLD].cl_alarmstate == CL_ALARM_DETECTED)
		{
			Cl_alarms_alarms[TEMP2_LOW_THRESHOLD].cl_alarmstate = CL_ALARM_INACTIVE;
			Cl_alarms_alarms[TEMP2_LOW_THRESHOLD].cl_detected_count = 0;
			cl_alarm_triggered = false;
		}
	}
}
	if(Cl_alarms_alarms[COND_STATUS_HIGH].cl_is_enabled)
	{
		//Cl_Int16Type cond,cond1;
		//cond = Cl_alarmThresholdTable.cond_high_threshold;
	//	cond1 = cond_final;

		//Cl_SendDatatoconsole(CON_TX_COMMAND_PRINTDATA,&cond,2);
		//Cl_SendDatatoconsole(CON_TX_COMMAND_PRINTDATA,&cond1,2);
		if(cond_final > Cl_alarmThresholdTable.cond_high_threshold) //Cl_alarms_alarms[COND_STATUS_HIGH].cl_upper)
		{
			///Cl_Uint16Type temp_ps;
			//temp_ps=cond_final;

			//Cl_SendDatatoconsole(CON_TX_COMMAND_PRINTTEXT,"CS=",3);
			//Cl_SendDatatoconsole(CON_TX_COMMAND_PRINTDATA,&temp_ps,2);
			Cl_alarms_alarms[COND_STATUS_HIGH].cl_alarmstate = CL_ALARM_DETECTED;
			Cl_MacEventQueue4.Cl_MacEventData[Cl_MacEventQueue4.Cl_MacNewEventcount][0]=COND_STATUS_HIGH;
			Cl_alarms_alarms[COND_STATUS_HIGH].cl_detected_count++;
			cl_alarm_triggered = true;
			cl_lastalarmid = COND_STATUS_HIGH;
			Cl_MacEventQueue4.Cl_MacEvent[Cl_MacEventQueue4.Cl_MacNewEventcount] = EVT_ALARM_TRIGGERED ;
			Cl_alarms_retcode =  Cl_UpdateMacAlarmEventTable();
			ASSERT_ERROR(Cl_alarms_retcode);
		}
		else
		{
			if(Cl_alarms_alarms[COND_STATUS_HIGH].cl_alarmstate == CL_ALARM_DETECTED)
			{
				Cl_alarms_alarms[COND_STATUS_HIGH].cl_alarmstate = CL_ALARM_INACTIVE;
				Cl_alarms_alarms[COND_STATUS_HIGH].cl_detected_count = 0;
				cl_alarm_triggered = false;
			}
		} 
		
	}
	
	if(Cl_alarms_alarms[COND_STATUS_LOW].cl_is_enabled)
	{
		//Cl_SendDatatoconsole(CON_TX_COMMAND_PRINTTEXT,"DETECT_LOW",11);
		//Cl_Uint16Type cond,cond1;
		//cond = Cl_alarmThresholdTable.cond_low_threshold;
		//cond1 = cond_final;
		//Cl_SendDatatoconsole(CON_TX_COMMAND_PRINTDATA,&cond,2);
		//Cl_SendDatatoconsole(CON_TX_COMMAND_PRINTDATA,&cond1,2);
		if(cond_final < Cl_alarmThresholdTable.cond_low_threshold)
		{
			//Cl_Uint16Type temp_ps;
			//temp_ps=cond_final;

			//Cl_SendDatatoconsole(CON_TX_COMMAND_PRINTTEXT,"CS=",3);
			//Cl_SendDatatoconsole(CON_TX_COMMAND_PRINTDATA,&temp_ps,2);
			Cl_alarms_alarms[COND_STATUS_LOW].cl_alarmstate = CL_ALARM_DETECTED;
			Cl_MacEventQueue4.Cl_MacEventData[Cl_MacEventQueue4.Cl_MacNewEventcount][0]=COND_STATUS_LOW;
			Cl_alarms_alarms[COND_STATUS_LOW].cl_detected_count++;
			cl_alarm_triggered = true;
			cl_lastalarmid = COND_STATUS_LOW;
			Cl_MacEventQueue4.Cl_MacEvent[Cl_MacEventQueue4.Cl_MacNewEventcount] = EVT_ALARM_TRIGGERED ;
			Cl_alarms_retcode =  Cl_UpdateMacAlarmEventTable();
			ASSERT_ERROR(Cl_alarms_retcode);
			
		}
		else
		{
			if(Cl_alarms_alarms[COND_STATUS_LOW].cl_alarmstate == CL_ALARM_DETECTED)
			{
				Cl_alarms_alarms[COND_STATUS_LOW].cl_alarmstate = CL_ALARM_INACTIVE;
				Cl_alarms_alarms[COND_STATUS_LOW].cl_detected_count = 0;
				cl_alarm_triggered = false;
			}
		}
	}
	
			
	if((Cl_alarms_alarms[COND_DAC_OPEN].cl_is_enabled) || (Cl_alarms_alarms[COND_DAC_RO].cl_is_enabled) || (Cl_alarms_alarms[COND_DAC_HIGH].cl_is_enabled) )
	{
			dac_cond_alarm = _NO_ALARM;
			Cl_alarms_retcode =  cl_get_dac_conductivity_info(&dac_cond_alarm);
			ASSERT_ERROR(Cl_alarms_retcode);
				switch(dac_cond_alarm)
				{
					case COND_DAC_OPEN:
						Cl_alarms_alarms[dac_cond_alarm].cl_alarmstate = CL_ALARM_DETECTED;
						Cl_alarms_alarms[COND_DAC_RO].cl_alarmstate = CL_ALARM_INACTIVE;
						Cl_alarms_alarms[COND_DAC_HIGH].cl_alarmstate = CL_ALARM_INACTIVE;
						Cl_MacEventQueue4.Cl_MacEventData[Cl_MacEventQueue4.Cl_MacNewEventcount][0]=dac_cond_alarm;
						Cl_alarms_alarms[dac_cond_alarm].cl_detected_count++;
						cl_alarm_triggered = true;
						cl_lastalarmid = dac_cond_alarm;
						Cl_MacEventQueue4.Cl_MacEvent[Cl_MacEventQueue4.Cl_MacNewEventcount] = EVT_ALARM_TRIGGERED ;
						Cl_alarms_retcode =  Cl_UpdateMacAlarmEventTable();
						ASSERT_ERROR(Cl_alarms_retcode);
					break;
					case COND_DAC_RO:
						Cl_alarms_alarms[dac_cond_alarm].cl_alarmstate = CL_ALARM_DETECTED;
						Cl_alarms_alarms[COND_DAC_OPEN].cl_alarmstate = CL_ALARM_INACTIVE;
						Cl_alarms_alarms[COND_DAC_HIGH].cl_alarmstate = CL_ALARM_INACTIVE;
						Cl_MacEventQueue4.Cl_MacEventData[Cl_MacEventQueue4.Cl_MacNewEventcount][0]=dac_cond_alarm;
						Cl_alarms_alarms[dac_cond_alarm].cl_detected_count++;
						cl_alarm_triggered = true;
						cl_lastalarmid = dac_cond_alarm;
						Cl_MacEventQueue4.Cl_MacEvent[Cl_MacEventQueue4.Cl_MacNewEventcount] = EVT_ALARM_TRIGGERED ;
						Cl_alarms_retcode =  Cl_UpdateMacAlarmEventTable();
						ASSERT_ERROR(Cl_alarms_retcode);
					break;
					case COND_DAC_HIGH:
						Cl_alarms_alarms[dac_cond_alarm].cl_alarmstate = CL_ALARM_DETECTED;
						Cl_alarms_alarms[COND_DAC_OPEN].cl_alarmstate = CL_ALARM_INACTIVE;
						Cl_alarms_alarms[COND_DAC_RO].cl_alarmstate = CL_ALARM_INACTIVE;
						Cl_MacEventQueue4.Cl_MacEventData[Cl_MacEventQueue4.Cl_MacNewEventcount][0]=dac_cond_alarm;
						Cl_alarms_alarms[dac_cond_alarm].cl_detected_count++;
						cl_alarm_triggered = true;
						cl_lastalarmid = dac_cond_alarm;
						Cl_MacEventQueue4.Cl_MacEvent[Cl_MacEventQueue4.Cl_MacNewEventcount] = EVT_ALARM_TRIGGERED ;
						Cl_alarms_retcode =  Cl_UpdateMacAlarmEventTable();
						ASSERT_ERROR(Cl_alarms_retcode);

					break;
				}

	}
		
	/*if((Cl_alarms_alarms[COND_STATUS_LOW].cl_is_enabled) || (Cl_alarms_alarms[COND_STATUS_HIGH].cl_is_enabled))
	{
		Cl_SendDatatoconsole(CON_TX_COMMAND_PRINTTEXT,"DETECT",11);
		cond_alarm = _NO_ALARM;
		cl_get_conductivity_info(&cond_alarm);
		switch(cond_alarm)
		{
			case COND_STATUS_LOW:
			if(Cl_alarms_alarms[COND_STATUS_LOW].cl_is_enabled)
			{
				Cl_SendDatatoconsole(CON_TX_COMMAND_PRINTTEXT,"DETECT_LOW",11);
							if(Cl_alarms_alarms[COND_STATUS_LOW].cl_alarmstate != CL_ALARM_DETECTED)
							{
								Cl_alarms_alarms[cond_alarm].cl_alarmstate = CL_ALARM_DETECTED;
								Cl_MacEventQueue1.Cl_MacEventData[Cl_MacEventQueue1.Cl_MacNewEventcount][0]=cond_alarm;
								Cl_alarms_alarms[cond_alarm].cl_detected_count++;
								cl_alarm_triggered = true;
								cl_lastalarmid = cond_alarm;
								Cl_MacEventQueue1.Cl_MacEvent[Cl_MacEventQueue1.Cl_MacNewEventcount] = EVT_ALARM_TRIGGERED ;
								Cl_UpdateMacAlarmEventTable();
							}
				
			}
			break;

			case COND_STATUS_HIGH:
			if(Cl_alarms_alarms[COND_STATUS_HIGH].cl_is_enabled)
			{
				Cl_SendDatatoconsole(CON_TX_COMMAND_PRINTTEXT,"DETECT_HIGH",11);
							if(Cl_alarms_alarms[COND_STATUS_HIGH].cl_alarmstate != CL_ALARM_DETECTED)
							{
								Cl_alarms_alarms[cond_alarm].cl_alarmstate = CL_ALARM_DETECTED;
								Cl_MacEventQueue1.Cl_MacEventData[Cl_MacEventQueue1.Cl_MacNewEventcount][0]=cond_alarm;
								Cl_alarms_alarms[cond_alarm].cl_detected_count++;
								cl_alarm_triggered = true;
								cl_lastalarmid = cond_alarm;
								Cl_MacEventQueue1.Cl_MacEvent[Cl_MacEventQueue1.Cl_MacNewEventcount] = EVT_ALARM_TRIGGERED ;
								Cl_UpdateMacAlarmEventTable();
							}
			}
			break;
			
			default:break;
			
		}
	}	*/


if (  gflow_en)//testing
{
	if((Cl_alarms_alarms[FLOW_NO_FLOW].cl_is_enabled)|| (Cl_alarms_alarms[FLOW_LOW_FLOWRATE].cl_is_enabled)|| (Cl_alarms_alarms[FLOW_HIGH_FLOWRATE].cl_is_enabled))
	{

		if(cl_sys_statbuffer.Flowstatus != prev_flowstatus )
		{
			// flow_watchdog = 1000000 ;
			 flow_watchdog = 0 ;
			 flow_count++;
			 flow_count_rate++;
		}
		else
		{
		//	flow_watchdog = 60;
		//	if(flow_watchdog > 0)
		//	flow_watchdog--;
		}
		
			//if(flow_watchdog < 999400)static Cl_Uint32Type flowstatus_off_alarm_count = 0;static Cl_Uint32Type flowstatus_off_alarm_count = 0;
			if(flow_watchdog > 40 )
			{
				if(Cl_alarms_alarms[FLOW_NO_FLOW].cl_alarmstate != CL_ALARM_DETECTED)
				{
					Cl_alarms_alarms[FLOW_NO_FLOW].cl_alarmstate = CL_ALARM_DETECTED;
					Cl_MacEventQueue4.Cl_MacEventData[Cl_MacEventQueue4.Cl_MacNewEventcount][0]=FLOW_NO_FLOW;
					Cl_alarms_alarms[FLOW_NO_FLOW].cl_detected_count++;
					
					Cl_alarms_alarms[FLOW_NO_FLOW].cl_alarmstate = CL_ALARM_INACTIVE;
				
					cl_alarm_triggered = true;
					
					cl_lastalarmid = FLOW_NO_FLOW;
			//		Cl_SendDatatoconsole(CON_TX_COMMAND_PRINTTEXT,"FLOW_OFF",8);
					
					Cl_MacEventQueue4.Cl_MacEvent[Cl_MacEventQueue4.Cl_MacNewEventcount] = EVT_ALERT_TRIGGERED ;
					Cl_alarms_retcode =  Cl_UpdateMacAlarmEventTable();
					ASSERT_ERROR(Cl_alarms_retcode);
					
				}
			}
			if(flow_count  > 2)
			{
				
				
				
				if(Cl_alarms_alarms[FLOW_HIGH_FLOWRATE].cl_alarmstate != CL_ALARM_DETECTED)
				{
					Cl_alarms_alarms[FLOW_HIGH_FLOWRATE].cl_alarmstate = CL_ALARM_DETECTED;
					Cl_MacEventQueue4.Cl_MacEventData[Cl_MacEventQueue4.Cl_MacNewEventcount][0]=FLOW_HIGH_FLOWRATE;
					Cl_alarms_alarms[FLOW_HIGH_FLOWRATE].cl_detected_count++;
					
					Cl_alarms_alarms[FLOW_HIGH_FLOWRATE].cl_alarmstate = CL_ALARM_INACTIVE;
					
					cl_alarm_triggered = true;
					
					cl_lastalarmid = FLOW_HIGH_FLOWRATE;
					Cl_MacEventQueue4.Cl_MacEvent[Cl_MacEventQueue4.Cl_MacNewEventcount] = EVT_ALERT_TRIGGERED ;
					Cl_alarms_retcode =  Cl_UpdateMacAlarmEventTable();
					ASSERT_ERROR(Cl_alarms_retcode);
					
				}
				
			}
		
		prev_flowstatus = cl_sys_statbuffer.Flowstatus;

	}
}
	
				 
		return CL_OK;
}

  /*
 * \brief  Cl_UpdateMacAlarmEventTable - Update the Cl_MacEventQueue1 buffer status
 * \param   
 * \ 
 * \return CL_OK on success. 
 * \Globals used : Cl_MacEventQueue1
 */

Cl_ReturnCodeType Cl_UpdateMacAlarmEventTable(void)
{
	
	/*
	if((cl_alarm_triggered == true) &&( Cl_MacEventQueue1.Cl_MacNewEventcount <= MAC_EVENT_COUNT_MAX ))


	{
		Cl_MacEventQueue1.Cl_MacNewEvent = true;
		Cl_MacEventQueue1.Cl_MacNewEventcount++;
	}
	*/
	
	if((cl_alarm_triggered == true) &&( Cl_MacEventQueue4.Cl_MacNewEventcount < MAC_EVENT_COUNT_MAX ))


	{
		Cl_MacEventQueue4.Cl_MacNewEvent = true;
		Cl_MacEventQueue4.Cl_MacNewEventcount++;
	}
	
return CL_OK;
}


Cl_ReturnCodeType Cl_UpdateMacBCEventTable(void)
{
	
	/*
	if((cl_alarm_triggered == true) &&( Cl_MacEventQueue1.Cl_MacNewEventcount <= MAC_EVENT_COUNT_MAX ))


	{
		Cl_MacEventQueue1.Cl_MacNewEvent = true;
		Cl_MacEventQueue1.Cl_MacNewEventcount++;
	}
	*/
	
	if((cl_alarm_triggered == true) &&( Cl_MacEventQueueBc.Cl_MacNewEventcount < MAC_EVENT_COUNT_MAX ))


	{
		Cl_MacEventQueueBc.Cl_MacNewEvent = true;
		Cl_MacEventQueueBc.Cl_MacNewEventcount++;
	}
	
return CL_OK;
}


  /*
 * \brief  Cl_AlarmConfigureAlarmType - configure the alarm table with type and values passed as parameters
 * \param   cl_alarm_id ,alarmtriggertype,cl_upper,cl_lower,count_threshold
 * \ 
 * \return CL_OK on success. 
 * \Globals used : Cl_alarms_alarms
 */
Cl_ReturnCodeType Cl_AlarmConfigureAlarmType(Cl_NewAlarmIdType cl_alarm_id , Cl_AlarmTriggerType alarmtriggertype,float cl_upper,float  cl_lower,Cl_Uint8Type  count_threshold )
{
	
Cl_alarms_alarms[cl_alarm_id].cl_triggertype = alarmtriggertype;
Cl_alarms_alarms[cl_alarm_id].cl_lower = cl_lower;
Cl_alarms_alarms[cl_alarm_id].cl_upper = cl_upper;
Cl_alarms_alarms[cl_alarm_id].count_threshold = count_threshold;

return CL_OK;
		
}
  /*
 * \brief  Cl_AlarmActivateAlarms - Activate  the alarm for which the  with id and activation status  passed as parameters
 * \param   
 * \ 
 * \return CL_OK on success. 
 * \Globals used : Cl_alarms_alarms
 */
Cl_ReturnCodeType Cl_AlarmActivateAlarms(Cl_NewAlarmIdType cl_alarm_id , Cl_BoolType status )
{
	Cl_ReturnCodeType cl_AlarmRetCode = CL_OK;
	
	if((cl_alarm_id > _NO_ALARM) && (cl_alarm_id < _ALARM_MAX_ID))
	{

	if (	(cl_alarm_id == LEVELSWITCH_OFF_TO_ON) || (cl_alarm_id == LEVELSWITCH_ON_TO_OFF) || (cl_alarm_id == FPCURRENTSTATUS))
			{
				Cl_alarms_alarms[cl_alarm_id].cl_is_enabled = true;
				Cl_alarms_alarms[cl_alarm_id].cl_alarmstate = CL_ALARM_ACTIVE;
				Cl_alarms_alarms[cl_alarm_id].cl_detected_count = 0;
				
			}
		if ( TEST_STUB_ENABLEGATE)
		{
			Cl_alarms_alarms[cl_alarm_id].cl_alarmID = cl_alarm_id;
			if ( status == true)
			{
				
				Cl_alarms_alarms[cl_alarm_id].cl_is_enabled = true;
				Cl_alarms_alarms[cl_alarm_id].cl_alarmstate = CL_ALARM_ACTIVE;
				Cl_alarms_alarms[cl_alarm_id].cl_detected_count = 0;
				
			}
			else
			{
				Cl_alarms_alarms[cl_alarm_id].cl_is_enabled = false;
				Cl_alarms_alarms[cl_alarm_id].cl_alarmstate = CL_ALARM_INACTIVE;
				Cl_alarms_alarms[cl_alarm_id].cl_detected_count = 0;
				
			}
			
			
		}
		
	}
	else
	{
		cl_AlarmRetCode = CL_ERROR;
	}
	
	

	ASSERT_ERROR(cl_AlarmRetCode);
	
	return(cl_AlarmRetCode);
	
	
}

  /*
 * \brief  Cl_AlarmResetAlarm - Reset the alarm to active state in the alarm table
 * \param   
 * \ 
 * \return CL_OK on success. 
 * \Globals used : Cl_alarms_alarms
 */
Cl_ReturnCodeType Cl_AlarmResetAlarm(Cl_NewAlarmIdType cl_alarm_id )
{
	Cl_ReturnCodeType cl_AlarmRetCode = CL_OK;
		if((cl_alarm_id > _NO_ALARM) && (cl_alarm_id < _ALARM_MAX_ID))
		{
					
					{
						
						Cl_alarms_alarms[cl_alarm_id].cl_alarmstate = CL_ALARM_ACTIVE;
						cl_AlarmRetCode = CL_OK;
					}
			
		}else
		{
			 cl_AlarmRetCode = CL_ERROR;
		}
	
		
	
	return(cl_AlarmRetCode);
	
	
}

  /*
 * \brief  Cl_AlarmMuteAlarms - Change the alarm state to mute in the alarm table
 * \param   
 * \ 
 * \return CL_OK on success. 
 * \Globals used : Cl_alarms_alarms
 */
  
  
 Cl_ReturnCodeType Cl_CheckBcEvent(){

	 
	 if(Cl_alarms_alarms[FPCURRENTSTATUS].cl_is_enabled)
	 {			//FPcurrentstatus_prev = cl_sys_statbuffer.FPcurrentstatus;
		 if(cl_sys_statbuffer.FPcurrentstatus == 1 && FPcurrentstatus_prev == 0 )
		 {
			 
			 
			 Cl_alarms_alarms[FPCURRENTSTATUS].cl_alarmstate = CL_ALARM_DETECTED;
			 Cl_MacEventQueueBc.Cl_MacEventData[Cl_MacEventQueueBc.Cl_MacNewEventcount][0]=FPCURRENTSTATUS;
			 Cl_alarms_alarms[FPCURRENTSTATUS].cl_detected_count++;
			 cl_alarm_triggered = true;
			 cl_lastalarmid = FPCURRENTSTATUS;
			// Cl_MacEventQueueBc.Cl_MacEvent[Cl_MacEventQueueBc.Cl_MacNewEventcount] = EVT_ALERT_TRIGGERED ;
			 //Cl_UpdateMacBCEventTable();
			// ASSERT_ERROR(Cl_alarms_retcode);
			 Cl_MacEventQueueBc.Cl_MacEvent[Cl_MacEventQueueBc.Cl_MacNewEventcount] = EVT_ALERT_CS ;
			 Cl_UpdateMacBCEventTable();//Cl_UpdateMacAlarmEventTable();
			 //ASSERT_ERROR(Cl_alarms_retcode);
			 
			 
		 }
		 else
		 {

			 Cl_alarms_alarms[FPCURRENTSTATUS].cl_alarmstate = CL_ALARM_INACTIVE;
			 Cl_alarms_alarms[FPCURRENTSTATUS].cl_detected_count = 0;

			 
		 }
		 FPcurrentstatus_prev = cl_sys_statbuffer.FPcurrentstatus;
		 
	 }
	 
 }
Cl_ReturnCodeType Cl_AlarmMuteAlarms(Cl_NewAlarmIdType cl_alarm_id)
{
	Cl_ReturnCodeType cl_alarmRetCode = CL_OK;
	
	if((cl_alarm_id > _NO_ALARM) && (cl_alarm_id < _ALARM_MAX_ID))
	{
	
	Cl_alarms_alarms[cl_alarm_id].cl_alarmstate = CL_ALARM_MUTE;
	cl_alarmRetCode = CL_OK;
	
	}else
	{
		cl_alarmRetCode = CL_ERROR;
	}
	ASSERT_ERROR(cl_alarmRetCode);
	return(cl_alarmRetCode);
	
}
  /*
 * \brief  Cl_AlarmMuteAlarms - Populate   the last triggered alarm id in the param passed
 * \param   *cl_alarm_id
 * \ 
 * \return CL_OK on success. 
 * \Globals used : Cl_alarms_alarms
 */
Cl_ReturnCodeType Cl_Alarm_GetLastAlarm(Cl_NewAlarmIdType* cl_alarm_id)
{
	Cl_ReturnCodeType cl_alarmRetCode = CL_ERROR;
	
	if((cl_alarm_id > _NO_ALARM) && (cl_alarm_id < _ALARM_MAX_ID))
	{	
		Cl_ReturnCodeType cl_alarmRetCode = CL_OK;
		*cl_alarm_id = cl_lastalarmid;
		cl_alarmRetCode = CL_OK;
		
	}
	else
	{
		cl_alarmRetCode = CL_ERROR;
		
	}
	ASSERT_ERROR(cl_alarmRetCode);
	return cl_alarmRetCode;
}
  /*
 * \brief  Cl_Alarm_GetAlarmStatus - Populate   the alarm status for  alarm id 
 * \param   cl_alarm_id ,* cl_status
 * \ 
 * \return CL_OK on success. 
 * \Globals used : Cl_alarms_alarms
 */
  
Cl_ReturnCodeType Cl_Alarm_GetAlarmStatus(Cl_NewAlarmIdType cl_alarm_id , Cl_BoolType* cl_status)
{
	Cl_ReturnCodeType cl_alarmRetCode = CL_OK;
	
	if((cl_alarm_id > _NO_ALARM) && (cl_alarm_id < _ALARM_MAX_ID))
	{
	
		 if (Cl_alarms_alarms[cl_alarm_id].cl_alarmstate  == CL_ALARM_DETECTED ||  Cl_alarms_alarms[cl_alarm_id].cl_alarmstate == CL_ALARM_MUTE) 
		 {
			 *cl_status = true;
		 }
		 else
		 {
			 *cl_status = false;
		 }
		 
		 cl_alarmRetCode = CL_OK;
	}else
	{
		cl_alarmRetCode = CL_ERROR;
	}
	 
		
	ASSERT_ERROR(cl_alarmRetCode);
	return cl_alarmRetCode;
	
}

  /*
 * \brief  Cl_Alarm_GetAlertStatus  - Populate   the alarm status for  alarm id 
 * \param   cl_alarm_id ,* cl_status
 * \ 
 * \return CL_OK on success. 
 * \Globals used : Cl_alarms_alarms
 */

Cl_ReturnCodeType Cl_Alarm_GetAlertStatus(Cl_NewAlarmIdType cl_alarm_id , Cl_BoolType* cl_status)
{
	Cl_ReturnCodeType cl_alarmRetCode = CL_ERROR;
	
	if((cl_alarm_id > _NO_ALARM) && (cl_alarm_id < _ALARM_MAX_ID))
	{	
		if (Cl_alarms_alarms[cl_alarm_id].cl_alarmstate  == CL_ALARM_DETECTED ||  Cl_alarms_alarms[cl_alarm_id].cl_alarmstate == CL_ALARM_MUTE)
		{
			*cl_status = true;
			
		}
		else
		{
			*cl_status = false;
			
		}
		cl_alarmRetCode = CL_OK;
	
	}else
	{
		cl_alarmRetCode = CL_ERROR;
	}
	ASSERT_ERROR(cl_alarmRetCode);
	return cl_alarmRetCode;
	
}

Cl_ReturnCodeType Cl_Alarm_TriggerDummyAlarm(Cl_NewAlarmIdType cl_alarm_id , Cl_BoolType cl_status)
{
	Cl_ReturnCodeType cl_alarmRetCode = CL_ERROR;
		
	if((cl_alarm_id > _NO_ALARM) && (cl_alarm_id < _ALARM_MAX_ID))
	{
			
			
			if(cl_status)
			{
				Cl_alarms_alarms[cl_alarm_id].cl_alarmstate = CL_ALARM_DETECTED;
				Cl_MacEventQueue4.Cl_MacEventData[Cl_MacEventQueue4.Cl_MacNewEventcount][0]=cl_alarm_id;
				Cl_alarms_alarms[cl_alarm_id].cl_detected_count++;
				cl_alarm_triggered = true;
				
				cl_lastalarmid = cl_alarm_id;
				Cl_MacEventQueue4.Cl_MacEvent[Cl_MacEventQueue4.Cl_MacNewEventcount] = EVT_ALARM_TRIGGERED ;
				 cl_alarmRetCode = Cl_UpdateMacAlarmEventTable();
				 ASSERT_ERROR(cl_alarmRetCode);
			}
			else
			{
				
				if(Cl_alarms_alarms[cl_alarm_id].cl_alarmstate == CL_ALARM_DETECTED)
				{
					Cl_alarms_alarms[cl_alarm_id].cl_alarmstate = CL_ALARM_INACTIVE;
					Cl_alarms_alarms[cl_alarm_id].cl_detected_count = 0;
					cl_alarm_triggered = false;
				}
				
				
			}
			cl_alarmRetCode = CL_OK;
	}
	else
	{
		cl_alarmRetCode = CL_ERROR;
	}
	ASSERT_ERROR(cl_alarmRetCode);
return CL_OK;
}
  /*
 * \brief  Cl_Alarm_TriggerAlarm  - Force Trigger an Alarm with alarm id and status passed as a param
 * \param   cl_alarm_id ,* cl_status
 * \ 
 * \return CL_OK on success. 
 * \Globals used : Cl_alarms_alarms,Cl_MacEventQueue1
 */

Cl_ReturnCodeType Cl_Alarm_TriggerAlarm(Cl_NewAlarmIdType cl_alarm_id , Cl_BoolType cl_status)
{
	Cl_ReturnCodeType cl_alarmRetCode = CL_OK;
	
if((cl_alarm_id > _NO_ALARM) && (cl_alarm_id < _ALARM_MAX_ID))
{
			
			
	if(Cl_alarms_alarms[cl_alarm_id].cl_is_enabled)
	{
			if(cl_status)
			{
				Cl_alarms_alarms[cl_alarm_id].cl_alarmstate = CL_ALARM_DETECTED;
				Cl_MacEventQueue4.Cl_MacEventData[Cl_MacEventQueue4.Cl_MacNewEventcount][0]=cl_alarm_id;
				Cl_alarms_alarms[cl_alarm_id].cl_detected_count++;
				cl_alarm_triggered = true;
				
				cl_lastalarmid = cl_alarm_id;
				if(FPCURRENTSTATUS == cl_alarm_id)
				{
					Cl_MacEventQueue4.Cl_MacEvent[Cl_MacEventQueue4.Cl_MacNewEventcount] = EVT_ALERT_TRIGGERED ;
				}
				else
				{
						Cl_MacEventQueue4.Cl_MacEvent[Cl_MacEventQueue4.Cl_MacNewEventcount] = EVT_ALARM_TRIGGERED ;
				}
			
				Cl_UpdateMacAlarmEventTable();
			}
			else
			{
				
				if(Cl_alarms_alarms[cl_alarm_id].cl_alarmstate == CL_ALARM_DETECTED)
				{
					Cl_alarms_alarms[cl_alarm_id].cl_alarmstate = CL_ALARM_INACTIVE;
					Cl_alarms_alarms[cl_alarm_id].cl_detected_count = 0;
					cl_alarm_triggered = false;
				}
				
				
			}
		
	}
	cl_alarmRetCode = CL_OK;
}else
{
	cl_alarmRetCode = CL_ERROR;
}
ASSERT_ERROR(cl_alarmRetCode);
return CL_OK;

}

  /*
 * \brief  Cl_Alarm_timmer_50millisec  - 50 millisec alarm timer 
 * \param  
 * \ 
 * \return CL_OK on success. 
 * \Globals used : Cl_alarms_alarms,Cl_MacEventQueue1
 */
  
Cl_ReturnCodeType Cl_Alarm_timmer_50millisec(void) // testing
{
	Cl_ReturnCodeType cl_alarm_retcode = CL_OK;
	static Cl_Uint16Type fiftymscount = 0;
	fiftymscount++;
//	 flow_watchdog = 999900 ;
	flow_count = 0;
//	if(flow_watchdog >= 0)
	flow_watchdog++	;
	
	if(fiftymscount > 20)
	{
		if (  gflow_en)//testing
		{
	//	Cl_SendDatatoconsole(CON_TX_COMMAND_PRINTTEXT,"FLOW_CNT",8);
	//	Cl_SendDatatoconsole(CON_TX_COMMAND_PRINTDATA,&flow_count_rate,2);
		}
		flow_count_rate =0;

		fiftymscount = 0;
	}
	
	return CL_OK;
	
}
