/*
 * cl_conductivity_detector.c
 *
 * Created: 11/11/2014 3:48:29 PM
 *  Author: user
 */ 
#include "../inc/cl_alarmdetector.h"
#include "../comp/inc/cl_conductivity_detector.h"
#include "cl_types.h"

extern Cl_Sys_statusType cl_sys_statbuffer;
extern Cl_AlarmThresholdType  Cl_alarmThresholdTable;
static Cl_Uint16Type cl_ten_pt_cond_avg = 0;
static Cl_Uint16Type cl_ten_pt_dac_cond_avg = 0;
static Sys_cond_statetype cond_state = CL_COND_STATE_NOSTATE;
static Sys_dac_cond_statetype dac_cond_state = CL_COND_STATE_NOSTATE;

Cl_ReturnCodeType cl_get_conductivity_info(Cl_NewAlarmIdType* cl_alarmID);
Cl_ReturnCodeType cl_get_dac_conductivity_info(Cl_NewAlarmIdType* cl_dac_alarmID);

  /*
 * \brief  cl_get_conductivity_info  - Tech information about the conductivity modules alarm status
 * \param  *cl_alarmID
 * \ 
 * \return CL_OK on success. 
 * \Globals used : Cl_alarms_alarms,Cl_MacEventQueue1
 */

Cl_ReturnCodeType cl_get_conductivity_info(Cl_NewAlarmIdType* cl_alarmID)
{
	static Cl_BoolType  started  = false;
	
	if(started == false)
	{
		cl_ten_pt_cond_avg  = cl_sys_statbuffer.cond_status;
		started = true;
	}else
	{
			cl_ten_pt_cond_avg = (9*cl_ten_pt_cond_avg + cl_sys_statbuffer.cond_status)/10;
	}

	
	
	switch(cond_state)
	{
		case CL_COND_STATE_NOSTATE:
		case CL_COND_STATE_NORMAL:
				if(cl_ten_pt_cond_avg < Cl_alarmThresholdTable.cond_low_threshold)
				{
					*cl_alarmID = COND_STATUS_LOW;
					cond_state = CL_COND_STATE_LOW;
				}
				if(cl_ten_pt_cond_avg > Cl_alarmThresholdTable.cond_high_threshold)
				{
				*cl_alarmID = COND_STATUS_HIGH;
				cond_state = CL_COND_STATE_HIGH;
				}
		break;
		case CL_COND_STATE_LOW:

				if(cl_ten_pt_cond_avg > Cl_alarmThresholdTable.cond_high_threshold)
				{
				*cl_alarmID = COND_STATUS_HIGH;
				cond_state = CL_COND_STATE_HIGH;
				}
				else if(cl_ten_pt_cond_avg >  Cl_alarmThresholdTable.cond_low_threshold)
				{

					cond_state = CL_COND_STATE_NORMAL;
				}
				else
				{
					
					*cl_alarmID = COND_STATUS_LOW;
					cond_state = CL_COND_STATE_LOW;
				}
		break;
		case CL_COND_STATE_HIGH:

				if(cl_ten_pt_cond_avg < Cl_alarmThresholdTable.cond_low_threshold)
				{
					*cl_alarmID = COND_STATUS_LOW;
					cond_state = CL_COND_STATE_LOW;
				}
				else if(cl_ten_pt_cond_avg < Cl_alarmThresholdTable.cond_high_threshold)
				{

					cond_state = CL_COND_STATE_NORMAL;
				}
				else
				{
					*cl_alarmID = COND_STATUS_HIGH;
					cond_state = CL_DAC_COND_STATE_HIGH;
				}
		break;
		default:break;
	}
	
	return CL_OK;
}

  /*
 * \brief  cl_get_dac_conductivity_info  - Tech information about the DAC conductivity modules alarm status
 * \param  *cl_alarmID
 * \ 
 * \return CL_OK on success. 
 * \Globals used : Cl_alarms_alarms,Cl_MacEventQueue1
 */
  
Cl_ReturnCodeType cl_get_dac_conductivity_info(Cl_NewAlarmIdType* cl_dac_alarmID)
{
cl_ten_pt_dac_cond_avg = (9*cl_ten_pt_dac_cond_avg + cl_sys_statbuffer.cond_status_dac)/10;	
switch(dac_cond_state)
	{
		case CL_DAC_COND_STATE_NOSTATE:
		//break;
		case CL_DAC_COND_STATE_OPEN:

		if((cl_ten_pt_dac_cond_avg > 50) &&(cl_ten_pt_dac_cond_avg < 2000))
		{
			*cl_dac_alarmID = COND_DAC_RO;
			dac_cond_state = CL_DAC_COND_STATE_RO;
		}

		if(cl_ten_pt_dac_cond_avg > 2000)
		{
			*cl_dac_alarmID = COND_DAC_HIGH;
			dac_cond_state = CL_DAC_COND_STATE_HIGH;
		}
		break;
		
		case CL_DAC_COND_ALARM_RO:
				if(cl_ten_pt_dac_cond_avg > 2000)
				{
					*cl_dac_alarmID = COND_DAC_HIGH;
					dac_cond_state = CL_DAC_COND_STATE_HIGH;
				}
				if(cl_ten_pt_dac_cond_avg < 50)
				{
					*cl_dac_alarmID = COND_DAC_OPEN;
					dac_cond_state = CL_DAC_COND_STATE_OPEN;					
				}
		break;
		case CL_DAC_COND_STATE_HIGH:
		if((cl_ten_pt_dac_cond_avg > 50) &&(cl_ten_pt_dac_cond_avg < 2000))
				{
					*cl_dac_alarmID = COND_DAC_RO;
					dac_cond_state = CL_DAC_COND_STATE_RO;
				}
		if(cl_ten_pt_dac_cond_avg < 50)
				{
					*cl_dac_alarmID = COND_DAC_OPEN;
					dac_cond_state = CL_DAC_COND_STATE_OPEN;
				}
		break;
	}
return CL_OK;
}
		
	
