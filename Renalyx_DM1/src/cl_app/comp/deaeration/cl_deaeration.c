/*
 * cl_deaeration.c
 *
 * Created: 7/30/2015 11:52:47 AM
 *  Author: user
 */ 

#include "cl_deaeration_include.h"
#include "cl_deaeration_proto.h"
#include "sv_interface_fun.h"

extern Cl_ReturnCodeType Cl_SysStat_GetSensor_Status_Query(Cl_SensorDeviceIdType, uint16_t*);
// extern Cl_Uint8Type sv_cntrl_activate_valve(sv_valvetype );
// extern Cl_Uint8Type sv_cntrl_deactivate_valve(sv_valvetype );

cl_deaeration_state_type cl_deaeration_state = CL_DEAER_STATE_IDLE;

Cl_ReturnCodeType cl_deaeration_controller(cl_deaeration_event_type cntrl_event)
{
	Cl_ReturnCodeType cl_ret_value = CL_OK;
	Cl_Uint16Type dac2_status  ;
	
	switch (cl_deaeration_state)
	{
		case CL_DEAER_STATE_IDLE:
				switch(cntrl_event)
				{
					case CL_DEAER_EVENT_ACTIVATE:
					cl_deaeration_state = CL_DEAER_ACTIVE;
					break;
					default:break;
				}
		break;
		case CL_DEAER_ACTIVE:
			switch(cntrl_event)
			{
				

				case CL_DEAER_EVENT_DEACTIVATE:
				cl_deaeration_state = CL_DEAER_STATE_IDLE;
				break;
				case CL_DEAER_EVENT_50MS:
				// check for deaeartion 
				Cl_SysStat_GetSensor_Status_Query(COND_DAC_RO,&dac2_status);
				if (dac2_status < 10)
				{
					sv_cntrl_activate_valve(VALVE_ID19);
					cl_deaeration_state = CL_DEAER_ACTIVE_INPROGRESS;
				}
				else
				{
					sv_cntrl_deactivate_valve(VALVE_ID19);	
				}
				break;
				default:
				break;
			}
		break;
		case CL_DEAER_ACTIVE_INPROGRESS:
					switch(cntrl_event)
					{
						
						case CL_DEAER_EVENT_DEACTIVATE:
						cl_deaeration_state = CL_DEAER_STATE_IDLE;
						break;
						case CL_DEAER_EVENT_500MS:
						Cl_SysStat_GetSensor_Status_Query(COND_DAC_RO,&dac2_status);
						if (dac2_status > 5)
						{
						sv_cntrl_deactivate_valve(VALVE_ID19);
						cl_deaeration_state = CL_DEAER_ACTIVE;
						}
					}
		
		break;
		default:break;
	}

	return CL_OK;
}