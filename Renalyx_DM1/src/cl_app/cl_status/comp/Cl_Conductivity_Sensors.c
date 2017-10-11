/*
 * Cl_Conductivity_Sensors.c
 *
 * Created: 4/13/2016 5:25:09 PM
 *  Author: user
 */ 

#include "cl_types.h"
#include "inc/Cl_Conductivity_Sensors.h"
#include "cl_alarmdetector.h"
#include "cl_consolecontroller.h"

extern Cl_Uint8Type sv_cs_setcondpotvalue(Cl_Uint16Type resistance);
extern Cl_ReturnCodeType 		Cl_SysStat_GetSensor_Status_Query(Cl_SensorDeviceIdType, uint16_t*);
extern Cl_ReturnCodeType cl_wait(Cl_Uint32Type );
extern Cl_ReturnCodeType  Cl_SendDatatoconsole(Cl_ConsoleTxCommandtype , Cl_Uint8Type* ,Cl_Uint8Type );

Cl_CondSensor_StateType Cond_Sensor_State = COND_STATE_IDLE;

Cl_ReturnCodeType  Cl_Conductivity_Sensor_Controller(Cl_CondSensor_EventType Cl_cond_Event);

Cl_ReturnCodeType  Cl_Conductivity_Sensor_Controller(Cl_CondSensor_EventType Cl_cond_Event)
{
		Cl_ReturnCodeType cl_retval = CL_OK;
		Cl_Uint16Type temp_val  = 0, cond_millivolts = 0;
		static Cl_Uint16Type cond_pot_val_2volts = (7000 * 1024)/10000; 
		static Cl_Uint16Type cond_millivolts_avg = 0,cond_3sec_avg=0;
		
		switch(Cond_Sensor_State)
		{
			
			case COND_STATE_IDLE:
				switch (Cl_cond_Event)
					{
						case COND_EVENT_INIT:
						sv_cs_setcondpotvalue(cond_pot_val_2volts);
						cl_wait(100);
						Cl_SysStat_GetSensor_Status_Query(SENSOR_COND_WIEN_STATUS,&temp_val);
						cond_millivolts = 5 * temp_val;
						if((cond_millivolts_avg > 1800) && (cond_millivolts_avg < 2200))
						{
							return CL_OK;
						}
						else
						{
							return CL_ERROR;
						}
						
						break;
		
						case COND_EVENT_1SEC:
		
						Cl_SysStat_GetSensor_Status_Query(SENSOR_COND_WIEN_STATUS,&temp_val);
						cond_millivolts = 0.8036 * temp_val;
						cond_millivolts_avg = cond_millivolts;
						cond_3sec_avg = cond_millivolts;
						Cond_Sensor_State = COND_STATE_ACTIVE;
						if((cond_millivolts_avg > 1800) && (cond_millivolts_avg < 2200))
						{
							return CL_OK;
						}
						else
						{
							sv_cs_setcondpotvalue(cond_pot_val_2volts);
							return CL_OK;
						}
						
						break;
						default:break;
					}
			break;
			case COND_STATE_ACTIVE:
				switch (Cl_cond_Event)
				{

					case COND_EVENT_1SEC:
		
					Cl_SysStat_GetSensor_Status_Query(SENSOR_COND_WIEN_STATUS,&temp_val);
					cond_millivolts = 0.8036 * temp_val;
					
					if((cond_millivolts_avg > 1800) && (cond_millivolts_avg < 2200))
					{
							cond_millivolts_avg = (cond_millivolts_avg * 19 +cond_millivolts)/20;
							cond_3sec_avg = (cond_millivolts_avg * 2 +cond_millivolts)/3;
						//	Cl_SendDatatoconsole(CON_TX_COMMAND_PRINTDATA,&cond_3sec_avg,2);
							//Cl_SendDatatoconsole(CON_TX_COMMAND_PRINTTEXT,"AVG",3);
						//	Cl_SendDatatoconsole(CON_TX_COMMAND_PRINTDATA,&cond_millivolts_avg,2);
						if (cond_3sec_avg > 2005 )
						{
							//Cl_SendDatatoconsole(CON_TX_COMMAND_PRINTTEXT,"MORE",4);
							cond_pot_val_2volts = cond_pot_val_2volts -1;
							sv_cs_setcondpotvalue(cond_pot_val_2volts );
						}
						else if (cond_3sec_avg < 1995 )
						{
							//Cl_SendDatatoconsole(CON_TX_COMMAND_PRINTTEXT,"LESS",4);
							cond_pot_val_2volts = cond_pot_val_2volts + 1;
							sv_cs_setcondpotvalue(cond_pot_val_2volts );
						}
						else
						{
			
						}
					}
					else
					{
						return CL_ERROR;
					}
					break;
					default:break;
				}
				
			break;
			default:break;
		}
		
	return CL_OK;
}