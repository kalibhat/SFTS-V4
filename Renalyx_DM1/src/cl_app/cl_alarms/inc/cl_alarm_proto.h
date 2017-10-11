/*
 * cl_alarm_proto.h
 *
 * Created: 4/25/2017 12:09:33 PM
 *  Author: user
 */ 


#ifndef CL_ALARM_PROTO_H_
#define CL_ALARM_PROTO_H_

#include "cl_types.h"
#include "cl_alarmdetector.h"
		
		Cl_ReturnCodeType Cl_Init_Alarms(void);
		Cl_ReturnCodeType Cl_alarms_scanforAlarms(void);
		Cl_ReturnCodeType Cl_Alarm_translate_sensor_values(void);
		Cl_ReturnCodeType Cl_LoadAlarmThresholdTable(void);
		Cl_ReturnCodeType Cl_AlarmActivateAlarms(Cl_NewAlarmIdType, Cl_BoolType  );
		Cl_ReturnCodeType Cl_AlarmMuteAlarms(Cl_NewAlarmIdType cl_alarm_id);
		Cl_ReturnCodeType Cl_GetLastAlarm(Cl_NewAlarmIdType cl_alarm_id);
		Cl_ReturnCodeType Cl_Alarm_GetAlarmStatus(Cl_NewAlarmIdType  , Cl_BoolType* );
		Cl_ReturnCodeType Cl_UpdateMacAlarmEventTable(void);
		Cl_ReturnCodeType Cl_AlarmResetAlarm(Cl_NewAlarmIdType cl_alarm_id );
		Cl_ReturnCodeType Cl_Alarm_TriggerDummyAlarm(Cl_NewAlarmIdType cl_alarm_id , Cl_BoolType cl_status);
		Cl_ReturnCodeType Cl_Alarm_TriggerAlarm(Cl_NewAlarmIdType cl_alarm_id , Cl_BoolType cl_status);
		Cl_ReturnCodeType Cl_Alarm_timmer_50millisec(void);
		Cl_ReturnCodeType Cl_AlarmConfigureAlarmType(Cl_NewAlarmIdType,Cl_AlarmTriggerType,float,float,Cl_Uint8Type);
Cl_ReturnCodeType Cl_UpdateMacBCEventTable(void);
Cl_ReturnCodeType Cl_CheckBcEvent(void);



#endif /* CL_ALARM_PROTO_H_ */