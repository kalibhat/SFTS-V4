/*
 * cl_alarmdetector_ext.h
 *
 * Created: 4/25/2017 11:15:06 AM
 *  Author: user
 */ 


#ifndef CL_ALARMDETECTOR_EXT_H_
#define CL_ALARMDETECTOR_EXT_H_


extern Cl_ReturnCodeType Cl_Init_Alarms(void);

extern Cl_ReturnCodeType Cl_alarms_scanforAlarms(void);
extern Cl_ReturnCodeType Cl_Alarm_translate_sensor_values(void);
extern Cl_ReturnCodeType Cl_LoadAlarmThresholdTable(void);
extern Cl_ReturnCodeType Cl_AlarmActivateAlarms(Cl_NewAlarmIdType, Cl_BoolType  );
extern Cl_ReturnCodeType Cl_AlarmMuteAlarms(Cl_NewAlarmIdType cl_alarm_id);
extern Cl_ReturnCodeType Cl_GetLastAlarm(Cl_NewAlarmIdType cl_alarm_id);
extern Cl_ReturnCodeType Cl_Alarm_GetAlarmStatus(Cl_NewAlarmIdType  , Cl_BoolType* );
extern Cl_ReturnCodeType Cl_UpdateMacAlarmEventTable(void);
extern Cl_ReturnCodeType Cl_AlarmResetAlarm(Cl_NewAlarmIdType cl_alarm_id );
extern Cl_ReturnCodeType Cl_Alarm_TriggerDummyAlarm(Cl_NewAlarmIdType cl_alarm_id , Cl_BoolType cl_status);
extern Cl_ReturnCodeType Cl_Alarm_TriggerAlarm(Cl_NewAlarmIdType cl_alarm_id , Cl_BoolType cl_status);
extern Cl_ReturnCodeType Cl_Alarm_timmer_50millisec(void);
extern Cl_ReturnCodeType Cl_AlarmConfigureAlarmType(Cl_NewAlarmIdType,Cl_AlarmTriggerType,float,float,Cl_Uint8Type);
extern Cl_ReturnCodeType Cl_CheckBcEvent(void);

#endif /* CL_ALARMDETECTOR_EXT_H_ */