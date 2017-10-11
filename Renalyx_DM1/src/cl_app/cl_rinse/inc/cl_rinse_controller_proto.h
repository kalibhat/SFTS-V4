/*
 * cl_rinse_controller_proto.h
 *
 * Created: 4/25/2017 6:22:48 PM
 *  Author: user
 */ 


#ifndef CL_RINSE_CONTROLLER_PROTO_H_
#define CL_RINSE_CONTROLLER_PROTO_H_



Cl_ReturnCodeType cl_rinse_processtimeoutevets(Cl_Rinse_Eevents cl_rinseevent);
Cl_ReturnCodeType  cl_rinse_translatemacevent(Cl_Mac_EventsType ,Cl_Rinse_Eevents* );
Cl_ReturnCodeType Cl_rinse_init(void);
Cl_ReturnCodeType Cl_Rinse_UpdateTimeInfo(void);
Cl_ReturnCodeType Cl_Rinse_CheckforRinseCompletion(void);
Cl_ReturnCodeType cl_rinse_notifydacandgotorinsestandby(void);
Cl_ReturnCodeType Cl_Rinse_Controller(Cl_Mac_EventsType);
Cl_ReturnCodeType  Cl_Rinse_ProcessAlarms(void);
Cl_ReturnCodeType Cl_RinseConvertalarmidtouint8data(Cl_NewAlarmIdType cl_alarmId,Cl_Uint8Type* data );
Cl_ReturnCodeType  CL_RinseAlarmActon(Cl_NewAlarmIdType cl_rinsealarmid);
Cl_ReturnCodeType  Cl_RinseFlowOn(void );
Cl_ReturnCodeType  Cl_RinseFlowOff(void );
Cl_ReturnCodeType  Cl_RinseStop(void);
Cl_ReturnCodeType Cl_Rinse_ResetAlertsforReassertion(void );
void Cl_Rinse_100msTMP(void);
Cl_ReturnCodeType Cl_Rinse_UpdateSecTick(void);
Cl_ReturnCodeType Cl_Rinse_StartRinse(void);
Cl_ReturnCodeType Cl_Rinse_StopRinse(void);
Cl_ReturnCodeType UpdateRinseMinuteTick(void);
Cl_ReturnCodeType Cl_Rinse_SendRinseStateData(void);
Cl_ReturnCodeType UpdateHeaterControls(void);
Cl_ReturnCodeType SetHeaterState(HeaterStateType Param_HeaterState);
Cl_ReturnCodeType cl_heater_heat(void);
Cl_ReturnCodeType  Cl_RinseSendAlarm(Cl_NewAlarmIdType cl_rinsealarmId );
Cl_ReturnCodeType Cl_Rinse_DisinfStop(void);

#endif /* CL_RINSE_CONTROLLER_PROTO_H_ */