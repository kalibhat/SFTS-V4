/*
 * cl_dlsiscontroller_proto.h
 *
 * Created: 4/25/2017 2:14:06 PM
 *  Author: user
 */ 


#ifndef CL_DLSISCONTROLLER_PROTO_H_
#define CL_DLSISCONTROLLER_PROTO_H_

Cl_ReturnCodeType Cl_dlsis_init(void);
Cl_ReturnCodeType cl_dlsis_translatemacevent(Cl_Mac_EventsType ,Cl_Dlsis_Eevents* );
Cl_ReturnCodeType Cl_Dlsis_UpdateTimeInfo(void);
Cl_ReturnCodeType Cl_Dlsis_controller(Cl_Mac_EventsType );
Cl_ReturnCodeType cl_Dlsis_CheckforDialysisCompletion(void);
Cl_ReturnCodeType cl_Dlsis_notifydacandgotorinsestandby(void);
Cl_ReturnCodeType Cl_Dlsis_ProcessAlarms( void);
Cl_ReturnCodeType Cl_Dlsis_StopDialysis(void);
Cl_ReturnCodeType Cl_Dlsis_PauseDialysis(void);
Cl_ReturnCodeType  CL_DlsisAlarmActon(Cl_NewAlarmIdType cl_dlsisalarmid);
Cl_ReturnCodeType  Cl_DlsisFillingFlowOn(void );
Cl_ReturnCodeType  Cl_DlsisFillingFlowOff(void );
Cl_ReturnCodeType Cl_Dlsis_SenddlsisData(void);
Cl_ReturnCodeType Cl_Dlsis_Get_data(Cl_ConsoleRxDataType DataId, Cl_Uint8Type size);
Cl_ReturnCodeType	Cl_Dlsis_StartDialysis(void);
Cl_ReturnCodeType	Cl_Dlsis_ResumeDialysis(void);
Cl_ReturnCodeType Cl_Dlsis_BypassDialysis(void);
Cl_ReturnCodeType	Cl_Dlsis_recoveredsate(void);
Cl_ReturnCodeType	Cl_Dlsis_safesate(void);
Cl_ReturnCodeType cl_temp_safestate(void);
Cl_ReturnCodeType cl_temp_recoveredstate(void);
Cl_ReturnCodeType cl_cond_safestate(void);
Cl_ReturnCodeType cl_cond_recoveredstate(void);
Cl_ReturnCodeType Cl_Dlsis_BypassOff(void);
Cl_ReturnCodeType Cl_Dlsis_ResetAlertsforReassertion(void );
Cl_ReturnCodeType dlsis_check_alarm(Cl_NewAlarmIdType dlsisalarm);
Cl_ReturnCodeType cl_Dlsis_checkforHeparinCompletion(void);


#endif /* CL_DLSISCONTROLLER_PROTO_H_ */