/*
 * cl_dprep_controller_proto.h
 *
 * Created: 4/25/2017 4:15:54 PM
 *  Author: user
 */ 


#ifndef CL_DPREP_CONTROLLER_PROTO_H_
#define CL_DPREP_CONTROLLER_PROTO_H_

Cl_ReturnCodeType Cl_dprep_init(void);
Cl_ReturnCodeType Cl_dprep_controller(Cl_Mac_EventsType);
Cl_ReturnCodeType  cl_dprep_translatemacevent(Cl_Mac_EventsType ,Cl_Dprep_Events* );
Cl_ReturnCodeType Cl_Dprep_UpdateTimeInfo(void);
Cl_ReturnCodeType cl_dprep_notifydacandgotodpreptandby(void);
Cl_ReturnCodeType cl_dprep_notifydacandgotodprep_postprimetandby(void);
Cl_ReturnCodeType Cl_Dprep_ProcessAlarms(void );
Cl_ReturnCodeType Cl_Dprep_UpdateAlarmTable( void);
Cl_ReturnCodeType  CL_DrepAlarmActon(Cl_NewAlarmIdType cl_dprepalarmid);
Cl_ReturnCodeType Cl_Dprep_CheckforfillingCompletion(void);
Cl_ReturnCodeType Cl_DprepFillingFlowOff(void);
Cl_ReturnCodeType Cl_DprepFillingFlowOn(void);
Cl_ReturnCodeType Cl_dprep_filling_FSM(Cl_Dprep_Events cl_dprepevent);
Cl_ReturnCodeType  Cl_DprepSelectDialysateInlet( void);
Cl_ReturnCodeType  Cl_DprepFlowOn(void );
Cl_ReturnCodeType  Cl_DprepFlowOff(void );
Cl_ReturnCodeType UpdateDprepMinuteTick(void);
Cl_ReturnCodeType Cl_Dprep_UpdateFillingTimeInfo(void);
Cl_ReturnCodeType UpdateDprepFillingMinuteTick(void);
Cl_ReturnCodeType Cl_Dprep_UpdatePrimeTimeInfo(void);
Cl_ReturnCodeType UpdateDprepPrimeMinuteTick(void);
Cl_ReturnCodeType UpdateDprepDialyserPrimeMinuteTick(void);
Cl_ReturnCodeType Cl_Dprep_UpdateDialyserPrimeTimeInfo(void);
Cl_ReturnCodeType cl_prep_checkforprimecompletion(void);
Cl_ReturnCodeType Cl_dprep_filling_FSM_Init(Cl_Dprep_Events cl_dprepevent);
Cl_ReturnCodeType Cl_Dprep_SendPrepStateData(Cl_Console_bulkdatatype);
Cl_ReturnCodeType Cl_Dprep_Stoppreparation(void);
Cl_ReturnCodeType Cl_Dprep_ResetAlertsforReassertion(void );
Cl_ReturnCodeType Cl_Dprep_SendtreatementData(void);
Cl_ReturnCodeType Cl_Dprep_setdata(Cl_ConsoleRxDataType DataId,cl_PrepDatatype cl_PrepData , Cl_Uint8Type size);
Cl_ReturnCodeType Cl_Dprep_Get_treatmentdata(Cl_ConsoleRxDataType DataId,cl_PrepDatatype cl_PrepData , Cl_Uint8Type size);
Cl_ReturnCodeType Cl_Dprep_Get_data(Cl_ConsoleRxDataType DataId,   Cl_Uint8Type size);

Cl_ReturnCodeType Cl_drepUpdateHeaterControls(void);
Cl_ReturnCodeType	Cl_Dprep_StartPreparation(void);
Cl_ReturnCodeType	Cl_Dprep_StartDialyserPrime(void);
Cl_ReturnCodeType Cl_dprep_StopMixing(void);

Cl_ReturnCodeType cl_dprep_activate_prime_related_alarms(void);
Cl_ReturnCodeType  Cl_dprepSendAlarm(Cl_NewAlarmIdType cl_dprepalarmId );
Cl_ReturnCodeType Cl_Check_Temp_Cond_Stability();
Cl_ReturnCodeType Cl_deactivate_prep_alarms();
Cl_ReturnCodeType Cl_Dprep_Priming_Stop();
Cl_ReturnCodeType Cl_activate_dialyzer_prep_alarms();
#endif /* CL_DPREP_CONTROLLER_PROTO_H_ */