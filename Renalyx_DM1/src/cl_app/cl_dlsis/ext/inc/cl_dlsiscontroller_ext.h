/*
 * cl_dlsiscontroller_ext.h
 *
 * Created: 4/25/2017 11:06:24 AM
 *  Author: user
 */ 


#ifndef CL_DLSISCONTROLLER_EXT_H_
#define CL_DLSISCONTROLLER_EXT_H_

#include "cl_dlsis_controller.h"


extern Cl_ReturnCodeType Cl_dlsis_init(void);
extern Cl_ReturnCodeType cl_dlsis_translatemacevent(Cl_Mac_EventsType ,Cl_Dlsis_Eevents* );
extern Cl_ReturnCodeType Cl_Dlsis_UpdateTimeInfo(void);
extern Cl_ReturnCodeType Cl_Dlsis_controller(Cl_Mac_EventsType );
extern Cl_ReturnCodeType cl_Dlsis_CheckforDialysisCompletion(void);
extern Cl_ReturnCodeType cl_Dlsis_notifydacandgotorinsestandby(void);
extern Cl_ReturnCodeType Cl_Dlsis_ProcessAlarms( void);
extern Cl_ReturnCodeType Cl_Dlsis_StopDialysis(void);
extern Cl_ReturnCodeType Cl_Dlsis_PauseDialysis(void);
extern Cl_ReturnCodeType  CL_DlsisAlarmActon(Cl_NewAlarmIdType cl_dlsisalarmid);
extern Cl_ReturnCodeType  Cl_DlsisFillingFlowOn(void );
extern Cl_ReturnCodeType  Cl_DlsisFillingFlowOff(void );
extern Cl_ReturnCodeType Cl_Dlsis_SenddlsisData(void);
extern Cl_ReturnCodeType Cl_Dlsis_Get_data(Cl_ConsoleRxDataType DataId, Cl_Uint8Type size);
extern Cl_ReturnCodeType	Cl_Dlsis_StartDialysis(void);
extern Cl_ReturnCodeType	Cl_Dlsis_ResumeDialysis(void);
extern Cl_ReturnCodeType Cl_Dlsis_BypassDialysis(void);
extern Cl_ReturnCodeType	Cl_Dlsis_recoveredsate(void);
extern Cl_ReturnCodeType	Cl_Dlsis_safesate(void);


#endif /* CL_DLSISCONTROLLER_EXT_H_ */