/*
 * cl_disinf_controller_proto.h
 *
 * Created: 4/25/2017 1:10:04 PM
 *  Author: user
 */ 


#ifndef CL_DISINF_CONTROLLER_PROTO._H_
#define CL_DISINF_CONTROLLER_PROTO._H_

#include "cl_types.h"

 Cl_ReturnCodeType Cl_disinf_init(void);
 Cl_ReturnCodeType Cl_disinf_start(void);
 Cl_ReturnCodeType Cl_disinf_controller(Cl_Mac_EventsType Cl_MacDisinfEvent);
 Cl_ReturnCodeType  Cl_disinf_ProcessAlarms(void);
 Cl_ReturnCodeType Cl_disinf_ResetAlertsforReassertion(void );
 Cl_ReturnCodeType Cl_Disinf_StartDisinf(void);
 Cl_ReturnCodeType Cl_Disinf_StopDisinf(void);
 Cl_ReturnCodeType Cl_Disinf_resumeDisinf(void);
 Cl_ReturnCodeType Cl_Disinf_UpdateTimeInfo(void);
 Cl_ReturnCodeType UpdateDisinfMinuteTick(void);
 Cl_ReturnCodeType Cl_Disinf_SendDisinfStateData(void);
 Cl_ReturnCodeType  cl_disinf_translatemacevent(Cl_Mac_EventsType Cl_MacdisinfEvt,Cl_disinf_Eevents* cl_disinf_event);
 Cl_ReturnCodeType Cl_Disinf_UpdateAlarmTable(CldisinfAlarmIdType*);
 




#endif /* CL_DISINF_CONTROLLER_PROTO._H_ */