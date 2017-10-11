/*
 * Cl_TimerManager_proto.h
 *
 * Created: 4/25/2017 7:08:05 PM
 *  Author: user
 */ 


#ifndef CL_TIMERMANAGER_PROTO_H_
#define CL_TIMERMANAGER_PROTO_H_

 Cl_ReturnCodeType  Cl_Timer_Create_Tick2Ms_Event(void);
 Cl_ReturnCodeType  Cl_Timer_Create_Tick5Ms_Event(void);
 Cl_ReturnCodeType  Cl_Timer_Create_Tick10Ms_Event(void);
 Cl_ReturnCodeType  Cl_Timer_Create_Tick20Ms_Event(void);
 Cl_ReturnCodeType  Cl_Timer_Create_Tick50Ms_Event(void);
 Cl_ReturnCodeType  Cl_Timer_Create_Tick100Ms_Event(void);
 Cl_ReturnCodeType  Cl_Timer_Create_Tick500Ms_Event(void);
 Cl_ReturnCodeType  Cl_Timer_Create_TickSec_Event(void);
 Cl_ReturnCodeType  Cl_Timer_Create_TickMin_Event(void);
 Cl_ReturnCodeType  Cl_Timer_Create_TickHour_Event(void);
 Cl_ReturnCodeType Cl_TimerTickCallback(void);
 Cl_ReturnCodeType cl_hep_pumpFeedback_timer(void);
 Cl_ReturnCodeType cl_hep_pump_enddetction_timer(void); // 20 ms clock
 Cl_ReturnCodeType  Cl_Timer_Create_Tick150Ms_Event(void);


#endif /* CL_TIMERMANAGER_PROTO_H_ */