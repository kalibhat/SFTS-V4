/*
 * cl_standby_controller_proto.h
 *
 * Created: 4/25/2017 7:06:16 PM
 *  Author: user
 */ 


#ifndef CL_STANDBY_CONTROLLER_PROTO_H_
#define CL_STANDBY_CONTROLLER_PROTO_H_


Cl_ReturnCodeType Cl_Standby_Controller(Cl_Mac_EventsType );
Cl_ReturnCodeType Cl_stby_init(void);
Cl_ReturnCodeType  cl_stby_translatemacevent(Cl_Mac_EventsType ,Cl_Stby_Events* );
Cl_ReturnCodeType Cl_StabyRinse_UpdateTimeInfo(void);
Cl_ReturnCodeType Cl_standby_UpdateAlarmTable(ClStbyAlarmIdType*  );
Cl_ReturnCodeType Cl_standby_ProcessAlarms(void );




#endif /* CL_STANDBY_CONTROLLER_PROTO_H_ */