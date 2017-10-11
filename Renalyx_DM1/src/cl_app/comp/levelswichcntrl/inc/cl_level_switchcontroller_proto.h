/*
 * cl_level_switchcontroller_proto.h
 *
 * Created: 4/25/2017 7:28:07 PM
 *  Author: user
 */ 


#ifndef CL_LEVEL_SWITCHCONTROLLER_PROTO_H_
#define CL_LEVEL_SWITCHCONTROLLER_PROTO_H_

Cl_ReturnCodeType cl_get_ls_switch_time(int16_t* ls_ontime,int16_t* ls_offtime);
Cl_ReturnCodeType cl_level_switchcontroller(levelswitchcntrl_event_type cntrl_event );
Cl_ReturnCodeType cl_level_switchcontroller(levelswitchcntrl_event_type cntrl_event);



#endif /* CL_LEVEL_SWITCHCONTROLLER_PROTO_H_ */