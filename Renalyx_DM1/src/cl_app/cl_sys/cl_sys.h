/*
 * cl_sys.h
 *
 * Created: 8/7/2014 11:17:52 AM
 *  Author: user
 */ 


#ifndef CL_SYS_H_
#define CL_SYS_H_






typedef enum 
{
	CL_SYS_STATE_IDLE,
	CL_SYS_STATE_WAIT_FOR_MODE,
	CL_SYS_STATE_NORMAL,
	CL_SYS_STATE_CALIBRATE,
	CL_SYS_STATE_MAX,
}Cl_Sys_StatesType;

#endif /* CL_SYS_H_ */