/*
 * Cl_balancing_chamber_controller.h
 *
 * Created: 11/2/2014 4:14:01 PM
 *  Author: user
 */ 


#ifndef CL_BALANCING_CHAMBER_CONTROLLER_H_
#define CL_BALANCING_CHAMBER_CONTROLLER_H_


typedef enum {
	BC_EVENT_50MS,
	BC_EVENT_500MS,
	BC_EVENT_SECOND,
	BC_EVENT_CS,
	BC_EVENT_START,
	BC_EVENT_STOP,
	BC_EVENT_STOP_FOR_BC1,
	BC_EVENT_START_BC1,
	BC_EVENT_STOP_FOR_BC2,
	BC_EVENT_START_BC2,
	BC_EVENT_PAUSE,
	BC_EVENT_RESUME,
	BC_EVENT_OPENFILL,
	BC_EVENT_STOP_RINSE,
	BC_EVENT_RESET,
	BC_EVENT_DISINF_INTAKE,
	BC_EVENT_TEST_SWITCH
} Cl_BC_EventType;

typedef enum
{
	CL_BC_STATE_IDLE,
	CL_BC_STATE_NC,
	CL_BC_STATE_OPENFILL_TRANSITION,
	CL_BC_STATE_OPENFILL,
	CL_BC_STATE_V15OPEN_TRANSITION,
	CL_BC_STATE_V15OPEN,
	CL_BC_STATE_V2OPEN_TRANSITION,
	CL_BC_STATE_V2OPEN,
	CL_BC_STATE_BO1_V4_TRANSITION,
	CL_BC_STATE_BO1_V4,
	CL_BC_STATE_BO2_V4_TRANSITION,
	CL_BC_STATE_BO2_V4,
	CL_BC_STATE_BO1_V13V14_TRANSITION,
	CL_BC_STATE_BO1_V13V14,
	CL_BC_STATE_BO2_V13V14_TRANSITION,
	CL_BC_STATE_BO2_V13V14,
	CL_BC_STATE_STOPPED_FOR_BC1,
	CL_BC_STATE_BC1_TRANSITION,
	CL_BC_STATE_STOPPED_FOR_BC2,
	CL_BC_STATE_BC2_TRANSITION,
	CL_BC_STATE_COMPLETED_BC1,
	CL_BC_STATE_COMPLETED_BC2,
	CL_BC_STATE_MAXSTATE
} Cl_BC_States;

#define CL_BC_OPENFILL_TIMEOUT  2
#endif /* CL_BALANCING_CHAMBER_C3ONTROLLER_H_ */
