/*
 * cl_selftest.h
 *
 * Created: 8/17/2017 11:25:08 AM
 *  Author: VarunK
 */ 


#ifndef CL_SELFTEST_H_
#define CL_SELFTEST_H_
//#include "cl_types.h"
#include "cl_mac_controller.h"
#include "cl_th.h"
#include "cl_consolecontroller.h"
#include "sv_stubs/inc/sv_stub.h"
#include "Platform/Service/sv_interface.h"
#include "cl_alarmdetector.h"
#include "cl_ufPumpController_ext.h"
#include "cl_ufPumpFeedback_ext.h"
#include "cl_ufPumpController.h"
#include  "cl_heparinfeedback.h"
#include "calibration.h"
#include "cl_heparinfeedback_ext.h"
#include "sv_interface_fun.h"
#include "cl_heparincontroller.h"
#include "cl_dlsis_controller.h"
#include "cl_status_ext.h"
#include "Cl_temperature_sensor_ext.h"
#include "cl_mac_controller_ext_.h"
#include "cl_ufPumpFeedback.h"
#include "cl_heparinfeedback.h"
#include "cl_bloodpumpFeedback.h"
#include "cl_consolecontroller_ext.h"

extern Cl_ReturnCodeType self_test(Cl_Mac_EventsType);

typedef enum
{
	EVENT_NULL_null,
	EVENT_start_selftest,
	
	EVENT_STOP_selftest,
	EVENT_selftest_TIMEOUT,
	EVENT_selftest_COMPLETED,
	EVENT_selftest_TICK_50MS,
	EVENT_selftest_TICK_100MS,
	EVENT_selftest_TICK_500MS,
	EVENT_selftest_TICK_SECOND,
	EVENT_selftest_TICK_MINUTE,
	EVENT_selftest_ALARM_PAAS,
	EVENT_selftest_ALARM_FAIL,
    EVENT_selftest_SYNC_ACK,
	EVENT_selftest_MAXEVENT
}cl_selftest_event;

typedef enum
{
	CL_SELFTEST_STATE_IDLE,
	CL_SELFTEST_STATE_sync,
	CL_SELFTEST_STATE_START_ALARMS_BUZZERS,
	CL_SELFTEST_STATE_STOP_ALARMS_BUZZERS,
	CL_SELFTEST_STATE_START_UFPUMP,
	CL_SELFTEST_STATE_UFPUMP_STATUS,
	CL_SELFTEST_STATE_START_HPPUMP,
	CL_SELFTEST_STATE_HPPUMP_STATUS,
	CL_SELFTEST_STATE_START_BDPUMP,
	CL_SELFTEST_STATE_BDPUMP_STATUS,
//	CL_SELFTEST_STATE_START_ACIDPUMP,
//	CL_SELFTEST_STATE_ACIDPUMP_STATUS,
	CL_SELFTEST_STATE_START_PISTON_PUMP,
	CL_SELFTEST_STATE_STOP_PISTON_PUMP,
	CL_SELFTEST_STATE_ACIDPUMP_STATUS,
	CL_SELFTEST_STATE_BICARBPUMP_STATUS,
	CL_SELFTEST_STATE_APT_STATUS,
	CL_SELFTEST_STATE_VPT_STATUS,
	CL_SELFTEST_STATE_SENSOR_TEMPSTATUS,
	CL_SELFTEST_STATE_SENSOR_PSTATUS,
	CL_SELFTEST_STATE_SENSOR_CONDSTATUS,
	CL_SELFTEST_STATE_SENSOR_VENOUSCLAMP,
	CL_SELFTEST_STATE_VENOUSCLAMP_STATUS,
	CL_SELFTEST_STATE_RUNNING,
	CL_SELFTEST_STATE_STOP,
	CL_SELFTEST_STATE_COMPLETED,
	CL_SELFTEST_STATE_MAXSTATE
	
	
} Cl_selftest_StatesType;

typedef enum
{
	CL_SELFTEST_RESULT_SYNC = 0,
	CL_SELFTEST_RESULT_ALARM,
	CL_SELFTEST_RESULT_UFPUMP,
	CL_SELFTEST_RESULT_HPPUMP,
	CL_SELFTEST_RESULT_BDPUMP,
	CL_SELFTEST_RESULT_ACIDPUMP,
	CL_SELFTEST_RESULT_BICARBPUMP,
	CL_SELFTEST_RESULT_APT_STATUS,
	CL_SELFTEST_RESULT_VPT_STATUS,
	CL_SELFTEST_RESULT_TEMPSTATUS1,
	CL_SELFTEST_RESULT_TEMPSTATUS2,
	CL_SELFTEST_RESULT_TEMPSTATUS3,
	CL_SELFTEST_RESULT_PSTATUS1,
	CL_SELFTEST_RESULT_PSTATUS2,
	CL_SELFTEST_RESULT_PSTATUS3,
	CL_SELFTEST_RESULT_CONDSTATUS1,
	CL_SELFTEST_RESULT_CONDSTATUS2,
	CL_SELFTEST_RESULT_CONDSTATUS3,
	CL_SELFTEST_RESULT_VENOUSCLAMP,
// 	CL_SELFTEST_RESULT_BLOOD_DTECTOR,
// 	CL_SELFTEST_RESULT_TMP,
// 	CL_SELFTEST_RESULT_LEVEL_SWITCH,
	
	CL_SELFTEST_RESULT_MAXSTATE
	
	
} Cl_selftest_Component_Type;

typedef enum
{
	
	TEST_FAIL,
	TEST_PAAS,
	TEST_ABORTED,
	TEST_MAXSTATE
	
	}Cl_selftest_T_F_Type;
	
	
	

typedef struct
{
	Cl_Uint8Type	Cl_TEST_DATA;
	 Cl_selftest_Component_Type Cl_selftest_Components[CL_SELFTEST_RESULT_MAXSTATE];
	
}Cl_selftest_status_Type;



extern Cl_ReturnCodeType cl_selftest_translatemacevent(Cl_Mac_EventsType ,cl_selftest_event*  );
extern Cl_ReturnCodeType Cl_selftest_init(void);

//#define  Cl_Uint16Type uint16_t

#define Expected_uf_test_value 300
#define Expected_uf_test_value1 400
#define Expected_hp_test_value1 200
#define Expected_hp_test_value2 400
#define Expected_bp_test_value1 20
#define Expected_bp_test_value2 80
#define Expected_bicarb_test_value1 2
#define Expected_bicarb_test_value2 10 
#define Expected_acid_test_value1 2
#define Expected_acid_test_value2 10
#define expected_higherapt_value 50
#define expected_lowerapt_value -50
#define expected_highervpt_value 50
#define expected_lowervpt_value -50
#define expected_lowertemp1_value 20
#define expected_highertemp1_value 28
#define expected_lowertemp2_value 20
#define expected_highertemp2_value 30
#define expected_lowertemp3_value 20
#define expected_highertemp3_value 32
#define expected_lowerPS1_value -500
#define expected_higherPS1_value 200
#define expected_lowerPS2_value -500
#define expected_higherPS2_value 200
#define expected_lowerPS3_value -500
#define expected_higherPS3_value 1200
#define expected_lowercond1_value 0
#define expected_highercond1_value 5
#define expected_lowercond2_value 0
#define expected_highercond2_value 5
#define expected_lowercond3_value 0
#define expected_highercond3_value 5




#endif /* CL_SELFTEST_H_ */