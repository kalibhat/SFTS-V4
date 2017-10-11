/*
 * cl_ufPumpController_ext.h
 *
 * Created: 4/25/2017 12:32:04 PM
 *  Author: user
 */ 


#ifndef CL_UFPUMPCONTROLLER_EXT_H_
#define CL_UFPUMPCONTROLLER_EXT_H_

#include "cl_ufPumpController.h"
extern Cl_ReturnCodeType cl_uf_controller(cl_uf_events uf_event , int32_t data);
extern Cl_ReturnCodeType cl_isouf_controller(Cl_Mac_EventsType Cl_MacDlsisEvent);
extern Cl_ReturnCodeType Cl_isouf_init(void);


#endif /* CL_UFPUMPCONTROLLER_EXT_H_ */