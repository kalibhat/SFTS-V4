/*
 * cl_treatment.h
 *
 * Created: 5/17/2016 11:54:34 AM
 *  Author: user
 */ 


#ifndef CL_TREATMENT_H_
#define CL_TREATMENT_H_

#include "cl_types.h"

typedef struct  
{

Cl_Uint32Type Val_dflow;
Cl_Uint32Type Val_settemp;
Cl_Uint32Type Val_heprate;
Cl_Uint32Type Val_apt;
Cl_Uint32Type Val_vpt;
Cl_Uint32Type Val_tmp;
Cl_Uint32Type Val_cond;
Cl_Uint32Type Val_ufrate;
Cl_Uint32Type Val_ufgoal;
Cl_Uint32Type Val_bolusvol;
Cl_Uint32Type Val_bloodrate;
Cl_Uint32Type Val_bicarbcond;
Cl_Uint32Type Val_hepstoptime;
Cl_Uint32Type Val_syringetype;
Cl_Uint32Type Val_heparincheck;
Cl_Uint32Type Val_minufrate;
Cl_Uint32Type Val_treattime;
Cl_Uint32Type Val_bloodratereturn;
Cl_Uint32Type Val_bloodratetreat;
Cl_Uint32Type Val_tempulimit;
Cl_Uint32Type Val_templlimit;
Cl_Uint32Type Val_tmpllimit;
Cl_Uint32Type Val_tmpulimit;
Cl_Uint32Type Val_vptllimit;
Cl_Uint32Type Val_vptulimit;
Cl_Uint32Type Val_ufllimit;
Cl_Uint32Type Val_ufulimit;
Cl_Uint32Type Val_dflowllimit;
Cl_Uint32Type Val_dflowulimit;
Cl_Uint32Type Val_condllimit;
Cl_Uint32Type Val_condulimit;
Cl_Uint32Type Val_aptllimit;
Cl_Uint32Type Val_aptulimit;



}TreatmentParamType;



typedef struct  
{
	Cl_Uint32Type 	Val_dflow;
	Cl_Uint32Type    Val_tmp;
	Cl_Uint32Type 	Val_setcond;
}DilysateParamType;

#endif /* CL_TREATMENT_H_ */