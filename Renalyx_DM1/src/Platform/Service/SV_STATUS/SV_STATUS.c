/*
 * SV_STATUS.c
 *
 * Created: 1/6/2014 10:35:41 AM
 *  Author: Geekay
 */ 

#include "SV_STATUS.h"





uint8_t sv_status_getlevelswitchstatus()			// Level Sensor
{
	uint8_t lvl = DD_RD_SENSIN(SV_SENSIN_ID1);
	return lvl;
}

uint8_t sv_status_getFPcurrentsense()		// Flow Pump Current Sensor
{
	uint8_t fp = DD_RD_SENSIN(SV_SENSIN_ID2);
	return fp;
}

uint8_t sv_status_getDGPcurrentsense()			// Degaussing Pump Current Sensor
{
	uint8_t dg = DD_RD_SENSIN(SV_SENSIN_ID3);
	return dg;
}

uint8_t sv_status_getUFPstatus()					// UF pump sensor
{
	uint8_t uf = DD_RD_SENSIN(SV_SENSIN_ID4);
	return uf;
}

uint8_t sv_status_getBPstatus()					// BP
{
	uint8_t bp = DD_RD_SENSIN(SV_SENSIN_ID5);
	return bp;
}

uint8_t sv_status_getbldstatus()
{
	uint8_t bld = DD_RD_SENSIN(SV_SENSIN_ID6);
	return bld;
}
 
uint8_t sv_status_getbdstatus()
{
	uint8_t bd = DD_RD_SENSIN(SV_SENSIN_ID7);
	return bd;
}

uint8_t sv_status_abdstatus()
{
	uint8_t abd = DD_RD_SENSIN(SV_SENSIN_ID8);
	return abd;
}

uint8_t sv_status_getdholderstatus()
{
	uint8_t hld = DD_RD_SENSIN(SV_SENSIN_ID10);
	hld   = ((hld << 1) | DD_RD_SENSIN(SV_SENSIN_ID9));
	return hld;
}

uint8_t sv_status_vcstatus()
{
	uint8_t V_C = DD_RD_SENSIN(SV_SENSIN_ID17);
	return V_C;
}


uint16_t sv_status_getsystemstatus(Sys_statusType* sensor_struct)
{
	//sv_sensortype sv_sensor = SV_SENSOR_NULL;
	
	Sys_statusType sensor_struct1;
	sensor_struct1.levelswitch1	= 0;
	sensor_struct1.FPcurrentstatus	= 1;
	sensor_struct1.DGPstatus = 1;
	//sv_sensor = SV_SENSOR_ID1;
	
	sensor_struct1.ps1status			= DD_CAN_READ_SENSORS(DD_CAN_SENSOR_ID1);
	sensor_struct1.ps2status			= DD_CAN_READ_SENSORS(DD_CAN_SENSOR_ID2);
	sensor_struct1.ps3status			= DD_CAN_READ_SENSORS(DD_CAN_SENSOR_ID3);

	
	sensor_struct1.Temp1status			= DD_CAN_READ_SENSORS(DD_CAN_SENSOR_ID5);
	sensor_struct1.Temp2status			= DD_CAN_READ_SENSORS(DD_CAN_SENSOR_ID6);
	sensor_struct1.Temp3status			= DD_CAN_READ_SENSORS(DD_CAN_SENSOR_ID7);
	sensor_struct1.CS1_Tempstatus		= DD_CAN_READ_SENSORS(DD_CAN_SENSOR_ID8);
	sensor_struct1.CS2_Tempstatus		= DD_CAN_READ_SENSORS(DD_CAN_SENSOR_ID9);
	sensor_struct1.CS3_Tempstatus		= DD_CAN_READ_SENSORS(DD_CAN_SENSOR_ID10);
	
	
//	sensor_struct1.Temp4status			= DD_READ_SENSORS(SV_SENSOR_ID13);
	
	sensor_struct1.cond_status			= DD_CAN_READ_SENSORS(DD_CAN_SENSOR_ID13);
	sensor_struct1.cond_status_dac		= DD_CAN_READ_SENSORS(DD_CAN_SENSOR_ID14);
	sensor_struct1.cond_acid			= DD_CAN_READ_SENSORS(DD_CAN_SENSOR_ID11);
	sensor_struct1.cond_bicarb			= DD_CAN_READ_SENSORS(DD_CAN_SENSOR_ID12);  // Gopal: getting CS2 in place of DAB  //DD_CAN_READ_SENSORS(DD_CAN_SENSOR_ID12);
//	sensor_struct1.cond_wien_status		= DD_READ_SENSORS(SV_SENSOR_ID12);
	
	sensor_struct1.aptstatus			= DD_CAN_READ_SENSORS(DD_CAN_SENSOR_ID19);
	sensor_struct1.vptstatus			= DD_CAN_READ_SENSORS(DD_CAN_SENSOR_ID20);
	
	sensor_struct1.acid_fd_back         = DD_CAN_READ_SENSORS(DD_CAN_SENSOR_ID21);
	sensor_struct1.bicarb_fd_back       = DD_CAN_READ_SENSORS(DD_CAN_SENSOR_ID22);
	sensor_struct1.bloodpumpdoor		= DD_RD_SENSIN(SV_SENSIN_ID14);
	sensor_struct1.BPstatus				= DD_RD_SENSIN(SV_SENSIN_ID5);
	sensor_struct1.BP_INC				= DD_RD_SENSIN(SV_SENSIN_ID15);
	sensor_struct1.BP_DEC				= DD_RD_SENSIN(SV_SENSIN_ID16);
	sensor_struct1.Heparin_rotation_marker = DD_RD_SENSIN(SV_SENSIN_ID13);
	sensor_struct1.Heparin_full_marker	= DD_RD_SENSIN(SV_SENSIN_ID6);
	sensor_struct1.Heparin_empty_marker = DD_RD_SENSIN(SV_SENSIN_ID7);

	
	sensor_struct1.VC_status			= DD_RD_SENSIN(SV_SENSIN_ID17);
	//sensor_struct1.VC_door_status		= DD_READ_SENSORS(SV_SENSIN_ID18);

	sensor_struct1.bdstatus				= DD_RD_SENSIN(SV_SENSIN_ID26);
	sensor_struct1.abdstatus			= DD_RD_SENSIN(SV_SENSIN_ID8);
	sensor_struct1.FPcurrentstatus		= DD_RD_SENSIN(SV_SENSIN_ID3);
	sensor_struct1.UFPstatus			= DD_RD_SENSIN(SV_SENSIN_ID4);
	
	sensor_struct1.levelswitch1			= DD_RD_SENSIN(SV_SENSIN_ID1);
	sensor_struct1.Flowstatus			= DD_RD_SENSIN(SV_SENSIN_ID12);
	sensor_struct1.bldstatus			= DD_RD_SENSIN(SV_SENSIN_ID27);
	
	sensor_struct1.holderstatus1		= DD_RD_SENSIN(SV_SENSIN_ID9);
	sensor_struct1.holderstatus2		= DD_RD_SENSIN(SV_SENSIN_ID10);
	sensor_struct1.calibration			= DD_RD_SENSIN(SV_SENSIN_ID19);
	
	sensor_struct1.bicarb_inlet_status	=  DD_RD_SENSIN(SV_SENSIN_ID21);   // manav03 20
	sensor_struct1.acid_inlet_status	=  DD_RD_SENSIN(SV_SENSIN_ID20);   // manav03 21
	sensor_struct1.CPU2_interupt		=  DD_RD_SENSIN(SV_SENSIN_ID22);
	sensor_struct1.CPU3_interrupt		=  DD_RD_SENSIN(SV_SENSIN_ID23);
	sensor_struct1.Redundancy_IRQ		=  DD_RD_SENSIN(SV_SENSIN_ID24);
		
	sensor_struct1.DGPstatus		= DD_RD_SENSIN(SV_SENSIN_ID2);
	sensor_struct1.PowerFailure		= DD_RD_SENSIN(SV_SENSIN_ID36);
	
	
	sensor_struct1.BP_ON_OFF = DD_RD_SENSIN(SV_SENSIN_ID25);
	
	sensor_struct->ps1status			= sensor_struct1.ps1status;
	sensor_struct->ps2status			= sensor_struct1.ps2status;
	sensor_struct->ps3status			= sensor_struct1.ps3status;
	sensor_struct->cond_status			= sensor_struct1.cond_status;
	sensor_struct->cond_status_dac		= sensor_struct1.cond_status_dac;
	sensor_struct->cond_wien_status		= sensor_struct1.cond_wien_status;
	sensor_struct->cond_acid			= sensor_struct1.cond_acid;
	sensor_struct->cond_bicarb			= sensor_struct1.cond_bicarb;
	sensor_struct->Temp1status			= sensor_struct1.Temp1status;
	sensor_struct->Temp2status			= sensor_struct1.Temp2status;
	sensor_struct->Temp3status			= sensor_struct1.Temp3status;
	sensor_struct->aptstatus			= sensor_struct1.aptstatus;
	sensor_struct->vptstatus			= sensor_struct1.vptstatus;
	sensor_struct->acid_fd_back			= sensor_struct1.acid_fd_back;
	sensor_struct->bicarb_fd_back       = sensor_struct1.bicarb_fd_back; 
		
	sensor_struct->levelswitch1			= sensor_struct1.levelswitch1;
	sensor_struct->FPcurrentstatus		= sensor_struct1.FPcurrentstatus;
	sensor_struct->DGPstatus		= sensor_struct1.DGPstatus;
	sensor_struct->UFPstatus			= sensor_struct1.UFPstatus;
	sensor_struct->BPstatus				= sensor_struct1.BPstatus;
	sensor_struct->bldstatus			= sensor_struct1.bldstatus;
	sensor_struct->bdstatus				= sensor_struct1.bdstatus;
	sensor_struct->abdstatus			= sensor_struct1.abdstatus;
	sensor_struct->holderstatus1		= sensor_struct1.holderstatus1;
	sensor_struct->holderstatus2		= sensor_struct1.holderstatus2;
	sensor_struct->bloodpumpdoor		= sensor_struct1.bloodpumpdoor;
	sensor_struct->Flowstatus			= sensor_struct1.Flowstatus;
	sensor_struct->Heparin_full_marker  = sensor_struct1.Heparin_full_marker;
	sensor_struct->Heparin_empty_marker = sensor_struct1.Heparin_empty_marker;
	sensor_struct->Heparin_rotation_marker = sensor_struct1.Heparin_rotation_marker;
	sensor_struct->acid_inlet_status = sensor_struct1.acid_inlet_status;
	sensor_struct->bicarb_inlet_status = sensor_struct1.bicarb_inlet_status;
	sensor_struct->VC_status = sensor_struct1.VC_status;
	sensor_struct->BP_DEC =sensor_struct1.BP_DEC;
	sensor_struct->BP_INC = sensor_struct1. BP_INC; 
	sensor_struct->BP_ON_OFF = sensor_struct1. BP_ON_OFF; 
	sensor_struct->PowerFailure = sensor_struct1.PowerFailure; 
	
	return 0;

}



