/*
 * STATUS.c
 *
 * Created: 23/5/2017 2:15:32 PM
 *  Author: Gopalkrishna Kalibhat
 */ 

#include "STATUS.h"
#define CAN_MID_MIDvA_dd(value) ((0x1FFC0000 & ((value) << 18)))
extern can_mb_conf_t can0_mailbox;
extern uint16_t fedbck_cnt1;
uint16_t cpu2_sv_status_getsystemstatus(Sys_statusType* sensor_struct)
{
	
	 Sys_statusType sensor_struct1;

	sensor_struct1.vptstatus			= CPU2_D_READ_SENSORS(SV_VPT);
	sensor_struct1.aptstatus			= CPU2_D_READ_SENSORS(SV_APT);
// 	sensor_struct1.Tempstatus			= CPU2_D_READ_SENSORS(SV_SENSOR_ID3);
// 	sensor_struct1.cond1_status			= CPU2_D_READ_SENSORS(SV_SENSOR_ID4);
// 	sensor_struct1.cond2_status			= CPU2_D_READ_SENSORS(SV_SENSOR_ID5);
	
	sensor_struct1.bp_door			= CPU2_D_RD_SENSIN(SV_BP_DOOR);
	sensor_struct1.bp_sense			= CPU2_D_RD_SENSIN(SV_BP_FB);
	sensor_struct1.clamp_fdbk		= CPU2_D_RD_SENSIN(SV_CLAMP_FB);
	sensor_struct1.clamp_door			= CPU2_D_RD_SENSIN(SV_CLAMP_DOOR);
	sensor_struct1.bdstatus				= CPU2_D_RD_SENSIN(SV_BD);
	sensor_struct1.abdstatus			= CPU2_D_RD_SENSIN(SV_ABD);
	sensor_struct1.rudncy_irq			= CPU2_D_RD_SENSIN(SV_REDUNDANCY_IRQ);
	
	
// 	sensor_struct->cond1_status			= sensor_struct1.cond1_status;
// 	sensor_struct->cond2_status			= sensor_struct1.cond2_status;
// 	sensor_struct->Tempstatus			= sensor_struct1.Tempstatus;
	sensor_struct->aptstatus			= sensor_struct1.aptstatus;
	sensor_struct->vptstatus			= sensor_struct1.vptstatus;
	
	
	sensor_struct->bdstatus				= sensor_struct1.bdstatus;
	sensor_struct->abdstatus			= sensor_struct1.abdstatus;
	sensor_struct->bp_door				= sensor_struct1.bp_door;
	sensor_struct->bp_sense				= sensor_struct1.bp_sense;
	sensor_struct->clamp_fdbk			= sensor_struct1.clamp_fdbk;
	sensor_struct->clamp_door			= sensor_struct1.clamp_door;
	sensor_struct->rudncy_irq			= sensor_struct1.rudncy_irq;
	
	return 0;

}

bool SV_put_sensor_data(Sys_statusType* sensor_struct)   {
	
 DD_CAN_MAILBOX_SENSOR_DATA_REG_TYPE  SV_CAN_MAILBOX_SENSOR_DATA_REG;
	
	
	SV_CAN_MAILBOX_SENSOR_DATA_REG.CPU_SENDER_type_reg        = HEMO_CPU_id ;
	SV_CAN_MAILBOX_SENSOR_DATA_REG.SENSOR_GROUP_ID_type_reg	  = GROUP_ID_7 ;
	SV_CAN_MAILBOX_SENSOR_DATA_REG.SENSOR_1                   = sensor_struct->aptstatus;
	SV_CAN_MAILBOX_SENSOR_DATA_REG.SENSOR_2	                  = sensor_struct->vptstatus;
	SV_CAN_MAILBOX_SENSOR_DATA_REG.SENSOR_3	                  = fedbck_cnt1;
	SV_SEND_CAN_MAILBOX(&SV_CAN_MAILBOX_SENSOR_DATA_REG);
	
	return ;
}


void SV_SEND_CAN_MAILBOX(DD_CAN_MAILBOX_SENSOR_DATA_REG_TYPE*  SV_CAN_MAILBOX_SENSOR_DATA_STRUCTURE)   {
	sv_data_size_type  sv_data_size;
	sv_data_size.bytearray[0]                                =SV_CAN_MAILBOX_SENSOR_DATA_STRUCTURE->CPU_SENDER_type_reg;
	sv_data_size.bytearray[1]                                =SV_CAN_MAILBOX_SENSOR_DATA_STRUCTURE->SENSOR_GROUP_ID_type_reg;
	
	sv_data_size.Twobyte[1]                                  =SV_CAN_MAILBOX_SENSOR_DATA_STRUCTURE->SENSOR_1;
	sv_data_size.Twobyte[2]                                  =SV_CAN_MAILBOX_SENSOR_DATA_STRUCTURE->SENSOR_2;
	sv_data_size.Twobyte[3]                                  =SV_CAN_MAILBOX_SENSOR_DATA_STRUCTURE->SENSOR_3;
	
	can0_mailbox.ul_datal                                    =sv_data_size.fourbyte[0];
	can0_mailbox.ul_datah                                    =sv_data_size.fourbyte[1];
	can0_mailbox.uc_length                                  = 8;
	can0_mailbox.ul_mb_idx =   MAILBOX_0;
	can0_mailbox.uc_obj_type = CAN_MB_TX_MODE ;
	can0_mailbox.uc_tx_prio = 1;
	can0_mailbox.uc_id_ver = 0;
	can0_mailbox.ul_id_msk = 0x7FC ;
	can0_mailbox.ul_id = CAN_MID_MIDvA_dd(MASTER_CPU_id); ;
	while (CAN_MAILBOX_NOT_READY == dd_can_mailbox_write(&can0_mailbox));	
	//dd_can_mailbox_write(&can0_mailbox);
	dd_can_global_send_transfer_cmd(CAN_TCR_MB0);
}