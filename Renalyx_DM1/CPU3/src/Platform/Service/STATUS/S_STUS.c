/*
 * S_STUS.c
 *
 * Created: 12/11/2014 11:08:47 AM
 *  Author: wid7
 */ 
 
#include "S_STUS.h"
#define CAN_MID_MIDvA_dd(value) ((0x1FFC0000 & ((value) << 18)))
extern can_mb_conf_t can0_mailbox;
 uint16_t acid_f_b;
 uint16_t bicarb_f_b;

// uint16_t cpu3_sv_status_getsystemstatus(Sys_statusType* sensor_struct)
// {
// 	
// 	Sys_statusType sensor_struct1;
// 
// 	sensor_struct1.acid_pump_feedback	= acid_count;
// 	sensor_struct1.bicar_pump_feedback  = bicarb_count;
// 	
// 	sensor_struct->acid_pump_feedback	= sensor_struct1.acid_pump_feedback;
// 	sensor_struct->bicar_pump_feedback  = sensor_struct1.bicar_pump_feedback;
// 	
// 	return 0;
// 
// }

bool SV_put_sensor_data(Sys_statusType* sensor_struct)   {
	
	DD_CAN_MAILBOX_SENSOR_DATA_REG_TYPE  SV_CAN_MAILBOX_SENSOR_DATA_REG;
	
	
	SV_CAN_MAILBOX_SENSOR_DATA_REG.CPU_SENDER_type_reg        = PROPORTIONING_CPU_id;
	SV_CAN_MAILBOX_SENSOR_DATA_REG.SENSOR_GROUP_ID_type_reg	  = GROUP_ID_8 ;
	SV_CAN_MAILBOX_SENSOR_DATA_REG.SENSOR_1                   = acid_f_b;
	SV_CAN_MAILBOX_SENSOR_DATA_REG.SENSOR_2	                  = bicarb_f_b;
	SV_CAN_MAILBOX_SENSOR_DATA_REG.SENSOR_3	                  = 0;
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