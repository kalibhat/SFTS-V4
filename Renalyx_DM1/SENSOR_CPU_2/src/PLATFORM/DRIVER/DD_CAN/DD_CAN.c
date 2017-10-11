


/*
 * DD_CAN.c
 *
 * Created: 2/21/2017 11:19:30 AM
 *  Author: Electrical
 */ 

#include "DD_CAN.h"
#include "Platform/Driver/DRIVER_CALLS.h"
#include "Platform/SERVICE/SV_STATUS/SV_STATUS.h"

/** Define the timemark mask. */
#define TIMEMARK_MASK              0x0000ffff

/* CAN timeout for synchronization. */
#define CAN_TIMEOUT                100000

/** The max value for CAN baudrate prescale. */
#define CAN_BAUDRATE_MAX_DIV       128

/** Define the scope for TQ. */
#define CAN_MIN_TQ_NUM             8
#define CAN_MAX_TQ_NUM             25

/** Define the fixed bit time value. */
#define CAN_BIT_SYNC               1
#define CAN_BIT_IPT                2

#define CAN_MID_MIDvA_dd(value) ((0x1FFC0000 & ((value) << 18)))
#define CAN_MFID_MFID_VA_dd(value)  ((0x1FFC0000 & ((value) << 18)))
#define CAN_MCR_MDLC_dd(value) (( (0xfu << 16) & ((value) << 16)))
#define CAN_MASK 0x7FC


sn65hvd530_ctrl_t  can0_transceiver ;
uint8_t volatile group_id_reg;

can_mb_conf_t can0_mailbox;
volatile uint32_t g_ul_recv_status = 0;
volatile bool master_requested_sensor_data = 0;
//DD_Sensor_status_type DD_Sensor_status ;
DD_MAIN_CPU_COMMAND_REG_TYPE DD_CPU_COMMAND ;

const can_bit_timing_t can_bit_time[] = {
	{ 8, (2 + 1), (1 + 1), (1 + 1), (2 + 1), 75},
	{ 9, (1 + 1), (2 + 1), (2 + 1), (1 + 1), 67},
	{10, (2 + 1), (2 + 1), (2 + 1), (2 + 1), 70},
	{11, (3 + 1), (2 + 1), (2 + 1), (3 + 1), 72},
	{12, (2 + 1), (3 + 1), (3 + 1), (3 + 1), 67},
	{13, (3 + 1), (3 + 1), (3 + 1), (3 + 1), 77},
	{14, (3 + 1), (3 + 1), (4 + 1), (3 + 1), 64},
	{15, (3 + 1), (4 + 1), (4 + 1), (3 + 1), 67},
	{16, (4 + 1), (4 + 1), (4 + 1), (3 + 1), 69},
	{17, (5 + 1), (4 + 1), (4 + 1), (3 + 1), 71},
	{18, (4 + 1), (5 + 1), (5 + 1), (3 + 1), 67},
	{19, (5 + 1), (5 + 1), (5 + 1), (3 + 1), 68},
	{20, (6 + 1), (5 + 1), (5 + 1), (3 + 1), 70},
	{21, (7 + 1), (5 + 1), (5 + 1), (3 + 1), 71},
	{22, (6 + 1), (6 + 1), (6 + 1), (3 + 1), 68},
	{23, (7 + 1), (7 + 1), (6 + 1), (3 + 1), 70},
	{24, (6 + 1), (7 + 1), (7 + 1), (3 + 1), 67},
	{25, (7 + 1), (7 + 1), (7 + 1), (3 + 1), 68}
};

void CAN0_Handler(void)   {
	uint32_t ul_status;
	
	
	ul_status = (CAN0 -> CAN_SR);
	if (ul_status & GLOBAL_MAILBOX_MASK) {
		ul_status = CAN0->CAN_MB[1].CAN_MSR ;
		if ((ul_status & CAN_MSR_MRDY) == CAN_MSR_MRDY) {
			can0_mailbox.ul_mb_idx = 1;
			can0_mailbox.ul_status = ul_status;
			dd_can_mailbox_read(&can0_mailbox);
			dd_can_extract_mailbox_data(&can0_mailbox);				
			g_ul_recv_status = 1;
		}
		
	}
}

static void dd_can_extract_mailbox_data(can_mb_conf_t *p_mailbox )   {
	dd_data_size_type dd_data_size;	
	dd_data_size.four_bytes = (p_mailbox->ul_datal);
	
	DD_CPU_COMMAND.CPU_SENDER_reg =  dd_data_size.bytes_array[0]  ;
	DD_CPU_COMMAND.CPU_COMMAND_reg = dd_data_size.bytes_array[1]  ;
	
	switch(DD_CPU_COMMAND.CPU_SENDER_reg)   {		
	   case MASTER_CPU_id :
	       switch(DD_CPU_COMMAND.CPU_COMMAND_reg)    {
		      case Sensor_status_query :
			     master_requested_sensor_data =1;
				 group_id_reg = 0;	
			  break;
			    
		      case Sensor_status_group_id_1: 
					group_id_reg = SENSOR_GROUP_ID_1;					
		            master_requested_sensor_data =1; 
			  break; 
			  case Sensor_status_group_id_2: 
			     group_id_reg =SENSOR_GROUP_ID_2;					
		         master_requested_sensor_data =1; 
			  break;     
			  case Sensor_status_group_id_3: 
				group_id_reg = SENSOR_GROUP_ID_3;					
		        master_requested_sensor_data =1; 
			  break;  
			  case Sensor_status_group_id_4: 
				 group_id_reg = SENSOR_GROUP_ID_4;					
		         master_requested_sensor_data =1; 
			  break;    
			  case Sensor_status_group_id_5: 
			     group_id_reg = SENSOR_GROUP_ID_5;					
		         master_requested_sensor_data =1; 
			  break;   
			  case Sensor_status_group_id_6: 
			     group_id_reg =SENSOR_GROUP_ID_6;					
		         master_requested_sensor_data =1; 
			  break;   
	          			
	          default:
	          break;			
	       }  
	   default:
	   break;
	}	
}


static uint32_t dd_can_mailbox_read(can_mb_conf_t *p_mailbox)   {

	uint32_t ul_status;
	uint8_t uc_index;
	uint32_t ul_retval;

	ul_retval = 0;
	uc_index = (uint8_t)p_mailbox->ul_mb_idx;
	ul_status = p_mailbox->ul_status;
	 /* Check whether there is overwriting happening in Receive with * Overwrite mode,
	 * or there're messages lost in Receive mode. */
	 
	if ((ul_status & CAN_MSR_MRDY) && (ul_status & CAN_MSR_MMI)) {	ul_retval = CAN_MAILBOX_RX_OVER;	}

	/* Read the message family ID. */
	p_mailbox->ul_fid =	CAN0->CAN_MB[uc_index].CAN_MFID & CAN_MFID_MFID_Msk;	

	/* Read received data length. */
	p_mailbox->uc_length = (ul_status & CAN_MSR_MDLC_Msk) >> CAN_MSR_MDLC_Pos;
	
	/* Read received data. */
	p_mailbox->ul_datal = CAN0->CAN_MB[uc_index].CAN_MDL;
	p_mailbox->ul_datah = CAN0->CAN_MB[uc_index].CAN_MDH;
	
	
	/* Read the mailbox status again to check whether the software needs to
	 * re-read mailbox data register. */
	p_mailbox->ul_status = CAN0->CAN_MB[uc_index].CAN_MSR;
	ul_status = p_mailbox->ul_status;
	if (ul_status & CAN_MSR_MMI) {ul_retval |= CAN_MAILBOX_RX_NEED_RD_AGAIN;	}
	else { ul_retval |= CAN_MAILBOX_TRANSFER_OK; }		
		
	/* Enable next receive process. */	 // Allows the reception of the next message
	CAN0->CAN_MB[uc_index].CAN_MCR = CAN_MCR_MTCR |CAN_MCR_MDLC(p_mailbox->uc_length);    
	return ul_retval;
}

		
uint32_t dd_can_mailbox_write(can_mb_conf_t *p_mailbox)
{
	uint32_t ul_status;
	uint8_t uc_index;

	uc_index = (uint8_t)p_mailbox->ul_mb_idx;
	/* Read the mailbox status firstly to check whether the mailbox is ready
	 *or not. */
	p_mailbox->ul_status = CAN0->CAN_MB[uc_index].CAN_MSR;
	ul_status = p_mailbox->ul_status;
	if (!(ul_status & CAN_MSR_MRDY)) {
		return CAN_MAILBOX_NOT_READY;
	}

	/* Write transmit identifier. */
	if (p_mailbox->uc_id_ver) { CAN0->CAN_MB[uc_index].CAN_MID = p_mailbox->ul_id | CAN_MAM_MIDE;	}
                         else {	CAN0->CAN_MB[uc_index].CAN_MID = p_mailbox->ul_id;}

	/* Write transmit data into mailbox data register. */
	CAN0->CAN_MB[uc_index].CAN_MDL = p_mailbox->ul_datal;
	if (p_mailbox->uc_length > 4) {  CAN0->CAN_MB[uc_index].CAN_MDH = p_mailbox->ul_datah;	}

	/* Write transmit data length into mailbox control register. */
	CAN0->CAN_MB[uc_index].CAN_MCR = CAN_MCR_MDLC(p_mailbox->uc_length);

	return CAN_MAILBOX_TRANSFER_OK;
}

void dd_can_global_send_transfer_cmd(uint8_t uc_mask)
{
	uint32_t ul_reg;

	ul_reg = CAN0->CAN_TCR & ((uint32_t) ~GLOBAL_MAILBOX_MASK);
	CAN0->CAN_TCR = ul_reg | uc_mask;
}


 void dd_can_mailbox_conf(can_mb_conf_t *p_mailbox)
{
	uint8_t uc_index;

	uc_index = (uint8_t)p_mailbox->ul_mb_idx;
	/* Check the object type of the mailbox. If it's used to disable the
	 * mailbox, reset the whole mailbox. */
	if (!p_mailbox->uc_obj_type) {
		CAN0->CAN_MB[uc_index].CAN_MMR = 0;
		CAN0->CAN_MB[uc_index].CAN_MAM = 0;
		CAN0->CAN_MB[uc_index].CAN_MID = 0;
		CAN0->CAN_MB[uc_index].CAN_MDL = 0;
		CAN0->CAN_MB[uc_index].CAN_MDH = 0;
		CAN0->CAN_MB[uc_index].CAN_MCR = 0;
		return;
	}
	
	/* Set the priority in Transmit mode. */                                                                   
	CAN0->CAN_MB[uc_index].CAN_MMR = (CAN0->CAN_MB[uc_index].CAN_MMR &  ~CAN_MMR_PRIOR_Msk) | (p_mailbox->uc_tx_prio << CAN_MMR_PRIOR_Pos);  
	/* Set the message ID and message acceptance mask for the mailbox in * other modes. */	
		CAN0 ->CAN_MB[uc_index].CAN_MAM = p_mailbox->ul_id_msk;
		CAN0 ->CAN_MB[uc_index].CAN_MID = p_mailbox->ul_id;	

	/* Set up mailbox in one of the five different modes. */                                                      
	CAN0 ->CAN_MB[uc_index].CAN_MMR = (CAN0 ->CAN_MB[uc_index].CAN_MMR & ~CAN_MMR_MOT_Msk) | (p_mailbox->uc_obj_type << CAN_MMR_MOT_Pos);		
}


static void dd_reset_mailbox_conf(can_mb_conf_t *p_mailbox)
{
	p_mailbox->ul_mb_idx = 0;
	p_mailbox->uc_obj_type = 0;
	p_mailbox->uc_id_ver = 0;
	p_mailbox->uc_length = 0;
	p_mailbox->uc_tx_prio = 0;
	p_mailbox->ul_status = 0;
	p_mailbox->ul_id_msk = 0;
	p_mailbox->ul_id = 0;
	p_mailbox->ul_fid = 0;
	p_mailbox->ul_datal = 0;
	p_mailbox->ul_datah = 0;
}


void SENSOR_DD_INIT_CAN()     {
	can_bit_timing_t  *p_bit_time ;	
	p_bit_time = (can_bit_timing_t *)&can_bit_time[6];
	uint32_t ul_flag;
	uint32_t ul_tick;
	uint8_t uc_prescale = 120;	
	uint32_t ul_sysclk;	
	CAN0 ->CAN_WPMR = 0x43414E00;  //Can write protection disabled
	CAN0 ->CAN_MR &= ~CAN_MR_CANEN ;  //0xFFFFFFFE; // DISABLE CAN0	
	CAN0->CAN_BR = CAN_BR_PHASE2(p_bit_time->uc_phase2 - 1) |CAN_BR_PHASE1(p_bit_time->uc_phase1 - 1) | CAN_BR_PROPAG(p_bit_time->uc_prog - 1) | CAN_BR_SJW(p_bit_time->uc_sjw - 1) |	CAN_BR_BRP(uc_prescale - 1);
	//CAN0 ->CAN_BR   = 0x01783334 ;   //50k for 84000000, uc_prog = 4, uc_phase1 = 4, uc_phase2 = 5, uc_sjw = 4, prescale = 120, smp =1 	
	
	can_mb_conf_t mb_config_t;
	
	//RESET ALL MAIL BOX 	
	//can_bit_timing_t *p_bit_time;
	/* Set the mailbox object type parameter to disable the mailbox. */	
	mb_config_t.uc_obj_type = CAN_MB_DISABLE_MODE;
	for (uint8_t i = 0; i < CANMB_NUMBER; i++) {
		mb_config_t.ul_mb_idx = i;
		dd_can_mailbox_conf(&mb_config_t);
	}
	// ALL MAIL BOX HAS BEEN RESET		
	CAN0->CAN_MR |= CAN_MR_CANEN ;  //0x00000001; // CAN IS ENABLED
	
	/* Wait until the CAN is synchronized with the bus activity. */
	ul_flag = 0;
	ul_tick = 0;
	while  (!(ul_flag & CAN_SR_WAKEUP) && (ul_tick < CAN_TIMEOUT))    {//(!(ul_flag & 0x100000 ) && (ul_tick < CAN_TIMEOUT)) {  //check can_bus wakeup bit
		ul_flag = (CAN0->CAN_SR);		
		ul_tick++;
	}

	/* Timeout or the CAN module has been synchronized with the bus. */
	if (CAN_TIMEOUT == ul_tick) { puts("CAN initialization (sync) ERROR" STRING_EOL);	} 
	else {	puts("CAN initialization is completed." STRING_EOL);
	}	
	//NVIC-> ISER reset_mailbox_conf(&can0_mailbox);
	can0_mailbox.ul_mb_idx =   MAILBOX_1;
	can0_mailbox.uc_obj_type = CAN_MB_RX_MODE ;
	can0_mailbox.uc_tx_prio = 0;
	can0_mailbox.uc_id_ver = 0;
	can0_mailbox.ul_id_msk =  CAN_MFID_MFID_VA_dd(CAN_MASK);
	can0_mailbox.ul_id = CAN_MID_MIDvA_dd(SENSOR_CPU_id) ;
	dd_can_mailbox_conf(&can0_mailbox);
	NVIC_EnableIRQ(CAN0_IRQn);
	CAN0->CAN_IER = CAN_IER_MB1;
	
	can0_mailbox.ul_mb_idx =   MAILBOX_0;
	can0_mailbox.uc_obj_type = CAN_MB_TX_MODE ;
	can0_mailbox.uc_tx_prio = 1;
	can0_mailbox.uc_id_ver = 0;
	can0_mailbox.ul_id_msk =CAN_MFID_MFID_VA_dd(CAN_MASK) ;
	can0_mailbox.ul_id = CAN_MID_MIDvA_dd(MASTER_CPU_id);
	dd_can_mailbox_conf(&can0_mailbox);	
	
	dd_CLEAR_OUTPUT_DATA(PIOA, CAN_STB_PIN);
	
}
