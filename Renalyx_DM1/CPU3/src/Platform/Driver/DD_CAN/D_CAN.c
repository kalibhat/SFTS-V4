/*
 * D_CAN.c
 *
 * Created: 4/10/2017 11:45:57 AM
 *  Author: Electrical
 */ 

#include "D_CAN.h"

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
#define CAN_MASK 0x7FF
sn65hvd530_ctrl_t  can0_transceiver ;

#define CAN_NORMAL_MODE  false
#define CAN_STANDBY_MODE  true

states state = CAN_IDLE;
readstate read = STATE1;

volatile uint16_t count_start=0;
volatile uint8_t start_c[2];
volatile uint16_t count_stop=0;
volatile uint8_t data_count =0;
volatile uint8_t data[3] = {0,0,0};
volatile bool sv_acc_flag = false;
volatile bool sv_end_flag = false;
volatile bool sv_rxrdy_flag = false;
volatile uint8_t data_buffer[1000];
volatile uint8_t data_buff[3] = {0,0,0};
volatile uint32_t address,address_1;
volatile static bool bool_s = false;
volatile uint8_t flag,flag_buff,flag_buff1,flag1,flag_buff_dummy = 0;
volatile uint8_t test=0;
uint8_t test_data = 170;
uint8_t stabilise_Conductivity =0;
uint16_t User_setConductivity, conductivity =0,avg_conductivity =0, System_Conductivity=0;   

extern uint16_t rc; 
void sn65hvda540_set_en(sn65hvda540_ctrl_t *p_component, uint32_t pin_idx)
{
	p_component->pio_en_idx = pin_idx;
}

void sn65hvda540_normal_mode(sn65hvda540_ctrl_t *p_component)
{
	/* Raise EN pin of SN65HVD234 to High Level (Vcc). */
	ioport_set_pin_level(p_component->pio_en_idx, CAN_NORMAL_MODE);
}

void sn65hvda540_standby_mode(sn65hvda540_ctrl_t *p_component)
{
	/* Raise EN pin of SN65HVD234 to High Level (Vcc). */
	ioport_set_pin_level(p_component->pio_en_idx, CAN_STANDBY_MODE);
}

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



can_mb_conf_t can0_mailbox;
volatile uint32_t g_ul_recv_status = 0;
temp_struct_type  temp_struct;


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



static void reset_mailbox_conf(can_mb_conf_t *p_mailbox)
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


static void dd_can_extract_mailbox_data(can_mb_conf_t *p_mailbox )   {
		 uint32_t status ;	
		 uint32_t ul_reg;
	     uint8_t master_command_byte, master_data_byte;
		 
	
	   temp_struct.temp_reg_32 = (p_mailbox->ul_datal);
	   master_command_byte  =  temp_struct.temp_reg_8[1]  ;
	   master_data_byte     =  temp_struct.temp_reg_8[1]  ;
	   sv_rxrdy_flag = true;
	   switch (state)  {
		   case CAN_IDLE:		
		      data[data_count] = master_command_byte;
		      data_buff[data_count] = data[data_count];		
			  data_buffer[data_count]= data_buff[data_count];
			  
			  Master_cpu_command_type diff;
			  diff = data_buff[0];
		      start_c[0] =diff;
		      switch (diff)	
			  {
				  case PROP_START_ACID_PUMP:
				  			          test =0;
				  			          count_start++;
				  			          flag_buff = 1;
				  			          state = CAN_IDLE;
				  
				  break;
				  
				  case PROP_START_BICARB_PUMP:
				  			          test =0;
				  			          count_start++;
				  			          flag_buff = 1;
				  			          state = CAN_IDLE;
				  
				  break;
			     case PROP_START_ALIGMENT:
			          test =0;
					  count_start++;
			          flag_buff = 1;
			          state = CAN_IDLE;
			    break;
				
			    case PROP_START_MIXING:
				 User_setConductivity = temp_struct.temp_reg_16[1];
// 						switch (temp_struct.temp_reg_16[1])
// 						{
// 							case 135:
// 							System_Conductivity = 147;
// 							break;
// 							
// 							case 136:
// 							System_Conductivity = 152;
// 							break;
// 							
// 							case 137:
// 							System_Conductivity = 159;
// 							break;
// 							
// 							case 138:
// 							System_Conductivity = 162;
// 							break;
// 							
// 							case 139:
// 							System_Conductivity = 164;
// 							break;
// 							
// 							case 140:
// 							System_Conductivity = 170;
// 							break;
// 							
// 							case 141:
// 							System_Conductivity = 174;
// 							break;
// 							
// // 							case 135:
// // 							break;
// // 							
// // 							case 135:
// // 							break;
// // 							
// // 							case 135:
// // 							break;
// 							
// 						}
			       

				    test=0;
					count_start++;					
			        flag_buff = 1;
			        state = CAN_IDLE;
			    break;
				
			    case PROP_STOP_MIXING:
			        test =0;
					count_stop++;
			        flag_buff = 1;
			        state = CAN_IDLE;
			    break;
				
			   /* case PROP_RECOVER:
			       //flag = 1;
			       flag_buff = 1;
			       state = IDLE;
			    break;*/
				
			    case PROP_COMPLETED:
			        //flag = 1;
			          flag_buff = 1;
			          state = CAN_IDLE;
			    break;
				
			    case PROP_RECOVER:
			         //flag = 1;
			         flag_buff = 1;
			         state = CAN_IDLE;
			    break;
			    case PROP_START_OPENFILL:
			        flag_buff=1;
					state= CAN_IDLE;		
			    break;
				
				
			    case PROP_STOP_OPENFILL:
				    flag_buff=1;
				    state= CAN_IDLE;
				
				break;
				
				
				case PROP_START_DISINFECTION:
				   flag_buff=1;
				   state= CAN_IDLE;
				break;
				
				
				case PROP_STOP_DISINFECTION:
				   flag_buff=1;
				   state= CAN_IDLE;
				break;
				
				case PROP_START_CITRIC_DISINFECTION:
				   flag_buff=1;
				   state= CAN_IDLE;
				break;
				
				case PROP_STOP_CITRIC_DISINFECTION:
				flag_buff=1;
				state= CAN_IDLE;
				break;
				
				case PROP_CONDUCTIVITY:
				stabilise_Conductivity ++;
//				avg_conductivity = avg_conductivity + temp_struct.temp_reg_16[1];
				conductivity = avg_conductivity + temp_struct.temp_reg_16[1];
// 				if (stabilise_Conductivity ==6)
// 				{
// 					avg_conductivity = avg_conductivity /6;
// 					stabilise_Conductivity = 0;
// 					conductivity = avg_conductivity/10;
// 				}
				
				state = CAN_IDLE;

				break;
				
				case PROP_UPDATE_ACID_RC:
				rc = temp_struct.temp_reg_16[1];
				break;
				
				case prop_BICAR_FEED_BACK:
				   test =0;
				   count_start++;
				flag_buff=1;
				state= CAN_IDLE;
				break;
				case prop_BICAR_FEED_BACK_STATUS:
				   test =0;
				   count_start++;
				flag_buff=1;
				state= CAN_IDLE;
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
	if (p_mailbox->uc_id_ver) { 
		CAN0->CAN_MB[uc_index].CAN_MID = p_mailbox->ul_id | CAN_MAM_MIDE;
	} else {
		CAN0->CAN_MB[uc_index].CAN_MID = p_mailbox->ul_id;
	}

	/* Write transmit data into mailbox data register. */
	CAN0->CAN_MB[uc_index].CAN_MDL = p_mailbox->ul_datal;
	if (p_mailbox->uc_length > 4) {  CAN0->CAN_MB[uc_index].CAN_MDH = p_mailbox->ul_datah;
	}

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


 void dd_reset_mailbox_conf(can_mb_conf_t *p_mailbox)
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

void DD_INIT_CAN()     {
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
/*	if (CAN_TIMEOUT == ul_tick) { puts("CAN initialization (sync) ERROR" STRING_EOL);	}
	else {	puts("CAN initialization is completed." STRING_EOL);
	}*/
	//NVIC-> ISER reset_mailbox_conf(&can0_mailbox);
	can0_mailbox.ul_mb_idx =   MAILBOX_1;
	can0_mailbox.uc_obj_type = CAN_MB_RX_MODE ;
	can0_mailbox.uc_tx_prio = 0;
	can0_mailbox.uc_id_ver = 0;
	can0_mailbox.ul_id_msk =  CAN_MFID_MFID_VA_dd(CAN_MASK);   // 0x7FC | CAN_MAM_MIDvA_Msk ;
	can0_mailbox.ul_id = CAN_MID_MIDvA_dd(PROPORTIONING_CPU_id) ;
	dd_can_mailbox_conf(&can0_mailbox);
	NVIC_EnableIRQ(CAN0_IRQn);
	CAN0->CAN_IER = CAN_IER_MB1;
	
	can0_mailbox.ul_mb_idx =   MAILBOX_0;
	can0_mailbox.uc_obj_type = CAN_MB_TX_MODE ;
	can0_mailbox.uc_tx_prio = 1;
	can0_mailbox.uc_id_ver = 0;
	can0_mailbox.ul_id_msk = CAN_MFID_MFID_VA_dd(CAN_MASK);// 0x7FC | CAN_MAM_MIDvA_Msk | CAN_MAM_MIDvB_Msk ;
	can0_mailbox.ul_id = CAN_MID_MIDvA_dd(MASTER_CPU_id);
	dd_can_mailbox_conf(&can0_mailbox);	
}






