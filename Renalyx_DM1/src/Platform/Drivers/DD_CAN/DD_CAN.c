
/*
 * DD_CAN.c
 *
 * Created: 2/21/2017 11:19:30 AM
 *  Author: Gopalkrishna Kalibhat
 */ 

#include "DD_CAN.h"
#include "Platform/Drivers/DD_GPIO/DD_GPIO.h"

#define PIN_CAN0_STANDBY_IDX       (PIO_PA3_IDX )

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
//#define CAN_MASK 0x7FC
#define CAN_MASK 0x7FF
sn65hvd530_ctrl_t  can0_transceiver ;

#define CAN_NORMAL_MODE  false
#define CAN_STANDBY_MODE  true
extern  volatile uint8_t array1[];
extern	volatile  uint32_t delay_cnt ;
extern  volatile uint32_t delay ;
extern volatile uint32_t g_ul_ms_ticks ;
uint16_t BPCanFeedBackValue = 0;
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
uint32_t can_flag=0;
volatile DD_Sensor_status_type DD_Sensor_status ;
volatile temp_struct_type temp_struct;
//can_structure_id_type can_structure_id;

void CAN0_Handler(void)   {
	uint32_t ul_status;
	ul_status = (CAN0 -> CAN_SR);
	if (ul_status & GLOBAL_MAILBOX_MASK) {
		ul_status = CAN0->CAN_MB[1].CAN_MSR ;
		if ((ul_status & CAN_MSR_MRDY) == CAN_MSR_MRDY) {
			can0_mailbox.ul_mb_idx = 1;
			can0_mailbox.ul_status = ul_status;
			dd_can_mailbox_read(&can0_mailbox);
			dd_can_dump_mailbox_data(&can0_mailbox);
			g_ul_recv_status = 1;
			can_flag = g_ul_recv_status;
		}
		
	}
}

 uint16_t dd_master_command(dd_cpu_id_type  cpu_id, MASTER_CMD_ID_TYPE command_id, uint16_t speed ) 
   {
	 dd_data_size_type dd_data_size;
	dd_data_size.bytes_array[0] =  MASTER_CPU_id;         //senders id;
	dd_data_size.bytes_array[1] =  command_id;
	can0_mailbox.ul_id = CAN_MID_MIDvA_dd(cpu_id); // receivers id
//	dd_data_size.bytes_array[2] =0;
	
//	dd_data_size.bytes_array[3] =0;
    dd_data_size.two_bytes[1] = speed;
	can0_mailbox.ul_datal =  dd_data_size.four_bytes;
	can0_mailbox.ul_datah =0;
	
	CAN0->CAN_IDR = 0xffffffff;
	NVIC_EnableIRQ(CAN0_IRQn);	
    can0_mailbox.ul_mb_idx =   MAILBOX_0;  // transmit for master
    can0_mailbox.uc_obj_type = CAN_MB_TX_MODE ;  // message type , transmitting or receiving
    can0_mailbox.uc_tx_prio = 0;                   // Message Priority
	can0_mailbox.uc_length = 4;                    // Can Message Length Header+data
    can0_mailbox.uc_id_ver = 0;                   // Can standard Version
    can0_mailbox.ul_id_msk = 0x7fc;			      // 
	uint32_t ul_status;	
	uint32_t ul_reg;
	uint32_t can_retun_cal ;
	while (CAN_MAILBOX_NOT_READY == dd_can_mailbox_write(&can0_mailbox));	
	
	NVIC_EnableIRQ(CAN0_IRQn);
	CAN0->CAN_IER = CAN_IER_MB1;
	
	return CAN_MAILBOX_TRANSFER_OK;
}


static void dd_can_dump_mailbox_data(can_mb_conf_t *p_mailbox )   {
	temp_struct.temp_reg_32 = (p_mailbox->ul_datal);
	DD_Sensor_status.CPU_SENDER_type_reg =  temp_struct.temp_reg_8[0]  ;
	DD_Sensor_status.SENSOR_GROUP_type_reg = temp_struct.temp_reg_8[1]  ;
	
	switch(DD_Sensor_status.CPU_SENDER_type_reg)   {
		
		case HEMO_CPU_id :
		
		switch(DD_Sensor_status.SENSOR_GROUP_type_reg)   {
			
			case Sensor_group_7 :
			DD_Sensor_status.aptstatus = temp_struct.temp_reg_16[1] ;		
			temp_struct.temp_reg_32 = (p_mailbox->ul_datah);			
			DD_Sensor_status.vptstatus  =  temp_struct.temp_reg_16[0] ;
			BPCanFeedBackValue =   temp_struct.temp_reg_16[1] ;
			break;
			
			default :
			break;
		}
		break;
		
		case PROPORTIONING_CPU_id :
				switch(DD_Sensor_status.SENSOR_GROUP_type_reg)   {
					
					case Sensor_group_8 :
					DD_Sensor_status.acid_feed_back = temp_struct.temp_reg_16[1] ;
					temp_struct.temp_reg_32 = (p_mailbox->ul_datah);
					DD_Sensor_status.bicar_feed_back  =  temp_struct.temp_reg_16[0] ;
					DD_Sensor_status.dummy_data =   temp_struct.temp_reg_16[1] ;
					break;
					
					default :
					break;
				}
				break;
		
		case SENSOR_CPU_id:
		
		switch(DD_Sensor_status.SENSOR_GROUP_type_reg)   {
			
			case Sensor_group_1 :													// Pressure Sensors 1,2 and 3
			DD_Sensor_status.ps1status = temp_struct.temp_reg_16[1] ;			
			temp_struct.temp_reg_32 = (p_mailbox->ul_datah);			
			DD_Sensor_status.ps2status  =  temp_struct.temp_reg_16[0] ;
			DD_Sensor_status.ps3status =   temp_struct.temp_reg_16[1] ;		
			  array1[1]	= 1;
			delay = g_ul_ms_ticks -  delay_cnt ;
			break;
			
			case Sensor_group_2 :													// Thermocouple, Temp1 and Temp2
			DD_Sensor_status.thermocouple_status = temp_struct.temp_reg_16[1] ;			
			temp_struct.temp_reg_32 = (p_mailbox->ul_datah);			
			DD_Sensor_status.Temp1status =  temp_struct.temp_reg_16[0] ;
			DD_Sensor_status.Temp2status=   temp_struct.temp_reg_16[1] ;	
			array1[2]	= 1;		
			break;
			
			case Sensor_group_3 :													// Temp3, CS1-Temp and CS2-Temp
			DD_Sensor_status.CS1_Tempstatus = temp_struct.temp_reg_16[1];			
			temp_struct.temp_reg_32 = (p_mailbox->ul_datah);			
			DD_Sensor_status.CS2_Tempstatus =  temp_struct.temp_reg_16[0] ;
			DD_Sensor_status.Temp3status=   temp_struct.temp_reg_16[1] ;			
			array1[3]	= 1;
			break;
			
			case Sensor_group_4 :													// CS3- Temp, CS1 and CS2
			DD_Sensor_status.CS3_Tempstatus = temp_struct.temp_reg_16[1];			
			temp_struct.temp_reg_32 = (p_mailbox->ul_datah);			
			DD_Sensor_status.CS1status =  temp_struct.temp_reg_16[0] ;
			DD_Sensor_status.CS2status=   temp_struct.temp_reg_16[1] ;	
			array1[4]	= 1;		
			break;
			
			case Sensor_group_5 :													// CS3, DAC1 and DAA
			DD_Sensor_status.CS3status = temp_struct.temp_reg_16[1];			
			temp_struct.temp_reg_32 = (p_mailbox->ul_datah);			
			DD_Sensor_status.DAC1status =  temp_struct.temp_reg_16[0] ;
			DD_Sensor_status.DAAstatus=   temp_struct.temp_reg_16[1] ;		
			array1[5]	= 1;	
			break;
			
			case Sensor_group_6 :													// DAB, DAC2 and flow Sensor
			DD_Sensor_status.DABstatus = temp_struct.temp_reg_16[1];			
			temp_struct.temp_reg_32 = (p_mailbox->ul_datah);			
			DD_Sensor_status.DAC2status =  temp_struct.temp_reg_16[0] ;
			DD_Sensor_status.flow_sensor_status=   temp_struct.temp_reg_16[1] ;	
			array1[6]	= 1;	
			
			break;
			
			default :
			break;
		} //switch(DD_Sensor_status.SENSOR_GROUP_type_reg)  ends here
		
		break;
		
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

		
static uint32_t dd_can_mailbox_write(can_mb_conf_t *p_mailbox)
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



static void dd_can_mailbox_conf(can_mb_conf_t *p_mailbox)
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

void DD_CAN_WRITE_PROPO(uint16_t data, uint16_t rc_count)
{
//     // sn65hvda540_normal_mode(&can0_transceiver);
// //	 DD_RESET_PIO(PA,3);
// 	 can0_mailbox.ul_mb_idx = MAILBOX_0;
// 	 can0_mailbox.ul_id = CAN_MID_MIDvA(MASTER_CPU_id);
// 	 can0_mailbox.ul_datal = 0x00;
// 	 can0_mailbox.ul_datah = 0x55AAAA55;
// 	 can0_mailbox.uc_length = 8;
// 	 dd_can_mailbox_write(&can0_mailbox);
// 	 dd_can_global_send_transfer_cmd(CAN_TCR_MB0);
MASTER_CMD_ID_TYPE dd_propo_cmd;
dd_propo_cmd = data;
dd_master_command(PROPORTIONING_CPU_id, dd_propo_cmd,rc_count);
dd_can_global_send_transfer_cmd(CAN_TCR_MB0);

}

void DD_CAN_WRITE_HEMO(uint16_t data, uint16_t speed)
{
	//     // sn65hvda540_normal_mode(&can0_transceiver);
	// //	 DD_RESET_PIO(PA,3);
	// 	 can0_mailbox.ul_mb_idx = MAILBOX_0;
	// 	 can0_mailbox.ul_id = CAN_MID_MIDvA(MASTER_CPU_id);
	// 	 can0_mailbox.ul_datal = 0x00;
	// 	 can0_mailbox.ul_datah = 0x55AAAA55;
	// 	 can0_mailbox.uc_length = 8;
	// 	 dd_can_mailbox_write(&can0_mailbox);
	// 	 dd_can_global_send_transfer_cmd(CAN_TCR_MB0);
	MASTER_CMD_ID_TYPE dd_hemo_cmd;
	dd_hemo_cmd = data;
	CAN0->CAN_IDR = 0xffffffff;
	dd_master_command(HEMO_CPU_id, dd_hemo_cmd, speed);
	dd_can_global_send_transfer_cmd(CAN_TCR_MB0);
	CAN0->CAN_IER = CAN_IER_MB1;
}

void DD_CAN_WRITE_SENSOR(uint16_t data)
{
	//     // sn65hvda540_normal_mode(&can0_transceiver);
	// //	 DD_RESET_PIO(PA,3);
	// 	 can0_mailbox.ul_mb_idx = MAILBOX_0;
	// 	 can0_mailbox.ul_id = CAN_MID_MIDvA(MASTER_CPU_id);
	// 	 can0_mailbox.ul_datal = 0x00;
	// 	 can0_mailbox.ul_datah = 0x55AAAA55;
	// 	 can0_mailbox.uc_length = 8;
	// 	 dd_can_mailbox_write(&can0_mailbox);
	// 	 dd_can_global_send_transfer_cmd(CAN_TCR_MB0);
	MASTER_CMD_ID_TYPE dd_sensor_cmd;
	dd_sensor_cmd = data;
	CAN0->CAN_IDR = 0xffffffff;
	dd_master_command(SENSOR_CPU_id, dd_sensor_cmd, 0);
	dd_can_global_send_transfer_cmd(CAN_TCR_MB0);
	CAN0->CAN_IER = CAN_IER_MB1;
}


void DD_CAN_READ_APTVPT(void)
{
	
	DD_CAN_WRITE_HEMO(BP_APT_VPT_SEND_TO_MASTER, 0);
	
}

void DD_CAN_READ_PROPO_FB(void)
{
	
	DD_CAN_WRITE_PROPO(prop_BICAR_FEED_BACK_STATUS, 0);
	
}




void DD_INIT_CAN()    

	{
				
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
	can0_mailbox.ul_id = CAN_MID_MIDvA_dd(MASTER_CPU_id) ;
	dd_can_mailbox_conf(&can0_mailbox);	
	
	can0_mailbox.ul_mb_idx =   MAILBOX_0;
	can0_mailbox.uc_obj_type = CAN_MB_TX_MODE ;
	can0_mailbox.uc_tx_prio = 1;
	can0_mailbox.uc_id_ver = 0;
	can0_mailbox.ul_id_msk = CAN_MFID_MFID_VA_dd(CAN_MASK);// 0x7FC | CAN_MAM_MIDvA_Msk | CAN_MAM_MIDvB_Msk ;
	can0_mailbox.ul_id = CAN_MID_MIDvA_dd(PROPORTIONING_CPU_id);
	dd_can_mailbox_conf(&can0_mailbox);	
		
		NVIC_EnableIRQ(CAN0_IRQn);
		CAN0->CAN_IER = CAN_IER_MB1;
// 					
	
	sn65hvda540_set_en(&can0_transceiver ,PIN_CAN0_STANDBY_IDX);
	sn65hvda540_normal_mode(&can0_transceiver);
		
		
	}
	
	
	

uint16_t DD_CAN_READ_SENSORS(DD_sensortype ID_SENSOR)
{
	uint16_t SENSOR_VAL;
	uint8_t channel = 0;
	//adc_chnl channel = channel_0;
	switch(ID_SENSOR)
	{
		case DD_CAN_SENSOR_ID1:
		
		SENSOR_VAL = DD_Sensor_status.ps1status;
		break;
		
		case DD_CAN_SENSOR_ID2:
		SENSOR_VAL = DD_Sensor_status.ps2status;
		break;
		
		case DD_CAN_SENSOR_ID3:
		SENSOR_VAL = DD_Sensor_status.ps3status;
		break;
		
		case DD_CAN_SENSOR_ID4:
		SENSOR_VAL = DD_Sensor_status.thermocouple_status;
		break;
		
		case DD_CAN_SENSOR_ID5:
		SENSOR_VAL = DD_Sensor_status.Temp1status;
		
		break;
		
		case DD_CAN_SENSOR_ID6:
		SENSOR_VAL = DD_Sensor_status.Temp2status;
		break;
		
		case DD_CAN_SENSOR_ID7:
		SENSOR_VAL = DD_Sensor_status.Temp3status;
		break;
		
		case DD_CAN_SENSOR_ID8:
		SENSOR_VAL = DD_Sensor_status.CS1_Tempstatus;
		break;
		
		case DD_CAN_SENSOR_ID9:
		SENSOR_VAL = DD_Sensor_status.CS2_Tempstatus;
		break;
		
		case DD_CAN_SENSOR_ID10:
		SENSOR_VAL = DD_Sensor_status.CS3_Tempstatus;
		break;
		
		case DD_CAN_SENSOR_ID11:
		SENSOR_VAL = DD_Sensor_status.CS1status;
		break;
		
		case DD_CAN_SENSOR_ID12:
		SENSOR_VAL = DD_Sensor_status.CS2status;
		break;
		
		case DD_CAN_SENSOR_ID13:
		SENSOR_VAL = DD_Sensor_status.CS3status;
		break;
		
		case DD_CAN_SENSOR_ID14:
		SENSOR_VAL = DD_Sensor_status.DAC1status;
		break;
		
		case DD_CAN_SENSOR_ID15:
		SENSOR_VAL = DD_Sensor_status.DAAstatus;
		break;
		
		case DD_CAN_SENSOR_ID16:
		SENSOR_VAL = DD_Sensor_status.DABstatus;
		break;
		
		case DD_CAN_SENSOR_ID17:
		SENSOR_VAL = DD_Sensor_status.DAC2status;
		break;
		
		case DD_CAN_SENSOR_ID18:
		SENSOR_VAL = DD_Sensor_status.flow_sensor_status;
		break;
		
		case DD_CAN_SENSOR_ID19:
		SENSOR_VAL = DD_Sensor_status.aptstatus;
		break;
		
		case DD_CAN_SENSOR_ID20:
		SENSOR_VAL = DD_Sensor_status.vptstatus;
		break;
		case DD_CAN_SENSOR_ID21:
		 SENSOR_VAL=DD_Sensor_status.acid_feed_back;
		 break;
		 case DD_CAN_SENSOR_ID22:
		 SENSOR_VAL=DD_Sensor_status.bicar_feed_back;
		 break;
 	}
	
		
	return SENSOR_VAL;
	
}

