/*
 * DD_CAN.h
 *
 * Created: 3/6/2017 6:01:33 PM
 *  Author: Electrical
 */ 


#ifndef DD_CAN_H_
#define DD_CAN_H_


#include "asf.h"


/** Define the Mailbox mask for eight mailboxes. */
#define GLOBAL_MAILBOX_MASK           0x000000ff

/** Disable all interrupt mask */
#define CAN_DISABLE_ALL_INTERRUPT_MASK 0xffffffff
#define STRING_EOL    "\r"



/** Define the mailbox mode. */
#define CAN_MB_DISABLE_MODE           0
#define CAN_MB_RX_MODE                1
#define CAN_MB_RX_OVER_WR_MODE        2
#define CAN_MB_TX_MODE                3
#define CAN_MB_CONSUMER_MODE          4
#define CAN_MB_PRODUCER_MODE          5

/** Define CAN mailbox transfer status code. */
#define CAN_MAILBOX_TRANSFER_OK       0     /**< Read from or write into mailbox
	                                           successfully. */
#define CAN_MAILBOX_NOT_READY         0x01  /**< Receiver is empty or
	                                           transmitter is busy. */
#define CAN_MAILBOX_RX_OVER           0x02  /**< Message overwriting happens or
	                                           there're messages lost in
	                                           different receive modes. */
#define CAN_MAILBOX_RX_NEED_RD_AGAIN  0x04  /**< Application needs to re-read
	                                           the data register in Receive with
	                                           Overwrite mode. */


#define HEMO_CPU_COMMAND_MESSAGE_ID    0
#define SENSOR_STATUS_MESSAGE_ID    1
#define PROPOTIONING_CPU_COMMAND MESSAGE_ID   2

#define SENSOR_STATUS_PRIORITY       2

#define MAILBOX_0 0
#define MAILBOX_1 1
#define MAILBOX_2 2
#define MAILBOX_3 3
#define MAILBOX_4 4
#define MAILBOX_5 5
#define MAILBOX_6 6
#define MAILBOX_7 7


#define CAN_MID_MIDvA_dd(value) ((0x1FFC0000 & ((value) << 18)))
#define CAN_MFID_MFID_VA_dd(value)  ((0x1FFC0000 & ((value) << 18)))
#define CAN_MCR_MDLC_dd(value) (( (0xfu << 16) & ((value) << 16)))
#define CAN_MASK 0x7FC



/** Define the struct for CAN message mailbox. */

typedef struct {
	uint32_t ul_mb_idx;
	uint8_t uc_obj_type;  /**< Mailbox object type, one of the six different
	                         objects. */
	uint8_t uc_id_ver;    /**< 0 stands for standard frame, 1 stands for
	                         extended frame. */
	uint8_t uc_length;    /**< Received data length or transmitted data
	                         length. */
	uint8_t uc_tx_prio;   /**< Mailbox priority, no effect in receive mode. */
	uint32_t ul_status;   /**< Mailbox status register value. */
	uint32_t ul_id_msk;   /**< No effect in transmit mode. */
	uint32_t ul_id;       /**< Received frame ID or the frame ID to be
	                         transmitted. */
	uint32_t ul_fid;      /**< Family ID. */
	uint32_t ul_datal;
	uint32_t ul_datah;
} can_mb_conf_t;

typedef struct {
	uint8_t uc_tq;      /**< CAN_BIT_SYNC + uc_prog + uc_phase1 + uc_phase2
	                       = uc_tq, 8 <= uc_tq <= 25. */
	uint8_t uc_prog;    /**< Propagation segment, (3-bits + 1), 1~8; */
	uint8_t uc_phase1;  /**< Phase segment 1, (3-bits + 1), 1~8; */
	uint8_t uc_phase2;  /**< Phase segment 2, (3-bits + 1), 1~8, CAN_BIT_IPT
	                       <= uc_phase2; */
	uint8_t uc_sjw;     /**< Resynchronization jump width, (2-bits + 1),
	                       min(uc_phase1, 4); */
	uint8_t uc_sp;      /**< Sample point value, 0~100 in percent. */
} can_bit_timing_t;



typedef struct {
	uint8_t  CPU_SENDER_type_reg;
	uint8_t  SENSOR_GROUP_type_reg;		
	uint16_t ps1status;
	uint16_t ps2status;        //GROUP 1
	uint16_t ps3status;	
	uint16_t thermocouple_status;	
	uint16_t Temp1status;      //GROUP 2
	uint16_t Temp2status;
	uint16_t CS1_Tempstatus;    //GROUP 3
	uint16_t CS2_Tempstatus;	
	uint16_t Temp3status;	
	uint16_t CS3_Tempstatus;
	uint16_t CS1status;
	uint16_t CS2status;         //GROUP 4	
	uint16_t CS3status;
	uint16_t DAC1status;        //GROUP 5
	uint16_t DAAstatus;	
	uint16_t DABstatus;
	uint16_t DAC2status;         //GROUP 6
	uint16_t flow_sensor_status;	
	uint16_t aptstatus;
	uint16_t vptstatus;	         //GROUP 7
	uint16_t DUMMY;
	
	
} DD_Sensor_status_type;

typedef struct {
	uint8_t  CPU_SENDER_reg;
	uint8_t  CPU_COMMAND_reg;
	uint8_t  CPU_COMMANG_SENSOR_GROUP_ID_reg;
} DD_MAIN_CPU_COMMAND_REG_TYPE;

typedef enum   {
	MASTER_CPU_id=10,
	HEMO_CPU_id=1	,
	PROPORTIONING_CPU_id,
	SENSOR_CPU_id,
}dd_cpu_id_type;

typedef enum   {
	Sensor_status_query=0X40,
	Sensor_status_group_id_1,
	Sensor_status_group_id_2,
	Sensor_status_group_id_3,
	Sensor_status_group_id_4,
	Sensor_status_group_id_5,
	Sensor_status_group_id_6,	
}dd_master_command_type;

typedef union {
	   uint32_t four_bytes;
	   uint16_t two_bytes[2];
	   uint8_t bytes_array[4];
}dd_data_size_type;


typedef struct {
	/** PIO dedicated to RS pin index. */
	uint32_t pio_rs_idx;
	/** PIO dedicated to EN pin index. */
	uint32_t pio_en_idx;
} sn65hvd530_ctrl_t;

uint16_t DD_CAN_MESSAGE_READ(void);
void SENSOR_DD_INIT_CAN(void);
void dd_can_mailbox_conf(can_mb_conf_t *p_mailbox);
static void dd_reset_mailbox_conf(can_mb_conf_t *p_mailbox);
uint32_t dd_can_mailbox_write(can_mb_conf_t *p_mailbox);
static uint32_t dd_can_mailbox_tx_remote_frame(can_mb_conf_t *p_mailbox);
static uint32_t dd_can_mailbox_read(can_mb_conf_t *p_mailbox);
static void dd_can_extract_mailbox_data(can_mb_conf_t *p_mailbox);
void dd_can_global_send_transfer_cmd(uint8_t uc_mask);
//uint16_t dd_can_master_command(dd_cpu_id_type  cpu_id,  );














#endif /* DD_CAN_H_ */