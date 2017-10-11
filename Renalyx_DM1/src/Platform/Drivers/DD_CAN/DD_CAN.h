
/*
 * DD_CAN.h
 *
 * Created: 2/21/2017 11:19:47 AM
 *  Author: Gopalkrishna Kalibhat
 */ 



#ifndef DD_CAN_H_
#define DD_CAN_H_
#include "asf.h"


/** Define the Mailbox mask for eight mailboxes. */
#define GLOBAL_MAILBOX_MASK           0x000000ff

/** Disable all interrupt mask */
#define CAN_DISABLE_ALL_INTERRUPT_MASK 0xffffffff



/** Define the mailbox mode. */
#define CAN_MB_DISABLE_MODE           0
#define CAN_MB_RX_MODE                1
#define CAN_MB_RX_OVER_WR_MODE        2
#define CAN_MB_TX_MODE                3
#define CAN_MB_CONSUMER_MODE          4
#define CAN_MB_PRODUCER_MODE          5

#define STRING_EOL    "\r"

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

typedef struct {
	/** PIO dedicated to RS pin index. */
	//uint32_t pio_rs_idx;
	/** PIO dedicated to EN pin index. */
	uint32_t pio_en_idx;
} sn65hvda540_ctrl_t;


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
	
	uint16_t Temp3status;
	uint16_t CS1_Tempstatus;    //GROUP 3
	uint16_t CS2_Tempstatus;
	
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
	uint16_t bpfeedback;
	
	uint16_t acid_feed_back;
	uint16_t bicar_feed_back;	         //GROUP 8 for selftest
	uint16_t dummy_data;
	
} DD_Sensor_status_type;

typedef enum   {
	 PROP_DUMMY,
	 PROP_START_ALIGMENT ,
	 PROP_START_MIXING ,                             
	 PROP_STOP_MIXING  ,                                 
	 PROP_START_DISINFECTION   , 
	 PROP_STOP_DISINFECTION ,                               
	 PROP_START_CITRIC_DISINFECTION  ,  
	 PROP_STOP_CITRIC_DISINFECTION ,                             
	 PROP_START_OPENFILL ,                             
	 PROP_STOP_OPENFILL ,                             
	 PROP_COMPLETED    ,                               
	 PROP_RECOVER   ,//;
	 PROP_START_ACID_PUMP,
	 PROP_START_BICARB_PUMP,
	 PROP_UPDATE_ACID_RC,              // added to update Acid RC 
	 PROP_UPDATE_BICARB_RC,			  // added to update Bicarb RC
	 PROP_CONDUCTIVITY,			 // added to set conductivity
	 prop_SELFTEST,			 // added to set conductivity
	 prop_BICAR_FEED_BACK,             //ADDED FOR SELFTEST
	 prop_BICAR_FEED_BACK_STATUS,
	 BP_START_BLOOD_PUMP = 0x20,
	 BP_STOP_PUMP,
	 BP_COMPLETED ,
	 BP_SPEED_CHANGE,
	 BP_DAC_REF ,
	 BP_INC ,
	 BP_DEC ,
	 BP_APT_VPT_SEND_TO_MASTER ,
	 BP_RECOVER ,
	 HEMO_START_BD_CALIBRATION ,
	 
	 SB_Sensor_status_query = 0x40, 
	 SB_Sensor_status_query_group1,
	 SB_Sensor_status_query_group2,
	 SB_Sensor_status_query_group3,
	 SB_Sensor_status_query_group4,
	 SB_Sensor_status_query_group5,
	 SB_Sensor_status_query_group6,
	 SB_Sensor_status_query_group7,
	 SB_Sensor_status_query_group8,
	 
	  
	 
	}MASTER_CMD_ID_TYPE;

typedef enum   {
	MASTER_CPU_id=10,
	HEMO_CPU_id	=1,
	PROPORTIONING_CPU_id,
	SENSOR_CPU_id ,	
	//HEMO_CPU_id = 5	,
}dd_cpu_id_type;

typedef enum   {
	Sensor_group_1=1,
	Sensor_group_2,
	Sensor_group_3,
	Sensor_group_4,
	Sensor_group_5,
	Sensor_group_6,
	Sensor_group_7,
	Sensor_group_8,	
	sensor_group_max
}dd_sensor_group_type;

typedef enum {
	DD_CAN_SENSOR_NULL = 0,
	DD_CAN_SENSOR_ID1 = 1,	 // PS1
	DD_CAN_SENSOR_ID2 = 2,	// ps2
	DD_CAN_SENSOR_ID3 = 3,	// ps3
	DD_CAN_SENSOR_ID4 = 4,	// thermocouple
	DD_CAN_SENSOR_ID5 = 5,	// ts1
	DD_CAN_SENSOR_ID6 = 6,	// ts2
	DD_CAN_SENSOR_ID7 = 7,	// ts3
	DD_CAN_SENSOR_ID8 = 8,	// cs1-ts
	DD_CAN_SENSOR_ID9 = 9,	// cs2 -ts
	DD_CAN_SENSOR_ID10 = 10, //cs3 -ts
	DD_CAN_SENSOR_ID11 = 11, // cs1
	DD_CAN_SENSOR_ID12 = 12, // cs2
	DD_CAN_SENSOR_ID13 = 13, // cs3
	DD_CAN_SENSOR_ID14 = 14,// DAC1
	DD_CAN_SENSOR_ID15 = 15, // DAA
	DD_CAN_SENSOR_ID16 = 16, // DAB
	DD_CAN_SENSOR_ID17 = 17, // DAC2
	DD_CAN_SENSOR_ID18 = 18, // Flow Sensor
	DD_CAN_SENSOR_ID19	= 19,// APT
	DD_CAN_SENSOR_ID20	= 20, // VPT
	DD_CAN_SENSOR_ID21  = 21,//ACID FEED BACK
	DD_CAN_SENSOR_ID22  = 22,//BICAR FEED BACK
	DD_SENSIN_ID_MAX
}DD_sensortype;

typedef union {
	   uint32_t temp_reg_32;
	   uint16_t temp_reg_16[2];
	   uint8_t temp_reg_8[4];
}temp_struct_type;

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
void DD_INIT_CAN(void);
static void dd_can_mailbox_conf(can_mb_conf_t *p_mailbox);
static void dd_reset_mailbox_conf(can_mb_conf_t *p_mailbox);
static uint32_t dd_can_mailbox_write(can_mb_conf_t *p_mailbox);
static uint32_t dd_can_mailbox_tx_remote_frame(can_mb_conf_t *p_mailbox);
static uint32_t dd_can_mailbox_read(can_mb_conf_t *p_mailbox);
static void dd_can_extract_mailbox_data(can_mb_conf_t *p_mailbox);
static void dd_can_dump_mailbox_data(can_mb_conf_t *p_mailbox );
uint16_t dd_master_command(dd_cpu_id_type  cpu_id, MASTER_CMD_ID_TYPE command_id, uint16_t speed );
void dd_can_global_send_transfer_cmd(uint8_t uc_mask);
void sn65hvda540_set_en(sn65hvda540_ctrl_t *p_component, uint32_t pin_idx);
void sn65hvda540_normal_mode(sn65hvda540_ctrl_t *p_component);
void sn65hvda540_standby_mode(sn65hvda540_ctrl_t *p_component);
void DD_CAN_WRITE_PROPO(uint16_t data, uint16_t rc_count);
void DD_CAN_WRITE_HEMO(uint16_t data, uint16_t speed);
void DD_CAN_WRITE_SENSOR(uint16_t data);
void DD_CAN_READ_APTVPT(void);
void DD_CAN_READ_PROPO_FB(void);
uint16_t DD_CAN_READ_SENSORS(DD_sensortype ID_SENSOR);
//uint16_t dd_master_command(dd_cpu_id_type  cpu_id, MASTER_CMD_ID_TYPE command_id );

#endif /* DD_CAN_H_ */





