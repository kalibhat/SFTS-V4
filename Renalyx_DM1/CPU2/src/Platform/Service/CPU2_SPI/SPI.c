/*
 * SPI.c
 *
 * Created: 7/5/2016 4:13:32 PM
 *  Author: renalyx
 */ 

#include "asf.h"
#include "spi.h"
#include "CPU2.h"


volatile max_data spi_data;
extern uint8_t DD_CONFIG_TIMER(uint8_t timer_no, uint8_t timer_res);
extern uint8_t DD_SET_TIMER(Tc *tc, uint32_t channel, IRQn_Type irq, uint32_t rc);
extern void DD_INIT_TIMER(void);
/* Chip select. */
#define SPI_CHIP_SEL 0
#define SPI_CHIP_PCS spi_get_pcs(SPI_CHIP_SEL)

/* Clock polarity. */
#define SPI_CLK_POLARITY 0

/* Clock phase. */
#define SPI_CLK_PHASE 0

/* Delay before SPCK. */
#define SPI_DLYBS 0x40

/* Delay between consecutive transfers. */
#define SPI_DLYBCT 0x10

/* SPI slave states for this example. */
#define SLAVE_STATE_IDLE           0
#define SLAVE_STATE_TEST           1
#define SLAVE_STATE_DATA           2
#define SLAVE_STATE_STATUS_ENTRY   3
#define SLAVE_STATE_STATUS         4
#define SLAVE_STATE_END            5

/* SPI example commands for this example. */
/* slave test state, begin to return RC_RDY. */
#define CMD_TEST     0x10101010

/* Slave data state, begin to return last data block. */
#define CMD_DATA     0x29380000

/* Slave status state, begin to return RC_RDY + RC_STATUS. */
#define CMD_STATUS   0x68390384

/* Slave idle state, begin to return RC_SYN. */
#define CMD_END      0x68390484

/* General return value. */
#define RC_SYN       0x55AA55AA

/* Ready status. */
#define RC_RDY       0x12345678

/* Slave data mask. */
#define CMD_DATA_MSK 0xFFFF0000

/* Slave data block mask. */
#define DATA_BLOCK_MSK 0x0000FFFF

/* Number of commands logged in status. */
#define NB_STATUS_CMD   20

/* Number of SPI clock configurations. */
#define NUM_SPCK_CONFIGURATIONS 4

/* SPI Communicate buffer size. */
#define COMM_BUFFER_SIZE   64

/* UART baudrate. */
#define UART_BAUDRATE      115200

/* Data block number. */
#define MAX_DATA_BLOCK_NUMBER  4

/* Max retry times. */
#define MAX_RETRY    4

/* Status block. */
struct status_block_t {
/** Number of data blocks. */
uint32_t ul_total_block_number;
/** Number of SPI commands (including data blocks). */
uint32_t ul_total_command_number;
/** Command list. */
uint32_t ul_cmd_list[NB_STATUS_CMD];
};

/* SPI clock setting (Hz). */
static uint32_t gs_ul_spi_clock = 500000;

/* Current SPI return code. */
static uint32_t gs_ul_spi_cmd = RC_SYN;

/* Current SPI state. */
static uint32_t gs_ul_spi_state = 0;

/* 64 bytes data buffer for SPI transfer and receive. */
static uint8_t gs_uc_spi_buffer[COMM_BUFFER_SIZE];

/* Pointer to transfer buffer. */
static uint8_t *gs_puc_transfer_buffer;

/* Transfer buffer index. */
static uint32_t gs_ul_transfer_index;

/* Transfer buffer length. */
static uint32_t gs_ul_transfer_length;

/* SPI Status. */
static struct status_block_t gs_spi_status;

static uint32_t gs_ul_test_block_number;

/* SPI clock configuration. */
static const uint32_t gs_ul_clock_configurations[] =
{ 500000, 1000000, 2000000, 5000000 };
	static void spi_slave_transfer(void *p_buf, uint32_t size)
	{
		gs_puc_transfer_buffer = p_buf;
		gs_ul_transfer_length = size;
		gs_ul_transfer_index = 0;
		spi_write(SPI_SLAVE_BASE, gs_puc_transfer_buffer[gs_ul_transfer_index], 0,
		0);
	}
	static void spi_slave_command_process(void)
	{
		if (gs_ul_spi_cmd == CMD_END) {
			gs_ul_spi_state = SLAVE_STATE_IDLE;
			gs_spi_status.ul_total_block_number = 0;
			gs_spi_status.ul_total_command_number = 0;
			} else {
			switch (gs_ul_spi_state) {
				case SLAVE_STATE_IDLE:
				/* Only CMD_TEST accepted. */
				if (gs_ul_spi_cmd == CMD_TEST) {
					gs_ul_spi_state = SLAVE_STATE_TEST;
				}
				break;

				case SLAVE_STATE_TEST:
				/* Only CMD_DATA accepted. */
				if ((gs_ul_spi_cmd & CMD_DATA_MSK) == CMD_DATA) {
					gs_ul_spi_state = SLAVE_STATE_DATA;
				}
				gs_ul_test_block_number = gs_ul_spi_cmd & DATA_BLOCK_MSK;
				break;

				case SLAVE_STATE_DATA:
				gs_spi_status.ul_total_block_number++;

				if (gs_spi_status.ul_total_block_number ==
				gs_ul_test_block_number) {
					gs_ul_spi_state = SLAVE_STATE_STATUS_ENTRY;
				}
				break;

				case SLAVE_STATE_STATUS_ENTRY:
				gs_ul_spi_state = SLAVE_STATE_STATUS;
				break;

				case SLAVE_STATE_END:
				break;
			}
		}
	}
	static void spi_slave_new_command(void)
	{
		switch (gs_ul_spi_state) {
			case SLAVE_STATE_IDLE:
			case SLAVE_STATE_END:
			gs_ul_spi_cmd = RC_SYN;
			spi_slave_transfer(&gs_ul_spi_cmd, sizeof(gs_ul_spi_cmd));
			break;

			case SLAVE_STATE_TEST:
			gs_ul_spi_cmd = RC_RDY;
			spi_slave_transfer(&gs_ul_spi_cmd, sizeof(gs_ul_spi_cmd));
			break;

			case SLAVE_STATE_DATA:
			if (gs_spi_status.ul_total_block_number < gs_ul_test_block_number) {
				spi_slave_transfer(gs_uc_spi_buffer, COMM_BUFFER_SIZE);
			}
			break;

			case SLAVE_STATE_STATUS_ENTRY:
			gs_ul_spi_cmd = RC_RDY;
			spi_slave_transfer(&gs_ul_spi_cmd, sizeof(gs_ul_spi_cmd));
			gs_ul_spi_state = SLAVE_STATE_STATUS;
			break;

			case SLAVE_STATE_STATUS:
			gs_ul_spi_cmd = RC_SYN;
			spi_slave_transfer(&gs_spi_status, sizeof(struct status_block_t));
			gs_ul_spi_state = SLAVE_STATE_END;
			break;
		}
	}

void SPI_Handler(void)
{
	uint32_t new_cmd = 0;
	static uint16_t data;
	uint8_t uc_pcs;

	if (spi_read_status(SPI_SLAVE_BASE) & SPI_SR_RDRF) {
		spi_read(SPI_SLAVE_BASE, &data, &uc_pcs);
		gs_puc_transfer_buffer[gs_ul_transfer_index] = data;
		gs_ul_transfer_index++;
		gs_ul_transfer_length--;
		if (gs_ul_transfer_length) {
			spi_write(SPI_SLAVE_BASE,
			gs_puc_transfer_buffer[gs_ul_transfer_index], 0, 0);
		}

		if (!gs_ul_transfer_length) {
			spi_slave_command_process();
			new_cmd = 1;
		}

		if (new_cmd) {
			if (gs_ul_spi_cmd != CMD_END) {
				gs_spi_status.ul_cmd_list[gs_spi_status.ul_total_command_number]
				= gs_ul_spi_cmd;
				gs_spi_status.ul_total_command_number++;
			}
			spi_slave_new_command();
		}
	}
}
void spi_slave_initialize(void)
{
	uint32_t i;

	/* Reset status */
	gs_spi_status.ul_total_block_number = 0;
	gs_spi_status.ul_total_command_number = 0;
	for (i = 0; i < NB_STATUS_CMD; i++) {
		gs_spi_status.ul_cmd_list[i] = 0;
	}
	gs_ul_spi_state = SLAVE_STATE_IDLE;
	gs_ul_spi_cmd = RC_SYN;

	puts("-I- Initialize SPI as slave \r");
	#if (SAMG55)
	/* Enable the peripheral and set SPI mode. */
	flexcom_enable(BOARD_FLEXCOM_SPI);
	flexcom_set_opmode(BOARD_FLEXCOM_SPI, FLEXCOM_SPI);
	#else
	/* Configure an SPI peripheral. */
	spi_enable_clock(SPI_SLAVE_BASE);
	#endif
	spi_disable(SPI_SLAVE_BASE);
	spi_reset(SPI_SLAVE_BASE);
	spi_set_slave_mode(SPI_SLAVE_BASE);
	spi_disable_mode_fault_detect(SPI_SLAVE_BASE);
	spi_set_peripheral_chip_select_value(SPI_SLAVE_BASE, SPI_CHIP_PCS);
	spi_set_clock_polarity(SPI_SLAVE_BASE, SPI_CHIP_SEL, SPI_CLK_POLARITY);
	spi_set_clock_phase(SPI_SLAVE_BASE, SPI_CHIP_SEL, SPI_CLK_PHASE);
	spi_set_bits_per_transfer(SPI_SLAVE_BASE, SPI_CHIP_SEL, SPI_CSR_BITS_8_BIT);
	spi_enable_interrupt(SPI_SLAVE_BASE, SPI_IER_RDRF);
	spi_enable(SPI_SLAVE_BASE);

	/* Start waiting command. */
	spi_slave_transfer(&gs_ul_spi_cmd, sizeof(gs_ul_spi_cmd));
}
void spi_master_initialize(void)
{
	puts("-I- Initialize SPI as master\r");

	#if (SAMG55)
	/* Enable the peripheral and set SPI mode. */
	flexcom_enable(BOARD_FLEXCOM_SPI);
	flexcom_set_opmode(BOARD_FLEXCOM_SPI, FLEXCOM_SPI);
	#else
	/* Configure an SPI peripheral. */
	spi_enable_clock(SPI_MASTER_BASE);
	#endif
	spi_disable(SPI_MASTER_BASE);
	spi_reset(SPI_MASTER_BASE);
	spi_set_lastxfer(SPI_MASTER_BASE);
	spi_set_master_mode(SPI_MASTER_BASE);
	spi_disable_mode_fault_detect(SPI_MASTER_BASE);
	
	uint32_t temp;
	temp = SPI_CHIP_PCS;
	spi_set_peripheral_chip_select_value(SPI_MASTER_BASE, SPI_CHIP_PCS);
	spi_set_peripheral_chip_select_value(SPI_MASTER_BASE, SPI_CHIP_PCS);
	spi_set_clock_polarity(SPI_MASTER_BASE, SPI_CHIP_SEL, SPI_CLK_POLARITY);
	spi_set_clock_phase(SPI_MASTER_BASE, SPI_CHIP_SEL, SPI_CLK_PHASE);
	spi_set_bits_per_transfer(SPI_MASTER_BASE, SPI_CHIP_SEL,
	SPI_CSR_BITS_8_BIT);
	spi_set_baudrate_div(SPI_MASTER_BASE, SPI_CHIP_SEL,
	(sysclk_get_cpu_hz() / gs_ul_spi_clock));
	spi_set_transfer_delay(SPI_MASTER_BASE, SPI_CHIP_SEL, SPI_DLYBS,
	SPI_DLYBCT);
	spi_enable(SPI_MASTER_BASE);
	PIOD->PIO_WPMR = 0x50494F00;
	PIOD->PIO_PER  = 0x00000080;
}
static void spi_set_clock_configuration(uint8_t configuration)
{
	gs_ul_spi_clock = gs_ul_clock_configurations[configuration];
	printf("Setting SPI clock #%lu ... \n\r", (unsigned long)gs_ul_spi_clock);
	spi_master_initialize();
}

static void spi_master_transfer(void *p_buf, uint32_t size)
{
	uint32_t i;
	uint8_t uc_pcs;
	static uint16_t data;

	uint8_t *p_buffer;

	p_buffer = p_buf;

	for (i = 0; i < size; i++) {
		spi_write(SPI_MASTER_BASE, p_buffer[i], 0, 0);
		/* Wait transfer done. */
		while ((spi_read_status(SPI_MASTER_BASE) & SPI_SR_RDRF) == 0);
		spi_read(SPI_MASTER_BASE, &data, &uc_pcs);
		p_buffer[i] = data;
	}
}
static void spi_master_read(void *p_buf, uint32_t size)
{
	uint32_t i;
	uint8_t uc_pcs;
	static uint16_t data;

	uint8_t *p_buffer;

	p_buffer = p_buf;
	
	for (i = 0; i < size +1; i++) {
		spi_write(SPI_MASTER_BASE, p_buffer[0], 0, 0);
		/* Wait transfer done. */
		while ((spi_read_status(SPI_MASTER_BASE) & SPI_SR_RDRF) == 0);
		spi_read(SPI_MASTER_BASE, &data, &uc_pcs);
		p_buffer[i] = data;
	}
}

void spi_master_go(void)
{
	volatile count=0;
	spi_master_initialize();
	volatile uint32_t cmd;
	uint32_t block;
	uint32_t i;
	static bool data_ready = false;
	
	uint8_t data_array[3];

	data_array[0] = 0x80;
	data_array[1] = 0b00100011;
	spi_master_transfer(&data_array, sizeof(data_array));


	data_array[0] = 0x83;
	data_array[1] = 0x80;	
	spi_master_transfer(&data_array, 2);
	data_array[0] = 0x84;
	data_array[1] = 0x00;
	spi_master_transfer(&data_array, 2);	
	data_array[0] = 0x85;
	data_array[1] = 0x40;
	spi_master_transfer(&data_array, 2);
	data_array[0] = 0x86;
	data_array[1] = 0x00;
	spi_master_transfer(&data_array, 2);
	while (count< 100)
		{
			count++;
		}
		count=0;
		
	data_array[0] = 0x80;
	data_array[1] = 0b11100011;
	spi_master_transfer(&data_array, 2);
	
		while (count< 100)
		{
			count++;
		}
		count=0;
				uint8_t temp;
				data_array[0] = 0x01;
				data_array[1] = 0x00;
				data_array[2] = 0x00;
					
					if( (PIOD->PIO_PDSR & (1<<7)) == 0)
					{

						data_ready = true;
						spi_master_read(&data_array, 2);
						spi_data.bytearray[1]=data_array[1];
						spi_data.bytearray[0]=data_array[2];
						cmd = spi_data.Twobyte;
						printf("%d\n",cmd);
						cmd = cmd/2;
					}
					else
					{
						data_ready = false;
					}
					
}