

/*
 * DD_SPI.h
 *
 * Created: 3/6/2017 6:14:19 PM
 *  Author: GOPALKRISHNA KALIBHAT
 */ 


#ifndef DD_SPI_H_
#define DD_SPI_H_
//#include "PLATFORM/SERVICE/SENSOR_SV_INTERFACE.h"

#define SPI_ID          ID_SPI0

/** SPI base address for SPI master mode*/
#define SPI_MASTER_BASE      SPI0
/** SPI base address for SPI slave mode, (on different board) */
#define SPI_SLAVE_BASE       SPI0
#define SPI_TIMEOUT       15000

/* Clock polarity. */
#define SPI_CLK_POLARITY_0  0
#define SPI_CLK_POLARITY_1  1


/* Clock phase. */
#define SPI_CLK_PHASE_0 0
#define SPI_CLK_PHASE_1 1

typedef union{
	
	uint8_t bytearray[2] ;
	uint16_t Twobyte  ;
	
}max_data;

// typedef enum
// {
// 	SPI_ERROR = -1,
// 	SPI_OK = 0,
// 	SPI_ERROR_TIMEOUT = 1,
// 	SPI_ERROR_ARGUMENT,
// 	SPI_ERROR_OVERRUN,
// 	SPI_ERROR_MODE_FAULT,
// 	SPI_ERROR_OVERRUN_AND_MODE_FAULT
// } spi_status_t;

// typedef enum spi_cs_behavior {
// 	/** CS does not rise until a new transfer is requested on different chip select. */
// 	SPI_CS_KEEP_LOW = SPI_CSR_CSAAT,
// 	/** CS rises if there is no more data to transfer. */
// 	SPI_CS_RISE_NO_TX = 0,
// 	/** CS is de-asserted systematically during a time DLYBCS. */
// 	SPI_CS_RISE_FORCED = SPI_CSR_CSNAAT
// } spi_cs_behavior_t;

uint16_t DD_SENSOR_CPU_SPI_READ(uint8_t pcs_value );
void dd_spi_set_writeprotect(uint32_t ul_enable);

void DD_INIT_SPI(void);

void dd_spi_set_clock_phase(uint16_t ul_pcs_group_id, uint32_t ul_phase);
void dd_spi_set_clock_polarity(uint16_t ul_pcs_group_id,uint32_t ul_polarity);

void dd_spi_set_transfer_delay(uint16_t ul_pcs_group_id, uint8_t uc_dlybs, uint8_t uc_dlybct);
void dd_spi_configure_cs_behavior(uint16_t ul_pcs_group_id, uint32_t ul_cs_behavior);
void dd_spi_set_bits_per_transfer(uint16_t ul_pcs_group_id, uint32_t ul_bits);
int16_t dd_spi_set_baudrate_div(uint16_t ul_pcs_group_id, uint8_t uc_baudrate_divider);

int16_t dd_spi_calc_baudrate_div(const uint32_t baudrate, uint32_t mck);
void dd_spi_set_delay_between_chip_select(uint32_t ul_delay);
 void dd_spi_set_peripheral_chip_select_value(uint16_t pcs_ul_value);
spi_status_t dd_spi_write(uint16_t us_data);
spi_status_t dd_spi_read(uint16_t *us_data, uint8_t *p_pcs);
void dd_spi_master_go(void);
static void dd_spi_master_read(void *p_buf, uint32_t size);
 void DD_SPI_SET_POT(void *p_buf, uint32_t size);


#endif /* DD_SPI_H_ */



