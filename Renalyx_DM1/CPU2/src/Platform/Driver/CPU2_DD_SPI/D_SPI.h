/*
 * SPI.h
 *
 * Created: 4/24/2017 7:12:13 PM
 *  Author: VarunK
 */ 


#ifndef SPI_H_
#define SPI_H_
/*
 * IncFile1.h
 *
 * Created: 3/13/2017 3:50:26 PM
 *  Author: VarunK
 */ 


#ifndef INCFILE1_H_
#define INCFILE1_H_





#endif /* INCFILE1_H_ */
/*
 * DD_SPI.h
 *
 * Created: 3/6/2017 6:14:19 PM
 *  Author: Electrical
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
typedef union{
	
	uint8_t bytearray[2] ;
	uint16_t Twobyte  ;
	
}maxm_data;





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

void dd_spi_master_initialize(void);

void dd_spi_set_clock_phase(uint16_t ul_pcs_group_id, uint32_t ul_phase);
void dd_spi_set_clock_polarity(uint16_t ul_pcs_group_id,uint32_t ul_polarity);

void dd_spi_set_transfer_delay(uint16_t ul_pcs_group_id, uint8_t uc_dlybs, uint8_t uc_dlybct);
void dd_spi_configure_cs_behavior(uint16_t ul_pcs_group_id, uint32_t ul_cs_behavior);
void dd_spi_set_bits_per_transfer(uint16_t ul_pcs_group_id, uint32_t ul_bits);
int16_t dd_spi_set_baudrate_div(uint16_t ul_pcs_group_id, uint8_t uc_baudrate_divider);

int16_t dd_spi_calc_baudrate_div(const uint32_t baudrate, uint32_t mck);
void dd_spi_set_delay_between_chip_select(uint32_t ul_delay);
 void dd_spi_set_peripheral_chip_select_value(uint16_t pcs_ul_value);
spi_status_t dd_spi_write(uint8_t us_data);
spi_status_t dd_spi_read(uint8_t *us_data);
void dd_spi_master_go(void);
 void dd_spi_master_read(void *p_buf, uint32_t size);
 void dd_spi_master_transfer(void *p_buf, uint32_t size);
 void dd_APT_CS(void);
 void dd_VPT_CS(void);


#endif /* DD_SPI_H_ */





#endif /* SPI_H_ */





