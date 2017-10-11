/*
 * SPI.h
 *
 * Created: 7/5/2016 4:13:41 PM
 *  Author: renalyx
 */ 


#ifndef SPI_H_
#define SPI_H_
#define SPI_ID          ID_SPI0

/** SPI base address for SPI master mode*/
#define SPI_MASTER_BASE      SPI0
/** SPI base address for SPI slave mode, (on different board) */
#define SPI_SLAVE_BASE       SPI0

typedef union{
	
	uint8_t bytearray[2] ;
	uint16_t Twobyte  ;
	
}max_data;



#endif /* SPI_H_ */