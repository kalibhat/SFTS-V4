/*
 * DD_TS3.c
 *
 * Created: 5/4/2017 7:00:11 PM
 *  Author: Electrical
 */ 

 #include "DD_TS3.h"
 extern uint8_t command_data_array[20];
 
 
 void DD_TS3_INIT() {
	 dd_spi_set_clock_polarity(1, SPI_CLK_POLARITY_1);
	 dd_spi_set_clock_phase(1, SPI_CLK_PHASE_0);
	 
	 command_data_array[0] = MAX31865_WRITE_CONFIGURATION; //0b00000110;
	 command_data_array[1] = MAXIM31865_CONFIGURATION_VBIAS | MAXIM31865_CONFIGURATION_AUTO_CONVERTION | MAXIM31865_CONFIGURATION_FAULT_STATUS_CLEAR | MAXIM31865_CONFIGURATION_50HZ_FILTER ;
	 
	 DD_SPI_PCS_DECODER_ACTIVE();
	 DD_TS3_CHIP_SELECT();
	 dd_spi_set_peripheral_chip_select_value(ISOLTR_2_CS_ID); 
	 dd_spi_master_transfer(&command_data_array, 2,0 ); 
	 
	 for(unsigned int count=0; count<1000; count++ )   {
		 ;
	 }
	 
	 dd_spi_set_clock_polarity(1, SPI_CLK_POLARITY_0);
	 dd_spi_set_clock_phase(1, SPI_CLK_PHASE_0);
	  for(unsigned int count=0; count<1000; count++ )   {
		 ;
	 }
	 
	 command_data_array[0] =  MAXIM_IC_TO_PT100 ;	 
	 dd_spi_set_peripheral_chip_select_value(ISOLTR_2_CS_ID);	 	 
	 DD_TS_MUX_CHIP_SELECT();
	 dd_spi_master_transfer(&command_data_array, 1, ISOLTR_2_CS_ID);
	 
	 for(unsigned int count=0; count<1000; count++ )   {
		 ;
	 }
	 
	 
 }