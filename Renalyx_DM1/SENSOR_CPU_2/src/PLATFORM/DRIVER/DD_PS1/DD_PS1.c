/*
 * DD_PS1.c
 *
 * Created: 5/4/2017 7:04:00 PM
 *  Author: Electrical
 */ 
#include "DD_PS1.h"

extern uint8_t command_data_array[20];
void DD_PS1_INIT()   {
	 DD_SPI_PCS_DECODER_DEACTIVE();
	 DD_DISABLE_ISOLATORS_MISO_PIN();
	
	 DD_SPI_PCS_DECODER_ACTIVE();
	 command_data_array[0] = ADS1120_RESET;		
 	 dd_spi_set_peripheral_chip_select_value(PS1_CS_N_ID); 	  
 	 dd_spi_master_transfer(&command_data_array, 1, PS1_CS_N_ID);
	  
	   for(unsigned int count=0; count<1000; count++ )   {
		   ;
	   }	  
	
	  
	  
	  command_data_array[0] = ADS1120_WREG_COMMAND | ADS1120_WREG_START_REG_ADR(0) | ADS1120_WREG_NO_OF_BYTES(3) ; //0x43;
	  command_data_array[1] = ADS1120_CONFIG_REG0_MUX(ADS1120_MUX_AIN1_AIN2) |  ADS1120_CONFIG_REG0_GAIN(ADS1120_DEVICE_GAIN_1) | ADS1120_CONFIG_REG0_PGA_BYPASS ;                                          //0b00110001;
	  command_data_array[2] = ADS1120_CONFIG_REG1_CM | ADS1120_CONFIG_REG1_MODE(ADS1120_MODE_MORMAL) | ADS1120_CONFIG_REG1_DR(ADS1120_DR_330_SPS);//0b00000100;
	  command_data_array[3] = ADS1120_CONFIG_REG2_50_60(ADS1120_50_REJECTION) | ADS1120_CONFIG_REG2_VREF(ADS1120_AVDD_AVSS); //0b11100000;
	  command_data_array[4] = 0;                       //0b00000000;
	  dd_spi_set_peripheral_chip_select_value(PS1_CS_N_ID); 
	  dd_spi_master_transfer(&command_data_array, 5, PS1_CS_N_ID);
	  
	  for(unsigned int count=0; count<1000; count++ )   {
		  ;
	  }
	  
	  command_data_array[0] = ADS1120_START_OR_SYNC;
	  dd_spi_set_peripheral_chip_select_value(PS1_CS_N_ID); 
	  dd_spi_master_transfer(&command_data_array, 1, PS1_CS_N_ID);	
	   for(unsigned int count=0; count<1000; count++ )   {
		   ;
	   }
}