/*
 * APT_VPT.c
 *
 * Created: 5/18/2017 2:40:57 PM
 *  Author: VarunK
 */ 


#include "asf.h"
#include "APT_VPT.h"
#include  "Platform/Driver/CPU2_DD_SPI/D_SPI.h"
uint16_t command_data_array[5];
	

void dd_APT_REG_INIT()
     	{
    	
		command_data_array[0] = ADS1120_RESET;
     	dd_spi_master_transfer(&command_data_array, 1);
		 
		 
		 for (int i=0;i<10000;i++)
		 {
	;
		 }
		 		 
		 command_data_array[0] = ADS1120_WREG_COMMAND | ADS1120_WREG_START_REG_ADR(0) | ADS1120_WREG_NO_OF_BYTES(3) ; //0x43;
		 command_data_array[1] = ADS1120_CONFIG_REG0_MUX(ADS1120_MUX_AIN1_AIN2) |  ADS1120_CONFIG_REG0_GAIN(ADS1120_DEVICE_GAIN_32) ;    //3E                                      //0b00110001;
		 command_data_array[2] = 0x04;      //ADS1120_CONFIG_REG1_CM | ADS1120_CONFIG_REG1_MODE(ADS1120_MODE_MORMAL) | ADS1120_CONFIG_REG1_DR(ADS1120_DR_20_SPS);//0b00000100; 04
		 command_data_array[3] = 0x98;                  //ADS1120_CONFIG_REG2_IDAC(ADS1120_IDAC_OFF) | ADS1120_CONFIG_REG2_50_60(ADS1120_FIR_50_60_REJECTION) | ADS1120_CONFIG_REG2_VREF(ADS1120_EXTERNAL_REF_REFP1_REFN1); //0b10011000   98;
		 command_data_array[4] = 0;                       //0b00000000;
		 // 							  dd_spi_set_peripheral_chip_select_value(PS1_CS_N_ID);
		 // 					 	  dd_spi_master_transfer(&command_data_array, sizeof(command_data_array), PS1_CS_N_ID);
		 dd_spi_master_transfer(&command_data_array, 5);
		 
		 
		 
										  
										  
						
		  command_data_array[0] = ADS1120_START_OR_SYNC;
		dd_spi_master_transfer(&command_data_array, 1);
  
		 
		   for (int i=0;i<10000;i++)
		   {
			   ;
		   }   
		   int i = 0;
 }
		   
		   
		   void dd_VPT_REG_INIT()
		   {
			   command_data_array[0] = ADS1120_RESET;
			   dd_spi_master_transfer(&command_data_array, 1);
			   
			   
			   for (int i=0;i<10000;i++)
			   {
				   ;
			   }
			   			   
			   command_data_array[0] = ADS1120_WREG_COMMAND | ADS1120_WREG_START_REG_ADR(0) | ADS1120_WREG_NO_OF_BYTES(3) ; //0x43;
			   command_data_array[1] = ADS1120_CONFIG_REG0_MUX(ADS1120_MUX_AIN1_AIN2) |  ADS1120_CONFIG_REG0_GAIN(ADS1120_DEVICE_GAIN_32) ;    //3E                                      //0b00110001;
			   command_data_array[2] = 0x04;      //ADS1120_CONFIG_REG1_CM | ADS1120_CONFIG_REG1_MODE(ADS1120_MODE_MORMAL) | ADS1120_CONFIG_REG1_DR(ADS1120_DR_20_SPS);//0b00000100; 04
			   command_data_array[3] = 0x98;                  //ADS1120_CONFIG_REG2_IDAC(ADS1120_IDAC_OFF) | ADS1120_CONFIG_REG2_50_60(ADS1120_FIR_50_60_REJECTION) | ADS1120_CONFIG_REG2_VREF(ADS1120_EXTERNAL_REF_REFP1_REFN1); //0b10011000   98;
			   command_data_array[4] = 0;                       //0b00000000;
			   // 							  dd_spi_set_peripheral_chip_select_value(PS1_CS_N_ID);
			   // 					 	  dd_spi_master_transfer(&command_data_array, sizeof(command_data_array), PS1_CS_N_ID);
			   dd_spi_master_transfer(&command_data_array, 5);
			   
			   
			   command_data_array[0] = ADS1120_START_OR_SYNC;
			   dd_spi_master_transfer(&command_data_array, 1);
			   
			   
			   for (int i=0;i<10000;i++)
			   {
				   ;
			   }
			   int i = 0;
		   }