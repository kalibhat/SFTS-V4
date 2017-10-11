/*
 * DD_THERMOCOUPLE.c
 *
 * Created: 5/4/2017 10:25:17 AM
 *  Author: Electrical
 */

#include "DD_THERMOCOUPLE.h"
extern uint8_t command_data_array[20];
extern volatile dd_data_size spi_data;
 
 void DD_THERMOCOUPLE_INIT()   {
	 
	  DD_SPI_PCS_DECODER_ACTIVE();
	  command_data_array[0] = ADS1248_RESET;
	  dd_spi_set_peripheral_chip_select_value(THERMOCOUPLE_CS_ID);
	  dd_spi_master_transfer(&command_data_array, 1, THERMOCOUPLE_CS_ID);
	  
	  for(unsigned int count=0; count<1000; count++ )   {
		  ;
	  }
	  
	  #if 0
	  command_data_array[0] = ADS1248_WREG_COMMAND |  ADS1248_WREG_FIRST_REG(0); //0b00000110;
	  command_data_array[1] = ADS1248_WREG_2_COMMAND |  ADS1248_WREG_2_NUM_OF_BYTES(6);
	  command_data_array[2] = 0x12; //ADS1248_MUX0_MUX_SP(ADS1248_MUX0_SP_AIN0) | ADS1248_MUX0_MUX_SN(ADS1248_MUX0_SN_AIN1);//MUX0 INITIALIZAION DONE HERE
	  command_data_array[3] = 0x1;    //VBIAS INITIALIZAION DONE HERE
	  command_data_array[4] = 0x30; //ADS1248_MUX1_VREFCON(ADS1248_INTERNAL_REF_ALWAYS_ON) |ADS1248_MUX1_REFSELT(ADS1248_INTERNAL_REF);//MUX1 INIT
	  command_data_array[5] =  0x52; //ADS1248_SYS0_DR(ADS1248_DR_SPS_20)| ADS1248_SYS0_PGA(ADS1248_PGA_32) ; //SYS0  INIT
	  command_data_array[6] = 01;
	  command_data_array[7] = 2;
	  command_data_array[8] = 3;
	  command_data_array[9] = 4;
	  command_data_array[10] = 5;
	  command_data_array[11] = 0;
	  command_data_array[12] = 0x06; //ADS1248_IDAC0_IMAG(ADS1248_IDAC_EXCITATION_CURRENT_1000UA) ; //IDAC0
	  command_data_array[13] = 0x89;// ADS1248_IDAC1_I1DIR(ADS1248_IEXC1) | ADS1248_IDAC1_I2DIR(ADS1248_IEXC2);   // IDAC1
	  command_data_array[14] = 0;
	  command_data_array[15] = 0;
	  command_data_array[16] = 2;
	  dd_spi_set_peripheral_chip_select_value(THERMOCOUPLE_CS_ID);
	  dd_spi_master_transfer(&command_data_array, 8, THERMOCOUPLE_CS_ID);
	  
	  #endif
	  // 	  for(unsigned int count=0; count<1; count++ )   {
	  // 		  ;
	  // 		  }
	  //
	  command_data_array[0] = ADS1248_WREG_COMMAND |  ADS1248_WREG_FIRST_REG(0); //0b00000110;
	  command_data_array[1] = ADS1248_WREG_2_COMMAND |  ADS1248_WREG_2_NUM_OF_BYTES(0);
	  command_data_array[2] = 0x01;
	  dd_spi_set_peripheral_chip_select_value(THERMOCOUPLE_CS_ID);
	  dd_spi_master_transfer(&command_data_array, 3, THERMOCOUPLE_CS_ID);
	  
	  
	  command_data_array[0] = ADS1248_WREG_COMMAND |  ADS1248_WREG_FIRST_REG(1); //0b00000110;
	  command_data_array[1] = ADS1248_WREG_2_COMMAND |  ADS1248_WREG_2_NUM_OF_BYTES(0);
	  command_data_array[2] = 0x00;
	  dd_spi_set_peripheral_chip_select_value(THERMOCOUPLE_CS_ID);
	  dd_spi_master_transfer(&command_data_array, 3, THERMOCOUPLE_CS_ID);
	  
	  command_data_array[0] = ADS1248_WREG_COMMAND |  ADS1248_WREG_FIRST_REG(2); //0b00000110;
	  command_data_array[1] = ADS1248_WREG_2_COMMAND |  ADS1248_WREG_2_NUM_OF_BYTES(0);
	  command_data_array[2] = 0x30;
	  dd_spi_set_peripheral_chip_select_value(THERMOCOUPLE_CS_ID);
	  dd_spi_master_transfer(&command_data_array, 3, THERMOCOUPLE_CS_ID);
	  

	  
	  command_data_array[0] = ADS1248_WREG_COMMAND |  ADS1248_WREG_FIRST_REG(3); //0b00000110;
	  command_data_array[1] = ADS1248_WREG_2_COMMAND |  ADS1248_WREG_2_NUM_OF_BYTES(0);
	  command_data_array[2] = 0x52;
	  dd_spi_set_peripheral_chip_select_value(THERMOCOUPLE_CS_ID);
	  dd_spi_master_transfer(&command_data_array, 3, THERMOCOUPLE_CS_ID);
	  
	  command_data_array[0] = ADS1248_WREG_COMMAND |  ADS1248_WREG_FIRST_REG(4); //0b00000110;
	  command_data_array[1] = ADS1248_WREG_2_COMMAND |  ADS1248_WREG_2_NUM_OF_BYTES(0);
	  command_data_array[2] = 0x00;
	  dd_spi_set_peripheral_chip_select_value(THERMOCOUPLE_CS_ID);
	  dd_spi_master_transfer(&command_data_array, 3, THERMOCOUPLE_CS_ID);
	  
	  command_data_array[0] = ADS1248_WREG_COMMAND |  ADS1248_WREG_FIRST_REG(5); //0b00000110;
	  command_data_array[1] = ADS1248_WREG_2_COMMAND |  ADS1248_WREG_2_NUM_OF_BYTES(0);
	  command_data_array[2] = 0x00;
	  dd_spi_set_peripheral_chip_select_value(THERMOCOUPLE_CS_ID);
	  dd_spi_master_transfer(&command_data_array, 3, THERMOCOUPLE_CS_ID);
	  
	  command_data_array[0] = ADS1248_WREG_COMMAND |  ADS1248_WREG_FIRST_REG(6); //0b00000110;
	  command_data_array[1] = ADS1248_WREG_2_COMMAND |  ADS1248_WREG_2_NUM_OF_BYTES(0);
	  command_data_array[2] = 0x00;
	  dd_spi_set_peripheral_chip_select_value(THERMOCOUPLE_CS_ID);
	  dd_spi_master_transfer(&command_data_array, 3, THERMOCOUPLE_CS_ID);
	  
	  command_data_array[0] = ADS1248_WREG_COMMAND |  ADS1248_WREG_FIRST_REG(7); //0b00000110;
	  command_data_array[1] = ADS1248_WREG_2_COMMAND |  ADS1248_WREG_2_NUM_OF_BYTES(0);
	  command_data_array[2] = 0x00;
	  dd_spi_set_peripheral_chip_select_value(THERMOCOUPLE_CS_ID);
	  dd_spi_master_transfer(&command_data_array, 3, THERMOCOUPLE_CS_ID);

	  command_data_array[0] = ADS1248_WREG_COMMAND |  ADS1248_WREG_FIRST_REG(8); //0b00000110;
	  command_data_array[1] = ADS1248_WREG_2_COMMAND |  ADS1248_WREG_2_NUM_OF_BYTES(0);
	  command_data_array[2] = 0x00;
	  
	  dd_spi_set_peripheral_chip_select_value(THERMOCOUPLE_CS_ID);
	  dd_spi_master_transfer(&command_data_array, 3, THERMOCOUPLE_CS_ID);
	  
	  command_data_array[0] = ADS1248_WREG_COMMAND |  ADS1248_WREG_FIRST_REG(9); //0b00000110;
	  command_data_array[1] = ADS1248_WREG_2_COMMAND |  ADS1248_WREG_2_NUM_OF_BYTES(0);
	  command_data_array[2] = 0x00;
	  
	  dd_spi_set_peripheral_chip_select_value(THERMOCOUPLE_CS_ID);
	  dd_spi_master_transfer(&command_data_array, 3, THERMOCOUPLE_CS_ID);
	  
	  command_data_array[0] = ADS1248_WREG_COMMAND |  ADS1248_WREG_FIRST_REG(10); //0b00000110;
	  command_data_array[1] = ADS1248_WREG_2_COMMAND |  ADS1248_WREG_2_NUM_OF_BYTES(0);
	  command_data_array[2] = 0x6;
	  
	  dd_spi_set_peripheral_chip_select_value(THERMOCOUPLE_CS_ID);
	  dd_spi_master_transfer(&command_data_array, 3, THERMOCOUPLE_CS_ID);
	  
	  command_data_array[0] = ADS1248_WREG_COMMAND |  ADS1248_WREG_FIRST_REG(11); //0b00000110;
	  command_data_array[1] = ADS1248_WREG_2_COMMAND |  ADS1248_WREG_2_NUM_OF_BYTES(0);
	  command_data_array[2] = 0x89;
	  
	  dd_spi_set_peripheral_chip_select_value(THERMOCOUPLE_CS_ID);
	  dd_spi_master_transfer(&command_data_array, 3, THERMOCOUPLE_CS_ID);
	  
	  command_data_array[0] = ADS1248_WREG_COMMAND |  ADS1248_WREG_FIRST_REG(12); //0b00000110;
	  command_data_array[1] = ADS1248_WREG_2_COMMAND |  ADS1248_WREG_2_NUM_OF_BYTES(0);
	  command_data_array[2] = 0x0;
	  
	  dd_spi_set_peripheral_chip_select_value(THERMOCOUPLE_CS_ID);
	  dd_spi_master_transfer(&command_data_array, 3, THERMOCOUPLE_CS_ID);
	  
	  command_data_array[0] = ADS1248_WREG_COMMAND |  ADS1248_WREG_FIRST_REG(13); //0b00000110;
	  command_data_array[1] = ADS1248_WREG_2_COMMAND |  ADS1248_WREG_2_NUM_OF_BYTES(0);
	  command_data_array[2] = 0x0;
	  
	  dd_spi_set_peripheral_chip_select_value(THERMOCOUPLE_CS_ID);
	  dd_spi_master_transfer(&command_data_array, 3, THERMOCOUPLE_CS_ID);
	  
	  command_data_array[0] = ADS1248_WREG_COMMAND |  ADS1248_WREG_FIRST_REG(14); //0b00000110;
	  command_data_array[1] = ADS1248_WREG_2_COMMAND |  ADS1248_WREG_2_NUM_OF_BYTES(0);
	  command_data_array[2] = 0x0;
	  
	  dd_spi_set_peripheral_chip_select_value(THERMOCOUPLE_CS_ID);
	  dd_spi_master_transfer(&command_data_array, 3, THERMOCOUPLE_CS_ID);
	  
	  
	  uint recv_array[20] = {};
	  
	  for(int j = 0; j<20;j++)
	  {
		  command_data_array[j]= 0xff;
	  }
	  dd_spi_set_peripheral_chip_select_value(THERMOCOUPLE_CS_ID);
	  #if 0
	  for(int j = 0; j<15;j++)
	  {
		  command_data_array[0] = ADS1248_RREG_COMMAND |  ADS1248_RREG_FIRST_REG(j); //0b00000110;
		  command_data_array[1] = ADS1248_RREG_2_COMMAND |  ADS1248_RREG_2_NUM_OF_BYTES(0);
		  for(unsigned int count=0; count<100000; count++ )   {
			  ;
		  }
		  dd_spi_master_read(&command_data_array, 3, THERMOCOUPLE_CS_ID);
		  recv_array[j]= command_data_array[2];
	  }
	  #endif
	  
	  command_data_array[0] = ADS1248_RREG_COMMAND |  ADS1248_RREG_FIRST_REG(0); //0b00000110;
	  command_data_array[1] = ADS1248_RREG_2_COMMAND |  ADS1248_RREG_2_NUM_OF_BYTES(0);

	  dd_spi_master_read(&command_data_array, 3, THERMOCOUPLE_CS_ID);
	  recv_array[0]= command_data_array[2];
	  
	  
	  command_data_array[0] = ADS1248_RREG_COMMAND |  ADS1248_RREG_FIRST_REG(1); //0b00000110;
	  command_data_array[1] = ADS1248_RREG_2_COMMAND |  ADS1248_RREG_2_NUM_OF_BYTES(0);

	  dd_spi_master_read(&command_data_array, 3, THERMOCOUPLE_CS_ID);
	  recv_array[1]= command_data_array[2];
	  
	  command_data_array[0] = ADS1248_RREG_COMMAND |  ADS1248_RREG_FIRST_REG(2); //0b00000110;
	  command_data_array[1] = ADS1248_RREG_2_COMMAND |  ADS1248_RREG_2_NUM_OF_BYTES(0);

	  dd_spi_master_read(&command_data_array, 3, THERMOCOUPLE_CS_ID);
	  recv_array[2]= command_data_array[2];
	  
	  
	  command_data_array[0] = ADS1248_RREG_COMMAND |  ADS1248_RREG_FIRST_REG(3); //0b00000110;
	  command_data_array[1] = ADS1248_RREG_2_COMMAND |  ADS1248_RREG_2_NUM_OF_BYTES(0);

	  dd_spi_master_read(&command_data_array, 3, THERMOCOUPLE_CS_ID);
	  recv_array[3]= command_data_array[2];
	  
	  
	  command_data_array[0] = ADS1248_RREG_COMMAND |  ADS1248_RREG_FIRST_REG(4); //0b00000110;
	  command_data_array[1] = ADS1248_RREG_2_COMMAND |  ADS1248_RREG_2_NUM_OF_BYTES(0);

	  dd_spi_master_read(&command_data_array, 3, THERMOCOUPLE_CS_ID);
	  recv_array[4]= command_data_array[2];
	  
	  command_data_array[0] = ADS1248_RREG_COMMAND |  ADS1248_RREG_FIRST_REG(5); //0b00000110;
	  command_data_array[1] = ADS1248_RREG_2_COMMAND |  ADS1248_RREG_2_NUM_OF_BYTES(0);

	  dd_spi_master_read(&command_data_array, 3, THERMOCOUPLE_CS_ID);
	  recv_array[5]= command_data_array[2];
	  
	  command_data_array[0] = ADS1248_RREG_COMMAND |  ADS1248_RREG_FIRST_REG(6); //0b00000110;
	  command_data_array[1] = ADS1248_RREG_2_COMMAND |  ADS1248_RREG_2_NUM_OF_BYTES(0);

	  dd_spi_master_read(&command_data_array, 3, THERMOCOUPLE_CS_ID);
	  recv_array[6]= command_data_array[2];
	  
	  command_data_array[0] = ADS1248_RREG_COMMAND |  ADS1248_RREG_FIRST_REG(7); //0b00000110;
	  command_data_array[1] = ADS1248_RREG_2_COMMAND |  ADS1248_RREG_2_NUM_OF_BYTES(0);

	  dd_spi_master_read(&command_data_array, 3, THERMOCOUPLE_CS_ID);
	  recv_array[7]= command_data_array[2];
	  
	  
	  command_data_array[0] = ADS1248_RREG_COMMAND |  ADS1248_RREG_FIRST_REG(8); //0b00000110;
	  command_data_array[1] = ADS1248_RREG_2_COMMAND |  ADS1248_RREG_2_NUM_OF_BYTES(0);

	  dd_spi_master_read(&command_data_array, 3, THERMOCOUPLE_CS_ID);
	  recv_array[8]= command_data_array[2];
	  
	  
	  command_data_array[0] = ADS1248_RREG_COMMAND |  ADS1248_RREG_FIRST_REG(9); //0b00000110;
	  command_data_array[1] = ADS1248_RREG_2_COMMAND |  ADS1248_RREG_2_NUM_OF_BYTES(0);

	  dd_spi_master_read(&command_data_array, 3, THERMOCOUPLE_CS_ID);
	  recv_array[9]= command_data_array[2];
	  
	  
	  command_data_array[0] = ADS1248_RREG_COMMAND |  ADS1248_RREG_FIRST_REG(10); //0b00000110;
	  command_data_array[1] = ADS1248_RREG_2_COMMAND |  ADS1248_RREG_2_NUM_OF_BYTES(0);

	  dd_spi_master_read(&command_data_array, 3, THERMOCOUPLE_CS_ID);
	  recv_array[10]= command_data_array[2];
	  
	  
	  command_data_array[0] = ADS1248_RREG_COMMAND |  ADS1248_RREG_FIRST_REG(11); //0b00000110;
	  command_data_array[1] = ADS1248_RREG_2_COMMAND |  ADS1248_RREG_2_NUM_OF_BYTES(0);

	  dd_spi_master_read(&command_data_array, 3, THERMOCOUPLE_CS_ID);
	  recv_array[11]= command_data_array[2];
	  
	  
	  command_data_array[0] = ADS1248_RREG_COMMAND |  ADS1248_RREG_FIRST_REG(12); //0b00000110;
	  command_data_array[1] = ADS1248_RREG_2_COMMAND |  ADS1248_RREG_2_NUM_OF_BYTES(0);

	  dd_spi_master_read(&command_data_array, 3, THERMOCOUPLE_CS_ID);
	  recv_array[12]= command_data_array[2];
	  
	  
	  command_data_array[0] = ADS1248_RREG_COMMAND |  ADS1248_RREG_FIRST_REG(13); //0b00000110;
	  command_data_array[1] = ADS1248_RREG_2_COMMAND |  ADS1248_RREG_2_NUM_OF_BYTES(0);

	  dd_spi_master_read(&command_data_array, 3, THERMOCOUPLE_CS_ID);
	  recv_array[13]= command_data_array[2];
	  
	  
	  command_data_array[0] = ADS1248_RREG_COMMAND |  ADS1248_RREG_FIRST_REG(14); //0b00000110;
	  command_data_array[1] = ADS1248_RREG_2_COMMAND |  ADS1248_RREG_2_NUM_OF_BYTES(0);

	  dd_spi_master_read(&command_data_array, 3, THERMOCOUPLE_CS_ID);
	  recv_array[14]= command_data_array[2];
	  
	  
	  
	  
	  
	  command_data_array[0] = ADS1248_WREG_COMMAND |  ADS1248_WREG_FIRST_REG(0); //0b00000110;
	  command_data_array[1] = ADS1248_WREG_2_COMMAND |  ADS1248_WREG_2_NUM_OF_BYTES(14);
	  command_data_array[2] = 0x01; //ADS1248_MUX0_MUX_SP(ADS1248_MUX0_SP_AIN0) | ADS1248_MUX0_MUX_SN(ADS1248_MUX0_SN_AIN1);//MUX0 INITIALIZAION DONE HERE
	  command_data_array[3] = 0;    //VBIAS INITIALIZAION DONE HERE
	  command_data_array[4] = 0x30; //ADS1248_MUX1_VREFCON(ADS1248_INTERNAL_REF_ALWAYS_ON) |ADS1248_MUX1_REFSELT(ADS1248_INTERNAL_REF);//MUX1 INIT
	  command_data_array[5] =  0x52; //ADS1248_SYS0_DR(ADS1248_DR_SPS_20)| ADS1248_SYS0_PGA(ADS1248_PGA_32) ; //SYS0  INIT
	  command_data_array[6] = 0;
	  command_data_array[7] = 0;
	  command_data_array[8] = 0;
	  command_data_array[9] = 0;
	  command_data_array[10] = 0;
	  command_data_array[11] = 0;
	  command_data_array[12] = 0x06; ADS1248_IDAC0_IMAG(ADS1248_IDAC_EXCITATION_CURRENT_1000UA) ; //IDAC0
	  command_data_array[13] = 0x89;// ADS1248_IDAC1_I1DIR(ADS1248_IEXC1) | ADS1248_IDAC1_I2DIR(ADS1248_IEXC2);   // IDAC1
	  command_data_array[14] = 0;
	  command_data_array[15] = 0;
	  command_data_array[16] = 0;
	  dd_spi_set_peripheral_chip_select_value(THERMOCOUPLE_CS_ID);
	  dd_spi_master_transfer(&command_data_array, 17, THERMOCOUPLE_CS_ID);
	  for(unsigned int count=0; count<1000; count++ )   {
		  ;
	  }
	  
	  dd_spi_set_peripheral_chip_select_value(THERMOCOUPLE_CS_ID);
	  command_data_array[0] = ADS1248_RREG_COMMAND |  ADS1248_RREG_FIRST_REG(0); //0b00000110;
	  command_data_array[1] = ADS1248_RREG_2_COMMAND |  ADS1248_RREG_2_NUM_OF_BYTES(14);
	  dd_spi_master_read(&command_data_array, 17, THERMOCOUPLE_CS_ID);
	  for(unsigned int count=0; count<1000; count++ )   {
		  ;
	  }
	  
	  
	  command_data_array[0] = 	ADS1248_RDATAC;
	  dd_spi_master_transfer(&command_data_array, 1, THERMOCOUPLE_CS_ID);
	  
	  for(unsigned int count=0; count<1000; count++ )   {
		  ;
	  }	 
 }