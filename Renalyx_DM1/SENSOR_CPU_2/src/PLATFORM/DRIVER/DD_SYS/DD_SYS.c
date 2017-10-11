/*
 * DD_SYS.c
 *
 * Created: 3/8/2017 3:18:10 PM
 *  Author: Electrical
 */ 
#include "DD_SYS.h"

extern uint8_t SPI_CHIP_SEL ;

uint8_t command_data_array[20];
uint16_t GAIN_FACTOR ;
volatile dd_data_size spi_data;


/************************************************************************/
/* ISOLATER 1 CHIP MULTIPLEXER PB3, PB4 PINS                                                                     */
/************************************************************************/
void DD_TS1_CHIP_SELECT()   {
	(PIOB->PIO_CODR) = ((0x1u << 3) | (0x1u << 4)); 
}

void DD_TS2_CHIP_SELECT()   {
	    PIOB->PIO_SODR = (0x1u << 3) ;
	    PIOB->PIO_CODR = (0x1u << 4) ;
}



void DD_CS1_TS_CHIP_SELECT()   {
	
	  (PIOB->PIO_CODR) = (0x1u << 3); 
	  (PIOB->PIO_SODR) = (0x1u << 4);
}

void DD_CS2_TS_CHIP_SELECT ()   {	

	 (PIOB->PIO_SODR) = ((0x1u << 3) | (0x1u << 4));

}

/************************************************************************/
/* ISOLATER 2 CHIP MULTIPLEXER PB5, PB6 PINS                                                                     */
/************************************************************************/

void DD_CS3_TS_CHIP_SELECT()   {
	(PIOB->PIO_CODR) = ((0x1u << 5) | (0x1u << 6));
}

void DD_TS3_CHIP_SELECT()   {
	PIOB->PIO_SODR = (0x1u << 5) ;
PIOB->PIO_CODR = (0x1u << 6) ;
}

void  DD_TS_MUX_CHIP_SELECT()   {
		
	PIOB->PIO_CODR = (0x1u << 5);
    PIOB->PIO_SODR = (0x1u << 6);	
}

void DD_SPARE_TS_CHIP_SELECT()   {
	
	PIOB->PIO_SODR = ((0x1u << 5) | (0x1u << 6));	
}

/************************************************************************/
/* ISOLATER 3 CHIP MULTIPLEXER PB7, PB8 PINS                                                                     */
/************************************************************************/

void DD_CONDUCTIVITY_DIGITAL_POT_CS()   {
	
	PIOB->PIO_CODR = ((0x1u << 7) | (0x1u << 8));	
}

void DD_CON_ADC_CHIP_SELECT()   {	
	 PIOB->PIO_SODR = (0x1u << 7) ;
	 PIOB->PIO_CODR = (0x1u << 8);	
}

void DD_AN_MUX_CHIPSEL_2()   {
	PIOB->PIO_CODR = (0x1u << 7) ;
	PIOB->PIO_SODR = (0x1u << 8) ;
}

void DD_AN_MUX_CHIPSEL_3()   {
	PIOB->PIO_SODR = (0x1u << 7) | (0x1u << 8) ;	
}

// ISOLATER 4 CHIP MULTIPLEXER PB14, PB15 PINS

void DD_AN_MUX_CHIPSEL_4()   {
	PIOB->PIO_CODR = (0x1u << 14) | (0x1u << 15) ;	
}

/************************************************************************/
/*                                                                      */
/************************************************************************/

void DD_SPI_PCS_DECODER_ACTIVE()   {
	PIOB->PIO_CODR =( (0x1u << 0) | (0x1u << 1) );
	PIOB->PIO_SODR = (0x1u << 2) ;	
}

void DD_SPI_PCS_DECODER_DEACTIVE()   {
	PIOB->PIO_SODR =( (0x1u << 0) | (0x1u << 1) );
	PIOB->PIO_CODR = (0x1u << 2) ;
}

 void DD_DISABLE_ISOLATORS_MISO_PIN(){
	
	dd_SET_OUTPUT_DATA(PIOA, SPI0_MISO_ISOLATER_1_DISABLE_PIN);
	dd_SET_OUTPUT_DATA(PIOA, SPI0_MISO_ISOLATER_2_DISABLE_PIN);
	dd_SET_OUTPUT_DATA(PIOA, SPI0_MISO_ISOLATER_3_DISABLE_PIN);
	dd_SET_OUTPUT_DATA(PIOA, SPI0_MISO_ISOLATER_4_DISABLE_PIN);
}

void DD_ENABLE_ISOLATOR_1(){	
	dd_CLEAR_OUTPUT_DATA(PIOA, SPI0_MISO_ISOLATER_1_DISABLE_PIN);
	dd_SET_OUTPUT_DATA(PIOA, SPI0_MISO_ISOLATER_2_DISABLE_PIN);
	dd_SET_OUTPUT_DATA(PIOA, SPI0_MISO_ISOLATER_3_DISABLE_PIN);
	dd_SET_OUTPUT_DATA(PIOA, SPI0_MISO_ISOLATER_4_DISABLE_PIN);
}

void DD_ENABLE_ISOLATOR_2(){
	
	dd_SET_OUTPUT_DATA(PIOA, SPI0_MISO_ISOLATER_1_DISABLE_PIN);
	dd_CLEAR_OUTPUT_DATA(PIOA, SPI0_MISO_ISOLATER_2_DISABLE_PIN);
	dd_SET_OUTPUT_DATA(PIOA, SPI0_MISO_ISOLATER_3_DISABLE_PIN);
	dd_SET_OUTPUT_DATA(PIOA, SPI0_MISO_ISOLATER_4_DISABLE_PIN);
}

void DD_ENABLE_ISOLATOR_3(){
	
	dd_SET_OUTPUT_DATA(PIOA, SPI0_MISO_ISOLATER_1_DISABLE_PIN);
	dd_SET_OUTPUT_DATA(PIOA, SPI0_MISO_ISOLATER_2_DISABLE_PIN);
	dd_CLEAR_OUTPUT_DATA(PIOA, SPI0_MISO_ISOLATER_3_DISABLE_PIN);
	dd_SET_OUTPUT_DATA(PIOA, SPI0_MISO_ISOLATER_4_DISABLE_PIN);
}

void DD_ENABLE_ISOLATOR_4(){
	
	dd_SET_OUTPUT_DATA(PIOA, SPI0_MISO_ISOLATER_1_DISABLE_PIN);
	dd_SET_OUTPUT_DATA(PIOA, SPI0_MISO_ISOLATER_2_DISABLE_PIN);
	dd_SET_OUTPUT_DATA(PIOA, SPI0_MISO_ISOLATER_3_DISABLE_PIN);
	dd_CLEAR_OUTPUT_DATA(PIOA, SPI0_MISO_ISOLATER_4_DISABLE_PIN);
}




uint16_t DD_INIT_ALL_SENSORS()   {
	 DD_SPI_PCS_DECODER_DEACTIVE();
	 DD_DISABLE_ISOLATORS_MISO_PIN();
	 for(int i=0; i<10000;i++) {}
	 	 
 	  DD_PS1_INIT();
	  DD_PS2_INIT();
	  DD_PS3_INIT();

/************************************************************************/
/* ISOLATOR 1 INITIALIZATION DONE HERE                                                                     
/************************************************************************/
	DD_SPI_PCS_DECODER_ACTIVE();
    DD_DISABLE_ISOLATORS_MISO_PIN();
	DD_ENABLE_ISOLATOR_1();	
	 for(int i=0; i < 1000;i++) {}

	DD_TS1_INIT();
    DD_TS2_INIT();	
    DD_CS1_TS_INIT();
    DD_CS2_TS_INIT();
	
    DD_SPI_PCS_DECODER_DEACTIVE();
	DD_DISABLE_ISOLATORS_MISO_PIN();	
/************************************************************************/
/* isolator 2 DEVICE INITIALIZATION DONE HERE                                                                     */
/************************************************************************/
 	
 	DD_SPI_PCS_DECODER_ACTIVE();	
	DD_DISABLE_ISOLATORS_MISO_PIN();
	DD_ENABLE_ISOLATOR_2();	
	for(int i=0; i<1000;i++) {}
	
	DD_CS3_TS_INIT();
	DD_TS3_INIT(); 	
// // 	DD_SPARE_TS_CHIP_SELECT();
// 	
	DD_SPI_PCS_DECODER_DEACTIVE();
 	DD_DISABLE_ISOLATORS_MISO_PIN();
	 for(int i=0; i<1000;i++) {}
// 	    
	  /************************************************************************/
	  /* CONDUCTIVITY_SENSOR_digital_POT_INIT_DONE HERE                                                                     */
	  /************************************************************************/
/*	  command_data_array[0] = AD5174_COMMAND(AD5174_WRITE_SERIAL_REG_TO_CONTRL_REG) ;
	  command_data_array[1]  =  AD5174_CONT_REG_ALLOW_UPDT_DIG_POT_BY_SER_INTERFACE ;
	  dd_spi_master_transfer(&command_data_array, 2 , ISOLTR_2_CS_ID);
	   
	  sensor_data AD5174_data;
	  uint16_t  dig_pot_value = 500 ;	  
	  AD5174_data.Twobyte = dig_pot_value;	 
	  command_data_array[0]  = AD5174_COMMAND(AD5174_WRITE_SERIAL_REG_TO_CONTRL_REG) |  AD5174_data.bytearray[1] ;
	  command_data_array[1]  = AD5174_data.bytearray[0]; 
	 
	 DD_CONDUCTIVITY_DIGITAL_POT_CS();
	// dd_spi_set_peripheral_chip_select_value(ISOLTR_3_CS_ID);	 
	 dd_spi_master_transfer(&command_data_array, 2 , ISOLTR_3_CS_ID); */
	 
	 
	 /************************************************************************/
	 /* CONDUCTIVITY_SENSOR MUX INIT DONE HERE                                                                     */
	 /************************************************************************/	
	 
/*	 command_data_array[0] = CS1_GAIN_X100 | CS2_GAIN_X100 | DAA_GAIN_X100 ;	
	 DD_AN_MUX_CHIPSEL_2();
	 dd_spi_set_peripheral_chip_select_value(ISOLTR_3_CS_ID);
	 dd_spi_master_transfer(&command_data_array, 1, ISOLTR_3_CS_ID);
	 
	 command_data_array[0] = DAB_GAIN_X100 | DAC1_GAIN_X100 ;
	 DD_AN_MUX_CHIPSEL_3();
	 dd_spi_set_peripheral_chip_select_value(ISOLTR_3_CS_ID);
	 dd_spi_master_transfer(&command_data_array, 1, ISOLTR_3_CS_ID);
	 
	  command_data_array[0] = DAC2_GAIN_X10 | CS3_GAIN_X100  ;
	  DD_AN_MUX_CHIPSEL_3();
	  dd_spi_set_peripheral_chip_select_value(ISOLTR_4_CS_ID);
	  dd_spi_master_transfer(&command_data_array, 1, ISOLTR_4_CS_ID );	 */
	 
	 return 0;
}



sensor_status_t DD_READ_SENSORS(sv_sensortype ID_SENSOR, uint16_t *sensor_status )   {
	uint16_t  work_reg ;
	*sensor_status=0;
	uint8_t conductivity_mux_reg_1, conductivity_mux_reg_2, conductivity_mux_reg_3;
	uint16_t CONDUCTIVITY_SENSOR_RMS_VOLTAGE;
	uint32_t timeout = SENSOR_TIMEOUT;
	
	switch(ID_SENSOR)
	{
		case SV_PS1_ID:
		   dd_SET_OUTPUT_DATA(PIOC, PD_PS1_SEN_PIN); 
		   DD_DISABLE_ISOLATORS_MISO_PIN();
		   command_data_array[0] = ADS1120_RDATA;
		   command_data_array[1] = 0;
		   command_data_array[2] = 0;
		   while (PS1_DRDY_N_status) {
			   if (!timeout--) {
				   return SENSOR_NOT_READY;
			   }
		   }		   
		   DD_SPI_PCS_DECODER_ACTIVE();	   		   		   
		   dd_spi_set_peripheral_chip_select_value(PS1_CS_N_ID);
		  // for (int i = 0; i <100 ; i++);	
		   dd_spi_master_read(&command_data_array, 3, PS1_CS_N_ID);
		   spi_data.bytearray[1]=command_data_array[1];
		   spi_data.bytearray[0]=command_data_array[2];
		   *sensor_status = spi_data.Twobyte;
		   DD_DISABLE_ISOLATORS_MISO_PIN();
	       return SENSOR_READ_OK;		  
		break;
				
		case SV_PS2_ID:
		   DD_DISABLE_ISOLATORS_MISO_PIN();
		   command_data_array[0] = ADS1120_RDATA;
		   command_data_array[1] = 0;
		   command_data_array[2] = 0;
		   
		    while (PS2_DRDY_N_status) {
			    if (!timeout--) {
				    return SENSOR_NOT_READY;
			    }
		    }		   
		 
		    DD_SPI_PCS_DECODER_ACTIVE();
			dd_spi_set_peripheral_chip_select_value(PS2_CS_N_ID);
			//for (int i = 0; i <100 ; i++);	
			dd_spi_master_read(&command_data_array, 3, PS2_CS_N_ID);
			spi_data.bytearray[1]=command_data_array[1];
			spi_data.bytearray[0]=command_data_array[2];
			*sensor_status = spi_data.Twobyte;
			DD_DISABLE_ISOLATORS_MISO_PIN();  
		    return SENSOR_READ_OK;
		break;
		
		case SV_PS3_ID:
		   DD_DISABLE_ISOLATORS_MISO_PIN();
		   command_data_array[0] = ADS1120_RDATA;
		   command_data_array[1] = 0;
		   command_data_array[2] = 0;
		   
		   while (PS3_DRDY_N_status) {
			   if (!timeout--) {
				   return SENSOR_NOT_READY;
			   }
		   }
 		DD_SPI_PCS_DECODER_ACTIVE();
		dd_spi_set_peripheral_chip_select_value(PS3_CS_N_ID);
	    //for (int i = 0; i <100 ; i++);	
		dd_spi_master_read(&command_data_array, 3, PS3_CS_N_ID);
	    spi_data.bytearray[1]=command_data_array[1];
		spi_data.bytearray[0]=command_data_array[2];
	    *sensor_status = spi_data.Twobyte;
		DD_DISABLE_ISOLATORS_MISO_PIN();
		return SENSOR_READ_OK;   		  
		   		
		break;
		
		case SV_THERMOCOUPLE_ID:
		 DD_SPI_PCS_DECODER_DEACTIVE();
		
		   DD_DISABLE_ISOLATORS_MISO_PIN();
		    DD_SPI_PCS_DECODER_ACTIVE();
		   command_data_array[0] = 0;
		   command_data_array[1] = 0;
		   command_data_array[2] = 0;
		    while (THERMOCOUPLE_DRDY_PIN) {
			    if (!timeout--) {
				    return SENSOR_NOT_READY;
			    }
		    }  
		   
			DD_SPI_PCS_DECODER_ACTIVE();
			dd_spi_set_peripheral_chip_select_value(SV_THERMOCOUPLE_ID);
			dd_spi_master_read(&command_data_array, 3, SV_THERMOCOUPLE_ID);
			spi_data.bytearray[1]=command_data_array[1];
			spi_data.bytearray[0]=command_data_array[2];
			*sensor_status = spi_data.Twobyte;
			DD_SPI_PCS_DECODER_DEACTIVE();			
			DD_DISABLE_ISOLATORS_MISO_PIN();	
			return SENSOR_READ_OK;	   		
		break;
		
		case SV_TS1_ID:
		   command_data_array[0] = 1;
		   command_data_array[1] = 0;
		   command_data_array[2] = 0;
		   
		    while (DRDY_TS1_status) {
			    if (!timeout--) {
				    return SENSOR_NOT_READY;
			    }
		    } 
		   
			   DD_SPI_PCS_DECODER_ACTIVE();
			   DD_DISABLE_ISOLATORS_MISO_PIN();
			   DD_ENABLE_ISOLATOR_1();			   
			   DD_TS1_CHIP_SELECT();			  	   
			   dd_spi_set_peripheral_chip_select_value(ISOLTR_1_CS_ID);
			   dd_spi_master_read(&command_data_array, 3, ISOLTR_1_CS_ID);			   
			   spi_data.bytearray[1]=command_data_array[1];
			   spi_data.bytearray[0]=command_data_array[2];
			   *sensor_status = spi_data.Twobyte; 			
			  // for(unsigned int count=0; count<1000; count++ )   { ;}
 			   DD_SPI_PCS_DECODER_DEACTIVE();
 			   DD_DISABLE_ISOLATORS_MISO_PIN();	
			   return SENSOR_READ_OK;		   	
		break;
		
		case SV_TS2_ID:
		   command_data_array[0] = 1;
		   command_data_array[1] = 0;
		   command_data_array[2] = 0;
		   
		   while (DRDY_TS2_status) {
			   if (!timeout--) {
				   return SENSOR_NOT_READY;
			   }
		   }
		   
		   
		   DD_SPI_PCS_DECODER_ACTIVE();
		   DD_DISABLE_ISOLATORS_MISO_PIN();
		   DD_ENABLE_ISOLATOR_1();			   
		   DD_TS2_CHIP_SELECT();	
		   dd_spi_set_peripheral_chip_select_value(ISOLTR_1_CS_ID);
		   dd_spi_master_read(&command_data_array, 3,ISOLTR_1_CS_ID );
		   spi_data.bytearray[1]=command_data_array[1];
		   spi_data.bytearray[0]=command_data_array[2];
		   *sensor_status = spi_data.Twobyte;		   
 		   DD_SPI_PCS_DECODER_DEACTIVE();
		   DD_DISABLE_ISOLATORS_MISO_PIN();	
		   return SENSOR_READ_OK;		    
		break;
		
		case SV_CS1_TS_ID:
		   command_data_array[0] = 1;
		   command_data_array[1] = 0;
		   command_data_array[2] = 0;
		   
		   while (DRDY_CS1_TS_status) {
			   if (!timeout--) {
				   return SENSOR_NOT_READY;
			   }
		   }		   
		   
			   DD_SPI_PCS_DECODER_ACTIVE();
			   DD_ENABLE_ISOLATOR_1();
			   DD_CS1_TS_CHIP_SELECT();
			   dd_spi_set_peripheral_chip_select_value(ISOLTR_1_CS_ID);
			   dd_spi_master_read(&command_data_array, 3, ISOLTR_1_CS_ID);
			   spi_data.bytearray[1]=command_data_array[1];
			   spi_data.bytearray[0]=command_data_array[2];
			   *sensor_status = spi_data.Twobyte;
		   
		   DD_SPI_PCS_DECODER_DEACTIVE();
		   DD_DISABLE_ISOLATORS_MISO_PIN();	
		   return SENSOR_READ_OK;	    
		break;
		
		case SV_CS2_TS_ID:
		   command_data_array[0] = 1;
		   command_data_array[1] = 0;
		   command_data_array[2] = 0;
		   
		   while (DRDY_CS2_TS_status) {
			   if (!timeout--) {
				   return SENSOR_NOT_READY;
			   }
		   }	   
		  
			   DD_SPI_PCS_DECODER_ACTIVE();
			   DD_ENABLE_ISOLATOR_1();
			   DD_CS2_TS_CHIP_SELECT();
			   dd_spi_set_peripheral_chip_select_value(ISOLTR_1_CS_ID);
			   dd_spi_master_read(&command_data_array, 3, ISOLTR_1_CS_ID);
			   spi_data.bytearray[1]=command_data_array[1];
			   spi_data.bytearray[0]=command_data_array[2];
			   *sensor_status = spi_data.Twobyte;
		   
		   DD_SPI_PCS_DECODER_DEACTIVE();
		   DD_DISABLE_ISOLATORS_MISO_PIN();	
		   
		   return SENSOR_READ_OK;	 
		break;	
		
		
		case SV_TS3_ID:
		   command_data_array[0] = 1;
		   command_data_array[1] = 0;
		   command_data_array[2] = 0;
		   
// 		   while (DRDY_TS3_status) {
// 			   if (!timeout--) {
// 				   return SENSOR_NOT_READY;
// 			   }
// 		   }
		   
		   
			   DD_SPI_PCS_DECODER_ACTIVE();
			   DD_TS3_CHIP_SELECT();
			   DD_ENABLE_ISOLATOR_2();
			   dd_spi_set_peripheral_chip_select_value(ISOLTR_2_CS_ID);
			   dd_spi_master_read(&command_data_array, 3,ISOLTR_2_CS_ID);
			   spi_data.bytearray[1]=command_data_array[1];
			   spi_data.bytearray[0]=command_data_array[2];
			   *sensor_status = spi_data.Twobyte;
		   
		    DD_SPI_PCS_DECODER_DEACTIVE();
		    DD_DISABLE_ISOLATORS_MISO_PIN();    
			
			return SENSOR_READ_OK;
		break;
		
		case SV_CS3_TS_ID:
		   command_data_array[0] = 1;
		   command_data_array[1] = 0;
		   command_data_array[2] = 0;
		   
		   while (DRDY_CS3_TS_status) {
			   if (!timeout--) {
				   return SENSOR_NOT_READY;
			   }
		   }	   
		   
			   DD_SPI_PCS_DECODER_ACTIVE();
			   DD_CS3_TS_CHIP_SELECT();
			   DD_ENABLE_ISOLATOR_2();
			   dd_spi_set_peripheral_chip_select_value(ISOLTR_2_CS_ID);
			   dd_spi_master_read(&command_data_array, 3, ISOLTR_2_CS_ID);
			   spi_data.bytearray[1]=command_data_array[1];
			   spi_data.bytearray[0]=command_data_array[2];
			   *sensor_status = spi_data.Twobyte;
		   
		   DD_SPI_PCS_DECODER_DEACTIVE();
		   DD_DISABLE_ISOLATORS_MISO_PIN();		  
		   
		   return SENSOR_READ_OK; 
		break;
		return SENSOR_READ_OK;
		
		case SV_CS1_ID:
		  
		   command_data_array[0] = ADC128S022_ADC_IP(ADC128S022_CONDUCTIVITY_SENSOR_RMS_VOLTAGE);
		   command_data_array[1] = 0;  		   
            DD_SPI_PCS_DECODER_ACTIVE();
            DD_CON_ADC_CHIP_SELECT();
			DD_DISABLE_ISOLATORS_MISO_PIN();
            DD_ENABLE_ISOLATOR_3();
            dd_spi_set_peripheral_chip_select_value(ISOLTR_3_CS_ID);
            dd_spi_master_read(&command_data_array, 2, ISOLTR_3_CS_ID);		           
			for(unsigned int count=0; count<1000; count++ )   { ; }				
		   command_data_array[0] = ADC128S022_ADC_IP(ADC128S022_CONDUCTIVITY_SENSOR_RMS_VOLTAGE);
		   command_data_array[1] = 0;  
		    dd_spi_master_read(&command_data_array, 2, ISOLTR_3_CS_ID);
		   spi_data.bytearray[1]=command_data_array[0];
           spi_data.bytearray[0]=command_data_array[1];			
           CONDUCTIVITY_SENSOR_RMS_VOLTAGE = spi_data.Twobyte;		   

			command_data_array[0] = ADC128S022_ADC_IP(ADC128S022_CS1_SEN_RMS_CURRENT);
		   command_data_array[1] = 0;  		   
            DD_SPI_PCS_DECODER_ACTIVE();
            DD_CON_ADC_CHIP_SELECT();
			DD_DISABLE_ISOLATORS_MISO_PIN();
            DD_ENABLE_ISOLATOR_3();
            dd_spi_set_peripheral_chip_select_value(ISOLTR_3_CS_ID);
            dd_spi_master_read(&command_data_array, 2, ISOLTR_3_CS_ID);		           
			for(unsigned int count=0; count<1000; count++ )   {;}				
		   command_data_array[0] = ADC128S022_ADC_IP(ADC128S022_CS1_SEN_RMS_CURRENT);
		   command_data_array[1] = 0;  
		   dd_spi_master_read(&command_data_array, 2, ISOLTR_3_CS_ID);
		   spi_data.bytearray[1]=command_data_array[0];
           spi_data.bytearray[0]=command_data_array[1];			   
		    uint16_t CS1_SEN_RMS_CURRENT;		
            CS1_SEN_RMS_CURRENT = spi_data.Twobyte;	
		   	*sensor_status = (CS1_SEN_RMS_CURRENT*1800)/ CONDUCTIVITY_SENSOR_RMS_VOLTAGE;				
            DD_SPI_PCS_DECODER_DEACTIVE();
            DD_DISABLE_ISOLATORS_MISO_PIN();		   
		break;

   
		
		case SV_CS2_ID:
		 command_data_array[0] = ADC128S022_ADC_IP(ADC128S022_CONDUCTIVITY_SENSOR_RMS_VOLTAGE);
		   command_data_array[1] = 0;  		   
            DD_SPI_PCS_DECODER_ACTIVE();
            DD_CON_ADC_CHIP_SELECT();
			DD_DISABLE_ISOLATORS_MISO_PIN();
            DD_ENABLE_ISOLATOR_3();
            dd_spi_set_peripheral_chip_select_value(ISOLTR_3_CS_ID);
            dd_spi_master_read(&command_data_array, 2, ISOLTR_3_CS_ID);		           
			for(unsigned int count=0; count<1000; count++ )   {;}
		   command_data_array[0] = ADC128S022_ADC_IP(ADC128S022_CONDUCTIVITY_SENSOR_RMS_VOLTAGE);
		   command_data_array[1] = 0;  
		    dd_spi_master_read(&command_data_array, 2, ISOLTR_3_CS_ID);
		   spi_data.bytearray[1]=command_data_array[0];
           spi_data.bytearray[0]=command_data_array[1];			
           CONDUCTIVITY_SENSOR_RMS_VOLTAGE = spi_data.Twobyte;	       
			
			command_data_array[0] = ADC128S022_ADC_IP(ADC128S022_CS2_SEN_RMS_CURRENT);
		   command_data_array[1] = 0;  		   
            DD_SPI_PCS_DECODER_ACTIVE();
            DD_CON_ADC_CHIP_SELECT();
			DD_DISABLE_ISOLATORS_MISO_PIN();
            DD_ENABLE_ISOLATOR_3();
            dd_spi_set_peripheral_chip_select_value(ISOLTR_3_CS_ID);
            dd_spi_master_read(&command_data_array, 2, ISOLTR_3_CS_ID);		           
			for(unsigned int count=0; count<1000; count++ )   {;}
			uint16_t CS2_SEN_RMS_CURRENT;			
		   command_data_array[0] = ADC128S022_ADC_IP(ADC128S022_CS2_SEN_RMS_CURRENT);
		   command_data_array[1] = 0;  
		   dd_spi_master_read(&command_data_array, 2, ISOLTR_3_CS_ID);
		   spi_data.bytearray[1]=command_data_array[0];
           spi_data.bytearray[0]=command_data_array[1];			
           CS2_SEN_RMS_CURRENT = spi_data.Twobyte;	
		   	*sensor_status = (CS2_SEN_RMS_CURRENT*1800)/ CONDUCTIVITY_SENSOR_RMS_VOLTAGE;            
            DD_SPI_PCS_DECODER_DEACTIVE();
            DD_DISABLE_ISOLATORS_MISO_PIN();
			
		break;		  
		
		case SV_CS3_ID:
		 command_data_array[0] = ADC128S022_ADC_IP(ADC128S022_CONDUCTIVITY_SENSOR_RMS_VOLTAGE);
		   command_data_array[1] = 0;  		   
            DD_SPI_PCS_DECODER_ACTIVE();
            DD_CON_ADC_CHIP_SELECT();
			DD_DISABLE_ISOLATORS_MISO_PIN();
            DD_ENABLE_ISOLATOR_3();
            dd_spi_set_peripheral_chip_select_value(ISOLTR_3_CS_ID);
            dd_spi_master_read(&command_data_array, 2, ISOLTR_3_CS_ID);		           
			for(unsigned int count=0; count<1000; count++ )   {;}
		   command_data_array[0] = ADC128S022_ADC_IP(ADC128S022_CONDUCTIVITY_SENSOR_RMS_VOLTAGE);
		   command_data_array[1] = 0;  
		    dd_spi_master_read(&command_data_array, 2, ISOLTR_3_CS_ID);
		   spi_data.bytearray[1]=command_data_array[0];
           spi_data.bytearray[0]=command_data_array[1];			
           CONDUCTIVITY_SENSOR_RMS_VOLTAGE = spi_data.Twobyte;				
			
			command_data_array[0] = ADC128S022_ADC_IP(ADC128S022_CS3_SEN_RMS_CURRENT);
		    command_data_array[1] = 0;            
            dd_spi_set_peripheral_chip_select_value(ISOLTR_3_CS_ID);
            dd_spi_master_read(&command_data_array, 2, ISOLTR_3_CS_ID);		           
			for(unsigned int count=0; count<1000; count++ )   {;}
			uint16_t CS3_SEN_RMS_CURRENT;			
		   command_data_array[0] = ADC128S022_ADC_IP(ADC128S022_CS3_SEN_RMS_CURRENT);
		   command_data_array[1] = 0;  
		   dd_spi_master_read(&command_data_array, 2, ISOLTR_3_CS_ID);
		   spi_data.bytearray[1]=command_data_array[0];
           spi_data.bytearray[0]=command_data_array[1];			
           CS3_SEN_RMS_CURRENT = spi_data.Twobyte;	
		   
		   *sensor_status = (CS3_SEN_RMS_CURRENT*1800)/ CONDUCTIVITY_SENSOR_RMS_VOLTAGE;            
            DD_SPI_PCS_DECODER_DEACTIVE();
            DD_DISABLE_ISOLATORS_MISO_PIN();
		
		    
		break;
		
		case SV_DAC1_ID:
		
		
	         command_data_array[0] = ADC128S022_ADC_IP(ADC128S022_CONDUCTIVITY_SENSOR_RMS_VOLTAGE);
	         command_data_array[1] = 0;
	         DD_SPI_PCS_DECODER_ACTIVE();
	         DD_CON_ADC_CHIP_SELECT();
	         DD_DISABLE_ISOLATORS_MISO_PIN();
	         DD_ENABLE_ISOLATOR_3();
	         dd_spi_set_peripheral_chip_select_value(ISOLTR_3_CS_ID);
	         dd_spi_master_read(&command_data_array, 2, ISOLTR_3_CS_ID);
	         for(unsigned int count=0; count<1000; count++ )   {
		         ;
	         }
	         command_data_array[0] = ADC128S022_ADC_IP(ADC128S022_CONDUCTIVITY_SENSOR_RMS_VOLTAGE);
	         command_data_array[1] = 0;
	         dd_spi_master_read(&command_data_array, 2, ISOLTR_3_CS_ID);
	         spi_data.bytearray[1]=command_data_array[0];
	         spi_data.bytearray[0]=command_data_array[1];
	         CONDUCTIVITY_SENSOR_RMS_VOLTAGE = spi_data.Twobyte;
	         // 			DD_SPI_PCS_DECODER_DEACTIVE();
	         //             DD_DISABLE_ISOLATORS_MISO_PIN();
	         
	         
	         
	         
	         // 			conductivity_mux_reg_2  &= ~DAC1_GAIN_Msk ;
	         // 			conductivity_mux_reg_2 |= DAC1_GAIN_X100;
	         // 	        command_data_array[0] = conductivity_mux_reg_2;
	         
	         //DD_ENABLE_ISOLATOR_4();
	         // 			DD_SPI_PCS_DECODER_ACTIVE();
	         // 	        dd_spi_set_peripheral_chip_select_value(ISOLTR_3_CS_ID);
	         // 	        DD_AN_MUX_CHIPSEL_3();
	         // 	        dd_spi_master_transfer(&command_data_array, 1, ISOLTR_3_CS_ID);
	         // 			for(unsigned int count=0; count<1000; count++ )   {
	         // 		      ;
	         // 	        }
	         // 			DD_SPI_PCS_DECODER_DEACTIVE();
	         //             DD_DISABLE_ISOLATORS_MISO_PIN();
	         
	         
	         command_data_array[0] = ADC128S022_ADC_IP(ADC128S022_DAC1_SEN_RMS_CURRENT);
	         command_data_array[1] = 0;
	         DD_SPI_PCS_DECODER_ACTIVE();
	         DD_CON_ADC_CHIP_SELECT();
	         DD_DISABLE_ISOLATORS_MISO_PIN();
	         DD_ENABLE_ISOLATOR_3();
	         dd_spi_set_peripheral_chip_select_value(ISOLTR_3_CS_ID);
	         dd_spi_master_read(&command_data_array, 2, ISOLTR_3_CS_ID);
	         for(unsigned int count=0; count<1000; count++ )   {
		         ;
	         }
	         uint16_t DAC1_SEN_RMS_CURRENT;
	         float DAC1_RESISTANCE, DAC1_CONDUCTANCE;
	         
	         command_data_array[0] = ADC128S022_ADC_IP(ADC128S022_DAC1_SEN_RMS_CURRENT);
	         command_data_array[1] = 0;
	         dd_spi_master_read(&command_data_array, 2, ISOLTR_3_CS_ID);
	         spi_data.bytearray[1]=command_data_array[0];
	         spi_data.bytearray[0]=command_data_array[1];
	         DAC1_SEN_RMS_CURRENT = spi_data.Twobyte;
	         DAC1_SEN_RMS_CURRENT;
	         DAC1_RESISTANCE =  (CONDUCTIVITY_SENSOR_RMS_VOLTAGE *1800) / (DAC1_SEN_RMS_CURRENT) ;
	         DAC1_RESISTANCE =  DAC1_RESISTANCE - 150;
	         DAC1_CONDUCTANCE = (1/DAC1_RESISTANCE)*1000000 ;
	         
	         *sensor_status =  (uint16_t)DAC1_CONDUCTANCE;  //(DAC1_SEN_RMS_CURRENT*2000)/ CONDUCTIVITY_SENSOR_RMS_VOLTAGE;
	         DD_SPI_PCS_DECODER_DEACTIVE();
	         DD_DISABLE_ISOLATORS_MISO_PIN();
	         break;
		
		case SV_DAB_ID:
	command_data_array[0] = ADC128S022_ADC_IP(ADC128S022_CONDUCTIVITY_SENSOR_RMS_VOLTAGE);
		   command_data_array[1] = 0;  		   
            DD_SPI_PCS_DECODER_ACTIVE();
            DD_CON_ADC_CHIP_SELECT();
			DD_DISABLE_ISOLATORS_MISO_PIN();
            DD_ENABLE_ISOLATOR_3();
            dd_spi_set_peripheral_chip_select_value(ISOLTR_3_CS_ID);
            dd_spi_master_read(&command_data_array, 2, ISOLTR_3_CS_ID);		           
			for(unsigned int count=0; count<1000; count++ )   { ; }
		   command_data_array[0] = ADC128S022_ADC_IP(ADC128S022_CONDUCTIVITY_SENSOR_RMS_VOLTAGE);
		   command_data_array[1] = 0;  
		    dd_spi_master_read(&command_data_array, 2, ISOLTR_3_CS_ID);
		   spi_data.bytearray[1]=command_data_array[0];
           spi_data.bytearray[0]=command_data_array[1];			
           CONDUCTIVITY_SENSOR_RMS_VOLTAGE = spi_data.Twobyte;				    
			DD_SPI_PCS_DECODER_DEACTIVE();
            DD_DISABLE_ISOLATORS_MISO_PIN();			
			
			command_data_array[0] = ADC128S022_ADC_IP(ADC128S022_DAB_SEN_RMS_CURRENT);
		    command_data_array[1] = 0;  		   
            DD_SPI_PCS_DECODER_ACTIVE();
            DD_CON_ADC_CHIP_SELECT();
			DD_DISABLE_ISOLATORS_MISO_PIN();
            DD_ENABLE_ISOLATOR_3();
            dd_spi_set_peripheral_chip_select_value(ISOLTR_3_CS_ID);
            dd_spi_master_read(&command_data_array, 2, ISOLTR_3_CS_ID);		           
			for(unsigned int count=0; count<1000; count++ )   {;}
			uint16_t DAB_SEN_RMS_CURRENT;
			float DAB_RESISTANCE, DAB_CONDUCTANCE;
			
		   command_data_array[0] = ADC128S022_ADC_IP(ADC128S022_DAB_SEN_RMS_CURRENT);
		   command_data_array[1] = 0;  
		   dd_spi_master_read(&command_data_array, 2, ISOLTR_3_CS_ID);
		   for(unsigned int count=0; count<1000; count++ )   {;}
		   spi_data.bytearray[1]=command_data_array[0];
           spi_data.bytearray[0]=command_data_array[1];			
           DAB_SEN_RMS_CURRENT = spi_data.Twobyte;	
		   
		      DAB_RESISTANCE =  (CONDUCTIVITY_SENSOR_RMS_VOLTAGE *1800) / (DAB_SEN_RMS_CURRENT) ;
	         DAB_RESISTANCE =  DAB_RESISTANCE-150 ;
	         DAB_CONDUCTANCE = (1/DAB_RESISTANCE)*1000000 ;
	         
	         *sensor_status =  (uint16_t)DAB_CONDUCTANCE;  //(DAC1_SEN_RMS_CURRENT*2000)/ CONDUCTIVITY_
		   
		   
		   
		            
            DD_SPI_PCS_DECODER_DEACTIVE();
            DD_DISABLE_ISOLATORS_MISO_PIN();
		   
		break;
		
		case SV_DAA_ID:
	command_data_array[0] = ADC128S022_ADC_IP(ADC128S022_CONDUCTIVITY_SENSOR_RMS_VOLTAGE);
		   command_data_array[1] = 0;  		   
            DD_SPI_PCS_DECODER_ACTIVE();
            DD_CON_ADC_CHIP_SELECT();
			DD_DISABLE_ISOLATORS_MISO_PIN();
            DD_ENABLE_ISOLATOR_3();
            dd_spi_set_peripheral_chip_select_value(ISOLTR_3_CS_ID);
            dd_spi_master_read(&command_data_array, 2, ISOLTR_3_CS_ID);		           
			for(unsigned int count=0; count<1000; count++ )   { ; }
		   command_data_array[0] = ADC128S022_ADC_IP(ADC128S022_CONDUCTIVITY_SENSOR_RMS_VOLTAGE);
		   command_data_array[1] = 0;  
		    dd_spi_master_read(&command_data_array, 2, ISOLTR_3_CS_ID);
		   spi_data.bytearray[1]=command_data_array[0];
           spi_data.bytearray[0]=command_data_array[1];			
           CONDUCTIVITY_SENSOR_RMS_VOLTAGE = spi_data.Twobyte;				    
			DD_SPI_PCS_DECODER_DEACTIVE();
            DD_DISABLE_ISOLATORS_MISO_PIN();			
			
			command_data_array[0] = ADC128S022_ADC_IP(ADC128S022_DAA_SEN_RMS_CURRENT);
		    command_data_array[1] = 0;  		   
            DD_SPI_PCS_DECODER_ACTIVE();
            DD_CON_ADC_CHIP_SELECT();
			DD_DISABLE_ISOLATORS_MISO_PIN();
            DD_ENABLE_ISOLATOR_3();
            dd_spi_set_peripheral_chip_select_value(ISOLTR_3_CS_ID);
            dd_spi_master_read(&command_data_array, 2, ISOLTR_3_CS_ID);		           
			for(unsigned int count=0; count<1000; count++ )   {;}
			uint16_t DAA_SEN_RMS_CURRENT;
			float DAA_RESISTANCE, DAA_CONDUCTANCE;
			
		   command_data_array[0] = ADC128S022_ADC_IP(ADC128S022_DAA_SEN_RMS_CURRENT);
		   command_data_array[1] = 0;  
		   dd_spi_master_read(&command_data_array, 2, ISOLTR_3_CS_ID);
		   spi_data.bytearray[1]=command_data_array[0];
           spi_data.bytearray[0]=command_data_array[1];			
           DAA_SEN_RMS_CURRENT = spi_data.Twobyte;	
		   
		      DAA_RESISTANCE =  (CONDUCTIVITY_SENSOR_RMS_VOLTAGE *1800) / (DAA_SEN_RMS_CURRENT) ;
	         DAA_RESISTANCE =  DAA_RESISTANCE - 150;
	         DAA_CONDUCTANCE = (1/DAA_RESISTANCE)*1000000 ;
	         
	         *sensor_status =  (uint16_t)DAA_CONDUCTANCE;  //(DAC1_SEN_RMS_CURRENT*2000)/ CONDUCTIVITY_
		   
		   
		   
		            
            DD_SPI_PCS_DECODER_DEACTIVE();
            DD_DISABLE_ISOLATORS_MISO_PIN();
		   
		break;
		
		
		
		case SV_DAC2_ID:
		command_data_array[0] = ADC128S022_ADC_IP(ADC128S022_CONDUCTIVITY_SENSOR_RMS_VOLTAGE);
		   command_data_array[1] = 0;  		   
            DD_SPI_PCS_DECODER_ACTIVE();
            DD_CON_ADC_CHIP_SELECT();
			DD_DISABLE_ISOLATORS_MISO_PIN();
            DD_ENABLE_ISOLATOR_3();
            dd_spi_set_peripheral_chip_select_value(ISOLTR_3_CS_ID);
            dd_spi_master_read(&command_data_array, 2, ISOLTR_3_CS_ID);		           
			for(unsigned int count=0; count<1000; count++ )   {
		      ;
	        }
		   command_data_array[0] = ADC128S022_ADC_IP(ADC128S022_CONDUCTIVITY_SENSOR_RMS_VOLTAGE);
		   command_data_array[1] = 0;  
		    dd_spi_master_read(&command_data_array, 2, ISOLTR_3_CS_ID);
		   spi_data.bytearray[1]=command_data_array[0];
           spi_data.bytearray[0]=command_data_array[1];			
           CONDUCTIVITY_SENSOR_RMS_VOLTAGE = spi_data.Twobyte;				    
			DD_SPI_PCS_DECODER_DEACTIVE();
            DD_DISABLE_ISOLATORS_MISO_PIN();		
			
			
			command_data_array[0] = ADC128S022_ADC_IP(ADC128S022_DAC2_SEN_RMS_CURRENT);
		    command_data_array[1] = 0;  		   
            DD_SPI_PCS_DECODER_ACTIVE();
            DD_CON_ADC_CHIP_SELECT();
			DD_DISABLE_ISOLATORS_MISO_PIN();
            DD_ENABLE_ISOLATOR_3();
            dd_spi_set_peripheral_chip_select_value(ISOLTR_3_CS_ID);
            dd_spi_master_read(&command_data_array, 2, ISOLTR_3_CS_ID);		           
			for(unsigned int count=0; count<1000; count++ )   {
		      ;
	        }
			uint16_t DAC2_SEN_RMS_CURRENT;
			
		   command_data_array[0] = ADC128S022_ADC_IP(ADC128S022_DAC2_SEN_RMS_CURRENT);
		   command_data_array[1] = 0;  
		   dd_spi_master_read(&command_data_array, 2, ISOLTR_3_CS_ID);
		   spi_data.bytearray[1]=command_data_array[0];
           spi_data.bytearray[0]=command_data_array[1];			
           DAC2_SEN_RMS_CURRENT = spi_data.Twobyte;	
		   *sensor_status = (DAC2_SEN_RMS_CURRENT*10000)/ CONDUCTIVITY_SENSOR_RMS_VOLTAGE;            
            DD_SPI_PCS_DECODER_DEACTIVE();
            DD_DISABLE_ISOLATORS_MISO_PIN();
		break;
		
		case SV_FLOW_SENSOR_ID:		
		break;
				
		default:
		break;
		
	}	
		
}