/*
 * GPIO.c
 *
 * Created: 12/2/2014 4:24:05 PM
 *  Author: wid7
 */ 
#include "GPIO.h"

void CPU2_D_INIT_PIO()
{
	/*pmc_enable_periph_clk(ID_PIOA);
	pmc_enable_periph_clk(ID_PIOB);
	pmc_enable_periph_clk(ID_PIOC);
	pmc_enable_periph_clk(ID_PIOD);*/
	
	PIOA->PIO_WPMR = 0x50494F00;
	PIOB->PIO_WPMR = 0x50494F00;
	PIOC->PIO_WPMR = 0x50494F00;
	PIOD->PIO_WPMR = 0x50494F00;
	
	PIOA->PIO_PUER = 0x00060000;			
	PIOA->PIO_PDR =	 0x3E060000;				// initialization for TWI //
	//PIOA->PIO_ABSR = 0x00090000;
	
	PIOA->PIO_PUER = 0x00000300;			// for scanf statement//
	
	PIOA->PIO_PER  = 0x00007C00;
	PIOA->PIO_OER  = 0x00002800;
	PIOA->PIO_ODR  = 0x00005400;
	PIOA->PIO_SODR = 0x00000800;
	PIOA->PIO_CODR = 0x00002000;
	PIOA->PIO_PUER = 0x00002800;
	
	
	PIOB->PIO_PER  = 0x06008000;
	PIOB->PIO_OER  = 0x06000000;
	PIOB->PIO_ODR  = 0x00008000;
	PIOB->PIO_SODR = 0x06000000;
	
	PIOC->PIO_PER  = 0x018FF06A;
	PIOC->PIO_OER  = 0x408FF002;
	PIOC->PIO_ODR  = 0x01000068;
	PIOC->PIO_SODR = 0x000BF002;
	PIOC->PIO_PUER = 0x408FF002;
	PIOC->PIO_CODR = 0x00840000;

	PIOD->PIO_PER  = 0x000006EF;
	PIOD->PIO_OER  = 0x0000064F;
	PIOD->PIO_ODR  = 0x00000020;
	PIOD->PIO_SODR = 0x0000064F;
	PIOD->PIO_PUER = 0x0000064F;
	
	gpio_configure_pin(PIN_CAN0_RX_IDX, PIN_CAN0_RX_FLAGS);
	gpio_configure_pin(PIN_CAN0_TX_IDX, PIN_CAN0_TX_FLAGS);
	gpio_configure_pin(PIN_CAN0_STANDBY_IDX, PIN_CAN0_STANDBY_FLAGS);
	
//	sn65hvda540_set_en(&can0_transceiver ,PIN_CAN0_STANDBY_IDX);
//	sn65hvda540_normal_mode(&can0_transceiver);
}



uint8_t CPU2_D_SET_PIO(pio_type peripheral,uint32_t pio)
{
	switch (peripheral)
	{
		case PA:
		PIOA->PIO_SODR = 1<<pio;
		break;
		case PB:
		PIOB->PIO_SODR = 1<<pio;
		break;
		case PC:
		PIOC->PIO_SODR = 1<<pio;
		break;
		case PD:
		PIOD->PIO_SODR = 1<<pio;
		break;	
	}
	return 0;
}

uint8_t CPU2_D_RESET_PIO(const pio_type peripheral,uint32_t pio)
  {
	  switch (peripheral)
	  {
		  case PA:
		  PIOA->PIO_CODR = 1<<pio;
		  break;
		  case PB:
		  PIOB->PIO_CODR = 1<<pio;
		  break;
		  case PC:
		  PIOC->PIO_CODR = 1<<pio;
		  break;
		  case PD:
		  PIOD->PIO_CODR = 1<<pio;
		  break;  
	  }
	  return 0;
  }
  
  uint16_t CPU2_D_READ_PIO( pio_type peripheral,uint32_t pio)
  {
	  uint16_t ret_val=0;
	  switch (peripheral)
	  {
		  case PA:
		  
		  if( (PIOA->PIO_PDSR & (1<<pio)) == 0)
		  {
			  return 0;
		  }
		  else
		  {
			  return 1;
		  } 
		  break;
		  case PB:
		  if( (PIOB->PIO_PDSR & (1<<pio)) == 0)
		  {
			  return 0;
		  }
		  else
		  {
			  return 1;
		  }
		  break;
		  case PC:
		  if( (PIOC->PIO_PDSR & (1<<pio)) == 0)
		  {
			  return 0;
		  }
		  else
		  {
			 ret_val = 1;
			 return(ret_val);
			 
		  }
		  break;
		  case PD:
		  if( (PIOD->PIO_PDSR & (1<<pio)) == 0)
		  {
			  return 0;
		  }
		  else
		  {
			  return 1;
		  }
		  break; 
	  }
}