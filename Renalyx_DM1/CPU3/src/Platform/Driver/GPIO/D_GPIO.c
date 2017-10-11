/*
 * D_GPIO.c
 *
 * Created: 12/9/2014 1:10:24 PM
 *  Author: wid7
 */ 
#include "D_GPIO.h"

void CPU3_D_INIT_PIO()
{
	pmc_enable_periph_clk(ID_PIOA);
	pmc_enable_periph_clk(ID_PIOB);
	pmc_enable_periph_clk(ID_PIOC);
	pmc_enable_periph_clk(ID_PIOD);
	
	
	PIOA->PIO_WPMR = 0x50494F00;
	PIOB->PIO_WPMR = 0x50494F00;
	PIOC->PIO_WPMR = 0x50494F00;
	PIOD->PIO_WPMR = 0x50494F00;
	
	PIOA->PIO_PUER = 0x00060000;			
	PIOA->PIO_PDR  = 0x1F870343;		//PIOA->PIO_PDR =	 0x00060000;				// initialization for TWI //
	PIOA->PIO_ABSR = 0x00090000;
	PIOA->PIO_PUER = 0x00000300;			// for scanf statement//
	PIOA->PIO_PUER = 0x00000300;
	PIOA->PIO_PER  = 0x0018C004;		//PIOA->PIO_PER  = 0x0018C000;
	PIOA->PIO_OER  = 0x00180004;		//PIOA->PIO_OER  = 0x0018C000;
	PIOA->PIO_SODR = 0x00180000;           //??????
	PIOA->PIO_ODR  = 0x0000C000;             //??????
	
		
	PIOB->PIO_PER  = 0x08204000;		//PIOB->PIO_OER  = 0x04204000;
	PIOB->PIO_OER  = 0x08204000;		//PIOB->PIO_OER  = 0x06204000;
	PIOB->PIO_PDR  = 0xF01800FF;
	PIOB->PIO_SODR = 0x06204000;
	PIOB->PIO_CODR = 0x04000000;
	
	PIOC->PIO_PER  = 0x0D8FF1FE;		//PIOC->PIO_PER  = 0x378FF1FE;
	PIOC->PIO_OER  = 0x008FF1FE;		//PIOC->PIO_OER  = 0x068FF1FE;
	PIOC->PIO_ODR  = 0x05000000;		//PIOC->PIO_ODR  = 0x14000000;
	PIOC->PIO_CODR = 0x060FF1FE;
	PIOC->PIO_CODR = 0x00800000;

	PIOD->PIO_PER  = 0X000001E10;		//PIOD->PIO_PER  = 0x0000000F;
	PIOD->PIO_OER  = 0x0000006F;
	//PIOD->PIO_SODR = 0x0000000F;
	PIOD->PIO_PUER = 0x0000000F;
	
	gpio_configure_pin(PIN_CAN0_RX_IDX, PIN_CAN0_RX_FLAGS);
	gpio_configure_pin(PIN_CAN0_TX_IDX, PIN_CAN0_TX_FLAGS);
	gpio_configure_pin(PIN_CAN0_STANDBY_IDX, PIN_CAN0_STANDBY_FLAGS);
	





}

uint8_t CPU3_D_SET_PIO(pio_type peripheral,uint32_t pio)
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

uint8_t CPU3_D_RESET_PIO(const pio_type peripheral,uint32_t pio)
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
  
  uint16_t CPU3_D_READ_PIO( pio_type peripheral,uint32_t pio)
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