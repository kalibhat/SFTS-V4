/*
 * DD_GPIO.c
 *
 * Created: 1/6/2014 10:45:11 AM
 *  Author: Gopalkrishna Kalibhat
 */ 
#include "DD_GPIO.h"
#include "cl_types.h"
#include "cl_consolecontroller.h"
extern Cl_ReturnCodeType  Cl_SendDatatoconsole(Cl_ConsoleTxCommandtype , uint8_t* ,uint8_t );


void DD_INIT_PIO()
{
	
	
PIOA->PIO_WPMR = 0x50494F00;			//PIOA->PIO_WPMR = 0x50494F00;
//PIOA->PIO_PER =  0x21C9ECFC;			//PIOA->PIO_PER  = 0x01C9ECF0;
PIOA->PIO_PER =  0x21C9ECFC;
PIOA->PIO_OER =  0x20F0B86C;			//PIOA->PIO_OER  = 0x00C0AC70;
PIOA->PIO_ODR =  0x01094494;			//PIOA->PIO_ODR  = 0x01094080;
PIOA->PIO_PDR =  0x1E361303;			//PIOA->PIO_PDR =  0x00060000;
PIOA->PIO_PUER = 0x00060000;
PIOA->PIO_ABSR = 0x00391000;
PIOA->PIO_DIFSR =0x00004000;			 //Blood pump on off debouncing PA14 //PIOA->PIO_DIFSR = 0x08000001;
 PIOA->PIO_SODR = 1<<2;
 PIOA->PIO_PER = 1<<2;
 PIOA->PIO_OER = 1<<2;

//PIOA->PIO_CODR = 0x0600A800;
//PIOA->PIO_SODR = 0x00004000;

PIOB->PIO_WPMR = 0x50494F00;			//PIOB->PIO_WPMR = 0x50494F00;
PIOB->PIO_PER  = 0x0FE7F700;			//PIOB->PIO_PER  = 0x0B45D300;     PIO enable Register
PIOB->PIO_OER  = 0x00026400;			//PIOB->PIO_OER  = 0x00004000;     Output Enable Register
PIOB->PIO_ODR  = 0x0FE59300; // 0x0FE59300			//PIOB->PIO_ODR  = 0x0B459300;     Output Disable Register
PIOB->PIO_PDR  = 0xF01800FF;			// Pio Disable Register
//PIOB->PIO_SODR = 0x04004000;
//PIOB->PIO_CODR = 0x02000000;

PIOC->PIO_WPMR = 0x50494F00;			//PIOC->PIO_WPMR = 0x50494F00;
PIOC->PIO_PER  = 0x7F9FFFFE;			//PIOC->PIO_PER  = 0x7F9FFFFE;
PIOC->PIO_OER  = 0x18E1FFFE;			//PIOC->PIO_OER  = 0x1881FFFE;
PIOC->PIO_ODR  = 0x671E0000;			//PIOC->PIO_ODR  = 0x671E0000;
PIOC->PIO_PDR  = 0x80600001;			//
PIOC->PIO_ABSR = 0x00600000;
//PIOC->PIO_CODR = 0x360830C4;
//PIOC->PIO_SODR = 0x00000002;

PIOD->PIO_WPMR = 0x50494F00;			//PIOD->PIO_WPMR = 0x50494F00;
PIOD->PIO_PER  = 0x000007FF;			//PIOD->PIO_PER  = 0x000007FF;
PIOD->PIO_OER  = 0x00000186;			//PIOD->PIO_OER  = 0x00000106;
PIOD->PIO_ODR  = 0X00000679;			//PIOD->PIO_ODR  = 0x000006F9;
//PIOD->PIO_CODR = 0x00000186;
//PIOD->PIO_SODR = 0x00000400;



//p_io = 21;
//peri = PC;
//DD_RESET_PIO(PC,21);
//PIOA->PIO_WPMR = 0<2;
 PIOC->PIO_CODR = 1<<21;               
 PIOB->PIO_SODR = 1<<17;
 PIOD->PIO_SODR = 1<<2;
gpio_configure_pin(PIN_PWM_LED0_GPIO,PIN_PWM_LED0_FLAGS);
gpio_configure_pin(PIN_PWM_LED0_GPIO,PIN_PWM_LED0_FLAGS);


gpio_configure_pin(PIN_CAN0_RX_IDX, PIN_CAN0_RX_FLAGS);
gpio_configure_pin(PIN_CAN0_TX_IDX, PIN_CAN0_TX_FLAGS);
gpio_configure_pin(PIN_CAN0_STANDBY_IDX, PIN_CAN0_STANDBY_FLAGS);


}

uint8_t DD_DIS_PERI_AND_RESET_PIO(pio_type peripheral,uint32_t pio)
{
				if((peripheral == PD) && ((pio == 7)||(pio == 8) ))
				{
				//	Cl_SendDatatoconsole(CON_TX_COMMAND_PRINTTEXT,"PD_7_8",6);
				}
	switch (peripheral)
	{
		case PA:
		PIOA->PIO_PER = 1<<pio;
		PIOA->PIO_CODR = 1<<pio;
		break;
		case PB:
		PIOB->PIO_PER = 1<<pio;
		PIOB->PIO_CODR = 1<<pio;
		break;
		case PC:
		PIOC->PIO_PER = 1<<pio;
		PIOC->PIO_CODR = 1<<pio;
		break;
		case PD:
		PIOD->PIO_PER = 1<<pio;
		PIOD->PIO_CODR = 1<<pio;
		break;
			
	}
}

uint8_t DD_EN_PERI_AND_SET_PIO(pio_type peripheral,uint32_t pio)
{
			if((peripheral == PD) && ((pio == 7)||(pio == 8) ))
		{
		//	Cl_SendDatatoconsole(CON_TX_COMMAND_PRINTTEXT,"PD_7_8",6);
		}
	switch (peripheral)
	{
		case PA:
		PIOA->PIO_PDR = 1<<pio;
	//	PIOA->PIO_SCDR = 1<<pio;
		break;
		case PB:
		PIOB->PIO_PDR = 1<<pio;
	//	PIOB->PIO_SODR = 1<<pio;
		break;
		case PC:
		PIOC->PIO_PDR = 1<<pio;
	//	PIOC->PIO_CODR = 1<<pio;
		break;
		case PD:
		PIOD->PIO_PDR = 1<<pio;
	//	PIOD->PIO_CODR = 1<<pio;
		break;
		

		
	}
}

uint8_t DD_SET_PIO(pio_type peripheral,uint32_t pio)
{
					if((peripheral == PD) && ((pio == 7)||(pio == 8) ))
				{
				//	Cl_SendDatatoconsole(CON_TX_COMMAND_PRINTTEXT,"PD_7_8",6);
				}
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

uint8_t DD_RESET_PIO(const pio_type peripheral,uint32_t pio)
  {
	  		  		if((peripheral == PD) && ((pio == 7)||(pio == 8) ))
		  		{
			//  		Cl_SendDatatoconsole(CON_TX_COMMAND_PRINTTEXT,"PD_7_8",6);
		  		}
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
  
  uint16_t DD_READ_PIO( pio_type peripheral,uint32_t pio)
  {
	  uint32_t read = 0;
	  uint16_t ret_val=0;
	  /*read= PIOC->PIO_ODSR;
	  printf("%d",pio);
	  printf("%d", read);*/
	  switch (peripheral)
	  {
		  case PA:
		  
		  if( (PIOA->PIO_PDSR & (1<<pio)) == 0)
		  {
			//  printf("A0");
			  return 0;
		  }
		  else
		  {
		//	  printf("A1");
			  return 1;
		  } 
		  break;
		  case PB:
		  if( (PIOB->PIO_PDSR & (1<<pio)) == 0)
		  {
		//	  printf("B0");
			  return 0;
		  }
		  else
		  {
			//  printf("B1");
			  return 1;
		  }
		  break;
		  case PC:
		 //   if(pio == 15)
		//	{
				
		//	read = PIOC->PIO_PDSR;
		//	if(read & pio)
		//		{
			//	printf("c");
			//	}
		//	}
			
		  if( (PIOC->PIO_PDSR & (1<<pio)) == 0)
		  {
			  
			
			//  printf("C0");
			  return 0;
		  }
		  else
		  {
			//  printf("C1");
			 // return 1;
			 ret_val = 1;
		//	 				printf("%d",ret_val);
			 return(ret_val);
			 
		  }
		  break;
		  case PD:
		  if( (PIOD->PIO_PDSR & (1<<pio)) == 0)
		  {
			//  printf("D0");
			  return 0;
		  }
		  else
		  {
			//  printf("D1");
			  return 1;
		  }
		  break;
		  
	  }
	  
	
  }
