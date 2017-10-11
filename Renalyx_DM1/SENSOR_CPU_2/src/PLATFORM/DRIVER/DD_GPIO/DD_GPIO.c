/*
 * DD_GPIO.c
 *
 * Created: 3/7/2017 11:31:50 AM
 *  Author: Electrical
 */ 
#include "DD_GPIO.h"

void SENSOR_D_INIT_PIO()	{	
	
	PIOA->PIO_WPMR = 0x50494F00;
	PIOB->PIO_WPMR = 0x50494F00;
	PIOC->PIO_WPMR = 0x50494F00;
	PIOD->PIO_WPMR = 0x50494F00;
	dd_SPI_pins_INIT();
	dd_CAN_pins_INIT();
	dd_UART_pins_init() ;
	dd_TEMP_SENSOR_PINS_INIT();
	dd_PRESSURE_SENSOR_GPIO_PINS();
	PIOA->PIO_ABSR  = 0x0;
	PIOB->PIO_ABSR = 0x00200000;
}

void dd_PRESSURE_SENSOR_GPIO_PINS()   {
	dd_enable_PIO(PIOC, PD_PS1_SEN_PIN);
	dd_enable_OUTPUT(PIOC, PD_PS1_SEN_PIN);
	dd_enable_PULLUP(PIOC, PD_PS1_SEN_PIN);
	dd_SET_OUTPUT_DATA(PIOC, PD_PS1_SEN_PIN);
	dd_enable_OUTPUT_WRITE(PIOC, PD_PS1_SEN_PIN);
	
	
	dd_enable_PIO(PIOC, PD_PS2_SEN_PIN);
	dd_enable_OUTPUT(PIOC, PD_PS2_SEN_PIN);
	dd_enable_PULLUP(PIOC, PD_PS2_SEN_PIN);
	dd_SET_OUTPUT_DATA(PIOC, PD_PS2_SEN_PIN);
	dd_enable_OUTPUT_WRITE(PIOC, PD_PS2_SEN_PIN);
	
	
	dd_enable_PIO(PIOC, PD_PS3_SEN_PIN);
	dd_enable_OUTPUT(PIOC, PD_PS3_SEN_PIN);
	dd_enable_PULLUP(PIOC, PD_PS3_SEN_PIN);
	dd_SET_OUTPUT_DATA(PIOC, PD_PS3_SEN_PIN);
	dd_enable_OUTPUT_WRITE(PIOC, PD_PS3_SEN_PIN);
	
	dd_enable_PIO(PIOD, PS1_DRDY_N_PIN);
	dd_disable_OUTPUT(PIOD,PS1_DRDY_N_PIN );
	dd_CLEAR_OUTPUT_DATA(PIOD,PS1_DRDY_N_PIN );
	dd_PIO_INTERRUPT_DISABLE(PIOD,PS1_DRDY_N_PIN );
	dd_disable_OUTPUT_WRITE(PIOD,PS1_DRDY_N_PIN );
	
	dd_enable_PIO(PIOD, PS2_DRDY_N_PIN);
	dd_disable_OUTPUT(PIOD,PS2_DRDY_N_PIN );
	dd_CLEAR_OUTPUT_DATA(PIOD,PS2_DRDY_N_PIN );
	dd_PIO_INTERRUPT_DISABLE(PIOD,PS2_DRDY_N_PIN );
	dd_disable_OUTPUT_WRITE(PIOD,PS2_DRDY_N_PIN );
	
	dd_enable_PIO(PIOD, PS3_DRDY_N_PIN);
	dd_disable_OUTPUT(PIOD,PS3_DRDY_N_PIN );
	dd_CLEAR_OUTPUT_DATA(PIOD,PS3_DRDY_N_PIN );
	dd_PIO_INTERRUPT_DISABLE(PIOD,PS3_DRDY_N_PIN );
	dd_disable_OUTPUT_WRITE(PIOD,PS3_DRDY_N_PIN );	
}

void dd_TEMP_SENSOR_PINS_INIT()   {
	
	dd_enable_PIO(PIOA, DRDY_TS1_PIN);
	dd_enable_PIO(PIOA, DRDY_TS2_PIN);
	dd_enable_PIO(PIOA, DRDY_TS3_PIN);
	dd_enable_PIO(PIOA, DRDY_CS1_TS_PIN);
	dd_enable_PIO(PIOA, DRDY_CS2_TS_PIN);
	dd_enable_PIO(PIOA, DRDY_CS3_TS_PIN);
	dd_enable_PIO(PIOA, THERMOCOUPLE_DRDY_PIN);	
 	
	dd_disable_OUTPUT(PIOA,DRDY_TS1_PIN );
	dd_disable_OUTPUT(PIOA,DRDY_TS2_PIN );
	dd_disable_OUTPUT(PIOA,DRDY_TS3_PIN );
	dd_disable_OUTPUT(PIOA,DRDY_CS1_TS_PIN );
	dd_disable_OUTPUT(PIOA,DRDY_CS2_TS_PIN );
	dd_disable_OUTPUT(PIOA,DRDY_CS3_TS_PIN );
	dd_disable_OUTPUT(PIOA,THERMOCOUPLE_DRDY_PIN );
	
	dd_CLEAR_OUTPUT_DATA(PIOA,DRDY_TS1_PIN );
	dd_CLEAR_OUTPUT_DATA(PIOA,DRDY_TS2_PIN );
	dd_CLEAR_OUTPUT_DATA(PIOA,DRDY_TS3_PIN );
	dd_CLEAR_OUTPUT_DATA(PIOA,DRDY_CS1_TS_PIN );
	dd_CLEAR_OUTPUT_DATA(PIOA,DRDY_CS2_TS_PIN );
	dd_CLEAR_OUTPUT_DATA(PIOA,DRDY_CS3_TS_PIN );
	dd_CLEAR_OUTPUT_DATA(PIOA,THERMOCOUPLE_DRDY_PIN );
	
	dd_PIO_INTERRUPT_DISABLE(PIOA,DRDY_TS1_PIN );
	dd_PIO_INTERRUPT_DISABLE(PIOA,DRDY_TS2_PIN );
	dd_PIO_INTERRUPT_DISABLE(PIOA,DRDY_TS3_PIN );
	dd_PIO_INTERRUPT_DISABLE(PIOA,DRDY_CS1_TS_PIN );
	dd_PIO_INTERRUPT_DISABLE(PIOA,DRDY_CS2_TS_PIN );
	dd_PIO_INTERRUPT_DISABLE(PIOA,DRDY_CS3_TS_PIN );
	dd_PIO_INTERRUPT_DISABLE(PIOA,THERMOCOUPLE_DRDY_PIN );
	
	dd_disable_OUTPUT_WRITE(PIOA,DRDY_TS1_PIN );
	dd_disable_OUTPUT_WRITE(PIOA,DRDY_TS2_PIN );
	dd_disable_OUTPUT_WRITE(PIOA,DRDY_TS3_PIN );
	dd_disable_OUTPUT_WRITE(PIOA,DRDY_CS1_TS_PIN );
	dd_disable_OUTPUT_WRITE(PIOA,DRDY_CS2_TS_PIN );
	dd_disable_OUTPUT_WRITE(PIOA,DRDY_CS3_TS_PIN );
	dd_disable_OUTPUT_WRITE(PIOA,THERMOCOUPLE_DRDY_PIN );	
}
	
	
	
void dd_SPI_pins_INIT()  {
		
		dd_disable_PIO(PIOA, SPI0_MOSI_PIN);
		dd_disable_PIO(PIOA, SPI0_MISO_PIN);
		dd_disable_PIO(PIOA, SPI0_SCLK_PIN);
		dd_disable_PIO(PIOA, SPI0_NPCS0_PIN);
		dd_disable_PIO(PIOA, SPI0_NPCS1_PIN);
		
		dd_disable_PIO(PIOB, SPI0_NPCS2_PIN);
		dd_disable_PIO(PIOB, SPI0_NPCS3_PIN);
	
		
// 		dd_enable_PIO(PIOA, SPI0_MISO_PIN);
// 		dd_disable_OUTPUT(PIOA, SPI0_MISO_PIN);
		
		
		
		//PIOA->PIO_MDER = 1u<<25;
		
// 		dd_disable_OUTPUT(PIOA, SPI0_MOSI_PIN);
// 		
// 		dd_disable_OUTPUT(PIOA, SPI0_SCLK_PIN);
// 		dd_disable_OUTPUT(PIOA, SPI0_NPCS0_PIN);
// 		dd_disable_OUTPUT(PIOA, SPI0_NPCS1_PIN);
// 		dd_disable_OUTPUT(PIOA, SPI0_NPCS2_PIN);
// 		dd_disable_OUTPUT(PIOA, SPI0_NPCS3_PIN);
		
// 		dd_CLEAR_OUTPUT_DATA(PIOA, SPI0_MOSI_PIN);
// 		dd_CLEAR_OUTPUT_DATA(PIOA, SPI0_MISO_PIN);
// 		dd_CLEAR_OUTPUT_DATA(PIOA, SPI0_SCLK_PIN);
// 		dd_CLEAR_OUTPUT_DATA(PIOA, SPI0_NPCS0_PIN);
// 		dd_CLEAR_OUTPUT_DATA(PIOA, SPI0_NPCS1_PIN);
// 		dd_CLEAR_OUTPUT_DATA(PIOA, SPI0_NPCS2_PIN);
// 		dd_CLEAR_OUTPUT_DATA(PIOA, SPI0_NPCS3_PIN);
		
// 		dd_enable_PULLUP(PIOA, SPI0_MOSI_PIN);
// 		dd_enable_PULLUP(PIOA, SPI0_MISO_PIN);
// 		dd_enable_PULLUP(PIOA, SPI0_SCLK_PIN);
// 		dd_enable_PULLUP(PIOA, SPI0_NPCS0_PIN);
// 		dd_enable_PULLUP(PIOA, SPI0_NPCS1_PIN);
// 		dd_enable_PULLUP(PIOA, SPI0_NPCS2_PIN);
// 		dd_enable_PULLUP(PIOA, SPI0_NPCS3_PIN);
		
// 		dd_disable_OUTPUT_WRITE(PIOA, SPI0_MOSI_PIN);
// 		//dd_disable_OUTPUT_WRITE(PIOA, SPI0_MISO_PIN);
// 		dd_disable_OUTPUT_WRITE(PIOA, SPI0_SCLK_PIN);
// 		dd_disable_OUTPUT_WRITE(PIOA, SPI0_NPCS0_PIN);
// 		dd_disable_OUTPUT_WRITE(PIOA, SPI0_NPCS1_PIN);
// 		dd_disable_OUTPUT_WRITE(PIOA, SPI0_NPCS2_PIN);
// 		dd_disable_OUTPUT_WRITE(PIOA, SPI0_NPCS3_PIN);
		
		
		
// 		dd_disable_OUTPUT(PIOB, SPI0_NPCS2_PIN);
// 		dd_disable_OUTPUT(PIOB, SPI0_NPCS3_PIN);
// 		
// 		dd_CLEAR_OUTPUT_DATA(PIOB, SPI0_NPCS2_PIN);
// 		dd_CLEAR_OUTPUT_DATA(PIOB, SPI0_NPCS3_PIN);
// 		
// 		dd_disable_OUTPUT_WRITE(PIOB, SPI0_NPCS2_PIN);
// 		dd_disable_OUTPUT_WRITE(PIOB, SPI0_NPCS3_PIN);
// 		
// 		dd_enable_PULLUP(PIOB, SPI0_NPCS2_PIN);
// 		dd_enable_PULLUP(PIOB, SPI0_NPCS3_PIN);		
		
		
		
		dd_enable_PIO(PIOA, SPI0_MISO_ISOLATER_1_DISABLE_PIN);
		dd_enable_OUTPUT(PIOA, SPI0_MISO_ISOLATER_1_DISABLE_PIN);
		dd_enable_PULLUP(PIOA, SPI0_MISO_ISOLATER_1_DISABLE_PIN);
		dd_SET_OUTPUT_DATA(PIOA, SPI0_MISO_ISOLATER_1_DISABLE_PIN);
		dd_enable_OUTPUT_WRITE(PIOA, SPI0_MISO_ISOLATER_1_DISABLE_PIN);
		
		dd_enable_PIO(PIOA, SPI0_MISO_ISOLATER_2_DISABLE_PIN);
		dd_enable_OUTPUT(PIOA, SPI0_MISO_ISOLATER_2_DISABLE_PIN);
		dd_enable_PULLUP(PIOA, SPI0_MISO_ISOLATER_2_DISABLE_PIN);
		dd_SET_OUTPUT_DATA(PIOA, SPI0_MISO_ISOLATER_2_DISABLE_PIN);
		dd_enable_OUTPUT_WRITE(PIOA, SPI0_MISO_ISOLATER_2_DISABLE_PIN);
		
		dd_enable_PIO(PIOA, SPI0_MISO_ISOLATER_3_DISABLE_PIN);
		dd_enable_OUTPUT(PIOA, SPI0_MISO_ISOLATER_3_DISABLE_PIN);
		dd_enable_PULLUP(PIOA, SPI0_MISO_ISOLATER_3_DISABLE_PIN);
		dd_SET_OUTPUT_DATA(PIOA, SPI0_MISO_ISOLATER_3_DISABLE_PIN);
		dd_enable_OUTPUT_WRITE(PIOA, SPI0_MISO_ISOLATER_3_DISABLE_PIN);
		
		dd_enable_PIO(PIOA, SPI0_MISO_ISOLATER_4_DISABLE_PIN);
		dd_enable_OUTPUT(PIOA, SPI0_MISO_ISOLATER_4_DISABLE_PIN);
		dd_enable_PULLUP(PIOA, SPI0_MISO_ISOLATER_4_DISABLE_PIN);
		dd_SET_OUTPUT_DATA(PIOA, SPI0_MISO_ISOLATER_4_DISABLE_PIN);
		dd_enable_OUTPUT_WRITE(PIOA, SPI0_MISO_ISOLATER_4_DISABLE_PIN);
		
		dd_enable_PIO(PIOB, SPIO_NPCS_G2A_PIN);
		dd_enable_OUTPUT(PIOB, SPIO_NPCS_G2A_PIN);
		dd_enable_PULLUP(PIOB, SPIO_NPCS_G2A_PIN);
		dd_CLEAR_OUTPUT_DATA(PIOB, SPIO_NPCS_G2A_PIN);
		dd_enable_OUTPUT_WRITE(PIOB, SPIO_NPCS_G2A_PIN);
		
		dd_enable_PIO(PIOB, SPIO_NPCS_G2B_PIN);
		dd_enable_OUTPUT(PIOB, SPIO_NPCS_G2B_PIN);
		dd_enable_PULLUP(PIOB, SPIO_NPCS_G2B_PIN);
		dd_CLEAR_OUTPUT_DATA(PIOB, SPIO_NPCS_G2B_PIN);
		dd_enable_OUTPUT_WRITE(PIOB, SPIO_NPCS_G2B_PIN);
		
		dd_enable_PIO(PIOB, SPIO_NPCS_G1_PIN);
		dd_enable_OUTPUT(PIOB, SPIO_NPCS_G1_PIN);
		dd_enable_PULLUP(PIOB, SPIO_NPCS_G1_PIN);
		dd_CLEAR_OUTPUT_DATA(PIOB, SPIO_NPCS_G1_PIN);		
		dd_enable_OUTPUT_WRITE(PIOB, SPIO_NPCS_G1_PIN);	
		
		
		dd_enable_PIO(PIOB, ISOLTR_1_SSA0_PIN);
		dd_enable_OUTPUT(PIOB, ISOLTR_1_SSA0_PIN);
		dd_enable_PULLUP(PIOB, ISOLTR_1_SSA0_PIN);
		dd_enable_OUTPUT_WRITE(PIOB, ISOLTR_1_SSA0_PIN);
		dd_CLEAR_OUTPUT_DATA(PIOB, ISOLTR_1_SSA0_PIN);
		
		dd_enable_PIO(PIOB, ISOLTR_1_SSA1_PIN);
		dd_enable_OUTPUT(PIOB, ISOLTR_1_SSA1_PIN);
		dd_enable_PULLUP(PIOB, ISOLTR_1_SSA1_PIN);
		dd_enable_OUTPUT_WRITE(PIOB, ISOLTR_1_SSA1_PIN);
		dd_CLEAR_OUTPUT_DATA(PIOB, ISOLTR_1_SSA1_PIN);
		
		dd_enable_PIO(PIOB, ISOLTR_2_SSA0_PIN);
		dd_enable_OUTPUT(PIOB, ISOLTR_2_SSA0_PIN);
		dd_enable_PULLUP(PIOB, ISOLTR_2_SSA0_PIN);
		dd_enable_OUTPUT_WRITE(PIOB, ISOLTR_2_SSA0_PIN);
		dd_CLEAR_OUTPUT_DATA(PIOB, ISOLTR_2_SSA0_PIN);
		
		dd_enable_PIO(PIOB, ISOLTR_2_SSA1_PIN);
		dd_enable_OUTPUT(PIOB, ISOLTR_2_SSA1_PIN);
		dd_enable_PULLUP(PIOB, ISOLTR_2_SSA1_PIN);
		dd_enable_OUTPUT_WRITE(PIOB, ISOLTR_2_SSA1_PIN);
		dd_CLEAR_OUTPUT_DATA(PIOB, ISOLTR_2_SSA1_PIN);
		
		dd_enable_PIO(PIOB, ISOLTR_3_SSA0_PIN);
		dd_enable_OUTPUT(PIOB, ISOLTR_3_SSA0_PIN);
		dd_enable_PULLUP(PIOB, ISOLTR_3_SSA0_PIN);
		dd_enable_OUTPUT_WRITE(PIOB, ISOLTR_3_SSA0_PIN);
		dd_CLEAR_OUTPUT_DATA(PIOB, ISOLTR_3_SSA0_PIN);
		
		dd_enable_PIO(PIOB, ISOLTR_3_SSA1_PIN);
		dd_enable_OUTPUT(PIOB, ISOLTR_3_SSA1_PIN);
		dd_enable_PULLUP(PIOB, ISOLTR_3_SSA1_PIN);
		dd_enable_OUTPUT_WRITE(PIOB, ISOLTR_3_SSA1_PIN);
		dd_CLEAR_OUTPUT_DATA(PIOB, ISOLTR_3_SSA1_PIN);
		
		dd_enable_PIO(PIOB, ISOLTR_4_SSA0_PIN);
		dd_enable_OUTPUT(PIOB, ISOLTR_4_SSA0_PIN);
		dd_enable_PULLUP(PIOB, ISOLTR_4_SSA0_PIN);
		dd_enable_OUTPUT_WRITE(PIOB, ISOLTR_4_SSA0_PIN);
		dd_CLEAR_OUTPUT_DATA(PIOB, ISOLTR_4_SSA0_PIN);
		
		dd_enable_PIO(PIOB, ISOLTR_4_SSA1_PIN);
		dd_enable_OUTPUT(PIOB, ISOLTR_4_SSA1_PIN);
		dd_enable_PULLUP(PIOB, ISOLTR_4_SSA1_PIN);
		dd_enable_OUTPUT_WRITE(PIOB, ISOLTR_4_SSA1_PIN);
		dd_CLEAR_OUTPUT_DATA(PIOB, ISOLTR_4_SSA1_PIN);		
	}
	
void dd_CAN_pins_INIT()  {
	dd_disable_PIO(PIOA, CAN_RX_PIN);
	dd_disable_OUTPUT(PIOA, CAN_RX_PIN);
	dd_CLEAR_OUTPUT_DATA(PIOA, CAN_RX_PIN);
	dd_disable_OUTPUT_WRITE(PIOA, CAN_RX_PIN);
	dd_disable_PULLUP(PIOA, CAN_RX_PIN);
	
	
	dd_disable_PIO(PIOA, CAN_TX_PIN);	
	dd_disable_OUTPUT(PIOA, CAN_TX_PIN);	
	dd_CLEAR_OUTPUT_DATA(PIOA, CAN_TX_PIN);	
	dd_disable_OUTPUT_WRITE(PIOA, CAN_TX_PIN);	
	dd_disable_PULLUP(PIOA, CAN_TX_PIN);
	
	dd_enable_PIO(PIOA, CAN_STB_PIN);
	dd_enable_OUTPUT(PIOA, CAN_STB_PIN);
	dd_CLEAR_OUTPUT_DATA(PIOA, CAN_STB_PIN);
	dd_enable_OUTPUT_WRITE(PIOA, CAN_STB_PIN);
	dd_enable_PULLUP(PIOA, CAN_STB_PIN);	
}
	
void dd_UART_pins_init()  {
	dd_disable_PIO(PIOA, UART_RX_PIN);
	dd_disable_OUTPUT(PIOA, UART_RX_PIN);
	dd_CLEAR_OUTPUT_DATA(PIOA, UART_RX_PIN);
	dd_disable_OUTPUT_WRITE(PIOA, UART_RX_PIN);
	dd_disable_PULLUP(PIOA, UART_RX_PIN);
	
	dd_disable_PIO(PIOA, UART_TX_PIN);	
	dd_disable_OUTPUT(PIOA, UART_TX_PIN);	
	dd_CLEAR_OUTPUT_DATA(PIOA, UART_TX_PIN);	
	dd_disable_OUTPUT_WRITE(PIOA, UART_TX_PIN);	
	dd_disable_PULLUP(PIOA, UART_TX_PIN);	
}
	
	
	
	
	
	
	
	
	//0x00F0 0000
	
	
	
	
	
	
	
	/*
	PIOA->PIO_PDR = 0x3;
	
	PIOA->PIO_PDR = 0X7E000000;  //enable spi0 gpio
	PIOA->PIO_PER= 0X8000;      // 
	PIOA->PIO_ODR = 0X8000;
	
	PIOA->PIO_PDR = 0x3;  //enable can gpio
	PIOA->PIO_OER = 0X4;
	//PIOA->PIO_PER= 0X8000;*/
	
	
	
	
/*	
	PIOA->PIO_PER = 0X003FC004;
	PIOB->PIO_PER = 0X000141FF;
	PIOC->PIO_PER = 0X000001DE;
	
	PIOA->PIO_OER = 0X00000004;
	PIOB->PIO_OER = 0X00000004;
	PIOB->PIO_OER = 0X000141FF; 
	
	PIOB->PIO_ABSR = 0XA00000;
	
	PIOA->PIO_ODR = 0X3FC000;  
	PIOC->PIO_ODR = 0X18;
	PIOD->PIO_ODR = 0X7;
	
	//PIOB->PIO_ODR = 0X00000000;
//	0
	
	
	
	//PIOA->PIO_ABSR = 0 ;
	//PIOB->PIO_ABSR =// 0X00A00000;
	
//	PIOA->PIO_ODR = 0X00C03F00;  //GPIO I/P
//	PIOB->PIO_ODR = 0X00000000;
//	PIOC->PIO_ODR = 0X00000018;0
//	
	
	
	
	
	
	
	
	
//	
//	 //GPIO O/P
//	PIOC->PIO_OER  =0X000001C6;
	
//	PIOA->PIO_PER = 0X00000004;
	
	
	
	
	
	//PIOC->PIO_PER  =0X000001C6;
	
//	PIOA->PIO_PUER = 0X00000004;
//	PIOB->PIO_PUER = 0X000141FF;
//	PIOC->PIO_PUER  =0X000001C6;
	*/



uint32_t dd_PIO_INTERRUPT_DISABLE(Pio *p_pio, uint32_t pin_no)	{
	
	if ((p_pio->PIO_ISR & (1u << pin_no)) == (1u << pin_no)) {
		p_pio->PIO_IDR = 1 << pin_no;
	}
	return 0;
}


uint32_t dd_PIO_INTERRUPT_ENABLE(Pio *p_pio, uint32_t pin_no)	{
	
	if ((p_pio->PIO_ISR & (1u << pin_no)) != (1u << pin_no)) {
		p_pio->PIO_IER = 1 << pin_no;
	}
	return 0;
}

uint32_t dd_enable_PIO(Pio *p_pio, uint32_t pin_no)	{
	
		//if ((p_pio->PIO_PSR & (1u << pin_no)) != (1u << pin_no)) {
			p_pio->PIO_PER = 1 << pin_no;
	//	}		
	return 0;
}

uint32_t dd_disable_PIO(Pio *p_pio, uint32_t pin_no)	{
	
	//if ((p_pio->PIO_PSR & (1u << pin_no)) == (1u << pin_no)) {
		p_pio->PIO_PDR = 1 << pin_no;
	//}
	return 0;
}

uint32_t dd_enable_OUTPUT(Pio *p_pio, uint32_t pin_no)	{
	
	//if ((p_pio->PIO_OSR & (1u << pin_no)) != (1u << pin_no)) {
		p_pio->PIO_OER = 1 << pin_no;
	//}
	return 0;
}

uint32_t dd_disable_OUTPUT(Pio *p_pio, uint32_t pin_no)	{
	
	//if ((p_pio->PIO_OSR & (1u << pin_no)) == (1u << pin_no)) {
		p_pio->PIO_ODR = 1 << pin_no;
	//}
	return 0;
}


uint32_t dd_enable_GLITCH_INPUT_FILTER(Pio *p_pio, uint32_t pin_no)	{
	
	if ((p_pio->PIO_IFSR & (1u << pin_no)) != (1u << pin_no)) {
		p_pio->PIO_IFER = 1 << pin_no;
	}
	return 0;
}

uint32_t dd_disable_GLITCH_INPUT_FILTER(Pio *p_pio, uint32_t pin_no)	{
	
	if ((p_pio->PIO_IFSR & (1u << pin_no)) == (1u << pin_no)) {
		p_pio->PIO_IFDR = 1 << pin_no;
	}
	return 0;
}



uint32_t dd_SET_OUTPUT_DATA(Pio *p_pio, uint32_t pin_no)	{
	
	//if ((p_pio->PIO_ODSR & (1u << pin_no)) != (1u << pin_no)) {
		p_pio->PIO_SODR = 1 << pin_no;
//	}
	return 0;
}

uint32_t dd_CLEAR_OUTPUT_DATA(Pio *p_pio, uint32_t pin_no)	{
	
	//if ((p_pio->PIO_ODSR & (1u << pin_no)) == (1u << pin_no)) {
		p_pio->PIO_CODR = 1 << pin_no;
	//}
	return 0;
}

uint32_t dd_enable_PULLUP(Pio *p_pio, uint32_t pin_no)	{
	
	//if ((p_pio->PIO_PUSR & (1u << pin_no)) != (1u << pin_no)) {
		p_pio->PIO_PUER = 1 << pin_no;
//	}
	return 0;
}

uint32_t dd_disable_PULLUP(Pio *p_pio, uint32_t pin_no)	{	
	
	//if ((p_pio->PIO_PUSR & (1u << pin_no)) == (1u << pin_no)) {
		p_pio->PIO_PUDR = 1 << pin_no;
	
	return 0;
}

uint32_t dd_enable_OUTPUT_WRITE(Pio *p_pio, uint32_t pin_no)	{	
	
//	if ((p_pio->PIO_OWSR & (1u << pin_no)) != (1u << pin_no)) {
		p_pio->PIO_OWER = 1 << pin_no;
//	}
	return 0;
}

uint32_t dd_disable_OUTPUT_WRITE(Pio *p_pio, uint32_t pin_no)	{	
	
	//if ((p_pio->PIO_OWSR & (1u << pin_no)) == (1u << pin_no)) {
		p_pio->PIO_OWDR = 1 << pin_no;
	//}
	return 0;
}

uint32_t dd_PERIPHERAL_SELECTION(Pio *p_pio , uint32_t AorB_REG)	{
	uint32_t ab_status;
	ab_status = p_pio->PIO_ABSR ;
	p_pio->PIO_ABSR =  (ab_status | AorB_REG);
	
	return 0;
}





