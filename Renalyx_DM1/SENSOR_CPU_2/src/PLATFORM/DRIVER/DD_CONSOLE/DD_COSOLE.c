/*
 * DD_COSOLE.c
 *
 * Created: 3/6/2017 5:52:00 PM
 *  Author: Electrical
 */ 

#include "DD_CONSOLE.h"



/*const sam_uart_opt_t uart_options = {
	.ul_baudrate = 115200,
	.ul_chmode = 0x00,
	.ul_mck = 84000000
	
};*/

/*const usart_serial_options_t uart_serial_options = {
	.baudrate = 115200,
	.paritytype = CONF_UART_PARITY,
	.stopbits = true
};*/

void SENSOR_D_INIT_UART()
{
	
	//sysclk_enable_peripheral_clock(ID_UART);
	/*UART->UART_CR	= 0x00000050;
	UART->UART_MR	= 0x00000800;
	UART->UART_BRGR = 0x00000046;*/
		
	//uart_init(UART,&uart_options);
	const usart_serial_options_t uart_serial_options = {
		.baudrate = CONF_UART_BAUDRATE,
		.paritytype = CONF_UART_PARITY
	};
	stdio_serial_init(UART, &uart_serial_options);
	puts(STRING_HEADER);
	
}


uint32_t DD_TX_UART(uint8_t sv_consoleTxdatabyte)
{
	uint32_t uart_stat;
	uart_stat= uart_write(UART,sv_consoleTxdatabyte);
	
	return uart_stat;
}

uint8_t DD_RX_UART()
{
	uint8_t rx_char;
	
	//uart_read(UART, rx_char);
	rx_char=UART->UART_RHR;
	
	return rx_char;
}
