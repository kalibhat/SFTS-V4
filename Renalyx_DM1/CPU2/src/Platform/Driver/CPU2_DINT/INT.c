/*
 * INT.c
 *
 * Created: 12/2/2014 3:20:19 PM
 *  Author: wid7
 */ 

#include "INT.h"

#define BOARD_TWI_IRQn          TWI0_IRQn

void CPU2_D_INIT_INT()
{
	
	NVIC_DisableIRQ(BOARD_TWI_IRQn);
	NVIC_ClearPendingIRQ(BOARD_TWI_IRQn);
	NVIC_SetPriority(BOARD_TWI_IRQn, 0);
	NVIC_EnableIRQ(BOARD_TWI_IRQn);
	
	NVIC_DisableIRQ(UART_IRQn);
	NVIC_ClearPendingIRQ(UART_IRQn);
	NVIC_SetPriority(UART_IRQn,0);
	NVIC_EnableIRQ(UART_IRQn);
	uart_enable_interrupt(UART,UART_SR_RXRDY);
		
	NVIC_DisableIRQ(TC0_IRQn);
	NVIC_ClearPendingIRQ(TC0_IRQn);
	NVIC_SetPriority(TC0_IRQn,0);
	NVIC_EnableIRQ(TC0_IRQn);

	NVIC_DisableIRQ(TC1_IRQn);
	NVIC_ClearPendingIRQ(TC1_IRQn);
	NVIC_SetPriority(TC1_IRQn,0);
	NVIC_EnableIRQ(TC1_IRQn);
	
// 	NVIC_DisableIRQ(TC2_IRQn);
// 	NVIC_ClearPendingIRQ(TC2_IRQn);
// 	NVIC_SetPriority(TC2_IRQn,0);
// 	NVIC_EnableIRQ(TC2_IRQn);
	
	NVIC_DisableIRQ(TC3_IRQn);
	NVIC_ClearPendingIRQ(TC3_IRQn);
	NVIC_SetPriority(TC3_IRQn,0);
	NVIC_EnableIRQ(TC3_IRQn);
	
	NVIC_DisableIRQ(TC6_IRQn);
	NVIC_ClearPendingIRQ(TC6_IRQn);
	NVIC_SetPriority(TC6_IRQn,0);
	NVIC_EnableIRQ(TC6_IRQn);
	
}