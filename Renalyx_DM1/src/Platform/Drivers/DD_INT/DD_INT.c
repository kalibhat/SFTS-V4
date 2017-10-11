/*
 * DD_INT.c
 *
 * Created: 1/7/2014 5:57:40 PM
 *  Author: Geekay
 */ 

#include "DD_INT.h"
#include "cl_app/cl_bc_cntrl/inc/Cl_bc_controller.h"
#include "cl_app/inc/cl_types.h"
#include "sv_interface.h"

#include "cl_app/cl_mac/inc/cl_mac_controller.h"
void pin_edge_handler(const uint32_t id, const uint32_t index);
extern Cl_ReturnCodeType  Cl_bc_controller(Cl_BC_EventType cl_bc_event);
void check_interrupt(void);
extern uint8_t sv_cntrl_activate_valve(sv_valvetype sv_valve_id);
extern uint8_t sv_cntrl_deactivate_valve(sv_valvetype sv_valve_id);
volatile bool chk_flag_toggle =0;

extern Cl_Mac_EventQueueType Cl_MacEventQueue1;


void DD_INIT_INT()
{
	
	
	NVIC_DisableIRQ(UART_IRQn);
	NVIC_ClearPendingIRQ(UART_IRQn);
	NVIC_SetPriority(UART_IRQn, 0);
	NVIC_EnableIRQ(UART_IRQn);
	uart_enable_interrupt(UART,UART_SR_RXRDY);
		
	NVIC_DisableIRQ(TC0_IRQn);
	NVIC_ClearPendingIRQ(TC0_IRQn);
	NVIC_SetPriority(TC0_IRQn, 0);
	NVIC_EnableIRQ(TC0_IRQn);
/*
	NVIC_DisableIRQ(TC1_IRQn);
	NVIC_ClearPendingIRQ(TC1_IRQn);
	NVIC_SetPriority(TC1_IRQn, 0);
	
	NVIC_DisableIRQ(TC2_IRQn);
	NVIC_ClearPendingIRQ(TC2_IRQn);
	NVIC_SetPriority(TC2_IRQn, 0);*/
	
	NVIC_DisableIRQ(TC3_IRQn);
	NVIC_ClearPendingIRQ(TC3_IRQn);
	NVIC_SetPriority(TC3_IRQn,0);
	NVIC_EnableIRQ(TC3_IRQn);
	
/*	
	NVIC_DisableIRQ(TC4_IRQn);
	NVIC_ClearPendingIRQ(TC4_IRQn);
	NVIC_SetPriority(TC4_IRQn, 0);
	
	NVIC_DisableIRQ(TC5_IRQn);
	NVIC_ClearPendingIRQ(TC5_IRQn);
	NVIC_SetPriority(TC5_IRQn, 0);
	*/
	NVIC_DisableIRQ(TC6_IRQn);
	NVIC_ClearPendingIRQ(TC6_IRQn);
	NVIC_SetPriority(TC6_IRQn, 0);
	NVIC_EnableIRQ(TC6_IRQn);
/*	
	NVIC_DisableIRQ(TC7_IRQn);
	NVIC_ClearPendingIRQ(TC7_IRQn);
	NVIC_SetPriority(TC7_IRQn, 0);
	
	NVIC_DisableIRQ(TC8_IRQn);
	NVIC_ClearPendingIRQ(TC8_IRQn);
	NVIC_SetPriority(TC8_IRQn, 0);
*/
	
	
	NVIC->ICER[6] = 0xFFFFFFFF;
	NVIC->ICPR[6] = 0XFFFFFFFF;
	NVIC_SetPriority(TC6_IRQn, 0);
	//NVIC_EnableIRQ(TC6_IRQn);
	NVIC->ISER[6] = 0xFFFFFFFF;
	
	
	// FOR TIMER2 OF 100ms MEANS 10HZ FREQUENCY
	NVIC->ICER[3] = 0xFFFFFFFF;
	NVIC->ICPR[3] = 0XFFFFFFFF;
	//NVIC_DisableIRQ(TC3_IRQn);
	//NVIC_ClearPendingIRQ(TC3_IRQn);
	NVIC_SetPriority(TC3_IRQn, 0);
	//NVIC_EnableIRQ(TC3_IRQn);
	NVIC->ISER[3] = 0xFFFFFFFF;
	
	
// 		pmc_enable_periph_clk(ID_PIOB);
// 		//	pio_set_output(PIOA, PIO_PA23, LOW, DISABLE, ENABLE);
// 		pio_set_input(PIOB, PIO_PB12, PIO_PULLUP);
// 		pio_handler_set(PIOB, ID_PIOB, PIO_PB12, PIO_IT_RISE_EDGE, pin_edge_handler);
// 		pio_enable_interrupt(PIOB, PIO_PB12);
// 		NVIC_SetPriority(PIOB_IRQn, 15);
//  		NVIC_EnableIRQ(PIOB_IRQn);
		
		

}

// 	void pin_edge_handler(const uint32_t id, const uint32_t index)
		// 		// 	{
		// 		// 		
		// 		// // 		NVIC_DisableIRQ(SysTick_IRQn );
		// 		//  //		NVIC_DisableIRQ(PIOB_IRQn);
		// 		// 		
		// 		// 		NVIC_ClearPendingIRQ(PIOB_IRQn);
		// 		// 		//	if ((id == ID_PIOB) && (index == PIO_PB12))
		// 		// 		{
		// 		// 			
		// 		// 			
		// 		// 									Cl_MacEventQueue1.Cl_MacEventData[Cl_MacEventQueue1.Cl_MacNewEventcount][0]= 60;//FPCURRENTSTATUS;
		// 		// 		
		// 		// 									Cl_MacEventQueue1.Cl_MacEvent[Cl_MacEventQueue1.Cl_MacNewEventcount] = 47;//EVT_ALERT_CS ;
		// 		// 		
		// 		// 			if(( Cl_MacEventQueue1.Cl_MacNewEventcount <= 10 ))   //MAC_EVENT_COUNT_MAX
		// 		// 
		// 		// 
		// 		// 			{
		// 		// 				Cl_MacEventQueue1.Cl_MacNewEvent = true;
		// 		// 				Cl_MacEventQueue1.Cl_MacNewEventcount++;
		// 		// 			}
		// 		// 			
		// 		// 			
		// 		// 			
		// 		// 			
		// 		// 			//		if (pio_get(PIOB, PIO_TYPE_PIO_INPUT, PIO_PB12))
		// 		// 						//	Cl_bc_controller(BC_EVENT_CS);
		// 		// 			{
		// 		// //				chk_flag_toggle = 1;
		// 		// //				check_interrupt();
		// 		// 			}
		// 		// 			
		// 		// 		}
		// 		// // 		NVIC_EnableIRQ(SysTick_IRQn );
		// 		// //		NVIC_EnableIRQ(PIOB_IRQn);
		// 		// 		
		// 		// 	}
		// 		// 
		// 		// 
		// 		// 	void check_interrupt(void)
		// 		// 	{
		// 		// 		if (chk_flag_toggle)
		// 		// 		{
		// 		// 			sv_cntrl_activate_valve(19);
		// 		// 		}
		// 		// 		else
		// 		// 		sv_cntrl_deactivate_valve(19);
		// 		// 		
		// 		// 		
		// 		// 	}