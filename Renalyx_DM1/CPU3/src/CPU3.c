/**
 * \file
 *
 * \brief Empty user application template
 *
 */

/**
 * \mainpage User Application template doxygen documentation
 *
 * \par Empty user application template
 *
 * Bare minimum empty user application template
 *
 * \par Content
 *
 * -# Include the ASF header files (through asf.h)
 * -# Minimal main function that starts with a call to board_init()
 * -# "Insert application code here" comment
 *
 */

/*
 * Include header files for all drivers that have been imported from
 * Atmel Software Framework (ASF).
 */
#include "asf.h"
#include "CPU3.h"
#include "conf_board.h"
#include "conf_clock.h"
#include "stdio_serial.h"
#include "Platform/Service/sv_interface.h"
#include "Platform/Driver/Driver_call.h"
#include "Platform/Service/CNTRL/S_CNTRL.h"
#include "Platform/Service/STATUS/S_STUS.h"
#include "Platform/Service/TIMER/S_TIMER.h"
#define FOREVER		1



extern void CPU3_S_SYSTEM_INIT(void);
extern uint32_t acid_f_b;
extern uint32_t bicarb_f_b;
extern volatile uint8_t flag_buff,flag_buff1,flag_buff_dummy,test;
extern uint8_t data_buff[3];
extern uint32_t data_count;
extern volatile uint16_t count_start;
extern volatile uint16_t count_stop;
volatile uint32_t count=0,count1=0,count_2=0;
volatile uint32_t count_test=0,flag_test=0;
TwobyteData data_stream;
extern time_tick;
extern uint8_t OneSecFlag;
tick_time cpu3_time;
cpu3_state state_3 = CPU3_STANDBY;
openfill_state openfillstate = STATE_IDLE;
volatile uint8_t fflag=0,fflag_bic=0,flag_nrmal = 0,flag_bic=0;
cpu3_status sensor_status;
uint32_t cond1_v,cond1_c,cond2_v, cond2_c, cond1, cond2;
 uint16_t mode,ra,rc=0,bic_ra,bic_rc, Set_RC=0;
extern Master_cpu_command_type;
void cpu3_sensorstatus(void);
void start_acidpump(void);
void start_bicarpump(void);
void stop_pump(void);
void increment_fun_a(void);
void decrement_fun_a(void);
void increment_fun_b(void);
void decrement_fun_b(void);
void start_pump(void);

extern table array_acid[64];
extern table array_bicar[64];
extern sn65hvd530_ctrl_t  can0_transceiver ;
extern volatile uint32_t g_ul_recv_status ;
extern can_mb_conf_t can0_mailbox;
extern uint32_t TC1_cnt , TC2_cnt, Cnt_TC0_total , Cnt_TC1_total;
extern uint16_t User_setConductivity, conductivity, avg_conductivity,System_Conductivity; 
//extern uint16_t User_setConductivity; 
Sys_statusType sv_sys_statbuffer;
uint8_t Correction_Count=0;
static void configure_console(void)
{
	const usart_serial_options_t uart_serial_options = {
		.baudrate = 115200,
		.paritytype = UART_MR_PAR_NO
	};
	/* Configure console UART. */
	sysclk_enable_peripheral_clock(CONSOLE_UART_ID);
	stdio_serial_init(CONSOLE_UART, &uart_serial_options);
}
const usart_serial_options_t uart_serial_options = {
	.baudrate = 115200,
	.paritytype = CONF_UART_PARITY,
	.stopbits = true
};
uint8_t feedcount_flag = 0;
uint32_t acid_count = 0;
uint32_t bicarb_count = 0;
void UART_Handler()
{
	static uint8_t cnt=0;
	cnt++;
	//printf("l\n");
	uint32_t status;
	//static dummy_1=0;
	uint8_t rx_val;
	status= UART->UART_SR;
	if(status & 0x01)
	{
		rx_val=UART->UART_RHR;
		rx_val=rx_val+127;
		if (cnt==1)
		{
			rc=rx_val;
		}
		else if (cnt==2)
		{
			bic_rc=rx_val;
			cnt=0;
		}
		printf("%d %d\n",rc,bic_rc);
		tc_stop(TC0,0);
		tc_stop(TC0,1);
		tc_write_rc(TC0,0,rc);
		tc_write_rc(TC0,1,bic_rc);
		tc_start(TC0,0);
		tc_start(TC0,1);
	}
}
int main (void)
{
	uint8_t uc_key;
	board_init();
	CPU3_S_SYSTEM_INIT();
	//configure_console();
	sysclk_enable_peripheral_clock(ID_UART);
	pmc_enable_periph_clk(ID_PIOA);
	UART->UART_CR	= 0x00000050;
	UART->UART_MR	= 0x00000800;
	UART->UART_BRGR = 0x00000046;
	PIOA->PIO_PUER = 0x00000300;
	
	stdio_serial_init(UART,&uart_serial_options);
	uart_enable(UART);
	uart_enable_rx(UART);
	uart_enable_tx(UART);
	
	NVIC_DisableIRQ(UART_IRQn);
	NVIC_ClearPendingIRQ(UART_IRQn);
	NVIC_SetPriority(UART_IRQn,0);
	NVIC_EnableIRQ(UART_IRQn);
	
	for (int i=0;i <100000;i++)
	{
		;
	}
	printf("welcome\n");
	
	pmc_enable_periph_clk(ID_PIOA);
	pmc_enable_periph_clk(ID_PIOB);
	pmc_enable_periph_clk(ID_PIOC);
	pmc_enable_periph_clk(ID_PIOD);
	
/*	PIOA->PIO_WPMR = 0x50494F00;
	PIOB->PIO_WPMR = 0x50494F00;
	PIOC->PIO_WPMR = 0x50494F00;
	PIOD->PIO_WPMR = 0x50494F00;
	
	PIOA->PIO_WPMR = 0x50494F00;
	PIOA->PIO_PUER = 0x00000300;
	PIOC->PIO_PER  = 0x378FF1FE;
	PIOC->PIO_OER  = 0x068FF1FE;
	PIOC->PIO_ODR  = 0x01000000;
	PIOC->PIO_CODR = 0x060FF1FE;
	PIOC->PIO_CODR = 0x00800000;

	PIOD->PIO_PER  = 0x0000000F;
	PIOD->PIO_OER  = 0x0000000F;
	//PIOD->PIO_SODR = 0x0000000F;
	PIOD->PIO_PUER = 0x0000000F;
	gpio_configure_pin(PIN_PWM_LED0_GPIO,PIN_PWM_LED0_FLAGS);
	gpio_configure_pin(PIN_PWM_LED1_GPIO,PIN_PWM_LED1_FLAGS);
	
	PIOB->PIO_PER	=	0x02000000;
	PIOB->PIO_ODR	=	0x02000000;		//PB25 Acid feedback pin
	PIOB->PIO_CODR	=	0x02000000;
	PIOB->PIO_IFER  =   0x02000000;
	PIOB->PIO_DIFSR =   0x02000000;
	
	PIOC->PIO_PER	=	0x14000000;
	PIOC->PIO_ODR	=	0x14000000;		//PC28 Bicarbonate feedback pin
	PIOC->PIO_CODR	=	0x14000000;
	PIOC->PIO_IFER  =   0x14000000;
	PIOC->PIO_DIFSR =   0x14000000; */
	
	static uint32_t dummy_1=0,flag_1=0,flag_1b=0,dummy_1b=0;
	static uint32_t status,status_bic,status_fdbck;
	static uint32_t prev_status=0, curr_status=0;
    static uint32_t prev_status_bic=0, curr_status_bic=0;
	static uint32_t prev_status_fdbck=0, curr_status_fdbck=0;
	status=(((PIOB->PIO_PDSR)>>25)&1);
	status_bic=(((PIOC->PIO_PDSR)>>28)&1);
	status_fdbck=(((PIOC->PIO_PDSR)>>26)&1);
	prev_status_bic=status_bic;
	prev_status_fdbck=status_fdbck;
	uint32_t stepcount = 1312499; // sanjeer - this is original value
	uint32_t secondcount = (1312499/2  + 328124)  ;   //
//	uint32_t stepcount = 1351873; // changed to get right cond for clinical trial, need to re fix after adjusting angles again
//		uint32_t stepcount = 797782;
	uint32_t stepcount1 = 6562;
	uart_enable_interrupt(UART,UART_SR_RXRDY);
	
	
	
PIOB->PIO_PER = 1 << 19 ; //TEST
PIOB->PIO_OER = 1 << 19 ; //TEST


// 
//  bic_rc = 210;
// start_bicarpump();
//  rc = 210;
//  start_acidpump();
//  	while (1)
//  	{
//  		;
//  	}
// 		
// 		
// 		
// 		
// 	//	printf("propo\n");
// 	}
	
//*********************************************************  CAN 13-04-2017 *******	//
// 		NVIC_EnableIRQ(CAN0_IRQn);
// 		CAN0->CAN_IER = CAN_IER_MB1;
// 		sn65hvda540_set_en(&can0_transceiver ,PIN_CAN0_STANDBY_IDX);
// 		sn65hvda540_normal_mode(&can0_transceiver);
// 		//	sn65hvda540_standby_mode(&can0_transceiver);
// 		//	sn65hvda540_normal_mode(&can0_transceiver);
// 		g_ul_recv_status = 1;
// 	while(1)  
// 	 {
// 		
// 		while (!g_ul_recv_status)  
// 		 {}
// 		
// 		g_ul_recv_status = 0;
// 		if ( (can0_mailbox.ul_datal == 0x11223344) &&  (can0_mailbox.uc_length == 8)  && (can0_mailbox.ul_datah == 0x55AAAA55) ) 
// 		{
// 			printf("Test1 passed");
// 			
// 			can0_mailbox.ul_mb_idx = MAILBOX_0;
// 			can0_mailbox.ul_id = CAN_MID_MIDvA(MASTER_CPU_id);
// 			can0_mailbox.ul_datal = 0x11223344;
// 			can0_mailbox.ul_datah = 0x55AAAA55;
// 			can0_mailbox.uc_length = 8;
// 			dd_can_mailbox_write(&can0_mailbox);
// 			dd_can_global_send_transfer_cmd(CAN_TCR_MB0);
// 		}
// 		else 
// 		 printf("Test1 failed" ); 
// 		}
	
//************************************************************************************//	
	
		
	
	while (FOREVER)
	{
		if(openfillstate != STATE_OPEN_FILL)
		{
			
		
				if(flag_1==0)
				{
					status=(((PIOA->PIO_PDSR)>>14)&1);
					if (status==1 && prev_status ==0)
					{
						dummy_1++;
						if (dummy_1==2)
						{
							tc_stop(TC0,0);
							dummy_1=0;
							flag_1=1;
						}
				
					}
					prev_status= status;
				}
				if((flag_1b==0) && (flag_1==1))
				{
			                    
					status_bic=(((PIOA->PIO_PDSR)>>15)&1);
			
					if (status_bic==1 && prev_status_bic ==0)
					{
						dummy_1b++;
						if (dummy_1b==2)
						{
							tc_stop(TC0,1);
							dummy_1b=0;
							flag_1b=1;
						}
				
					}
					prev_status_bic= status_bic;
				}
		}
	
		
		if(feedcount_flag == 1)
		{
			
				status=(((PIOA->PIO_PDSR)>>14)&1);
				if (status==1 && prev_status ==0)
				{
					acid_count++;
					acid_f_b = acid_count;
					TC1_cnt = 0;
					
				}
				prev_status= status;
				
					
					
			
			
				status_bic=(((PIOA->PIO_PDSR)>>15)&1);
				
				if (status_bic==1 && prev_status_bic ==0)
				{
				      bicarb_count ++;
					  bicarb_f_b = bicarb_count;
				 TC2_cnt = 0;
				 
				}
				prev_status_bic= status_bic;
				
				
				
		}
 		if ((flag_nrmal == 1) && (flag_1 == 1) && (flag_1b == 1) && (OneSecFlag == 1)  )   // 
		{
			status_fdbck=(((PIOC->PIO_PDSR)>>26)&1);
			
//			if (status_fdbck==0 && prev_status_fdbck ==1)
			if (status_fdbck==1 && prev_status_fdbck ==0)
			{
				stop_pump();
				Correction_Count++;

// 				if(conductivity > (System_Conductivity -10))
// 				{
// 					if(Correction_Count >= 5)
// 					{
// 						
// 					
// 				
// 				if((conductivity > (System_Conductivity +1)) || (conductivity < (System_Conductivity -1) ) )
// 				{
// 					
// 			
// 					if (conductivity < System_Conductivity)
// 					{
// 						if (conductivity < (System_Conductivity -10) ) 
// 						{
// 					
// 		//					if ((Set_RC - rc) < 3 || (rc - Set_RC) < 3 )
// 							{
// 								rc = rc-1;
// 								if (rc < (Set_RC -4))
// 								{
// 									rc = Set_RC;
// 									Correction_Count =0;
// 								}
// 							}
// 						// 					else
// 						// 						rc = Set_RC;
// 					   }	
// 					}
// 				 else if((conductivity > (System_Conductivity -10)))
// 					{
// 	//					if ((Set_RC - rc) < 3 || (rc - Set_RC) < 3 )
// 					
// 						{
// 							rc = rc+1;
// 							if (rc > (Set_RC + 4))
// 							{
// 								rc = Set_RC;
// 								Correction_Count =0;
// 							}
// 						}
// 						// 					else
// 						// 					rc = Set_RC;
// 					}
// 				}
// 				}
// 				}

//				rc=Set_RC;
				start_pump();
				OneSecFlag = 0;
				
				startTimer1(TC0,2,TC2_IRQn,stepcount);
				startTimer2(TC1,0,TC3_IRQn,secondcount);
			}
			prev_status_fdbck= status_fdbck;
		}

			switch (state_3)
			{
				case IDLE_ACID:
					//printf("acid alignment\n");
					rc = 1000;
					start_acidpump();
					state_3 = ALIGNMENT_ACID;
					flag_nrmal=0;
				break;
				
				case ALIGNMENT_ACID:
					if (flag_1==1)
					{
						//printf("done acid alignment\n");
						state_3 = IDLE_BICAR;
					}
					flag_nrmal=0;
				break;
				
				case IDLE_BICAR:
					//printf("bicar alignment\n");
					bic_rc = 1000;
					start_bicarpump();
					state_3 = ALIGNMENT_BICAR;
					flag_nrmal=0;
				break;
				
				case ALIGNMENT_BICAR:
						
					if (flag_1b==1)
					{
						//printf("Done with bicar alignment\n");
						state_3 = CPU3_STANDBY;
					}
					flag_nrmal=0;
				break;
				case CPU3_STANDBY:
	
					if(flag_buff==1)
					{
						Master_cpu_command_type dummy;
						dummy=data_buff[0];
						switch (dummy)
						{ 
							case PROP_START_ACID_PUMP:
															rc=210;
															start_acidpump();
							
							break;
							
							case PROP_START_BICARB_PUMP:
															bic_rc=210;
															start_bicarpump();
							
							break;
							case PROP_START_OPENFILL:
					//		startTimer1(TC0,2,TC2_IRQn,stepcount);
					feedcount_flag = 1; 
								rc=210;
								bic_rc=210;
								start_acidpump();
								start_bicarpump();
								state_3 =CPU3_STANDBY;
								openfillstate = STATE_OPEN_FILL; 
							break;
							case PROP_STOP_OPENFILL:
							//printf("stop_openfill\n");
							feedcount_flag = 0; 
							acid_count = 0;
							bicarb_count = 0;
								stop_pump();
								state_3 =CPU3_STANDBY;
								flag_nrmal=0;
								openfillstate = STATE_IDLE;
							break;
							case PROP_START_ALIGMENT:
		//					printf("start_aligment\n");
							flag_1=0;
							flag_1b=0;
							flag_nrmal=0;
								state_3 = IDLE_ACID;
							break;
							case PROP_START_DISINFECTION:
								rc=210;
								start_acidpump();
								state_3 = CPU3_STANDBY;
							break;
							case PROP_STOP_DISINFECTION:
								stop_pump();
								state_3 = CPU3_STANDBY;
							break;
							case PROP_START_CITRIC_DISINFECTION:
								bic_rc=210;
								start_bicarpump();
								state_3 = CPU3_STANDBY;
							break;
							case PROP_STOP_CITRIC_DISINFECTION:
								stop_pump();
								state_3 = CPU3_STANDBY;
							break;
							
							case prop_BICAR_FEED_BACK:   //selftest bicar
							//   bic_rc = 210; 
							rc= 210;
							bic_rc = 210;
							
							   feedcount_flag = 1; 
							   startTimer1(TC0,2,TC2_IRQn,stepcount);
							    start_pump();
						
							   flag_nrmal=0;  // 1
							   state_3 = CPU3_STANDBY;
							break;
							case prop_BICAR_FEED_BACK_STATUS:
						//	uint8_t acid_feed_back_counter = 0;
					//		        bicarb_f_b_counter = bicarb_count;
					//				acid_feed_back_counter = acid_count;
					SV_put_sensor_data(&sv_sys_statbuffer);
// 								stop_pump();
// 									flag_1=0;
// 									flag_1b=0;
// 									flag_nrmal=0;
// 									state_3 = CPU3_STANDBY;
							break;
							case PROP_START_MIXING:
							OneSecFlag = 1 ;
								switch(User_setConductivity)
									{
										case 135:
										Set_RC= 200;
										bic_rc = 200;
										break;
										
										case 136:
										Set_RC= 197;
										bic_rc = 197;
										break;
										
										case 137:
										Set_RC= 194;
										bic_rc = 194;
										break;
										
										case 138:
										Set_RC= 191;
										bic_rc = 191;
										break;
										
										case 139:
										Set_RC= 188;
										bic_rc = 188;
										break;
										
										case 140:
										Set_RC= 184;
										bic_rc = 184;
										break;
										
										case 141:
										Set_RC= 181;
										bic_rc = 181;
										break;
										
// 										case 142:
// 										Set_RC= 187;
// 										bic_rc =187;
// 										break;
// 										
// 										case 143:
// 										Set_RC= 184;
// 										bic_rc = 184;
// 										break;
// 										
// 										case 144:
// 										Set_RC= 181;
// 										bic_rc = 181;
// 										break;
// 										
// 										case 145:
// 										Set_RC= 178;
// 										bic_rc = 178;
// 										break;
										// 												case 146:
										// 												Set_RC= 186;
										// 												bic_rc = 186;
										// 												break;
										//
										// 												case 147:
										// 												Set_RC= 195;
										// 												bic_rc = 195;
										// 												break;
										//
										// 												case 148:
										// 												Set_RC= 192;
										// 												bic_rc = 192;
										// 												break;
										//
										// 												case 149:
										// 												Set_RC= 192;
										// 												bic_rc = 192;
										// 												break;
										//
										// 												case 150:
										// 												Set_RC= 189;
										// 												bic_rc = 189;
										// 												break;
										//
										// 												case 151:
										// 												Set_RC= 189;
										// 												bic_rc = 189;
										// 												break;
										//
										// 												case 152:
										// 												Set_RC= 186;
										// 												bic_rc = 186;
										// 												break;
										//
										// 												case 153:
										// 												Set_RC= 186;
										// 												bic_rc = 186;
										// 												break;
										//
										// 												case 154:
										// 												Set_RC= 183;
										// 												bic_rc = 183;
										// 												break;
										// 												case 155:
										// 												Set_RC= 183;
										// 												bic_rc = 183;
										// 												break;
										//
										// 												case 156:
										// 												Set_RC= 180;
										// 												bic_rc = 180;
										// 												break;
										//
										// 												case 157:
										// 												Set_RC= 180;
										// 												bic_rc = 180;
										// 												break;
										//
										// 												case 158:
										// 												Set_RC= 177;
										// 												bic_rc = 177;
										// 												break;
										//
										// 												case 159:
										// 												Set_RC = 177;
										// 												bic_rc = 177;
										// 												break;
												
														
									}
						
							feedcount_flag = 1; // test - to be removed
							rc = Set_RC;
							startTimer1(TC0,2,TC2_IRQn,stepcount);
						
// 								start_acidpump();
// 								start_bicarpump();

								start_pump();
								flag_nrmal=1;
								state_3 = CPU3_STANDBY;
							break;
							case PROP_STOP_MIXING:
								//printf("stop\n");
								stop_pump();
								state_3 =CPU3_STANDBY;
								flag_nrmal=0;
							break;
							/*case COMPLETED:
								//printf("completed\n");
								printf("count_start=%d count_stop=%d\n",count_start,count_stop);
								count_start = 0;
								count_stop = 0;*/
							
							break;
						default:
							break;
						}
						
					data_buff[0] = (uint8_t)0;
					flag_buff = 0;
					data_count = 0;
				break;
		
			/*	case CPU3_ACID_ROSTATE:
				if (flag_buff==1)
				{
					switch (data_buff[0])
					{
						case STOP_MIXING:
						printf("stop\n");
						PIOA->PIO_CODR = 0x00004000;
						PIOA->PIO_SODR = 0x00008000;
						//stop_pump();
						state_3 =CPU3_STANDBY;
						break;
						default:
						break;
					}
					
				}
			/*	else
				{
					cpu3_sensorstatus();	
					switch (time_tick)
					{
						case TICK_50MS:
						printf("acid_romixing\n");
						//cpu3_sensorstatus();
						if (sensor_status.rdncy_irq == 0)
						{
							//stop_pump();
							state_3 =CPU3_SAFETY_MEASURE;
						}
						else
						{
							printf("state change\n");
							state_3 =CPU3_BICAR_STATE;
						}
						break;
						default:
						break;
					}
			
				}*/
				/*data_buff[0] = (uint8_t)0;
				flag_buff = 0;
				data_count = 0;
				break;*/
				
				/*case CPU3_BICAR_STATE:
				if (flag_buff==1)
				{
					switch (data_buff[0])
					{
						case STOP_MIXING:
						printf("stop\n");
						stop_pump();
						state_3 =CPU3_STANDBY;
						break;
						default:
						break;
					}
				}
				else
				{
					switch(time_tick)
					{
						case TICK_50MS:
						cpu3_sensorstatus();
						printf("bicarmixing\n");
						if (sensor_status.rdncy_irq == 0)
						{
							stop_pump();
							state_3 =CPU3_SAFETY_MEASURE;
						}
						else
						{
							printf("enter bicar\n");
							start_bicarpump();
							state_3 =CPU3_MONITORING;
						}
						break;
						default:
						break;
					}
				}
				break;
				
				case CPU3_MONITORING:
				if (flag_buff==1)
				{
					switch (data_buff[0])
					{
						case STOP_MIXING:
						printf("stop\n");
						stop_pump();
						state_3 =CPU3_STANDBY;
						break;
						default:
						break;
					}
				}
				else
				{
				switch(time_tick)
				{
					case TICK_50MS:
					cpu3_sensorstatus();
					if (sensor_status.rdncy_irq == 0)
					{
						printf("fault line\n");
						stop_pump();
						state_3 =CPU3_SAFETY_MEASURE;
					}
					cond1_v = sensor_status.cond1_voltage;
					cond1_c = sensor_status.cond1_current;
					cond2_v = sensor_status.cond2_voltage;
					cond2_c = sensor_status.cond2_current;
					// formula to calculate cond1 and cond2;
					if (6 < cond1 <= 7)
					{
						if ( 13 < cond2 <= 14)
						{
							state_3 =CPU3_MONITORING;
						}
						else
						{
							stop_pump();
							cpu3_sv_cntrl_activate_pin(RUDNCY_INT);
							cpu3_sv_cntrl_activate_pin(CPU_INT);
							state_3 =CPU3_SAFETY_MEASURE;
						}
					}
					else
					{
						stop_pump();
						cpu3_sv_cntrl_activate_pin(RUDNCY_INT);
						cpu3_sv_cntrl_activate_pin(CPU_INT);
						state_3 =CPU3_SAFETY_MEASURE;
					}
					
					break;
					default:
					break;
				}
				}
				break;*/
				
				/*case CPU3_SAFETY_MEASURE:
					if (flag_buff == 1)
					{
						switch (data_buff[0])
						{
							case RECOVER:
								cpu3_sv_cntrl_deactivate_pin(RUDNCY_INT);
								cpu3_sv_cntrl_deactivate_pin(CPU_INT);
								start_acidpump();
								start_bicarpump();
								cpu3_sensorstatus();
								switch(time_tick)
								{
									case TICK_500MS:
									if (6 < cond1 <= 7)
									{
										if ( 13 < cond2 <= 14)
										{
											cpu3_sv_cntrl_deactivate_pin(RUDNCY_INT);
											cpu3_sv_cntrl_deactivate_pin(CPU_INT);
											state_3 =CPU3_MONITORING;
										}
										else
										{
											stop_pump();
											cpu3_sv_cntrl_activate_pin(RUDNCY_INT);
											cpu3_sv_cntrl_activate_pin(CPU_INT);
										}
										
									}
									else
									{
										stop_pump();
										cpu3_sv_cntrl_activate_pin(RUDNCY_INT);
										cpu3_sv_cntrl_activate_pin(CPU_INT);
									}
									break;
								}
							break;
							default:
							break;
						}
					}			
				break;
				default:
				break;
			}
		}*/
	}
 	}
	}
	}


void start_acidpump()
{
	PIOA->PIO_CODR  = 0x00180000;				// brake clear
	PIOD->PIO_PER   = 0x00000003;
	PIOC->PIO_PER  = 0x000001FE;
	PIOD->PIO_WPMR	= 0x50494F00;
	PIOC->PIO_WPMR  = 0x50494F00;
	PIOD->PIO_OWER  = 0x00000003;
	PIOD->PIO_OWDR  = 0x0000000C;
	PIOC->PIO_OWER  = 0x000001FE;
	PIOC->PIO_OWDR  = 0x000FF000;
	
	PIOD->PIO_ODSR = array_acid[count].dir;
	PIOC->PIO_ODSR = array_acid[count].AB;
	
	cpu3_sv_cntrl_activatepump(ACID_PUMP);
	acid_startTimer1(TC0,0,TC0_IRQn,rc);
}


	
void start_pump()
{
	PIOA->PIO_CODR  = 0x00180000;				// brake clear
	PIOD->PIO_PER   = 0x00000003;
	PIOC->PIO_PER  = 0x000001FE;
	PIOD->PIO_WPMR	= 0x50494F00;
	PIOC->PIO_WPMR  = 0x50494F00;
	PIOD->PIO_OWER  = 0x00000003;
	PIOD->PIO_OWDR  = 0x0000000C;
	PIOC->PIO_OWER  = 0x000001FE;
	PIOC->PIO_OWDR  = 0x000FF000;
	
	PIOD->PIO_ODSR = array_acid[count].dir;
	PIOC->PIO_ODSR = array_acid[count].AB;
	
	
	
	PIOD->PIO_WPMR	= 0x50494F00;
	PIOC->PIO_WPMR  = 0x50494F00;
	PIOD->PIO_OWER  = 0x0000000C;
	PIOD->PIO_OWDR = 0x000000003;
	PIOC->PIO_OWER  = 0x000FF000;
	PIOC->PIO_OWDR  = 0x000001FE;
	PIOB->PIO_CODR  = 0x00204000;			// brake clear
			
	PIOD->PIO_ODSR = array_bicar[count].dir;
	PIOC->PIO_ODSR = array_bicar[count].AB;
			
	cpu3_sv_cntrl_activatepump(ACID_PUMP);
	cpu3_sv_cntrl_activatepump(BICAR_PUMP);
	
	
	acid_startTimer1(TC0,0,TC0_IRQn,rc);	
}


void start_bicarpump()
{
	//printf("start bicar pump\n");
		//cpu3_sv_cntrl_deactivate_pin(BICARBONATE_VALVE);
		PIOD->PIO_WPMR	= 0x50494F00;
		PIOC->PIO_WPMR  = 0x50494F00;
		PIOD->PIO_OWER  = 0x0000000C;
		PIOD->PIO_OWDR = 0x000000003;
		PIOC->PIO_OWER  = 0x000FF000;
		PIOC->PIO_OWDR  = 0x000001FE;
		PIOB->PIO_CODR  = 0x00204000;			// brake clear
		
		PIOD->PIO_ODSR = array_bicar[count1].dir;
		PIOC->PIO_ODSR = array_bicar[count1].AB;
		
		cpu3_sv_cntrl_activatepump(BICAR_PUMP);
		bicar_startTimer1(TC0,1,TC1_IRQn,bic_rc);
}

void stop_pump()
{
	CPU3_S_STOP_TIMER(0);
	CPU3_S_STOP_TIMER(1);
	CPU3_S_STOP_TIMER(3);
	CPU3_S_STOP_TIMER(4);
}

void cpu3_sensorstatus(void)
{
	Sys_statusType sv_sys_statbuffer;
	cpu3_sv_status_getsystemstatus(&sv_sys_statbuffer);
	sensor_status.rdncy_irq=sv_sys_statbuffer.rudncy_irq;
	sensor_status.cond1_voltage=sv_sys_statbuffer.cond1_status_voltage;
	sensor_status.cond1_current=sv_sys_statbuffer.cond1_status_current;
	sensor_status.cond2_voltage=sv_sys_statbuffer.cond2_status_voltage;
	sensor_status.cond2_current=sv_sys_statbuffer.cond2_status_current;
	sensor_status.temp=sv_sys_statbuffer.Tempstatus;
}