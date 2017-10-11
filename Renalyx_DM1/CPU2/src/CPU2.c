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
#include "CPU2.h"
#include "spi.h"
#include "conf_board.h"
#include "conf_clock.h"
#include "Platform/Service/CPU2_S_interface.h"
#include "Platform/Driver/Driver_calls.h"
#include "Platform/Service/CPU2_S_CNTRL/CNTRL.h"
#include "Platform/Service/CPU2_S_TIMER/S_TIMER.h"
#include "Platform/Service/CPU2_S_STATUS/STATUS.h"
#define FOREVER		1
#define SPI_Handler     SPI0_Handler
#define SPI_IRQn        SPI0_IRQn

extern void spi_slave_initialize(void);
extern void CPU2_S_SYSTEM_INIT(void);
extern volatile uint8_t flag_buff,count_2;
extern uint8_t data_buff[3];
extern uint8_t flag_1;
extern uint32_t data_count,data_count1,fedbck_cnt2=0;
extern uint32_t fedbck_cnt;//,fedbck_cnt1;
extern uint16_t fedbck_cnt1;
extern uint32_t time_tick;
//extern void spi_master_go(void);
TwobyteData data_stream;
cpu2_state states_2 = CPU2_STANDBY;
uint32_t var;
Sys_statusType sv_sys_statbuffer;
volatile uint32_t mode,ra,rc;
volatile uint32_t default_speed = 68;
cpu2_status sensor_status;
extern volatile uint16_t count_start,count_stop;
extern volatile start_c[3];
volatile uint32_t DUTY = 0x000003AA;
volatile uint8_t flag_2=0,flag_3=0,modify=0,modify_1=0;
volatile uint8_t d_state=0;
extern uint32_t bp_speed;
extern volatile uint8_t bd_start_calibration , bp_start_command , bp_stop_command  ,bp_Speed_change_command , bp_inc , bp_dec;
//void DD_INIT_TIMER(void);
//uint8_t DD_CONFIG_TIMER(uint8_t timer_no, uint8_t timer_res);
//uint8_t DD_SET_TIMER(Tc *tc, uint32_t channel, IRQn_Type irq, uint32_t rc);

volatile speed_data speed_d;
volatile table array[64]=
{
	{79,8192},{79,24576},{79,57344},{78,73728},{78,106496},{77,122880},{76,139264},{76,172032},{75,188416},{74,204800},{72,204800},
	{71,221184},{70,237568},{68,237568},{67,253952},{65,253952},{00,253952},{01,253952},{03,253952},{04,237568},{06,237568},{07,221184},
	{8,204800},{10,204800},{11,188416},{12,172032},{12,139264},{13,122880},{14,106496},{14,73728},{15,57344},{15,24576},{15,00000},
	{15,16384},{15,49152},{14,65536},{14,98304},{13,114688},{12,131072},{12,163840},{11,180224},{10,196608},{8,196608},{07,212992},{06,229376},
	{04,229376},{03,245760},{01,245760},{64,245760},{65,245760},{67,245760},{68,229376},{70,229376},{71,212992},{72,196608},{74,196608},{75,180224},
	{76,163840},{76,131072},{77,114688},{78,98304},{78,65536},{79,49152},{79,16384}
};
typedef struct 
{
	uint16_t ml;
	uint16_t duty_value;
	uint32_t fdbck;
}lut;
volatile lut blood_array[20]= 
{
	{50,700,50},{60,750,57},{65,750,64},{70,750,70},{75,750,74},{80,750,78},{85,750,82},{90,750,86},{95,750,90},{100,760,100},{110,780,110},{120,810,120},
	{140,810,140},{150,850,150},{200,970,200},{300,1200,290},{400,1250,350}	
};
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
//volatile int value=0;
volatile uint32_t fed_value=0;
extern uint32_t array_dummy[100];
extern uint32_t var;
uint8_t command =  0;
int main (void)
 {
	
	speed_data speed_d;
	static uint32_t status=0;
	static uint32_t prev_status=0;
	sysclk_init();
	board_init();
	CPU2_S_SYSTEM_INIT();
	configure_console();
	//CPU2_S_START_TIMER(0,);
	printf("welcome\n");
	printf("Version1.1\n");
	PIOA->PIO_WPMR = 0x50494F00;
	PIOA->PIO_PUER = 0x00000300;
	//PIOA->PIO_PUER = 0x00060000;
	PIOA->PIO_PDR =	 0x00060000;
	PIOA->PIO_ABSR = 0x00090000;
	pmc_enable_periph_clk(ID_PIOD);
	pmc_enable_periph_clk(ID_PIOC);
	pmc_enable_periph_clk(ID_PWM);

	PIOD->PIO_WPMR = 0x50494F00;
	PIOD->PIO_PER  = 0x000002CF;
	PIOD->PIO_OER  = 0x0000024F;
	PIOD->PIO_ODR  = 0x00000080;				//m1,m2,m3,m4,directionA,BrakeA
	PIOD->PIO_CODR = 0x0000024F;
	PIOD->PIO_PUER = 0x0000024F;

	PIOC->PIO_WPMR = 0x50494F00;
	PIOC->PIO_PER  = 0x0007F800;
	PIOC->PIO_OER  = 0x4007F800;				//m1,m2,m3,m4,directionB,BrakeB
	PIOC->PIO_CODR = 0x0007F000;
	PIOC->PIO_PUER = 0x4007F000;

	PIOA->PIO_PUER = 0x00000300;
	PIOA->PIO_PUER = 0x00060000;
	
	PWM->PWM_WPCR = 0x50574D00;
	//PWM->PWM_DIS = 0x000000FF;
	
	PWM->PWM_CH_NUM[4].PWM_CMR = PWM_CMR_CPOL|PWM_CMR_CPRE_MCK_DIV_32|PWM_CMR_CES|PWM_CMR_DTE|PWM_CMR_DTHI|PWM_CMR_DTLI;

	PWM->PWM_CH_NUM[4].PWM_CPRD = 0x00000A41;
	PWM->PWM_CH_NUM[4].PWM_CDTY = DUTY;			//35% duty cycle
	PIOD->PIO_WPMR	= 0x50494F00;
	PIOD->PIO_PER   = 0x0000024F;
	PIOD->PIO_OER   = 0x0000024F;
	PIOD->PIO_PUER  = 0x0000000F;
	PIOD->PIO_SODR  = 0x00000200;
	status=(((PIOA->PIO_PDSR)>>14)&1);
	prev_status=status;
	NVIC_DisableIRQ(SPI_IRQn);
	NVIC_ClearPendingIRQ(SPI_IRQn);
	NVIC_SetPriority(SPI_IRQn, 0);
	NVIC_EnableIRQ(SPI_IRQn);

//	spi_slave_initialize();
	//spi_master_initialize();
	//pio_enable_interrupt(PIOA,PIO_IER_P14);
	uint32_t expire_time = 6562490;			//10 sec
	uint32_t stepcount = 6562;			//10 sec
	startTimer1(TC0,0,TC0_IRQn,expire_time);
	int i=0;
	startTimer3(TC0,2,TC2_IRQn,stepcount);
	
//	start_bloodpump();
//       CPU2_D_RESET_PIO(PIO_TYPE_PIO_C,11);    // reseting calibrate pin for BD/ABD 4july2017
// 	 /* int i=0;*/
// 	  for(int i=0; i<10000000; i++)
// 	  {
// 		  ;
// 	  }
// 	  
 	   CPU2_D_SET_PIO(PIO_TYPE_PIO_C,11);
 	  
	  
	
	CPU2_D_SET_PIO(PIO_TYPE_PIO_A,13);      // VENOUS CLAMP POWER ENABLED
	CPU2_D_SET_PIO(PIO_TYPE_PIO_A,11);     // BYPASS VALVE POWER ENABLED
	while (FOREVER)
	{

		/*if (flag_buff == 0)
		{
			cpu2_sensorstatus();
			//uint32_t data = sensor_status.cond1;
			//printf("cond_data=%d\n",data);
			
			if (sensor_status.bldpump_door == 1)
			{
				printf("blood pump door open\n");
				cpu2_sv_cntrl_activate_pin(RUDNCY_INT);
				cpu2_sv_cntrl_activate_pin(CPU_INT);
				// send data to the cpu1
				safety_measure_activate();
				states_2 = CPU2_SAFETYSTATE;	
			}
			else
			{
				//start_bloodpump();
				states_2 = CPU2_BLOODPUMP;
			}
			
			/*if ()				// fpr conductivity and temp
			{
			}*/
		//}
		
		while (flag_buff >= 1)
		{ 
			command = data_buff[0];
				if (command == BP_APT_VPT_SEND_TO_MASTER)
				{
					//printf("start\n");
									
					//send CAN Message for APT and VPT
					SV_put_sensor_data(&sv_sys_statbuffer);
					command = (uint8_t)0;
				//	flag_buff--;
				}
				
				if(command == BD_CALIBRATION){
					
					CPU2_D_RESET_PIO(PIO_TYPE_PIO_C,11);    // reseting calibrate pin for BD/ABD 4july2017
					// 	 /* int i=0;*/
					 	  for(int i=0; i<10000000; i++)
						  {
					 		  ;
					 	  }
					
					 	   CPU2_D_SET_PIO(PIO_TYPE_PIO_C,11);
					command = (uint8_t)0;
					
					
				}
			//	continue;
								
			//printf("enter\n");
			switch (states_2)
			{
				case CPU2_STANDBY:
				
					if (bp_start_command == BP_START_BLOOD_PUMP)
					{
						//printf("start\n");
						bp_start_command = 0;
						start_bloodpump();
						states_2 = CPU2_BLOODPUMP;
					}
					command = (uint8_t)0;
				//	flag_buff = 0;
					data_count = 0;			
					
				break;
				
				case CPU2_BLOODPUMP:
				
// 
// 					switch (command)
// 					{

						if (bp_start_command == BP_START_BLOOD_PUMP)
						{	
							bp_start_command =  0;
							start_bloodpump();
							//printf("start\n");
							if (modify_1==1)
							{
								modify=1;
								flag_3=1;
								flag_1=1;
								flag_2=1;
								count_2=2;
							}
						//	flag_buff--;
							states_2 = CPU2_BLOODPUMP;
						}
					//	break;

						if (bp_stop_command == BP_STOP_PUMP)
						{
							
						bp_stop_command = 0;
						//printf("stop\n");
							stop_bloodpump();
							//states_2 = CPU2_STANDBY;
							fedbck_cnt=0;
							modify=0;
							flag_3=0;
							flag_1=0;
							flag_2=0;
							count_2=0;
							states_2 = CPU2_BLOODPUMP;
						//	flag_buff--;
						}
					//	break;
					/*	case COMPLETED:
							printf("count_start:%d count_stop:%d,\n",count_start,count_stop);
							count_start = 0;
							count_stop = 0;
							
							//CPU2_D_INIT_IIC();
							states_2 = CPU2_STANDBY;
						break;*/
							if (bp_Speed_change_command == BP_SPEED_CHANGE)
							{
								
							bp_Speed_change_command = 0;
						//case BP_SPEED_CHANGE:
	//					printf("speedchange\n");
						//printf("%d\n",data_count1);
// 							if (data_count1 == 2)
// 							{
// 								data_stream.bytearray[0] = data_buff[1];
// 								data_stream.bytearray[1] = data_buff[2];
// 								speed_d.byte[0]= data_stream.bytearray[0];
// 								speed_d.byte[1]= data_stream.bytearray[1];
// 								//printf("%d\n",data_stream.Twobyte);
								speedchange_bloodpump(bp_speed);
								states_2 = CPU2_BLOODPUMP;
								modify=1;
								modify_1=1;
								data_count1=0;
							}			
							//}
					//	break;
						
							if (bp_inc == BP_INC)
							{
								bp_inc = 0;
						//	}
						//	printf("inc\n");
							fed_value = fed_value+10;
							//printf("%d\n",fed_value);
							}
					//	break;
						case BP_DEC:
							if (bp_dec == BP_DEC)
							{
								bp_dec = 0;
					//	printf("dec\n");
							fed_value = fed_value-10;
					//	printf("%d\n",fed_value);
							}
// 						break;
// 						
// 						default:
// 						break;
			//		}
					command = (uint8_t)0;
					//flag_buff = 0;
					data_count = 0;
				break;
				
				case CPU2_SAFETYSTATE:
				//printf("%d\n",data_buff[0]);
				switch (command)
				{
					case BP_RECOVER:
					cpu2_sensorstatus();
					
					if (sensor_status.bldpump_door == 0)
					{
						states_2 = CPU2_BLOODPUMP;
						cpu2_sv_cntrl_deactivate_pin(RUDNCY_INT);
						cpu2_sv_cntrl_deactivate_pin(CPU_INT);
						safety_measure_deactivate();
					}
					else
					{
						cpu2_sv_cntrl_activate_pin(RUDNCY_INT);
						cpu2_sv_cntrl_activate_pin(CPU_INT);
						safety_measure_activate();
						states_2 = CPU2_SAFETYSTATE;
					}
					break;
					
					default:
					break;
					//similarly for the temp and conductivity
				}
				break;
				
				default:
				break;
				
			}
			command = (uint8_t)0;
			//data_buff[1] = (uint8_t)0;
			//data_buff[2] = (uint8_t)0;
		//	flag_buff = 0;
		flag_buff--;
//			data_count = 0;
		}
		
		
				
		if (!( time_tick % 5))
		{
			// read APT and VPT from spi
			cpu2_sensorstatus();
		}
		if(modify==1)
		{
// 			status=(((PIOC->PIO_PDSR)>>9)&1);
// 			if (status==1 && prev_status ==0)
// 			{
// 				fedbck_cnt++;
// 			}
// 			prev_status= status;
// 		
//
			if (flag_1==1 && flag_2==1)
			{
	//			printf("test string forever = %d\n",fedbck_cnt1);
				if (count_2==2)
				{
					flag_3=1;
				}
				if (flag_3==1)
				{
					if (fedbck_cnt1 > fed_value )
					{
					//	printf("more... %d\n",fed_value);
						/*if (fedbck_cnt1>fedbck_cnt2)
						{
							DUTY=DUTY-2;
							PWM->PWM_CH_NUM[4].PWM_CDTYUPD = DUTY;
							printf("DUTY=%d\n",DUTY);
						}*/
								if (fedbck_cnt1 > fed_value+100)
								{
									//printf("decrment by 60....\n");
									DUTY=DUTY-60;
									PWM->PWM_CH_NUM[4].PWM_CDTYUPD = DUTY;
							//		printf("DUTY=%d\n",DUTY);
								}
								else if (fedbck_cnt1 > fed_value+80)
								{
									//printf("decrment by 50....\n");
									DUTY=DUTY-50;
									PWM->PWM_CH_NUM[4].PWM_CDTYUPD = DUTY;
								//	printf("DUTY=%d\n",DUTY);
								}
								else if ((fedbck_cnt1 >= fed_value+20) && (fedbck_cnt1 <=fed_value+80))
								{
									//printf("decrment by 40....\n");
									DUTY=DUTY-40;
									PWM->PWM_CH_NUM[4].PWM_CDTYUPD = DUTY;
								//	printf("DUTY=%d\n",DUTY);
								}
								else if (fedbck_cnt1 > fed_value+10)
								{
									//printf("decrment by 20....\n");
									DUTY=DUTY-20;
									PWM->PWM_CH_NUM[4].PWM_CDTYUPD = DUTY;
								//	printf("DUTY=%d\n",DUTY);
								}
								
								else if (fedbck_cnt1 > fed_value+5)
								{
									//printf("decrment by 10....\n");
									DUTY=DUTY-10;
									PWM->PWM_CH_NUM[4].PWM_CDTYUPD = DUTY;
								//	printf("DUTY=%d\n",DUTY);
								}
								else
								{
									//printf("decrment by 2....\n");
									DUTY=DUTY-2;
									PWM->PWM_CH_NUM[4].PWM_CDTYUPD = DUTY;
								//	printf("DUTY=%d\n",DUTY);
								}
								
						}
							
						//fedbck_cnt2=fedbck_cnt1;
						
					else if (fedbck_cnt1 < fed_value)
					{
						//	printf("less...\n");
							DUTY=DUTY+1;
							PWM->PWM_CH_NUM[4].PWM_CDTYUPD = DUTY;
						//	printf("DUTY=%d\n",DUTY);
						
					}
				}
				//fedbck_cnt1=0;
				flag_1=0;
			}
		}
		
		
		
		
	}	
}

void start_bloodpump()
{
	/*PIOD->PIO_WPMR	= 0x50494F00;
	PIOC->PIO_WPMR  = 0x50494F00;
	PIOD->PIO_OWER  = 0x0000024F;
	PIOC->PIO_OWER  = 0x0003E000;
	PIOC->PIO_CODR  = 0x00001000;
	
	PIOD->PIO_ODSR = array[0].A;
	PIOC->PIO_ODSR = array[0].B;
	cpu2_sv_cntrl_activatepump(BLOODPUMP);
	rc = 231;				//10 rotation RC COUNT
	ra = 4179;				//35 rotation				225-68 = 162   THEN RA COUNT VALUE WILL BE 3976 TO GET DEFAULT ROTATIION OF 35
	mode =1;
	//startTimer1(TC0,0,TC0_IRQn,rc);
	//startTimer2(TC0,1,TC1_IRQn,ra);
	defaultspeed_bloodpump(rc);				// 10 rotation rc count is 231
	rampingspeed_bloodpump(ra);	*/
	
	PIOD->PIO_SODR = 0x00000008;				// setting of dac input that is 8
	PIOD->PIO_CODR = 0x00000040;
	PIOD->PIO_CODR = 0x00000200;
	cpu2_sv_cntrl_activatepump(BLOODPUMP);		//35 rotation ra count will be 4179	
	
}
void stop_bloodpump()
{
	cpu2_sv_cntrl_deactivatepump(BLOODPUMP);
	PIOD->PIO_CODR = 0x0000000F;
	PIOD->PIO_SODR = 0x00000200;
	//PIOC->PIO_SODR = 0x00001000;
	//PIOC->PIO_CODR	= 0x00040000;
	//tc_stop(TC0,0);
	//tc_stop(TC0,1);
	/*PIOD->PIO_SODR = 0x00000200;
	PIOC->PIO_SODR = 0x00001000;
	cpu2_sv_cntrl_deactivatepump(BLOODPUMP);
	CPU2_S_STOP_TIMER(0);
	CPU2_S_STOP_TIMER(1);*/
}
void speedchange_bloodpump(uint32_t spd_change)
{
	int i;
	
	//printf("Before %d\n",spd_change);
	
	for(i=0;i<20;i++)
	{
		if(spd_change == blood_array[i].ml)
		{
			printf("value matched\n");
			DUTY=blood_array[i].duty_value;
			fed_value=blood_array[i].fdbck;
			//value=i;
			break;
		}
		else if((spd_change > blood_array[i].ml) && (spd_change < blood_array[i+1].ml))
		{
			printf("manipulation\n");
			DUTY = blood_array[i].duty_value+((blood_array[i+1].duty_value-blood_array[i].duty_value)/(blood_array[i+1].ml-blood_array[i].ml))*(spd_change-blood_array[i].ml);
			//value=19;
			//fed_value = blood_array[i].fdbck+((blood_array[i+1].fdbck-blood_array[i].fdbck)/(blood_array[i+1].ml-blood_array[i].ml))*(spd_change-blood_array[i].ml);
			fed_value = ((spd_change-blood_array[i].ml)*(blood_array[i+1].fdbck-blood_array[i].fdbck)/(blood_array[i+1].ml-blood_array[i].ml))+blood_array[i].fdbck;
			//blood_array[value].fdbck=fed_value;
			//printf("After %d\n",fed_value);
			break;
		}
	}
	//spd_change=spd_change+200;
	printf("After %d\n",DUTY);
	PWM->PWM_CH_NUM[4].PWM_CDTYUPD = DUTY;
	flag_2=1;
	/*uint16_t imd = spd_change/10;
	uint32_t buf1 = 2310/imd;
	if (default_speed > buf1)
	{
		mode = 1;
		default_speed = buf1;
		uint32_t val = rc-default_speed;
		ra = (84000000/(128*val))-1;
		//startTimer2(TC0,1,TC0_IRQn,ra);
		rampingspeed_bloodpump(ra);
	}
	else if (default_speed < buf1)
	{
		mode = 0;
		default_speed = buf1;
		uint32_t val = default_speed - rc;
		ra = (84000000/(128*val))-1;
		//startTimer2(TC0,1,TC1_IRQn,ra);
		rampingspeed_bloodpump(ra);
	}*/
	
}
void dacinput_bloodpump(uint32_t var1)
{
	PWM->PWM_CH_NUM[6].PWM_CDTYUPD = var1;
}
void defaultspeed_bloodpump(uint32_t speedcount)
{
	CPU2_S_START_TIMER(1,speedcount);
}

void rampingspeed_bloodpump(uint32_t rampingspeed)
{
	CPU2_S_START_TIMER(2,rampingspeed);
}

void cpu2_sensorstatus(void)
{
	
	
	cpu2_sv_status_getsystemstatus(&sv_sys_statbuffer);
	
	sensor_status.abd=sv_sys_statbuffer.abdstatus;
	sensor_status.bd=sv_sys_statbuffer.bdstatus;
	sensor_status.bldpump_door=sv_sys_statbuffer.bp_door;
	sensor_status.bldpump_sense=sv_sys_statbuffer.bp_sense;
	sensor_status.clamp_fdbk=sv_sys_statbuffer.clamp_fdbk;
	sensor_status.clamp_door=sv_sys_statbuffer.clamp_door;
	sensor_status.rdncy_irq=sv_sys_statbuffer.rudncy_irq;
// 	sensor_status.cond1=sv_sys_statbuffer.cond1_status;
// 	sensor_status.cond2=sv_sys_statbuffer.cond2_status;
// 	sensor_status.temp=sv_sys_statbuffer.Tempstatus;
	sensor_status.apt=sv_sys_statbuffer.aptstatus;
	sensor_status.vpt=sv_sys_statbuffer.vptstatus;	
}

void safety_measure_activate(void)
{
	cpu2_sv_cntrl_activate_pin(CLAMP_PWR);
	stop_bloodpump();
	cpu2_sv_cntrl_activate_pin(BYPS_PWR);
}
void safety_measure_deactivate(void)
{
	cpu2_sv_cntrl_deactivate_pin(CLAMP_PWR);
	start_bloodpump();
	cpu2_sv_cntrl_deactivate_pin(BYPS_PWR);
	//states_2= CPU2_BLOODPUMP;
}




