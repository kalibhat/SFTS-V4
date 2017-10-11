/*
 * HANDLER.c
 *
 * Created: 12/3/2014 11:32:20 AM
 *  Author: wid7
 */ 
#include "HANDLER.h"
#include "CPU2.h"
#include "Platform/Service/CPU2_S_interface.h"
#include "Platform/Driver/Driver_calls.h"
uint32_t time_tick;
uint32_t count,cnt;

extern void spi_master_go(void);
extern uint32_t default_speed;
extern uint32_t ra,rc;
extern uint32_t mode;

extern table array[64];
volatile uint8_t flag_1=0,modify;
volatile uint32_t fedbck_cnt=0,count_2=0;
uint16_t fedbck_cnt1 = 0;
volatile uint32_t i2c_counter = 0 ;
extern bool sv_acc_flag, sv_end_flag, sv_rxrdy_flag;
volatile uint8_t spi_count=0;
uint32_t time_tick = 0;
uint32_t tick_5 = 0 ;
uint32_t time_tick_5 = 0;
uint32_t newfbcnt =0;
uint32_t mscnt5 = 0;
static uint32_t statusnew=0;
	static uint32_t prev_statusnew=0;
void TC0_Handler()
{
	spi_count++;
	//printf("test string handler");
	printf("count = \t%d\n",newfbcnt);
	//fedbck_cnt1=fedbck_cnt;
	fedbck_cnt1 = newfbcnt;
	//printf("5mscount = %d\n",mscnt5);
	
	
	if (modify==1)                                   // Timer interrupt every 10 sec
	{
		mscnt5 = 0;
		newfbcnt = 0;
		count_2++;
		flag_1=1;
		fedbck_cnt=0;
	}
	//spi_master_go();
		
	/*PIOD->PIO_WPMR	= 0x50494F00;
	PIOD->PIO_OWER  = 0x0000024F;
	PIOC->PIO_WPMR  = 0x50494F00;
	PIOC->PIO_OWER  = 0x0003E000;
	
	count = count+1;
	if(count<64)
	{
		PIOD->PIO_ODSR = array[count].A;
		PIOC->PIO_ODSR = array[count].B;
	}
	if (count > 63)
	{
		count = 0;
		PIOD->PIO_ODSR = array[count].A;
		PIOC->PIO_ODSR = array[count].B;
	}*/
	if ((tc_get_status(TC0, 0) & TC_SR_CPCS) == TC_SR_CPCS)
	{
		
	}
}

void TC1_Handler()
{
	cnt++;
	if (mode == 1)
	{
		if ( rc == default_speed)
		{
		//printf("timer stop\n");
			tc_stop(TC0,1);                                   // Timer interrupt based on RC value
		}
		else
		{
			if (cnt == 2)
			{
				//printf("temer2\n");
				rc = rc-1;
				tc_stop(TC0,0);
				//tc_write_rc(TC0,0,rc);
				startTimer1(TC0,0,TC0_IRQn,rc);
				cnt = 0;
			}
		}
	}
	else if (mode == 0)
	{
		if ( rc == default_speed)
		{
			tc_stop(TC0,1);
		}
		else
		{
			if (cnt == 2)
			{
				
				rc = rc+1;
				tc_write_rc(TC0,0,rc);
				cnt = 0;
			}
		}
	}
	if ((tc_get_status(TC0, 1) & TC_SR_CPCS) == TC_SR_CPCS)
	{
		
	}
}

void TC2_Handler()
{
	
	
	if(time_tick++ > 65000)
	{
		time_tick = 0;
	}
// 	if (sv_acc_flag == true && sv_end_flag == false)                               // Timer interrupt every 10 msec
// 	{
// 		
// 		i2c_counter++;
// 		if (i2c_counter > 2)
// 		{
// 			CPU2_D_INIT_IIC();
// 			i2c_counter=0;
// 			sv_acc_flag = false;
// 			sv_rxrdy_flag = false;
// 			sv_end_flag = false;
// 			//printf("e\n");
// 		}
// 	}
// 	else
// 	{
// 		i2c_counter = 0;
// 	}
// 	if ((tc_get_status(TC0, 2) & TC_SR_CPCS) == TC_SR_CPCS)
// 	{
// 		
// 	}
}
void TC6_Handler()
{
	NVIC_ClearPendingIRQ(TC6_IRQn);
	uint32_t tick =0,tick_10 = 0,tick_50=0,tick_100=0,tick_500=0;                       // Timer interrupt every 1 msec
	tick_5++;
	
	// CPU2_D_RESET_PIO(PIO_TYPE_PIO_C,16);
	if(tick_5 == 1){
		
		
	//	 CPU2_D_SET_PIO(PIO_TYPE_PIO_C,16);
		
		// manav05 i9ncrement a variable
		mscnt5++ ;
		
		statusnew=(((PIOC->PIO_PDSR)>>9)&1);
			if (statusnew==1 && prev_statusnew ==0)
			{
				newfbcnt++;
			}
			prev_statusnew= statusnew;
		
		tick++;
		tick_5 = 0;
		time_tick = TICK_5MS;
		
	}
	
	if (tick == 2)
	{
		tick_10++;
		tick = 0;
		time_tick = TICK_10MS;                                            
	}
	if (tick_10 == 5)
	{
		tick_50++;
		tick_10 = 0;
		time_tick = TICK_50MS;
	}
	if (tick_50 == 2)
	{
		tick_100++;
		tick_50 = 0;
		time_tick = TICK_100MS;
	}
	if (tick_100 == 5)
	{
		tick_500++;
		tick_100 = 0;
		time_tick = TICK_500MS;
	}
	if (tick_500 == 2)
	{
		tick_500 = 0;
		time_tick = TICK_SEC;
	}
	
	if ((tc_get_status(TC2, 0) & TC_SR_CPCS) == TC_SR_CPCS)
	{
		//printf("i m in handler tc6\n");
	}
}
