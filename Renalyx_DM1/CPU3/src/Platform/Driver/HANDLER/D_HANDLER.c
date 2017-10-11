/*
 * D_HANDLER.c
 *
 * Created: 12/9/2014 2:55:07 PM
 *  Author: wid7
 */ 
#include "D_HANDLER.h"
#include "CPU3.h"
#include "Platform/Driver/TIMER/D_TIMER.h"
#include "Platform/Driver/IIC/D_IIC.h"
//#include "Platform/Driver/INT/D_INT.h"
//#include "twi.h"
static volatile uint8_t toggle = 0;
 volatile uint32_t TC1_cnt= 0;
 volatile uint32_t TC2_cnt= 0;
 volatile uint32_t  Cnt_TC0_total = 0;
 volatile uint32_t Cnt_TC1_total = 0;
uint8_t OneSecFlag = 0;
table array_acid[64] =
{
	{3,30},{3,286},{3,414},{3,78},{3,206},{3,470},{3,38},{3,166},{3,442},{3,106},{3,98},{3,380},{3,236},{3,228},{3,504},{3,496},
	{2,480},{2,496},{2,504},{2,228},{2,236},{2,380},{2,98},{2,106},{2,442},{2,166},{2,38},{2,470},{2,206},{2,78},{2,414},{2,286},
	{0,30},{0,286},{0,414},{0,78},{0,206},{0,470},{0,38},{0,166},{0,442},{0,106},{0,98},{0,380},{0,236},{0,228},{0,504},{0,496},
	{1,480},{1,496},{1,504},{1,228},{1,236},{1,380},{1,98},{1,106},{1,442},{1,166},{1,38},{1,470},{1,206},{1,78},{1,414},{1,286}
};
table array_bicar[64] =
{
	{12,61440},{12,126976},{12,258048},{12,319488},{12,450560},{12,512000},{12,573440},{12,704512},{12,765952},{12,827392},{12,819200},{12,880640},{12,942080},{12,933888},{12,995328},{12,987136},
	{8,983040},{8,987136},{8,995328},{8,933888},{8,942080},{8,880640},{8,819200},{8,827392},{8,765952},{8,704512},{8,573440},{8,512000},{8,450560},{8,319488},{8,258048},{8,126976},
	{0,61440},{0,126976},{0,258048},{0,319488},{0,450560},{0,512000},{0,573440},{0,704512},{0,765952},{0,827392},{0,819200},{0,880640},{0,942080},{0,933888},{0,995328},{0,987136},
	{4,983040},{4,987136},{4,995328},{4,933888},{4,942080},{4,880640},{4,819200},{4,827392},{4,765952},{4,704512},{4,573440},{4,512000},{4,450560},{4,319488},{4,258048},{4,126976},
};
volatile uint32_t i2c_counter = 0 ;
extern bool sv_acc_flag, sv_end_flag, sv_rxrdy_flag;
acid_state a_state = STATE_1;
bicar_state b_state = STATE_4;
extern volatile uint32_t count,cnt,count1;
volatile uint32_t time_tick;
volatile uint32_t tick =0,tick_10 = 0,tick_50=0,tick_100=0,tick_500=0;
extern uint32_t default_speed, bic_default_speed;
extern uint32_t ra,rc,bic_ra,bic_rc, Set_RC;
extern volatile uint8_t fflag,fflag_bic,flag_nrmal,flag_bic;
volatile uint8_t flag_t = 0;
volatile uint32_t stepcount;

extern void start_acidpump();
extern void start_bicarpump();

/*uint32_t stepcount_1 = 656249;
uint32_t PIOB_Handler_interrupt(Pio *p_pio,uint32_t pio)
{
	uint32_t acid_rc_value = 656249;
	switch (a_state)
	{
		case STATE_1:
		if (PIOB->PIO_ISR & (1<<pio))
		{
			printf("first hig acid\n");
			a_state = STATE_1_1;
		}
		break;
		
		case STATE_1_1:
		//printf("%d\n",state);
		if (PIOB->PIO_ISR & (1<<pio))
		{
			printf("second high acid\n");
			tc_stop(TC0,0);
			fflag =1;
			a_state = STATE_1;
			pio_disable_interrupt(PIOB,PIO_IER_P25);
		}
		break;
	}
	
}
uint32_t PIOC_Handler_interrupt(Pio *p_pio)
{
	
	uint32_t bicar_rc_value = 656249;
	stepcount = 1312499;					//2sec
	
	if (flag_nrmal==1)
	{
		//printf("enter\n");
		if (PIOC->PIO_ISR & (1<<26))
		{
			rc = 47;
			bic_rc = 47;
			printf("l\n");
			start_bicarpump();
			start_acidpump();
			startTimer1(TC0,2,TC2_IRQn,stepcount);
		}
	}
	else if(flag_bic==1)
	{
		switch (b_state)
		{
			case STATE_4:
			if (PIOC->PIO_ISR & (1<<28))
			{
				//printf("first high bicar\n");
				b_state= STATE_4_4;
			}
			break;
			
			case STATE_4_4:
			
			if (PIOC->PIO_ISR & (1<<28))
			{
				//printf("second high bicar\n");
				tc_stop(TC0,1);
				fflag_bic=1;
				b_state = STATE_4;
				pio_disable_interrupt(PIOC,PIO_IER_P28);
			}
			break;
		}
	}
}*/

void TC0_Handler()
{
	TC1->TC_CHANNEL[0].TC_IDR = TC_IER_CPCS;
	
	PIOD->PIO_WPMR	= 0x50494F00;
	PIOC->PIO_WPMR  = 0x50494F00;
	PIOD->PIO_OWER  = 0x00000003;
	PIOD->PIO_OWDR  = 0x0000000C;
	PIOC->PIO_OWER  = 0x000001FE;
	PIOC->PIO_OWDR  = 0x000FF000;
	count = count+1;
	TC1_cnt++;
	Cnt_TC0_total++;
	
	
	if(count<64)
	{
		PIOD->PIO_ODSR = array_acid[count].dir;
		PIOC->PIO_ODSR = array_acid[count].AB;
		//printf("%x %x\n",array_acid[count].dir,array_acid[count].AB);
	}
	if (count >=64)
	{
		count = 0;
		PIOD->PIO_ODSR = array_acid[count].dir;
		PIOC->PIO_ODSR = array_acid[count].AB;
		//printf("%x %x\n",array_acid[count].dir,array_acid[count].AB);
	}
	
		PIOD->PIO_WPMR	= 0x50494F00;
		PIOC->PIO_WPMR  = 0x50494F00;
		PIOD->PIO_OWER  = 0x0000000C;
		PIOD->PIO_OWDR  = 0x00000003;
		PIOC->PIO_OWER  = 0x000FF000;
		PIOC->PIO_OWDR  = 0x000001FE;

		if(count<64)
		{
			PIOD->PIO_ODSR = array_bicar[count].dir;
			PIOC->PIO_ODSR = array_bicar[count].AB;
		}
		if (count >=64)
		{
			count = 0;
			PIOD->PIO_ODSR = array_bicar[count].dir;
			PIOC->PIO_ODSR = array_bicar[count].AB;
		}
	
	
	if ((tc_get_status(TC0, 0) & TC_SR_CPCS) == TC_SR_CPCS)
	{
		;
	}
	TC1->TC_CHANNEL[0].TC_IER = TC_IER_CPCS;
}







// void TC0_Handler()
// {
// 	TC1->TC_CHANNEL[0].TC_IDR = TC_IER_CPCS;
// 	
// 	PIOD->PIO_WPMR	= 0x50494F00;
// 	PIOC->PIO_WPMR  = 0x50494F00;
// 	PIOD->PIO_OWER  = 0x00000003;
// 	PIOD->PIO_OWDR  = 0x0000000C;
// 	PIOC->PIO_OWER  = 0x000001FE;
// 	PIOC->PIO_OWDR  = 0x000FF000;
// 	count = count+1;
// 	TC1_cnt++;
//    Cnt_TC0_total++;
// 	
// 	if(count<64)
// 	{
// 		PIOD->PIO_ODSR = array_acid[count].dir;
// 		PIOC->PIO_ODSR = array_acid[count].AB;
// 		//printf("%x %x\n",array_acid[count].dir,array_acid[count].AB);
// 	}
// 	if (count >=64)
// 	{
// 		count = 0;
// 		PIOD->PIO_ODSR = array_acid[count].dir;
// 		PIOC->PIO_ODSR = array_acid[count].AB;
// 		//printf("%x %x\n",array_acid[count].dir,array_acid[count].AB);
// 	}
// 	if ((tc_get_status(TC0, 0) & TC_SR_CPCS) == TC_SR_CPCS)
// 	{
// 	
// 	}
// 	TC1->TC_CHANNEL[0].TC_IER = TC_IER_CPCS;
// }

void TC1_Handler()
{	
	TC1->TC_CHANNEL[0].TC_IDR = TC_IER_CPCS;
	PIOD->PIO_WPMR	= 0x50494F00;
	PIOC->PIO_WPMR  = 0x50494F00;
	PIOD->PIO_OWER  = 0x0000000C;
	PIOD->PIO_OWDR  = 0x00000003;
	PIOC->PIO_OWER  = 0x000FF000;
	PIOC->PIO_OWDR  = 0x000001FE;
	count1 = count1+1;
	TC2_cnt++;
	Cnt_TC1_total++;
	if(count1<64)
	{
		PIOD->PIO_ODSR = array_bicar[count1].dir;
		PIOC->PIO_ODSR = array_bicar[count1].AB;
	}
	if (count1 >=64)
	{
		count1 = 0;
		PIOD->PIO_ODSR = array_bicar[count1].dir;
		PIOC->PIO_ODSR = array_bicar[count1].AB;
	}
	if ((tc_get_status(TC0, 1) & TC_SR_CPCS) == TC_SR_CPCS)
	{
		
		
	}
	TC1->TC_CHANNEL[0].TC_IER = TC_IER_CPCS;
}
void TC2_Handler()
{
// 	if (sv_acc_flag == true && sv_end_flag == false)
// 	{
// 		i2c_counter++;
// 		if (i2c_counter > 1)
// 		{
// 			//CPU3_D_INIT_IIC();
// 			i2c_counter=0;
// 			sv_acc_flag = false;
// 			sv_rxrdy_flag = false;
// 			sv_end_flag = false;
// 		}
// 	}
// 	else
// 	{
// 		i2c_counter = 0;
// 	}
	//printf("reached 2sec\n");
	tc_stop(TC0,0);
//	tc_stop(TC0,1);

// if(!toggle)
// {
// 	PIOB->PIO_SODR = 1 << 19 ;
// 	toggle = 1;
// }
// else
{
//	PIOB->PIO_CODR = 1 << 19 ;
	toggle = 0;
}

	if ((tc_get_status(TC0, 2) & TC_SR_CPCS) == TC_SR_CPCS)
	{
		
	}
}
void TC3_Handler()
{
	OneSecFlag = 1;
	
	if ((tc_get_status(TC1, 0) & TC_SR_CPCS) == TC_SR_CPCS)
	{
		
	}
	tc_stop(TC1,0);
}

void TC7_Handler(){
	
	OneSecFlag = 1;
	
	if ((tc_get_status(TC2, 1) & TC_SR_CPCS) == TC_SR_CPCS)
	{
		
	}
	tc_stop(TC2,1);
	
}

void TC6_Handler()
{
	NVIC_ClearPendingIRQ(TC6_IRQn);
	tick++;
	if (tick == 10)
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
		//printf("1sec\n");
	}
	
	if ((tc_get_status(TC2, 0) & TC_SR_CPCS) == TC_SR_CPCS)
	{
		//printf("i m in handler tc6\n");
	}
}