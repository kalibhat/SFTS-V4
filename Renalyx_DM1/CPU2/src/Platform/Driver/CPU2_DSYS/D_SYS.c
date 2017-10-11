/*
 * D_SYS.c
 *
 * Created: 12/3/2014 3:29:18 PM
 *  Author: wid7
 */ 
/*
 * DD_SYS.c
 *
 * Created: 1/6/2014 10:46:00 AM
 *  Author: Geekay
 */ 

#include "D_SYS.h"
#include "Platform/Service/CPU2_S_interface.h"
extern uint32_t DUTY;
uint8_t CPU2_D_SET_PUMP(sv_pumptype id_pump)
{
	switch (id_pump)
	{
		
		case BLOODPUMP: //BLOOD PUMP
			PWM->PWM_CH_NUM[4].PWM_CPRD = 0x00000A41;
			PWM->PWM_CH_NUM[4].PWM_CDTY = DUTY;			//35% duty cycle
			PWM->PWM_ENA =0x00000010;
			gpio_configure_pin(PIN_PWM_LED0_GPIO,PIN_PWM_LED0_FLAGS);
		break;
		default:
		break;
	}
	
	return 0;
}

uint8_t CPU2_D_RESET_PUMP(sv_pumptype id_pump)
{
	switch (id_pump)
	{
		
		case BLOODPUMP: //BLOOD PUMP
			PWM->PWM_DIS =0x00000010;
		break;
		
		default:
		break;
		
	}
	return 0;
}


uint8_t CPU2_D_SET_VALVE(sv_valvetype id_valve)
{
	uint32_t p_io = 0, peri;
	switch(id_valve)
	{
		
		case HEMO_IF4:
		p_io = 21;
		peri = PA;
		break;
		
		case CLAMP_PWR:
		p_io = 13;
		peri = PA;
		break;
		
		case HEMO_IF3:
		p_io = 22;
		peri = PA;
		break;
		
		case BYPS_PWR:
		p_io = 11;
		peri = PA;
		break;
		
		case RUDNCY_INT:
		p_io = 23;
		peri = PC;
		break;
		
		case CPU_INT:
		p_io = 18;
		peri = PC;
		break;
		
		default:
		break;
	}
	
	CPU2_D_SET_PIO(peri,p_io);
	return 0;
}


uint8_t CPU2_D_RESET_VALVE(sv_valvetype id_valve)
{
	uint32_t p_io = 0, peri;
	switch(id_valve)
	{
// 		case VENOUS_PUMP:
// 		p_io = 25;
// 		peri = PB;
// 		break;
		
// 		case VENOUS_VALVE:
// 		p_io = 26;
// 		peri = PB;
// 		break;
// 		
// 		case ART_PUMP:
// 		p_io = 10;
// 		peri = PD;
// 		break;
// 		
		case HEMO_IF4:
		p_io = 21;
		peri = PA;
		break;
		
		case CLAMP_PWR:
		p_io = 13;
		peri = PA;
		break;
		
		case HEMO_IF3:
		p_io = 22;
		peri = PA;
 		break;
		
		case BYPS_PWR:
		p_io = 11;
		peri = PA;
		break;
		
		case RUDNCY_INT:
		p_io = 23;
		peri = PC;
		break;
		
		case CPU_INT:
		p_io = 18;
		peri = PC;
		break;
		
		default:
		break;
	}
	CPU2_D_RESET_PIO(peri,p_io);
	return 0;
}

uint8_t CPU2_D_RD_SENSIN(sv_sensintype id_sensein)
{
	uint32_t p_io, peri;
	uint8_t pio_val= 0;
	switch(id_sensein)
	{
		case SV_BP_DOOR:
		p_io = 10;               // bp_door
		peri = PC;
		break;
		
		case SV_BP_FB:					// pump_sense
		p_io = 9;
		peri = PC;
		break;
		
		case SV_CLAMP_FB:					// clamp_feedback
		p_io = 7;
		peri = PC;
		break;
		
		case SV_CLAMP_DOOR:					// clamp_door
		
		p_io = 7;
		peri = PC;
		break;
		
		case SV_BD:					// blood dectector
		
		p_io = 12;
		peri = PB;
		break;
		
		case SV_ABD:					// ABD
		
		p_io = 8;
		peri = PC;
		break;
		
		case SV_REDUNDANCY_IRQ:					//Redundancy_irq
		p_io = 24;
		peri = PC;
		break;
		
		default:
		break;
	}
	
	pio_val = CPU2_D_READ_PIO(peri, p_io);
	
	return pio_val;
	
}


uint16_t CPU2_D_READ_SENSORS(sv_sensortype ID_SENSOR)
{
	int16_t SENSOR_VAL;
	 uint8_t channel = 0;
	//adc_chnl channel = channel_0;
	 switch(ID_SENSOR)
	 {
		 case SV_APT:
		 //channel = channel_7;
		 SENSOR_VAL = DD_READ_SENSORS(SV_APT);
		 channel = 0;
		 break;
		 
		 case SV_VPT:
		SENSOR_VAL = DD_READ_SENSORS(SV_VPT);
		 channel =  1;
		 break;
		  
// 		 case SV_SENSOR_ID3:
// 		 //channel = channel_5;
// 		 channel =  2;
// 		 break;
// 		 
// 		 case SV_SENSOR_ID4:
// 		 //channel = channel_4;
// 		 channel =  3;
// 		 break;
// 		 
// 		 case SV_SENSOR_ID5:
// 		 //channel = channel_3;
// 		 channel =  4;
// 		 break;
		 
		 default:
		 break;
		 
	 }
	
//	SENSOR_VAL= CPU2_D_READ_ADC(channel);
	
	return SENSOR_VAL;
	
}

uint8_t CPU2_D_CONFIG_TIMER(uint8_t timer_no, uint8_t timer_res)
{
	uint32_t ch_no=0,rc;
	double rc_cal;
	rc_cal = ((84000000/(128 * timer_res)) -1);
	rc = rc_cal;

	switch (timer_no)
	{
		case 0:
		CPU2_D_MAIN_TIMER( TC2, ch_no, TC6_IRQn, rc);
		break;
		
		case 1:
		rc = timer_res;
		startTimer1(TC0,0,TC0_IRQn,rc);
		break;
		
		case 2:
		rc = timer_res;
		startTimer2(TC0,1,TC1_IRQn,rc);
		break;
		
		default:
		break;
		
	}
}

uint8_t CPU2_D_STOP_TIMER(uint8_t timer_no)
{
	//uint32_t ch_no=0;
	switch (timer_no)
	{
		case 0:
		tc_stop(TC0,0);
		break;
		
		case 1:
		tc_stop(TC0,1);
		break;
		
		case 2:
		tc_stop(TC2,0);
		break;
		
		default:
		break;	
	}
}

