/*
 * D_SYS.c
 *
 * Created: 12/10/2014 1:18:04 PM
 *  Author: wid7
 */ 
#include "D_SYS.h"
#include "Platform/Service/sv_interface.h"

uint8_t CPU3_D_SET_PUMP(sv_pumptype id_pump)
{
	PWM->PWM_CH_NUM[4].PWM_CPRD = 0x00000A41;
	PWM->PWM_CH_NUM[4].PWM_CDTY = 0x00000700;
	
	PWM->PWM_CH_NUM[5].PWM_CPRD = 0x00000A41;
	PWM->PWM_CH_NUM[5].PWM_CDTY = 0x00000700;
	
	switch (id_pump)
	{
		case ACID_PUMP: //ACID PUMP
		
		PWM->PWM_ENA =0x00000010;
		gpio_configure_pin(PIN_PWM_LED0_GPIO,PIN_PWM_LED0_FLAGS);
		break;
		
		case BICAR_PUMP:
		PWM->PWM_ENA =0x00000020;
		gpio_configure_pin(PIN_PWM_LED1_GPIO,PIN_PWM_LED1_FLAGS);
		break;
		
		default:
		break;
	}
	
	return 0;
}

uint8_t CPU3_D_RESET_PUMP(sv_pumptype id_pump)
{
	switch (id_pump)
	{
		
		case ACID_PUMP: //BLOOD PUMP
		PWM->PWM_DIS =0x00000010;
		break;
		
		case BICAR_PUMP:
		PWM->PWM_DIS =0x00000020;
		break;
		
		default:
		break;
		
	}
	return 0;
}


uint8_t CPU3_D_SET_VALVE(sv_valvetype id_valve)
{
	uint32_t p_io = 0, peri = 0;
	switch(id_valve)
	{
// 		case ACID_VALVE:
// 		p_io = 14;
// 		peri = PA;
// 		break;
		
		case CPU_INT:
		p_io = 27;
		peri = PB;
		break;
		
// 		case BICARBONATE_VALVE:
// 		p_io = 15;
// 		peri = PA;
// 		break;
		
		
		case PROPO_IF3:
		p_io = 6;
		peri = PD;
		break;
		
		case PROPO_IF4:
		p_io = 5;
		peri = PD;
		break;
		
		case RUDNCY_INT:
		p_io = 23;
		peri = PC;
		break;
						
		default:
		break;
	}
	
	CPU3_D_SET_PIO(peri,p_io);
	return 0;
}


uint8_t CPU3_D_RESET_VALVE(sv_valvetype id_valve)
{
	uint32_t p_io = 0, peri = 0;
	switch(id_valve)
	{
			// 		case ACID_VALVE:
		// 		p_io = 14;
		// 		peri = PA;
		// 		break;
		
		case CPU_INT:
		p_io = 27;
		peri = PB;
		break;
		
		// 		case BICARBONATE_VALVE:
		// 		p_io = 15;
		// 		peri = PA;
		// 		break;
		
		
		case PROPO_IF3:
		p_io = 6;
		peri = PD;
		break;
		
		case PROPO_IF4:
		p_io = 5;
		peri = PD;
		break;
		
		case RUDNCY_INT:
		p_io = 23;
		peri = PC;
		break;
		
		default:
		break;
		
	}
	CPU3_D_RESET_PIO(peri,p_io);
	return 0;
}

uint8_t CPU3_D_RD_SENSIN(sv_sensintype id_sensein)
{
	uint32_t p_io, peri;
	uint8_t pio_val= 0;
	switch(id_sensein)
	{
		case SV_SENSIN_ID1:
		p_io = 14;							// Acid F/B
		peri = PA;
		break;
		
		case SV_SENSIN_ID2:					// Bicarb F/B
		p_io = 15;
		peri = PA;
		break;
		
		case SV_SENSIN_ID3:					// Flow Pump CS
		p_io = 26;
		peri = PC;
		break;
		
		case SV_SENSIN_ID4:					// B_PROPO_I/F1
		
		p_io = 8;
		peri = PD;
		break;
		
		case SV_SENSIN_ID5:					// B_PROPO_I/F2
		
		p_io = 7;
		peri = PD;
		break;
		
// 		case SV_SENSIN_ID6:					// ABD
// 		
// 		p_io = 5;
// 		peri = PD;
// 		break;
		
		case SV_SENSIN_ID7:					//Redundancy_irq
		p_io = 24;
		peri = PC;
		break;
		
		default:
		break;
	}
	
	pio_val = CPU3_D_READ_PIO(peri, p_io);
	
	return pio_val;
	
}


uint16_t CPU3_D_READ_SENSORS(sv_sensortype ID_SENSOR)
{
	int16_t SENSOR_VAL;
	uint8_t channel = 0;
	//adc_chnl channel = channel_0;
	switch(ID_SENSOR)
	{
		case SV_SENSOR_ID1:			//Acid Current A
		//channel = channel_7;
		channel = 7;
		break;
						
		case SV_SENSOR_ID2:				//Acid current B
		//channel =  channel_6;
		channel =  6;
		break;
		
		case SV_SENSOR_ID3:				// Bicarb Current A
		//channel = channel_5;
		channel =  5;
		break;
		
		case SV_SENSOR_ID4:				// Bicarb current B
		//channel = channel_4;
		channel =  3;
		break;
		
// 		case SV_SENSOR_ID5:				// temp
// 		//channel = channel_3;
// 		channel =  3;
// 		break;
		
		default:
		break;
		
	}
	
	SENSOR_VAL= CPU3_D_READ_ADC(channel);
	
	return SENSOR_VAL;
	
}

uint8_t CPU3_D_CONFIG_TIMER(uint8_t timer_no, uint8_t timer_res)
{
	uint32_t ch_no=0,rc;
	double rc_cal;
	rc_cal = ((84000000/(128 * timer_res)) -1);
	rc = rc_cal;

	switch (timer_no)
	{
		case 0:
		CPU3_D_MAIN_TIMER( TC2, ch_no, TC6_IRQn, rc);
		break;
		
		case 1:
		rc = timer_res;
		acid_startTimer1(TC0,0,TC0_IRQn,rc);
		break;
		
		case 2:
		rc = timer_res;
		//acid_startTimer2(TC0,1,TC1_IRQn,rc);
		break;
		
		case 3:
		rc = timer_res;
		//bicar_startTimer2(TC1,0,TC1_IRQn,rc);
		break;
		
		case 4:
		rc = timer_res;
		//bicar_startTimer2(TC1,1,TC1_IRQn,rc);
		break;
		
		
		default:
		break;
		
	}
}

uint8_t CPU3_D_STOP_TIMER(uint8_t timer_no)
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
		
		case 3:
		tc_stop(TC1,0);
		break;
		
		case 4:
		tc_stop(TC1,1);
		break;
		
		default:
		break;
	}
}