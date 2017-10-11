/*
 * DD_SYS.c
 *
 * Created: 1/6/2014 10:46:00 AM
 *  Author: Geekay
 */ 

#include "DD_SYS.h"
#include "Platform/Service/sv_interface.h"
#include "Platform/Drivers/DD_ADC/DD_ADC.h"
#include "cl_types.h"
#include "cl_consolecontroller.h"


#define BPDOOR 
#define BP_POWERCONTROL
#define BP_ON_OFF
#define HP_DIR
#define HP_ENABLE
#define VENOUSCLAMP
#define UFPUMP_ENABLE
#define HEATER
#define ALARM
#define INT

extern int gmstick ;
extern Cl_ReturnCodeType cl_wait(uint32_t );
extern Cl_ReturnCodeType  Cl_SendDatatoconsole(Cl_ConsoleTxCommandtype , uint8_t* ,uint8_t );
extern uint8_t DD_DIS_PERI_AND_RESET_PIO(pio_type peripheral,uint32_t pio);
extern uint8_t DD_EN_PERI_AND_SET_PIO(pio_type peripheral,uint32_t pio);

uint8_t DD_SET_VALVE_1_16_IO(void);
uint8_t DD_SET_VALVE_17_20_IO(void);
uint8_t DD_SET_VALVE_21_32_IO(void);                // for resetting Alarm Lights

uint8_t DD_LATCH_STROBE_X1_X16(void);
uint8_t DD_LATCH_STROBE_X17_X20(void);


//uint8_t DD_RESET_VALVE_1_16_IO(void);
//uint8_t DD_RESET_VALVE_17_20_IO(void);
//uint8_t DD_RESET_VALVE_21_32_IO(void);                // for resetting Alarm Lights

static uint16_t dd_valve_port_1_16 = 0 ;
static uint16_t dd_valve_port_17_20 = 0 ;
static uint16_t dd_valve_port_21_32 = 0 ;

static bool ByPass = 0 , LoopBack = 0;

extern bool isouf;

//void BC_Switch1(void);
//void BC_Switch2(void);


uint8_t DD_ENABLE_BYPASS()
{
				DD_SET_VALVE (VALVE_ID15);
				DD_RESET_VALVE (VALVE_ID13);
				DD_RESET_VALVE (VALVE_ID14);
				ByPass = true;
	
	return 0;
}
uint8_t DD_DISABLE_BYPASS()
{
				DD_RESET_VALVE (VALVE_ID15);
				DD_SET_VALVE (VALVE_ID13);
				DD_SET_VALVE (VALVE_ID14);
				ByPass = false;
	
	return 0;
}


uint8_t DD_ENABLE_LOOPBACK()
{
	
	LoopBack = true;
				DD_SET_VALVE (VALVE_ID16);
				DD_RESET_VALVE (VALVE_ID18);
				
	return 0;
}
uint8_t DD_DISABLE_LOOPBACK()
{
	
	LoopBack = false;
				DD_RESET_VALVE (VALVE_ID16);
				DD_SET_VALVE (VALVE_ID18);
	return 0;
}
uint8_t DD_SET_PUMP(sv_pumptype id_pump)
{
	
	uint32_t p_io,peri;
	

	switch (id_pump)
	{
		case DCMOTOR1:  // DC MOTOR 1
		p_io = 20;
		peri = PA;
		PWM->PWM_ENA =0x00000004;
		gpio_configure_pin(PIN_PWM_LED5_GPIO,PIN_PWM_LED5_FLAGS);
		DD_EN_PERI_AND_SET_PIO(peri,p_io);
		//DD_SET_PIO(peri,p_io);
		break;
		
		case DCMOTOR2: // DC MOTOR 2
		p_io = 21;
		peri = PA;
		PWM->PWM_ENA =0x00000001;
		gpio_configure_pin(PIN_PWM_LED3_GPIO,PIN_PWM_LED3_FLAGS);
		DD_EN_PERI_AND_SET_PIO(peri,p_io);
		//DD_SET_PIO(peri,p_io);
		break;
		
		case BLOODPUMP: //BLOOD PUMP
	
		break;
		
		case UFPUMP: // UF PUMP
		p_io = 22;
		peri = PC;
		DD_RESET_PIO(peri,p_io);
		PWM->PWM_ENA =0x00000020;
		gpio_configure_pin(PIN_PWM_LED1_GPIO,PIN_PWM_LED1_FLAGS);
		
		p_io = 15;
		peri = PA;
		DD_RESET_PIO(peri,p_io);
		break;
		
		case HEPARINPUMP: // HEPARIN PUMP

		PWM->PWM_ENA =0x00000010;
//		gpio_configure_pin(PIN_PWM_LED0_GPIO,PIN_PWM_LED0_FLAGS);
		p_io = 2;
		peri = PD;
		DD_RESET_PIO(peri,p_io);

		break;
				
	}
	
	
	
	return 0;
}
uint8_t DD_SET_HEATER()
{
	uint32_t p_io,peri;
	
	p_io = 28;
	peri = PC;                               // tURN ON SAFETY RELAY  WAIT FOR 100ms BEFORE TRIAC (pwm) ON 
	DD_SET_PIO(peri,p_io);
	cl_wait(100);
	
	PWM->PWM_ENA =0x00000002;
	p_io = 12;
	peri = PA;
	//PIOA->PIO_ABSR = 0x00390000;            // TURN ON pwm ( TRIAC) AFTER 100ms DELAY
	PIOA->PIO_PDR = 1<<p_io;
	DD_EN_PERI_AND_SET_PIO(peri,p_io);
	
	
	
	//gpio_configure_pin(PIN_PWM_LED4_GPIO,PIN_PWM_LED4_FLAGS);
	
	return 0;
}

uint8_t DD_RESET_HEATER()
{
	uint32_t p_io,peri;
	
	PWM->PWM_DIS =0x00000002;
	//gpio_configure_pin(PIN_PWM_LED4_GPIO,PIN_PWM_LED4_FLAGS);
	
	
	p_io = 12;
	peri = PA;                                        // TURN OFF pwm ( TRIAC) AFTER 100ms DELAY
	//PIOA->PIO_ABSR = 0x00390000;
	PIOA->PIO_PDR = 1<<p_io;
	DD_DIS_PERI_AND_RESET_PIO(peri,p_io);
	
	cl_wait(100);
	p_io = 28;
	peri = PC;                                    // tURN OFF SAFETY RELAY  WAIT FOR 100ms BEFORE TRIAC (pwm) ON 
	DD_RESET_PIO(peri,p_io);
	return 0;
}


uint8_t DD_SET_ALARM(sv_alarmtype sv_alarm)
{
	uint32_t p_io,peri;
	peri=PC;
	p_io = 12;
	DD_SET_PIO(peri,p_io);
	
	return 0;
}

uint8_t DD_RESET_ALARM(sv_alarmtype sv_alarm)
{
	uint32_t p_io,peri;
	peri=PC;
	p_io = 12;
	DD_RESET_PIO(peri,p_io);
	
	return 0;
}



uint8_t DD_RESET_PUMP(sv_pumptype id_pump)
{
	
	uint32_t p_io,peri;
	switch (id_pump)
	{
		case DCMOTOR1:  // DC MOTOR 1
		p_io = 20;
		peri = PA;
		//DD_SET_DUTY(2,0);
		
		
		PWM->PWM_DIS =0x00000004;
		PIOA->PIO_ABSR = 0x00391000;
		PIOA->PIO_PDR = 1<<p_io;
		DD_DIS_PERI_AND_RESET_PIO(peri,p_io);
		
		//DD_RESET_PIO(peri,p_io);
		
		break;
		
		case DCMOTOR2: // DC MOTOR 2
		p_io = 21;
		peri = PA;
	//	DD_SET_DUTY(0,0);
		
		PWM->PWM_DIS =0x00000001;
		PIOA->PIO_ABSR = 0x00391000;
		PIOA->PIO_PDR = 1<<p_io;
		DD_DIS_PERI_AND_RESET_PIO(peri,p_io);
		
	//	DD_RESET_PIO(peri,p_io);
		
		break;
		
		case BLOODPUMP: //BLOOD PUMP

		break;
		
		case UFPUMP: // UF PUMP
		p_io = 22;
		peri = PC;
		DD_DIS_PERI_AND_RESET_PIO(peri,p_io);
	//	DD_SET_PIO(peri,p_io);
		PWM->PWM_DIS =0x00000020;
		
		p_io = 15;
		peri = PA;
		DD_SET_PIO(peri,p_io);
		break;
		
		case HEPARINPUMP: // HEPARIN PUMP
	
		p_io = 2;
		peri = PD;
		DD_SET_PIO(peri,p_io);
		//DD_DIS_PERI_AND_RESET_PIO(peri,p_io);
		//DD_SET_DUTY(4,0);
		PWM->PWM_DIS =0x00000010;
		break;
	
	}
	
	
	
	return 0;
	
}

uint8_t DD_INC_SPEED(sv_pumptype id_pump,uint32_t speed)
{
	
	uint32_t prd_val, dty_val;
	uint8_t pwm_ch_no;
	switch (id_pump)
	{
		case DCMOTOR1:  // DC MOTOR 1
		pwm_ch_no = 2;
		// convert into duty count
		DD_SET_DUTY(pwm_ch_no,dty_val);
		break;
		
		case DCMOTOR2: // DC MOTOR 2
		pwm_ch_no = 0;
		// convert into duty count
		DD_SET_DUTY(pwm_ch_no,dty_val);
		break;
		
		case BLOODPUMP: //BLOOD PUMP
		pwm_ch_no = 6;
		// convert into duty count
		DD_SET_PRDY(pwm_ch_no,prd_val);
		break;
		
		case UFPUMP: // UF PUMP
		pwm_ch_no = 5;
		// convert into duty count
		DD_SET_PRDY(pwm_ch_no,prd_val);
		break;
		
		case HEPARINPUMP: // HEPARIN PUMP
		pwm_ch_no = 4;
		// convert into duty count
		DD_SET_PRDY(pwm_ch_no,prd_val);
		break;
	}
	
	return 0;
}


uint8_t DD_DEC_SPEED(sv_pumptype id_pump,uint32_t speed)
{
	uint32_t prd_val, dty_val;
	uint8_t pwm_ch_no;
	switch (id_pump)
	{
		case DCMOTOR1:  // DC MOTOR 1
		pwm_ch_no = 2;
		// convert into duty count
		DD_SET_DUTY(pwm_ch_no,dty_val);
		break;
		
		case DCMOTOR2: // DC MOTOR 2
		pwm_ch_no = 0;
		// convert into duty count
		DD_SET_DUTY(pwm_ch_no,dty_val);
		break;
		
		case BLOODPUMP: //BLOOD PUMP
		pwm_ch_no = 6;
		// convert into duty count
		//DD_SET_PRDY(pwm_ch_no,prd_val);
		break;
		
		case UFPUMP: // UF PUMP
		pwm_ch_no = 5;
		// convert into duty count
		DD_SET_PRDY(pwm_ch_no,prd_val);
		break;
		
		case HEPARINPUMP: // HEPARIN PUMP
		pwm_ch_no = 4;
		// convert into duty count
		DD_SET_PRDY(pwm_ch_no,prd_val);
		break;
	}
	
	return 0;
}

uint8_t DD_SET_SPEED(sv_pumptype id_pump,uint32_t speed)
{
	uint32_t prd_val, dty_val;
	uint8_t pwm_ch_no;
	

	switch (id_pump)
	{
		case DCMOTOR1:  // DC MOTOR 1
		pwm_ch_no = 2;
		dty_val = speed;
		// convert into duty count
		DD_SET_DUTY(pwm_ch_no,dty_val);
		break;
		
		case DCMOTOR2: // DC MOTOR 2
		pwm_ch_no = 0;
		dty_val = speed;
		// convert into duty count
		DD_SET_DUTY(pwm_ch_no,dty_val);
		break;
		
		case BLOODPUMP: //BLOOD PUMP
	//	pwm_ch_no = 6;
		switch(speed)
		{
			case 200:
			dty_val = 0x00000170;
			prd_val =  0x00000375;
			break;
			case 240:
			dty_val = 0x00000150;
			prd_val =  0x00000300;
			break;
			case 280:
			dty_val = 0x00000130;
			prd_val =  0x0000280;
			break;
			case 320:
			dty_val = 0x00000120;
			prd_val =  0x00000250;
			break;
			case 360:
			dty_val = 0x00000110;
			prd_val =  0x00000210;
			break;
			default:
			//dty_val = speed/2;
			break;
		}
		
		// convert into duty count
		DD_SET_DUTY(pwm_ch_no,dty_val);
		DD_SET_PRDY(pwm_ch_no,prd_val);
		break;
		
		case UFPUMP: // UF PUMP
		pwm_ch_no = 5;
		dty_val = speed/2;
		// convert into duty count
	//	DD_SET_PRDY(pwm_ch_no,prd_val);
		DD_SET_DUTY(pwm_ch_no,dty_val);
		DD_SET_PRDY(pwm_ch_no,speed);
		break;
		
		case HEPARINPUMP: // HEPARIN PUMP
		pwm_ch_no = 4;
		// convert into duty count
		dty_val = speed/2;
		
		DD_SET_DUTY(pwm_ch_no,dty_val);
		DD_SET_PRDY(pwm_ch_no,speed);
		//DD_SET_PRDY(pwm_ch_no,prd_val);
		break;
	}
	
	return 0;
}

uint8_t DD_SET_PUMPFREQ(sv_pumptype id_pump,uint32_t freq)
{
		uint32_t prd_val =0x00001000;
		uint8_t pwm_ch_no;
		
			
			prd_val = freq;
			
			switch (id_pump)
			{
				case DCMOTOR1:  // DC MOTOR 1
				pwm_ch_no = 2;
								
				DD_SET_PRDY(pwm_ch_no,prd_val);
								
				break;
				
				case DCMOTOR2: // DC MOTOR 2
				pwm_ch_no = 0;
								
				DD_SET_PRDY(pwm_ch_no,prd_val);
							
				
				break;
				default:
				break;
			}
	
		
	return 0;
}


uint8_t DD_CHNG_HTR(uint32_t dty_val)
{
	;
	uint8_t pwm_ch_no = 1;
	
	DD_SET_DUTY(pwm_ch_no,dty_val);
	
	return 0;
}


uint8_t DD_SET_DIR()               // Heparin Direction
{
	uint32_t p_io, peri;
	
	p_io=1;
	peri = PD;
	
	DD_SET_PIO(peri,p_io);
	
	return 0;
}

uint8_t DD_RESET_DIR()   // Heparin Direction
{
	uint32_t p_io, peri;
	
	p_io=1;
	peri = PD;
	
	DD_RESET_PIO(peri,p_io);
	
	return 0;
}
uint8_t DD_SET_RED()               // RED
{
	DD_SET_VALVE(ALARM_RED);
	return 0;
}
uint8_t DD_SET_GREEN()               // GREEN
{
	DD_SET_VALVE(ALARM_GREEN);
	return 0;
}
uint8_t DD_SET_YELLOW()               // YELLOW
{
	DD_SET_VALVE(ALARM_AMBER);
	return 0;
}
uint8_t DD_SET_BUZZER()               // BUZZZER
{
	DD_SET_VALVE(ALARM_BUZZER);
	return 0;
}
uint8_t DD_RESET_RED()               // RED
{
	DD_RESET_VALVE(ALARM_RED);
	return 0;
}
uint8_t DD_RESET_GREEN()               // GREEN
{
	DD_RESET_VALVE(ALARM_GREEN);
	return 0;
}
uint8_t DD_RESET_YELLOW()               // YELLOW
{
	DD_RESET_VALVE(ALARM_AMBER);
	return 0;
}
uint8_t DD_RESET_BUZZER()               // BUZZZER
{
	DD_RESET_VALVE(ALARM_BUZZER);
	return 0;
}
uint8_t DD_SET_VALVE(sv_valvetype id_valve)
{
	uint32_t p_io, peri;

     if((id_valve >=1) && (id_valve <= 16))
	
		{
	
		dd_valve_port_1_16 |= 1 << id_valve - 1;

		DD_SET_VALVE_1_16_IO();

		DD_LATCH_STROBE_X1_X16();
		
		}

	   else	if((id_valve >=17) && (id_valve <= VALVE_MAX))
		{

					dd_valve_port_17_20 |= 1 << id_valve - 17;         // Earlier 6

					DD_SET_VALVE_17_20_IO();

			        DD_LATCH_STROBE_X17_X20();
		}
		
		else if ((id_valve >=29) && (id_valve <= 32))                                  // for alarm indicators
		{
			dd_valve_port_17_20 |= 1 << id_valve - 17;
//			DD_SET_VALVE_21_32_IO();
DD_SET_VALVE_17_20_IO();
			DD_LATCH_STROBE_X17_X20();
		}
				
	

	return 0;
}

uint8_t DD_RESET_VALVE(sv_valvetype id_valve)
{
	uint32_t p_io, peri;

	if(id_valve == VALVE_ID1)
	{
		Cl_SendDatatoconsole(CON_TX_COMMAND_PRINTTEXT,"V17_0",6);
	}
	
	if((id_valve >=1) && (id_valve <= 16))
	{
	
		dd_valve_port_1_16 = dd_valve_port_1_16 &= ~(1 << id_valve - 1);
		DD_SET_VALVE_1_16_IO();
		DD_LATCH_STROBE_X1_X16();
		
	}

	else if((id_valve >=17) && (id_valve <= VALVE_MAX))
	{
		dd_valve_port_17_20 = dd_valve_port_17_20 &= ~(1 << id_valve - 17);
		DD_SET_VALVE_17_20_IO();
		DD_LATCH_STROBE_X17_X20();
		
	}
	
	else if ((id_valve >=29) && (id_valve <= 32))                                  // for alarm indicators
	{
		dd_valve_port_17_20 = dd_valve_port_17_20 &= ~(1 << id_valve - 17);
//		DD_SET_VALVE_21_32_IO();
DD_SET_VALVE_17_20_IO();
		DD_LATCH_STROBE_X17_X20();
	}

	
	return 0;
}

uint8_t DD_SET_VALVE_1_16_IO()

{
	uint8_t count = 1;
	uint32_t p_io, peri;
	 while (count <= 16)
	 {
			switch (count)
			{	
			
			case VALVE_ID1:
			p_io = 10;
			peri = PB;
			break;
			
			case VALVE_ID2:
			p_io = 2;
			peri = PC;
			break;
			
			case VALVE_ID3:
			p_io = 3;
			peri = PC;
			break;
			
			case VALVE_ID4:
			p_io = 4;
			peri = PC;
			break;
			
			case VALVE_ID5:
			p_io = 5;
			peri = PC;
			break;
	        case VALVE_ID6:
			p_io = 6;
			peri = PC;
			break;
						
			case VALVE_ID7:
			p_io = 7;
			peri = PC;
			break;
			case VALVE_ID8:
			p_io = 8;
			peri = PC;
			break;
						
			case VALVE_ID9:
			p_io = 9;
			peri = PC;
			break;
						
			case VALVE_ID10:
			p_io = 10;
			peri = PC;
			break;
							
			case VALVE_ID11:
			p_io = 11;
			peri = PC;
			break;
						
			case VALVE_ID12:
			p_io = 12;
			peri = PC;
			break;
						
			case VALVE_ID13:
			p_io = 13;
			peri = PC;
			break;
						
			case VALVE_ID14:
			p_io = 14;
			peri = PC;
			break;
						
			case VALVE_ID15:
			p_io = 15;
			peri = PC;
			break;
		
			case VALVE_ID16:
			p_io = 16;
			peri = PC;
			break;
					
			default:
			break;
				}
		if (dd_valve_port_1_16 & 1<< count-1 )
						{
							if (count == VALVE_ID1 )
							{
								DD_SET_PIO(peri,p_io);
							}
							else
							DD_RESET_PIO(peri,p_io);
						}
						else
						{
							if (count == VALVE_ID1 )
							{
								DD_RESET_PIO(peri,p_io);
							}
							else
							DD_SET_PIO(peri,p_io);
						}
			count++;
		}
	return 0;
}

uint8_t DD_SET_VALVE_17_20_IO()
{
		uint8_t count = 17;
		uint32_t p_io, peri;
		while (count <= 32)  // 32
		{
			switch (count)
			{
				
					case VALVE_ID17:
					p_io = 1;
					peri = PC;
					break;
				
					case VALVE_ID18:
					p_io = 2;
					peri = PC;
					break;
				
					case VALVE_ID19:
					p_io = 3;
					peri = PC;
					break;
				
					case VALVE_ID20:
					p_io = 4;
					peri = PC;
					break;
				
					case ALARM_BUZZER:
					p_io = 13;
					peri = PC;
					break;
					case ALARM_RED:
					p_io = 14;
					peri = PC;
					break;
					case ALARM_AMBER:
					p_io = 15;
					peri = PC;
					break;
				
					case ALARM_GREEN:
					p_io = 16;
					peri = PC;
					break;
				
					default:
					break;
			}
					if (dd_valve_port_17_20 & 1<< count-17 )
	
					{

						DD_RESET_PIO(peri,p_io);
					}
			else
			{
			DD_SET_PIO(peri,p_io);
			}
				
		count++;
		if (count == 21)
		{
			count = 29;
		}
	}
	
	return 0;
}

uint8_t DD_SET_VALVE_21_32_IO()
{
	uint8_t count = 29;
	uint32_t p_io, peri;
	while (count <= 32)  // 32
	{
		switch (count)
		{
						
			case ALARM_BUZZER:
			p_io = 13;
			peri = PC;
			break;
			case ALARM_RED:
			p_io = 14;
			peri = PC;
			break;
			case ALARM_AMBER:
			p_io = 15;
			peri = PC;
			break;
			
			case ALARM_GREEN:
			p_io = 16;
			peri = PC;
			break;
			
			default:
			break;
		}
		if (dd_valve_port_17_20 & 1<< count-17 )
		
		{

			DD_RESET_PIO(peri,p_io);
		}
		else
		{
			DD_SET_PIO(peri,p_io);
		}
		
		count++;
	}
	
	return 0;
	
}




uint8_t DD_SET_CLAMP()		// Venous Clamp
{
	uint32_t p_io, peri;
	
	p_io=27;
	peri = PC;
	
	DD_SET_PIO(peri,p_io);
	
	return 0;
}

uint8_t DD_RESET_CLAMP()   // Venous Clamp
{
	uint32_t p_io, peri;
	
	p_io=27;
	peri = PC;
	
	DD_RESET_PIO(peri,p_io);
	
	return 0;
}

uint8_t DD_RD_SENSIN(sv_sensintype id_sensein)
{
	uint32_t p_io, peri;
	uint8_t pio_val= 0;
	switch(id_sensein)
	{
		case SV_SENSIN_ID1:
	
		p_io = 7;               // lvl
		peri = PA;
		break;
		
		case SV_SENSIN_ID2:					// dgpfeedback
		p_io = 25;
		peri = PC;
		break;
		
		case SV_SENSIN_ID3:					// fpcurrentsens
		p_io = 12;
		peri = PB;
		break;
		
		case SV_SENSIN_ID4:					// uf_feeddback
		
		p_io = 0;
		peri = PD;
		break;
		
		case SV_SENSIN_ID5:					// BP_feedback
		
		p_io = 30;
		peri = PC;
		break;
		
		case SV_SENSIN_ID6:					// HP_START
		
		p_io = 6;
		peri = PD;
		break;
		
		case SV_SENSIN_ID7:					//HP_STOP
		p_io = 9;
		peri = PD;
		break;
		
		case SV_SENSIN_ID8:					//ABD
		p_io = 5;
		peri = PD;
		break;
		
		case SV_SENSIN_ID9:					// holder sense1
		p_io = 8;
		peri = PB;
		break;
		
		case SV_SENSIN_ID10:				// holder sense 2
		
		p_io = 18;
		peri = PB;
		break;
		
		case SV_SENSIN_ID11:					//fpfeedback
		p_io = 26;
		peri = PC;
		break;
		
		case SV_SENSIN_ID12:		//flow_switch
		p_io =19;
		peri =PA;
		break;
		case SV_SENSIN_ID13: //heparin_feedback
		p_io =3;
		peri =PD;
		break;
		case SV_SENSIN_ID14: //bloodpump_door
		p_io =16;
		peri =PB;
		break;
		case SV_SENSIN_ID15: // bp_inc
		p_io =18;
		peri =PC;
		break;
		case SV_SENSIN_ID16: // bp_dec
		p_io =4;
		peri =PD;
		break;
		case SV_SENSIN_ID17: //venous_clamp_feedback
		p_io =22;
		peri =PB;
		break;
		case SV_SENSIN_ID18: //venous_clamp_door
		p_io =10;
		peri =PD;
		break;
		case SV_SENSIN_ID19:
 		p_io =23;					//B_PROPO_I/F3
 		peri =PB;
		break;
		case SV_SENSIN_ID20:  // acid in
		p_io =9;
		peri =PB;
		
		break;
		case SV_SENSIN_ID21:				// bicarb in
		p_io =25;
		peri =PB;
		break;
		case SV_SENSIN_ID22:		//cpu2_interrupt
		p_io =24;
		peri =PB;
		break;
		case SV_SENSIN_ID23:		//cpu3_interrupt
			p_io =27;
			peri =PB;
		break;
		case SV_SENSIN_ID24:		//redundancy_irq
			p_io =24;
			peri =PC;
		break;
		case SV_SENSIN_ID25:			//Blood pump manual on off
		p_io =14;
		peri =PA;
		break;
		case SV_SENSIN_ID26:				//blood detector
			p_io =15;
			peri =PB;
		break;
		case SV_SENSIN_ID27:			//blood_leak_detector
		p_io =29;
		peri =PC;
		break;
		case SV_SENSIN_ID28:			//ufpump_ic_fault
		p_io =20;
		peri =PC;
		break;
		case SV_SENSIN_ID29:			//ufpump_home
		p_io =17;
		peri =PC;
		break;
		case SV_SENSIN_ID30:			//disinfectant_fdbck
		p_io =19;
		peri =PC;
		break;
		case SV_SENSIN_ID31:			//b_hemo_f4
		p_io =16;
		peri =PA;
		break;
		case SV_SENSIN_ID32:			//b_hemo_f3
		p_io =24;
		peri =PA;
		break;
		case SV_SENSIN_ID33:			//B_PROPO_I/F4
		p_io =21;
		peri =PB;
		break;
		case SV_SENSIN_ID34:			//Calibration
		p_io =10;
		peri =PA;
		break;
		case SV_SENSIN_ID35:			//BATTERY_LOAD_CURRENT
		p_io =2;
		peri =PA;
		break;
		case SV_SENSIN_ID36:			//POWER_FAILURE_INDICATION
		p_io =26;
		peri =PB;
		break;
		case SV_SENSIN_ID37:			//SYSTEM_SHUT_DOWN
		p_io =4;
			peri =PA;
		break;
		default:
		break;
	}
	
	pio_val = DD_READ_PIO(peri, p_io);
	
	return pio_val;
	
}



uint8_t DD_CONSOLE_TX(uint8_t sv_consoleTxdatabyte)
{
	uint8_t sv_retval = 1;
	sv_retval = (uint8_t) DD_TX_UART(sv_consoleTxdatabyte);
	
	return sv_retval;
}

uint8_t DD_CONSOLE_RX(uint8_t dd_consoleRxdatabyte )
{
	uint8_t sv_consoleRxdatabyte = dd_consoleRxdatabyte;
	
	sv_console_receivedbyte(sv_consoleRxdatabyte);
	
	
	return 1;
}

uint8_t DD_CONFIG_TIMER(uint8_t timer_no, uint8_t timer_res)
{
	uint32_t ch_no=0,rc;
	double rc_cal;
	rc_cal = ((84000000/(128 * (1/timer_res))) -1);
	rc = rc_cal;

	switch (timer_no)
	{
		case 0:
		
		DD_SET_TIMER( TC0, ch_no, TC0_IRQn, rc);
		break;
		
		case 1:
		rc=3281249; //value to get 5sec delay (mlk/128*interrupt frequency)-1 number counts to get 1sec
		DD_SET_TIMER( TC1, ch_no, TC3_IRQn, rc);
		break;
		
		case 2:
		
		rc=1968749; //value to get 1sec delay (mlk/128*interrupt frequency)-1 number counts to get 1sec
		DD_SET_TIMER(TC2, ch_no, TC6_IRQn, rc);
		
		break;
		
	}
}

uint8_t DD_STOP_TIMER(uint8_t timer_no)
{
	uint32_t ch_no=0;
	switch (timer_no)
	{
		case 0:
		
		tc_stop(TC0,ch_no);
		
		break;
		
		case 1:
		tc_stop(TC1,ch_no);
		break;
		
		case 2:
		
		tc_stop(TC2,ch_no);
		
		break;
		
	}
	
}

uint8_t DD_SET_FLOW_PATH(sv_flowpathtype sv_flowpath)
{
	static  bool valve_status = 0;
	static bool alarm_on = false;
	
	
	switch(sv_flowpath)
	{
			
		case FLOW_PATH_BC_ALARM:
			DD_RESET_VALVE (VALVE_ID5);
			DD_RESET_VALVE (VALVE_ID6);
			DD_RESET_VALVE (VALVE_ID7);
			DD_RESET_VALVE (VALVE_ID8);
			DD_RESET_VALVE (VALVE_ID9);
			DD_RESET_VALVE (VALVE_ID10);
			DD_RESET_VALVE (VALVE_ID11);
			DD_RESET_VALVE (VALVE_ID12);
		break;
				
		case FLOW_PATH_IDLE_RINSE:
	
		DD_RESET_VALVE(VALVE_ID1);
		DD_RESET_VALVE (VALVE_ID2);
		DD_RESET_VALVE (VALVE_ID3);
		DD_RESET_VALVE (VALVE_ID4);
		DD_RESET_VALVE (VALVE_ID5);
		DD_RESET_VALVE (VALVE_ID6);
		DD_RESET_VALVE (VALVE_ID7);
		DD_RESET_VALVE (VALVE_ID8);
		DD_RESET_VALVE (VALVE_ID9);
		DD_RESET_VALVE (VALVE_ID10);
		DD_RESET_VALVE (VALVE_ID11);
		DD_RESET_VALVE (VALVE_ID12);
		DD_RESET_VALVE (VALVE_ID13);
		DD_RESET_VALVE (VALVE_ID14);
		DD_RESET_VALVE (VALVE_ID15);
		DD_RESET_VALVE (VALVE_ID16);
		DD_RESET_VALVE (VALVE_ID17);
		DD_RESET_VALVE (VALVE_ID18);
		DD_RESET_VALVE (VALVE_ID19);
	
		DD_RESET_PUMP(DCMOTOR2);
		DD_RESET_PUMP(DCMOTOR1);
				
				
		break;
		case V15_FLUSH:
		
		DD_SET_VALVE(VALVE_ID1);
	//	DD_RESET_VALVE (VALVE_ID2);
		DD_RESET_VALVE (VALVE_ID3);
		DD_RESET_VALVE (VALVE_ID4);
		DD_RESET_VALVE (VALVE_ID5);
		DD_RESET_VALVE (VALVE_ID6);
		DD_RESET_VALVE (VALVE_ID7);
		DD_RESET_VALVE (VALVE_ID8);
		DD_RESET_VALVE (VALVE_ID9);
		DD_RESET_VALVE (VALVE_ID10);
		DD_RESET_VALVE (VALVE_ID11);
		DD_RESET_VALVE (VALVE_ID12);
		DD_RESET_VALVE (VALVE_ID13);
		DD_RESET_VALVE (VALVE_ID14);
		DD_SET_VALVE (VALVE_ID15);
		DD_RESET_VALVE (VALVE_ID16); 
		DD_RESET_VALVE (VALVE_ID17);
		DD_SET_VALVE (VALVE_ID18);
		DD_RESET_VALVE (VALVE_ID19);
		
		DD_RESET_PUMP(DCMOTOR2);
		DD_RESET_PUMP(DCMOTOR1);
		
		
		break;
		
		case V2_FLUSH:
		
		DD_SET_VALVE(VALVE_ID1);
	//	DD_SET_VALVE (VALVE_ID2);
		DD_RESET_VALVE (VALVE_ID3);
		DD_RESET_VALVE (VALVE_ID4);
		DD_RESET_VALVE (VALVE_ID5);
		DD_RESET_VALVE (VALVE_ID6);
		DD_RESET_VALVE (VALVE_ID7);
		DD_RESET_VALVE (VALVE_ID8);
		DD_RESET_VALVE (VALVE_ID9);
		DD_RESET_VALVE (VALVE_ID10);
		DD_RESET_VALVE (VALVE_ID11);
		DD_RESET_VALVE (VALVE_ID12);
		DD_RESET_VALVE (VALVE_ID13);
		DD_RESET_VALVE (VALVE_ID14);
		DD_RESET_VALVE (VALVE_ID15);
		DD_RESET_VALVE (VALVE_ID16);
		DD_RESET_VALVE (VALVE_ID17);
		DD_SET_VALVE (VALVE_ID18);
		DD_RESET_VALVE (VALVE_ID19);
		
		DD_SET_PUMP(DCMOTOR1);
		DD_RESET_PUMP(DCMOTOR2);
		
		break;

				case BC_FLUSH_1_V15:

				//	DD_SET_VALVE(VALVE_ID1);
				//	DD_RESET_VALVE (VALVE_ID2);
			//	DD_RESET_VALVE (VALVE_ID3);
			//	DD_SET_VALVE (VALVE_ID4);
				DD_RESET_VALVE (VALVE_ID5);
				DD_RESET_VALVE (VALVE_ID8);
				DD_RESET_VALVE (VALVE_ID10);
				DD_RESET_VALVE (VALVE_ID11);
				cl_wait(50);
				DD_SET_VALVE (VALVE_ID6);
				DD_SET_VALVE (VALVE_ID7);			
				DD_SET_VALVE (VALVE_ID9);
				DD_SET_VALVE (VALVE_ID12);
				DD_RESET_VALVE (VALVE_ID13);
				DD_RESET_VALVE (VALVE_ID14);

				DD_SET_VALVE (VALVE_ID15);
				DD_RESET_VALVE (VALVE_ID16);
				DD_SET_VALVE (VALVE_ID17);
				DD_SET_VALVE (VALVE_ID18);
			//	DD_RESET_VALVE (VALVE_ID19);
				
			//	DD_SET_PUMP(DCMOTOR1);
			//	DD_SET_PUMP(DCMOTOR2);
				
				
				
				break;
		case BC_FLUSH_2_V15:
		
	//	DD_SET_VALVE(VALVE_ID1);
	//	DD_RESET_VALVE (VALVE_ID2);
	//	DD_RESET_VALVE (VALVE_ID3);
	//	DD_SET_VALVE (VALVE_ID4);
		DD_RESET_VALVE (VALVE_ID6);
		DD_RESET_VALVE (VALVE_ID7);
		DD_RESET_VALVE (VALVE_ID9);
		DD_RESET_VALVE (VALVE_ID12);
		cl_wait(50);
		DD_SET_VALVE (VALVE_ID5);
		DD_SET_VALVE (VALVE_ID8);
		DD_SET_VALVE (VALVE_ID10);
		DD_SET_VALVE (VALVE_ID11);
		
		DD_RESET_VALVE (VALVE_ID13);
		DD_RESET_VALVE (VALVE_ID14);

		DD_SET_VALVE (VALVE_ID15);
		DD_RESET_VALVE (VALVE_ID16);
		DD_SET_VALVE (VALVE_ID17);
		DD_SET_VALVE (VALVE_ID18);
	//	DD_RESET_VALVE (VALVE_ID19);
		
	//	DD_SET_PUMP(DCMOTOR1);
	//	DD_SET_PUMP(DCMOTOR2);
		
		
		break;
		
		case BC_FLUSH_1_V13_14:
		
	//	DD_SET_VALVE(VALVE_ID1);
	//	DD_RESET_VALVE (VALVE_ID2);
	//	DD_RESET_VALVE (VALVE_ID3);
	//	DD_RESET_VALVE (VALVE_ID4);
	DD_RESET_VALVE (VALVE_ID20);
		DD_RESET_VALVE (VALVE_ID5);
		DD_RESET_VALVE (VALVE_ID8);
		DD_RESET_VALVE (VALVE_ID10);
		DD_RESET_VALVE (VALVE_ID11);
		cl_wait(100);		
		DD_SET_VALVE (VALVE_ID6);
		DD_SET_VALVE (VALVE_ID7);
		DD_SET_VALVE (VALVE_ID9);
		DD_SET_VALVE (VALVE_ID12);
	//	DD_SET_VALVE (VALVE_ID13);
	//	DD_SET_VALVE (VALVE_ID14);
		
	//	DD_RESET_VALVE (VALVE_ID16);
		if( ByPass && !isouf)
		{
			DD_SET_VALVE (VALVE_ID15);
			DD_RESET_VALVE (VALVE_ID13);
			DD_RESET_VALVE (VALVE_ID14);
		}
		else if (isouf)
		{
			DD_SET_VALVE (VALVE_ID15);
			DD_RESET_VALVE (VALVE_ID13);
			DD_SET_VALVE (VALVE_ID14);
		}
		else{
			DD_RESET_VALVE (VALVE_ID15);
			DD_SET_VALVE (VALVE_ID13);
			DD_SET_VALVE (VALVE_ID14);
		}
		
		
		if( LoopBack)
		{
			DD_SET_VALVE (VALVE_ID16);	
			DD_RESET_VALVE (VALVE_ID18);
		}
		else
		{
			DD_RESET_VALVE (VALVE_ID16);	
			DD_SET_VALVE (VALVE_ID18);
		}
	//	DD_RESET_VALVE (VALVE_ID16);
	//	DD_SET_VALVE (VALVE_ID17);
	//	DD_SET_VALVE (VALVE_ID18);
	//	DD_RESET_VALVE (VALVE_ID19); // commented for testing
		
	//	DD_SET_PUMP(DCMOTOR1);
	//	DD_SET_PUMP(DCMOTOR2);
		
		break;
		
		case BC_FLUSH_2_V13_14:
		
	//	DD_SET_VALVE(VALVE_ID1);
	//	DD_RESET_VALVE (VALVE_ID2);
	//	DD_RESET_VALVE (VALVE_ID3);
	//	DD_RESET_VALVE (VALVE_ID4);
	DD_RESET_VALVE (VALVE_ID20);
		DD_RESET_VALVE (VALVE_ID6);
		DD_RESET_VALVE (VALVE_ID7);
		DD_RESET_VALVE (VALVE_ID9);
		DD_RESET_VALVE (VALVE_ID12);
		cl_wait(100);				
		DD_SET_VALVE (VALVE_ID5);
		DD_SET_VALVE (VALVE_ID8);	
		DD_SET_VALVE (VALVE_ID10);
		DD_SET_VALVE (VALVE_ID11);
		
	//	DD_SET_VALVE (VALVE_ID13);
	//	DD_SET_VALVE (VALVE_ID14);
	//	DD_SET_VALVE (VALVE_ID15);
	//	DD_RESET_VALVE (VALVE_ID16);
		if( ByPass && !isouf)
		{
			DD_SET_VALVE (VALVE_ID15);
			DD_RESET_VALVE (VALVE_ID13);
			DD_RESET_VALVE (VALVE_ID14);
		}
		else if (isouf)
		{
			DD_SET_VALVE (VALVE_ID15);	
			DD_RESET_VALVE (VALVE_ID13);
			DD_SET_VALVE (VALVE_ID14);
		}
		else{
			DD_RESET_VALVE (VALVE_ID15);
			DD_SET_VALVE (VALVE_ID13);
			DD_SET_VALVE (VALVE_ID14);
		}
		
		if( LoopBack)
		{
			DD_SET_VALVE (VALVE_ID16);
			DD_RESET_VALVE (VALVE_ID18);	
		}
		else
		{
			
			DD_SET_VALVE (VALVE_ID18);
			DD_RESET_VALVE (VALVE_ID16);		
		}
		
	//	DD_RESET_VALVE (VALVE_ID16);
	//	DD_SET_VALVE (VALVE_ID17);
	//	DD_SET_VALVE (VALVE_ID18);
	//	DD_RESET_VALVE (VALVE_ID19); //// commented for testing
		
	//	DD_SET_PUMP(DCMOTOR1);
	//	DD_SET_PUMP(DCMOTOR2);
		
		break;
		
		case  BC_FLUSH_OPEN:
		
	//	DD_SET_VALVE(VALVE_ID1);
	//	DD_RESET_VALVE (VALVE_ID2);
	//	DD_RESET_VALVE (VALVE_ID3);
		DD_RESET_VALVE (VALVE_ID4);
		DD_SET_VALVE (VALVE_ID5);
		DD_SET_VALVE (VALVE_ID6);
		DD_SET_VALVE (VALVE_ID7);
		DD_SET_VALVE (VALVE_ID8);
		DD_SET_VALVE (VALVE_ID9);
		DD_SET_VALVE (VALVE_ID10);
		DD_SET_VALVE (VALVE_ID11);
		DD_SET_VALVE (VALVE_ID12);
		DD_SET_VALVE (VALVE_ID13);
		DD_SET_VALVE (VALVE_ID14);
		DD_SET_VALVE (VALVE_ID15);
		DD_RESET_VALVE (VALVE_ID16);
		DD_SET_VALVE (VALVE_ID17);
		DD_SET_VALVE (VALVE_ID18);
		DD_SET_VALVE (VALVE_ID19);
		DD_SET_VALVE (VALVE_ID20);		
	//	DD_SET_PUMP(DCMOTOR1);
	//	DD_SET_PUMP(DCMOTOR2);
		
		
		break;
	
		case  BC_FLUSH_OPEN_DIS:
		
		//	DD_SET_VALVE(VALVE_ID1);
		//	DD_RESET_VALVE (VALVE_ID2);
		//	DD_RESET_VALVE (VALVE_ID3);
		DD_SET_VALVE (VALVE_ID4);
		DD_SET_VALVE (VALVE_ID5);
		DD_SET_VALVE (VALVE_ID6);
		DD_SET_VALVE (VALVE_ID7);
		DD_SET_VALVE (VALVE_ID8);
		DD_SET_VALVE (VALVE_ID9);
		DD_SET_VALVE (VALVE_ID10);
		DD_SET_VALVE (VALVE_ID11);
		DD_SET_VALVE (VALVE_ID12);
		DD_SET_VALVE (VALVE_ID13);
		DD_SET_VALVE (VALVE_ID14);
		DD_SET_VALVE (VALVE_ID15);
		DD_SET_VALVE (VALVE_ID16);
		DD_SET_VALVE (VALVE_ID17);
		DD_RESET_VALVE (VALVE_ID18);
		DD_SET_VALVE (VALVE_ID19);
		//	DD_SET_PUMP(DCMOTOR1);
		//	DD_SET_PUMP(DCMOTOR2);
		
		
		break;
		
		
	
		case BC_FLUSH_1_V13_14_LOOP:
		
	//	DD_SET_VALVE(VALVE_ID1);
	//	DD_RESET_VALVE (VALVE_ID2);
	//	DD_RESET_VALVE (VALVE_ID3);
	//	DD_RESET_VALVE (VALVE_ID4);
		DD_RESET_VALVE (VALVE_ID5);
		DD_RESET_VALVE (VALVE_ID8);
		DD_RESET_VALVE (VALVE_ID10);
		DD_RESET_VALVE (VALVE_ID11);
		cl_wait(100);		
		DD_SET_VALVE (VALVE_ID6);
		DD_SET_VALVE (VALVE_ID7);
		DD_SET_VALVE (VALVE_ID9);
		DD_SET_VALVE (VALVE_ID12);
	//	DD_SET_VALVE (VALVE_ID13);
	//	DD_SET_VALVE (VALVE_ID14);
		
		DD_SET_VALVE (VALVE_ID16);
		DD_RESET_VALVE (VALVE_ID15);
	
	//	DD_SET_VALVE (VALVE_ID17);
		DD_RESET_VALVE (VALVE_ID18);
	//	DD_RESET_VALVE (VALVE_ID19); // commented for testing
		
	//	DD_SET_PUMP(DCMOTOR1);
	//	DD_SET_PUMP(DCMOTOR2);
		
		break;
		
		case BC_FLUSH_2_V13_14_LOOP:
		
	//	DD_SET_VALVE(VALVE_ID1);
	//	DD_RESET_VALVE (VALVE_ID2);
	//	DD_RESET_VALVE (VALVE_ID3);
	//	DD_RESET_VALVE (VALVE_ID4);
	
		DD_RESET_VALVE (VALVE_ID6);
		DD_RESET_VALVE (VALVE_ID7);
		DD_RESET_VALVE (VALVE_ID9);
		DD_RESET_VALVE (VALVE_ID12);
		cl_wait(100);				
		DD_SET_VALVE (VALVE_ID5);
		DD_SET_VALVE (VALVE_ID8);	
		DD_SET_VALVE (VALVE_ID10);
		DD_SET_VALVE (VALVE_ID11);
		
	//	DD_SET_VALVE (VALVE_ID13);
	//	DD_SET_VALVE (VALVE_ID14);
	//	DD_SET_VALVE (VALVE_ID15);
	//	DD_RESET_VALVE (VALVE_ID16);
		DD_RESET_VALVE (VALVE_ID15);
		DD_SET_VALVE (VALVE_ID16);
	//	DD_SET_VALVE (VALVE_ID17);
		DD_RESET_VALVE (VALVE_ID18);
	//	DD_RESET_VALVE (VALVE_ID19); //// commented for testing
		
	//	DD_SET_PUMP(DCMOTOR1);
	//	DD_SET_PUMP(DCMOTOR2);
		
		break;
		
		
		case FLOW_PATH_ISO_UF:
		

		DD_RESET_VALVE (VALVE_ID5);
		DD_RESET_VALVE (VALVE_ID6);
		DD_RESET_VALVE (VALVE_ID7);
		DD_RESET_VALVE (VALVE_ID8);
		DD_RESET_VALVE (VALVE_ID9);
		DD_RESET_VALVE (VALVE_ID10);
		DD_RESET_VALVE (VALVE_ID11);
		DD_RESET_VALVE (VALVE_ID12);

		if( ByPass)
		{
			DD_SET_VALVE (VALVE_ID15);
			DD_RESET_VALVE (VALVE_ID13);
			DD_RESET_VALVE (VALVE_ID14);
		}
		else
		{
			DD_RESET_VALVE (VALVE_ID15);	
			DD_SET_VALVE (VALVE_ID13);
			DD_SET_VALVE (VALVE_ID14);
		}
		if( LoopBack)
		{
			DD_SET_VALVE (VALVE_ID16);
			DD_RESET_VALVE (VALVE_ID18);
		}
		else
		{
			DD_RESET_VALVE (VALVE_ID16);	
			DD_SET_VALVE (VALVE_ID18);
		}
		break;
	}
	
	return 0;
	
}

//uint8_t DD_LATCH_STROBE_X1_X5(void)
uint8_t DD_LATCH_STROBE_X1_X16(void)
{
	
	DD_RESET_PIO (PA, 13);
	cl_wait(2);
	DD_SET_PIO (PA, 13);
	cl_wait(2);
	DD_RESET_PIO (PA, 13);
	cl_wait(2);
	
	return 0;
}

uint8_t DD_LATCH_STROBE_X17_X20(void)
{
	
	DD_RESET_PIO (PA, 11);
	cl_wait(2);
	DD_SET_PIO (PA, 11);
	cl_wait(2);
	DD_RESET_PIO (PA, 11);
	cl_wait(2);
	
	return 0;
}


