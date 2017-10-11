/*
 * SV_CNTRL.c
 *
 * Created: 1/6/2014 10:35:04 AM
 *  Author: Geekay
 */ 

#include "SV_CNTRL.h"
#include "cl_types.h"
#include "DD_DRIVER_EXPORT_CALLS.h"
#include "cl_consolecontroller.h"
//#define BLOODPUMP_IIC_ADR 0x60
#ifndef BLOODPUMP_IIC_ADR
#define BLOODPUMP_IIC_ADR 0x0E
#endif
//#define BLOODPUMP_IIC_ADR 0x61
void DD_IIC_SET_BLOODPUP(uint8_t iic_address, uint32_t data,uint8_t length);
extern void DD_IIC_READ_BLDPMP( uint8_t iic_address,uint8_t command,uint16_t* p_data);
extern Cl_ReturnCodeType  Cl_SendDatatoconsole(Cl_ConsoleTxCommandtype , uint8_t* ,uint8_t );
extern 	uint32_t ul_cur_ticks, delayticks;//testing
extern volatile uint32_t g_ul_ms_ticks ;
extern float dummy1;
extern bool Loopbackstatus;
extern bool Bypassstatus;
typedef union
{
	uint8_t byte[4] ;
	uint32_t data;
	}bloodpumpcmdType;
uint8_t sv_cntrl_activatepump(sv_pumptype sv_pump_id)
{
	//uint8_t bloodpumpspeed[3];
	bloodpumpcmdType bloodpumpcmd;
	
	bloodpumpcmd.data =0;

	
	
	if(sv_pump_id != BLOODPUMP)
	{
		DD_SET_PUMP(sv_pump_id);
	
	
	} 
	else	
	{
		bloodpumpcmd.byte[0] = 1;
//		DD_IIC_SET_BLOODPUP(BLOODPUMP_IIC_ADR,bloodpumpcmd.data,1);
		DD_CAN_WRITE_HEMO(BP_START_BLOOD_PUMP,0);
	}
	
	return 0;
}

uint8_t sv_cntrl_deactivatepump(sv_pumptype sv_pump_id)
{
	bloodpumpcmdType bloodpumpcmd;
	bloodpumpcmd.data =0;
	//uint8_t bloodpumpspeed[3];
	
	if(sv_pump_id == HEPARINPUMP){
		
		Cl_AlarmActivateAlarms(HP_OVERRUN,false);
		Cl_AlarmActivateAlarms(HP_UNDERRUN,false);
		
	}
	
	if(sv_pump_id != BLOODPUMP)
	{
		DD_RESET_PUMP(sv_pump_id);

	} else
	
	
	{
		bloodpumpcmd.byte[0] = 2;
	//	DD_IIC_SET_BLOODPUP(BLOODPUMP_IIC_ADR,bloodpumpcmd.data,1);
	DD_CAN_WRITE_HEMO(BP_STOP_PUMP,0);
	}
	
	
	return 0;
}
uint8_t sv_cntrl_poweronheater(void)
{
	 DD_SET_HEATER();
	 return 0;
}
uint8_t sv_cntrl_poweroffheater(void)
{
	//Cl_SendDatatoconsole(CON_TX_COMMAND_PRINTTEXT,( uint8_t*)"HEAT_OFF",8);
	DD_RESET_HEATER();
	return 0;
}



uint8_t sv_cntrl_activate_valve(sv_valvetype sv_valve_id)
{
	//if(sv_valve_id == VALVE_ID4)
	//{
	//	DD_RESET_VALVE(sv_valve_id);
	//}
	//else
	{
		DD_SET_VALVE(sv_valve_id);
	}
	
	
	return 0;
}

uint8_t sv_cntrl_deactivate_valve(sv_valvetype sv_valve_id)
{
	
	if(sv_valve_id == VALVE_ID16)
	{
		DD_RESET_HEATER();
	}
	DD_RESET_VALVE(sv_valve_id);
	
	return 0;
}


uint8_t sv_cntrl_setHepa_dir(void)
{
	DD_SET_DIR();
	
	return 0;
}



uint8_t sv_cntrl_resetHepa_dir()
{
	DD_RESET_DIR();
	
	return 0;	
}

uint8_t sv_cntrl_incpumpspeed(sv_pumptype sv_pump_id,uint32_t sv_unit)
{
	uint8_t bloodpumpspeed[3];
	if(sv_pump_id != BLOODPUMP)
	{
	DD_INC_SPEED(sv_pump_id,sv_unit);
	}
	else
	{
		bloodpumpspeed[0] = 6;
	//	DD_IIC_SET_BLOODPUP( BLOODPUMP_IIC_ADR, &bloodpumpspeed ,1);
	DD_CAN_WRITE_HEMO(BP_INC,0);
	}
	
	return 0;
}

uint8_t sv_cntrl_decpumpspeed(sv_pumptype sv_pump_id,uint32_t sv_unit)
{
	uint8_t bloodpumpspeed[3];
	if(sv_pump_id != BLOODPUMP)
	{
	DD_DEC_SPEED(sv_pump_id,sv_unit);
	}
	else
	{
		bloodpumpspeed[0] = 7;
	//	DD_IIC_SET_BLOODPUP( BLOODPUMP_IIC_ADR, &bloodpumpspeed ,1);
	DD_CAN_WRITE_HEMO(BP_DEC,0);
	}
	return 0;
}

uint8_t sv_cntrl_setpumpspeed(sv_pumptype sv_pump_id,uint32_t speed)
{
	bloodpumpcmdType bloodpumpspeed;
	bloodpumpspeed.data =0;
		//bloodpumpspeed.data = speed;
		bloodpumpspeed.byte[0] = 0x04 ;
		bloodpumpspeed.byte[1] = 0xff & speed ;
		bloodpumpspeed.byte[2] = 0xff & speed >> 8 ;
		
		if(sv_pump_id != BLOODPUMP)
		{
				DD_SET_SPEED(sv_pump_id,speed);
		} else
		
		
		{
			

		//	   DD_IIC_SET_BLOODPUP( BLOODPUMP_IIC_ADR,  bloodpumpspeed.data , 0x03);
		DD_CAN_WRITE_HEMO(BP_SPEED_CHANGE, speed);
		}
		
	
	return 0;
}
void sv_spi_blood_read_temp(uint16_t* pdata)
{
//	DD_IIC_READ_BLDPMP( BLOODPUMP_IIC_ADR,8,pdata);
	
}

uint8_t sv_cntrl_incheater(int32_t dty_val)


{
//	dty_val = 2900;
	
	dummy1 = dty_val; 
	
	if(dty_val > 7500)
	{
		dty_val = 7500;
	}
	// dty_val = dty_val * 0.66;
	 
	 if ( dty_val > 0)
	 {
		 DD_CHNG_HTR(dty_val);
	 }
	
	
	return 0;
}


uint8_t sv_cntrl_decheater(int32_t dty_val)
{
		 if ( dty_val > 0)
	 {
		 DD_CHNG_HTR(dty_val);
	 }
	
	
	return 0;
}

uint8_t sv_cntrl_activatevenousclamp(void)
{
	DD_SET_CLAMP();
	
	return 0;
}

uint8_t sv_cntrl_deactivatevenousclamp(void)
{
	DD_RESET_CLAMP();
	
	return 0;
}

uint8_t sv_cntrl_enable_loopback()
{

	DD_ENABLE_LOOPBACK();	
	Loopbackstatus = true;
	return 0;

}


uint8_t sv_cntrl_disable_loopback()
{

	DD_DISABLE_LOOPBACK();
	Loopbackstatus = false;
	return 0;

}


uint8_t sv_cntrl_enable_bypass()
{

	DD_ENABLE_BYPASS();
	//sv_cntrl_deactivatepump(UFPUMP);
	Bypassstatus = true;
	return 0;

}


uint8_t sv_cntrl_disable_bypass()
{

	DD_DISABLE_BYPASS();
	Bypassstatus = false;
	return 0;

}





uint8_t sv_cntrl_setflowpath(sv_flowpathtype sv_flowpath)
{
		delayticks = g_ul_ms_ticks - ul_cur_ticks;
		ul_cur_ticks = g_ul_ms_ticks;
	DD_SET_FLOW_PATH(sv_flowpath);
	
	return 0;

}


uint8_t sv_cntrl_setpumpfreq(sv_pumptype sv_pump_id,uint32_t freq)
{
	DD_SET_PUMPFREQ(sv_pump_id,freq);
	return 0;
}
uint8_t sv_cntrl_setredalarm()
{
	DD_SET_RED();
	return 0;
}
uint8_t sv_cntrl_setyellowalarm()
{
	DD_SET_YELLOW();
	return 0;
}
uint8_t sv_cntrl_setgreenalarm()
{
	DD_SET_GREEN();
	return 0;
}
uint8_t sv_cntrl_buzzer()
{
	DD_SET_BUZZER();
	return 0;
}
uint8_t sv_cntrl_resetredalarm()
{
	DD_RESET_RED();
	return 0;
}
uint8_t sv_cntrl_resetyellowalarm()
{
	DD_RESET_YELLOW();
	return 0;
}
uint8_t sv_cntrl_resetgreenalarm()
{
	DD_RESET_GREEN();
	return 0;
}
uint8_t sv_cntrl_nobuzzer()
{
	DD_RESET_BUZZER();
	return 0;
}
uint8_t sv_cntrl_setoutput(uint32_t p_io, uint32_t peri)
{
	//uint32_t p_io, peri;
	DD_SET_PIO(peri,p_io);
	return 0;
}

uint8_t sv_cntrl_resetoutput(uint32_t p_io, uint32_t peri)
{
	//uint32_t p_io, peri;
	DD_RESET_PIO(peri,p_io);
	
	return 0;
}