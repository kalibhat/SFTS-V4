/*
 * SV_PROP.c
 *
 * Created: 3/19/2015 3:25:33 PM
 *  Author: user
 */ 
#include "asf.h"
#include "cl_types.h"
#include "DD_DRIVER_EXPORT_CALLS.h"
#include "cl_consolecontroller.h"

#ifndef CPU_PROP_IIC_ADDR
#define  CPU_PROP_IIC_ADDR 0x70
#endif
//#define  CPU_PROP_IIC_ADDR 0x70
extern void DD_IIC_SEND_PROP(uint8_t , uint8_t* );
extern void DD_IIC_CONSOLE_SEND(uint8_t iic_address, uint8_t* data,uint8_t length);
extern Cl_ReturnCodeType cl_wait(uint32_t ul_dly_ticks);
extern void DD_IIC_READ_PROP(uint8_t iic_address, uint16_t* data);
extern Cl_ReturnCodeType  Cl_SendDatatoconsole(Cl_ConsoleTxCommandtype , uint8_t* ,uint8_t );
void sv_prop_startmixing(void);
void sv_prop_stopmixing(void);
void sv_prop_startopenfill(void);
void sv_prop_stopopenfill(void);
void sv_prop_start_disinfect_intake(void);
void sv_prop_stop_disinfect_intake(void);
void sv_prop_start_citricdisinfect_intake(void);
void sv_prop_stop_citricdisinfect_intake(void);
void sv_prop_set_propo_ACIDrc(uint16_t data);
void sv_prop_set_propo_BICARBrc(uint16_t data);
void sv_prop_propo_CONDUCTIVITY(uint16_t data);
static uint8_t data = 0;
uint16_t count = 0;

extern Cl_Uint32Type Treatdata[ID_MAX_TREAT_PARAM] ;

void sv_prop_startpropeo_aligning(void)
{
uint8_t dataarray[16] = { 0xaa,0xcc,0xdd,1,2,3,4,5,6,7,8,9,0xA,0xB,0xee,0xbb};
	//	cl_wait(20);
		//DD_IIC_SEND_PROP( CPU_PROP_IIC_ADDR, 0);
		//DD_IIC_CONSOLE_SEND(0x77, &dataarray ,16);
		 SV_CAN_WRITE_PROPO( PROP_START_ALIGMENT,0);

}
void sv_prop_startmixing()
{

	{
	//	cl_wait(20);
	//	DD_IIC_SEND_PROP( CPU_PROP_IIC_ADDR, 1);
	//	DD_IIC_SEND_PROP( CPU_PROP_IIC_ADDR, 0);
	
		SV_CAN_WRITE_PROPO( PROP_START_MIXING,Treatdata[ID_setcond]);
	
	}
	
}

void sv_prop_bicarb_selftest()   //for selftest
{
	SV_CAN_WRITE_PROPO( prop_BICAR_FEED_BACK,0);
}

void sv_prop_bicarb_selftest_f_b()
{
	SV_CAN_WRITE_PROPO( prop_BICAR_FEED_BACK_STATUS,0);
}




void sv_prop_startacidpump()
{
	SV_CAN_WRITE_PROPO( PROP_START_ACID_PUMP,0);
			
}

void sv_prop_startbicarbpump()
{
	SV_CAN_WRITE_PROPO( PROP_START_BICARB_PUMP,0);
	
}


void sv_prop_set_propo_ACIDrc(uint16_t data)
{
	
	uint16_t rc_count = data;
	SV_CAN_WRITE_PROPO( PROP_UPDATE_ACID_RC,data);                 // Gopal: Send Acid RC (speed) to Propo CPU via Can
}
void sv_prop_set_propo_BICARBrc(uint16_t data)
{
	
	uint16_t rc_count = data;
	SV_CAN_WRITE_PROPO( PROP_UPDATE_BICARB_RC,data);                // Gopal: Send Bicarb RC (speed) to Propo CPU via Can
}

void sv_prop_propo_CONDUCTIVITY(uint16_t data)
{
	uint16_t conductivity = data;
	SV_CAN_WRITE_PROPO( PROP_CONDUCTIVITY,conductivity);      // Gopal: Send  conducivity value to Propo CPU via Can
}
void sv_prop_stopmixing()
{
	//DD_IIC_SEND_PROP( CPU_PROP_IIC_ADDR, 2);
	SV_CAN_WRITE_PROPO( PROP_STOP_MIXING,0);
}


void sv_prop_start_disinfect_intake(void)
{


	{
	//	cl_wait(20);
	//	DD_IIC_SEND_PROP( CPU_PROP_IIC_ADDR, 3);
		SV_CAN_WRITE_PROPO( PROP_START_DISINFECTION,0);

	}
	
}
void sv_prop_stop_disinfect_intake(void)
{


	{
	//	cl_wait(20);
	//	DD_IIC_SEND_PROP( CPU_PROP_IIC_ADDR, 4);
		SV_CAN_WRITE_PROPO( PROP_STOP_DISINFECTION,0);

	}
	
}
void sv_prop_start_citricdisinfect_intake(void)
{


	{
	//	cl_wait(20);
		//DD_IIC_SEND_PROP( CPU_PROP_IIC_ADDR, 8);
		SV_CAN_WRITE_PROPO( PROP_START_CITRIC_DISINFECTION,0);


	}
	
}
void sv_prop_stop_citricdisinfect_intake(void)
{


	{
	//	cl_wait(20);
		//DD_IIC_SEND_PROP( CPU_PROP_IIC_ADDR, 9);
		SV_CAN_WRITE_PROPO( PROP_STOP_CITRIC_DISINFECTION,0);


	}
	
}
void sv_prop_startopenfill()
{
	Cl_SendDatatoconsole(CON_TX_COMMAND_PRINTTEXT,"OPENFIL",6);
//	DD_IIC_SEND_PROP( CPU_PROP_IIC_ADDR, 5);
	SV_CAN_WRITE_PROPO( PROP_START_OPENFILL,0);
}
void sv_prop_stopopenfill()
{
	Cl_SendDatatoconsole(CON_TX_COMMAND_PRINTTEXT,"STOPFIL",6);
	//DD_IIC_SEND_PROP( CPU_PROP_IIC_ADDR, 6);
	SV_CAN_WRITE_PROPO( PROP_STOP_OPENFILL,0);
}

