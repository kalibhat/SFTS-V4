/*
 * SV_CS.c
 *
 * Created: 9/19/2014 4:11:18 PM
 *  Author: user
 */ 
#include "asf.h"
#include "SV_CS.h"

#define  CS_POT_IIC_ADDR 0x2c
#define  COND_POT_IIC_ADDR 0x2f
AD5174_COMMAND_TYPE;
uint16_t command_data_array[3];


extern void DD_READ_IIC(uint8_t iic_address, uint16_t* data);
extern void DD_WRITE_IIC(uint8_t iic_address, uint16_t data);

void DD_IIC_SET_POT(uint8_t iic_address, uint16_t* data);



void DD_IIC_SEND_PROP(uint8_t iic_address, uint8_t* data);


uint8_t sv_cs_readpotvalue(uint16_t* resistance);
uint8_t sv_cs_setpotvalue(uint16_t resistance) ;
uint8_t sv_cs_setcondpotvalue(uint16_t resistance);

uint8_t sv_cs_setpotvalue(uint16_t resistance) 
{
	 uint8_t iic_address;
	 uint16_t data;
	//	DD_IIC_SET_POT( CS_POT_IIC_ADDR, resistance);
		
		SV_CS_SET_POT(resistance);
		
		}
uint8_t sv_cs_setcondpotvalue(uint16_t resistance)
{
	uint8_t iic_address;
	uint16_t data;

	
	DD_IIC_SET_POT( COND_POT_IIC_ADDR, resistance);
	
}

uint8_t sv_cs_readpotvalue(uint16_t* resistance)
{
		uint8_t iic_address;
		uint8_t data;
		//DD_SET_POT( CS_POT_IIC_ADDR, &data);
		*resistance = data;
	
}

void SV_CS_SET_POT(uint16_t* resistance)
{
	uint16_t data = resistance;
	data = data & 0x3ff;
	command_data_array[0] = AD5174_COMMAND(AD5174_WRITE_SERIAL_REG_TO_CONTRL_REG) | 0x02; //tempreg.temp_8bit[1] ;
	command_data_array[1]  = AD5174_COMMAND(AD5174_WRITE_SERIALREG_TO_RDAC) | data;
//	command_data_array[1]  = AD5174_COMMAND(AD5174_WRITE_SERIALREG_TO_RDAC)|(0x3ff & *resistance);
	//command_data_array[1]  = 0x04c2;
	
	DD_SPI_SET_POT(&command_data_array, 2);
}