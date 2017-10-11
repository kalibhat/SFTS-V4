/*
 * SV_SYS.c
 *
 * Created: 1/6/2014 10:32:46 AM
 *  Author: Geekay
 */

#include "SV_SYS.h"
#include "Platform/Service/SV_TIMER/SV_TIMER.h"
#include "cl_types.h"
extern void DD_INIT_IIC(void);
extern Cl_Uint8Type sum[11];
extern Cl_Uint8Type checksum_all[11];

void SV_SYSTEM_INIT()
{
	//uint32_t prd_val=1000,dty_val = 100;
	DD_INIT_CLOCK();
	DD_INIT_PIO();
	DD_INIT_UART();
	DD_INIT_TIMER();
//	DD_INIT_ADC();
	DD_INIT_PWM();
	DD_INIT_NVM();
	DD_INIT_INT();
	DD_INIT_IIC();
	DD_INIT_SPI();

    DD_INIT_CAN();
//     nvram_read(CAL_PS1);
// 	nvram_read(CAL_APT);
// 	nvram_read(CAL_COND);
// 	nvram_read(CAL_HEP);
// 	nvram_read(CAL_TEMP_TS1);
// 	nvram_read(CAL_CHECKSUM);
// 	for (int i=0;i<10;i++)
// 	{
// 		if (checksum_all[i] != sum[i])
// 		{
// 			//printf("error\n");
// 		}
// 	}
	
	SV_START_TIMER(0,1);
	//printf("1");
	//DD_SET_PRDY(CH_0,prd_val);
	//DD_SET_DUTY(CH_1,dty_val);
}