/*
 * cl_utilities.c
 *
 * Created: 1/15/2014 10:09:34 AM
 *  Author: user
 */ 
#include "cl_types.h"

Cl_ReturnCodeType cl_systemtickcallback(void);
Cl_ReturnCodeType cl_wait(Cl_Uint32Type ul_dly_ticks);
Cl_ReturnCodeType Cl_MilliTickcallBack(void);
Cl_ReturnCodeType cl_memset(Cl_Uint8Type* , Cl_Uint8Type );

volatile Cl_Uint32Type g_ul_ms_ticks = 0;

static void mdelay(Cl_Uint32Type ul_dly_ticks)
{
	Cl_Uint32Type ul_cur_ticks;

	ul_cur_ticks = g_ul_ms_ticks;
	while ((g_ul_ms_ticks - ul_cur_ticks) < ul_dly_ticks);
}

Cl_ReturnCodeType cl_wait(Cl_Uint32Type ul_dly_ticks)
{
	mdelay(ul_dly_ticks);
	return CL_OK;
}

Cl_ReturnCodeType cl_systemtickcallback(void)
{
	Cl_ReturnCodeType Cl_retVal;
	Cl_retVal = Cl_MilliTickcallBack();
	g_ul_ms_ticks++;
	return  CL_OK;
}


Cl_ReturnCodeType cl_memset(Cl_Uint8Type* pdata, Cl_Uint8Type datasize)
{
	Cl_Uint8Type tempcnt = 0 ;
	Cl_Uint8Type* pdata1;
	
	pdata1 = (Cl_Uint8Type*)pdata;
	
	for (tempcnt = 0 ; tempcnt < datasize ; tempcnt++)
	{
		pdata[tempcnt] = 0;
	//	*pdata = 1;
	//	pdata++;
	//	*pdata1 = (Cl_Uint8Type*)0;
	//	pdata1 =  pdata1 +1;
	}
	return CL_OK;
}

/*float cl_utilities_CalConductivity(uint16_t raw_cond,float temperature)    // Gopal Machine utelities fn
{
			float conductivity, conductance;
			conductance = raw_cond;// /1000; 
			conductivity = conductance;
	//		conductivity = conductance * 1.272;                             // Conductance * cell constant = conductivity
	//		conductivity = conductance * 0.1703908;
	//		conductivity = (0.0001*conductivity*conductivity + 0.032*conductivity) * 100;                          // Gopal: Need to check for offset if required based on the observations in IBP 02082017
 			conductivity = (conductivity/(1+(temperature  -25.0)*0.021));                    // conductivity compensated at 25 degrees using win-situ formula
           conductivity = (conductivity / 7.600831379);

//			return conductivity;
			return conductivity; ///10;
}*/

float cl_utilities_CalConductivity(uint16_t raw_cond,float temperature)
{
			float conductivity, conductance;
			conductance = raw_cond;// /1000; 
//			conductivity = (raw_cond -66.67)/100;                                 // raw conductance contains conductance of fixed 150 ohm resistance
	//		cond_final = 0.0001*cond1*cond1 + 0.032*cond1 +0.91 + 0.4;
//			conductivity = conductance * 1.272;                             // Conductance * cell constant = conductivity
			conductivity = conductance * 0.004544786 *100;
	//		conductivity = (0.0001*conductivity*conductivity + 0.032*conductivity) * 100;                          // Gopal: Need to check for offset if required based on the observations in IBP 02082017
// 			conductivity = (conductivity/(1+(temperature  -25.0)*0.021));                    // conductivity compensated at 25 degrees using win-situ formula

			return conductivity; ///10;
}