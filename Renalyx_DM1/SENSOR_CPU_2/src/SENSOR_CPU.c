/**
 * \file
 *
 * \brief Empty user application template
 *
 */

/**
 * \mainpage User Application template doxygen documentation
 *
 * \par Empty user application template
 *
 * Bare minimum empty user application template
 *
 * \par Content
 *
 * -# Include the ASF header files (through asf.h)
 * -# "Insert system clock initialization code here" comment
 * -# Minimal main function that starts with a call to board_init()
 * -# "Insert application code here" comment
 *
 */

/*
 * Include header files for all drivers that have been imported from
 * Atmel Software Framework (ASF).
 */
/*
 * Support and FAQ: visit <a href="http://www.atmel.com/design-support/">Atmel Support</a>
 */
#include <asf.h>

#include "asf.h"
#include "SENSOR_CPU.h"


	
//extern SV_Sensor_status_type  ;
extern volatile bool master_requested_sensor_data;
extern void SV_SYSTEM_INIT(void);
extern void SV_get_sensor_data(SV_Sensor_status_type* sensor_struct);
extern bool SV_put_sensor_data(SV_Sensor_status_type* sensor_struct);

SV_Sensor_status_type volatile sensor_data, g_sensor_data;


int main (void){
	/* Insert system clock initialization code here (sysclk_init()). */
	
   
	board_init();
	SV_SYSTEM_INIT(); 
	/* Insert application code here, after the board has been initialized. */
	uint32_t count = 0;
	while(FOREVER) {	
	for(int i=0;i<10;i++) {}
		SV_get_sensor_data(&sensor_data);
// 		g_sensor_data.Temp2status = (g_sensor_data.Temp2status *9 + sensor_data.Temp2status)/10;
// 		g_sensor_data.CS3_Tempstatus = (g_sensor_data.CS3_Tempstatus *9 + sensor_data.CS3_Tempstatus)/10;
// 		g_sensor_data.CS3status = (g_sensor_data.CS3status *9 + sensor_data.CS3status)/10;
// 		sensor_data.CS3_Tempstatus = g_sensor_data.CS3_Tempstatus;
// 		sensor_data.CS3status = g_sensor_data.CS3status;
// 		sensor_data.Temp2status = g_sensor_data.Temp2status ;

//		g_sensor_data.CS3status = (g_sensor_data.CS3status *9 + sensor_data.CS3status)/10;         // Gopal: this and next line should be uncommented for new sensor hardware as DAC1 is CS3
//		sensor_data.CS3status = g_sensor_data.CS3status;
		//g_sensor_data.DAC1status = (g_sensor_data.DAC1status *29 + sensor_data.DAC1status)/30;     // Gopal: this and next line shd be commented for new sensor hardware as CS3 is final conductivity
		//sensor_data.CS3status = sensor_data.DAC1status;
		
		
		//sensor_data.CS1status = sensor_data.DAAstatus;
		
// 		g_sensor_data.CS1_Tempstatus = (sensor_data.CS1_Tempstatus);
// 		g_sensor_data.CS2_Tempstatus = (sensor_data.CS2_Tempstatus);
// 		g_sensor_data.CS3_Tempstatus = (sensor_data.CS3_Tempstatus);
		
	
		
		if(master_requested_sensor_data==1) 	
		{			
				

  			 SV_put_sensor_data(&sensor_data);			
			master_requested_sensor_data=0;
		}	
	   
	}
}
