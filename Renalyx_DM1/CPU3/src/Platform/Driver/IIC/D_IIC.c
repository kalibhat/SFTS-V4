/*
 * IIC.c
 *
 * Created: 12/9/2014 2:23:01 PM
 *  Author: wid7
 */ 
// #include "D_IIC.h"
// #include "CPU3.h"
// 
// states state = IDLE;
// readstate read = STATE1;
// 
// volatile uint16_t count_start=0;
// volatile uint8_t start_c[2];
// volatile uint16_t count_stop=0;
// volatile uint8_t data_count =0;
// volatile uint8_t data[3] = {0,0,0};
// volatile bool sv_acc_flag = false;
// volatile bool sv_end_flag = false;
// volatile bool sv_rxrdy_flag = false;
// volatile uint8_t data_buffer[1000];
// volatile uint8_t data_buff[3] = {0,0,0};
// volatile uint32_t address,address_1;
// volatile static bool bool_s = false;
// volatile uint8_t flag,flag_buff,flag_buff1,flag1,flag_buff_dummy = 0;
// volatile uint8_t test=0;
// uint8_t test_data = 170;
// 
// void CPU3_D_INIT_IIC()
// {
// 	pmc_enable_periph_clk(BOARD_ID_TWI_SLAVE);
// 	twi_slave_init(BOARD_BASE_TWI_SLAVE, SLAVE_ADDRESS);
// 	/* Clear receipt buffer */
// 	twi_read_byte(BOARD_BASE_TWI_SLAVE);
// 	
// 	twi_enable_interrupt(BOARD_BASE_TWI_SLAVE, TWI_SR_SVACC);
// }
// volatile uint16_t status_array[100];
// volatile int counter=0;
// void BOARD_TWI_Handler(void)
// {
// 	uint32_t status;
// 	status=twi_get_interrupt_status(BOARD_BASE_TWI_SLAVE);
// 	
// 	if ((status & TWI_SR_SVACC) == TWI_SR_SVACC)
// 	{
// 		sv_acc_flag = true;
// 		sv_end_flag = false;
// 		twi_disable_interrupt(BOARD_BASE_TWI_SLAVE, TWI_IDR_SVACC);
// 		
// 		twi_enable_interrupt(BOARD_BASE_TWI_SLAVE, TWI_IER_RXRDY | TWI_IER_GACC
// 		| TWI_IER_NACK | TWI_IER_EOSACC | TWI_IER_SCL_WS);
// 		
// 		
// 	}
// 
// 	if ((status & TWI_SR_GACC) == TWI_SR_GACC) {
// 		puts("General Call Treatment\n\r");
// 		puts("not treated");
// 	}
// 	if (((status & TWI_SR_SVACC) == TWI_SR_SVACC) && ((status & TWI_SR_GACC) == 0)&& ((status & TWI_SR_RXRDY ) == TWI_SR_RXRDY))
// 	{
// 		sv_rxrdy_flag = true;
// 		switch (state)
// 		{
// 			case IDLE:
// 				
// 					data[data_count] = twi_read_byte(BOARD_BASE_TWI_SLAVE);
// 					data_buff[data_count] = data[data_count];
// 					
// 					data_buffer[data_count]=data_buff[data_count];
// 					
// 			cntrl_name diff;
// 			diff = data_buff[0];
// 			start_c[0] = diff;
// 			switch (diff)
// 			{
// 				case START_ALIGMENT:
// 				
// 					test = 0;
// 					count_start++;
// 					//flag = 1;
// 					flag_buff=1;
// 					state = IDLE;
// 				break;
// 				case START_MIXING:
// 				//printf("got start\n");
// 				test = 0;
// 				count_start++;
// 				//flag = 1;
// 				flag_buff=1;
// 				state = IDLE;
// 				break;
// 				case STOP_MIXING:
// 				test = 0;
// 				//printf("got stop\n");
// 				count_stop++;
// 				//flag = 1;
// 				flag_buff = 1;
// 				state = IDLE;
// 				break;
// 				case RECOVER:
// 				//flag = 1;
// 				flag_buff = 1;
// 				state = IDLE;
// 				break;
// 				case COMPLETED:
// 				flag_buff = 1;
// 				state = IDLE;
// 				break;
// 				case START_OPENFILL:
// 				//flag = 1;
// 				flag_buff = 1;
// 				state = IDLE;
// 				break;
// 				case STOP_OPENFILL:
// 				//flag = 1;
// 				flag_buff = 1;
// 				state = IDLE;
// 				break;
// 				default:
// 				break;
// 				/*case SPEED_CHANGE:
// 				//printf("speed\n");
// 				state = WAIT;
// 				flag = 0;
// 				flag_buff = 0;
// 				break;
// 				case DAC_REF:
// 				state = WAIT;
// 				flag = 0;
// 				flag_buff = 0;
// 				break;*/
// 			}
// 			break;
// 			
// 			case WAIT:
// 			
// 			if (data_count <= 2)
// 			{
// 				
// 				data[data_count] = twi_read_byte(BOARD_BASE_TWI_SLAVE);
// 				data_buff[data_count] = data[data_count];
// 				state = WAIT;
// 				flag = 0;
// 				flag_buff = 0;
// 				
// 				if (data_count == 2 )
// 				{
// 					//printf("%d \t %d \t %d\n",data_buff[0],data_buff[1],data_buff[2]);
// 					flag = 1;
// 					flag_buff = 1;
// 					state = IDLE;
// 					
// 				}
// 			}
// 			break;
// 		}
// 		
// 		data_count++;
// 		
// 	}
// 	
// 	else 
// 	{
// 		if (((status & TWI_SR_TXRDY) == TWI_SR_TXRDY)
// 		&& ((status & TWI_SR_TXCOMP) == TWI_SR_TXCOMP)
// 		&& ((status & TWI_SR_EOSACC) == TWI_SR_EOSACC))
// 		{
// 			sv_acc_flag = false;
// 			sv_rxrdy_flag = false;
// 			sv_end_flag = true;
// 			data_count = 0;
// 			data[0]= (uint8_t)0;
// 			data[1]= (uint8_t)0;
// 			data[2]= (uint8_t)0;
// 			
// 			twi_enable_interrupt(BOARD_BASE_TWI_SLAVE, TWI_SR_SVACC);
// 			twi_disable_interrupt(BOARD_BASE_TWI_SLAVE, TWI_IDR_RXRDY | TWI_IDR_GACC |
// 			TWI_IDR_NACK | TWI_IDR_EOSACC | TWI_IDR_SCL_WS);
// 		}
// 		else {
// 			if (((status & TWI_SR_SVACC) == TWI_SR_SVACC)&& ((status & TWI_SR_GACC) == 0)&& ((status & TWI_SR_SVREAD) == TWI_SR_SVREAD)
// 			&& ((status & TWI_SR_NACK) == 0)) 
// 			{
// 				switch (read)
// 				{
// 					case STATE1:
// 						twi_write_byte(BOARD_BASE_TWI_SLAVE,start_c[0]);
// 						start_c[0] = 255;
// 						read = STATE1;
// 					break;
// 					/*case STATE2:
// 						if ((status & TWI_SR_TXRDY) == TWI_SR_TXRDY)
// 						{
// 							twi_write_byte(BOARD_BASE_TWI_SLAVE,start_c[1]);
// 							read = STATE1;
// 						}
// 					break;*/
// 				}
// 				//printf("reading\n");
// 				/* Write one byte of data from slave to master device */
// 				//printf("count_start:%d\n",count_start);
// 				//twi_write_byte(BOARD_BASE_TWI_SLAVE,count_start);
// 			}
// 			}	
// 	}
// }
