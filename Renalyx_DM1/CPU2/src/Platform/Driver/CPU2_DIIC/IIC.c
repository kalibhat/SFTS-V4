/*
 * IIC.c
 *
 * Created: 12/3/2014 12:35:40 PM
 *  Author: wid7
 */ 

// #include "IIC.h"
// #include "CPU2.h"
// #include "Platform/Service/CPU2_SPI/SPI.h"
// //volatile uint32_t count_start, count_stop=0;
// volatile uint16_t count_start=0;
// volatile uint16_t count_stop=0;
// volatile uint8_t start_c[3];
// states state =  HEMO_IDLE;
// readstate read = STATE1;
// 
// uint8_t data[3] = {0,0,0};
// volatile uint8_t data_buff[3] = {0,0,0};
// volatile uint8_t flag,flag_buff,flag_buff1,flag1 = 0;
// volatile uint32_t array_dummy[10]={NULL};
// 	volatile uint32_t var=0;
// 	volatile bool sv_acc_flag = false;
// 	volatile bool sv_end_flag = false;
// 	volatile bool sv_rxrdy_flag = false;
// 	extern speed_data speed_d;
// 	volatile uint8_t count_d=0;
// 	extern volatile max_data spi_data;
// void CPU2_D_INIT_IIC()
// {
// // 	pmc_enable_periph_clk(BOARD_ID_TWI_SLAVE);
// // 	twi_slave_init(BOARD_BASE_TWI_SLAVE, SLAVE_ADDRESS);
// // 	/* Clear receipt buffer */
// // 	twi_read_byte(BOARD_BASE_TWI_SLAVE);
// // 	
// // 	twi_enable_interrupt(BOARD_BASE_TWI_SLAVE, TWI_SR_SVACC);
// }
// uint32_t i=0;
// void BOARD_TWI_Handler(void)
// {
// 	printf("interrupt\n");
// 		uint32_t status;
// 		status = twi_get_interrupt_status(BOARD_BASE_TWI_SLAVE);
// 		//var=status;
// 		/*array_dummy[var]=status;
// 		var++;
// 		if (var==10)
// 		{
// 			for (i=0;i<10;i++)
// 			{
// 				printf("%d\n",array_dummy[i]);
// 			}
// 			var=0;
// 		}*/
// 		if ((status & TWI_SR_SVACC) == TWI_SR_SVACC)
// 		{
// 			sv_acc_flag = true;
// 			sv_end_flag = false;
// 			twi_disable_interrupt(BOARD_BASE_TWI_SLAVE, TWI_IDR_SVACC);
// 			
// 			twi_enable_interrupt(BOARD_BASE_TWI_SLAVE, TWI_IER_RXRDY | TWI_IER_GACC
// 			| TWI_IER_NACK | TWI_IER_EOSACC | TWI_IER_SCL_WS| TWI_SR_TXRDY);
// 		}
// 	
// 		if ((status & TWI_SR_GACC) == TWI_SR_GACC) {
// 			puts("General Call Treatment\n\r");
// 			puts("not treated");
// 		}
// 		if (((status & TWI_SR_SVACC) == TWI_SR_SVACC) && ((status & TWI_SR_GACC) == 0)&& ((status & TWI_SR_RXRDY ) == TWI_SR_RXRDY))
// 		{
// 				sv_rxrdy_flag = true;
// 			switch (state)
// 			{
// 				case IDLE:
// 				
// 				data[data_count] = twi_read_byte(BOARD_BASE_TWI_SLAVE);
// 				data_buff[data_count] = data[data_count];
// 				
// 				start_c[0] =data_buff[0];
// 				switch (data_buff[0])
// 				{
// 					case START_PUMP:
// 					//printf("%d\n",var);
// 					//printf("s\n");
// 					//flag = 1;
// 					//count_start++;
// 					flag_buff = 1;
// 					state = IDLE;
// 					break;
// 					case STOP_PUMP:
// 					//printf("%d\n",var);
// 					//printf("st\n");
// 					//flag = 1;
// 					//count_stop++;
// 					flag_buff = 1;
// 					state = IDLE;
// 					break;
// 					case COMPLETED:
// 					//flag = 1;
// 					flag_buff = 1;
// 					state = IDLE;
// 					break;
// 					case BP_INC:
// 					//flag = 1;
// 					flag_buff = 1;
// 					state = IDLE;
// 					break;
// 					case BP_DEC:
// 					//flag = 1;
// 					flag_buff = 1;
// 					state = IDLE;
// 					break;
// 					case RECOVER:
// 					//flag = 1;
// 					flag_buff = 1;
// 					state = IDLE;
// 					break;
// 					case SPEED_CHANGE:
// 					//printf("speed\n");
// 					state = WAIT;
// 					flag = 0;
// 					flag_buff = 0;
// 					break;
// 					case TEMP_READ:
// 					//printf("speed\n");
// 					state = IDLE;
// 					flag = 0;
// 					flag_buff = 0;
// 					break;
// 					case DAC_REF:
// 					state = WAIT;
// 					flag = 0;
// 					flag_buff = 0;
// 					break;
// 					default:
// 					break;
// 				}
// 				break;
// 				
// 				case WAIT:
// 				
// 				if (data_count <= 2)
// 				{
// 					data[data_count] = twi_read_byte(BOARD_BASE_TWI_SLAVE);
// 					data_buff[data_count] = data[data_count];
// 					start_c[data_count]= data_buff[data_count];
// 					state = WAIT;
// 					flag = 0;
// 					flag_buff = 0;
// 					
// 					if (data_count == 2 )
// 					{
// 					//printf("%d \t %d \t %d\n",data_buff[0],data_buff[1],data_buff[2]);
// 					data_count1=data_count;
// 						flag = 1;
// 						flag_buff = 1;
// 						state = IDLE;	
// 					}
// 				}
// 				break;
// 			}
// 			data_count++;
// 			
// 		}
// 		
// 		else
// 		{
// 			if (((status & TWI_SR_TXRDY) == TWI_SR_TXRDY)
// 			&& ((status & TWI_SR_TXCOMP) == TWI_SR_TXCOMP)
// 			&& ((status & TWI_SR_EOSACC) == TWI_SR_EOSACC))
// 			{
// 				/* End of transfer, end of slave access */
// 				sv_acc_flag = false;
// 				sv_rxrdy_flag = false;
// 				sv_end_flag = true;
// 				data_count = 0;
// 				data[0]= (uint8_t)0;
// 				data[1]= (uint8_t)0;
// 				data[2]= (uint8_t)0;
// 				twi_enable_interrupt(BOARD_BASE_TWI_SLAVE, TWI_SR_SVACC);
// 				twi_disable_interrupt(BOARD_BASE_TWI_SLAVE, TWI_IDR_RXRDY | TWI_IDR_GACC |
// 				TWI_IDR_NACK | TWI_IDR_EOSACC | TWI_IDR_SCL_WS |TWI_SR_TXRDY);
// 			}
// 			else {
// 				if (((status & TWI_SR_SVACC) == TWI_SR_SVACC)&& ((status & TWI_SR_GACC) == 0)&& ((status & TWI_SR_SVREAD) == TWI_SR_SVREAD)
// 				&& ((status & TWI_SR_NACK) == 0))
// 				{
// 					switch (read)
// 					{
// 						case STATE1:
// 						if ((start_c[0] != SPEED_CHANGE) & (start_c[0] != TEMP_READ))
// 						{
// 							twi_write_byte(BOARD_BASE_TWI_SLAVE,start_c[0]);
// 							read = STATE1;
// 						}
// 						else if ((start_c[0] == TEMP_READ))
// 						{
// 							if ((status & TWI_SR_TXRDY))
// 							{
// 								twi_write_byte(BOARD_BASE_TWI_SLAVE,spi_data.bytearray[0]);
// 								read = STATE4;
// 							}						
// 						}
// 	
// 						else 
// 						{
// 							if ((status & TWI_SR_TXRDY)) 
// 							{
// 								twi_write_byte(BOARD_BASE_TWI_SLAVE,start_c[0]);
// 								read = STATE2;
// 							}	
// 						}
// 						break;
// 						case STATE2:
// 								if ((status & TWI_SR_TXRDY)) 
// 								{
// 								twi_write_byte(BOARD_BASE_TWI_SLAVE,start_c[1]);
// 								read = STATE3;
// 								}	
// 						break;	
// 						case STATE3:
// 							if ((status & TWI_SR_TXRDY))
// 							{
// 								twi_write_byte(BOARD_BASE_TWI_SLAVE,start_c[2]);
// 								read = STATE1;
// 								start_c[0]=0;
// 								start_c[1]=0;
// 								start_c[2]=0;
// 							}
// 						break;
// 						case STATE4:
// 						if ((status & TWI_SR_TXRDY))
// 						{
// 							twi_write_byte(BOARD_BASE_TWI_SLAVE,spi_data.bytearray[1]);
// 							if (spi_data.bytearray[1] != 70)
// 							{
// 								printf("AAAA\n");
// 							}
// 							start_c[0]=0;
// 							read = STATE1;
// 						}
// 						break;
// 	
// 						
// 						default:break;
// 	
// 					}
// 				}
// 			}
// 		}
//}


