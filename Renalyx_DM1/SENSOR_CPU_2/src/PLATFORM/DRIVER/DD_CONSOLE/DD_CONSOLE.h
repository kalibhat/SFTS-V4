/*
 * DD_CONSOLE.h
 *
 * Created: 3/6/2017 5:52:36 PM
 *  Author: Electrical
 */ 


#ifndef DD_CONSOLE_H_
#define DD_CONSOLE_H_

#ifndef DD_UART_H_
#define DD_UART_H_

#include "asf.h"

#define CONF_UART            CONSOLE_UART
/** Baudrate setting */
#define CONF_UART_BAUDRATE   115200
/** Parity setting */
#define CONF_UART_PARITY     UART_MR_PAR_NO
#define STRING_EOL    "\r"


#define STRING_HEADER "-- CAN Example --\r\n" \
                      "-- "BOARD_NAME" --\r\n" \
                      "-- Compiled: "__DATE__" "__TIME__" --"STRING_EOL


void SENSOR_D_INIT_UART(void);

uint32_t DD_TX_UART(uint8_t sv_consoleTxdatabyte);

uint8_t DD_RX_UART(void);
#endif /* DD_UART_H_ */




#endif /* DD_CONSOLE_H_ */