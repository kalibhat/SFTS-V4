/*
 * DD_GPIO.h
 *
 * Created: 3/7/2017 11:32:04 AM
 *  Author: Electrical
 */ 


#ifndef DD_GPIO_H_
#define DD_GPIO_H_

#include "asf.h"
/* GPIO Support */
#define PIO_TYPE_Pos                  27
/* The pin is controlled by the peripheral A. */
#define PIO_TYPE_PIO_A         (0x1u << PIO_TYPE_Pos)
/* The pin is controlled by the peripheral B. */
#define PIO_TYPE_PIO_B         (0x2u << PIO_TYPE_Pos)
/* The pin is controlled by the peripheral C. */
#define PIO_TYPE_PIO_C         (0x3u << PIO_TYPE_Pos)
/* The pin is controlled by the peripheral D. */
#define PIO_TYPE_PIO_D         (0x4u << PIO_TYPE_Pos)

typedef enum _pio
{
	#if (SAM3XA)
	PA    = PIO_TYPE_PIO_A ,
	PB    = PIO_TYPE_PIO_B ,
	PC    = PIO_TYPE_PIO_C ,
	PD    = PIO_TYPE_PIO_D ,
	#endif
} pio_type;

#define SPI0_MOSI_PIN 26 // PORTA
#define SPI0_MISO_PIN 25 // PORTA
#define SPI0_SCLK_PIN 27 // PORTA
#define SPI0_NPCS0_PIN 28 // PORTA
#define SPI0_NPCS1_PIN 29 // PORTA

#define SPI0_NPCS2_PIN 21 // PORTB
#define SPI0_NPCS3_PIN 23 //PORTB

#define SPI0_MISO_ISOLATER_1_DISABLE_PIN 6 // PORTA
#define SPI0_MISO_ISOLATER_2_DISABLE_PIN 5 //PORTA
#define SPI0_MISO_ISOLATER_3_DISABLE_PIN 4 // PORTA
#define SPI0_MISO_ISOLATER_4_DISABLE_PIN 3 //PORTA



#define CAN_TX_PIN 0  // PORTA
#define CAN_RX_PIN 1  // PORTA
#define CAN_STB_PIN 2 // PORTA

#define UART_RX_PIN 8 // PORTA
#define UART_TX_PIN 9 // PORTA

#define ISOLTR_1_SSA0_PIN 3 // PORTB
#define ISOLTR_1_SSA1_PIN 4 // PORTB

// #define ISOLTR_1_SSA0_PIN 14 // PORTA
//  #define ISOLTR_1_SSA1_PIN 15 // PORTA


#define ISOLTR_2_SSA0_PIN 5 // PORTB
#define ISOLTR_2_SSA1_PIN 6 // PORTB

#define ISOLTR_3_SSA0_PIN 7 // PORTB
#define ISOLTR_3_SSA1_PIN 8 // PORTB

#define ISOLTR_4_SSA0_PIN 14 // PORTB
#define ISOLTR_4_SSA1_PIN 15 // PORTB

#define PD_PS1_SEN_PIN 8 // PORTC
#define PD_PS2_SEN_PIN 7 // PORTC
#define PD_PS3_SEN_PIN 6 // PORTC

#define SPIO_NPCS_G2A_PIN 0 // PORTB
#define SPIO_NPCS_G2B_PIN 1 // PORTB
#define SPIO_NPCS_G1_PIN 2 // PORTB

#define THERMOCOUPLE_DRDY_PIN 14 // PORTA
#define DRDY_TS1_PIN 15 // PORTA
#define DRDY_TS2_PIN 16 // PORTA
#define DRDY_TS3_PIN 20 // PORTA

#define DRDY_CS1_TS_PIN 17 // PORTA
#define DRDY_CS2_TS_PIN 18 // PORTA
#define DRDY_CS3_TS_PIN 19 // PORTA
#define SPARE_TS_DRDY_PIN 21 // PORTA

#define PS1_DRDY_N_PIN 2 // PORTD
#define PS2_DRDY_N_PIN 1 // PORTD
#define PS3_DRDY_N_PIN 0 // PORTD

#define BLOOD_LEAK_DETECTOR_PIN 4 // PORTC
#define BLD_ZERO_CONFIRM_OP_PIN 3 // PORTC



#define BLD_ZERO_PIN 1 // PORTC
#define BLD_TEST_PIN 2 // PORTC









#define SPI0_NPCS3 23 //PORTB




void SENSOR_D_INIT_PIO(void);
uint8_t SENSOR_D_SET_PIO( pio_type peripheral,uint32_t pio);
uint8_t SENSOR_D_RESET_PIO(const pio_type peripheral,uint32_t pio);
uint16_t SENSOR_D_READ_PIO(pio_type peripheral,uint32_t pio);
uint32_t dd_disable_PIO(Pio *p_pio, uint32_t ul_id);
uint32_t dd_enable_PIO(Pio *p_pio, uint32_t ul_id);
uint32_t dd_disable_OUTPUT(Pio *p_pio, uint32_t pin_no);
uint32_t dd_enable_OUTPUT(Pio *p_pio, uint32_t pin_no);
uint32_t dd_enable_GLITCH_INPUT_FILTER(Pio *p_pio, uint32_t pin_no);
uint32_t dd_disable_GLITCH_INPUT_FILTER(Pio *p_pio, uint32_t pin_no);
uint32_t dd_SET_OUTPUT_DATA(Pio *p_pio, uint32_t pin_no);
uint32_t dd_CLEAR_OUTPUT_DATA(Pio *p_pio, uint32_t pin_no);
uint32_t dd_enable_PULLUP(Pio *p_pio, uint32_t pin_no);
uint32_t dd_disable_PULLUP(Pio *p_pio, uint32_t pin_no);
uint32_t dd_enable_OUTPUT_WRITE(Pio *p_pio, uint32_t pin_no);
uint32_t dd_disable_OUTPUT_WRITE(Pio *p_pio, uint32_t pin_no);
uint32_t dd_PIO_INTERRUPT_DISABLE(Pio *p_pio, uint32_t pin_no);
uint32_t dd_PIO_INTERRUPT_ENABLE(Pio *p_pio, uint32_t pin_no);
void dd_SPI_pins_INIT(void); 
void dd_UART_pins_init(void);
void dd_CAN_pins_INIT(void);
void dd_PRESSURE_SENSOR_GPIO_PINS(void);
void dd_TEMP_SENSOR_PINS_INIT(void);



#endif /* DD_GPIO_H_ */