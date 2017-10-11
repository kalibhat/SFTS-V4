/*
 * CFile1.c
 *
 * Created: 3/13/2017 3:50:14 PM
 *  Author: VarunK
 */ 


#include "asf.h"
#include "spi.h"
#include "Platform/Drivers/DD_SPI/DD_SPI.h"
//#include "SENSOR_CPU.h"
//#include "PLATFORM/DRIVER/DD_SYS/DD_SYS.h"


uint8_t SPI_CHIP_SEL=0;


#define SPI_CHIP_PCS ((~(1u<<(SPI_CHIP_SEL)))& 0xF)



/* Delay before SPCK. */
#define SPI_DLYBS 0x80

/* Delay between consecutive transfers. */
#define SPI_DLYBCT 0x10

#define  ul_value 8

static uint32_t gs_ul_spi_clock = 5000000;
volatile max_data spi_data;
spi_status_t;




void dd_spi_configure_cs_behavior( uint16_t ul_pcs_group_id, uint32_t ul_cs_behavior) {
	if (ul_cs_behavior == SPI_CS_RISE_FORCED) {
		SPI0->SPI_CSR[0] &= (~SPI_CSR_CSAAT);
		SPI0->SPI_CSR[0] |= SPI_CSR_CSNAAT;
		} else if (ul_cs_behavior == SPI_CS_RISE_NO_TX) {
		SPI0->SPI_CSR[0] &= (~SPI_CSR_CSAAT);
		SPI0->SPI_CSR[0] &= (~SPI_CSR_CSNAAT);
		} else if (ul_cs_behavior == SPI_CS_KEEP_LOW) {
		SPI0->SPI_CSR[0] |= SPI_CSR_CSAAT;
	}
}

void dd_spi_set_delay_between_chip_select(uint32_t ul_delay) {
	SPI0->SPI_MR &= (~SPI_MR_DLYBCS_Msk);
	SPI0->SPI_MR |= SPI_MR_DLYBCS(ul_delay);
}

void dd_spi_set_transfer_delay(uint16_t ul_pcs_group_id, uint8_t uc_dlybs, uint8_t uc_dlybct)  {
	SPI0->SPI_CSR[ul_pcs_group_id] &= ~(SPI_CSR_DLYBS_Msk | SPI_CSR_DLYBCT_Msk);
	SPI0->SPI_CSR[ul_pcs_group_id] |= SPI_CSR_DLYBS(uc_dlybs) | SPI_CSR_DLYBCT(uc_dlybct);
}

void dd_spi_set_bits_per_transfer(uint16_t ul_pcs_group_id, uint32_t ul_bits)
{
	SPI0->SPI_CSR[ul_pcs_group_id] &= (~SPI_CSR_BITS_Msk);
	SPI0->SPI_CSR[ul_pcs_group_id] |= ul_bits;
}

void dd_spi_set_writeprotect(uint32_t ul_enable)   {
	
	if (ul_enable) {
		SPI0->SPI_WPMR = (0x5350490u << 1) | SPI_WPMR_WPEN;
	} else {
		SPI0->SPI_WPMR =  (0x5350490u << 1);
	}
	
}

void dd_spi_set_clock_polarity(uint16_t ul_pcs_group_id, uint32_t ul_polarity)   {
	
	if (ul_polarity) {
		SPI0->SPI_CSR[ul_pcs_group_id] |= SPI_CSR_CPOL;
		} else {
		SPI0->SPI_CSR[ul_pcs_group_id] &= (~SPI_CSR_CPOL);
	}	
}

void dd_spi_set_clock_phase(uint16_t ul_pcs_group_id, uint32_t ul_phase)
{
	if (ul_phase) {
		SPI0->SPI_CSR[ul_pcs_group_id] |= SPI_CSR_NCPHA;
		} else {
		SPI0->SPI_CSR[ul_pcs_group_id] &= (~SPI_CSR_NCPHA);
	}
}

int16_t dd_spi_set_baudrate_div(uint16_t ul_pcs_group_id,uint8_t uc_baudrate_divider)
{
	/* Programming the SCBR field to 0 is forbidden */
	if (!uc_baudrate_divider)
	return -1;

	SPI0->SPI_CSR[ul_pcs_group_id] &= (~SPI_CSR_SCBR_Msk);
	SPI0->SPI_CSR[ul_pcs_group_id] |= SPI_CSR_SCBR(uc_baudrate_divider);
	return 0;
}

void dd_spi_set_peripheral_chip_select_value(uint16_t pcs_ul_value)
{
	SPI0->SPI_MR &= (~SPI_MR_PCS_Msk);
	SPI0->SPI_MR |= ((SPI_MR_PCS_Msk & ((pcs_ul_value) << SPI_MR_PCS_Pos)));
}

void DD_SPI_SET_POT(void *p_buf, uint32_t size)  {
	uint32_t i;
	uint8_t uc_pcs;
	static uint16_t data;

	uint16_t *p_buffer;

	p_buffer = p_buf;

	for (i = 0; i < size; i++) {
		dd_spi_write(p_buffer[i]);
		
		for (uint32_t  j=0;j<10000;j++)
		{
			;
		}
		/* Wait transfer done. */
		while  ( ((SPI0->SPI_SR) & SPI_SR_RDRF) == 0);
		dd_spi_read(&data, &uc_pcs);
		p_buffer[i] = data;
	}
}

spi_status_t dd_spi_read(uint16_t *us_data, uint8_t *p_pcs) {
	uint32_t timeout = SPI_TIMEOUT;
	static uint32_t reg_value;

	while (!(SPI0->SPI_SR & SPI_SR_RDRF)) {
		if (!timeout--) {
			return SPI_ERROR_TIMEOUT;
		}
	}

	reg_value = SPI0->SPI_RDR;
	if ((SPI0->SPI_MR & SPI_MR_PS)) {
		*p_pcs = (uint8_t) ((reg_value & SPI_RDR_PCS_Msk) >> SPI_RDR_PCS_Pos);
	}
	*us_data = (uint16_t) (reg_value & SPI_RDR_RD_Msk);

	return SPI_OK;
}

spi_status_t dd_spi_write(uint16_t us_data)  {
	uint32_t timeout = SPI_TIMEOUT;
	uint32_t value;

	while (!(SPI0->SPI_SR & SPI_SR_TDRE)) {
		if (!timeout--) {
			return SPI_ERROR_TIMEOUT;
		}
	}
	value = SPI_TDR_TD(us_data);
	SPI0->SPI_TDR = value;

	return SPI_OK;
}


static void dd_spi_master_read(void *p_buf, uint32_t size)
{
	uint32_t i;
	uint8_t uc_pcs;
	static uint16_t data;

	uint8_t *p_buffer;

	p_buffer = p_buf;
	
	for (i = 0; i < size +1; i++) {
		dd_spi_write(p_buffer[0]);
		/* Wait transfer done. */		
		while(((SPI0->SPI_SR) & SPI_SR_RDRF) == 0);
		dd_spi_read(&data, &uc_pcs);
		p_buffer[i] = data;
	}
}

 

// uint16_t DD_SENSOR_CPU_SPI_READ( uint8_t pcs_value)  {
// 	
// 	
// 	
// 	
// 	return 0;
// }

void dd_spi_master_go()  {
	
volatile count=0;
//spi_master_initialize();
volatile uint32_t cmd;
uint32_t block;
uint32_t i;
static bool data_ready = false;

uint8_t data_array[3];

data_array[0] = 0x80;
data_array[1] = 0b00100011;
DD_SPI_SET_POT(&data_array, sizeof(data_array));


data_array[0] = 0x83;
data_array[1] = 0x80;
DD_SPI_SET_POT(&data_array, 2);
data_array[0] = 0x84;
data_array[1] = 0x00;
DD_SPI_SET_POT(&data_array, 2);
data_array[0] = 0x85;
data_array[1] = 0x40;
DD_SPI_SET_POT(&data_array, 2);
data_array[0] = 0x86;
data_array[1] = 0x00;
DD_SPI_SET_POT(&data_array, 2);
while (count< 100)
{
	count++;
}
count=0;

data_array[0] = 0x80;
data_array[1] = 0b11100011;
DD_SPI_SET_POT(&data_array, 2);

while (count< 100)
{
	count++;
}
count=0;
uint8_t temp;
data_array[0] = 0x01;
data_array[1] = 0x00;
data_array[2] = 0x00;

if( (PIOD->PIO_PDSR & (1<<7)) == 0)
{

	data_ready = true;
	dd_spi_master_read(&data_array, 2);
	spi_data.bytearray[1]=data_array[1];
	spi_data.bytearray[0]=data_array[2];
	cmd = spi_data.Twobyte;
	//printf("%d\n",cmd);
	cmd = cmd/2;
}
else
{
	data_ready = false;
}
}

//void dd_spi_master_initialize(void)
void DD_INIT_SPI()
{
	PIOA->PIO_WPMR = 0x50494F00;
	//PIOA->PIO_PER  = 0x10000000;
	//PIOA->PIO_OER  = 0x10000000;         //CS0
	PIOA->PIO_PDR  = 0x1E000000;         //MISO,MOSI,SCK
//	PIOA->PIO_ABSR = 0x00000000;
	
	
	SPI0->SPI_WPMR = 0x53504900;         // SPI_WPMR_WPEN
	//puts("-I- Initialize SPI as master\r");
	//dd_spi_set_writeprotect(0);
	
	PMC->PMC_WPMR = 0x504D4300;
	spi_enable_clock(SPI_MASTER_BASE);
	
	
	
	SPI0->SPI_CR = SPI_CR_SPIDIS;
	//spi_disable(SPI_MASTER_BASE);
	
	SPI0->SPI_CR = SPI_CR_SWRST;
	//spi_reset(SPI_MASTER_BASE);
	
	SPI0->SPI_CR = SPI_CR_LASTXFER;
	//spi_set_lastxfer(SPI_MASTER_BASE);
	
	SPI0->SPI_MR |= SPI_MR_MSTR;
	//spi_set_master_mode(SPI_MASTER_BASE);
	
	SPI0->SPI_MR |= SPI_MR_MODFDIS;
	//spi_disable_mode_fault_detect(SPI_MASTER_BASE);
	
	
	
	
	
	
	//chip selection procedure done here
	SPI0->SPI_MR &= (~SPI_MR_PS); // SPI SET FIXED PERIPHERAL.	/ 	SPI0->SPI_MR &= (~SPI_MR_PCSDEC); // The chip selects are directly connected to a peripheral device.
	SPI0->SPI_MR &= (~SPI_MR_PCS_Msk);
	SPI0->SPI_MR &= (~SPI_MR_PCSDEC);
	SPI0->SPI_MR |= SPI_MR_PCS(SPI_CHIP_PCS);

	// 	SPI0->SPI_MR = 0x00000001;
	// 	SPI0->SPI_CSR = 0x10800080;
	
	
	//spi_set_peripheral_chip_select_value(SPI_MASTER_BASE, SPI_CHIP_PCS);
	dd_spi_set_clock_polarity(SPI_CHIP_SEL, SPI_CLK_POLARITY_0);
	dd_spi_set_clock_phase(SPI_CHIP_SEL, SPI_CLK_PHASE_0);
	dd_spi_set_bits_per_transfer(SPI_CHIP_SEL,SPI_CSR_BITS_16_BIT);
	dd_spi_set_baudrate_div(SPI_CHIP_SEL, 0xff );	//(sysclk_get_peripheral_hz()/ gs_ul_spi_clock)
	dd_spi_set_transfer_delay(SPI_CHIP_SEL, SPI_DLYBS,SPI_DLYBCT);
	SPI0->SPI_CR = SPI_CR_SPIEN;
}
