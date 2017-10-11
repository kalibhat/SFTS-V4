/*
 * DD_SPI.c
 *
 * Created: 3/6/2017 6:14:04 PM
 *  Author: Electrical
 */ 


#include "asf.h"
#include "DD_SPI.h"
#include "SENSOR_CPU.h"
#include "PLATFORM/DRIVER/DD_SYS/DD_SYS.h"
#include "Platform/Driver/DRIVER_CALLS.h"
#include "PLATFORM//DRIVER//DD_CONSOLE/DD_CONSOLE.h"


uint8_t SPI_CHIP_SEL=0;


#define SPI_CHIP_PCS ((~(1u<<(SPI_CHIP_SEL)))& 0xF)



/* Delay before SPCK. */
#define SPI_DLYBS 0x100

/* Delay between consecutive transfers. */
#define SPI_DLYBCT 0x040

#define  ul_value 8

static uint32_t gs_ul_spi_clock = 330000;
volatile max_data spi_data;




void dd_spi_configure_cs_behavior( CHIP_SEL_GROUP_TYPE ul_pcs_group_id, uint32_t ul_cs_behavior) {
	if (ul_cs_behavior == SPI_CS_RISE_FORCED) {
		SPI0->SPI_CSR[ul_pcs_group_id] &= (~SPI_CSR_CSAAT);
		SPI0->SPI_CSR[ul_pcs_group_id] |= SPI_CSR_CSNAAT;
		} else if (ul_cs_behavior == SPI_CS_RISE_NO_TX) {
		SPI0->SPI_CSR[ul_pcs_group_id] &= (~SPI_CSR_CSAAT);
		SPI0->SPI_CSR[ul_pcs_group_id] &= (~SPI_CSR_CSNAAT);
		} else if (ul_cs_behavior == SPI_CS_KEEP_LOW) {
		SPI0->SPI_CSR[ul_pcs_group_id] |= SPI_CSR_CSAAT;
	}
}

void dd_spi_set_delay_between_chip_select(uint32_t ul_delay) {
	SPI0->SPI_MR &= (~SPI_MR_DLYBCS_Msk);
	SPI0->SPI_MR |= SPI_MR_DLYBCS(ul_delay);
}

void dd_spi_set_transfer_delay(CHIP_SEL_GROUP_TYPE ul_pcs_group_id, uint8_t uc_dlybs, uint8_t uc_dlybct)  {
	SPI0->SPI_CSR[ul_pcs_group_id] &= ~(SPI_CSR_DLYBS_Msk | SPI_CSR_DLYBCT_Msk);
	SPI0->SPI_CSR[ul_pcs_group_id] |= SPI_CSR_DLYBS(uc_dlybs) | SPI_CSR_DLYBCT(uc_dlybct);
}

void dd_spi_set_bits_per_transfer(CHIP_SEL_GROUP_TYPE ul_pcs_group_id, uint32_t ul_bits)
{
	SPI0->SPI_CSR[ul_pcs_group_id] &= (~SPI_CSR_BITS_Msk);
	SPI0->SPI_CSR[ul_pcs_group_id] |= ul_bits;
}

void dd_spi_set_writeprotect(uint32_t ul_enable)   {
	
	if (ul_enable) {
		SPI0->SPI_WPMR = (0x535049u << 8) | SPI_WPMR_WPEN;
	} else {
		SPI0->SPI_WPMR =  (0x53504900);
	}
	
}

void dd_spi_set_clock_polarity(CHIP_SEL_GROUP_TYPE ul_pcs_group_id, uint32_t ul_polarity)   {
	
	if (ul_polarity) {
		SPI0->SPI_CSR[ul_pcs_group_id] |= SPI_CSR_CPOL;
		} else {
		SPI0->SPI_CSR[ul_pcs_group_id] &= (~SPI_CSR_CPOL);
	}	
}

void dd_spi_set_clock_phase(CHIP_SEL_GROUP_TYPE ul_pcs_group_id, uint32_t ul_phase)
{
	if (ul_phase) {
		SPI0->SPI_CSR[ul_pcs_group_id] |= SPI_CSR_NCPHA;
		} else {
		SPI0->SPI_CSR[ul_pcs_group_id] &= (~SPI_CSR_NCPHA);
	}
}

int16_t dd_spi_set_baudrate_div(CHIP_SEL_GROUP_TYPE ul_pcs_group_id,uint8_t uc_baudrate_divider)
{
	/* Programming the SCBR field to 0 is forbidden */
	if (!uc_baudrate_divider)
	return -1;

	SPI0->SPI_CSR[ul_pcs_group_id] &= (~SPI_CSR_SCBR_Msk);
	SPI0->SPI_CSR[ul_pcs_group_id] |= SPI_CSR_SCBR(uc_baudrate_divider);
	return 0;
}

void dd_spi_set_peripheral_chip_select_value(SPI_DECODER_MUX_ID_TYPE pcs_ul_value)
{
	SPI0->SPI_MR &= (~SPI_MR_PCS_Msk);
	SPI0->SPI_MR |= ((SPI_MR_PCS_Msk & ((pcs_ul_value) << SPI_MR_PCS_Pos)));
}


void dd_spi_master_transfer(void *p_buf, uint32_t size, SPI_DECODER_MUX_ID_TYPE uc_pcs)  {
	uint32_t i;
	uint8_t uc__temp_pcs;
	static uint8_t data;

	uint8_t *p_buffer;

	p_buffer = p_buf;

	for (i = 0; i < size; i++) {		
			dd_spi_write(p_buffer[i], uc_pcs,0); 	
		/* Wait transfer done. */
		while  ( ((SPI0->SPI_SR) & SPI_SR_RDRF) == 0);		  
		dd_spi_read(&data);
		p_buffer[i] = data;
	}
}

void dd_spi_master_read(void *p_buf, uint32_t size, SPI_DECODER_MUX_ID_TYPE uc_pcs )   {
	uint32_t i;
	//uint8_t uc_pcs;
	uint8_t data;

	uint8_t *p_buffer;

	p_buffer = p_buf;
	
	for (i = 0; i < size ; i++) {  
		  
		dd_spi_write(p_buffer[0], uc_pcs, 0);
		/* Wait transfer done. */
		while ( ((SPI0->SPI_SR) & SPI_SR_RDRF) == 0);

		  
		dd_spi_read(&data);

		

		p_buffer[i] = data;
		
	}
}






spi_status_t  dd_spi_write(uint8_t us_data, SPI_DECODER_MUX_ID_TYPE uc_pcs, uint8_t uc_last) {
	uint32_t timeout = SPI_TIMEOUT;
	uint32_t value;
	uint8_t cs_reg;
	while (!(SPI0->SPI_SR & SPI_SR_TDRE)) {
		if (!timeout--) {
			return SPI_ERROR_TIMEOUT;
		}
	}
	//cs_reg = ((~(1u<<(uc_pcs)))& 0xF)                
	//value = SPI_MR_PCS(uc_pcs);
	//(SPI0->SPI_MR)|= value;
	
	if (uc_last) {(SPI0->SPI_CR) |= SPI_CR_LASTXFER; }
	else {  (SPI0->SPI_CR) &= (~SPI_CR_LASTXFER);   }
	
	value = SPI_TDR_TD(us_data);
	SPI0->SPI_TDR = value;
	return SPI_OK;
}


spi_status_t dd_spi_read(uint8_t *us_data) {
	uint32_t timeout = SPI_TIMEOUT;
	static uint32_t reg_value;

	while (!(SPI0->SPI_SR & SPI_SR_RDRF)) {
		if (!timeout--) {
			return SPI_ERROR_TIMEOUT;
		}
	}

	reg_value = SPI0->SPI_RDR;	
	*us_data =  (reg_value & SPI_RDR_RD_Msk);
	
// 		if(*us_data == 0xff)
// 		{
// 			return SPI_ERROR;
// 		}
// 		else
// 		{
// 			return SPI_OK;
// 		}
	return SPI_OK;
}

void SENSOR_DD_INIT_SPI(void)   {
	
	//dd_spi_set_writeprotect(0);	
	SPI0->SPI_WPMR =  0x53504900;
	
	 
	 puts(STRING_HEADER);	
	 
	 sysclk_enable_peripheral_clock(ID_SPI0);
	 		
	SPI0->SPI_CR = SPI_CR_SPIDIS;
	SPI0->SPI_CR = SPI_CR_SWRST;		
	SPI0->SPI_CR = SPI_CR_LASTXFER;	
	
	
	SPI0->SPI_MR |= SPI_MR_MSTR;	
	SPI0->SPI_MR |= SPI_MR_MODFDIS ;	
	SPI0->SPI_MR |= SPI_MR_PCSDEC;
	
	//chip selection procedure done here
	//SPI0->SPI_MR &= ~SPI_MR_PS; // SPI SET FIXED PERIPHERAL.	
	//SPI0->SPI_MR &= (~SPI_MR_PCSDEC); // The chip selects are directly connected to a peripheral device.	
	SPI0->SPI_MR &= (~SPI_MR_PCS_Msk);
	SPI0->SPI_MR |= SPI_MR_PCS(SPI_CHIP_PCS);	
	
	//spi_set_peripheral_chip_select_value(SPI_MASTER_BASE, SPI_CHIP_PCS);	
	dd_spi_set_clock_polarity(0, SPI_CLK_POLARITY_0);
	dd_spi_set_clock_phase(0, SPI_CLK_PHASE_1);	
	dd_spi_set_bits_per_transfer(0,SPI_CSR_BITS_8_BIT);	
	dd_spi_set_baudrate_div(0, (sysclk_get_peripheral_hz()/ gs_ul_spi_clock));	
	dd_spi_set_transfer_delay(0, SPI_DLYBS,SPI_DLYBCT);
	
	dd_spi_set_clock_polarity(1, SPI_CLK_POLARITY_0);
	dd_spi_set_clock_phase(1, SPI_CLK_PHASE_1);
	dd_spi_set_bits_per_transfer(1,SPI_CSR_BITS_8_BIT);
	dd_spi_set_baudrate_div(1, (sysclk_get_peripheral_hz()/ gs_ul_spi_clock));
	dd_spi_set_transfer_delay(1, SPI_DLYBS,SPI_DLYBCT);
	
	dd_spi_set_clock_polarity(2, SPI_CLK_POLARITY_0);
	dd_spi_set_clock_phase(2, SPI_CLK_PHASE_1);
	dd_spi_set_bits_per_transfer(2,SPI_CSR_BITS_8_BIT);
	dd_spi_set_baudrate_div(2, (sysclk_get_peripheral_hz()/ gs_ul_spi_clock));
	dd_spi_set_transfer_delay(2, SPI_DLYBS,SPI_DLYBCT);
	
	dd_spi_set_clock_polarity(3, SPI_CLK_POLARITY_0);
	dd_spi_set_clock_phase(3, SPI_CLK_PHASE_1);
	dd_spi_set_bits_per_transfer(3,SPI_CSR_BITS_8_BIT);
	dd_spi_set_baudrate_div(3, (sysclk_get_peripheral_hz()/ gs_ul_spi_clock));
	dd_spi_set_transfer_delay(3, SPI_DLYBS,SPI_DLYBCT);
	
	
	
	SPI0->SPI_CR = SPI_CR_SPIEN;		
}
