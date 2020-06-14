#define F_CPU 8000000UL
#define BAUDRATE 9600UL
#define BAUD_PRESCALLER (((F_CPU / (BAUDRATE * 16UL))) - 1)

#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>

//#include <stdio.h>

#include "setup_rtd.h"

void ClockSetup()
{
	//Ställer in extern klocka och prescalers.
	CCP			=	CCP_IOREG_gc;
	OSC.XOSCCTRL = OSC_FRQRANGE_2TO9_gc | OSC_XOSCSEL_XTAL_16KCLK_gc;		//Freqrange 2-9 MHz, Startuptime = 16K cycles
	
	CCP			=	CCP_IOREG_gc;
	OSC.CTRL	=	OSC_XOSCEN_bm;
	while(!(OSC_STATUS & OSC_XOSCRDY_bm));
	
	CCP			=	CCP_IOREG_gc;				//Tillåt ändring av klockan. Configuration Change Protection av
	CLK.CTRL	=	CLK_SCLKSEL_XOSC_gc;		//Välj externklocka

	CCP			=	CCP_IOREG_gc;
	CLK.PSCTRL	=	CLK_PSADIV_1_gc | CLK_PSBCDIV_1_1_gc;	//Div: clk_per4 = 1 Div: CLKper2 =1, CLK_cpu = 1, CLK_per =1
	
	CCP			= CCP_IOREG_gc;
	OSC.XOSCFAIL = OSC_XOSCFDEN_bm;
	
	//CCP			=	CCP_IOREG_gc;
	//CLK.LOCK	=	CLK_LOCK_bm;				//Lås klockan tills nästa reset
}
void RtdEnable()
{
	PORTD.OUTSET = PIN0_bm;	//Aktivera strömkälla för rtd
}
void RtdDisable()
{
	PORTD.OUTCLR = PIN0_bm;	//Av-aktivera strömkälla för rtd
}
void PortSetup()
{
	//PortA konfig.
	PORTA.DIRCLR	=	PIN3_bm | PIN4_bm |PIN5_bm;			//IN:	Pin3 = RTD1-AIN1
	//		Pin4 = RTD1-AIN2, Pin5 = RTD1-REFP0;
	//PortD konfig.
	PORTD.DIRSET	=	PIN0_bm;							//OUT:	Pin0 = 3.3 AVR-RTD1
	
	//PortE konfig.
	PORTE.DIRSET	=	0xff;								//OUT: Pin0..7 = LED-PE0..7
	
	PORTF.DIRCLR = PIN2_bm;  //RX
	PORTF.DIRSET = PIN3_bm;	//TX
}
uint8_t readAdcCalibByte(uint8_t index)
{
	cli();
	uint8_t reg = SREG;
	NVM.CMD = NVM_CMD_READ_CALIB_ROW_gc;
	uint8_t calibByte = pgm_read_byte(index);
	NVM.CMD = NVM_CMD_NO_OPERATION_gc;
	SREG = reg;
	sei();
	return calibByte;
}
void AdcSetup()
{
	//ADCA.CALL = readAdcCalibByte(PRODSIGNATURES_ADCACAL0);		//Läs in kalibrering från minne
	//ADCA.CALH = readAdcCalibByte(PRODSIGNATURES_ADCACAL1);
	
	//ADC-setup, Clk_adc använder CLK_per
	//ADCA temp

	ADCA.PRESCALER	= ADC_PRESCALER_DIV4_gc;
	ADCA.CTRLB		= ADC_RESOLUTION_12BIT_gc; //| ADC_CONMODE_bm; //ADC-CONMODE_bm = 1
	ADCA.CTRLA		= ADC_ENABLE_bm;


	//ADCA.REFCTRL		= 0x01<<4; //	 ADC_REFSEL_INTVCC_gc-- Vcc / 1.6
	
	ADCA.CH1.MUXCTRL	=	ADC_CH_MUXPOS_PIN3_gc | ADC_CH_MUXNEG_PIN4_gc ;		//RTD1-AIN1 = P; RTD2-AIN2 = N
	ADCA.CH2.MUXCTRL	=	ADC_CH_MUXPOS_PIN5_gc;

	//ADCB.CAL = "kalibreringsvärde från fabrik"
	//The CALL and CALH register pair hold the 12-bit calibration value. The ADC pipeline is calibrated during production
	//programming, and the calibration value must be read from the signature row and written to the CAL register from
	//software.
}
void UsartSetup()
{
	USARTF0.BAUDCTRLA = (uint8_t)(BAUD_PRESCALLER);
	USARTF0.BAUDCTRLB = (uint8_t)(BAUD_PRESCALLER>>8);
	
	USARTF0.CTRLB	=  USART_RXEN_bm | USART_TXEN_bm;
	USARTF0.CTRLC	= USART_CMODE_ASYNCHRONOUS_gc | USART_PMODE_DISABLED_gc | USART_CHSIZE_8BIT_gc;
}
void UsartTx(unsigned char data)
{
	//while(!(USARTF0.STATUS & USART_RXCIF_bm));
	//ch=USARTF0.DATA; //receive character from user
	while(!(USARTF0.STATUS & USART_DREIF_bm));
	USARTF0.DATA=data;
}
void UsartRx(unsigned char data)
{
	while(!(USARTF0.STATUS & USART_RXCIF_bm));
	data=USARTF0.DATA; //receive character from user
	//while(!(USARTF0.STATUS & USART_DREIF_bm));
	//USARTF0.DATA=data;
}
void UsartTxString(char* StringPtr)
{
	while(*StringPtr != 0x00)
	{
		UsartTx(*StringPtr);
		StringPtr++;
	}	
}