/*
 * pHsetup.c
 *
 * Created: 2020-05-28 12:54:50
 *  Author: vili2
 */ 
#define F_CPU 8000000UL
#define BAUDRATE 9600UL
#define BAUD_PRESCALLER (((F_CPU / (BAUDRATE * 16UL))) - 1)
#define blink_freq 100

#include <avr/io.h>
#include <stdio.h>
#include <util/delay.h>
#include "pHsetup.h"

void ClockSetup()
{
	//Ställer in extern klocka och prescalers.
	CCP			=	CCP_IOREG_gc;
	OSC.XOSCCTRL = OSC_FRQRANGE_2TO9_gc | OSC_XOSCPWR_bm |OSC_XOSCSEL_XTAL_16KCLK_gc;		//Freqrange 2-9 MHz, Startuptime = 16K cycles
	
	CCP			=	CCP_IOREG_gc;
	OSC.CTRL	=	OSC_XOSCEN_bm;
	while(!(OSC_STATUS & OSC_XOSCRDY_bm));
	
	CCP			=	CCP_IOREG_gc;				
	CLK.CTRL	=	CLK_SCLKSEL_XOSC_gc;		

	CCP			=	CCP_IOREG_gc;
	CLK.PSCTRL	=	CLK_PSADIV_1_gc | CLK_PSBCDIV_1_1_gc;	//Div: clk_per4 = 1 Div: CLKper2 =1, CLK_cpu = 1, CLK_per =1
}
void PortSetup()
{
	//PortA konfig.
	PORTA.DIRCLR	=	PIN1_bm;		//IN:	Pin1 = PH-measure
	
	PORTE.DIRSET		= 0xff;
}
void AdcSetup()
{
	//ADC-setup, Clk_adc använder CLK_per
	//ADCA för pH
	ADCA.CTRLA			= ADC_ENABLE_bm;
	ADCA.PRESCALER		= ADC_PRESCALER_DIV4_gc;
	ADCA.CTRLB			= ADC_RESOLUTION_12BIT_gc;
	ADCA.REFCTRL		= ADC_REFSEL_INTVCC_gc;				//ADC_REFSEL_INTVCC_gc-- Vcc / 1.6
	ADCA.CH0.MUXCTRL	=	ADC_CH_MUXPOS_PIN1_gc;		//Kanal1 -> PIN0 PH-measure
}
uint16_t AdcSamplePH()
{
	uint16_t sample;
	
	ADCA.CH0.CTRL = ADC_CH_START_bm | ADC_CH_GAIN_1X_gc | ADC_CH_INPUTMODE_SINGLEENDED_gc;					//Starta adc på pHMeasure
	while(ADCA_CTRLA & ADC_CH0START_bm);				// Vänta på att AD är startad
	
	while(!(ADCA_CH0_INTFLAGS & ADC_CH_CHIF_bm));		//Vänta på AD omvandling kanal0
	
	sample = ADCA.CH0RES -0x044;						//Resultat kanal0 - offset
	
	ADCA.INTFLAGS = ADC_CH0IF_bm;						//Nollställ ADC-interuptflaggor
	return sample;
}
float GetVoltage(uint16_t sample)
{
	return ((float)(sample*2.0625))/(4096);
}
uint16_t AdcMedPH()
{
	uint32_t	sum = 0;
	uint16_t	i;
	
	// Medelvärde
	for (i = 0; i < 128; i++)
	{
		sum	= sum + AdcSamplePH();
	}
	return sum/128;
}
float AdcConvPH(uint16_t phresult_med)
{
	float phresult = 0;
	float ph_temp = 0;
	float ph_out = 0;
	
	ph_temp = GetVoltage(phresult_med);
	
	//phresult = (ph_temp - 1.187 );		//Kompensera för sensorns offset 
	phresult = (ph_temp - 1.25 );			//Ideal sensor 
	
	ph_out = 7 + (phresult)/0.060;			//Kompensera för sensorns gain
	
	return ph_out;
}
float  GetPh()
{
	uint16_t phresult_med = 0;
	
	float phresult = 0;
	
	phresult_med = AdcMedPH();
	phresult = AdcConvPH(phresult_med);
	
	return phresult;
}
void UsartSetup()
{
	USARTF0.BAUDCTRLA = (uint8_t)(BAUD_PRESCALLER);
	USARTF0.BAUDCTRLB = (uint8_t)(BAUD_PRESCALLER>>8);
	
	USARTF0.CTRLB	=  USART_RXEN_bm | USART_TXEN_bm;
	USARTF0.CTRLC	= USART_CMODE_ASYNCHRONOUS_gc | USART_PMODE_DISABLED_gc | USART_CHSIZE_8BIT_gc;
	
	PORTF.DIRSET = PIN3_bm;
	PORTF.DIRCLR = PIN2_bm;
}
void UsartTx(unsigned char data)
{
	while(!(USARTF0.STATUS & USART_DREIF_bm));
	USARTF0.DATA=data;
}
void UsartRx(unsigned char data)
{
	while(!(USARTF0.STATUS & USART_RXCIF_bm));
	data=USARTF0.DATA;
}
void UsartTxString(char* StringPtr)
{
	while(*StringPtr != 0x00)
	{
		UsartTx(*StringPtr);
		StringPtr++;
	}
}