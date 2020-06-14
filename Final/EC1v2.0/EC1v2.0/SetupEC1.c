/*
 * SetupEC1.c
 *
 * Created: 2020-05-26 08:51:34
 *  Author: vili2
 */ 
#define F_CPU 8000000UL
#define BAUDRATE 9600UL
#define BAUD_PRESCALLER (((F_CPU / (BAUDRATE * 16UL))) - 1)
#define blink_freq 100

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#include "SetupEC1.h"


void ClockSetup()
{
	//Ställer in extern klocka och prescalers.
	CCP			 =	CCP_IOREG_gc;									//Tillåt ändring av klockan. Configuration Change Protection av
	OSC.XOSCCTRL = OSC_FRQRANGE_2TO9_gc | OSC_XOSCPWR_bm | OSC_XOSCSEL_XTAL_16KCLK_gc;		//Freqrange 2-9 MHz, Startuptime = 16K cycles
	
	CCP			 =	CCP_IOREG_gc;
	OSC.CTRL	=	OSC_XOSCEN_bm;
	while(!(OSC_STATUS & OSC_XOSCRDY_bm));
	
	CCP			=	CCP_IOREG_gc;				
	CLK.CTRL	=	CLK_SCLKSEL_XOSC_gc;		//Välj extern klocka

	CCP			=	CCP_IOREG_gc;
	CLK.PSCTRL	=	CLK_PSADIV_1_gc | CLK_PSBCDIV_1_1_gc;	//Div: clk_per4 = 1 Div: CLKper2 =1, CLK_cpu = 1, CLK_per =1
	
	CCP			= CCP_IOREG_gc;
	OSC.XOSCFAIL = OSC_XOSCFDEN_bm;
	
}
void EnableEC1()
{
	//DAC på
	DACB.CH0DATA = 300;								//Styr exalteringsspänningen
	PORTE.OUTSET = 0xff;							//LED på
	PORTC.OUTCLR = PIN2_bm;							//Transistorer av -> sensor känner exalteringsspänningen
}
void DisableEC1()
{
	DACB.CH0DATA = 0;
	PORTE.OUTCLR = 0xff;
	PORTC.OUTSET = PIN2_bm;								//Transistorer på -> sensor jordad
}
void PortSetupEC1()
{
	//PortB konfig.
	PORTB.DIRCLR	=	PIN0_bm | PIN1_bm; 						//IN:	Pin0 = ECS-vout1, Pin1 = ECS-vout2.
	PORTB.DIRSET	=	PIN2_bm;								//OUT:	Pin2 = ECS-DAC-1.2
	
	//PortC konfig.
	PORTC.DIRSET	=	PIN1_bm | PIN2_bm;						//OUT: Pin1=ECS-SWITCH, Pin2 = Turn off ecs

	//PortE konfig.
	PORTE.DIRSET	=	0xff;									//OUT: Pin0..7 = LED-PE0..7
	
	PORTF.DIRSET	= 0xff;										//OUT: Testpinnar
}
uint8_t readCalibByte(uint8_t index)
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
void DacSetup()
{
	
	DACB.CH0GAINCAL = 0x08;				//readCalibByte(PRODSIGNATURES_DACB0GAINCAL);		//Läs in kalibrering från minne
	DACB.CH0OFFSETCAL = 0x06;			//readCalibByte(PRODSIGNATURES_DACB0OFFCAL);
	
	DACB.CTRLA = DAC_CH0EN_bm | DAC_ENABLE_bm;						//Kanal 0 och enable
	DACB.CTRLB = DAC_CHSEL_SINGLE_gc;								//single mode
	
	DACB.CTRLC = DAC_REFSEL_AVCC_gc;								//För 3.3 V
	
	DACB.CH0DATA = 0;					// CHnDATA = VDACn*4095/Vref	
}
void AdcSetupEC()
{
	//ADCB.CALH = 0x00; //"kalibreringsvärde från fabrik"
	//ADCB.CALL = 0x00
	//ADC-setup, Clk_adc använder CLK_per
	//ADCB för EC
	ADCB.CTRLA		=		ADC_ENABLE_bm;
	ADCB.PRESCALER	=		ADC_PRESCALER_DIV4_gc;  //F_adc = 125000 Hz
	ADCB.CTRLB		=		ADC_RESOLUTION_12BIT_gc;	//12-bit, right adjusted
	ADCB.REFCTRL	=		ADC_REFSEL_INTVCC_gc;		// << 4;		// Vcc / 1.6
	
	ADCB.CH0.MUXCTRL = ADC_CH_MUXPOS_PIN0_gc;	//PORTB PIN0 ECS-vout1
	ADCB.CH1.MUXCTRL = ADC_CH_MUXPOS_PIN1_gc;	//PORTB PIN1 ECS-vout2
}
data_t GetData(float sample1, float sample2)
{
		data_t data;
		
		data.konduktivitet =1.05* (float)sample2/(float)sample1; 
		data.resistans = 1000* (float)sample1/(float)sample2;
		data.voltage1 = GetVoltage(sample1);
		data.voltage2 = GetVoltage(sample2);
		return data;	
}
uint16_t SampleVoltage1()
{
	ADCB.CH0.CTRL = ADC_CH_START_bm | ADC_CH_GAIN_1X_gc | ADC_CH_INPUTMODE_SINGLEENDED_gc;
	while(ADCB.CTRLA & ADC_CH0START_bm);
	while(!(ADCB.CH0.INTFLAGS & ADC_CH_CHIF_bm));
	ADCB.INTFLAGS = ADC_CH0IF_bm;
	return ADCB.CH0RES;
}
uint16_t SampleVoltage2()
{
	ADCB.CH1.CTRL = ADC_CH_START_bm | ADC_CH_GAIN_1X_gc | ADC_CH_INPUTMODE_SINGLEENDED_gc;
	while(ADCB.CTRLA & ADC_CH1START_bm);
	while(!(ADCB.CH1.INTFLAGS & ADC_CH_CHIF_bm));
	ADCB.INTFLAGS = ADC_CH1IF_bm;
	return ADCB.CH1RES;
}
void LowRider()
{
	_delay_ms(blink_freq);
	PORTE.OUT = 0b01111111;
	_delay_ms(blink_freq);
	PORTE.OUT = 0b10111111;
	_delay_ms(blink_freq);
	PORTE.OUT = 0b11011111;
	_delay_ms(blink_freq);
	PORTE.OUT = 0b11101111;
	_delay_ms(blink_freq);
	PORTE.OUT = 0b11110111;
	_delay_ms(blink_freq);
	PORTE.OUT = 0b11111011;
	_delay_ms(blink_freq);
	PORTE.OUT = 0b11111101;
	_delay_ms(blink_freq);
	PORTE.OUT = 0b11111110;
}
void TimerSetup()
{
	//Timer0 Setup
	// Timer low -> +VEXC
	// Timer high -> -VEXC
	TCC0.CTRLA	=	TC_CLKSEL_DIV2_gc;
	TCC0.CTRLB	=	TC_WGMODE_FRQ_gc | TC0_CCBEN_bm;
	
	TCC0.INTCTRLB = TC_CCBINTLVL_HI_gc;
	
	TCC0.CCA	=	200-1;
	TCC0.CCB	=	200-1;
	//ECS-SWITCH: f_freq =  8M / (2 * CLKSEL_DIV (CCB + 1) )
	
	TCC0.CCC = 0xffff;
	TCC0.CCD = 0xffff;
}
float GetVoltage(float ecresult)
{
	float ec;
	ec = (ecresult*2.0625)/(4096);
	return ec;
}
void InterruptSetup()
{
	PMIC.CTRL = PMIC_HILVLEN_bm | PMIC_MEDLVLEN_bm | PMIC_LOLVLEN_bm;	//interrupt
}
void UsartSetup()
{
	USARTF0.BAUDCTRLA = (uint8_t)(BAUD_PRESCALLER);
	USARTF0.BAUDCTRLB = (uint8_t)(BAUD_PRESCALLER>>8);
	
	USARTF0.CTRLB	=  USART_RXEN_bm | USART_TXEN_bm;
	USARTF0.CTRLC	= USART_CMODE_ASYNCHRONOUS_gc | USART_PMODE_DISABLED_gc | USART_CHSIZE_8BIT_gc;
	
	PORTF.DIRCLR = PIN2_bm;  //RX
	PORTF.DIRSET = PIN3_bm;	//TX
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