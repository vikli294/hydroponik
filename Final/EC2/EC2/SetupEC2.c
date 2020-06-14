#define F_CPU 8000000UL
#define BAUDRATE 9600UL
#define BAUD_PRESCALLER (((F_CPU / (BAUDRATE * 16UL))) - 1)
#define blink_freq 100
#define C16 0.0000025
#define R28 1008
#define EC_OFFSET 1

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

#include "SetupEC2.h"


void ClockSetup()
{
	//Ställer in extern klocka och prescalers.
	CCP			 =	CCP_IOREG_gc;
	OSC.XOSCCTRL = OSC_FRQRANGE_2TO9_gc | OSC_XOSCPWR_bm | OSC_XOSCSEL_XTAL_16KCLK_gc;		//Freqrange 2-9 MHz, Startuptime = 16K cycles
	
	CCP			 =	CCP_IOREG_gc;
	OSC.CTRL	=	OSC_XOSCEN_bm;
	while(!(OSC_STATUS & OSC_XOSCRDY_bm));
	
	CCP			=	CCP_IOREG_gc;				
	CLK.CTRL	=	CLK_SCLKSEL_XOSC_gc;		

	CCP			=	CCP_IOREG_gc;
	CLK.PSCTRL	=	CLK_PSADIV_1_gc | CLK_PSBCDIV_1_1_gc;	//Div: clk_per4 = 1 Div: CLKper2 =1, CLK_cpu = 1, CLK_per =1
	
	CCP			= CCP_IOREG_gc;
	OSC.XOSCFAIL = OSC_XOSCFDEN_bm;
}
void PortSetupEC2()
{
	//PortB konfig.	
	PORTB.DIRSET	=	PIN3_bm;								//dac
	PORTB.DIRSET	=	PIN2_bm;								//OUT:	Pin2 = ECS-DAC-1.2
	//PortC konfig.
	PORTC.DIRSET	=	PIN0_bm | PIN3_bm;						//OUT: Pin0=ECK-SWITCH, Pin3 = Turn off eck
	PORTC.DIRCLR	=	PIN4_bm | PIN5_bm;						//IN: Pin4 = ECK-MEASURE, Pin5 = ECK-TRIGG
	//PortE konfig.
	PORTE.DIRSET	=	0xff;									//OUT: Pin0..7 = LED-PE0..7
	
	//PortF
	PORTF.DIRSET	= PIN0_bm;
}
void EnableEC2()
{
	DACB.CH1DATA = 824;								//DAC på
	PORTE.OUTSET = 0xff;							//LED på
	PORTC.OUTCLR = PIN3_bm;
	PORTF.OUTSET = PIN0_bm;
}
void DisableEC2()
{
	DACB.CH1DATA = 0;								//DAC av
	PORTE.OUTCLR = 0xff;							//LED av
	PORTC.OUTSET = PIN3_bm;							//Sensor jordad.
	PORTF.OUTCLR = PIN0_bm;
}
void TimerSwitchSetup()
{
	//Timer0 Setup
	TCC0.CTRLA	=	TC_CLKSEL_DIV2_gc;
	TCC0.CTRLB	=	TC_WGMODE_FRQ_gc | TC0_CCAEN_bm;		//PortC.PIN0 -> switch
	
	TCC0.CCA	=	60000-1;
	TCC0.CCB	=	60000-1;		
	//ECK-SWITCH: f_freq =  8M / (2 * CLKSEL_DIV (CCB + 1) )
	
	TCC0.CCC = 0xffff;
	TCC0.CCD = 0xffff;
	// Timer low -> +VEXC
	// Timer high -> -VEXC
}
void TimerStop()
{
	TCD0.CTRLA = TC_CLKSEL_OFF_gc;
	TCD0.CTRLB = TC_WGMODE_NORMAL_gc;
}
void TimerStart()
{
	TCD0.CTRLA = TC_CLKSEL_DIV2_gc;
	TCD0.CTRLB = TC_WGMODE_NORMAL_gc;

}
void TimerReset()
{
	TCD0.CTRLA = TC_CLKSEL_OFF_gc;
	TCD0.CNT = 0;
}
uint16_t GetCycles()
{
	return TCD0.CNT;
}
float Cycle2Time(uint16_t cycles_t)
{
	float tid = 0;
	return tid = cycles_t*0.00000025; 
}
float Time2EC(float tid)
{
	float ec;
	ec = 1/((tid/C16)-R28)*EC_OFFSET;
	return ec;
}
void DacSetup()
{	
	DACB.CH1GAINCAL = 0x85; //readCalibByte(PRODSIGNATURES_DACB0GAINCAL);		//Läs in kalibrering från minne
	DACB.CH1OFFSETCAL = 0x0C; //readCalibByte(PRODSIGNATURES_DACB0OFFCAL);
	
	DACB.CTRLA = DAC_CH1EN_bm | DAC_ENABLE_bm;						//Kanal 1 och enable
	DACB.CTRLB = DAC_CHSEL_SINGLE1_gc;								//single mode
	DACB.CTRLC = DAC_REFSEL_AVCC_gc;								//För 3.3 V
	DACB.CH1DATA = 812;								
	// CHnDATA = VDACn*4095/Vref
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
void InterruptSetup()
{
	PMIC.CTRL = PMIC_HILVLEN_bm | PMIC_MEDLVLEN_bm | PMIC_LOLVLEN_bm;	//interrupt
	
	PORTC.PIN4CTRL = PORT_ISC_RISING_gc;
	PORTC.PIN5CTRL = PORT_ISC_FALLING_gc;
	
	PORTC.INT0MASK =	PIN4_bm;
	PORTC.INT1MASK =	PIN5_bm; 
	PORTC.INTCTRL = PORT_INT0LVL_HI_gc | PORT_INT1LVL_MED_gc;
	sei();
}