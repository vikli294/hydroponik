/*
 * rtd2.c
 *
 * Created: 2020-05-25 11:19:41
 * Author : vili2
 */ 
#define F_CPU 8000000UL
#define blink_freq 100

#define BAUDRATE 9600UL
#define BAUD_PRESCALLER (((F_CPU / (BAUDRATE * 16UL))) - 1)

#include <util/delay.h>
#include <avr/io.h>

void ClockSetup()
{
	//Ställer in extern klocka och prescalers.
	CCP			=	CCP_IOREG_gc;
	OSC.XOSCCTRL = OSC_FRQRANGE_2TO9_gc | OSC_XOSCSEL_XTAL_16KCLK_gc;		//Freqrange 2-9 MHz, Startuptime = 16K cycles
	
	CCP			=	CCP_IOREG_gc;
	OSC.CTRL	=	OSC_XOSCEN_bm;
	while(!(OSC_STATUS & OSC_XOSCRDY_bm));
	
	CCP			=	CCP_IOREG_gc;				//Tillåt ändring av klockan. Configuration Change Protection av
	CLK.CTRL	=	CLK_SCLKSEL_XOSC_gc;		//Välj extern klocka

	CCP			=	CCP_IOREG_gc;
	CLK.PSCTRL	=	CLK_PSADIV_1_gc | CLK_PSBCDIV_1_1_gc;	//Div: clk_per4 = 1 Div: CLKper2 =1, CLK_cpu = 1, CLK_per =1
	
	CCP			=	CCP_IOREG_gc;
	CLK.LOCK	=	CLK_LOCK_bm;				//Lås klockan tills nästa reset
}
void PortSetup()
{
	PORTA.DIRCLR = PIN2_bm; // RTD-2 IN
	PORTD.DIRSET = PIN1_bm;	//Enable rtd
	
	PORTE.DIRSET = 0xff;
}
void EnableRtd()
{
	PORTD.OUTSET = PIN1_bm;
}
void DisableRtd()
{
	PORTD.OUTCLR = PIN1_bm;
}
void AdcSetup()
{
	//ADCA.CALL = readAdcCalibByte(PRODSIGNATURES_ADCACAL0);		//Läs in kalibrering från minne
	//ADCA.CALH = readAdcCalibByte(PRODSIGNATURES_ADCACAL1);
	
	//ADC-setup, Clk_adc använder CLK_per
	//ADCA temp

	ADCA.PRESCALER	= ADC_PRESCALER_DIV4_gc;
	ADCA.CTRLB		= ADC_RESOLUTION_12BIT_gc; //ADC-CONMODE_bm = 1
	ADCA.CTRLA		= ADC_ENABLE_bm;

	
	
	ADCA.CH3.CTRL		= ADC_CH_INPUTMODE_SINGLEENDED_gc;

	//ADCA.REFCTRL		= ADC_REFSEL_INTVCC_gc; //	 ADC_REFSEL_INTVCC_gc-- Vcc / 1.6
	ADCA.REFCTRL		= 0x01<<4; //	 ADC_REFSEL_INTVCC_gc-- Vcc / 1.6
	
	ADCA.CH3.MUXCTRL	=	ADC_CH_MUXPOS_PIN2_gc;		//RTD1-AIN1 = P; RTD2-AIN2 = N

	//ADCB.CAL = "kalibreringsvärde från fabrik"
	//The CALL and CALH register pair hold the 12-bit calibration value. The ADC pipeline is calibrated during production
	//programming, and the calibration value must be read from the signature row and written to the CAL register from
	//software.
}

uint16_t sample() 
{
	uint16_t data;
	ADCA.CH3.CTRL = ADC_CH_START_bm | ADC_CH_GAIN_1X_gc | ADC_CH_INPUTMODE_SINGLEENDED_gc;
	
	while(ADCA_CTRLA & ADC_CH3START_bm);			// Vänta på att AD är startad
	while(!(ADCA_CH3_INTFLAGS & ADC_CH_CHIF_bm));	//Vänta på AD omvandling kanal1
	
	ADCA.INTFLAGS = ADC_CH3IF_bm;					//Nollställ ADC-interuptflagga
	return data = ADCA.CH3.RES -  0x031; //0x00C2 jord offset
}
float code2voltage(uint16_t code)
{
	float voltage = ((float)code/4096) * 2.0625;
	return voltage;
}
float voltage2res(float voltage)
{
	float res = (voltage * 270.3)/(5-voltage) - 1; //-1 offset
	return res;
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
float res2temp(float res)
{
	//return ((res/100)-1 ) / 0.00385; // +- kalibrerings-offset DÅLIG METOD
	// T = (R/R0 – 1) / ?   where R0 = 100, and ? = 0.00385


	//return 2.56965*res - 256.678;	//t = A*res + B		A = 2.56965 degC/ohm; B = -256.678 degC/ohm; err@ 25degC = 0.302119
	return 2.57578*res - 257.631 + 0.2;
}
int main(void)
{
	ClockSetup();
	PortSetup();
	AdcSetup();
	UsartSetup();
	
	EnableRtd();
	//DisableRtd();
	
    float voltage;
	float resistance;
	float temp;
    char string[80];
	uint32_t data = 0;
	
	while (1) 
    {
		data = 0;
		LowRider();
		for (uint16_t i = 0; i < 512 ; i++)
		{
			data = data + sample();
			_delay_ms(10);
		} 
		data = data / 512;
		
		voltage = code2voltage(data);
		resistance = voltage2res(voltage);
		temp = res2temp(resistance);
		/*sprintf(string, "Voltage:%f \r", voltage);
		UsartTxString(string);	
		sprintf(string, "Resistance:%f \r", resistance);
		UsartTxString(string);
		*/	
		sprintf(string, "Temp:%.1f \r", temp);
		UsartTxString(string);
		/*		
		sprintf(string, "%.1f \r", temp);
		UsartTxString(string);	*/
		_delay_ms(880);
    }
}

