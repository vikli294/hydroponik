/*
 * rtd1.c
 *
 * Created: 2020-05-14 08:55:48
 * Author : Viktor
 */ 
#define F_CPU 8000000UL
#define blink_freq 1000

#include <avr/io.h>
#include <util/delay.h>
#include "setup_rtd.h"

uint16_t AdcSampleRtd1PT()
{
	uint16_t  sample;
	ADCA.REFCTRL		=	ADC_REFSEL_INT1V_gc;						//	 ADC_REFSEL_INTVCC_gc-- Vcc / 1.6
	ADCA.CTRLB			=	ADC_RESOLUTION_12BIT_gc | ADC_CONMODE_bm;	
	ADCA.CH1.CTRL		=	ADC_CH_START_bm | ADC_CH_GAIN_1X_gc | ADC_CH_INPUTMODE_DIFFWGAIN_gc;
	
	
	while(ADCA_CTRLA & ADC_CH1START_bm);			// Vänta på att AD är startad
	while(!(ADCA_CH1_INTFLAGS & ADC_CH_CHIF_bm));	//Vänta på AD omvandling kanal1
	
	ADCA.INTFLAGS = ADC_CH1IF_bm;					//Nollställ ADC-interuptflagga
	return sample = ADCA.CH1.RES;					
}
uint16_t AdcSampleRtd1Ref()
{
	uint16_t sample;
	ADCA.REFCTRL		= ADC_REFSEL_INTVCC_gc; //	 ADC_REFSEL_INTVCC_gc-- Vcc / 1.6
	ADCA.CH2.CTRL = ADC_CH_START_bm | ADC_CH_GAIN_1X_gc | ADC_CH_INPUTMODE_SINGLEENDED_gc;
	ADCA.CTRLB		= ADC_RESOLUTION_12BIT_gc; 
	
	while(ADCA_CTRLA & ADC_CH2START_bm);			// Vänta på att AD är startad
	while(!(ADCA_CH2_INTFLAGS & ADC_CH_CHIF_bm));	//Vänta på AD omvandling kanal2
	
	ADCA.INTFLAGS = ADC_CH2IF_bm;					//Nollställ ADC-interuptflagga
	return sample = ADCA.CH2.RES;
}

uint32_t meanPT()
{
	uint32_t sum = 0;
	for (uint32_t i = 0; i < 256; i++)
	{
		sum = sum + AdcSampleRtd1PT();
	}
	return sum/256 - 0x0002;// result - offset
}
uint32_t meanRef()
{
	uint32_t sum = 0;
	for (uint32_t i = 0; i < 256; i++)
	{
		sum = sum + AdcSampleRtd1Ref();
	}
	return sum/256  - 0x0014; // result - offset
}

float code2voltageDiff(uint16_t code)
{
		return ((float)code/(2048));
}
float code2voltageSingle(uint16_t code)
{
		return ((float)code/4096)*2.0625;
}

float ptRes()
{
	/* Rtd1Ref / 1.65k = I 
	ptRes = Rtd1PT / I;
	ptRes = (Rtd1PT*1.65k) / Rtd1Ref;
	*/
	float samplePT = code2voltageDiff(meanPT());
	float sampleRef = code2voltageSingle(meanRef());
	float res = (1650*samplePT)/sampleRef;
	
	return res;
}
float res2temp(float res)
{
	//return 2.56965*res - 256.678;					//t = A*res + B		A = 2.56965 degC/ohm; B = -256.678 degC/ohm; err@ 25degC = 0.302119
	return 2.57578*res - 257.631 - 0.6;
}
float resMed()
{
	float sum;
	for (uint32_t i = 0; i < 256; i++)
	{
		sum = sum + ptRes();
		_delay_ms(10);
	}
	return sum/256;
}
float getTemp()
{
	return res2temp(resMed());
}
float res2(uint16_t code)
{
	float res = 1650*((float)code/2048);
	return res;
}

int main(void)
{
	ClockSetup();
	PortSetup();
	RtdEnable();
	AdcSetup();
	UsartSetup();
	
	float temp = 5.69;
	float PtVoltage;
	float RefVoltage;
	float resistance2;
	
	char string[80];
	 
	
    while (1) 
    {
			RtdEnable();
			_delay_ms(1);
			
			temp = getTemp();
			
			
			PtVoltage = code2voltageDiff(meanPT());
			RefVoltage = code2voltageSingle(meanRef());
			resistance2 = res2(meanPT());
			
			RtdDisable();
			if( (temp < 100) & (temp > -5) )
			{
				/*sprintf(string, "PtV:%.6f \r", PtVoltage);
				UsartTxString(string);	
			
				sprintf(string, "RefV:%.6f \r", RefVoltage);
				UsartTxString(string);	
			*/
				sprintf(string, "Temp:%.1f \r", temp);
				UsartTxString(string);	
			/*
				sprintf(string, "---------\r\n");
				UsartTxString(string);
				sprintf(string, "%.1f \r", temp);
				UsartTxString(string);	
			*/
			}	
			_delay_ms(100);
    }
}
