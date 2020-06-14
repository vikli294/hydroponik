/*
 * EC1.c
 *
 * Created: 2020-05-26 08:50:42
 * Author : vili2
 */ 
#define F_CPU 8000000UL
#define ANTAL_SAMPLES 128

#include <avr/io.h>
#include <util/delay.h>
#include "SetupEC1.h"
#include <avr/interrupt.h>

ISR(TCC0_CCB_vect)
{
	cli();
	
	_delay_us(1);
	uint32_t sum1 = 0;
	uint32_t sum2 = 0;
	for(uint16_t i = 0; i < ANTAL_SAMPLES; i++)
	{
		sum1 += SampleVoltage1();
		sum2 += SampleVoltage2();
	}
	float voltage1Avg = sum1/ANTAL_SAMPLES;
	float voltage2Avg = sum2/ANTAL_SAMPLES;
	
	voltage1Avg = (voltage1Avg + 0x50) * 1; //(Resultat + Offset-kalibrering ) * Gain-Kallibrering;
	voltage2Avg = (voltage2Avg + 0xB0) * 1;
	
	if(voltage1Avg < 0.09 | voltage2Avg < 0.09) //Om sample tas i den negativa delen av perioden
	{
		reti();
	}
	char string[80];
	
	data_t data = GetData(voltage1Avg, voltage2Avg);
	
	// Skicka data
	
	sprintf(string, "V1: %f \r", data.voltage1);
	UsartTxString(string);
		
	sprintf(string, "V2: %f \r", data.voltage2);
	UsartTxString(string);
		
	sprintf(string, "Resistans: %f \r", data.resistans);
	UsartTxString(string);
		
	sprintf(string, "EC: %f \r", data.konduktivitet);
	UsartTxString(string);
	_delay_ms(500);
	reti();
}
ISR(PORTC_INT0_vect)
{
	cli();
	reti();
}

ISR(BADISR_vect)
{
	LowRider();
	reti();
}
int main(void)
{
	cli();
    ClockSetup();
	PortSetupEC1();
	AdcSetupEC();
	DacSetup();
	TimerSetup();
	UsartSetup();
	EnableEC1();
	InterruptSetup();
	sei();
	
    while (1) 
    {	
		//Vänta på interrupt
    }
}