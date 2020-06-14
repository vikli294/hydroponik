/*
 * pH.c
 *
 * Created: 2020-05-13 11:17:10
 * Author : Viktor
 */ 
#define F_CPU 8000000UL

#include <avr/io.h>
#include <stdio.h>
#include <util/delay.h>
#include "pHsetup.h"


int main(void)
{
	ClockSetup();
	PortSetup();
	AdcSetup();
	UsartSetup();
	
	float ph = 7;
	float voltage = 1337;
	float sum;
	uint16_t sample1 = 0;
	char string[80];
	
    /* Replace with your application code */
    while (1) 
    {
		for(uint16_t i = 0 ; i < 128; i++)
		{
		ph = GetPh();
		_delay_ms(1);
		sum = sum + ph;
		}
		sum = sum/128;
		sample1 = AdcSamplePH();
		voltage = GetVoltage(sample1);
		/*
		sprintf(string, "%.1f \r", sum);
		UsartTxString(string);
		_delay_ms(1000);
		*/
		ph = 7.1;
		sprintf(string, "pH: %.1f \r", ph);
		UsartTxString(string);
		/*
		sprintf(string, "voltage: %.2f \r", voltage);
		UsartTxString(string);*/
		
		sum = 0;
		//_delay_ms(100);
    }
}

