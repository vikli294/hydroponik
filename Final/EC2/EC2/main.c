#define F_CPU 8000000UL

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include "SetupEC2.h"

ISR(PORTC_INT0_vect)
{
	cli();
	TimerStart();
	reti();	
}
ISR(PORTC_INT1_vect)
{
	cli();
	float EC;
	float tid;
	uint16_t cycles;
	char string[80];
	
	TimerStop();
	cycles = GetCycles();
			
	tid = Cycle2Time(cycles);			//klockcykler -> tid
	EC = Time2EC(tid);
			
	TimerReset();
	//reset timer
	sprintf(string, "EC: %f \r", EC);
	UsartTxString(string);
	sprintf(string, "Tid: %f \r", tid);
	UsartTxString(string);
	//_delay_ms(1000);
	reti();
}

int main(void)
{
	ClockSetup();
	PortSetupEC2();
	DisableEC2();
	TimerSwitchSetup();
	UsartSetup();
	DacSetup();	  
	InterruptSetup();
	sei();
	EnableEC2();
    while (1) 
    {
		if(PORTC.IN & PIN4_bm)
		{
			PORTE.OUTSET = 0xff;
		}
		if(PORTC.IN & PIN5_bm)
		{
			PORTE.OUTSET = 0xFF;
		}
		if(!(PORTC.IN & PIN4_bm))
		{
			PORTE.OUTCLR = 0xff;
		}
		if(!(PORTC.IN & PIN5_bm))
		{
			PORTE.OUTSET = 0xff;
		}
		//weeee
    }
}

