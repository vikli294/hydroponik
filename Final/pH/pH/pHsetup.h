/*
 * pHsetup.h
 *
 * Created: 2020-05-28 12:52:12
 *  Author: vili2
 */ 


#ifndef PHSETUP_H_
#define PHSETUP_H_

#define F_CPU 8000000UL
#define BAUDRATE 9600UL
#define BAUD_PRESCALLER (((F_CPU / (BAUDRATE * 16UL))) - 1)
#define blink_freq 100

#include <util/delay.h>

void ClockSetup();
void PortSetup();
void AdcSetup();
uint16_t AdcSamplePH();
float GetVoltage(uint16_t sample);
uint16_t AdcMedPH();
float AdcConvPH(uint16_t phresult_med);
float  GetPh();
void UsartSetup();
void UsartTx(unsigned char data);
void UsartRx(unsigned char data);
void UsartTxString(char* StringPtr);


#endif /* PHSETUP_H_ */