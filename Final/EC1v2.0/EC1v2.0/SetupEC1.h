/*
 * SetupEC1.h
 *
 * Created: 2020-05-26 08:51:16
 *  Author: vili2
 */ 
#define F_CPU 8000000UL

#ifndef SETUPEC1_H_
#define SETUPEC1_H_

typedef struct  
{
	float konduktivitet;
	float resistans;
	float voltage1;
	float voltage2;
}data_t;

void ClockSetup();
void EnableEC1();
void DisableEC1();
void PortSetupEC1();
void DacSetup();
void AdcSetupEC();
data_t GetData(float, float);
uint16_t SampleVoltage1();
uint16_t SampleVoltage2();
void LowRider();
void TimerSetup();
float GetVoltage(float ecresult);
void InterruptSetup();
void UsartSetup();
void UsartTx(unsigned char data);
void UsartRx(unsigned char data);
void UsartTxString(char* StringPtr);

#endif /* SETUPEC1_H_ */