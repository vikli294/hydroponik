/*
 * setup_rtd.h
 *
 * Created: 2020-05-14 13:54:21
 *  Author: Viktor
 */ 


#ifndef SETUP_1_H_
#define SETUP_1_H_

void ClockSetup();
void RtdEnable();
void RtdDisable();
void PortSetup();
uint8_t readAdcCalibByte(uint8_t index);
void AdcSetup();
void UsartSetup();
void UsartTx(unsigned char data);
void UsartRx(unsigned char data);
void UsartTxString(char* StringPtr);


#endif /* SETUP_1_H_ */