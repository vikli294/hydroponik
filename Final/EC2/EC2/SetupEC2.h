#define F_CPU 8000000UL

#ifndef SETUPEC2_H_
#define SETUPEC2_H_

void ClockSetup();
void PortSetupEC2();
void EnableEC2();
void DisableEC2();
void TimerSetup();
void TimerStop();
void TimerStart();
uint16_t GetCycles();
float Cycle2Time(uint16_t cycles_t);
float Time2EC(float tid);
void DacSetup();
void UsartSetup();
void UsartTx(unsigned char data);
void UsartRx(unsigned char data);
void UsartTxString(char* StringPtr);
void InterruptSetup();




#endif /* SETUPEC2_H_ */