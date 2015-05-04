#ifndef ADC_H
#define ADC_H
#include <stdint.h>

extern uint32_t IR_L;

void ADC0_InitTimer3ATriggerSeq3PD3(uint32_t period);

void ADC0_InitTimer3BTriggerSeq2PD2(uint32_t period);

#endif
