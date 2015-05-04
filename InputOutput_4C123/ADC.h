//Software-triggered one sample initialization



int ADC_Open(unsigned int channelNum);

unsigned short ADC_In(void);

//Timer-triggered
//int ADC_Collect(unsigned int channelNum, unsigned int fs,unsigned short buffer[], unsigned int numberOfSamples); 
void ADC_Collect(uint8_t channelNum, uint32_t fs, void(*task)(unsigned long));
int ADC_Status(void);
