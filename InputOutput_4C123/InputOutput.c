#include "tm4c123gh6pm.h"
#include "ST7735.h"
#include "ADC.h"
#include "can0.h"
#include "PWM.h"
#include "PLL.h"

uint32_t IR_L;
unsigned long Ping1, Ping2, Ping3, Ping4, IR_R;
static uint32_t ROutput, LOutput;
int i;


void getSensorValues(void) {
	Ping1 = Mailbox/100000;
	Ping2 = Mailbox2/100000;
	Ping3 = Mailbox3/100000;
	Ping4 = Mailbox4/100000;
	IR_R = Mailbox5&0xFFF;
}

void printSensorValues(int line) {
	ST7735_SetCursor(0,line);ST7735_OutUDec(Ping1);
	ST7735_OutString("   ");
	ST7735_SetCursor(4,line);ST7735_OutUDec(Ping2);
	ST7735_OutString("   ");
	ST7735_SetCursor(8,line);ST7735_OutUDec(Ping3);
	ST7735_OutString("   ");
	ST7735_SetCursor(0,line+5);ST7735_OutUDec(IR_L);
	ST7735_OutString("   ");
	ST7735_SetCursor(6,line+5);ST7735_OutUDec(IR_R);
	ST7735_OutString("   ");
}



int main(void){
	PLL_Init();
	Output_Init();
	CAN0_Open();
	ADC0_InitTimer3ATriggerSeq3PD3(5500);
	
	InitMotors();
	while(1) {
		getSensorValues();
		if (Ping1 < 10) {
			ControlMotors(40000, 40000);
			ST7735_SetCursor(0,1);
			ST7735_OutString("Stopped");
			printSensorValues(3);
			while(1) {
				ST7735_SetCursor(0,1);ST7735_OutString("Stopped");
				getSensorValues();
				printSensorValues(0);
			}
		}
//		else if(IR_L < 2000 && IR_R < 1500) {
//			ControlMotors(65000,70000);
//			ST7735_SetCursor(0,8);ST7735_OutString("FO");
//		}
//		else if(IR_L < IR_R) {
//			ControlMotors(70000,60000);
//			ST7735_SetCursor(0,8);ST7735_OutString("LE");
//		}
//		else if(IR_R < IR_L) {
//			ControlMotors(60000,70000);
//			ST7735_SetCursor(0,8);ST7735_OutString("RI");
//		}
	}
}
