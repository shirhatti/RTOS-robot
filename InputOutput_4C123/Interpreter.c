// Interpreter.c
// Runs on LM4F120/TM4C123
// Tests the UART0 to implement bidirectional data transfer to and from a
// computer running HyperTerminal.  This time, interrupts and FIFOs
// are used.
// Daniel Valvano
// September 12, 2013
// Modified by Kenneth Lee, Dalton Altstaetter 2/9/2015

/* This example accompanies the book
   "Embedded Systems: Real Time Interfacing to Arm Cortex M Microcontrollers",
   ISBN: 978-1463590154, Jonathan Valvano, copyright (c) 2014
   Program 5.11 Section 5.6, Program 3.10

 Copyright 2014 by Jonathan W. Valvano, valvano@mail.utexas.edu
    You may use, edit, run or distribute this file
    as long as the above copyright notice remains
 THIS SOFTWARE IS PROVIDED "AS IS".  NO WARRANTIES, WHETHER EXPRESS, IMPLIED
 OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES OF
 MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE.
 VALVANO SHALL NOT, IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL,
 OR CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
 For more information about my classes, my research, and my books, see
 http://users.ece.utexas.edu/~valvano/
 */

// U0Rx (VCP receive) connected to PA0
// U0Tx (VCP transmit) connected to PA1
#include <stdio.h>
#include <stdint.h>
#include "PLL.h"
#include "UART.h"
#include "ST7735.h"
#include "ADC.h"
#include <rt_misc.h>
#include <string.h>
#include "OS.h"
#include "ifdef.h"
#include "PWM.h"

uint32_t RightDuty = 20000;
uint32_t LeftDuty = 20000;

void Interpreter(void);
//---------------------OutCRLF---------------------
// Output a CR,LF to UART to go to a new line
// Input: none
// Output: none
void OutCRLF(void){
  UART_OutChar(CR);
  UART_OutChar(LF);
}

#define PE4  (*((volatile unsigned long *)0x40024040))
#ifdef INTERPRETER
extern void Producer(unsigned long data);
extern void Consumer(void);
void Interpreter(void){
	char input_str[30];
	int input_num,i,device,line;
	int freq, numSamples;
	UART_Init();              // initialize UART
	OutCRLF();
	OutCRLF();
	
	//Print Interpreter Menu
	printf("Debugging Interpreter Lab 1\n\r");
	printf("Commands:\n\r");
	printf("LCD - print a message to the LCD\n\r");
	printf("OS-K - Kill the Interpreter\n\r");
	#ifdef PROFILER
	printf("PROFILE - get profiling info for past events\n\r");
	#endif
	printf("RM+ - increase speed of right motor\n\r");
	printf("RM- - decrease speed of right motor\n\r");
	printf("RMF - right motor goes forward\n\r");
	printf("RMB - right motor goes backward\n\r");
  printf("LM+ - increase speed of left motor\n\r");
	printf("LM- - decrease speed of left motor\n\r");
	printf("LMF - left motor goes forward\n\r");
	printf("LMB - left motor goes backward\n\r");
	
	while(1){
		//PE4^=0x10;
		printf("\n\rEnter a command:\n\r");
		for(i=0;input_str[i]!=0;i++){input_str[i]=0;}		//Flush the input_str
		UART_InString(input_str,30);
		if(!strcmp(input_str,"LCD")){	
			printf("\n\rMessage to Print: ");
			for(i=0;input_str[i]!=0;i++){input_str[i]=0;}		//Flush the input_str
			UART_InString(input_str,30);
			printf("\n\rNumber to Print: ");
			input_num=UART_InUDec();
			printf("\n\rDevice to Print to: ");
			device = UART_InUDec();
			printf("\n\rLine to Print to: ");
			line = UART_InUDec();
			ST7735_Message(device,line,input_str,input_num);
		} else if(!strcmp(input_str,"OS-K")){
			OS_Kill();
			#ifdef PROFILER
		} else if(!strcmp(input_str,"PROFILE")){
				printf("\n\rThreadAddress\tThreadAction\tThreadTime\n\r");
				for(i=0; i<PROFSIZE; i++){
					printf("%lu\t\t%lu\t\t%lu\n\r",(unsigned long)ThreadArray[i],ThreadAction[i],ThreadTime[i]/80000);
				}
			#endif 
		} else if(!strcmp(input_str,"RM+")){
			RightDuty+=100;
			if(RightDuty>=4000) RightDuty = 4000;
			PWM0A_Duty(RightDuty);
		} else if(!strcmp(input_str,"RM-")){
			RightDuty-=100;
			if(RightDuty<=100) RightDuty = 100;
			PWM0A_Duty(RightDuty);
		} else if(!strcmp(input_str,"RMF")){
			RightMotorForward();
		} else if(!strcmp(input_str,"RMB")){
			RightMotorBackward();
		} else if(!strcmp(input_str,"LM+")){
			LeftDuty+=100;
			if(LeftDuty>=4000) LeftDuty = 4000;
			PWM0B_Duty(LeftDuty);
		} else if(!strcmp(input_str,"LM-")){
			LeftDuty-=100;
			if(LeftDuty<=100) LeftDuty = 100;
			PWM0B_Duty(LeftDuty);
		} else if(!strcmp(input_str,"LMF")){
			LeftMotorForward();
		} else if(!strcmp(input_str,"LMB")){
			LeftMotorBackward();
		} else{
			printf("\n\rInvalid Command. Try Again\n\r");
		}
	}
}
#endif

