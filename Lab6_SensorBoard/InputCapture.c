// InputCapture.c
// Runs on LM4F120/TM4C123
// Use Timer0A in edge time mode to request interrupts on the rising
// edge of PB0 (CCP0), and count the pulses.
// Daniel Valvano
// September 11, 2013

/* This example accompanies the book
   "Embedded Systems: Real Time Interfacing to Arm Cortex M Microcontrollers",
   ISBN: 978-1463590154, Jonathan Valvano, copyright (c) 2013
   Example 6.1, Program 6.1

 Copyright 2013 by Jonathan W. Valvano, valvano@mail.utexas.edu
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

// external signal connected to PB6 (T0CCP0) (trigger on rising edge)
#include <stdint.h>
#include "inc/tm4c123gh6pm.h"
#include "InputCapture.h"
#include "can0.h"

uint8_t sendArray[4];
uint8_t sendArray1[4];
uint8_t sendArray2[4];
uint8_t sendArray3[4];
uint8_t sendArray4[4];
uint8_t sendArray5[4];
uint8_t sendArray6[4];
uint8_t sendArray7[4];

#define NVIC_EN0_INT19          0x00080000  // Interrupt 19 enable
#define NVIC_EN1_INT35					0x00000008	// Interrupt 35 enable
#define NVIC_EN1_INT36					0x00000010	// Interrupt 36 enable

#define TIMER_CFG_16_BIT        0x00000004  // 16-bit timer configuration,
                                            // function is controlled by bits
                                            // 1:0 of GPTMTAMR and GPTMTBMR
#define TIMER_TAMR_TACMR        0x00000004  // GPTM TimerA Capture Mode
#define TIMER_TAMR_TAMR_CAP     0x00000003  // Capture mode
#define TIMER_TAMR_TASNAPS			0x00000080	// Snapshot at time of Capture Event
#define TIMER_TAMR_TACDIR				0x00000010	// Count Up
#define TIMER_CTL_TAEN          0x00000001  // GPTM TimerA Enable
#define TIMER_CTL_TAEVENT_POS   0x00000000  // Positive edge
#define TIMER_IMR_CAEIM         0x00000004  // GPTM CaptureA Event Interrupt
                                            // Mask
#define TIMER_ICR_CAECINT       0x00000004  // GPTM CaptureA Event Interrupt
                                            // Clear
#define TIMER_TAILR_M           0xFFFFFFFF  // GPTM Timer A Interval Load
                                            // Register
																						
#define NVIC_EN2_INT70					0x00000040	
#define TIMER_CFG_32_BIT_TIMER  0x00000000  // 32-bit timer configuration
#define TIMER_TAMR_TACDIR       0x00000010  // GPTM Timer A Count Direction
#define TIMER_TAMR_TAMR_PERIOD  0x00000002  // Periodic Timer mode
#define TIMER_IMR_TATOIM        0x00000001  // GPTM TimerA Time-Out Interrupt
                                            // Mask
#define TIMER_ICR_TATOCINT      0x00000001  // GPTM TimerA Time-Out Raw
                                            // Interrupt

void DisableInterrupts(void); // Disable interrupts
void EnableInterrupts(void);  // Enable interrupts
long StartCritical (void);    // previous I bit, disable interrupts
void EndCritical(long sr);    // restore I bit to previous value
void WaitForInterrupt(void);  // low power mode
void (*PeriodicTask)(void);   // user function

uint32_t Flag, Flag1, Flag2, Flag3, Flag4, Flag5, Flag6, Flag7;
uint32_t Per, Per1, Per2, Per3, Per4, Per5, Per6, Per7;
uint32_t Period, Period1, Period2, Period3, Period4, Period5, Period6, Period7;              // (1/clock) units
uint32_t First, First1, First2, First3, First4, First5, First6, First7;               // Timer0A first edge
int32_t Done, Done1, Done2, Done3, Done4, Done5, Done6, Done7;                // set each rising
//------------TimerCapture_Init------------
// Initialize Timer0A in edge time mode to request interrupts on
// the rising edge of PB0 (CCP0).  The interrupt service routine
// acknowledges the interrupt and calls a user function.
// Input: task is a pointer to a user function
// Output: none
void TimerCapture_Init(void(*task)(void)){long sr;
  sr = StartCritical();
	Flag = Flag1 = Flag2 = Flag3 = Flag4 = Flag5 = Flag6 = Flag7 = 0;
	
  SYSCTL_RCGCTIMER_R |= 0x0F;// activate timer0,1,2,3
  SYSCTL_RCGCGPIO_R |= 0x02; // activate port B
  while((SYSCTL_PRGPIO_R&0x0002) == 0){};// ready?

  PeriodicTask = task;             // user function 
  GPIO_PORTB_DIR_R &= ~0xFF;       // make PB6 in
  GPIO_PORTB_AFSEL_R |= 0xFF;      // enable alt funct on PB6
  GPIO_PORTB_DEN_R |= 0xFF;        // enable digital I/O on PB6
                                   // configure PB6 as T0CCP0
  GPIO_PORTB_PCTL_R = (GPIO_PORTB_PCTL_R&0x00000000)+0x77777777;
  GPIO_PORTB_AMSEL_R &= ~0xFF;     // disable analog functionality on PB6

// Timer0 Configuration		
  TIMER0_CTL_R &= ~(TIMER_CTL_TAEN|TIMER_CTL_TBEN); // disable timer0A during setup
  TIMER0_CFG_R = TIMER_CFG_16_BIT; // configure for 16-bit timer mode
                                   // configure for capture mode, default down-count settings
  TIMER0_TAMR_R = (TIMER_TAMR_TACMR|TIMER_TAMR_TAMR_CAP);
                                   // configure for falling edge event
	TIMER0_TBMR_R = (TIMER_TBMR_TBCMR|TIMER_TBMR_TBMR_CAP);
  TIMER0_CTL_R &= 0;
	TIMER0_CTL_R |= (TIMER_CTL_TAEVENT_BOTH|TIMER_CTL_TBEVENT_BOTH);
  TIMER0_TAILR_R = 0x0000FFFF;  // max start value
	TIMER0_TBILR_R = 0x0000FFFF;
	TIMER0_TAPR_R = 0xFF;							 // 23Prescale to max of 19.66 ms (echo 750 us + max 18.5 ms) 
	TIMER0_TBPR_R = 0xFF;
  TIMER0_IMR_R |= (TIMER_IMR_CAEIM|TIMER_IMR_CBEIM); // enable capture match interrupt
  TIMER0_ICR_R = (TIMER_ICR_CAECINT|TIMER_ICR_CBECINT);// clear timer0A capture match flag
  TIMER0_CTL_R |= (TIMER_CTL_TAEN|TIMER_CTL_TBEN);  // enable timer0A 16-b, +edge timing, interrupts
                                   // Timer0A=priority 2
  NVIC_PRI4_R = (NVIC_PRI4_R&0x00FFFFFF)|0x40000000; // top 3 bits
  NVIC_EN0_R |= NVIC_EN0_INT19;     // enable interrupt 19 in NVIC
	
	NVIC_PRI5_R = (NVIC_PRI5_R&0xFFFFFF00)|0x00000040;
	NVIC_EN0_R |= NVIC_EN0_INT20;

// Timer1 Configuration
  TIMER1_CTL_R &= ~(TIMER_CTL_TAEN|TIMER_CTL_TBEN); // disable timer0A during setup
  TIMER1_CFG_R = TIMER_CFG_16_BIT; // configure for 16-bit timer mode
                                   // configure for capture mode, default down-count settings
  TIMER1_TAMR_R = (TIMER_TAMR_TACMR|TIMER_TAMR_TAMR_CAP);
                                   // configure for falling edge event
	TIMER1_TBMR_R = (TIMER_TBMR_TBCMR|TIMER_TBMR_TBMR_CAP);
  TIMER1_CTL_R &= 0;
	TIMER1_CTL_R |= (TIMER_CTL_TAEVENT_BOTH|TIMER_CTL_TBEVENT_BOTH);
  TIMER1_TAILR_R = 0x0000FFFF;  // max start value
	TIMER1_TBILR_R = 0x0000FFFF;
	TIMER1_TAPR_R = 0xFF;							 // 23Prescale to max of 19.66 ms (echo 750 us + max 18.5 ms) 
	TIMER1_TBPR_R = 0xFF;
  TIMER1_IMR_R |= (TIMER_IMR_CAEIM|TIMER_IMR_CBEIM); // enable capture match interrupt
  TIMER1_ICR_R = (TIMER_ICR_CAECINT|TIMER_ICR_CBECINT);// clear timer0A capture match flag
  TIMER1_CTL_R |= (TIMER_CTL_TAEN|TIMER_CTL_TBEN);  // enable timer0A 16-b, +edge timing, interrupts
                                   // Timer0A=priority 2
  NVIC_PRI5_R = (NVIC_PRI5_R&0xFFFF00FF)|0x00004000; // top 3 bits
  NVIC_EN0_R |= NVIC_EN0_INT21;     // enable interrupt 21 in NVIC
	
	NVIC_PRI5_R = (NVIC_PRI5_R&0xFF00FFFF)|0x00400000;
	NVIC_EN0_R |= NVIC_EN0_INT22;

// Timer2 Configuration
  TIMER2_CTL_R &= ~(TIMER_CTL_TAEN|TIMER_CTL_TBEN); // disable timer0A during setup
  TIMER2_CFG_R = TIMER_CFG_16_BIT; // configure for 16-bit timer mode
                                  // configure for capture mode, default down-count settings
  TIMER2_TAMR_R = (TIMER_TAMR_TACMR|TIMER_TAMR_TAMR_CAP);
                                   // configure for falling edge event
	TIMER2_TBMR_R = (TIMER_TBMR_TBCMR|TIMER_TBMR_TBMR_CAP);
  TIMER2_CTL_R &= 0;
	TIMER2_CTL_R |= (TIMER_CTL_TAEVENT_BOTH|TIMER_CTL_TBEVENT_BOTH);
  TIMER2_TAILR_R = 0x0000FFFF;  // max start value
	TIMER2_TBILR_R = 0x0000FFFF;
	TIMER2_TAPR_R = 0xFF;							 // 23Prescale to max of 19.66 ms (echo 750 us + max 18.5 ms) 
	TIMER2_TBPR_R = 0xFF;
  TIMER2_IMR_R |= (TIMER_IMR_CAEIM|TIMER_IMR_CBEIM); // enable capture match interrupt
  TIMER2_ICR_R = (TIMER_ICR_CAECINT|TIMER_ICR_CBECINT);// clear timer0A capture match flag
  TIMER2_CTL_R |= (TIMER_CTL_TAEN|TIMER_CTL_TBEN);  // enable timer0A 16-b, +edge timing, interrupts
                                   // Timer0A=priority 2
  NVIC_PRI5_R = (NVIC_PRI5_R&0x00FFFFFF)|0x40000000; // top 3 bits
  NVIC_EN0_R |= NVIC_EN0_INT23;     // enable interrupt 21 in NVIC
	
	NVIC_PRI6_R = (NVIC_PRI6_R&0xFFFFFF00)|0x00000040;
	NVIC_EN0_R |= NVIC_EN0_INT24;
	
// Timer3 Configuration
  TIMER3_CTL_R &= ~(TIMER_CTL_TAEN|TIMER_CTL_TBEN); // disable timer0A during setup
  TIMER3_CFG_R = TIMER_CFG_16_BIT; // configure for 16-bit timer mode
                                  // configure for capture mode, default down-count settings
  TIMER3_TAMR_R = (TIMER_TAMR_TACMR|TIMER_TAMR_TAMR_CAP);
                                   // configure for falling edge event
	TIMER3_TBMR_R = (TIMER_TBMR_TBCMR|TIMER_TBMR_TBMR_CAP);
  TIMER3_CTL_R &= 0;
	TIMER3_CTL_R |= (TIMER_CTL_TAEVENT_BOTH|TIMER_CTL_TBEVENT_BOTH);
  TIMER3_TAILR_R = 0x0000FFFF;  // max start value
	TIMER3_TBILR_R = 0x0000FFFF;
	TIMER3_TAPR_R = 0xFF;							 // 23Prescale to max of 19.66 ms (echo 750 us + max 18.5 ms) 
	TIMER3_TBPR_R = 0xFF;
  TIMER3_IMR_R |= (TIMER_IMR_CAEIM|TIMER_IMR_CBEIM); // enable capture match interrupt
  TIMER3_ICR_R = (TIMER_ICR_CAECINT|TIMER_ICR_CBECINT);// clear timer0A capture match flag
  TIMER3_CTL_R |= (TIMER_CTL_TAEN|TIMER_CTL_TBEN);  // enable timer0A 16-b, +edge timing, interrupts
                                   // Timer0A=priority 2
  NVIC_PRI8_R = (NVIC_PRI8_R&0x00FFFFFF)|0x40000000; // top 3 bits
  NVIC_EN1_R |= NVIC_EN1_INT35;     // enable interrupt 21 in NVIC
	
	NVIC_PRI9_R = (NVIC_PRI9_R&0xFFFFFF00)|0x00000040; // top 3 bits
  NVIC_EN1_R |= NVIC_EN1_INT36;     // enable interrupt 21 in NVIC
	
  EndCritical(sr);
}

void Timer0A_Handler(void){
  TIMER0_ICR_R = TIMER_ICR_CAECINT;// acknowledge timer0A capture match
  Period = (First - TIMER0_TAR_R)&0xFFFFFF;// 24 bits, 12.5ns resolution
  First = TIMER0_TAR_R;            // setup for next
  Done = 1;
	
	if(Flag == 0) 
	{
		Flag = 1;
	}
	else
	{
		GPIO_PORTF_DATA_R = GPIO_PORTF_DATA_R^0x04; // toggle PF2
		Per = Period;
		Per = (Per * 332)/16;
		sendArray[0] = (Per&0xFF000000)>>24;
		sendArray[1] = (Per&0x00FF0000)>>16;
		sendArray[2] = (Per&0x0000FF00)>>8;
		sendArray[3] = (Per&0x000000FF);
		
		//Add a semaphore here
		CAN0_SendData(sendArray, 1);
		Flag = 0;
	}
}

void Timer0B_Handler(void){
  TIMER0_ICR_R = TIMER_ICR_CBECINT;// acknowledge timer0A capture match
  Period1 = (First1 - TIMER0_TBR_R)&0xFFFFFF;// 24 bits, 12.5ns resolution
  First1 = TIMER0_TBR_R;            // setup for next
  Done1 = 1;
	
	if(Flag1 == 0) 
	{
		Flag1 = 1;
	}
	else
	{
		GPIO_PORTF_DATA_R = GPIO_PORTF_DATA_R^0x04; // toggle PF2
		Per1 = Period1;
		Per1 = (Per1 * 332)/16;
		sendArray1[0] = (Per1&0xFF000000)>>24;
		sendArray1[1] = (Per1&0x00FF0000)>>16;
		sendArray1[2] = (Per1&0x0000FF00)>>8;
		sendArray1[3] = (Per1&0x000000FF);
		
		//Add a semaphore here
		CAN0_SendData(sendArray1, 2);
		Flag1 = 0;
	}
}

void Timer1A_Handler(void){
  TIMER1_ICR_R = TIMER_ICR_CAECINT;// acknowledge timer0A capture match
  Period2 = (First2 - TIMER1_TAR_R)&0xFFFFFF;// 24 bits, 12.5ns resolution
  First2 = TIMER1_TAR_R;            // setup for next
  Done2 = 1;
	
	if(Flag2 == 0) 
	{
		Flag2 = 1;
	}
	else
	{
		GPIO_PORTF_DATA_R = GPIO_PORTF_DATA_R^0x08; // toggle PF2
		Per2 = Period2;
		Per2 = (Per2 * 332)/16;
		sendArray2[0] = (Per2&0xFF000000)>>24;
		sendArray2[1] = (Per2&0x00FF0000)>>16;
		sendArray2[2] = (Per2&0x0000FF00)>>8;
		sendArray2[3] = (Per2&0x000000FF);
		
		//Add a semaphore here
		CAN0_SendData(sendArray2, 3);
		Flag2 = 0;
	}
}

void Timer1B_Handler(void){
  TIMER1_ICR_R = TIMER_ICR_CBECINT;// acknowledge timer0A capture match
  Period3 = (First3 - TIMER1_TBR_R)&0xFFFFFF;// 24 bits, 12.5ns resolution
  First3 = TIMER1_TBR_R;            // setup for next
  Done3 = 1;
	
	if(Flag3 == 0) 
	{
		Flag3 = 1;
	}
	else
	{
		GPIO_PORTF_DATA_R = GPIO_PORTF_DATA_R^0x04; // toggle PF2
		Per3 = Period3;
		Per3 = (Per3 * 332)/16;
		sendArray3[0] = (Per3&0xFF000000)>>24;
		sendArray3[1] = (Per3&0x00FF0000)>>16;
		sendArray3[2] = (Per3&0x0000FF00)>>8;
		sendArray3[3] = (Per3&0x000000FF);
		
		//Add a semaphore here
		CAN0_SendData(sendArray3, 4);
		Flag3 = 0;
	}
}

void Timer2A_Handler(void){
  TIMER2_ICR_R = TIMER_ICR_CAECINT;// acknowledge timer0A capture match
  Period4 = (First4 - TIMER2_TAR_R)&0xFFFFFF;// 24 bits, 12.5ns resolution
  First4 = TIMER2_TAR_R;            // setup for next
  Done4 = 1;
	
	if(Flag4 == 0) 
	{
		Flag4 = 1;
	}
	else
	{
		GPIO_PORTF_DATA_R = GPIO_PORTF_DATA_R^0x08; // toggle PF2
		Per4 = Period4;
		Per4 = (Per4 * 332)/16;
		sendArray4[0] = (Per4&0xFF000000)>>24;
		sendArray4[1] = (Per4&0x00FF0000)>>16;
		sendArray4[2] = (Per4&0x0000FF00)>>8;
		sendArray4[3] = (Per4&0x000000FF);
		
		//Add a semaphore here
		CAN0_SendData(sendArray4, 5);
		Flag4 = 0;
	}
}

void Timer2B_Handler(void){
  TIMER2_ICR_R = TIMER_ICR_CBECINT;// acknowledge timer0A capture match
  Period5 = (First5 - TIMER2_TBR_R)&0xFFFFFF;// 24 bits, 12.5ns resolution
  First5 = TIMER2_TBR_R;            // setup for next
  Done5 = 1;
	
	if(Flag5 == 0) 
	{
		Flag5 = 1;
	}
	else
	{
		GPIO_PORTF_DATA_R = GPIO_PORTF_DATA_R^0x04; // toggle PF2
		Per5 = Period5;
		Per5 = (Per5 * 332)/16;
		sendArray5[0] = (Per5&0xFF000000)>>24;
		sendArray5[1] = (Per5&0x00FF0000)>>16;
		sendArray5[2] = (Per5&0x0000FF00)>>8;
		sendArray5[3] = (Per5&0x000000FF);
		
		//Add a semaphore here
		CAN0_SendData(sendArray5, 6);
		Flag5 = 0;
	}
}

void Timer3A_Handler(void){
  TIMER3_ICR_R = TIMER_ICR_CAECINT;// acknowledge timer0A capture match
  Period6 = (First6 - TIMER3_TAR_R)&0xFFFFFF;// 24 bits, 12.5ns resolution
  First6 = TIMER3_TAR_R;            // setup for next
  Done6 = 1;
	
	if(Flag6 == 0) 
	{
		Flag6 = 1;
	}
	else
	{
		GPIO_PORTF_DATA_R = GPIO_PORTF_DATA_R^0x08; // toggle PF2
		Per6 = Period6;
		Per6 = (Per6 * 332)/16;
		sendArray6[0] = (Per6&0xFF000000)>>24;
		sendArray6[1] = (Per6&0x00FF0000)>>16;
		sendArray6[2] = (Per6&0x0000FF00)>>8;
		sendArray6[3] = (Per6&0x000000FF);
		
		//Add a semaphore here
		CAN0_SendData(sendArray6, 7);
		Flag6 = 0;
	}
}

void Timer3B_Handler(void){
  TIMER3_ICR_R = TIMER_ICR_CBECINT;// acknowledge timer0A capture match
  Period7 = (First7 - TIMER3_TBR_R)&0xFFFFFF;// 24 bits, 12.5ns resolution
  First7 = TIMER3_TBR_R;            // setup for next
  Done7 = 1;
	
	if(Flag7 == 0) 
	{
		Flag7 = 1;
	}
	else
	{
		GPIO_PORTF_DATA_R = GPIO_PORTF_DATA_R^0x04; // toggle PF2
		Per7 = Period3;
		Per7 = (Per7 * 332)/16;
		sendArray7[0] = (Per7&0xFF000000)>>24;
		sendArray7[1] = (Per7&0x00FF0000)>>16;
		sendArray7[2] = (Per7&0x0000FF00)>>8;
		sendArray7[3] = (Per7&0x000000FF);
		
		//Add a semaphore here
		CAN0_SendData(sendArray7, 8);
		Flag7 = 0;
	}
}

void Init_Timer4A(void) {
	long sr;
	volatile unsigned long delay;
	
	sr = StartCritical();
  SYSCTL_RCGCTIMER_R |= 0x10;
	
  delay = SYSCTL_RCGCTIMER_R;
	delay = SYSCTL_RCGCTIMER_R;
	
  TIMER4_CTL_R &= ~TIMER_CTL_TAEN; // 1) disable timer1A during setup
                                   // 2) configure for 32-bit timer mode
  TIMER4_CFG_R = TIMER_CFG_32_BIT_TIMER;
                                   // 3) configure for periodic mode, default down-count settings
  TIMER4_TAMR_R = TIMER_TAMR_TAMR_PERIOD;
  TIMER4_TAILR_R = 0xFFFFFFFF - 1;     // 4) reload value
                                   // 5) clear timer1A timeout flag
  TIMER4_ICR_R = TIMER_ICR_TATOCINT;
  TIMER4_IMR_R |= TIMER_IMR_TATOIM;// 6) arm timeout interrupt
								   // 7) priority shifted to bits 15-13 for timer1A
  NVIC_PRI17_R = (NVIC_PRI17_R&0xFF00FFFF)|(1 << 21);	//3
  NVIC_EN2_R = NVIC_EN2_INT70;     // 8) enable interrupt 21 in NVIC
  TIMER4_TAPR_R = 0;
  TIMER4_CTL_R |= TIMER_CTL_TAEN;  // 9) enable timer1A
	
  EndCritical(sr);
}

void Timer4A_Handler(void){ 
  TIMER4_ICR_R = TIMER_ICR_TATOCINT;// acknowledge timer1A timeout
}

// ******** OS_Time ************
// return the system time 
// Inputs:  none
// Outputs: time in 12.5ns units, 0 to 4294967295
// The time resolution should be less than or equal to 1us, and the precision 32 bits
// It is ok to change the resolution and precision of this function as long as 
//   this function and OS_TimeDifference have the same resolution and precision 
unsigned long OS_Time(void) { 
	return TIMER4_TAILR_R - TIMER4_TAV_R;
}

void Timer4A_Wait(unsigned long delay){
  volatile unsigned long elapsedTime;
  unsigned long startTime = TIMER4_TAV_R;
  do {
    elapsedTime = startTime-TIMER4_TAV_R;
  }
  while(elapsedTime <= delay);
}
