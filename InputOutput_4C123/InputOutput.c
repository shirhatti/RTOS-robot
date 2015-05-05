#include "tm4c123gh6pm.h"
#include "ST7735.h"
#include "ADC.h"
#include "can0.h"
#include "PWM.h"
#include "PLL.h"
#include "InputCapture.h"

uint32_t IR_L, IR_R;
uint32_t Ping1 = 4000000, Ping2= 9900000, Ping3=9900000, Ping4;
static uint32_t ROutput, LOutput;
int i;

#define PB6  (*((volatile unsigned long *)0x40005100))
#define PB7  (*((volatile unsigned long *)0x40004200))
#define PA0	 (*((volatile unsigned long *)0x40004042))

#define NVIC_EN2_INT92	0x10000000	// Interrupt 92 enable

void DisableInterrupts(void); // Disable interrupts
void EnableInterrupts(void);  // Enable interrupts
long StartCritical (void);    // previous I bit, disable interrupts
void EndCritical(long sr);    // restore I bit to previous value
void WaitForInterrupt(void);  // low power mode

volatile uint32_t Count;      // incremented on interrupt
uint32_t pingTime;
uint32_t TimeDiff, FirstTime;
uint32_t jitter;

void UserTask2(void){
//  GPIO_PORTF_DATA_R = GPIO_PORTF_DATA_R^0x04; // toggle PF2
  Count = Count + 1;
}

static int counter;
void Init_Timer5A(uint32_t period) {
	long sr;
	volatile unsigned long delay;
	
	sr = StartCritical();
	counter = 0;
  SYSCTL_RCGCTIMER_R |= 0x20;
	
  delay = SYSCTL_RCGCTIMER_R;
	delay = SYSCTL_RCGCTIMER_R;
	
  TIMER5_CTL_R &= ~TIMER_CTL_TAEN; // 1) disable timer1A during setup
                                   // 2) configure for 32-bit timer mode
  TIMER5_CFG_R = TIMER_CFG_32_BIT_TIMER;
                                   // 3) configure for periodic mode, default down-count settings
  TIMER5_TAMR_R = TIMER_TAMR_TAMR_PERIOD;
  TIMER5_TAILR_R = period - 1;     // 4) reload value
                                   // 5) clear timer1A timeout flag
  TIMER5_ICR_R = TIMER_ICR_TATOCINT;
  TIMER5_IMR_R |= TIMER_IMR_TATOIM;// 6) arm timeout interrupt
								   // 7) priority shifted to bits 31-29 for timer2A
  NVIC_PRI23_R = (NVIC_PRI23_R&0xFFFFFF00)|(1 << 5);	
  NVIC_EN2_R = NVIC_EN2_INT92;     // 8) enable interrupt 23 in NVIC
  TIMER5_TAPR_R = 0;
  TIMER5_CTL_R |= TIMER_CTL_TAEN;  // 9) enable timer2A
	//page 155
	//page 104 //interrupt number 92 = priority 23
  EndCritical(sr);
}

void Timer5A_Handler(void){ 
	unsigned long sr;
	
	TIMER5_ICR_R = TIMER_ICR_TATOCINT;// acknowledge timer2A timeout
	
	if(counter == 50)
	{
		counter = 1;
		
		sr = StartCritical();
//		GPIO_PORTB_AFSEL_R &= ~0x40; // regular port function
//		GPIO_PORTB_DIR_R |= 0x40;    // make PD3-0 out
//		GPIO_PORTB_PCTL_R = (GPIO_PORTB_PCTL_R&0xF0FFFFFF)+0x00000000;
//		PB7 = 0x00;
//		PB7 = 0x80;
//		Timer4A_Wait(800);	// 10 us
//		PB7 = 0x00;
		PA0 = 0x00;
		PA0 = 0x40;
		Timer4A_Wait(800);	// 10 us
		PA0 = 0x00;		
//		GPIO_PORTB_DIR_R &= ~0x40;       // make PB6 in
//		GPIO_PORTB_AFSEL_R |= 0x40;      // enable alt funct on PB6
//		GPIO_PORTB_PCTL_R = (GPIO_PORTB_PCTL_R&0xF0FFFFFF)+0x07000000;
		
		EndCritical(sr);
	}
	else {
		counter++;
	}
}

void Switch_Init(void){  
	unsigned long volatile delay;
  delay = SYSCTL_RCGC2_R;
  GPIO_PORTA_DIR_R &= ~0x0C;    // (c) make PA4,0 in (built-in button)
  GPIO_PORTA_AFSEL_R &= ~0x0C;  //     disable alt funct on PA4,0
  GPIO_PORTA_DEN_R |= 0x0C;     //     enable digital I/O on PA4,0
//  GPIO_PORTF_PCTL_R &= ~0x0000000F; //  configure PA4,0 as GPIO
  GPIO_PORTA_AMSEL_R &= ~0x0C;  //     disable analog functionality on PA4,0
  GPIO_PORTA_PUR_R |= 0x0C;     //     enable weak pull-up on PF4,0
  GPIO_PORTA_IS_R &= ~0x0C;     // (d) PA4,PA0 is edge-sensitive
  GPIO_PORTA_IBE_R &= ~0x0C;    //     PA4,PA0 is not both edges
  GPIO_PORTA_IEV_R &= ~0x0C;    //     PA4,PA0 falling edge event
  GPIO_PORTA_ICR_R = 0x0C;      // (e) clear flags 4,0
  GPIO_PORTA_IM_R |= 0x0C;      // (f) arm interrupt on PA4,PF0
  NVIC_PRI0_R = (NVIC_PRI0_R&0xFFFFFF00)|0x00000040; // (g) priority 2
  NVIC_EN0_R = 0x00000001;      // (h) enable interrupt 30 in NVIC
}

uint8_t switchPressed[] = {0,0,0,1};
void GPIOPortA_Handler(void){ // called on touch of either SW1 or SW2
  if(GPIO_PORTA_RIS_R&0x04){  // SW2 touch
    GPIO_PORTA_ICR_R = 0x04;  // acknowledge flag0
//		CAN0_SendData(switchPressed, 5);
  }
  if(GPIO_PORTA_RIS_R&0x08){  // SW1 touch
    GPIO_PORTA_ICR_R = 0x08;  // acknowledge flag4
//		CAN0_SendData(switchPressed, 6);
  }
}

void getSensorValues(void) {
//	Ping1 = Mailbox/100000;
//	Ping2 = Mailbox2/100000;
//	Ping3 = Mailbox3/100000;
//	Ping4 = Mailbox4/100000;
//	IR_R = Mailbox5&0xFFF;
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
	volatile uint32_t delay;
	int i;
	
	PLL_Init();
	Output_Init();
//	CAN0_Open();
	
	DisableInterrupts();
	
	SYSCTL_RCGCGPIO_R |= 0x20;
  while((SYSCTL_PRGPIO_R&0x0020) == 0){};// ready?
  Count = 0;                       // allow time to finish activating
//  GPIO_PORTF_DIR_R |= 0x04;        // make PF2 out (built-in LED)
//  GPIO_PORTF_AFSEL_R &= ~0x04;     // disable alt funct on PF2
//  GPIO_PORTF_DEN_R |= 0x04;        // enable digital I/O on PF2
//                                   // configure PF2 as GPIO
//  GPIO_PORTF_PCTL_R = (GPIO_PORTF_PCTL_R&0xFFFFF0FF)+0x00000000;
//  GPIO_PORTF_AMSEL_R = 0;          // disable analog functionality on PF
	
	GPIO_PORTF_LOCK_R = 0x4C4F434B;   // 2) unlock GPIO Port F
	GPIO_PORTF_CR_R = 0x1F;           // allow changes to PF4-0
	GPIO_PORTF_AMSEL_R = 0x00;        // 3) disable analog on
    
	GPIO_PORTF_DIR_R &= ~0x03;       // make PB6 in
  GPIO_PORTF_AFSEL_R |= 0x03;      // enable alt funct on PB6
  GPIO_PORTF_DEN_R |= 0x03;        // enable digital I/O on PB6
                                   // configure PB6 as T0CCP0
  GPIO_PORTF_PCTL_R = (GPIO_PORTF_PCTL_R&0xFFFFFF00)+0x00000077;
  GPIO_PORTF_AMSEL_R &= ~0xFF;     // disable analog functionality on PB6
		
	SYSCTL_RCGCGPIO_R  |= 0x01;
	delay = SYSCTL_RCGCGPIO_R;      // 2) allow time for clock to stabilize
  delay = SYSCTL_RCGCGPIO_R;
	GPIO_PORTA_DIR_R |= 0x01;  			// 3.11) make PA6 output
  GPIO_PORTA_AFSEL_R &= ~0x01; 		// 4.11) disable alternate function on PA6
  GPIO_PORTA_DEN_R |= 0x01;  			// 5.11) enable digital I/O on PA6
  GPIO_PORTA_AMSEL_R = 0; 				// 6.11) disable analog functionality on PA6	
//	Switch_Init();
	
	Init_Timer4A();
	Timer4A_Wait(80000000); //wait 1 sec
	Init_Timer5A(80000);
	ADC0_InitTimer3ATriggerSeq3PD3(6000);
	ADC0_InitTimer3BTriggerSeq2PD2(5500);
	
	TimerCapture_Init(UserTask2);
	InitMotors();
	
	EnableInterrupts();
	
	
	while(1) {
		getSensorValues();
		printSensorValues(0);
		if (Ping1 < 18) {
			ControlMotors(40000, 40000);
			ST7735_SetCursor(0,1);
			ST7735_OutString("Stopped");
			printSensorValues(3);
//			while(1) {
				ST7735_SetCursor(0,1);ST7735_OutString("Stopped");
				getSensorValues();
				printSensorValues(0);
//			}
			if(IR_L > IR_R) {
				ControlMotors(35000,35000);
				ST7735_SetCursor(0,8);ST7735_OutString("BL");
			}
			else {
				ControlMotors(35000,35000);
				ST7735_SetCursor(0,8);ST7735_OutString("BR");
			}
			for(i = 0; i < 1440000; i++);
		}
		else if (((Ping2 > 80) != (Ping3 > 80)) && (Ping1 < 40)) {
			if (Ping2>80) {
				//spot turn right
				ControlMotors(20000, 60000);
			}
			if (Ping3>80) {
				//spot turn left
				ControlMotors(60000, 20000); 
			}
		//	ControlMotors(40000,40000);
		}
		else if(IR_L < 1500 && IR_R < 1500) {
			ControlMotors(70000,70000);
			ST7735_SetCursor(0,8);ST7735_OutString("FO");
		}
		else if(IR_L < IR_R) {
			ControlMotors(75000,50000);
			ST7735_SetCursor(0,8);ST7735_OutString("LE");
		}
		else if(IR_R < IR_L) {
			ControlMotors(50000,75000);
			ST7735_SetCursor(0,8);ST7735_OutString("RI");
		}
	}
}
