// Timer.c

#include "TIMER.h"
#include "tm4c123gh6pm.h"


void(*HandlerTaskArray[12])(void); // Holds the function pointers to the threads that will be launched

volatile unsigned int* timerCtrlBuf[12] = {TIMER0_CTL_PTR_R, TIMER0_CTL_PTR_R, TIMER1_CTL_PTR_R, TIMER1_CTL_PTR_R,
																									TIMER2_CTL_PTR_R, TIMER2_CTL_PTR_R, TIMER3_CTL_PTR_R, TIMER3_CTL_PTR_R,
																									TIMER4_CTL_PTR_R, TIMER4_CTL_PTR_R, TIMER5_CTL_PTR_R, TIMER5_CTL_PTR_R};

static int usedTimers[12];
static int timerCount = -1;	


void (*PeriodicTask)(void);   // user function

// ***************** Timer3_Init ****************
// Activate Timer3 interrupts to run user task periodically
// Inputs:  task is a pointer to a user function
//          period in units (1/clockfreq)
// Outputs: none
void Timer3_Init(void(*task)(void), uint32_t period){
  SYSCTL_RCGCTIMER_R |= 0x08;   // 0) activate TIMER3
  PeriodicTask = task;          // user function
  TIMER3_CTL_R = 0x00000000;    // 1) disable TIMER3A during setup
  TIMER3_CFG_R = 0x00000000;    // 2) configure for 32-bit mode
  TIMER3_TAMR_R = 0x00000002;   // 3) configure for periodic mode, default down-count settings
  TIMER3_TAILR_R = period-1;    // 4) reload value
  TIMER3_TAPR_R = 0;            // 5) bus clock resolution
  TIMER3_ICR_R = 0x00000001;    // 6) clear TIMER3A timeout flag
  TIMER3_IMR_R = 0x00000001;    // 7) arm timeout interrupt
  NVIC_PRI8_R = (NVIC_PRI8_R&0x00FFFFFF)|0x80000000; // 8) priority 4
// interrupts enabled in the main program after all devices initialized
// vector number 51, interrupt number 35
  NVIC_EN1_R = 1<<(35-32);      // 9) enable IRQ 35 in NVIC
  TIMER3_CTL_R = 0x00000001;    // 10) enable TIMER3A
}

void Timer3A_Handler(void){
  TIMER3_ICR_R = TIMER_ICR_TATOCINT;// acknowledge TIMER3A timeout
  (*PeriodicTask)();                // execute user task
}
																									
																									
																									

void TIMER_ClearPeriodicTime(int timer)
{
	// when writing to the TAV or TBV register
	// it loads it into the TAR register on the next cycle
	// this has to be done bc the TAR is READ only
	// where as the TAV has READ/WRITE capabilities.
	switch(timer)
	{
		case 0: // TIMERA0
			TIMER0_CTL_R &= ~TIMER_CTL_TAEN; // disable TimerA0
			TIMER0_TAV_R = 0; // set Timer0A Value to 0
			TIMER0_CTL_R |= TIMER_CTL_TAEN; // enable TimerA0
			break;
		
		case 1: // TIMERB0
			TIMER0_CTL_R &= ~TIMER_CTL_TBEN; // disable TimerB0
			TIMER0_TBV_R = 0; // set Timer0B Value to 0
			TIMER0_CTL_R |= TIMER_CTL_TBEN; // enable TimerB0
			break;
		
		case 2: // TIMERA1
			TIMER1_CTL_R &= ~TIMER_CTL_TAEN; // disable TimerA1
			TIMER1_TAV_R = 0; // set Timer1A Value to 0
			TIMER1_CTL_R |= TIMER_CTL_TAEN; // enable TimerA1
			break;
		
		case 3:	// TIMERB1
			TIMER1_CTL_R &= ~TIMER_CTL_TBEN; // disable TimerB1
			TIMER1_TBV_R = 0; // set Timer1B Value to 0
			TIMER1_CTL_R |= TIMER_CTL_TBEN; // enable TimerB1
			break;
		
		case 4:	// TIMERA2
			TIMER2_CTL_R &= ~TIMER_CTL_TAEN; // disable TimerA2
			TIMER2_TAV_R = 0; // set Timer2A Value to 0
			TIMER2_CTL_R |= TIMER_CTL_TAEN; // enable TimerA2
			break;
		
		case 5:	// TIMERB2
			TIMER2_CTL_R &= ~TIMER_CTL_TBEN; // disable TimerB2
			TIMER2_TBV_R = 0; // set Timer2B Value to 0
			TIMER2_CTL_R |= TIMER_CTL_TBEN; // enable TimerB2
			break;
		
		case 6: // TIMERA3
			TIMER3_CTL_R &= ~TIMER_CTL_TAEN; // disable TimerA3
			TIMER3_TAV_R = 0; // set Timer3A Value to 0
			TIMER3_CTL_R |= TIMER_CTL_TAEN; // enable TimerA3
			break;
		
		case 7:	// TIMERB3
			TIMER3_CTL_R &= ~TIMER_CTL_TBEN; // disable TimerB3
			TIMER3_TBV_R = 0; // set Timer3B Value to 0
			TIMER3_CTL_R |= TIMER_CTL_TBEN; // enable TimerB3
			break;
		
		case 8: // TIMERA4
			TIMER4_CTL_R &= ~TIMER_CTL_TAEN; // disable TimerA4
			TIMER4_TAV_R = 0; // set Timer4A Value to 0
			TIMER4_CTL_R |= TIMER_CTL_TAEN; // enable TimerA4
			break;
		
		case 9: // TIMERB4
			TIMER4_CTL_R &= ~TIMER_CTL_TBEN; // disable TimerB4
			TIMER4_TBV_R = 0; // set TimerB4 Value to 0
			TIMER4_CTL_R |= TIMER_CTL_TBEN; // enable TimerB4
			break;
		
		case 10: // TIMERA5
			TIMER5_CTL_R &= ~TIMER_CTL_TAEN; // disable TimerA5
			TIMER5_TAV_R = 0; // set Timer5A Value to 0
			TIMER5_CTL_R |= TIMER_CTL_TAEN; // enable TimerA5
			break;
		
		case 11: // TIMERB5
			TIMER5_CTL_R &= ~TIMER_CTL_TBEN; // disable TimerB5
			TIMER5_TBV_R = 0; // set Timer5B Value to 0
			TIMER5_CTL_R |= TIMER_CTL_TBEN; // enable TimerB5
			break;
		
		default:
			break;
	}
}

// Returns the number of bus cycles in a full period
unsigned long TIMER_ReadTimerPeriod(int timer)
{
	unsigned long busCyclePerPeriod = 0;
	switch(timer)
	{
		case 0:
			busCyclePerPeriod = TIMER0_TAILR_R;
			break;
		
		case 1:
			busCyclePerPeriod = TIMER0_TBILR_R;
			break;
		
		case 2:
			busCyclePerPeriod = TIMER1_TAILR_R;
			break;
		
		case 3:
			busCyclePerPeriod = TIMER1_TBILR_R;
			break;
		
		case 4:
			busCyclePerPeriod = TIMER2_TAILR_R;
			break;
		
		case 5:
			busCyclePerPeriod = TIMER2_TBILR_R;
			break;
		
		case 6:
			busCyclePerPeriod = TIMER3_TAILR_R;
			break;
		
		case 7:
			busCyclePerPeriod = TIMER3_TBILR_R;
			break;
		
		case 8:
			busCyclePerPeriod = TIMER4_TAILR_R;
			break;
		
		case 9:
			busCyclePerPeriod = TIMER4_TBILR_R;
			break;
		
		case 10:
			busCyclePerPeriod = TIMER5_TAILR_R;
			break;
		
		case 11:
			busCyclePerPeriod = TIMER5_TBILR_R;
			break;
		
		default:
			break;
	}
	return busCyclePerPeriod;
}



// this configures the timers for 32-bit mode, periodic mode
int TIMER_TimerInit(void(*task)(void), int timer, unsigned long desiredFrequency, unsigned long priority)
{
	int delay;
	unsigned long cyclesPerPeriod;
	
	// will fail if frequency is a decimal number close to 0 relative to the bus speed
	cyclesPerPeriod = CLOCKSPEED_80MHZ/desiredFrequency; 
	
	switch(timer)
	{
		case 0:		//TimerA0
			SYSCTL_RCGCTIMER_R |= SYSCTL_RCGCTIMER_R0;   // activate timer0
			delay = SYSCTL_RCGCTIMER_R;   // allow time to finish activating
			TIMER0_CTL_R &= ~TIMER_CTL_TAEN; // disable TimerA0
			TIMER0_CFG_R  = TIMER_CFG_32_BIT_TIMER; // configure for 32-bit mode
			TIMER0_TAMR_R = TIMER_TAMR_TAMR_PERIOD;
			TIMER0_TAILR_R = cyclesPerPeriod-1;
			TIMER0_TAPR_R = 0; // set prescale = 0
			TIMER0_ICR_R = TIMER_ICR_TATOCINT; // clear timeout flag, friendly since writing a 0 does nothing
			TIMER0_IMR_R |= TIMER_IMR_TATOIM; // arm the timeout interrupt
			NVIC_PRI4_R = (NVIC_PRI4_R & ~NVIC_PRI4_INT19_M)|(priority << NVIC_PRI4_INT19_S); //clears PRI bits then shifts the mask into the appropriate place
			//TIMER0_CTL_R |= TIMER_CTL_TAEN; // enable TimerA0, do this in OS_Launch(.)
			HandlerTaskArray[0] = task; // fill function pointer array w/address of task
			break;
		
		case 1:  	//TimerB0
			SYSCTL_RCGCTIMER_R |= SYSCTL_RCGCTIMER_R0;   // activate timer0
			delay = SYSCTL_RCGCTIMER_R;   // allow time to finish activating
		  TIMER0_CTL_R &= ~TIMER_CTL_TBEN; // disable TimerB0
			TIMER0_CFG_R  = TIMER_CFG_32_BIT_TIMER; // configure for 32-bit mode
			TIMER0_TBMR_R = TIMER_TBMR_TBMR_PERIOD;
			TIMER0_TBILR_R = cyclesPerPeriod-1;
			TIMER0_TBPR_R = 0; // set prescale = 0
			TIMER0_ICR_R = TIMER_ICR_TBTOCINT; // clear timeout flag, friendly since writing a 0 does nothing
			TIMER0_IMR_R |= TIMER_IMR_TBTOIM; // arm the timeout interrupt
			NVIC_PRI5_R = (NVIC_PRI5_R & ~NVIC_PRI5_INT20_M)|(priority << NVIC_PRI5_INT20_S); //clears PRI bits then shifts the mask into the appropriate place
			//TIMER0_CTL_R |= TIMER_CTL_TBEN; // enable TimerB0, do this in OS_Launch(.)
			HandlerTaskArray[1] = task; // fill function pointer array w/address of task
			break;
		
		case 2:		//TimerA1
			SYSCTL_RCGCTIMER_R |= SYSCTL_RCGCTIMER_R1;   // activate timer1
			delay = SYSCTL_RCGCTIMER_R;   // allow time to finish activating
			TIMER1_CTL_R &= ~TIMER_CTL_TAEN; // disable TimerA1
			TIMER1_CFG_R  = TIMER_CFG_32_BIT_TIMER; // configure for 32-bit mode
			TIMER1_TAMR_R = TIMER_TAMR_TAMR_PERIOD;
			TIMER1_TAILR_R = cyclesPerPeriod-1;
			TIMER1_TAPR_R = 0; // set prescale = 0
			TIMER1_ICR_R = TIMER_ICR_TATOCINT; // clear timeout flag, friendly since writing a 0 does nothing
			TIMER1_IMR_R |= TIMER_IMR_TATOIM; // arm the timeout interrupt
			NVIC_PRI5_R = (NVIC_PRI5_R & ~NVIC_PRI5_INT21_M)|(priority << NVIC_PRI5_INT21_S); //clears PRI bits then shifts the mask into the appropriate place
			//TIMER1_CTL_R |= TIMER_CTL_TAEN; // enable TimerA1, do this in OS_Launch(.)
			HandlerTaskArray[2] = task; // fill function pointer array w/address of task
			break;
		
		case 3:		//TimerB1
			SYSCTL_RCGCTIMER_R |= SYSCTL_RCGCTIMER_R1;   // activate timer1
			delay = SYSCTL_RCGCTIMER_R;   // allow time to finish activating
			TIMER1_CTL_R &= ~TIMER_CTL_TBEN; // disable TimerB1
			TIMER1_CFG_R  = TIMER_CFG_32_BIT_TIMER; // configure for 32-bit mode
			TIMER1_TBMR_R = TIMER_TBMR_TBMR_PERIOD;
			TIMER1_TBILR_R = cyclesPerPeriod-1;
			TIMER1_TBPR_R = 0; // set prescale = 0
			TIMER1_ICR_R = TIMER_ICR_TBTOCINT; // clear timeout flag, friendly since writing a 0 does nothing
			TIMER1_IMR_R |= TIMER_IMR_TBTOIM; // arm the timeout interrupt
			NVIC_PRI5_R = (NVIC_PRI5_R & ~NVIC_PRI5_INT22_M)|(priority << NVIC_PRI5_INT22_S);
			//TIMER1_CTL_R |= TIMER_CTL_TBEN; // enable TimerB1, do this in OS_Launch(.)
			HandlerTaskArray[3] = task; // fill function pointer array w/address of tasks
			break;
		
		case 4:		//TimerA2
			SYSCTL_RCGCTIMER_R |= SYSCTL_RCGCTIMER_R2;   // activate timer2
			delay = SYSCTL_RCGCTIMER_R;   // allow time to finish activating
			TIMER2_CTL_R &= ~TIMER_CTL_TAEN; // disable TimerA2
			TIMER2_CFG_R  = TIMER_CFG_32_BIT_TIMER; // configure for 32-bit mode
			TIMER2_TAMR_R = TIMER_TAMR_TAMR_PERIOD;
			TIMER2_TAILR_R = cyclesPerPeriod-1;
			TIMER2_TAPR_R = 0; // set prescale = 0
			TIMER2_ICR_R = TIMER_ICR_TATOCINT; // clear timeout flag, friendly since writing a 0 does nothing
			TIMER2_IMR_R |= TIMER_IMR_TATOIM; // arm the timeout interrupt
			NVIC_PRI5_R = (NVIC_PRI5_R & ~NVIC_PRI5_INT23_M)|(priority << NVIC_PRI5_INT23_S); //clears PRI bits then shifts the mask into the appropriate place
			//TIMER2_CTL_R |= TIMER_CTL_TAEN; // enable TimerA2, do this in OS_Launch(.)
			HandlerTaskArray[4] = task; // fill function pointer array w/address of tasks
			break;
		
		case 5:		//TimerB2
			SYSCTL_RCGCTIMER_R |= SYSCTL_RCGCTIMER_R2;   // activate timer2
			delay = SYSCTL_RCGCTIMER_R;   // allow time to finish activating
			TIMER2_CTL_R &= ~TIMER_CTL_TBEN; // disable TimerB2
			TIMER2_CFG_R  = TIMER_CFG_32_BIT_TIMER; // configure for 32-bit mode
			TIMER2_TBMR_R = TIMER_TBMR_TBMR_PERIOD;
			TIMER2_TBILR_R = cyclesPerPeriod-1;
			TIMER2_TBPR_R = 0; // set prescale = 0
			TIMER2_ICR_R = TIMER_ICR_TBTOCINT; // clear timeout flag, friendly since writing a 0 does nothing
			TIMER2_IMR_R |= TIMER_IMR_TBTOIM; // arm the timeout interrupt
			NVIC_PRI6_R = (NVIC_PRI6_R & ~NVIC_PRI6_INT24_M)|(priority << NVIC_PRI6_INT24_S); //clears PRI bits then shifts the mask into the appropriate place
			//TIMER2_CTL_R |= TIMER_CTL_TBEN; // enable TimerB2, do this in OS_Launch(.)
			HandlerTaskArray[5] = task; // fill function pointer array w/address of tasks
			break;
		
		case 6:		//TimerA3
			SYSCTL_RCGCTIMER_R |= SYSCTL_RCGCTIMER_R3;   // activate timer3
			delay = SYSCTL_RCGCTIMER_R;   // allow time to finish activating
			TIMER3_CTL_R &= ~TIMER_CTL_TAEN; // disable TimerA3
			TIMER3_CFG_R  = TIMER_CFG_32_BIT_TIMER; // configure for 32-bit mode
			TIMER3_TAMR_R = TIMER_TAMR_TAMR_PERIOD;
			TIMER3_TAILR_R = cyclesPerPeriod-1;
			TIMER3_TAPR_R = 0; // set prescale = 0
			TIMER3_ICR_R = TIMER_ICR_TATOCINT; // clear timeout flag, friendly since writing a 0 does nothing
			TIMER3_IMR_R |= TIMER_IMR_TATOIM; // arm the timeout interrupt
			NVIC_PRI8_R = (NVIC_PRI8_R & ~NVIC_PRI8_INT35_M)|(priority << NVIC_PRI8_INT35_S); //clears PRI bits then shifts the mask into the appropriate place
			//TIMER3_CTL_R |= TIMER_CTL_TAEN; // enable TimerA3, do this in OS_Launch(.)
			HandlerTaskArray[6] = task; // fill function pointer array w/address of tasks
			break;
		
		case 7:		//TimerB3
			SYSCTL_RCGCTIMER_R |= SYSCTL_RCGCTIMER_R3;   // activate timer3
			delay = SYSCTL_RCGCTIMER_R;   // allow time to finish activating
			TIMER3_CTL_R &= ~TIMER_CTL_TBEN; // disable TimerB3
			TIMER3_CFG_R  = TIMER_CFG_32_BIT_TIMER; // configure for 32-bit mode
			TIMER3_TBMR_R = TIMER_TBMR_TBMR_PERIOD;
			TIMER3_TBILR_R = cyclesPerPeriod-1;
			TIMER3_TBPR_R = 0; // set prescale = 0
			TIMER3_ICR_R = TIMER_ICR_TBTOCINT; // clear timeout flag, friendly since writing a 0 does nothing
			TIMER3_IMR_R |= TIMER_IMR_TBTOIM; // arm the timeout interrupt
			NVIC_PRI9_R = (NVIC_PRI9_R & ~NVIC_PRI9_INT36_M)|(priority << NVIC_PRI9_INT36_S); //clears PRI bits then shifts the mask into the appropriate place
			//TIMER3_CTL_R |= TIMER_CTL_TBEN; // enable TimerB3, do this in OS_Launch(.)
			HandlerTaskArray[7] = task; // fill function pointer array w/address of tasks
			break;
		
		case 8:		//TimerA4
			SYSCTL_RCGCTIMER_R |= SYSCTL_RCGCTIMER_R4;   // activate timer4
			delay = SYSCTL_RCGCTIMER_R;   // allow time to finish activating
			TIMER4_CTL_R &= ~TIMER_CTL_TAEN; // disable TimerA4
			TIMER4_CFG_R  = TIMER_CFG_32_BIT_TIMER; // configure for 32-bit mode
			TIMER4_TAMR_R = TIMER_TAMR_TAMR_PERIOD;
			TIMER4_TAILR_R = cyclesPerPeriod-1;
			TIMER4_TAPR_R = 0; // set prescale = 0
			TIMER4_ICR_R = TIMER_ICR_TATOCINT; // clear timeout flag, friendly since writing a 0 does nothing
			TIMER4_IMR_R |= TIMER_IMR_TATOIM; // arm the timeout interrupt
			NVIC_PRI17_R = (NVIC_PRI17_R & ~NVIC_PRI17_INTC_M)|(priority << NVIC_PRI17_INTC_S); //clears PRI bits then shifts the mask into the appropriate place
			//TIMER4_CTL_R |= TIMER_CTL_TAEN; // enable TimerA4, do this in OS_Launch(.)
			HandlerTaskArray[8] = task; // fill function pointer array w/address of tasks
			break;
		
		case 9:		//TimerB4
			SYSCTL_RCGCTIMER_R |= SYSCTL_RCGCTIMER_R4;   // activate timer4
			delay = SYSCTL_RCGCTIMER_R;   // allow time to finish activating
			TIMER4_CTL_R &= ~TIMER_CTL_TBEN; // disable TimerB4
			TIMER4_CFG_R  = TIMER_CFG_32_BIT_TIMER; // configure for 32-bit mode
			TIMER4_TBMR_R = TIMER_TBMR_TBMR_PERIOD;
			TIMER4_TBILR_R = cyclesPerPeriod-1;
			TIMER4_TBPR_R = 0; // set prescale = 0
			TIMER4_ICR_R = TIMER_ICR_TBTOCINT; // clear timeout flag, friendly since writing a 0 does nothing
			TIMER4_IMR_R |= TIMER_IMR_TBTOIM; // arm the timeout interrupt
			NVIC_PRI17_R = (NVIC_PRI17_R & ~NVIC_PRI17_INTD_M)|(priority << NVIC_PRI17_INTD_S); //clears PRI bits then shifts the mask into the appropriate place
			//TIMER4_CTL_R |= TIMER_CTL_TBEN; // enable TimerB4, do this in OS_Launch(.)
			HandlerTaskArray[9] = task; // fill function pointer array w/address of tasks
			break;
		
		case 10:	//TimerA5
			SYSCTL_RCGCTIMER_R |= SYSCTL_RCGCTIMER_R5;   // activate timer5
			delay = SYSCTL_RCGCTIMER_R;   // allow time to finish activating
			TIMER5_CTL_R &= ~TIMER_CTL_TAEN; // disable TimerA5
			TIMER5_CFG_R  = TIMER_CFG_32_BIT_TIMER; // configure for 32-bit mode
			TIMER5_TAMR_R = TIMER_TAMR_TAMR_PERIOD;
			TIMER5_TAILR_R = cyclesPerPeriod-1;
			TIMER5_TAPR_R = 0; // set prescale = 0
			TIMER5_ICR_R = TIMER_ICR_TATOCINT; // clear timeout flag, friendly since writing a 0 does nothing
			TIMER5_IMR_R |= TIMER_IMR_TATOIM; // arm the timeout interrupt
			NVIC_PRI23_R = (NVIC_PRI23_R & ~NVIC_PRI23_INTA_M)|(priority << NVIC_PRI23_INTA_S); //clears PRI bits then shifts the mask into the appropriate place
			//TIMER5_CTL_R |= TIMER_CTL_TAEN; // enable TimerA5, do this in OS_Launch(.)
			HandlerTaskArray[10] = task; // fill function pointer array w/address of tasks
			break;
		
		case 11:	//TimerB5
			SYSCTL_RCGCTIMER_R |= SYSCTL_RCGCTIMER_R5;   // activate timer5
			delay = SYSCTL_RCGCTIMER_R;   // allow time to finish activating
			TIMER5_CTL_R &= ~TIMER_CTL_TBEN; // disable TimerB5
			TIMER5_CFG_R  = TIMER_CFG_32_BIT_TIMER; // configure for 32-bit mode
			TIMER5_TBMR_R = TIMER_TBMR_TBMR_PERIOD;
			TIMER5_TBILR_R = cyclesPerPeriod-1;
			TIMER5_TBPR_R = 0; // set prescale = 0
			TIMER5_ICR_R = TIMER_ICR_TBTOCINT; // clear timeout flag, friendly since writing a 0 does nothing
			TIMER5_IMR_R |= TIMER_IMR_TBTOIM; // arm the timeout interrupt
			NVIC_PRI23_R = (NVIC_PRI23_R & ~NVIC_PRI23_INTB_M)|(priority << NVIC_PRI23_INTB_S); //clears PRI bits then shifts the mask into the appropriate place
			//TIMER5_CTL_R |= TIMER_CTL_TBEN; // enable TimerB5, do this in OS_Launch(.)
			HandlerTaskArray[11] = task; // fill function pointer array w/address of tasks
			break;
		
		default:
			return -1;
	}
	
	timerCount++; // used for launching the correct number of threads that were successfully initialized
	usedTimers[timerCount] = timer; // store the timerID of which timer to launch
	return 0;
}

// Read Timer Count in the timer																									
unsigned long TIMER_ReadTimerValue(int timer)
{
	int32_t status;
	unsigned long count;
	status = StartCritical(); // if it interrupts after it read thhe
	// value from TIMERX_TAR_R and then interrupts before leaving
	// it will return an incorrect Timer count value
	switch(timer)
	{
		case 0: // TimerA0
			count =  TIMER0_TAR_R; // Read value, x, stored in Timer0, 0 < x < TIMER0_TAILR
			break;
		
		case 1: // TimerB0
			count = TIMER0_TBR_R; // Read value, x, stored in Timer0, 0 < x < TIMER0_TBILR
			break;
			
		case 2: // TimerA1
			count = TIMER1_TAR_R; // Read value, x, stored in Timer1, 0 < x < TIMER1_TAILR
			break;
		
		case 3: // TimerB1
			count = TIMER1_TBR_R; // Read value, x, stored in Timer1, 0 < x < TIMER1_TBILR
			break;
		
		case 4: // TimerA2
			count = TIMER2_TAR_R; // Read value, x, stored in Timer2, 0 < x < TIMER2_TAILR
			break;
		
		case 5: // TimerB2
			count = TIMER2_TBR_R; // Read value, x, stored in Timer2, 0 < x < TIMER2_TBILR
			break;
		
		case 6: // TimerA3
			count = TIMER3_TAR_R; // Read value, x, stored in Timer3, 0 < x < TIMER3_TAILR
			break;
		
		case 7: // TimerB3
			count = TIMER3_TBR_R; // Read value, x, stored in Timer3, 0 < x < TIMER3_TBILR
			break;
		
		case 8: // TimerA4
			count = TIMER4_TAR_R; // Read value, x, stored in Timer4, 0 < x < TIMER4_TAILR
			break;
		
		case 9: // TimerB4
			count = TIMER4_TBR_R; // Read value, x, stored in Timer4, 0 < x < TIMER4_TBILR
			break;
		
		case 10: // TimerA5
			count = TIMER5_TAR_R; // Read value, x, stored in Timer5, 0 < x < TIMER5_TAILR
			break;
		
		case 11: // timerB5
			count = TIMER5_TBR_R; // Read value, x, stored in Timer5, 0 < x < TIMER5_TBILR
			break;
		
		default:
			break;
	}
	EndCritical(status);
	return count;
}


// enables interrupts in the NVIC vector table
// setting the bit enables the interrupt, trying to
// clear the bit has no effect in disabling the interrupts
// it requires setting the appropriate bits in the DIS Registers
void TIMER_NVIC_EnableTimerInt(int timer)
{
	switch(timer)
	{
		case 0: // Timer0A
			NVIC_EN0_R = NVIC_EN0_INT19;
			break;
		
		case 1: // Timer0B
			NVIC_EN0_R = NVIC_EN0_INT20;
			break;
		
		case 2: // Timer1A
			NVIC_EN0_R = NVIC_EN0_INT21;
			break;
			
		case 3: // Timer1B
			NVIC_EN0_R = NVIC_EN0_INT22;
			break;
			
		case 4: // Timer2A
			NVIC_EN0_R = NVIC_EN0_INT23;
			break;
		
		case 5: // Timer2B
			NVIC_EN0_R = NVIC_EN0_INT24;
			break;
		
		case 6: // Timer3A
			NVIC_EN1_R = NVIC_EN1_INT35;
			break;
			
		case 7: // Timer3B
			NVIC_EN1_R = NVIC_EN1_INT36;
			break;
		
		case 8: // Timer4A
			NVIC_EN2_R = NVIC_EN2_INT70;
			break;
			
		case 9: // Timer4B
			NVIC_EN2_R = NVIC_EN2_INT71;
			break;
			
		case 10: // Timer5A
			NVIC_EN2_R = NVIC_EN2_INT92;
			break;
		case 11: // Timer5B
			NVIC_EN2_R = NVIC_EN2_INT93;
			break;
	}
}				

// disables interrupts in the NVIC vector table
// setting the bit disables the interrupt, trying to
// clear the bit has no effect in re-enabling the interrupts
// it requires setting the appropriate bits in the EN Registers
void TIMER_NVIC_DisableTimerInt(int timer)
{
	switch(timer)
	{
		case 0: // Timer0A
			NVIC_DIS0_R = NVIC_DIS0_INT19;
			break;
		
		case 1: // Timer0B
			NVIC_DIS0_R = NVIC_DIS0_INT20;
			break;
		
		case 2: // Timer1A
			NVIC_DIS0_R = NVIC_DIS0_INT21;
			break;
			
		case 3: // Timer1B
			NVIC_DIS0_R = NVIC_DIS0_INT22;
			break;
			
		case 4: // Timer2A
			NVIC_DIS0_R = NVIC_DIS0_INT23;
			break;
		
		case 5: // Timer2B
			NVIC_DIS0_R = NVIC_DIS0_INT24;
			break;
		
		case 6: // Timer3A
			NVIC_DIS1_R = NVIC_DIS1_INT35;
			break;
			
		case 7: // Timer3B
			NVIC_DIS1_R = NVIC_DIS1_INT36;
			break;
		
		case 8: // Timer4A
			NVIC_DIS2_R = NVIC_DIS2_INT70;
			break;
			
		case 9: // Timer4B
			NVIC_DIS2_R = NVIC_DIS2_INT71;
			break;
			
		case 10: // Timer5A
			NVIC_DIS2_R = NVIC_DIS2_INT92;
			break;
		case 11: // Timer5B
			NVIC_DIS2_R = NVIC_DIS2_INT93;
			break;
	}
}																							

void Timer0A_Handler(void)
{
	TIMER0_ICR_R = TIMER_ICR_TATOCINT; // acknowledge interrupt flag
	(*(HandlerTaskArray[0]))(); // start Timer0A task
}

void Timer0B_Handler(void)
{
	TIMER0_ICR_R = TIMER_ICR_TBTOCINT; // acknowledge interrupt flag
	(*(HandlerTaskArray[1]))(); // start Timer0B task
}

void Timer1A_Handler(void)
{
	TIMER1_ICR_R = TIMER_ICR_TATOCINT; // acknowledge interrupt flag
	(*(HandlerTaskArray[2]))(); // start Timer1A task
}

void Timer1B_Handler(void)
{
	TIMER1_ICR_R = TIMER_ICR_TBTOCINT; // acknowledge interrupt flag
	(*(HandlerTaskArray[3]))(); // start Timer1B task
}

//void Timer2A_Handler(void)
//{
//	TIMER2_ICR_R = TIMER_ICR_TATOCINT; // acknowledge interrupt flag
//	(*(HandlerTaskArray[4]))(); // start Timer2A task
//}

//void Timer2B_Handler(void)
//{
//	TIMER2_ICR_R = TIMER_ICR_TBTOCINT; // acknowledge interrupt flag
//	(*(HandlerTaskArray[5]))(); // start Timer2B task
//}

//void Timer3A_Handler(void)
//{
//	TIMER3_ICR_R = TIMER_ICR_TATOCINT; // acknowledge interrupt flag
//	(*(HandlerTaskArray[6]))(); // start Timer3A task
//}

void Timer3B_Handler(void)
{
	TIMER3_ICR_R = TIMER_ICR_TBTOCINT; // acknowledge interrupt flag
	(*(HandlerTaskArray[7]))(); // start Timer3B task
}

void Timer4A_Handler(void)
{
	TIMER4_ICR_R = TIMER_ICR_TATOCINT; // acknowledge interrupt flag
	(*(HandlerTaskArray[8]))(); // start Timer4A task
}

void Timer4B_Handler(void)
{
	TIMER4_ICR_R = TIMER_ICR_TBTOCINT; // acknowledge interrupt flag
	(*(HandlerTaskArray[9]))(); // start Timer4B task
}

void Timer5A_Handler(void)
{
	TIMER5_ICR_R = TIMER_ICR_TATOCINT; // acknowledge interrupt flag
	(*(HandlerTaskArray[10]))(); // // start Timer5A task
}

void Timer5B_Handler(void)
{
	TIMER5_ICR_R = TIMER_ICR_TBTOCINT; // acknowledge interrupt flag
	(*(HandlerTaskArray[11]))(); // start Timer5B task
}

