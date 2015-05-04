//Timer.h

#include <stdint.h>
// these are stored in a buffer to easily start tasks by enabling the timer
#define TIMER0_CTL_PTR_R            (volatile uint32_t *)0x4003000C
#define TIMER1_CTL_PTR_R            (volatile uint32_t *)0x4003100C
#define TIMER2_CTL_PTR_R            (volatile uint32_t *)0x4003200C
#define TIMER3_CTL_PTR_R            (volatile uint32_t *)0x4003300C
#define TIMER4_CTL_PTR_R            (volatile uint32_t *)0x4003400C
#define TIMER5_CTL_PTR_R            (volatile uint32_t *)0x4003500C
	
#define CLOCKSPEED_80MHZ			80000000 // 80 MHz
#define CLOCKSPEED_50MHZ			80000000 // 80 MHz



extern void(*HandlerTaskArray[12])(void);
extern volatile unsigned int* timerCtrlBuf[12];
extern void OS_DisableInterrupts(void); // Disable interrupts
extern void OS_EnableInterrupts(void);  // Enable interrupts
extern int32_t StartCritical(void);
extern void EndCritical(int32_t primask);



// ***************** Timer3_Init ****************
// Activate Timer3 interrupts to run user task periodically
// Inputs:  task is a pointer to a user function
//          period in units (1/clockfreq)
// Outputs: none
void Timer3_Init(void(*task)(void), uint32_t period);



int TIMER_TimerInit(void(*task)(void), int timer, unsigned long desiredFrequency, unsigned long priority);

void TIMER_ClearPeriodicTime(int timer);

unsigned long TIMER_ReadTimerPeriod(int timer);

// Read Timer Count in the timer																									
unsigned long TIMER_ReadTimerValue(int timer);


// enables interrupts in the NVIC vector table
// setting the bit enables the interrupt, trying to
// clear the bit has no effect in disabling the interrupts
// it requires setting the appropriate bits in the DIS Registers
void TIMER_NVIC_EnableTimerInt(int timer);

// disables interrupts in the NVIC vector table
// setting the bit disables the interrupt, trying to
// clear the bit has no effect in re-enabling the interrupts
// it requires setting the appropriate bits in the EN Registers
void TIMER_NVIC_DisableTimerInt(int timer);

