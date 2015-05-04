;/*****************************************************************************/
; OSasm.s: low-level OS commands, written in assembly                       */
; Runs on LM4F120/TM4C123
; A very simple real time operating system with minimal features.
; Daniel Valvano
; January 29, 2015
;
; This example accompanies the book
;  "Embedded Systems: Real Time Interfacing to ARM Cortex M Microcontrollers",
;  ISBN: 978-1463590154, Jonathan Valvano, copyright (c) 2015
;
;  Programs 4.4 through 4.12, section 4.2
;
;Copyright 2015 by Jonathan W. Valvano, valvano@mail.utexas.edu
;    You may use, edit, run or distribute this file
;    as long as the above copyright notice remains
; THIS SOFTWARE IS PROVIDED "AS IS".  NO WARRANTIES, WHETHER EXPRESS, IMPLIED
; OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES OF
; MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE.
; VALVANO SHALL NOT, IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL,
; OR CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
; For more information about my classes, my research, and my books, see
; http://users.ece.utexas.edu/~valvano/
; */

        AREA |.text|, CODE, READONLY, ALIGN=2
        THUMB
        REQUIRE8
        PRESERVE8

        EXTERN  RunPt            ; currently running thread
		EXTERN  HighestPriority  ;holds the bit mapping for each priority
		EXTERN  ProxyThread
		EXTERN  ProxyChange
		EXTERN 	ThreadStart
		EXTERN 	ThreadEnd
		EXTERN	ThreadArray
		EXTERN  ThreadCount
		EXTERN  Timer1_TAILR_Ptr
        EXPORT  OS_DisableInterrupts
        EXPORT  OS_EnableInterrupts
        EXPORT  StartOS
        EXPORT  PendSV_Handler
		EXPORT  HighestPri
		;EXPORT  SysTick_Handler


OS_DisableInterrupts
        CPSID   I
        BX      LR


OS_EnableInterrupts
        CPSIE   I
        BX      LR

; Does a context switch on demand
PendSV_Handler
	CPSID   I                  ; 2) Prevent interrupt during switch
    PUSH    {R4-R11}           ; 3) Save remaining regs r4-11
    LDR     R0, =RunPt         ; 4) R0=pointer to RunPt, old thread
    LDR     R1, [R0]           ;    R1 = RunPt
    STR     SP, [R1]           ; 5) Save SP into TCB
	
	LDR   	R2, =ProxyChange
	LDR		R3, [R2]			;R3 has ProxyChange
	CMP		R3, #0
	BNE		JumpToHigher
	LDR 	R1, [R1,#4]			   ; R1 = RunPt, [R1,#4] = RunPt->next
	STR     R1, [R0]           ;    RunPt = R1 (RunPt = RunPt->next)
	LDR     SP, [R1]           ; 7) new thread SP; SP = RunPt->sp;

; profile information
;	LDR		R7, =ThreadArray
;	LDR		R8, =ThreadCount ; index of where we are in the array
;	LDR		R9, [R8]	; R9 = ThreadCount
;	CMP		R9, #400	; check if ThreadCount == 100*4bytes
;	BEQ		Done		; If i have already captured 100 events, then don't profile
;	
;	ADD		R10, R7, R9 ; R10 = R7 + R9 = &ThreadArray + ThreadCount
;	STR		R1,[R10]		;ThreadArray[ThreadCount] = RunPt , this is the new RunPt
;	
;	LDR		R11, =ThreadStart ; R11 = &ThreadStart
;	ADD		R10, R11, R9 ; R10 = R11+R9 = &ThreadStart + ThreadCount
;	LDR		R4, =Timer1_TAILR_Ptr
;	LDR		R4, [R4] ; R4 = &TIMER1_TAILR_R
;	LDR		R4, [R4] ; R4 = TIMER1_TAILR_R
;	STR		R4, [R10]	; ThreadStart = TIMER1_TAILR_R
	
;	LDR		R9, [R8]; tempVar = ThreadCount
;	ADD		R9,#4  ; tempVar++ == tempVar + 4 bytes
;	STR		R9,[R8]; ThreadCount = tempVar
; profile information	
	
	B		Done
JumpToHigher	
	LDR		R4, =ProxyThread	;R2 = pointer to HigherRunPt
	LDR		R5, [R4]			;R3 = HigherRunPt
	STR 	R5, [R0]			; RunPt = HigherRunPt
		
; profile information
;	LDR		R7, =ThreadArray
;	LDR		R8, =ThreadCount ; index of where we are in the array
;	LDR		R9, [R8]	; R9 = ThreadCount
;	CMP		R9, #400	; check if ThreadCount == 100*4bytes
;	BEQ		HigherRunPtDone		; If i have already captured 100 events, don't profile
;	
;	ADD		R10, R7, R9 ; R10 = R7 + R9 = &ThreadArray + ThreadCount
;	STR		R5,[R10]	; ThreadArray[ThreadCount] = RunPt , R5 holds HigherRunPt
;	
;	LDR		R11, =ThreadStart 	; R11 = &ThreadStart
;	ADD		R10, R11, R9 		; R10 = R11+R9 = &ThreadStart + ThreadCount
;	LDR		R4, =Timer1_TAILR_Ptr
;	LDR		R4, [R4] 			; R4 = &TIMER1_TAILR_R
;	LDR		R4, [R4] 			; R4 = TIMER1_TAILR_R
;	STR		R4, [R10]			; ThreadStart = TIMER1_TAILR_R
;	
;	LDR		R9, [R8]			; tempVar = ThreadCount
;	ADD		R9,#4  				; tempVar++ == tempVar + 4 bytes
;	STR		R9,[R8]				; Result: ThreadCount = ThreadCount++
;	
; profile information

HigherRunPtDone	
	MOV 	R6, #0
	STR		R6, [R2]			;Clear PriorityChange flag
	LDR 	SP, [R5]			; SP = HigherRunPt->sp
Done
    POP     {R4-R11}           ; 8) restore regs r4-11
    CPSIE   I                  ; 9) tasks run with interrupts enabled
    BX      LR                 ; 10) restore R0-R3,R12,LR,PC,PSR




StartOS
    LDR     R0, =RunPt         ; currently running thread
    LDR     R2, [R0]           ; R2 = value of RunPt
    LDR     SP, [R2]           ; new thread SP; SP = RunPt->stackPointer;
    POP     {R4-R11}           ; restore regs r4-11
    POP     {R0-R3}            ; restore regs r0-3
    POP     {R12}
    POP     {LR}               ; discard LR from initial stack
    POP     {LR}               ; start location
    POP     {R1}               ; discard PSR
    CPSIE   I                  ; Enable interrupts at processor level
    BX      LR                 ; start first thread



HighestPri
	LDR 	R1, =HighestPriority   ;
	LDR 	R2, [R1]
	CLZ		R0, R2
	BX		LR
	
;sleeping
;	LDR R1, [R1,#4]
;	LDR R2, [R1, #8]
;	CMP	R2,	#0
;	BNE	sleeping


; Does a periodic context switch
;SysTick_Handler
;	CPSID   I                  ; 2) Prevent interrupt during switch
;    PUSH    {R4-R11}           ; 3) Save remaining regs r4-11
;    LDR     R0, =RunPt         ; 4) R0=pointer to RunPt, old thread
;    LDR     R1, [R0]           ;    R1 = RunPt
;    STR     SP, [R1]           ; 5) Save SP into TCB
;    LDR     R1, [R1,#4]        ; 6) R1 = RunPt->next
;    STR     R1, [R0]           ;    RunPt = R1
;    LDR     SP, [R1]           ; 7) new thread SP; SP = RunPt->sp;
;    POP     {R4-R11}           ; 8) restore regs r4-11
;    CPSIE   I                  ; 9) tasks run with interrupts enabled
;    BX      LR                 ; 10) restore R0-R3,R12,LR,PC,PSR

; Spin-Lock counting Semaphore
;OS_Wait ;R0 points to counter
;	LDREX	R1,[R0] ; counter
;	SUBS	R1, #1	; counter - 1
;	ITT		PL		; ok if >= 0
;	STREXPL	R2,R1,[R0]	; try update
;	CMPPL	R2, #0	; succeed?
;	BNE		OS_Wait	; no, try again
;	BX		LR

; Spin-Lock counting Semaphore
;OS_Signal ; R0 points to counter
;	LDREX	R1,	[R0]	; counter
;	ADD 	R1,	#1		; counter + 1
;	STREX	R2, R1, [R0]	; try update
;	CMPPL	R2, #0		; succeed?
;	BNE		OS_Signal	; no, try again
;	BX		LR



; DA 2/18/15 This was derived from Spin-Lock Semaphore
; and might be incorrect, changed subtract to LDR R0,#0
; Binary Semaphore
;OS_bWait ;R0 points to counter
;	LDREX	R1,[R0] ; flag
;	LDR		R1, #0	; flag = 0
;	ITT		PL		; ok if >= 0
;	STREXPL	R2,R1,[R0]	; try update
;	CMPPL	R2, #0	; succeed?
;	BNE		OS_Wait	; no, try again
;	BX		LR

; DA 2/18/15 This was derived from the Spin-Lock Semaphore
; and might be incorrect, I just changed Add R1,#1 to LDR R0,#1
; Binary Semaphore
;OS_bSignal ; R0 points to flag
;	LDREX	R1,	[R0]	; flag
;	LDR 	R1,	#1		; flag = 1
;	STREX	R2, R1, [R0]	; try update
;	CMPPL	R2, #0		; succeed?
;	BNE		OS_Signal	; no, try again
;	BX		LR




    ALIGN
    END
		
