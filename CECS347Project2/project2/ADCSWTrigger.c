// ADCSWTrigger.c
// Runs on LM4F120/TM4C123
// Provide functions that initialize ADC0 SS3 to be triggered by
// software and trigger a conversion, wait for it to finish,
// and return the result.
// Daniel Valvano, Peter Doan, Naoaki Takatsu
// March 29, 2018

/* This example accompanies the book
   "Embedded Systems: Real Time Interfacing to Arm Cortex M Microcontrollers",
   ISBN: 978-1463590154, Jonathan Valvano, copyright (c) 2013

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

#include "tm4c123gh6pm.h"
#include "SysTick.h"

void ADC0_InitSWTriggerSeq2_Ch1(void){ volatile unsigned long delay;
  SYSCTL_RCGC2_R |= 0x00000010;   // 1) activate clock for Port E
  delay = SYSCTL_RCGC2_R;         //    allow time for clock to stabilize
  GPIO_PORTE_DIR_R &= ~0x0E;      // 2) make PE3-1 input
  GPIO_PORTE_AFSEL_R |= 0x0E;     // 3) enable alternate function on PE3-1
  GPIO_PORTE_DEN_R &= ~0x0E;      // 4) disable digital I/O on PE3-1
  GPIO_PORTE_AMSEL_R |= 0x0E;     // 5) enable analog function on PE3-1
  SYSCTL_RCGC0_R |= 0x00010000;   // 6) activate ADC0 
  delay = SYSCTL_RCGC2_R;         
  SYSCTL_RCGC0_R &= ~0x00000300;  // 7) configure for 125K 
  ADC0_SSPRI_R = 0x3210;          // 8) Sequencer 3 is lowest priority
  ADC0_ACTSS_R &= ~0x0004;        // 9) disable sample sequencer 2
  ADC0_EMUX_R &= ~0x0F00;         // 10) seq3 is software trigger
  ADC0_SSMUX2_R = 0x0210; // 11) channel Ain1-3 (PE2-0)
  ADC0_SSCTL2_R = 0x0600;         // 12) no TS2 D2, yes IE2 END2
	ADC0_IM_R &= ~0x0004; // disable SS2 interrupts
  ADC0_ACTSS_R |= 0x0004;         // 13) enable sample sequencer 2
}

void ADC0_In12(unsigned long *ain1, unsigned long *ain2, unsigned long *ain3){
	SysTick_Init();
	SysTick_Wait10ms(10);
  ADC0_PSSI_R = 0x0004;            // 1) initiate SS2
  while((ADC0_RIS_R&0x04)==0){};   // 2) wait for conversion done
	*ain3 = ADC0_SSFIFO2_R&0xFFF;    // 3A) read first result (potentiometer)
  *ain2 = ADC0_SSFIFO2_R&0xFFF;    // 3B) read second result (left IR)
	*ain1 = ADC0_SSFIFO2_R&0xFFF;    // 3B) read third result (right IR)
  ADC0_ISC_R = 0x0004;             // 4) acknowledge completion
}
