// CECS347Lab1.c
// Runs on LM4F120 or TM4C123
// Use SysTick interrupts to implement a software PWM to drive
// a DC motor at a given duty cycle.  The built-in button SW1
// increases the speed, and SW2 decreases the speed.
// Daniel Valvano, Jonathan Valvano, Naoaki Takatsu
// Feb 1, 2017

/* This example accompanies the book
   "Embedded Systems: Introduction to ARM Cortex M Microcontrollers",
   ISBN: 978-1469998749, Jonathan Valvano, copyright (c) 2013
   "Embedded Systems: Real Time Interfacing to ARM Cortex M Microcontrollers",
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

#include <stdint.h>
#include "tm4c123gh6pm.h"

// basic functions defined at end of startup.s
void DisableInterrupts(void); // Disable interrupts
void EnableInterrupts(void);  // Enable interrupts
void WaitForInterrupt(void);  // low power mode

unsigned long H,L;            // high and low tick

void Systick_Init(void){
  SYSCTL_RCGC2_R |= 0x00000001; // activate clock for port A
  H = L = 400;                // 50%
  NVIC_ST_CTRL_R = 0;           // disable SysTick during setup
  NVIC_ST_RELOAD_R = L-1;       // reload value for 500us
  NVIC_ST_CURRENT_R = 0;        // any write to current clears it
  NVIC_SYS_PRI3_R = (NVIC_SYS_PRI3_R&0x00FFFFFF)|0x40000000; // priority 2
  NVIC_ST_CTRL_R = 0x00000007;  // enable with core clock and interrupts
}

void Switch_Init(void){

  SYSCTL_RCGC2_R |= 0x00000020; // (a) activate clock for port F
  H = L;
  GPIO_PORTF_LOCK_R = 0x4C4F434B; // unlock GPIO Port F
  GPIO_PORTF_CR_R = 0x13;         // allow changes to PF4,1,0
  GPIO_PORTF_DIR_R &= ~0x11;    // (c) PF4,0 input
	GPIO_PORTF_DIR_R |= 0x02;     //     PF1 output
  GPIO_PORTF_AFSEL_R &= ~0x13;  //     disable alt funct on PF4,1,0
  GPIO_PORTF_DEN_R |= 0x13;     //     enable digital I/O on PF4,1,0
  GPIO_PORTF_PCTL_R &= ~0x000F00FF; //  configure PF4,1,0 as GPIO
  GPIO_PORTF_AMSEL_R &= ~0x13;  //     disable analog functionality on PF4,1,0
  GPIO_PORTF_PUR_R |= 0x13;     //     enable weak pull-up on PF4,1,0
  GPIO_PORTF_IS_R &= ~0x13;     // (d) PF4-PF0 is edge-sensitive
  GPIO_PORTF_IBE_R &= ~0x13;    //     PF4-PF0 is not both edges
  GPIO_PORTF_IEV_R &= ~0x13;    //     PF4-PF0 falling edge event
  GPIO_PORTF_ICR_R = 0x13;      // (e) clear flags 4,1,0
  GPIO_PORTF_IM_R |= 0x13;      // (f) arm interrupt on PF4,1,0
  NVIC_PRI7_R = (NVIC_PRI7_R&0xFF00FFFF)|0x00400000; // (g) priority 2
  NVIC_EN0_R = 0x40000000;      // (h) enable interrupt 30 in NVIC
}
// L range: 8000,16000,24000,32000,40000,48000,56000,64000,72000
// power:   10%    20%   30%   40%   50%   60%   70%   80%   90%
void GPIOPortF_Handler(void){ // called on touch of either SW1 or SW2
  if(GPIO_PORTF_RIS_R&0x01){  // SW2 touch
    GPIO_PORTF_ICR_R = 0x01;  // acknowledge flag0
    if(L>80) L = L-80;    // slow down
  }
  if(GPIO_PORTF_RIS_R&0x10){  // SW1 touch
    GPIO_PORTF_ICR_R = 0x10;  // acknowledge flag4
    if(L<720) L = L+80;   // speed up
  }
  H = 800-L; // constant period of 1ms, variable duty cycle
}

void SysTick_Handler(void){
  if(GPIO_PORTF_DATA_R&0x02){   // toggle PF1
    GPIO_PORTF_DATA_R &= ~0x02; // make PF1 low
    NVIC_ST_RELOAD_R = L-1;     // reload value for low phase
  } else{
    GPIO_PORTF_DATA_R |= 0x02;  // make PF1 high
    NVIC_ST_RELOAD_R = H-1;     // reload value for high phase
  }
}

int main(void){
  DisableInterrupts();  // disable interrupts while initializing
  Systick_Init();         // SysTick interrupts
  Switch_Init();        // arm PF4,1,0 for falling edge interrupts
  EnableInterrupts();   // enable after all initialization are done
  while(1){
    // main program is free to perform other tasks
    WaitForInterrupt(); // low power mode
  }
}
