// Period_Measurement.c
// Runs on TM4C123
// Use Timer0A in input edge mode 16-bits count.  
// Timer0A use PB6 as imput.
// PB7 connects to Ultrasonic sensor trigger pin.
// By Min He
// March 30th, 2018

/* This example used the following book example, Example 8.3, as reference
   "Embedded Systems: Real Time Operating Systems for Arm Cortex M Microcontrollers",
   ISBN: 978-1466468863, Jonathan Valvano, copyright (c) 2014

 Copyright 2018 by Min He, min.he@csulb.edu
    You may use, edit, run or distribute this file
    as long as the above copyright notice remains
 THIS SOFTWARE IS PROVIDED "AS IS".  NO WARRANTIES, WHETHER EXPRESS, IMPLIED
 OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES OF
 MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE.
 VALVANO SHALL NOT, IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL,
 OR CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
 */

#include <stdint.h>
#include "SysTick.h"
#include "tm4c123gh6pm.h"

#define MC_LEN 0.0625      // length of one machine cyce in microsecond for 16MHz clock
#define SOUND_SPEED 0.0343 // centimeter per micro-second
#define MAX_DURATION 0xFFFF

void Timer0_Init(void);
void WaitForInterrupt(void);  // low power mode
void EnableInterrupts(void);

uint32_t period=0;
uint8_t done=0;

// the following variables are for testing purpose, need to move inside main after testing
uint32_t distance=0;
uint32_t first_time = 0;
uint32_t second_time = 0;
uint32_t first_read=0, second_read=0;
uint8_t OutOfRange = 0;

int main(void){
	
//  PLL_Init(Bus80MHz);               // 80 MHz clock
	SysTick_Init();         // use default 16MHz clock
  Timer0_Init();          // initialize timer0A 
  EnableInterrupts();
  while(1){
		GPIO_PORTB_DATA_R &= ~0x80; // send low to trigger
		SysTick_Wait1us(2);
		GPIO_PORTB_DATA_R |= 0x80; // send high to trigger
		SysTick_Wait1us(10);
		GPIO_PORTB_DATA_R &= ~0x80; // send low to trigger
    TIMER0_IMR_R |= 0x00000004;    // enable capture mode event interrupt
	  TIMER0_CTL_R = 0x0000000D;    // Enable TIMER0A capture mode: both edges
    TIMER0_TAILR_R = MAX_DURATION;    // reload start value
		
		// Use general purpose timer input edge mode 16 bits count, 
		// detectable range: (65535*62.58*10^(-3)*0.0343)/2=70.2cm
		// Notice that the detect range for HC - SR04 ultrasonic sensor is 400cm
    // Since our application only need to detect obstcle within 70cm, 
    // 16 bits count is good enough for us.		
		SysTick_Wait(MAX_DURATION);
	  TIMER0_CTL_R = 0x00000000;    // disable TIMER0A 
    TIMER0_IMR_R &= ~0x00000004;    // disable capture mode event interrupt
		
    if (done) {
			// The speed of sound is approximately 340 meters per second, 
			// or  .0343 c/µS.
      // Distance = (duration * 0.0343)/2;
		  distance = (period*MC_LEN*SOUND_SPEED)/2;		
		  done = 0;
			OutOfRange = 0;
		}
		else { // out of range			
			OutOfRange = 1;
		  distance = 0;		
		}
	}
}
// ***************** Timer0_Init ****************
// Activate TIMER0 interrupts to run user task periodically
// Inputs:  task is a pointer to a user function
//          period in units (1/clockfreq)
// Outputs: none
void Timer0_Init(void){
  SYSCTL_RCGCTIMER_R |= 0x01;      // activate timer0
  SYSCTL_RCGCGPIO_R |= 0x0002;     // activate port B
  while((SYSCTL_PRGPIO_R&0x0002) == 0){};// ready?
  GPIO_PORTB_AFSEL_R |= 0x40;      // enable alt funct on PB6
  GPIO_PORTB_DEN_R |= 0x40;        // enable digital I/O on PB6
                                   // configure PB6 as T0CCP0
  GPIO_PORTB_PCTL_R = (GPIO_PORTB_PCTL_R&0xF0FFFFFF)+0x07000000;
  GPIO_PORTB_AMSEL_R &= ~0x40;     // disable analog functionality on PB6
		
	// PB7 connects to Ultrasonic sensor trigger pin
  GPIO_PORTB_AFSEL_R &= ~0x80;      // disable alt funct on PB7
  GPIO_PORTB_DEN_R |= 0x80;        // enable digital I/O on PB7
                                   // configure PB7 as GPIO
  GPIO_PORTB_PCTL_R &= ~0xF0000000;
  GPIO_PORTB_AMSEL_R &= ~0x80;     // disable analog functionality on PB7
	GPIO_PORTB_DIR_R |= 0x80;        // PB7 is output

  TIMER0_CTL_R &= ~0x0000000F;    // 1) disable TIMER0A during setup
  TIMER0_CFG_R = 0x00000004;    // 2) configure for 16-bit timer mode
	TIMER0_TAMR_R = 0x00000007;   // 3) edge time capture mode: count down
  TIMER0_TAILR_R = MAX_DURATION;    // 4) start value
  TIMER0_ICR_R = 0x00000004;    // 6) clear TIMER0A capture flag
  TIMER0_IMR_R &= ~0x00000004;    // 7) disable capture mode event interrupt
  
	NVIC_PRI4_R = (NVIC_PRI4_R&0x1FFFFFFF)|0x80000000; // 8) priority 2
  // interrupts enabled in the main program after all devices initialized
  // vector number 35, interrupt number 19
  NVIC_EN0_R = 1<<19;           // 9) enable IRQ 19 in NVIC
}

void Timer0A_Handler(void)
{
	static uint32_t first = 0;
	
  TIMER0_ICR_R = TIMER_ICR_CAECINT;// acknowledge TIMER0A capture interrupt
	if ((GPIO_PORTB_DATA_R & 0x40)==0x40) { //rising edge
		first = TIMER0_TAR_R;  
		first_time = first; // this line of code is for debugging purpose, can be removed
		done = 0;
	}
	else if (first != 0){
		period = (first - TIMER0_TAR_R)&0x00FFFFFF; // 24 bits counter
		second_time = TIMER0_TAR_R; // this line of code is for debugging purpose, can be removed
		done = 1;
		first = 0;
	  TIMER0_CTL_R = 0x00000000;    // disable TIMER0A 
    TIMER0_IMR_R &= ~0x00000004;    // disable capture mode event interrupt
	}                                 
}
