// CECS347Lab2.c
// Runs on TM4C123
// Use PWM0/PB6 and PWM1/PB7 to generate pulse-width modulated outputs.
// Daniel Valvano
// March 28, 2014

/* This example accompanies the book
   "Embedded Systems: Real Time Interfacing to ARM Cortex M Microcontrollers",
   ISBN: 978-1463590154, Jonathan Valvano, copyright (c) 2014
  Program 6.7, section 6.3.2

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
#include <stdint.h>
#include "tm4c123gh6pm.h"

#define PWM_2_GENB_ACTCMPBD_ONE 0x00000C00  // Set the output signal to 1
#define PWM_2_GENB_ACTLOAD_ZERO 0x00000008  // Set the output signal to 0

#define SYSCTL_RCC_USEPWMDIV    0x00100000  // Enable PWM Clock Divisor
#define SYSCTL_RCC_PWMDIV_M     0x000E0000  // PWM Unit Clock Divisor
#define SYSCTL_RCC_PWMDIV_2     0x00000000  // /2

// basic functions defined at end of startup.s
void DisableInterrupts(void); // Disable interrupts
void EnableInterrupts(void);  // Enable interrupts
void WaitForInterrupt(void);  // low power mode

// Output on PF1/M1PWM5
void PWM1B_Init(uint16_t period, uint16_t duty)
	{
  SYSCTL_RCGCPWM_R |= 0x02;             // 1) activate PWM1
  SYSCTL_RCGCGPIO_R |= 0x20;            // 2) activate port F
	SYSCTL_RCGC2_R |= 0x00000020; // (a) activate clock for port F
  volatile unsigned long delay = SYSCTL_RCGCGPIO_R; // allow time to finish activating
  SYSCTL_RCC_R |= SYSCTL_RCC_USEPWMDIV; // 3) use PWM divider
  SYSCTL_RCC_R &= ~SYSCTL_RCC_PWMDIV_M; //    clear PWM divider field
  SYSCTL_RCC_R += SYSCTL_RCC_PWMDIV_2;  //    configure for /2 divider
  PWM1_2_CTL_R = 0;                     // 4) re-loading down-counting mode
  PWM1_2_GENB_R = (PWM_2_GENB_ACTCMPBD_ONE|PWM_2_GENB_ACTLOAD_ZERO);
  // PF1 goes low on LOAD
  // PF1 goes high on CMPB down
  PWM1_2_LOAD_R = period - 1;           // 5) cycles needed to count down to 0
  PWM1_2_CMPB_R = duty - 1;             // 6) count value when output rises
  PWM1_2_CTL_R |= 0x00000001;           // 7) start PWM1
	PWM1_ENABLE_R |= 0x00000020;          // enable PF1/M1PWM5
	
  GPIO_PORTF_LOCK_R = 0x4C4F434B; // unlock GPIO Port F
  GPIO_PORTF_CR_R |= 0x02;         // allow changes to PF1
	GPIO_PORTF_DIR_R |= 0x02;     //     PF1 output
	GPIO_PORTF_AFSEL_R |= 0x02;           // enable alt funct on PF1
  GPIO_PORTF_PCTL_R &= ~0x000000F0;     // configure PF1 as M0PWM1
  GPIO_PORTF_PCTL_R |= 0x00000050;
  GPIO_PORTF_AMSEL_R &= ~0x02;          // disable analog functionality on PF1
  GPIO_PORTF_DEN_R |= 0x02;             // enable digital I/O on PF1
  GPIO_PORTF_PUR_R |= 0x02;     //     enable weak pull-up on PF1
  GPIO_PORTF_IS_R &= ~0x02;     // (d) PF1 is edge-sensitive
  GPIO_PORTF_IBE_R &= ~0x02;    //     PF1 is not both edges
  GPIO_PORTF_IEV_R &= ~0x02;    //     PF1 falling edge event
  GPIO_PORTF_ICR_R = 0x02;      // (e) clear flags 1
	
}

// change duty cycle of PF1
// duty is number of PWM clock cycles output is high  (2<=duty<=period-1)
void PWM1B_Duty(uint16_t duty)
{

  PWM1_2_CMPB_R = duty - 1;             // 6) count value when output rises

}

void PortF_Init(void){

  SYSCTL_RCGC2_R |= 0x00000020; // (a) activate clock for port F
	volatile unsigned long delay;
  GPIO_PORTF_LOCK_R = 0x4C4F434B; // unlock GPIO Port F
  GPIO_PORTF_CR_R |= 0x11;         // allow changes to PF4,0
  GPIO_PORTF_DIR_R &= ~0x11;    // (c) PF4,0 input
	GPIO_PORTF_PCTL_R &= ~0x000F000F;     // clear PF4,0
  GPIO_PORTF_AMSEL_R &= ~0x11;          // disable analog functionality on PF4,0
  GPIO_PORTF_DEN_R |= 0x11;             // enable digital I/O on PF4,0
  GPIO_PORTF_PUR_R |= 0x11;     //     enable weak pull-up on PF4,0
  GPIO_PORTF_IS_R &= ~0x11;     // (d) PF4-PF0 is edge-sensitive
  GPIO_PORTF_IBE_R &= ~0x11;    //     PF4-PF0 is not both edges
  GPIO_PORTF_IEV_R &= ~0x11;    //     PF4-PF0 falling edge event
  GPIO_PORTF_ICR_R = 0x11;      // (e) clear flags 4,0
  GPIO_PORTF_IM_R |= 0x11;      // (f) arm interrupt on PF4,0
  NVIC_PRI7_R = (NVIC_PRI7_R&0xFF00FFFF)|0x00400000; // (g) priority 2
  NVIC_EN0_R = 0x40000000;      // (h) enable interrupt 30 in NVIC
	
}


// L range: 5000,10000,15000,20000,25000,30000,35000,40000,45000
// power:   10%    20%   30%   40%   50%   60%   70%   80%   90%
void GPIOPortF_Handler()
{ // called on touch of either SW1 or SW2
	
	uint16_t new_duty;
	
  if(GPIO_PORTF_RIS_R&0x01) // SW2 touch
	{  
  
		GPIO_PORTF_ICR_R = 0x01;  // acknowledge flag0
		if(PWM1_2_CMPB_R > 5000)
		{
			new_duty = PWM1_2_CMPB_R - 5000;
		  PWM1B_Duty(new_duty);
		}
		
  }
	
  if(GPIO_PORTF_RIS_R&0x10) // SW1 touch
	{  
    
			GPIO_PORTF_ICR_R = 0x10;  // acknowledge flag4
			if(PWM1_2_CMPB_R < 45000)
			{
				new_duty = PWM1_2_CMPB_R + 5000;
			  PWM1B_Duty(new_duty);
			}
	
	}
	
}

int main(void)
{
	
	uint16_t period = 50000;	// period
  DisableInterrupts();  // disable interrupts while initializing
  PortF_Init();        // arm PF4,0 for falling edge interrupts
	PWM1B_Init(period, (period / 2)); // initialize PWM and PF1
  EnableInterrupts();   // enable after all initialization are done
	GPIOPortF_Handler();  // interrupt handler for push button PF4,0

  while(1)
	{
    // main program is free to perform other tasks
    WaitForInterrupt(); // low power mode
  }

}
