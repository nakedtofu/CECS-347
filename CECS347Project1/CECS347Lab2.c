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
#include <stdbool.h>
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

volatile unsigned char speed_menu = 0, led_menu = 0;
uint16_t period = 50000;	// period
bool forward = 1;

void Speed_Init(uint16_t period, uint16_t duty)
{
	
	SYSCTL_RCGCPWM_R |= 0x02;             // 1) activate PWM1	
	SYSCTL_RCGCGPIO_R |= 0x01;            //  activate port A
	SYSCTL_RCGC2_R |= 0x00000001; // (a) activate clock for port A
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
	
  GPIO_PORTA_CR_R |= 0xC0;         // allow changes to PA7,6
  GPIO_PORTA_DIR_R |= 0xC0;    // (c) PA7,6 output
	GPIO_PORTA_PCTL_R &= ~0xFF000000;     // clear PA7,6
  GPIO_PORTA_AMSEL_R &= ~0x3C;          // disable analog functionality on PA5,4,3,2
  GPIO_PORTA_DEN_R |= 0x3C;             // enable digital I/O on PA5,4,3,2
  GPIO_PORTA_PUR_R |= 0x3C;     //     enable weak pull-up on PA5,4,3,2
  GPIO_PORTA_IS_R &= ~0x3C;     // (d) PA5,4,3,2 is edge-sensitive
  GPIO_PORTA_IBE_R &= ~0x3C;    //     PA5,4,3,2 is not both edges
  GPIO_PORTA_IEV_R &= ~0x3C;    //     PA5,4,3,2 falling edge event
  GPIO_PORTA_ICR_R = 0x3C;      // (e) clear flags PA5,4,3,2
	
}

void Direction_Init(void)
{
	
	SYSCTL_RCGCGPIO_R |= 0x01;            //  activate port A
	SYSCTL_RCGC2_R |= 0x00000001; // (a) activate clock for port A
	volatile unsigned long delay;
  GPIO_PORTA_CR_R |= 0x3C;         // allow changes to PA5,4,3,2
  GPIO_PORTA_DIR_R |= 0x3C;    // (c) PA5,4,3,2 output
	GPIO_PORTA_PCTL_R &= ~0x000FFFF00;     // clear PA5,4,3,2
  GPIO_PORTA_AMSEL_R &= ~0x3C;          // disable analog functionality on PA5,4,3,2
  GPIO_PORTA_DEN_R |= 0x3C;             // enable digital I/O on PA5,4,3,2
  GPIO_PORTA_PUR_R |= 0x3C;     //     enable weak pull-up on PA5,4,3,2
  GPIO_PORTA_IS_R &= ~0x3C;     // (d) PA5,4,3,2 is edge-sensitive
  GPIO_PORTA_IBE_R &= ~0x3C;    //     PA5,4,3,2 is not both edges
  GPIO_PORTA_IEV_R &= ~0x3C;    //     PA5,4,3,2 falling edge event
  GPIO_PORTA_ICR_R = 0x3C;      // (e) clear flags PA5,4,3,2
	
}

//led init
void Switch_Init(void)
{

	SYSCTL_RCGCGPIO_R |= 0x20;            // 2) activate port F
	SYSCTL_RCGC2_R |= 0x00000020; // (a) activate clock for port F
	volatile unsigned long delay;
  GPIO_PORTF_LOCK_R = 0x4C4F434B; // unlock GPIO Port F
  GPIO_PORTF_CR_R |= 0x0E;         // allow changes to PF3,2,1
  GPIO_PORTF_DIR_R |= 0x0E;    // (c) PF3,2,1 output
	GPIO_PORTF_PCTL_R &= ~0x0000FFF0;     // clear PF3,2,1
  GPIO_PORTF_AMSEL_R &= ~0x0E;          // disable analog functionality on PF3,2,1
  GPIO_PORTF_DEN_R |= 0x0E;             // enable digital I/O on PF3,2,1
  GPIO_PORTF_PUR_R |= 0x0E;     //     enable weak pull-up on PF3,2,1
  GPIO_PORTF_IS_R &= ~0x0E;     // (d) PF3,2,1 is edge-sensitive
  GPIO_PORTF_IBE_R &= ~0x0E;    //     PF3,2,1 is not both edges
  GPIO_PORTF_IEV_R &= ~0x0E;    //     PF3,2,1 falling edge event
  GPIO_PORTF_ICR_R = 0x0E;      // (e) clear flags 3,2,1
	
}

//button init
void PortF_Init(void){

	SYSCTL_RCGCGPIO_R |= 0x20;            // 2) activate port F
  SYSCTL_RCGC2_R |= 0x00000020; // (a) activate clock for port F
	speed_menu = 0;
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

// change duty cycle of PF1
// duty is number of PWM clock cycles output is high  (2<=duty<=period-1)
void PWM1B_Duty(uint16_t duty)
{

  PWM1_2_CMPB_R = duty - 1;             // 6) count value when output rises

}

//port F handler
void GPIOPortF_Handler() // called on touch of either SW1 or SW2
{ 
	
	uint16_t new_duty;
	
	//direction handler
  if(GPIO_PORTF_RIS_R&0x01) // SW2 touch
	{  
  
		GPIO_PORTF_ICR_R = 0x01;  // acknowledge flag0
		forward = ~forward;
		
  }
	
	//speed handler
  if(GPIO_PORTF_RIS_R&0x10) // SW1 touch
	{  
    
		GPIO_PORTF_ICR_R = 0x10;  // acknowledge flag4
		speed_menu = (speed_menu+1)%4;
		
		switch(speed_menu)
		{
			
			case 0:
				new_duty = 0;
			case 1:
				new_duty = period * .25;
			case 2:
				new_duty = period * .5;
			case 3:
				new_duty = period;
			default:
				new_duty = 0;
			
		}
		
		PWM1B_Duty(new_duty);	
	
	}
	
	//led handler
	if((forward == 1) & (speed_menu != 0)) //forward green
	{
		
		GPIO_PORTF_DATA_R &= ~0x0E;
		GPIO_PORTF_DATA_R |= 0x08;
		
	}
	
	else if((forward == 0) & (speed_menu != 0)) //backward blue
	{
		
		GPIO_PORTF_DATA_R &= ~0x0E;
		GPIO_PORTF_DATA_R |= 0x04;
		
	}
	
	else //stopped red
	{
		
		GPIO_PORTF_DATA_R &= ~0x0E;
		GPIO_PORTF_DATA_R |= 0x02;
	
	}
	
}

void Direction_Handler(void)
{
	
	if(forward == 1)
	{
		
		GPIO_PORTA_DATA_R &= ~0x3C;
		GPIO_PORTA_DATA_R |= 0x28; //clear PA5-2 and set PA5,3 to 1
		
	}
	
	else if(forward == 0)
	{
		
		GPIO_PORTA_DATA_R &= ~0x3C;
		GPIO_PORTA_DATA_R |= 0x14; //clear PA5-2 and set PA4,2 to 1
		
	}
	
}

int main(void)
{
	
  DisableInterrupts();  // disable interrupts while initializing
  PortF_Init();        // arm PF4,0 for falling edge interrupts
	Speed_Init(period, (period / 2)); // initialize PWM and PF1
  EnableInterrupts();   // enable after all initialization are done
	GPIOPortF_Handler();  // interrupt handler for push button PF4,0
	Direction_Handler(); // direction handler for PA5-2

  while(1)
	{
    // main program is free to perform other tasks
    WaitForInterrupt(); // low power mode
  }

}
