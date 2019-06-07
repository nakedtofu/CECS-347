// CECS347Lab3.c
// Runs on TM4C123
// Use PWM0/PB6 and PWM1/PB7 to generate pulse-width modulated outputs.
// Daniel Valvano, Peter Doan, Naoaki Takatsu
// March 29, 2018

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
#include "Nokia5110.c"
#include "ADCSWTrigger.c"
#include "SysTick.h"

#define PWM_0_GENA_ACTCMPAD_ONE 0x000000C0  // Set the output signal to 1
#define PWM_0_GENA_ACTLOAD_ZERO 0x00000008  // Set the output signal to 0
#define PWM_0_GENB_ACTCMPBD_ONE 0x00000C00  // Set the output signal to 1
#define PWM_0_GENB_ACTLOAD_ZERO 0x00000008  // Set the output signal to 0

#define SYSCTL_RCC_USEPWMDIV    0x00100000  // Enable PWM Clock Divisor
#define SYSCTL_RCC_PWMDIV_M     0x000E0000  // PWM Unit Clock Divisor
#define SYSCTL_RCC_PWMDIV_2     0x00000000  // /2
// basic functions defined at end of startup.s
void DisableInterrupts(void); // Disable interrupts
void EnableInterrupts(void);  // Enable interrupts
void WaitForInterrupt(void);  // low power mode

volatile unsigned char speed_menu = 0;
volatile unsigned long ADCvalue;
volatile unsigned long dist;

uint16_t period = 40000;	// period
bool forward = 1;

#define Black   0x00
#define Red     0x02
#define Blue    0x04
#define Green   0x08
#define Yellow  0x0A
#define SkyBlue 0x0C
#define White   0x0E
#define Pink    0x06


// Color    LED(s) PortF
// black    ---    0
// red      R--    0x02
// blue     --B    0x04
// green    -G-    0x08
// yellow   RG-    0x0A
// sky blue -GB    0x0C
// white    RGB    0x0E
// pink     R-B    0x06

void Delay(void){unsigned long volatile time;
  time = 727240*200/(91*4);  // 0.1sec   200 => 400 
  while(time){
        time--;
  }
}

void PWM0A_Init(uint16_t period, uint16_t duty){
  SYSCTL_RCGCPWM_R |= 0x01;             // 1) activate PWM0
  SYSCTL_RCGCGPIO_R |= 0x02;            // 2) activate port B
  while((SYSCTL_PRGPIO_R&0x02) == 0){};
  GPIO_PORTB_AFSEL_R |= 0x40;           // enable alt funct on PB6
  GPIO_PORTB_PCTL_R &= ~0x0F000000;     // configure PB6 as PWM0
  GPIO_PORTB_PCTL_R |= 0x04000000;
  GPIO_PORTB_AMSEL_R &= ~0x40;          // disable analog functionality on PB6
  GPIO_PORTB_DEN_R |= 0x40;             // enable digital I/O on PB6
  SYSCTL_RCC_R = 0x00100000 |           // 3) use PWM divider
      (SYSCTL_RCC_R & (~0x000E0000));   //    configure for /2 divider
  PWM0_0_CTL_R = 0;                     // 4) re-loading down-counting mode
  PWM0_0_GENA_R = 0xC8;                 // low on LOAD, high on CMPA down
  // PB6 goes low on LOAD
  // PB6 goes high on CMPA down
  PWM0_0_LOAD_R = period - 1;           // 5) cycles needed to count down to 0
  PWM0_0_CMPA_R = duty - 1;             // 6) count value when output rises
  PWM0_0_CTL_R |= 0x00000001;           // 7) start PWM0
  PWM0_ENABLE_R |= 0x00000001;          // enable PB6/M0PWM0
}

void PWM0A_Duty(uint16_t duty){
  PWM0_0_CMPA_R = duty - 1;             // 6) count value when output rises
}

void PWM0B_Init(uint16_t period, uint16_t duty){
  volatile unsigned long delay;
  SYSCTL_RCGCPWM_R |= 0x01;             // 1) activate PWM0
  SYSCTL_RCGCGPIO_R |= 0x02;            // 2) activate port B
  delay = SYSCTL_RCGCGPIO_R;            // allow time to finish activating
  GPIO_PORTB_AFSEL_R |= 0x80;           // enable alt funct on PB7
  GPIO_PORTB_PCTL_R &= ~0xF0000000;     // configure PB7 as M0PWM1
  GPIO_PORTB_PCTL_R |= 0x40000000;
  GPIO_PORTB_AMSEL_R &= ~0x80;          // disable analog functionality on PB7
  GPIO_PORTB_DEN_R |= 0x80;             // enable digital I/O on PB7
  SYSCTL_RCC_R |= SYSCTL_RCC_USEPWMDIV; // 3) use PWM divider
  SYSCTL_RCC_R &= ~SYSCTL_RCC_PWMDIV_M; //    clear PWM divider field
  SYSCTL_RCC_R += SYSCTL_RCC_PWMDIV_2;  //    configure for /2 divider
  PWM0_0_CTL_R = 0;                     // 4) re-loading down-counting mode
  PWM0_0_GENB_R = (PWM_0_GENB_ACTCMPBD_ONE|PWM_0_GENB_ACTLOAD_ZERO);
  // PB7 goes low on LOAD
  // PB7 goes high on CMPB down
  PWM0_0_LOAD_R = period - 1;           // 5) cycles needed to count down to 0
  PWM0_0_CMPB_R = duty - 1;             // 6) count value when output rises
  PWM0_0_CTL_R |= 0x00000001;           // 7) start PWM0
  PWM0_ENABLE_R |= 0x00000002;          // enable PB7/M0PWM1
}

void PWM0B_Duty(uint16_t duty){
  PWM0_0_CMPB_R = duty - 1;             // 6) count value when output rises
}
void PortB_Init(void)
{
	volatile unsigned long delay;
	SYSCTL_RCGC2_R |= 0x00000002; // (a) activate clock for port A
  delay = SYSCTL_RCGC2_R;
	GPIO_PORTB_CR_R = 0x3C;         // allow changes to PB5,4,3,2
  GPIO_PORTB_DIR_R |= 0x3C;    // (c) make PB4,3 out
	GPIO_PORTB_AFSEL_R &= ~0x3C;		//		disable alt function on PB4,3
  GPIO_PORTB_DEN_R |= 0x3C;     //     enable digital I/O on PB4,3
  GPIO_PORTB_PCTL_R &= ~0x00FFFF00; //  configure PB4,3 as GPIO
  GPIO_PORTB_AMSEL_R &= ~0x3C;  //     disable analog functionality on PF4,0        
}

//button init
void PortF_Init(void){
	unsigned long volatile delay;
	SYSCTL_RCGCGPIO_R |= 0x20;            // 2) activate port F and A
  SYSCTL_RCGC2_R |= 0x00000020; // (a) activate clock for port F
	delay = SYSCTL_RCGC2_R;   
  GPIO_PORTF_LOCK_R = 0x4C4F434B; // unlock GPIO Port F
  GPIO_PORTF_CR_R |= 0x1F;         // allow changes to PF4,0
  GPIO_PORTF_DIR_R &= ~0x11;    // (c) PF4,0 input
	GPIO_PORTF_DIR_R |= 0x0E;    // (c) make PF3-1 out (built-in led)
	GPIO_PORTF_AFSEL_R &= ~0x1F;  //     disable alt funct on PF4,0
  GPIO_PORTF_PCTL_R &= ~0x000FFFFF; //  configure PF4-0 as GPIO
  GPIO_PORTF_AMSEL_R &= ~0x1F;          // disable analog functionality on PF4,0
  GPIO_PORTF_DEN_R |= 0x1F;             // enable digital I/O on PF4,0
  GPIO_PORTF_PUR_R |= 0x1F;     //     enable weak pull-up on PF4,0
  GPIO_PORTF_IS_R &= ~0x1F;     // (d) PF4-PF0 is edge-sensitive
  GPIO_PORTF_IBE_R &= ~0x1F;    //     PF4-PF0 is not both edges
  GPIO_PORTF_IEV_R &= ~0x1F;    //     PF4-PF0 falling edge event
  GPIO_PORTF_ICR_R = 0x1F;      // (e) clear flags 4,0
  GPIO_PORTF_IM_R |= 0x11;      // (f) arm interrupt on PF4,0
	
  NVIC_PRI7_R = (NVIC_PRI7_R&0xFF00FFFF)|0x00400000; // (g) priority 2
  NVIC_EN0_R = 0x40000000;      // (h) enable interrupt 30 in NVIC
	
}

//port F handler
void GPIOPortF_Handler() // called on touch of either SW1 or SW2
{ 
	
	uint16_t new_duty;

	//direction handler
  if(GPIO_PORTF_RIS_R&0x01) // SW2 touch
	{  
		Delay();
		GPIO_PORTF_ICR_R = 0x01;  // acknowledge flag0
		forward ^= 1;
  }
	
	//speed handler
  if(GPIO_PORTF_RIS_R&0x10) // SW1 touch
	{  
    Delay();
		GPIO_PORTF_ICR_R = 0x10;  // acknowledge flag4
		speed_menu = (( speed_menu + 1 ) % 4 );

		if(speed_menu == 1)
		{
			new_duty = period * .25;
			Nokia5110_SetCursor(7, 4);          // seven leading spaces, 4th row
			Nokia5110_OutUDec(25);
		}
		else if(speed_menu == 2)
		{
			new_duty = period * .5;
			Nokia5110_SetCursor(7, 4);          // seven leading spaces, bottom row
			Nokia5110_OutUDec(50);
		}
		else if(speed_menu == 3)
		{
			new_duty = period - 1;
			Nokia5110_SetCursor(7, 4);          // seven leading spaces, bottom row
			Nokia5110_OutUDec(100);
		}
		else // speed = 0
		{
			new_duty = 1;
			Nokia5110_SetCursor(7, 4);          // seven leading spaces, bottom row
			Nokia5110_OutUDec(0);
		}
		
		PWM0A_Duty(new_duty);	
		PWM0B_Duty(new_duty);
	
	}
	
	//led handler (maybe in the main while loop)
	if(speed_menu == 0)
	{
		GPIO_PORTF_DATA_R &= ~0x0E;
		GPIO_PORTF_DATA_R |= Red;
		Nokia5110_SetCursor(3, 4);          // three leading spaces, bottom row
		Nokia5110_OutChar('S');
	}
	
	else if(forward == 1)
	{ 	//7 6 5 4 3 2 1 0
			//0 0 0 1 0 1 0 0
		GPIO_PORTB_DATA_R &= ~0x3C;
		GPIO_PORTB_DATA_R |= 0x14; 
			
		GPIO_PORTF_DATA_R &= ~0x0E;
		GPIO_PORTF_DATA_R |= Green;
		Nokia5110_SetCursor(3, 4);          // three leading spaces, bottom row
		Nokia5110_OutChar('F');
	}
	
	else 
	{
			//7 6 5 4 3 2 1 0
			//0 0 1 0 1 0 0 0
		GPIO_PORTB_DATA_R &= ~0x3C;
		GPIO_PORTB_DATA_R |= 0x28; 
			
		GPIO_PORTF_DATA_R &= ~0x0E;
		GPIO_PORTF_DATA_R |= Blue;
		Nokia5110_SetCursor(3, 4);          // three leading spaces, bottom row
		Nokia5110_OutChar('R');
	}
	Nokia5110_SetCursor(0, 5);          // three leading spaces, bottom row
  Nokia5110_OutString("ADC:");
	Nokia5110_OutUDec(ADCvalue);
	Delay();                
}

//adc value from 10cm-80cm, some adc values modified to make values unique
int adclist[71] =
{
3044,
2820,
2624,
2450,
2300,
2169,
2052,
1927,
1834,
1764,
1698,
1615,
1588,
1514,
1466,
1412,
1368,
1324,
1280,
1250,
1194,
1184,
1121,
1110,
1086,
1055,
1039,
1016,
995,
958,
936,
924,
906,
900,
862,
839,
832,
812,
796,
794,
773,
760,
738,
728,
711,
706,
692,
681,
662,
666,
644,
640,
624,
623,
618,
602,
601,
577,
580,
574,
543,
540, //originally 543
537, //originally 543
534,
526,
514,
505,
502,
486,
472,
470
};

int main(void)
{
	DisableInterrupts();  // disable interrupts while initializing
	ADC0_InitSWTriggerSeq3_Ch1();         // ADC initialization PE2/AIN1
	PortF_Init();        
	PortB_Init();
	PWM0A_Init(period, 0);
	PWM0B_Init(period, 0);
  EnableInterrupts();   // enable after all initialization are done
	GPIO_PORTF_DATA_R &= ~0x0E;
	GPIO_PORTF_DATA_R |= Red;
	
	Nokia5110_Init();
  Nokia5110_Clear();
  Nokia5110_OutString("* ADC Test *");
	Nokia5110_OutString("************");
	Nokia5110_OutString("ADC:        ");
	Nokia5110_OutString("TDist:      ");
	Nokia5110_OutString("CDist:      ");
	
	//initial value
	Nokia5110_SetCursor(3, 4);          // three leading spaces, bottom row
	Nokia5110_OutChar('S');
	Nokia5110_OutString("       ");
	Nokia5110_OutChar('0');
	//int delay;
  while(1)
	{
		int t_dist;
		Delay();
    ADCvalue =  ADC0_InSeq3();
		Nokia5110_SetCursor(7, 2);
		Nokia5110_OutUDec(ADCvalue);
		
		t_dist = 0;
		while(ADCvalue < adclist[t_dist]){
			t_dist++;
			if(t_dist > (sizeof(adclist)/sizeof(adclist[0])))
				break;
		}
		t_dist = t_dist + 10;
		Nokia5110_SetCursor(5, 3);
		Nokia5110_OutUDec(t_dist);
		Nokia5110_OutString("cm");
		
		Nokia5110_SetCursor(5, 4);
		dist = dist = -3.3654785 + (40628.2059 / ADCvalue);
		Nokia5110_OutUDec(dist);
		Nokia5110_OutString("cm");
		
  }

}
