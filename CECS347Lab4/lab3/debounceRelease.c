// This is example program based on Lab2_HelloLaunchPad by Daniel Valvano, 
// Jonathan Valvano and Ramesh Yerraballi. The program runs on the 
// LaunchPad TM4C123
// 
// Modified by Dr.Min He

// LaunchPad built-in hardware
// SW1 left switch is negative logic PF4 on the Launchpad
// SW2 right switch is negative logic PF0 on the Launchpad
// red LED connected to PF1 on the Launchpad
// blue LED connected to PF2 on the Launchpad
// green LED connected to PF3 on the Launchpad

// 1. Pre-processor Directives Section
// Constant declarations to access port registers using 
// symbolic names instead of addresses
#include "tm4c123gh6pm.h"

// 2. Declarations Section
//   Global Variables
unsigned long In;  // input from PF4
unsigned long Out; // outputs to PF3,PF2,PF1 (multicolor LED)

//   Function Prototypes
void PortF_Init(void);
void Delay(void);


// 3. Subroutines Section
// MAIN: Mandatory for a C Program to be executable
int main(void){  
  unsigned char p_count=0,r_count=0;
  unsigned char pressed=0, released = 1;  
  PortF_Init();        // Call initialization of port PF4 PF2    

  while(1){
		In = GPIO_PORTF_DATA_R&0x10; // read PF4:sw1 into In
    if((In == 0x00)&& released){ 			// zero means SW1 is pressed
			
			  p_count ++;
			  if (p_count>=80) {
				  pressed = 1;
				  released = 0;
				  p_count = 0;
					Delay();                     // wait 0.01 sec
			  }
		}
	
			
		if ((In == 0x10)&& pressed){
			
			  r_count ++;
			  if (r_count>=80) {
				  pressed = 0;
				  released = 1;

				  r_count = 0;
				  GPIO_PORTF_DATA_R ^= 0x02;  // turn on red LED
				  Delay();                     // wait 0.01 sec
			}
		}
	}
}

// Subroutine to initialize port F pins for input and output
// PF4 and PF0 are input SW1 and SW2 respectively
// PF3,PF2,PF1 are outputs to the LED
// Inputs: None
// Outputs: None
// Notes: These five pins are connected to hardware on the LaunchPad
void PortF_Init(void){ volatile unsigned long delay;
  SYSCTL_RCGC2_R |= 0x00000020;     // 1) F clock
  delay = SYSCTL_RCGC2_R;           // delay   
  GPIO_PORTF_LOCK_R = 0x4C4F434B;   // 2) unlock PortF PF0  
  GPIO_PORTF_CR_R = 0x1F;           // allow changes to PF4-0       
  GPIO_PORTF_AMSEL_R = 0x00;        // 3) disable analog function
  GPIO_PORTF_PCTL_R = 0x00000000;   // 4) GPIO clear bit PCTL  
  GPIO_PORTF_DIR_R = 0x0E;          // 5) PF4,PF0 input, PF3,PF2,PF1 output   
  GPIO_PORTF_AFSEL_R = 0x00;        // 6) no alternate function
  GPIO_PORTF_PUR_R = 0x11;          // enable pullup resistors on PF4,PF0       
  GPIO_PORTF_DEN_R = 0x1F;          // 7) enable digital pins PF4-PF0        
}
// Color    LED(s) PortF
// dark     ---    0
// red      R--    0x02
// blue     --B    0x04
// green    -G-    0x08
// yellow   RG-    0x0A
// sky blue -GB    0x0C
// white    RGB    0x0E
// pink     R-B    0x06

// Subroutine to wait 0.1 sec
// Inputs: None
// Outputs: None
// Notes: ...
void Delay(void){unsigned long volatile time;
  time = 727240*20/91;  // 0.01sec
  while(time){
		time--;
  }
}
