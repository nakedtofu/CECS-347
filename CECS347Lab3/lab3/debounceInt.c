// This is example program based on Lab2_HelloLaunchPad by Daniel Valvano, 
// Jonathan Valvano and Ramesh Yerraballi. The program runs on the 
// LaunchPad TM4C123
// 
// Modified by Dr.Min He to demonstrate switch debounce interrupt approach

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

// constants
#define RELEASED 0
#define PRESSED 1

// 2. Declarations Section
//   Global Variables
unsigned char pressed=0, released = 1;
unsigned char prev_s, current_s;	
unsigned char count=0;

//   Function Prototypes
extern void EnableInterrupts(void);
void PortF_Init(void);
void Delay(void);

// 3. Subroutines Section
// MAIN: Mandatory for a C Program to be executable
int main(void){  
  PortF_Init();        // Call initialization of port PF4 PF2 
  EnableInterrupts();           // (i) Clears the I bit
  current_s=RELEASED;
  prev_s = current_s;	

  while(1){
		if (prev_s != current_s) {
			Delay();
			prev_s = current_s;
		}
	}
}

// Subroutine to initialize port F pins for input and output
// PF4 and PF0 are input SW1 and SW2 respectively
// PF3,PF2,PF1 are outputs to the LED
// Inputs: None
// Outputs: None
// Notes: These five pins are connected to hardware on the LaunchPad
void PortF_Init(void){ 
	volatile unsigned long delay;
	
  SYSCTL_RCGC2_R |= 0x00000020;     // 1) F clock
  delay = SYSCTL_RCGC2_R;           // delay   
  GPIO_PORTF_LOCK_R = 0x4C4F434B;   // 2) unlock PortF PF0  
  GPIO_PORTF_CR_R = 0x1F;           // allow changes to PF4-0       
  GPIO_PORTF_AMSEL_R = 0x00;        // 3) disable analog function
  GPIO_PORTF_PCTL_R = 0x00000000;   // 4) GPIO clear bit PCTL  
  GPIO_PORTF_DIR_R = 0x0E;          // 5) PF4,PF0 input, PF3,PF2,PF1 output   
  GPIO_PORTF_AFSEL_R = 0x00;        // 6) no alternate function
  GPIO_PORTF_PUR_R = 0x10;          // enable pullup resistors on PF4       
  GPIO_PORTF_DEN_R = 0x1F;          // 7) enable digital pins PF4-PF0        
  GPIO_PORTF_PUR_R |= 0x10;     //     enable weak pull-up on PF4

	// interrupt setup
  GPIO_PORTF_IS_R &= ~0x10;     // (d) PF4 is edge-sensitive
  GPIO_PORTF_IBE_R |= 0x10;    //     PF4 is both edges
  GPIO_PORTF_ICR_R = 0x10;      // (e) clear flag4
  GPIO_PORTF_IM_R |= 0x10;      // (f) arm interrupt on PF4
  NVIC_PRI7_R = (NVIC_PRI7_R&0xFF00FFFF)|0x00A00000; // (g) priority 5
  NVIC_EN0_R = 0x40000000;      // (h) enable interrupt 30 in NVIC
}

void GPIOPortF_Handler(void){
  unsigned long In;  // input from PF4
	
  GPIO_PORTF_ICR_R = 0x10;      // acknowledge flag4
	
	In = GPIO_PORTF_DATA_R&0x10; // read PF4:sw1 into In
  if((In == 0x00)&& (current_s==RELEASED)){ // zero means SW1 is pressed
		current_s=PRESSED;
	}	
			
	if ((In == 0x10)&& (current_s==PRESSED)){		
		current_s=RELEASED;
		GPIO_PORTF_DATA_R ^= 0x02;  // toggle red LED
	}
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
