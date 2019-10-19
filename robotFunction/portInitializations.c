#include "tm4c123gh6pm.h"


// GPIO Port F Initializations
void PortF_Init(void){ 
	volatile unsigned long delay;
  SYSCTL_RCGC2_R |= 0x00000020;     // 1) F clock
  delay = SYSCTL_RCGC2_R;           // delay   
  GPIO_PORTF_LOCK_R = 0x4C4F434B;   // 2) unlock PortF PF0  
  GPIO_PORTF_CR_R = 0x0E;           // allow changes to PF4-0       
  GPIO_PORTF_AMSEL_R = 0x00;        // 3) disable analog function
  GPIO_PORTF_PCTL_R = 0x00000000;   // 4) GPIO clear bit PCTL  
  GPIO_PORTF_DIR_R = 0x0E;          // 5) PF4,PF0 input, PF3,PF2,PF1 output   
  GPIO_PORTF_AFSEL_R = 0x00;        // 6) no alternate function    
  GPIO_PORTF_DEN_R = 0x0E;          // 7) enable digital pins PF4-PF0        
}

// GPIO Port B Initializations
void PortB_Init(void){ 
	volatile unsigned long delay;
  SYSCTL_RCGC2_R |= 0x00000002;     // 1) B clock
  delay = SYSCTL_RCGC2_R;           // delay   
  GPIO_PORTB_CR_R |= 0xCF;           // allow changes to PB3-0    
  GPIO_PORTB_AMSEL_R &= ~0xCF;        // 3) disable analog function
  GPIO_PORTB_PCTL_R &= ~0xFF000000;   // 4) GPIO clear bit PCTL  
  GPIO_PORTB_PCTL_R |= 0x44000000; 
	GPIO_PORTB_DIR_R |= 0x0F;          // 5) PB3-0 Output
  GPIO_PORTB_AFSEL_R |= 0xC0;        // 6) no alternate function    
  GPIO_PORTB_DEN_R |= 0xCF;          // 7) enable digital pins PB3-0       
}

// GPIO Port D Initializations
void PortD_Init(void){ 
	volatile unsigned long delay;
  SYSCTL_RCGC2_R |= 0x00000008;     // 1) D clock
  delay = SYSCTL_RCGC2_R;           // delay   
  GPIO_PORTD_CR_R |= 0xFF;           // allow changes to PD0-PD7  
  GPIO_PORTD_AMSEL_R &= ~0xFF;        // 3) disable analog function
  GPIO_PORTD_PCTL_R &= ~0x000000FF;   // 4) GPIO clear bit PCTL  
  GPIO_PORTD_PCTL_R |= 0x00000044; 
	GPIO_PORTD_DIR_R |= 0xFF;          // 5) PD7-0 Output
  GPIO_PORTD_AFSEL_R |= 0x00;        // 6) no alternate function    
  GPIO_PORTD_DEN_R |= 0xFF;          // 7) enable digital pins PD7-0     
}
