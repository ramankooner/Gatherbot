// Servo Motor - Robotic Arm: Created 7/21/2019

#include "tm4c123gh6pm.h"
#include "PWM.h"
#include "portInitializations.h"

// GPIO & Miscellaneous Functions
void EnableInterrupts(void);

int main(void){

	PortF_Init(); //On-board LEDs
	GPIO_PORTF_DATA_R = 0x02;
	M0PWM3_Init(40000, 10000); //PB5
	
	while(1){
		GPIO_PORTF_DATA_R = 0x08;
	}
}

