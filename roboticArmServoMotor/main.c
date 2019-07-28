// Servo Motor - Robotic Arm: Created 7/21/2019

#include "tm4c123gh6pm.h"
#include "PWM.h"
#include "portInitializations.h"

// GPIO & Miscellaneous Functions
void EnableInterrupts(void);

int main(void){

	PortF_Init(); //On-board LEDs
	GPIO_PORTF_DATA_R = 0x02;
	//M0PWM3_Init(40000, 35000); //PB5   SECOND JOINT
	M0PWM3_Init(40000, 35500); // PB5 HAND - 35500 CLOSE/30000 OPEN
	//M1PWM3_Init(40000, 5000); //PA7    THIRD JOINT
	while(1){
		GPIO_PORTF_DATA_R = 0x08;
	}
}

