#include "delayFunctions.h"
#include "PWM.h"
#include "tm4c123gh6pm.h"

// JOINT 1 - BOTTOM SERVO - PB5
// JOINT 2 - SECOND SERVO - PB6
// JOINT 3 - THIRD SERVO  - PB7
// JOINT 4 - HAND         - PB4

// GPIO Port F Initializations
void PortF_RobotMovement(void){ 
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

// ROBOTIC ARM MOVEMENT FUNCTIONS 

// X1 = Starting PWM
// X2 = Ending PWM
// (*f) is the duty cycle function you want to control
// This function is used for going from a lower PWM to a higher PWM
void increasePWM(uint16_t x1, uint16_t x2, void (*f)(uint16_t)) {
	uint16_t i;
	
	for(i = x1; i <= x2; i=i+10) {
		(*f)(i);
		Delay3();
	}
}

// X1 = Starting PWM
// X2 = Ending PWM
// (*f) is the duty cycle function you want to control
// This function is used for going from a higher PWM to a lower PWM
void decreasePWM(uint16_t x1, uint16_t x2, void (*f)(uint16_t)) {
	uint16_t i;
	
	for(i = x1; i >= x2; i=i-10) {
		(*f)(i);
		Delay3();
	}
}

void resetArm(void){
	Delay2();
	M0PWM3_Duty(720); //PB5 - To Center
	Delay2();
	M0PWM0_Duty(400); // PB6
	Delay2();
	M0PWM1_Duty_new(1800); //PB7 - Reset Height
}

void dropArm(void){
	Delay2();
	M0PWM1_Duty_new(1800); //PB7 - Reset Height
	Delay2();
	M0PWM3_Duty(255); //PB5 - To Drop Off
	Delay2();
	Delay2();
	M0PWM2_Duty(320);  //Hand drops ball
}

void pickUp(int pickUpValue){
	
	// RESET JOINTS TO RESET POSITIONS
	Delay2();
	M0PWM3_Duty(pickUpValue); //Center PB5
	Delay2();
	M0PWM2_Duty(320); //Open hand PA7
	Delay2();
	M0PWM0_Duty(400); //Reset Joint 2 PB6
	Delay2();
	M0PWM1_Duty_new(1800); // Reset Joint 3 PB7
	Delay2();
	
	
	// START THE MOVEMENT
	increasePWM(400, 850, M0PWM0_Duty); // PB6
	Delay2();
	decreasePWM(1800, 1500, M0PWM1_Duty_new); // PB7
	Delay2();
	increasePWM(850, 1050, M0PWM0_Duty); // PB6
	Delay2();
	decreasePWM(1500, 1150, M0PWM1_Duty_new); // Joint 3 Stage 2 PB7
	Delay2();
	increasePWM(1050, 1250, M0PWM0_Duty); // PB6
	Delay2();
	increasePWM(1250, 1500, M0PWM0_Duty); // PB6
	Delay2();
	
	// PICK UP THE BALL
	GPIO_PORTF_DATA_R = 0x02;
	M0PWM2_Init(15625, 560); //Grab  PA7
	
	// GET ARM READY FOR DROP OFF
	Delay2();
	decreasePWM(1500, 850, M0PWM0_Duty); // PB6
	Delay2();
	increasePWM(1150, 1800, M0PWM1_Duty_new); // PB7
	Delay2();
	decreasePWM(850, 400, M0PWM0_Duty); // PB6
}

float armPickUpLocation(int xCord) {
	float pickUpPWM;
	
	//pickUpPWM = (-xCord/3) + 753;
	//pickUpPWM = (-xCord) + 760;
	//https://www.analyzemath.com/parabola/three_points_para_calc.html <- website for calculation
	pickUpPWM = (0.00620*(xCord*xCord)) - (2.5*xCord) + 790;
	return pickUpPWM;
}


void armMovement(int pickUpCoord) {
	pickUp(pickUpCoord);
	resetArm();
	dropArm();
	resetArm();
}

void dropOffMovement(void) {
	int i;
	// EDIT THIS FUNCTION SO ROBOT DOES A 180 TURN
	GPIO_PORTB_DATA_R = 0x09; //One forward, one backward
	M0PWM6_Duty(8200);
	M0PWM7_Duty(8200);
	for(i = 0; i < 3; i++){
		Delay2();
		GPIO_PORTF_DATA_R = 0x04;
	}
	M0PWM6_Duty(3);
	M0PWM7_Duty(3);
	GPIO_PORTF_DATA_R = 0x08;
	Delay2();
	GPIO_PORTB_DATA_R = 0x05;
	M0PWM6_Duty(6700);
	M0PWM7_Duty(6700);
	for(i = 0; i < 2; i++){
		Delay2();
		GPIO_PORTF_DATA_R = 0x04;
	}
	while(1){
		M0PWM6_Duty(3);
		M0PWM7_Duty(3);
	}
}

/*
void oldPickUp(void) {
	Delay2();
	M0PWM3_Duty(700); //Center PB5
	Delay2();
	M0PWM2_Duty(320); //Open hand PA7
	Delay2();
	M0PWM0_Duty(400); //Reset Joint 2 PB6
	Delay2();
	M0PWM1_Duty_new(1800); // Reset Joint 3 PB7
	Delay2();
	
	// START
	M0PWM0_Duty(850); // Joint 2 Stage 1 PB6
	Delay2();
	M0PWM1_Duty_new(1500); // Joint 3 Stage 1 PB7
	Delay2();
	M0PWM0_Duty(1050); // PB6
	Delay2();
	M0PWM1_Duty_new(1150); // Joint 3 Stage 2 PB7
	Delay2();
	Delay2();
	M0PWM0_Duty(1250); // PB6
	Delay2();
	Delay2();
	M0PWM0_Duty(1375); // PB6
	Delay2();
	Delay2();
	M0PWM0_Duty(1450); // PB6
	Delay2();
	Delay2();
	M0PWM0_Duty(1500); // Joint 2 Stage 2 PB6
	Delay2();
	Delay2();
	Delay2();
	M0PWM2_Init(15625, 550); //Grab  PA7
	Delay2();
	M0PWM0_Duty(850); // PB6
	Delay2();
	M0PWM1_Duty_new(1500); // PB7
	Delay2();
	M0PWM0_Duty(400); // PB6
}
*/
