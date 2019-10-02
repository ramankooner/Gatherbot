#include "delayFunctions.h"
#include "PWM.h"

// ROBOTIC ARM MOVEMENT FUNCTIONS 

void resetArm(void){
	Delay2();
	M0PWM3_Duty(720); //PB5 - To Center
	Delay2();
	Delay2();
	M0PWM0_Duty(400); 
	Delay2();
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
	Delay2();
	M1PWM3_Duty(320);  //Hand drops ball
}

void pickUp(void){
	Delay2();
	M0PWM3_Duty(700); //Center
	Delay2();
	M1PWM3_Duty(320); //Open hand
	Delay2();
	M0PWM0_Duty(400); //Reset Joint 2 PB6
	Delay2();
	M0PWM1_Duty_new(1800); // Reset Joint 3 PB7
	Delay2();
	M0PWM0_Duty(850); // Joint 2 Stage 1
	Delay2();
	M0PWM1_Duty_new(1500); // Joint 3 Stage 1
	Delay2();
	M0PWM0_Duty(1050);
	Delay2();
	M0PWM1_Duty_new(1150); // Joint 3 Stage 2
	Delay2();
	Delay2();
	M0PWM0_Duty(1250);
	Delay2();
	Delay2();
	M0PWM0_Duty(1375);
	Delay2();
	Delay2();
	M0PWM0_Duty(1450);
	Delay2();
	Delay2();
	M0PWM0_Duty(1500); // Joint 2 Stage 2
	Delay2();
	Delay2();
	Delay2();
	M1PWM3_Init(15625, 550); //Grab 
	Delay2();
	M0PWM0_Duty(850);
	Delay2();
	M0PWM1_Duty_new(1500);
	Delay2();
	M0PWM0_Duty(400);
}
