// Servo Motor - Robotic Arm: Created 7/21/2019
// Added UART Communication Protocol - 9/12/2019
// Added Robotic Arm Movement - 9/29/2019

#include "tm4c123gh6pm.h"
#include "PWM.h"
#include "portInitializations.h"
#include "UART.h"
#include "Nokia5110.h"
#include "uartCommunication.h"
#include "PLL.h"
#include "robotArmMovement.h"
#include "delayFunctions.h"
#include <stdio.h>
#include <math.h>

#define Kp 0.1
#define Ki 0.1
#define Kd 0.1

#define dt 0.01 // Execution Time of the Loop

// GPIO & Miscellaneous Functions
void EnableInterrupts(void);
float controlLoop(float setPoint, float processVariable);
void motorPIDcontrol(float motorPIDOutput);
	
unsigned char n;
int i;

// UART Variables
char buffer[7];
int check_value, check_sum;
int finalXCoordinateValue, finalYCoordinateValue;
int checkDisplay;

// Motor Control
int leftPWMSpeed;
int rightPWMSpeed;
float motorSpeed; 
	
int main(void){
	
	PLL_Init();
	UART_Init();
	PortF_Init(); //On-board LEDs
	PortB_Init(); // Motor Direction Control 
	PortD_Init();
	GPIO_PORTF_DATA_R = 0x00;
	
	/*
	// ARM MOVEMENT
	
	// Initialize Arm
	M0PWM3_Init(15625, 720); //PB5 - To Center
	Delay2();
	M0PWM0_Init(15625, 400); // PB6
	Delay2();
	M0PWM1_Init_new(15625, 1800); //PB7 - Reset Height	
	
	// EXECUTE ROBOTIC ARM MOVEMENT
	armMovement();
	*/
		
	// NOTE - CHANGE THIS TO WORK WITH PD0 AND PD1
	//      - CHANGE DIRECTION CONTROL TO PD2,PD3,PD4,PD5
	
	// PD0
	//M0PWM6_Init(15625, 14000);
	
	// PD1
	//M0PWM7_Init(15625, 14000);
	
	// PORT D
	// PD2-PD5
	// 0x14 - Forward
	// 0x28 - Backwards
	// GPIO_PORTD_DATA_R = 0x14;
	
	// PB6
	M0PWM0_Init(15625, 14000);

	// PB7 
	M0PWM1_Init_new(15625, 14000);

	// Control the Direction of the Motors
	// 0x05 - Forward
	// 0x0A - Backwards
	GPIO_PORTB_DATA_R = 0x05;
	
	
	// GREEN COLOR FOR POWER CHECK
	GPIO_PORTF_DATA_R = 0x08;
		
	while(1) {
		
		/*
		// UART COMMUNICATION
		for ( i = 0; i < sizeof(buffer); i++) {
			n = UART_InChar();
			
			buffer[i] = n;
		}
		
		check_value = buffer[0] + buffer[1] + buffer[2] + buffer[3] + buffer[4] + buffer[5];
		
		check_sum = check_value & 0x7F;
		
		// Data is good
		if (check_sum == buffer[6]) {
			GPIO_PORTF_DATA_R = 0x08;
			
			if (buffer[0] == 0x41) {
				// Display the check sum from PI as a decimal
				checkDisplay = (int) buffer[6];
		
				// Convert the X and Y coordinates to Decimal numbers
				finalXCoordinateValue = charToDecimal(buffer[2], buffer[3]);
		
				finalYCoordinateValue = charToDecimal(buffer[4], buffer[5]);
			}
		} 
		
		// Data is corrupt
		else {
			GPIO_PORTF_DATA_R = 0x02; 
		}
		
		Nokia5110_SetCursor(3,0);
		Nokia5110_OutUDec(finalXCoordinateValue);
		
		Nokia5110_SetCursor(3,1);
		Nokia5110_OutUDec(finalYCoordinateValue);
		
		Nokia5110_SetCursor(3,2);
		Nokia5110_OutChar(buffer[0]);
		
		Nokia5110_SetCursor(3,3);
		Nokia5110_OutChar(buffer[1]);
		
		Nokia5110_SetCursor(3,4);
		Nokia5110_OutUDec(checkDisplay); */
		
		
		// Execute PID Loop if a ball is in view of the camera
		motorSpeed = controlLoop(320, finalXCoordinateValue);
		motorPIDcontrol(motorSpeed);
	}
}

float controlLoop(float setPoint, float processVariable) {
	
	static float preError = 0;
	static float integralControl = 0;
	float error;
	float derivativeControl;
	float outputControl;
	
	error = setPoint - processVariable;
	
	// Integral Control
	integralControl = integralControl + error;
	
	// Derivative Control
	derivativeControl = error - preError;
	
	// Output
	// Output should be a ratio of the two PWMs
	outputControl = (error * Kp) + (integralControl * Ki) + (derivativeControl * Kd);
	
	preError = error;
	
	return outputControl;
}

void motorPIDcontrol(float motorPIDOutput) {
	
	float leftMotorSpeed;
	float rightMotorSpeed;
	
	// The Motors are going at 51% duty and will change based on PID Output
	leftMotorSpeed = 8000 - motorPIDOutput;
	rightMotorSpeed = 8000 + motorPIDOutput;
	
	// Get the Floor of the Float Values
	// Send these Values to the Motor PWMs
	leftPWMSpeed = floor(leftMotorSpeed);
	rightPWMSpeed = floor(rightMotorSpeed);
	
	//M0PWM6_Duty(leftPWMSpeed);
	//M0PWM7_Duty(rightPWMSpeed);
}


