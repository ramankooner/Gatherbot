// Servo Motor - Robotic Arm: Created 7/21/2019
// Added UART Communication Protocol - 9/12/2019
// Added Robotic Arm Movement - 9/29/2019
// Test
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

// Motor PID Control
// Consider: 24 0 15 / 24 0 0 / 12 0 0 / 21 0.009 0.00009
// 90 .0009 45
#define Kp 90
#define Ki 0.0009
#define Kd 45

// Drop Off PID Control
#define dKp 1500
#define dKi 0
#define dKd 0

// GPIO & Miscellaneous Functions
void EnableInterrupts(void);

// PID Loop for Centering a Ball
float controlLoop(float setPoint, float processVariable);
float controllerLoop(float setPoint, float processVariable);
void motorPIDcontrol(float motorPIDOutput);
void distancePIDcontrol(float distanceOut);

// PID Loop for Drop Off Location
float dropOffControlLoop(float dSP, float dPV);
void dropOffControl(float dOutput);
	
unsigned char n;
int i,k,j,g;

// ARM Variables
float pickUpValue;
int xValue;
int pickUpFlag;
float leftSpeed, rightSpeed;

// UART Variables
int uartFlag;
char buffer[11];
char distanceBuffer[10];
int check_value, check_sum;
int finalXCoordinateValue, finalYCoordinateValue;
int dFinalX, dFinalDistance;
int finalDistance;
int dFinalDistance;
int checkDisplay;
int command;

// Motor Control
int leftPWMSpeed;
int rightPWMSpeed;
float motorSpeed, distanceSpeed; 

// Drop Off Control
int dLeftPWMSpeed;
int dRightPWMSpeed;
float dMotorSpeed;
int ballCount;

enum state {
	SEARCH_BALL,
	APPROACH_BALL,
	STOP_CAR,
	ADJUST_DISTANCE,
	STOP_CAR2,
	GET_DISTANCE,
	PICK_UP,
	CHECK_COUNT,
	SEARCH_DROPOFF,
	APPROACH_DROPOFF,
	DROPOFF_STOP,
	BACKUP_DROPOFF
} state;
	
int main(void){
	
	PLL_Init();
	UART_Init();
	PortF_Init(); //On-board LEDs
	PortB_Init(); // Motor Direction Control and Arm Control
	PortD_Init(); // Motor Movement Control
	Nokia5110_Init();
	Nokia5110_Clear();
	GPIO_PORTF_DATA_R = 0x00;
	
	ballCount = 0;
	
	
	// ARM MOVEMENT
	// Initialize Arm
	M0PWM3_Init(15625, 720);      // PB5 - To Center
	Delay2();
	M0PWM0_Init(15625, 400);      // PB6
	Delay2();
	M0PWM1_Init_new(15625, 1800); // PB7 - Reset Height	
	Delay2();
	M0PWM2_Init(15625, 320);      // PB4 - Open hand 
	
	// EXECUTE ROBOTIC ARM MOVEMENT 
	// X-coordinate value - TESTING
	//xValue = 80;
	
	// Executes the Pick Up movement only - TESTING
	//pickUpValue = armPickUpLocation(xValue);
	
	//pickUp(pickUpValue);
	
	
	// MOTOR CONTROL
	
	// NOTE - CHANGE THIS TO WORK WITH PD0 AND PD1
	//      - CHANGE DIRECTION CONTROL TO PD2,PD3,PD4,PD5
	
	// Initialize Motors
	M0PWM6_Init(15625, 3);
	M0PWM7_Init(15625, 3);
	
	// Control the Direction of the Motors
	// 0x05 - Backwards
	// 0x0A - Forward
	GPIO_PORTB_DATA_R = 0x0A;

	// GREEN COLOR FOR POWER CHECK
	GPIO_PORTF_DATA_R = 0x02;
	
	// UART Flag
	uartFlag = 1;
	pickUpFlag = 0;
	ballCount = 0;
	
	while(1) {
		
		// UART COMMUNICATION
		if (uartFlag == 1) {
			GPIO_PORTB_DATA_R = 0x0A;
			n = UART_InChar();
			
			//PACKAGE: START | COMMAND | XCOORD1 | XCOORD0 | YCOORD1 | YCOORD0 | BALL DISTANCE | DROPOFF X1 | DROPOFF X0 | DROPOFF DISTANCE | CHECK SUM
			if (n == 0x41) { //Start of Package
				buffer[0] = n;
				
				for(i = 1; i < sizeof(buffer); i++) { //Fetch rest of package
					n = UART_InChar();
					buffer[i] = n;
				}
				
				// Check for corruption
				check_value = buffer[0] + buffer[1] + buffer[2] + buffer[3] + buffer[4] + buffer[5] + buffer[6]+ buffer[7] + buffer[8] + buffer[9];
				check_sum = check_value & 0x7F;
				if(check_sum == buffer[10]) {
					// Display the check sum from PI as a decimal
					checkDisplay = (int) buffer[10];
					command = (int) buffer[1];
					
					// Convert the X and Y coordinates to Decimal numbers
					finalXCoordinateValue = charToDecimal(buffer[2], buffer[3]);
					finalYCoordinateValue = charToDecimal(buffer[4], buffer[5]);
					
					// Convert Distance to Decimal Number
					finalDistance = singleCharToDecimal(buffer[6]);
					
					// Drop Off
					dFinalX = charToDecimal(buffer[7],buffer[8]);
					dFinalDistance = singleCharToDecimal(buffer[9]);
				
				}
				else {
					uartFlag = 0;
				}
			}
			else {
				uartFlag = 0;
			}
		}
		
		// UART FLAG == 0
		// Error in buffer -- Reset the buffer then start over
		// Don't care about skipping one coordinate
		if (uartFlag == 0) {
			
			// Empty the buffer
			for(k = 0; k < sizeof(buffer); k++) {
				buffer[k] = 0;
			}
			// Set Flag back to 1 to take in data again
			uartFlag = 1;
		}
		
		// START BYTE
		
		Nokia5110_SetCursor(3,4);
		Nokia5110_OutUDec(checkDisplay); 
		Nokia5110_SetCursor(3,5);
		Nokia5110_OutUDec(command);
		
		// FIND DROP OFF
			switch(state) {
				
				case SEARCH_BALL:
				
						
						// TEMP - SEARCH IN PLACE
						// Slow Pivot
							
						// move forward
						Nokia5110_SetCursor(3,3);
						Nokia5110_OutUDec(40);
						if (buffer[1] != 0x41) {
						//	GPIO_PORTB_DATA_R = 0x09;
							for(g = 0; g < 1; g++) {
								M0PWM6_Duty(5200);
								M0PWM7_Duty(3);
								Delay3();
							}
						}
						
						else {
							GPIO_PORTB_DATA_R = 0x0A;
							for (g = 0; g < 2; g++) {
								M0PWM6_Duty(3);
								M0PWM7_Duty(3);
								Delay2();
							}
							state = APPROACH_BALL;
						}
						
							// object avoidance code

					break;
				
				case APPROACH_BALL:
					
					Nokia5110_SetCursor(3,3);
					Nokia5110_OutUDec(50);
					GPIO_PORTF_DATA_R = 0x08;
		//			if (buffer[1] == 0x41) {
					if (finalDistance > 20) {
						motorSpeed = controlLoop(80, finalXCoordinateValue);
						motorPIDcontrol(motorSpeed);
					}
					else {
						state = STOP_CAR;
					}
			//			if (finalDistance < 20) {
			//				state = STOP_CAR;
		//				}
	//				}
					
					break;
				
				case STOP_CAR:
					GPIO_PORTF_DATA_R = 0x0A;
					M0PWM6_Duty(3);
					M0PWM7_Duty(3);
					for (g = 0; g < 3; g++) {
						Delay2();
					}
					
					state = ADJUST_DISTANCE;
					
					break; 
					
				case ADJUST_DISTANCE:
					GPIO_PORTF_DATA_R = 0x04;
					distanceSpeed = controllerLoop(17, finalDistance);
			
					distancePIDcontrol(distanceSpeed + 2);
					
					if (finalDistance >=15 && finalDistance <= 17) {
						state = STOP_CAR2;
					} 
					
					break;
				
				case STOP_CAR2:
					GPIO_PORTF_DATA_R = 0x02;
					M0PWM6_Duty(3);
					M0PWM7_Duty(3);
					for (g = 0; g < 3; g++) {
						Delay2();
					}
					
					state = GET_DISTANCE;
					
					break;
					
				case GET_DISTANCE:
					
					if (finalDistance >=15 && finalDistance <= 17) {
						state = PICK_UP;
					}
					else {
						state = ADJUST_DISTANCE;
					}
					
					break;
				
				case PICK_UP:
					// Executes the Pick Up movement 
					GPIO_PORTF_DATA_R = 0x0C;
			
					pickUpValue = armPickUpLocation(finalXCoordinateValue);
					armMovement(pickUpValue);
					
					ballCount++;
					state = CHECK_COUNT;
				
					break;
				
				case CHECK_COUNT:
					if(ballCount == 2){
						state = SEARCH_DROPOFF;
					}
					else {
						state = SEARCH_BALL;
					}
					break;
					
				
				case SEARCH_DROPOFF:
					GPIO_PORTF_DATA_R = 0x04;
					if (buffer[1] != 0x45) {
						//	GPIO_PORTB_DATA_R = 0x09;
							for(g = 0; g < 1; g++) {
								M0PWM6_Duty(5200);
								M0PWM7_Duty(3);
								Delay3();
							}
						}
						
						else {
							GPIO_PORTB_DATA_R = 0x0A;
							for (g = 0; g < 2; g++) {
								M0PWM6_Duty(3);
								M0PWM7_Duty(3);
								Delay2();
							}
							state = APPROACH_DROPOFF;
						}
					break;
				
				case APPROACH_DROPOFF:
					
					GPIO_PORTF_DATA_R = 0x08;
				
					if (dFinalDistance > 20) {
						motorSpeed = controlLoop(80, dFinalX);
						motorPIDcontrol(motorSpeed);
					}
					else {
						state = DROPOFF_STOP;
					}
					
				case DROPOFF_STOP:
					GPIO_PORTF_DATA_R = 0x0A;
					M0PWM6_Duty(3);
					M0PWM7_Duty(3);
					for (g = 0; g < 3; g++) {
						Delay2();
					}
					
					state = BACKUP_DROPOFF;
					break;
				
				case BACKUP_DROPOFF:
					
					dropOffMovement();
					ballCount = 0;
					state = SEARCH_BALL;
				
					break;	
			}
		}	
	
}

/*
PID Notes from Eric:

Camera Feedback for Process Variable: 0 to 160.
Desired Set Point: 80
Process Variable is data read from UART

*/


// PID CONTROL FOR CENTERING BALL
float controlLoop(float setPoint, float processVariable) {
	
	static float preError = 0;
	static float integralControl = 0;
	static float prevIntegral = 0;
	float error;
	float derivativeControl;
	float outputControl;
	
	error = setPoint - processVariable;
	
	// Integral Control
	prevIntegral = integralControl;
	integralControl = integralControl + error*Ki;
	
	// Overflow Check 
	if((prevIntegral > 0) && (error > 0) && (integralControl < 0)){
		integralControl = prevIntegral;
	}
  if((prevIntegral < 0) && (error < 0) && (integralControl > 0)){
		integralControl = prevIntegral;
	}
	
	// Derivative Control
	derivativeControl = error - preError;
	
	// Output
	// Output should be a ratio of the two PWMs
	outputControl = (error * Kp) + (integralControl) + (derivativeControl * Kd);
	
	preError = error;

	return outputControl;
	
}

float controllerLoop(float setPoint, float processVariable) {
	
	static float prevDError = 0;
	static float prevProportional = 0;
	static float proportionalControl = 0;
	float derror;
	float outputDControl;
	
	prevProportional = proportionalControl;
	derror = setPoint - processVariable;
	proportionalControl = derror * dKp;
	
	// Overflow Check 
	if((prevDError > 0) && (derror > 0) && (prevProportional < 0)){
		proportionalControl = prevProportional;
	}
  if((prevDError < 0) && (derror < 0) && (prevProportional > 0)){
		proportionalControl = prevProportional;
	}

	// Output
	// Output should be a ratio of the two PWMs
	outputDControl = proportionalControl;

	return outputDControl;
	
}

void distancePIDcontrol(float distanceOut) {
	
	
	if (distanceOut < 0) {
		distanceOut = distanceOut * (-1);
		GPIO_PORTB_DATA_R = 0x0A;
	}
	else {
		distanceOut = distanceOut;
		GPIO_PORTB_DATA_R = 0x05;
	}
	
	leftSpeed = floor(distanceOut);
	rightSpeed = floor(distanceOut);
	
	if(leftSpeed < 0) leftSpeed = 2;
	else if (leftSpeed > 15000) leftSpeed = 6000;
	if(rightSpeed < 0) rightSpeed = 2;
	else if (rightSpeed > 15000) rightSpeed = 6000;	
	
	
	
	// Update Speeds
	M0PWM6_Duty(leftSpeed);
	M0PWM7_Duty(rightSpeed);
}

void motorPIDcontrol(float motorPIDOutput) {
	
	float leftMotorSpeed;
	float rightMotorSpeed;
	
	// The Motors are going at 51% duty and will change based on PID Output
	leftMotorSpeed = 6000 - motorPIDOutput;
	rightMotorSpeed = 6000 + motorPIDOutput;
	
	if(leftMotorSpeed < 0) leftMotorSpeed = 2;
	else if (leftMotorSpeed > 15000) leftMotorSpeed = 15000;
	if(rightMotorSpeed < 0) rightMotorSpeed = 2;
	else if (rightMotorSpeed > 15000) rightMotorSpeed = 15000;	
	
	// Get the Floor of the Float Values
	// Send these Values to the Motor PWMs
	leftPWMSpeed = floor(leftMotorSpeed);
	rightPWMSpeed = floor(rightMotorSpeed);
	
	// Update Speeds
	M0PWM6_Duty(leftPWMSpeed);
	M0PWM7_Duty(rightPWMSpeed);
	
	// DISPLAY SPEEDS ON LCD
	Nokia5110_SetCursor(3,0);
	Nokia5110_OutUDec(leftPWMSpeed);
	Nokia5110_SetCursor(3,1);
	Nokia5110_OutUDec(rightPWMSpeed);
}


