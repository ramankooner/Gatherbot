// Servo Motor - Robotic Arm: Created 7/21/2019
// Added UART Communication Protocol - 9/12/2019
// Added Robotic Arm Movement - 9/29/2019
// Added PID Loop
// Added Drop off Movement
// Added Distance Control
// Added Gate Movement

// Notes from 12/12/19
// Increase Search Speed because robot sometimes does not move
// Fix distance from 18 and 16 because robot does not move
// Sometimes the ball does not go down the pipe - turn the end of the pipe more
// Fix drop off approach and lighting for green color

// Color    LED(s) PortF
// dark     ---    0
// red      R--    0x02
// blue     --B    0x04
// green    -G-    0x08
// yellow   RG-    0x0A
// sky blue -GB    0x0C
// white    RGB    0x0E
// pink     R-B    0x06

#include "tm4c123gh6pm.h"
#include "PWM.h"
#include "portInitializations.h"
#include "UART.h"
#include "uartCommunication.h"
#include "PLL.h"
#include "robotArmMovement.h"
#include "delayFunctions.h"
#include <stdio.h>
#include <math.h>

// Motor PID Control
#define Kp 90
#define Ki 0.0009
#define Kd 45

// Drop Off PID Control
#define dKp 1300 //2000
#define dKi 0
#define dKd 0

// GPIO & Miscellaneous Functions
void EnableInterrupts(void);

// PID Loop for Centering a Ball
float controlLoop(float setPoint, float processVariable);
float controllerLoop(float setPoint, float processVariable);
void motorPIDcontrol(float motorPIDOutput);
void distancePIDcontrol(float distanceOut);

// For Loop Variables
unsigned char n;
int i, k, g;

// ARM Variables
float pickUpValue;

// UART Variables
int uartFlag;
char buffer[11];
int check_value, check_sum;
int finalXCoordinateValue, finalYCoordinateValue;
int dFinalX, dFinalDistance;
int finalDistance, dFinalDistance;
int checkDisplay;

// Motor Control
int leftPWMSpeed, rightPWMSpeed;
float motorSpeed, distanceSpeed;
float dropSpeed, dSpeed;

// Drop Off Control
float dMotorSpeed;
int ballCount;

// Data Sample Variables
int coord_count, buffer_count;

enum state {
	SEARCH_BALL,
	APPROACH_BALL,
	STOP_CAR,
	ADJUST_DISTANCE,
	GET_DISTANCE,
	CHECK_COORD,
	PICK_UP,
	BUFFER_RESET,
	CHECK_COUNT,
	SEARCH_DROPOFF,
	APPROACH_DROPOFF,
	DROPOFF_STOP,
	ADJUST_DROPOFF,
	BACKUP_DROPOFF,
	FINAL_RESET
} state;
	
int main(void){
	
	PLL_Init();
	UART_Init();
	PortF_Init(); //On-board LEDs
	PortB_Init(); // Motor Direction Control and Arm Control
	PortD_Init(); // Motor Movement Control

	// ARM MOVEMENT
	// Initialize Arm
	Delay2();
	M0PWM3_Init(15625, 720);      // PB5 - To Center
	Delay2();
	M1PWM3_Init(15625, 400);      // PA7 - Second Joint
	Delay2();
	M1PWM2_Init(15625, 1800);     // PA6 - Third Joint
	Delay2();
	M0PWM2_Init(15625, 320);      // PB4 - Open hand 
	Delay2();
	M0PWM4_Init(15625, 300);      // PE4 - Close The Gate - 630 = Open, 300 = Close
	Delay2();
	
	// MOTOR CONTROL
	// Initialize Motors
	M0PWM6_Init(15625, 3);
	M0PWM7_Init(15625, 3);
	
	// Control the Direction of the Motors
	// 0x05 - Forward
	// 0x0A - Backward
	GPIO_PORTB_DATA_R = 0x05;

	// No Color on MCU
	GPIO_PORTF_DATA_R = 0x00;
	
	// Flags and States
	uartFlag = 1;
	ballCount = 0;
	buffer_count = 0;
	coord_count = 0;
	state = SEARCH_BALL;
	
	// MCU Start Up Detection
	Delay2();
	GPIO_PORTF_DATA_R = 0x0E;
	Delay2();

	while(1) {
		
		// UART COMMUNICATION
		if (uartFlag == 1) {

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
		if(uartFlag == 0) {
			
			// Empty the buffer
			for(k = 0; k < sizeof(buffer); k++) {
				buffer[k] = 0;
			}
			// Set Flag back to 1 to take in data again
			uartFlag = 1;
		}
		
		// FIND DROP OFF
		switch(state) {
			
			case SEARCH_BALL:
					GPIO_PORTF_DATA_R = 0x02;
					
					// TEMP - SEARCH IN PLACE
					// Slow Pivot
					// move forward
				
					if (buffer[1] != 0x43) { // No ball found
						for(g = 0; g < 1; g++) {
							M0PWM6_Duty(3500);
							M0PWM7_Duty(2);
							Delay();
						}
					}
					else { //Ball found. Pause, then update state.
						for (g = 0; g < 2; g++) {
							M0PWM6_Duty(2);
							M0PWM7_Duty(2);
							Delay2();
						}
						state = APPROACH_BALL;
					}
					
						// object avoidance code

				break;
			
			case APPROACH_BALL:
			
				GPIO_PORTF_DATA_R = 0x08;

				if (finalDistance > 25) {
					motorSpeed = controlLoop(80, finalXCoordinateValue);
					motorPIDcontrol(motorSpeed);
				}
				else {
					M0PWM6_Duty(2);
					M0PWM7_Duty(2);
					state = STOP_CAR;
				}
				
				break;
			
			case STOP_CAR:
				GPIO_PORTF_DATA_R = 0x0A;
			
				M0PWM6_Duty(2);
				M0PWM7_Duty(2);
				for (g = 0; g < 2; g++) {
					Delay2();
				}
				
				state = ADJUST_DISTANCE;
				
				break; 
				
			case ADJUST_DISTANCE:
			
				GPIO_PORTF_DATA_R = 0x04;
								
				distanceSpeed = controllerLoop(17, finalDistance);
		
				distancePIDcontrol(distanceSpeed + 2);
					
				if (finalDistance == 17) {
					state = GET_DISTANCE;
				}

				break;
			
			case GET_DISTANCE:
				
				if(finalDistance == 17){ //Second check for distance == 17
					state = CHECK_COORD;
				}
				else{
					state = ADJUST_DISTANCE;
				}	
				break;
				
			case CHECK_COORD:
				
				if(finalDistance != 17 ){ //Second check for distance.
					state = ADJUST_DISTANCE;
					coord_count = 0;
				}
				
				else {
					if (coord_count == 10){ //10 iterations of UART read (Check X-Coordinate)
						coord_count = 0;
						state = PICK_UP;
					}	
					else{
						coord_count += 1;
						state = CHECK_COORD;
					}
				}
				
				break;
			
			case PICK_UP:
				// Executes the Pick Up movement 
				GPIO_PORTF_DATA_R = 0x06;
				GPIO_PORTB_DATA_R = 0x05;
			
				pickUpValue = armPickUpLocation(finalXCoordinateValue);
				armMovement(pickUpValue);
		
				Delay2();
				ballCount++;

				state = BUFFER_RESET;
			
				break;
			
			case BUFFER_RESET:
				
				if(buffer_count == 10){ //10 iterations of UART read (Flush out excess data samples)
					buffer_count = 0;
					state = CHECK_COUNT;
				}
				else{
					buffer_count += 1;
					state = BUFFER_RESET;
				}
				break;
			
			case CHECK_COUNT:
				
				if(ballCount >= 10) {
					state = SEARCH_DROPOFF;
				}
				else {
					state = SEARCH_BALL;
				}
				break;
			
			case SEARCH_DROPOFF:
				
				GPIO_PORTF_DATA_R = 0x06;
			
				if (buffer[1] != 0x45) {
					GPIO_PORTB_DATA_R = 0x05;

					for(g = 0; g < 1; g++) {
						M0PWM6_Duty(5200);
						M0PWM7_Duty(3);
						Delay3();
					}
					state = SEARCH_DROPOFF;
				}
				
				else {
					GPIO_PORTB_DATA_R = 0x05;
					for (g = 0; g < 2; g++) {
						M0PWM6_Duty(2);
						M0PWM7_Duty(2);
						Delay2();
					}
					state = APPROACH_DROPOFF;
				}
				
				break;
			
			case APPROACH_DROPOFF:
				
				GPIO_PORTF_DATA_R = 0x0E;
			
				Delay2();
				Delay2();
	
				if (dFinalDistance > 65) {
					GPIO_PORTB_DATA_R = 0x05;
					dMotorSpeed = controlLoop(80, dFinalX);
					motorPIDcontrol(dMotorSpeed);
				}
				else {
					state = DROPOFF_STOP;
				}
				
			case DROPOFF_STOP:
				GPIO_PORTF_DATA_R = 0x0C;
			
				M0PWM6_Duty(2);
				M0PWM7_Duty(2);
				for (g = 0; g < 3; g++) {
					Delay2();
				}
				
				state = ADJUST_DROPOFF;
				
				break;
			
			case ADJUST_DROPOFF:
					
				GPIO_PORTF_DATA_R = 0x04;
			
				dropSpeed = controllerLoop(35, dFinalDistance);
			
				distancePIDcontrol(dropSpeed + 2);
			
				if (dFinalDistance >= 33 && dFinalDistance < 37) {
					M0PWM6_Duty(2);
					M0PWM7_Duty(2);
					Delay2();
					state = BACKUP_DROPOFF;
				}
			
				break;
			
			case BACKUP_DROPOFF:
				GPIO_PORTF_DATA_R = 0x0A;
			
				Delay2();
			
				// 180 Degree Turn and Back up car
				dropOffMovement();      
			
				// Set Motor Direction To Forward
				GPIO_PORTB_DATA_R = 0x05;
			
				// Reset Ball Count
				ballCount = 0; 
			
				state = FINAL_RESET; 
			
				break;	
			
			case FINAL_RESET:
				
				if(buffer_count == 10){ //10 iterations of UART read (Flush out excess data samples)
					buffer_count = 0;
					state = SEARCH_BALL;
				}
				else{
					buffer_count += 1;
					state = BUFFER_RESET;
				}
				
				break;
				
		}//End State Machine
		
	}	//End Superloop
	
} //End Main


// PID CONTROL 
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

void motorPIDcontrol(float motorPIDOutput) {
	
	float leftMotorSpeed;
	float rightMotorSpeed;
	
	// The Motors are going at 51% duty and will change based on PID Output
	leftMotorSpeed = 3000 - motorPIDOutput;
	rightMotorSpeed = 3000 + motorPIDOutput;
	
	if(leftMotorSpeed < 0) leftMotorSpeed = 2;
	else if (leftMotorSpeed > 10000) leftMotorSpeed = 10000;
	if(rightMotorSpeed < 0) rightMotorSpeed = 2;
	else if (rightMotorSpeed > 10000) rightMotorSpeed = 10000;	
	
	// Get the Floor of the Float Values
	// Send these Values to the Motor PWMs
	leftPWMSpeed = floor(leftMotorSpeed);
	rightPWMSpeed = floor(rightMotorSpeed);
	
	// Update Speeds
	M0PWM6_Duty(leftPWMSpeed);
	M0PWM7_Duty(rightPWMSpeed);

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
	
	
	// Check if Error == 1
	//      Robot will not move on 2000 PWM so increase PWM
	//            when the Error == 1
	if (derror == 1) {
		proportionalControl = 2800;
	}
	
	// Output
	// Output should be a ratio of the two PWMs
	outputDControl = proportionalControl;

	return outputDControl;
	
}

void distancePIDcontrol(float distanceOut) {
	
	if (distanceOut < 0) {
		distanceOut = distanceOut * (-1);
		
		dSpeed = floor(distanceOut);
		
		if(dSpeed < 0) dSpeed = 2;
		else if (dSpeed > 3000) dSpeed = 3000;
	
		GPIO_PORTB_DATA_R = 0x05;
		
		// Update Speeds
		M0PWM6_Duty(dSpeed);
		M0PWM7_Duty(dSpeed);
	}
	
	else {
		distanceOut = distanceOut;
		
		dSpeed = floor(distanceOut);

		if(dSpeed < 0) dSpeed = 2;
		else if (dSpeed > 3000) dSpeed = 3000;
	
		GPIO_PORTB_DATA_R = 0x0A;
		
		// Update Speeds
		M0PWM6_Duty(dSpeed);
		M0PWM7_Duty(dSpeed);
	}
	
	
}

