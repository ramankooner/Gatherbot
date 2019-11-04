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
#define Kp 21
#define Ki 0.009
#define Kd 0.00009

// Drop Off PID Control
#define dKp 0
#define dKi 0
#define dKd 0

// GPIO & Miscellaneous Functions
void EnableInterrupts(void);

// PID Loop for Centering a Ball
float controlLoop(float setPoint, float processVariable);
void motorPIDcontrol(float motorPIDOutput);

// PID Loop for Drop Off Location
float dropOffControlLoop(float dSP, float dPV);
void dropOffControl(float dOutput);
	
unsigned char n;
int i,k;

// ARM Variables
float pickUpValue;
int xValue;

// UART Variables
int uartFlag;
char buffer[8];
int check_value, check_sum;
int finalXCoordinateValue, finalYCoordinateValue;
int finalDistance;
int dFinalDistance;
int checkDisplay;

// Motor Control
int leftPWMSpeed;
int rightPWMSpeed;
float motorSpeed; 

// Drop Off Control
int dLeftPWMSpeed;
int dRightPWMSpeed;
float dMotorSpeed;
int ballCount;
	
int main(void){
	
	PLL_Init();
	UART_Init();
	PortF_Init(); //On-board LEDs
	PortB_Init(); // Motor Direction Control 
	//PortD_Init();
	Nokia5110_Init();
	Nokia5110_Clear();
	GPIO_PORTF_DATA_R = 0x00;
	
	ballCount = 0;
	
	// ARM MOVEMENT
	// Initialize Arm
	/*
	M0PWM3_Init(15625, 720);      // PB5 - To Center
	Delay2();
	M0PWM0_Init(15625, 400);      // PB6
	Delay2();
	M0PWM1_Init_new(15625, 1800); // PB7 - Reset Height	
	Delay2();
	M0PWM2_Init(15625, 320);      // PB4 - Open hand 
	*/
	// EXECUTE ROBOTIC ARM MOVEMENT 
	
	
	// X-coordinate value - TESTING
	//xValue = 492;

	// Executes the Pick Up movement only - TESTING
	//pickUpValue = armPickUpLocation(xValue);
	
	//pickUp((int) pickUpValue);
	
	// Executes the Full Arm Motion
	// armMovement(xValue);



	// MOTOR CONTROL
	
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
	M0PWM0_Init(15625, 6000);
	
	// PB7 
	M0PWM1_Init_new(15625, 6000);

	// Control the Direction of the Motors
	// 0x05 - Backwards
	// 0x0A - Forward
	GPIO_PORTB_DATA_R = 0x0A;
	
	
	// GREEN COLOR FOR POWER CHECK
	GPIO_PORTF_DATA_R = 0x08;
	
	// UART Flag
	uartFlag = 1;
	
	while(1) {
		
		// UART COMMUNICATION
		if (uartFlag == 1) {
			n = UART_InChar();
			
			if (n == 0x41) { //Start of Package
				buffer[0] = n;
				
				for(i = 1; i < sizeof(buffer); i++) { //Fetch rest of package
					n = UART_InChar();
					buffer[i] = n;
				}
				
				// Check for data corruption
				check_value = buffer[0] + buffer[1] + buffer[2] + buffer[3] + buffer[4] + buffer[5] + buffer[6];
				check_sum = check_value & 0x7F;
				if(check_sum == buffer[7]) {
					// Display the check sum from PI as a decimal
					checkDisplay = (int) buffer[7];
					
					// Convert the X and Y coordinates to Decimal numbers
					finalXCoordinateValue = charToDecimal(buffer[2], buffer[3]);
					finalYCoordinateValue = charToDecimal(buffer[4], buffer[5]);
					
					// Convert Distance to Decimal Number
					finalDistance = singleCharToDecimal(buffer[6]);
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
		
		/*
		// DISPLAY BUFFER ON LCD
		Nokia5110_SetCursor(3,0);
		Nokia5110_OutUDec(buffer[0]);
		
		Nokia5110_SetCursor(3,1);
		Nokia5110_OutUDec(buffer[1]);
		*/
		
		
		// START BYTE
		Nokia5110_SetCursor(3,2);
		Nokia5110_OutUDec(finalXCoordinateValue);
		Nokia5110_SetCursor(3,3);
		Nokia5110_OutUDec(finalDistance);
		Nokia5110_SetCursor(3,4);
		Nokia5110_OutUDec(checkDisplay); 
		
		
		// FIND DROP OFF BOX
		if (finalDistance > 17) {
			motorSpeed = controlLoop(300, finalXCoordinateValue);
			motorPIDcontrol(motorSpeed);
			GPIO_PORTF_DATA_R = 0x08;
		}
		else {
			// Update Speeds to Stop Robot
			M0PWM0_Duty(3);
			M0PWM1_Duty_new(3);
			GPIO_PORTF_DATA_R = 0x02;
			Delay2();
			dropOffMovement();
			
		}
		
		/*
		// Execute Drop Off Movement if Ball count is max
		if (ballCount == 5) {
			// Need to implement a searching algorithm
			
			// If drop off location is found then execute PID Loop on it
			if (dFinalDistance > 12) {
				dMotorSpeed = controlLoop(300, finalXCoordinateValue, dKp, dKi, dKd);
				motorPIDcontrol(dMotorSpeed);
			}
			else {
				// Change this value to whatever value we need
				if (dFinalDistance > 8) {
				
					// Ensure robot moves forward
					GPIO_PORTD_DATA_R = 0x14;
					
					M0PWM0_Duty(5000);
					M0PWM1_Duty_new(5000);
				}
				else if (dFinalDistance < 6) {
					
					// Reverse the Car
					GPIO_PORTD_DATA_R = 0x28;
					
					M0PWM0_Duty(5000);
					M0PWM1_Duty_new(5000);
				}
				else if (dFinalDistance == 6) {
					
					// Execute the Drop Off Movement (Turn Around, Empty Out, Close)
					dropOffMovement();
					
					//Delay2();
					
					// Reset the Ball Count
					//ballCount = 0;
					
					// Start Searching for new Ping Pong Balls
				}
				else {
					
					GPIO_PORTF_DATA_R = 0x08;
					M0PWM0_Duty(3);
					M0PWM1_Duty_new(3);
				}
			}
		}
		*/
		
		/*
		// PICKING UP BALLS
		// Execute PID Loop if a ball is in view of the camera
		if (finalDistance > 12) {
			motorSpeed = controlLoop(300, finalXCoordinateValue, Kp, Ki, Kd);
			motorPIDcontrol(motorSpeed);
		}
		
		else {
		
			// Update Speeds
			M0PWM0_Duty(3);
			M0PWM1_Duty_new(3);
			
			GPIO_PORTF_DATA_R = 0x02;
			
			Delay2();
			
			if (finalDistance > 6) {
				
				// Ensure robot moves forward
				
				GPIO_PORTD_DATA_R = 0x14;
				
				M0PWM0_Duty(5000);
				M0PWM1_Duty_new(5000);
			}
			else if (finalDistance < 6) {
				
				// Reverse the Car
				
				GPIO_PORTD_DATA_R = 0x28;
				
				M0PWM0_Duty(5000);
				M0PWM1_Duty_new(5000);
			}
			else if (finalDistance == 6) {
				M0PWM0_Duty(3);
				M0PWM1_Duty_new(3);
				
				// Execute Arm Pick Up Motion
				// pickUpValue = armPickUpLocation(finalXCoordinateValue);
				// armMovement(pickUpValue);
				// Delay2();
				// ballCount++;
			}
			else {
				GPIO_PORTF_DATA_R = 0x08;
				M0PWM0_Duty(3);
				M0PWM1_Duty_new(3);
			}
		}
		*/
		
	}
}

// PID CONTROL FOR CENTERING BALL
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
	if (error == 0) GPIO_PORTF_DATA_R = 0x0A;
	else GPIO_PORTF_DATA_R = 0x04;
	
	return outputControl;
	
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
	
	// DISPLAY BUFFER ON LCD
	Nokia5110_SetCursor(3,0);
	Nokia5110_OutUDec(leftPWMSpeed);
		
	Nokia5110_SetCursor(3,1);
	Nokia5110_OutUDec(rightPWMSpeed);
	
	// Update Speeds
	M0PWM0_Duty(leftPWMSpeed);
	M0PWM1_Duty_new(rightPWMSpeed);
}
