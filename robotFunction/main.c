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

// Consider: 24 0 15 / 24 0 0 / 12 0 0 / 21 0.009 0.00009
#define Kp 21
#define Ki 0.009
#define Kd 0.00009

// GPIO & Miscellaneous Functions
void EnableInterrupts(void);
float controlLoop(float setPoint, float processVariable);
void motorPIDcontrol(float motorPIDOutput);
	
unsigned char n;
int i,k;

// ARM Variables
float pickUpPWM;
int xValue;

// UART Variables
int uartFlag;
char buffer[8];
//char buffer[7];
int check_value, check_sum;
int finalXCoordinateValue, finalYCoordinateValue;
int finalDistance;
int checkDisplay;

// Motor Control
int leftPWMSpeed;
int rightPWMSpeed;
float motorSpeed; 
	
int main(void){
	
	PLL_Init();
	UART_Init();
	PortF_Init(); //On-board LEDs
	//PortB_Init(); // Motor Direction Control 
	//PortD_Init();
	Nokia5110_Init();
	Nokia5110_Clear();
	GPIO_PORTF_DATA_R = 0x00;
	
	// ARM MOVEMENT
	// Initialize Arm
	M0PWM3_Init(15625, 720); //PB5 - To Center
	Delay2();
	M0PWM0_Init(15625, 400); // PB6
	Delay2();
	M0PWM1_Init_new(15625, 1800); //PB7 - Reset Height	
	Delay2();
	M0PWM2_Init(15625, 320); // PB4 - Open hand 
	
	M0PWM3_Init(15625, 720);
	
	// X-coordinate value
	xValue = 492;
	
	// EXECUTE ROBOTIC ARM MOVEMENT
	pickUpPWM = (-xValue/3) + 753;
	
	pickUp((int) pickUpPWM);

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
	
	
	/* DC MOTOR TEMPORARY
	// PB6
	//M0PWM0_Init(15625, 6000);

	// PB7 
	//M0PWM1_Init_new(15625, 6000);

	// Control the Direction of the Motors
	// 0x05 - Backwards
	// 0x0A - Forward
	GPIO_PORTB_DATA_R = 0x0A;
	*/
	
	// GREEN COLOR FOR POWER CHECK
	//GPIO_PORTF_DATA_R = 0x08;
	
	// UART Flag
	uartFlag = 1;
	
	while(1) {
		// UART COMMUNICATION
		if (uartFlag == 1) {
			n = UART_InChar();
			if (n == 0x41) {
				buffer[0] = n;
				for(i = 1; i < sizeof(buffer); i++) {
					n = UART_InChar();
					buffer[i] = n;
				}
				check_value = buffer[0] + buffer[1] + buffer[2] + buffer[3] + buffer[4] + buffer[5] + buffer[6];
				check_sum = check_value & 0x7F;
				if(check_sum == buffer[7]) {
					//GPIO_PORTF_DATA_R = 0x08;
					// Display the check sum from PI as a decimal
					checkDisplay = (int) buffer[7];
					// Convert the X and Y coordinates to Decimal numbers
					finalXCoordinateValue = charToDecimal(buffer[2], buffer[3]);
					finalYCoordinateValue = charToDecimal(buffer[4], buffer[5]);
					// Convert Distance to Decimal Number
					finalDistance = singleCharToDecimal(buffer[6]);
				}
				else {
					//GPIO_PORTF_DATA_R = 0x02;
					uartFlag = 0;
				}
			}
			else {
				//GPIO_PORTF_DATA_R = 0x04;
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
		
		/*
		// Execute PID Loop if a ball is in view of the camera
		if (finalDistance > 12){
			motorSpeed = controlLoop(300, finalXCoordinateValue);
			motorPIDcontrol(motorSpeed);
		}
		else{
			// Update Speeds
			M0PWM0_Duty(3);
			M0PWM1_Duty_new(3);
			GPIO_PORTF_DATA_R = 0x02;
		}
		*/
		
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
	if (error == 0) GPIO_PORTF_DATA_R = 0x0A;
	else GPIO_PORTF_DATA_R = 0x0C;
	
	/*
	if((error >= -40) && (error < 40)) {
		outputControl = 0;
	}
	*/
	
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
