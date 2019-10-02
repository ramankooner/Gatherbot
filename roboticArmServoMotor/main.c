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

// GPIO & Miscellaneous Functions
void EnableInterrupts(void);

unsigned char n;
int i;

char buffer[7];
int check_value, check_sum;
int finalXCoordinateValue, finalYCoordinateValue;
int checkDisplay;

int main(void){
	
	PLL_Init();
	UART_Init();
	PortF_Init(); //On-board LEDs
	GPIO_PORTF_DATA_R = 0x00;
	

	// Initialize Arm
	M0PWM3_Init(15625, 720); //PB5 - To Center
	Delay2();
	M0PWM0_Init(15625, 400); 
	Delay2();
	M0PWM1_Init_new(15625, 1800); //PB7 - Reset Height	
	
	// EXECUTE ROBOTIC ARM MOVEMENT
	pickUp();
	resetArm();
	dropArm();
	resetArm();

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
	}
}



