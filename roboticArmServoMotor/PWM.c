#include <stdint.h>
#include "tm4c123gh6pm.h"

// period is 16-bit number of PWM clock cycles in one period (3<=period)
// period for PB6 and PB7 must be the same
// duty is number of PWM clock cycles output is high  (2<=duty<=period-1)
// PWM clock rate = processor clock rate/SYSCTL_RCC_PWMDIV
//                = BusClock/2 
//                = 80 MHz/2 = 40 MHz (in this example)
// Output on PB6/M0PWM0




//********************************************
//****************  PB6  *********************
//***************  M0PWM0 ********************
//********************************************
void M0PWM0_Init(uint16_t period, uint16_t duty){
  SYSCTL_RCGCPWM_R |= 0x01;             // 1) activate PWM0
  SYSCTL_RCGCGPIO_R |= 0x02;            // 2) activate port B
  while((SYSCTL_PRGPIO_R&0x02) == 0){};
  GPIO_PORTB_AFSEL_R |= 0x40;           // enable alt funct on PB6
  GPIO_PORTB_PCTL_R &= ~0x0F000000;     // configure PB6 as PWM0
  GPIO_PORTB_PCTL_R |= 0x04000000;
  GPIO_PORTB_AMSEL_R &= ~0x40;          // disable analog functionality on PB6
  GPIO_PORTB_DEN_R |= 0x40;             // enable digital I/O on PB6
  SYSCTL_RCC_R = 0x00100000 |           // 3) use PWM divider
      (SYSCTL_RCC_R & (~0x000E0000));   //    configure for /2 divider
  PWM0_0_CTL_R = 0;                     // 4) re-loading down-counting mode
  PWM0_0_GENA_R = 0xC8;                 // low on LOAD, high on CMPA down
  // PB6 goes low on LOAD
  // PB6 goes high on CMPA down
  PWM0_0_LOAD_R = period - 1;           // 5) cycles needed to count down to 0
  PWM0_0_CMPA_R = duty - 1;             // 6) count value when output rises
  PWM0_0_CTL_R |= 0x00000001;           // 7) start PWM0
  PWM0_ENABLE_R |= 0x00000001;          // enable PB6/M0PWM0
}
// change duty cycle of PB6
// duty is number of PWM clock cycles output is high  (2<=duty<=period-1)
void M0PWM0_Duty(uint16_t duty){
  PWM0_0_CMPA_R = duty - 1;             // 6) count value when output rises
}


//********************************************
//****************  PB7  *********************
//***************  M0PWM1 ********************
//********************************************
void M0PWM1_Init(uint16_t period, uint16_t duty){
  volatile unsigned long delay;
  SYSCTL_RCGCPWM_R |= 0x01;             // 1) activate PWM0
  SYSCTL_RCGCGPIO_R |= 0x02;            // 2) activate port B
  delay = SYSCTL_RCGCGPIO_R;            // allow time to finish activating
  GPIO_PORTB_AFSEL_R |= 0x80;           // enable alt funct on PB7
  GPIO_PORTB_PCTL_R &= ~0xF0000000;     // configure PB7 as M0PWM1
  GPIO_PORTB_PCTL_R |= 0x40000000;
  GPIO_PORTB_AMSEL_R &= ~0x80;          // disable analog functionality on PB7
  GPIO_PORTB_DEN_R |= 0x80;             // enable digital I/O on PB7
  SYSCTL_RCC_R |= SYSCTL_RCC_USEPWMDIV; // 3) use PWM divider
  SYSCTL_RCC_R &= ~SYSCTL_RCC_PWMDIV_M; //    clear PWM divider field
  SYSCTL_RCC_R += SYSCTL_RCC_PWMDIV_2;  //    configure for /2 divider
  PWM0_0_CTL_R = 0;                     // 4) re-loading down-counting mode
  PWM0_0_GENB_R = (PWM_0_GENB_ACTCMPBD_ONE|PWM_0_GENB_ACTLOAD_ZERO);
  // PB7 goes low on LOAD
  // PB7 goes high on CMPB down
  PWM0_0_LOAD_R = period - 1;           // 5) cycles needed to count down to 0
  PWM0_0_CMPB_R = duty - 1;             // 6) count value when output rises
  PWM0_0_CTL_R |= 0x00000001;           // 7) start PWM0
  PWM0_ENABLE_R |= 0x00000002;          // enable PB7/M0PWM1
}
// change duty cycle of PB7
// duty is number of PWM clock cycles output is high  (2<=duty<=period-1)
void M0PWM1_Duty(uint16_t duty){
  PWM0_0_CMPB_R = duty - 1;             // 6) count value when output rises
}


//********************************************
//****************  PB5  *********************
//***************  M0PWM3 ********************
//********************************************
void M0PWM3_Init(uint16_t period, uint16_t duty) {
	volatile unsigned long delay;
	SYSCTL_RCGCPWM_R |= 0x02;             // Activate PWM0
	SYSCTL_RCGCGPIO_R |= 0x02;            // Activate port B
	delay = SYSCTL_RCGCGPIO_R;
	GPIO_PORTB_AFSEL_R |= 0x20;           // Alt funct on PB5
	GPIO_PORTB_PCTL_R &= 0xFF0FFFFF;      // configure PB5 as M0PWM3
	GPIO_PORTB_PCTL_R |= 0x00400000;
	GPIO_PORTB_DIR_R |= 0x20;             // Set PB5 output
	GPIO_PORTB_AMSEL_R &= ~0x20;          // disable analong funct on PB5
	GPIO_PORTB_DEN_R |= 0x20;             // enable digital I/O on PB5
	
	SYSCTL_RCGCPWM_R |= 0x01;             // Active PWM0	
	SYSCTL_RCGCGPIO_R |= 0x02;            // Clock for Port B
	SYSCTL_RCC_R &= ~0x00100000;          
	
	PWM0_1_CTL_R = 0x00;                  // re-loading down
	PWM0_1_GENB_R |= 0x00000C08;          // low on load
	PWM0_1_LOAD_R = period - 1;           // cycles needed to count to 0
	PWM0_1_CMPB_R = duty;                 // count value when output rises
	PWM0_1_CTL_R |= 0x00000001;           // start PWM1
	PWM0_ENABLE_R |= 0x08;                // enable M0PWM3
}

void M0PWM3_Duty(uint16_t duty) {
	PWM0_1_CMPB_R = duty - 1;
}


//********************************************
//****************  PA7  *********************
//***************  M1PWM3 ********************
//********************************************
void M1PWM3_Init(uint16_t period, uint16_t duty) {
	volatile unsigned long delay;
	SYSCTL_RCGCPWM_R |= 0x02;             // Activate PWM1
	SYSCTL_RCGCGPIO_R |= 0x01;            // Activate port A
	delay = SYSCTL_RCGCGPIO_R;
	GPIO_PORTA_AFSEL_R |= 0x80;           // Alt funct on PA7
	GPIO_PORTA_PCTL_R &= 0x0FFFFFFF;     // configure PA7 as M1PWM3
	GPIO_PORTA_PCTL_R |= 0x50000000;
	GPIO_PORTA_DIR_R |= 0x80;             // Set PA7 output
	GPIO_PORTA_AMSEL_R &= ~0x80;          // disable analong funct on PA7
	GPIO_PORTA_DEN_R |= 0x80;             // enable digital I/O on PA7
	
	SYSCTL_RCGCPWM_R |= 0x02;             // Active PWM1	
	SYSCTL_RCGCGPIO_R |= 0x01;            // Clock for Port A
	SYSCTL_RCC_R &= ~0x00100000;          
	
	PWM1_1_CTL_R = 0x00;                  // re-loading down
	PWM1_1_GENB_R |= 0x00000C08;          // low on load
	PWM1_1_LOAD_R = period - 1;              // cycles needed to count to 0
	PWM1_1_CMPB_R = duty;                  // count value when output rises
	PWM1_1_CTL_R |= 0x00000001;           // start PWM1
	PWM1_ENABLE_R |= 0x08;                // enable M1PWM3
}

void M1PWM3_Duty(uint16_t duty) {
	PWM1_1_CMPB_R = duty - 1;
}

