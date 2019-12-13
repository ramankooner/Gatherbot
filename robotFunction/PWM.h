// PWM.h
// Use PWM0/PB6, PWM1/PB7, PWM2/PB5, PWM3/PA7 to generate pulse-width modulated outputs.

#include <stdint.h>

// PB6
void M0PWM0_Init(uint16_t period, uint16_t duty);
void M0PWM0_Duty(uint16_t duty);

// PB7 
void M0PWM1_Init_new(uint16_t period, uint16_t duty);
void M0PWM1_Duty_new(uint16_t duty);

// PB4
void M0PWM2_Init(uint16_t period, uint16_t duty);
void M0PWM2_Duty(uint16_t duty);

// PB5
void M0PWM3_Init(uint16_t period, uint16_t duty);
void M0PWM3_Duty(uint16_t duty);

// PE4
void M0PWM4_Init(uint16_t period, uint16_t duty);
void M0PWM4_Duty(uint16_t duty);

// PD0
void M0PWM6_Init(uint16_t period, uint16_t duty);
void M0PWM6_Duty(uint16_t duty);

// PD1
void M0PWM7_Init(uint16_t period, uint16_t duty);
void M0PWM7_Duty(uint16_t duty);

// PA7 
void M1PWM3_Init(uint16_t period, uint16_t duty);
void M1PWM3_Duty(uint16_t duty);

void M1PWM2_Init(uint16_t period, uint16_t duty);
void M1PWM2_Duty(uint16_t duty);
