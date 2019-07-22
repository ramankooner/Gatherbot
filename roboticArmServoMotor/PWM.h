// PWM.h
// Use PWM0/PB6, PWM1/PB7, PWM2/PB5, PWM3/PA7 to generate pulse-width modulated outputs.

#include <stdint.h>

// PB6
void M0PWM0_Init(uint16_t period, uint16_t duty);
void M0PWM0_Duty(uint16_t duty);

// PB7
void M0PWM1_Init(uint16_t period, uint16_t duty);
void M0PWM1_Duty(uint16_t duty);

// PB5
void M0PWM3_Init(uint16_t period, uint16_t duty);
void M0PWM3_Duty(uint16_t duty);

// PA7
void M1PWM3_Init(uint16_t period, uint16_t duty);
void M1PWM3_Duty(uint16_t duty);

