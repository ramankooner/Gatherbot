#include <stdint.h>
// These functions will be used to control the robotic arm

// Reset the arm to its original position in the center of the robot
void resetArm(void);

// Drop the ball into the PVC pipe drop off area
void dropArm(void);

// Pick up the ball from the ground
// Ball will be about 6 inches from center of the car
void pickUp(void);


void increasePWM(uint16_t x1, uint16_t x2, void (*f)(uint16_t));
void decreasePWM(uint16_t x1, uint16_t x2, void (*f)(uint16_t)); 
