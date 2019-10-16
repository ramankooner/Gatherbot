#include "delayFunctions.h"

// DELAY FUNCTIONS 

void Delay(void){ unsigned long volatile time;
  time = 727240*20/91;  // 0.01sec
  while(time){
		time--;
  }
}

void Delay2(void) {
	int i;
	for(i=0; i < 30; i++) {
		Delay();
	}
}

void Delay3(void) {
	int i;
	for(i=0; i < 2; i++) {
		Delay();
	}
}
