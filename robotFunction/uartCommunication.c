#include "stdio.h"
#include "stdlib.h"
#include "math.h"
#include "string.h"

long long decToBinary(int n) {
	long long binaryNumber = 0;
	int remainder, i = 1;
	
	while (n != 0) {
		remainder = n%2;
		n /= 2;
		binaryNumber += remainder*i;
		i *= 10;
	}
	return binaryNumber;
}

int binaryToDecimal(long long n) {
	int decimalNumber = 0, i = 0, remainder;
	
	while(n != 0) {
		remainder = n%10;
		n /= 10;
		decimalNumber += remainder*pow(2, i);
		++i;
	}
	return decimalNumber;
}

int concat(int a, int b) 
{ 
    char s1[20]; 
    char s2[20]; 
		int c;
    // Convert both the integers to string 
    sprintf(s1, "%07d", a); 
    sprintf(s2, "%07d", b); 
  
    // Concatenate both strings 
    strcat(s1, s2); 
  
    // Convert the concatenated string 
    // to integer 
    c = atoi(s1); 
  
    // return the formed integer 
    return c; 
} 

int charToDecimal(char coordinate1, char coordinate0) {
	int cord1, cord0;
	long cordBin1, cordBin0;
	
	unsigned int concatenatedCord;
	
	int finalValue;
	
	// Decimal Values of the x-coordinates
	cord1 = (int) coordinate1; // upper
	cord0 = (int) coordinate0; // lower 
	
	// Binary Value of the x-coordinates
	cordBin1 = decToBinary(cord1);
	cordBin0 = decToBinary(cord0);
		
	// Concantenated x-coordinate value
	concatenatedCord = concat(cordBin1, cordBin0);
		
	// Decimal X-coordinate value
	finalValue = binaryToDecimal(concatenatedCord);
	
	return finalValue;
}
