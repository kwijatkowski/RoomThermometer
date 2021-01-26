//#ifndef _UTILS_h
//#define _UTILS_h
//
//#if defined(ARDUINO) && ARDUINO >= 100
//#include "arduino.h"
//#else
//#include "WProgram.h"
//#endif
//#endif

#include "stdlib.h"
#include "stdio.h" 

int* declareIntArray(int size);
char* declareCarArray(int size);
int numberDigits(int input);
char* intToString(int in, int digitsToPrint);