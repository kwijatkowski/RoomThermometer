#include "stdlib.h"
#include "stdio.h" 
#include <ArduinoTrace.h>

int* declareIntArray(int size);
char* declareCarArray(int size);
int numberDigits(int input);
char* intToString(int in, int digitsToPrint, char* str);