#include "stdlib.h"
#include "stdio.h" 
#include "Utils.h"
#include <ArduinoTrace.h>

int* declareIntArray(int size) {
    return (int*)malloc(size * sizeof(int));
}

char* declareCarArray(int size) {
    return (char*)malloc(size * sizeof(char));
}

int numberDigits(int input) {
    int count = 0;
    while (input != 0) {
        input /= 10;     // n = n/10
        ++count;
    }

    if (count == 0) { //input must be 0, no ints having 0 digits
        count = 1;
    }

    return count;
}


char* intToString(int in, int digitsToPrint, char* str) {
    int digits = numberDigits(in);

    // Serial.print(in);
    int leading0s = digitsToPrint - digits;
    str = (char*)malloc(sizeof(char) * (digitsToPrint + 1));

    if (leading0s <= 0) {
        sprintf(str, "%d", in);
    }
    else {

        char* leading0sChar = (char*)malloc((leading0s + 1) * sizeof(char));
        for (int i = 0; i <= leading0s; i++) { //<= to add line end char
            if (i == leading0s) {
                leading0sChar[i] = 0; //null char at the end
            }
            else {
                leading0sChar[i] = '0';
            }
        }

        sprintf(str, "%s%d", leading0sChar, in);
        free(leading0sChar);
    }
    return str;
}

//void intToString(int in, int digitsToPrint, char** str) {
//    int digits = numberDigits(in);
//
//    // Serial.print(in);
//    int leading0s = digitsToPrint - digits;
//    *str = (char*)malloc(sizeof(char) * (digitsToPrint+1));
//
//
//    if (leading0s <= 0) {
//        TRACE();
//        sprintf(*str, "%d", in);
//    }
//    else {
//        TRACE();
//        char* leading0sChar = (char*)malloc((leading0s + 1) * sizeof(char));
//        TRACE();
//        for (int i = 0; i <= leading0s; i++) { //<= to add line end char
//            if (i == leading0s) {
//                leading0sChar[i] = 0; //null char at the end
//            }
//            else {
//                leading0sChar[i] = '0';
//            }
//        }
//        TRACE();
//        sprintf(*str, "%s%d", leading0sChar, in);
//        TRACE();
//        free(&leading0sChar);
//        TRACE();
//    }
//}