#include "stdlib.h"
#include "stdio.h" 
#include "Utils.h"

int* declareIntArray(int size) {
    return (int*)malloc(size * sizeof(int));
}

char* declareCharArray(int size) {
    return (char*)malloc(size * sizeof(char));
}

int numberDigits(int input) {
    int count = 0;
    while (input != 0) {
        input /= 10;     // n = n/10
        ++count;
    }
    return count;
}

char* intToString(int in, int digitsToPrint) {
    int digits = numberDigits(in);

    // Serial.print(in);
    int leading0s = digitsToPrint - digits;
    char* str = declareCharArray(digitsToPrint);

    if (leading0s <= 0) {
        sprintf(str, "%d", in);
    }
    else {
        char* leading0sChar = declareCharArray(leading0s);
        for (int i = 0; i < leading0s; i++) {
            leading0sChar[i] = '0';
        }
        sprintf(str, "%s%d", leading0sChar, in);
        free(leading0sChar);
    }
    return str;
}