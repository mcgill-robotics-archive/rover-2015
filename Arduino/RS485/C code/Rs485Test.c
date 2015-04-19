#include "Rs485Test.h"
#include "helper.h"
#include <stdio.h>

int main () {

    short address = 102;
    short function = 200;
    uint argumentLo = 20;
    uint argumentMid1 = 150;
    uint argumentMid2 = 99;
    uint argumentHi = 67;
    uint argument = ((argumentLo << 24) + (argumentMid1<<16) + (argumentMid2 <<8) + argumentHi);
    uint termination = 255;


    uint messageLo = (address << 24) + (function << 16) + (argument >> 16);
    uint messageHi = 0b11101111111011111111111111111111;



    // create char buffers for printing binary representations
    long_buffer[LONG_BUFF] = '\0';
    int_buffer[INT_BUFF] = '\0';

    toBin(messageLo, long_buffer, LONG_BUFF);
    printf("messageLo = %d = 0b%s \n" , messageLo, long_buffer);

    short array[4];
    short_buffer[SHORT_BUFF] = '\0';
    splitLong(messageLo, array, 4);
    for (int i = 0; i < 4; i++){
        toBin(array[i], short_buffer, SHORT_BUFF);
        printf("Part %d: %d = 0b%s\n", i, array[i],short_buffer);
    }

    uint intArray[] = {messageLo,messageHi};
    uint getAddress = (intArray[0] & 0b11111111000000000000000000000000) >> 24;
    uint getFunction = (intArray[0] & 0b00000000111111110000000000000000) >> 16;
    uint getArgumentLo = (intArray[0] & 0b00000000000000001111111100000000) >> 8;
    uint getArgumentMid1 = (intArray[0] & 0b00000000000000000000000011111111);

    toBin(getAddress, int_buffer, INT_BUFF);
    printf("getAddress = %d = 0b%s \n" , getAddress, int_buffer);
    printf("getFunction = %d \n", getFunction);
    printf("getArgumentLo = %d \n", getArgumentLo);
    printf("getArgumentMid1 = %d \n", getArgumentMid1);

    printf("header: ");
    int header = buildHeader(address,function);

    toBin(header, int_buffer, INT_BUFF);
    printf("0b%s \n", int_buffer);
    return 0;
}