#include <stdio.h>

typedef unsigned int uint;

int main () {

    uint address = 102;
    uint function = 200;
    uint argumentLo = 20;
    uint argumentMid1 = 150;
    uint argumentMid2 = 99;
    uint argumentHi = 67;
    uint argument = ((argumentLo << 24) + (argumentMid1<<16) + (argumentMid2 <<8) + argumentHi);
    uint termination = 255;
    

    uint messageLo = (address << 24) + (function << 16) + (argument >> 16);
    uint messageHi = 0b11101111111011111111111111111111;

    printf("messageLo = %d \n" , messageLo);

    uint intArray[] = {messageLo,messageHi};
    uint getAddress = (intArray[0] & 0b11111111000000000000000000000000) >> 24;
    uint getFunction = (intArray[0] & 0b00000000111111110000000000000000) >> 16;
    uint getArgumentLo = (intArray[0] & 0b00000000000000001111111100000000) >> 8;
    uint getArgumentMid1 = (intArray[0] & 0b00000000000000000000000011111111);

    printf("getAddress = %d \n" , getAddress);
    printf("getFunction = %d \n", getFunction);
    printf("getArgumentLo = %d \n", getArgumentLo);
    printf("getArgumentMid1 = %d \n", getArgumentMid1);

    return 0;
}