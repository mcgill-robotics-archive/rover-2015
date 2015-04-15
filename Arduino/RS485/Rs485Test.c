#include <stdio.h>

typedef unsigned int uint;

int main () {

    uint address = 102;
    uint command = 0;
    uint function = 127;
    uint argumentLo = 200;
    uint argumentMid1 = 20;
    uint argumentMid2 = 99;
    uint argumentHi = 67;
    uint termination = 255;
	

    uint messageLo;
    messageLo = (address << 24) + (command << 23) + (function << 16) + (argumentLo << 8) + (argumentMid1);
    uint messageHi = 0b11101111111011111111111111111111;

    printf("messageLo = %d \n" , messageLo);

    uint intArray[] = {messageLo,messageHi};
    uint getAddress = (intArray[0] & 0b11111111000000000000000000000000) >> 24;
    uint getCommand = (intArray[0] & 0b00000000100000000000000000000000) >> 23;
    uint getFunction = (intArray[0] & 0b00000000011111110000000000000000) >> 16;
    uint getArgumentLo = (intArray[0] & 0b00000000000000001111111100000000) >> 8;
    uint getArgumentMid1 = (intArray[0] & 0b00000000000000000000000011111111);

    printf("getAddress = %d \n" , getAddress);
    printf("getCommand = %d \n" , getCommand);
    printf("getFunction = %d \n", getFunction);
    printf("getArgumentLo = %d \n", getArgumentLo);
    printf("getArgumentMid1 = %d \n", getArgumentMid1);

    return 0;
}