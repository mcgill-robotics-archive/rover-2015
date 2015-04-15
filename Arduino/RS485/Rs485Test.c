#include <stdio.h>

int main(void) {

int address = 102;
int command = 0;
int function = 127;
int argumentLo = 200;
int argumentMid1 = 20;
int argumentMid2 = 99;
int argumentHi = 67;
int termination = 255;
	
// your code goes here
unsigned int messageLo;
messageLo = (address << 24) + (command << 23) + (function << 16) + (argumentLo << 8) + (argumentMid1);
unsigned int messageHi = 0b11101111111011111111111111111111;

printf("messageLo = %d \n" , messageLo);

unsigned int intArray[] = {messageLo,messageHi};
unsigned int getAddress = (intArray[0] & 0b11111111000000000000000000000000) >> 24;
unsigned int getCommand = (intArray[0] & 0b00000000100000000000000000000000) >> 23;
unsigned int getFunction = (intArray[0] & 0b00000000011111110000000000000000) >> 16;
unsigned int getArgumentLo = (intArray[0] & 0b00000000000000001111111100000000) >> 8;
unsigned int getArgumentMid1 = (intArray[0] & 0b00000000000000000000000011111111);
printf("getAddress = %d \n" , getAddress);
printf("getCommand = %d \n" , getCommand);
printf("getFunction = %d \n", getFunction);
printf("getArgumentLo = %d \n", getArgumentLo);
printf("getArgumentMid1 = %d \n", getArgumentMid1);
return 0;
}