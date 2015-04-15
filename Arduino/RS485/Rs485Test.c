#include <stdio.h>

int main(void) {
// your code goes here
unsigned int messageLo = 0b11101111111011111111111111111111;
unsigned int messageHi = 0b11101111111011111111111111111111;

printf("messageLo = %d \n" , messageLo);

unsigned int intArray[] = {messageLo,messageHi};
unsigned int address = (intArray[0] & 0b11111111000000000000000000000000) >> 24;
unsigned int function = (intArray[0] & 0b00000000111111110000000000000000) >> 16;
printf("address = %d \n" , address);
printf("function = %d \n", function);
return 0;
}