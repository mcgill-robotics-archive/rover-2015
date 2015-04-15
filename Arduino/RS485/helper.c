//
// Created by David Lavoie-Boutin on 15-04-15.
//

#include "helper.h"


int buildHeader(short address, short function){
    return (int) (address << 8 | function) ;
}

char *toBin(int a, char *buffer, int buf_size) {
    buffer += (buf_size - 1);
    for (int i = buf_size; i > 0; i--) {
        /*
         * #mad_c_skills the next uncommented line basically replaces this entire comment block.
         * //extract lsb of number
         * int tmp = a & 1;
         * //convert to char
         * char tmpChar = (char) (tmp + '0');
         * // update buffer at location
         * *buffer = tmpChar;
         * // decrement buffer pointer
         * buffer --;
         */
        *buffer-- = (char) ((a & 1) + '0');
        a >>= 1;
    }
    return buffer;
}
