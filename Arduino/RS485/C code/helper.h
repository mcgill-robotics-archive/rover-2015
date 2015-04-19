//
// Created by David Lavoie-Boutin on 15-04-15.
//

#ifndef RS485_HELPER_H
#define RS485_HELPER_H


int buildHeader(short, short);
char *toBin(int, char *, int);

/**
 * split a number in an array of shorts
 * long, the number to split
 * short*, pointer to the array where to store the shorts
 * int, the length of the array
 */
short* splitLong(long, short*, int);

#endif //RS485_HELPER_H
