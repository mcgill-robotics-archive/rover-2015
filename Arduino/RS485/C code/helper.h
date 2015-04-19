//
// Created by David Lavoie-Boutin on 15-04-15.
//

#ifndef RS485_HELPER_H
#define RS485_HELPER_H


int buildHeader(short, short);
char *toBin(int, char *, int);
short* splitLong(long, short*, int);

#endif //RS485_HELPER_H
