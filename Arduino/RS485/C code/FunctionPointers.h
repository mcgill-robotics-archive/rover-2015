//
// Created by David Lavoie-Boutin on 15-04-26.
//

#ifndef RS485_FUNCTIONPOINTERS_H
#define RS485_FUNCTIONPOINTERS_H

int print1(int a);
int print2(int a);
int print3(int a);

int (*pointer1)(int a);
int (*pointer2)(int a);
int (*pointer3)(int a);

#endif //RS485_FUNCTIONPOINTERS_H
