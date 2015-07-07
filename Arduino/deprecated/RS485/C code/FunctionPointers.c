//
// Created by David Lavoie-Boutin on 15-04-26.
//

#include <stdio.h>
#include "FunctionPointers.h"

int print1(int a){
    printf("This is print1\n");
    return 1;
}

int print2(int a){
    printf("This is print 2 with %d\n",a);
    return 1;
}

int print3(int a){
    printf("This is print 3");
    return 1;
}

int addInt(int n, int m) {
    return n+m;
}

/*
 * returns pointer to proper numbered function
 */
int (*parseFunctionNumber(int a))(int b){
    printf("Got argument %d\n",a);
    int (*function)(int);
    switch (a){
        case 1:
            function = &print1;
            return function;
        case 2:
            return pointer2;
        case 3:
            return pointer3;
        default:
            ;
     }
    return 0;
}

int (*functionFactory2(int n))(int, int) {
    printf("Got parameter %d\n", n);
    int (*functionPtr)(int,int) = &addInt;
    return functionPtr;
}

int main(int argc, char ** argv){
    pointer1 = &print1;
    pointer2 = &print2;
    pointer3 = &print3;
    pointer2 = parseFunctionNumber(2)(2);

    pointer2(123);
    parseFunctionNumber(2)(1234);
    printf("%d",functionFactory2(32)(14,42));
    return 0;
}