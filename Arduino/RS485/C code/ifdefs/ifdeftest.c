//
// Created by David Lavoie-Boutin on 15-04-29.
//

#include <stdio.h>
#include "ifdeftest.h"

#ifdef MASTER
void print1(){
    printf("1\n");
}
#endif
#ifdef SLAVE
void print1(){
    printf("2\n");
}
#endif

int main(int argc, char ** argv){
    printf("123");
    print1();
    return 0;
}