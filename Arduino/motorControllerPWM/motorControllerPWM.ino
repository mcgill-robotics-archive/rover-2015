/*
 * PWM motor control for DRV8308
 * Uses serial communication to set register values accordingly
 * 
 * Author David Lavoie-Boutin & Ali Lashari
 */

#include "register.h"
#include <SPI.h>


//Control Pins
const int ENB = 4; //pin 25 - Active High
const int CLK = 5; //31 - Frequency Matching Clock Input
const int RST = 10; //30 - Active High
const int DIR = 9; //29 - 0 Is Forward / 1 Is Reverse
const int BRK = 8; //28 - Active High
const int LCK = 6; //27 - Input Only, Low When Speed Is Locked
const int FLT = 12; //26 - Input Only, Fault Indication
const int SCS = 7; //1 - Slave Select, Active High

Registers mode;
int DataIn=0;

String ModeString[50] = {
    "AG_SETPT",
    "ENPOL",
    "DIRPOL",
    "BRKPOL",
    "SYNRECT",
    "PWMF",
    "SPDMODE",
    "FGSEL",
    "BRKMOD",
    "RETRY",
    "ADVANCE",
    "SPDREVS",
    "MINSPD",
    "BASIC",
    "SPEDTH",
    "MOD120",
    "LRTIME",
    "HALLRST",
    "DELAY",
    "AUTOADV",
    "AUTOGAIN",
    "ENSINE",
    "TDRIVE",
    "DTIME",
    "IDRIVE",
    "INTCLK",
    "SPDGAIN",
    "HALLPOL",
    "BYPFILT",
    "FILK1",
    "FILK2",
    "BYPCOMP",
    "COMK1",
    "AA_SETPT",
    "COMK2",
    "OCPDEG",
    "OCPTH",
    "OVTH",
    "VREG_EN",
    "LOOPGAIN",
    "SPED",
    "RLOCK",
    "VMOV",
    "CPFAIL",
    "UVLO",
    "OTS",
    "CPOC",
    "OCP"
};

boolean ModeSelect = false;

//Register Addresses
const byte AR00 = B00000000;
const byte AR01 = B00000001;
const byte AR02 = B00000010;
const byte AR03 = B00000011;
const byte AR04 = B00000100;
const byte AR05 = B00000101;
const byte AR06 = B00000110;
const byte AR07 = B00000111;
const byte AR08 = B00001000;
const byte AR09 = B00001001;
const byte AR0A = B00001010;
const byte AR0B = B00001011;
const byte AR2A = B00101010;

//Read/Write Commands
const byte WRT = B10000000;
const byte RED = B00000000;

void setup(){
    //set PINS
    Serial.begin(9600);
    Serial.setTimeout(50);
    while (!Serial){}
}


void send(byte address, byte mode, int value){
    //spi shit
}

void loop(){
  Registers mode;
    if(ModeSelect == false){
    if(Serial.available() > 0){
      int inByte = Serial.parseInt();
      ModeSelect = true;
      mode = static_cast<Registers>(inByte);
      Serial.println(ModeString[inByte]);
    }
  }
  if(ModeSelect == true){
    parse(mode);
  }
}