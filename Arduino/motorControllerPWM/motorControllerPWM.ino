#include <Wire.h>

#include <Adafruit_INA219.h>

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
const int CLK = 10; //31 - Frequency Matching Clock Input
const int RST = 5; //30 - Active High
const int DIR = 9; //29 - 0 Is Forward / 1 Is Reverse
const int BRK = 8; //28 - Active High
const int LCK = 6; //27 - Input Only, Low When Speed Is Locked
const int FLT = 12; //26 - Input Only, Fault Indication
const int SCS = 7; //1 - Slave Select, Active High

int ModeSelect = 0;

int DATA1 = 19861;
int DataRec = 0;
byte TD1;
byte TD2;
byte DR1;
byte DR2;
boolean WrConfirm = false;

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
byte AR2A = B00101010;

//Read/Write Commands
const byte WRT = B10000000;
const byte RED = B00000000;

void setup(){
    TCCR1A = _BV (WGM10) | _BV (WGM11) | _BV (COM1B1);  // Phase Correct
    TCCR1B = _BV (WGM13) | _BV (CS10);                  // Phase Correct / Prescale 1
    OCR1A = 200;                                        // Sets Top to correspond to frequency 
    OCR1B = 0;                                          // Sets duty cycle (duty cycle = 0CR1B/OCR1A)         

    //set PINS
    Serial.begin(9600);
    Serial.setTimeout(50);
    while (!Serial){}//Suspend program progression till serial communication is established
    pinMode(SCS,OUTPUT); // Slave select
    pinMode(RST,OUTPUT);
    pinMode(ENB,OUTPUT);

    pinMode(CLK, OUTPUT); //PWM OUTPUT

    digitalWrite(ENB, HIGH);
    digitalWrite(SCS, LOW);
    digitalWrite(RST, LOW);
    SPI.begin(); // Start the SPI library
    SPI.setClockDivider(SPI_CLOCK_DIV8); //Set SPI freq at 8 Mhz
    SPI.setBitOrder(MSBFIRST); //Sent in order of read write command, address, and data MSB to LSB
    SPI.setDataMode(SPI_MODE0); //Clock idle low, Rising edge sampling 
    delay(10);
}

boolean WriteRegister(byte ADD, int DATA){
  bitWrite(ADD,7,0);
  for (int i=0; i<8; i++){
    bitWrite(TD1, i , bitRead(DATA, i));
    bitWrite(TD2, i, bitRead(DATA, i+8));
  }
  // Serial.println("Writing");
  // Serial.print("Writing Register = ");
  // Serial.print(String(ADD, BIN));
  // Serial.print(" With Data = ");
  // Serial.println(String(DATA, BIN));
  digitalWrite(SCS, HIGH);
  SPI.transfer(ADD);
  SPI.transfer(TD2);
  SPI.transfer(TD1);
  digitalWrite(SCS, LOW);
  // Serial.println("Confirming Write");
  return (DATA == ReadRegister(ADD));
  
}

int ReadRegister(byte RADD){
  bitWrite(RADD,7,1);
  // Serial.println("Reading");
  digitalWrite(SCS, HIGH);
  SPI.transfer(RADD);
  DR2 = SPI.transfer(0x00);
  DR1 = SPI.transfer(0x00);
  digitalWrite(SCS, LOW);
  // Serial.println(String(DR2, BIN));
  // Serial.println(String(DR1, BIN));
  for (int i=0; i<8; i++){
    bitWrite(DataRec, i , bitRead(DR1, i));
    bitWrite(DataRec, i+8, bitRead(DR2, i));
  }
  bitWrite(RADD,7,0);
  // Serial.print("Register = ");
  // Serial.print(String(RADD, BIN));
  // Serial.print(" Holds Data = ");
  // Serial.println(String(DataRec, BIN));
  return DataRec;
}

int ReadFault(){
  bitWrite(AR2A,7,1);
  // Serial.println("Reading");
  digitalWrite(SCS, HIGH);
  SPI.transfer(AR2A);
  DR2 = SPI.transfer(0x00);
  DR1 = SPI.transfer(0x00);
  digitalWrite(SCS, LOW);
  // Serial.println(String(DR2, BIN));
  // Serial.println(String(DR1, BIN));
  for (int i=0; i<8; i++){
    bitWrite(DataRec, i , bitRead(DR1, i));
    bitWrite(DataRec, i+8, bitRead(DR2, i));
  }
  bitWrite(AR2A,7,0);
  Serial.print("Register = ");
  Serial.print(String(AR2A, BIN));
  Serial.print(" Holds Data = ");
  Serial.println(String(DataRec, BIN));
  return DataRec;
}

void loop(){
  int inByte;
  Registers mode;
    if(ModeSelect == false){
    if(Serial.available() > 0){
      inByte = Serial.parseInt();
      ModeSelect = true;
      mode = static_cast<Registers>(inByte);
      
    }
  }
  if(ModeSelect == true){
    switch (mode) {
      Serial.println("Switch");
        case SPECIAL:
          // special hard coded commands
            Serial.println("Special command");
            while(!Serial.available()){
                delay(10);
            }
            if(Serial.available() > 0){
                int dataIn = Serial.parseInt();
                switch(dataIn){
                    case 10:
                      Serial.println("RESETTING");
                      digitalWrite(ENB, LOW);
                      digitalWrite(RST, HIGH);
                      delay(100);
                      digitalWrite(RST, LOW);
                      digitalWrite(ENB, HIGH);
                      Serial.println("Finished");
                      break;

                    case 11:
                        Serial.println("Disabling");
                        digitalWrite(ENB, LOW);
                        Serial.println("Done");
                      break;

                    case 12:
                        Serial.println("Enabling");
                        digitalWrite(ENB, HIGH);
                        Serial.println("Done");
                      break;
                      
                    case 13:
                      ReadFault();                    }
            }
            ModeSelect=false;
            break; //end register update
            
        case SET_SPEED:
        int abc;
            Serial.println("Speed");
            while(!Serial.available()){
                delay(10);
            }
            abc=Serial.parseInt();
            OCR1B = 2 * abc;
            ModeSelect=false;
            break;
            
        default:
            Serial.println(ModeString[inByte]);
            parse(mode);
    }
    
  }
}
