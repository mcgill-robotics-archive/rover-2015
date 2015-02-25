#include <SPI.h>
int SCS = 7;
int RST = 10;
int ModeSelect = 0;
int ENB = 4;
int DATA1 = 19861;
int DataRec = 0;
byte TD1;
byte TD2;
byte DR1;
byte DR2;
boolean WrConfirm = false;
byte ADD1 = 0b00000000;

void setup() {
  Serial.begin(9600);
  Serial.setTimeout(50); // Set serial.parseInt() timeout to 50ms
  while (!Serial){
    ;
  } //Suspend program progression till serial communication is established
  pinMode(SCS,OUTPUT); // Slave select
  pinMode(RST,OUTPUT);
  pinMode(ENB,OUTPUT);
  digitalWrite(ENB, HIGH);
  digitalWrite(SCS, LOW);
  digitalWrite(RST, LOW);
  SPI.begin(); // Start the SPI library
  SPI.setClockDivider(SPI_CLOCK_DIV8); //Set SPI freq at 8 Mhz
  SPI.setBitOrder(MSBFIRST); //Sent in order of read write command, address, and data MSB to LSB
  SPI.setDataMode(SPI_MODE0); //Clock idle low, Rising edge sampling 
  delay(10);
}

void loop() {
  if (Serial.available()>0){
    ModeSelect = Serial.parseInt();
    switch(ModeSelect){

    case 1:
      if (WriteRegister(ADD1, DATA1)==1){
        Serial.println("Write Confirmed");
      }
      else{
        Serial.println("Write Failed");
      }
      break;

    case 101:
      DataRec = ReadRegister(ADD1);
      break;

    case 10:
      Serial.println("RESETTING");
      digitalWrite(ENB, LOW);
      digitalWrite(RST, HIGH);
      delay(100);
      digitalWrite(RST, LOW);
      digitalWrite(ENB, HIGH);
      break;



    case 11:
      digitalWrite(ENB, LOW);
      break;



    case 12:
      digitalWrite(ENB, HIGH);
      break;
    }
  }

  delay(100);

}

boolean WriteRegister(byte ADD, int DATA){
  bitWrite(ADD,7,0);
  for (int i=0; i<8; i++){
    bitWrite(TD1, i , bitRead(DATA, i));
    bitWrite(TD2, i, bitRead(DATA, i+8));
  }
  Serial.println("Writing");
  Serial.print("Writing Register = ");
  Serial.print(String(ADD, BIN));
  Serial.print(" With Data = ");
  Serial.println(String(DATA, BIN));
  digitalWrite(SCS, HIGH);
  SPI.transfer(ADD);
  SPI.transfer(TD2);
  SPI.transfer(TD1);
  digitalWrite(SCS, LOW);
  WrConfirm = (DATA == ReadRegister(ADD));
  return WrConfirm;
}

int ReadRegister(byte ADD){
  bitWrite(ADD,7,1);
  Serial.println("Reading");
  digitalWrite(SCS, HIGH);
  SPI.transfer(ADD);
  DR2 = SPI.transfer(0x00);
  DR1 = SPI.transfer(0x00);
  digitalWrite(SCS, LOW);
  for (int i=0; i<8; i++){
    bitWrite(DataRec, i , bitRead(DR1, i));
    bitWrite(DataRec, i+8, bitRead(DR2, i));
  }
  bitWrite(ADD,7,0);
  Serial.print("Register = ");
  Serial.print(String(ADD, BIN));
  Serial.print(" Holds Data = ");
  Serial.println(String(DataRec, BIN));
  return DataRec;
}





