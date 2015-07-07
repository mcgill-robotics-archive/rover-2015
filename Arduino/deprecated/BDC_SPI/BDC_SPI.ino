#include <SPI.h>
int SCS = 7;
int ModeSelect = 0;
boolean WrConfirm = false;
byte ADD1 = 0b00000000;
int ENB = 4;
int DATA1 = 19861;
int DataRec = 0;
byte TD1;
byte TD2;
byte DR1;
byte DR2;

void setup() {
  Serial.begin(9600);
  Serial.setTimeout(50); // Set serial.parseInt() timeout to 50ms
  while (!Serial){;
  }
  pinMode(SCS,OUTPUT); // Slave select
  int ENB = 4;
  digitalWrite(ENB, HIGH);
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
      //Serial.println(String(DataRec, BIN));
      break;
    }
    delay(100);
  }
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
  //Serial.println("Confirming Write");
  WrConfirm = (DATA == ReadRegister(ADD));
  return WrConfirm;
}

int ReadRegister(byte RADD){
  bitWrite(RADD,7,1);
  Serial.println("Reading");
  digitalWrite(SCS, HIGH);
  SPI.transfer(RADD);
  DR2 = SPI.transfer(0x00);
  DR1 = SPI.transfer(0x00);
  digitalWrite(SCS, LOW);
  //Serial.println(String(DR2, BIN));
  //Serial.println(String(DR1, BIN));
  for (int i=0; i<8; i++){
    bitWrite(DataRec, i , bitRead(DR1, i));
    bitWrite(DataRec, i+8, bitRead(DR2, i));
  }
  bitWrite(RADD,7,0);
  Serial.print("Register = ");
  Serial.print(String(RADD, BIN));
  Serial.print(" Holds Data = ");
  Serial.println(String(DataRec, BIN));
  return DataRec;
}


