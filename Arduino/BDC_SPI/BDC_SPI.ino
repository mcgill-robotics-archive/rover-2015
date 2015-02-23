#include <SPI.h>
int SCS = 9;
int ModeSelect = 0;
boolean WrConfirm = false;
byte ADD1 = 0;
int DATA1 = 0;
int DataRec = 0;

void setup() {
	Serial.begin(9600);
	Serial.setTimeout(50); // Set serial.parseInt() timeout to 50ms
	while (!Serial){
	;}
	pinMode(SCS,OUTPUT); // Slave select
	SPI.begin(); // Start the SPI library
	SPI.setClockDivider(SPI_CLOCK_DIV2); //Set SPI freq at 8 Mhz
	SPI.setBitOrder(MSBFIRST); //Sent in order of read write command, address, and data MSB to LSB
	SPI.setDataMode(SPI_MODE0); //Clock idle low, Rising edge sampling 
	delay(10);
}

void loop() {
	if (Serial.available()>0){
		ModeSelect = Serial.parseInt();
		switch(ModeSelect){
			case 1:
				Serial.println(WriteRegister(ADD1, DATA1));
				break;
			case 101:
                                DataRec = ReadRegister(ADD1);
				Serial.println(String(DataRec, BIN));
				break;
		}
		delay(100);
	}
}

boolean WriteRegister(byte ADD, int DATA){
	bitWrite(ADD,7,0);
	digitalWrite(SCS, HIGH);
	SPI.transfer(ADD);
	delay(2);
	SPI.transfer(DATA);
	digitalWrite(SCS, LOW);
	WrConfirm = (DATA == ReadRegister(ADD));
	return WrConfirm;
}

int ReadRegister(byte ADD){
	bitWrite(ADD,7,1);
	digitalWrite(SCS, HIGH);
	int DATAREC = SPI.transfer(ADD);
	delayMicroseconds(5);
	digitalWrite(SCS, LOW);
	return DATAREC;
}

