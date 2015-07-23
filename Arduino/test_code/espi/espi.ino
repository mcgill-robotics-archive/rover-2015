#include <SPI.h>;

int DRE_3 = 32;
int DRE_2 = 34;
int DRE_1 = 36;
int DRE_0 = 38;
float ax=0.00;

void setup()
{
  Serial.begin(9600);
  while(!Serial){;}
  pinMode(DRE_3, OUTPUT);
  pinMode(DRE_2, OUTPUT);
  pinMode(DRE_1, OUTPUT);
  pinMode(DRE_0, OUTPUT);
  digitalWrite(DRE_3,1); // 0 to enable, 1 to disable. Only enable one at a time.
  digitalWrite(DRE_2,0);
  digitalWrite(DRE_1,1);
  digitalWrite(DRE_0,1);
  SPI.begin();
  SPI.setClockDivider(SPI_CLOCK_DIV8);
  SPI.setBitOrder(MSBFIRST);
  SPI.setDataMode(SPI_MODE2);
}

void loop()
{
  byte dA = SPI.transfer(0x00);
  byte dB = SPI.transfer(0x00);
  int x= ((dA & 0x7F)<<6) | (dB>>2);
  Serial.print(x);
  Serial.print(" - ");
  ax = x*360.000/8191.000;
  Serial.println(ax, 1);
  delay(10);
}

