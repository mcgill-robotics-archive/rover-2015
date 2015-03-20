//pins for SDA, SCL already declared as variables
#include <Wire.h>

const int MPU = 0x68; //I2C address of the MPU-6050
int16_t ax, ay, az, gx, gy, gz, tmp;
void setup()
{
  Wire.begin();
  Wire.beginTransmission(MPU);
  Wire.write(0x6B);  //PWR
  Wire.write(0); //wakes up MPU-6050
  Serial.begin(9600);
}

void loop()
{
  Wire.requestFrom(SCL,6);
  
  while(Wire.available())
  {
   char c = Wire.read();
   Serial.print(c); 
  }
  delay(500);
}
