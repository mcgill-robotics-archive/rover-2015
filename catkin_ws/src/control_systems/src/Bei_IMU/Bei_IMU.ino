//pins for SDA, SCL already declared as variables
#include <Wire.h>

const int mpu = 0x68; //I2C address of the MPU-6050
long double ax, ay, az, gx, gy, gz, tmp;
//offsets
double axo = 0;
double ayo = 0;
double azo = 0;
double tmpo = 0;
double gxo = 0;
double gyo = 0;
double gzo = 0;

long double sumforavg = 0;
int measures = 0;
void setup()
{
  Wire.begin();
  Wire.beginTransmission(mpu);
  Wire.write(0x6B);  //PWR
  Wire.write(0); //wakes up MPU-6050
  Wire.endTransmission(true);
  Serial.begin(115200);
}

void loop()
{
  Wire.beginTransmission(mpu);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(mpu,14,true);
  
  
  ax=Wire.read()<<8|Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)    
  ay=Wire.read()<<8|Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  az=Wire.read()<<8|Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  tmp=Wire.read()<<8|Wire.read();  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  gx=Wire.read()<<8|Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  gy=Wire.read()<<8|Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  gz=Wire.read()<<8|Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)

  //factor in offsets
  ax  += axo;
  ay  += ayo;
  az  += azo;
  tmp += tmpo;
  gx  += gxo;
  gy  += gyo;
  gz  += gzo;
  
  
  //magnitude of acceleration
  long double accel = 9.81 * sqrt(ax*ax + ay*ay + az*az)/16269.73;
  //sumforavg += accel;
  //measures ++;
  //double average = sumforavg/measures;
  //Serial.print("accel = "); Serial.println(accel);
  //Serial.print("avgaccel = "); Serial.print(average);
  //Serial.print(", curr =  ");Serial.println((double)accel);
  Serial.println((double)accel);
  
  delay(50);
}
