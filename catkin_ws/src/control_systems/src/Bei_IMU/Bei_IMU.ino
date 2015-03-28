//current problems 
//the accelerometer is connected to the gyroscope, which is 
//not calibrated. When I calibrate the gyro I must also adjust
//the vector for the acceleration

//pins for SDA, SCL already declared as variables
#include <Wire.h>

const int mpu = 0x68; //I2C address of the MPU-6050
double ax, ay, az, gx, gy, gz, tmp, mx, my, mz;

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
  //initialize communication
  Wire.beginTransmission(mpu);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(mpu,14,true);
  
  
  ax= Wire.read()<<8|Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)    
  ay= Wire.read()<<8|Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  az= Wire.read()<<8|Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  tmp=Wire.read()<<8|Wire.read();  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  gx= Wire.read()<<8|Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  gy= Wire.read()<<8|Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  gz= Wire.read()<<8|Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
  
  //Magnetometer readings
  mx= Wire.read()<<8|Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
  my= Wire.read()<<8|Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
  mz= Wire.read()<<8|Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
  
  //print out values
  Serial.print(gx);
  Serial.print(",");Serial.print(gy);
  Serial.print(",");Serial.print(gz);
  Serial.print(",");Serial.print(ax);
  Serial.print(",");Serial.print(ay);
  Serial.print(",");Serial.print(az);
  Serial.print(",");Serial.print(mx);
  Serial.print(",");Serial.print(my);
  Serial.print(",");Serial.println(mz);
  //no delay so that arduino will print at maximum rate
}
