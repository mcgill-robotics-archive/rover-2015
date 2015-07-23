// Code for the Arduino Mega on the extremities board.

#include <Servo.h>

void setup()
{
}

void loop()
{
  setElbowVelocity(-50);
}

/*
    HOW TO CONNECT THE MOTORS:
    
  Linear actuators:
    (Elbow)
      -Green: M2B
      -Black: M2A
    (Shoulder)
      -White: M1B
      -Red: M1A
  Banebot: TO BE DETERMINED
    (Wrist)
      -
      -
*/

void setClawDisplacement(int disp)
{
}

void setWristRotationVelocity(int vel)
{
}

void setWristVelocity(int vel) // Sets the velocity of the shoulder link. -100 is the max speed down and 100 is the max speed up.
{
  setServoVelocity(vel, 3);
}

// --------- COMPLETED vvv -------------//

void setShoulderVelocity(int vel) // Sets the velocity of the shoulder link. -100 is the max speed down and 100 is the max speed up.
{
  setServoVelocity(vel, 4);
}

void setElbowVelocity(int vel) // Sets the velocity of the elbow link. -100 is the max speed down and 100 is the max speed up.
{
  setServoVelocity(vel, 5);
}

void setServoVelocity(int vel, int pin) // Sets the velocity of a servo. 0 stops the motor. -100 is the max speed in one direction and 100 is the max speed in the other.
{
  int minim = 30; // Minimum and maximum values for servo.write(). Experimentally determined.
  int maxim = 150;
  Servo servo;
  servo.attach(pin);
  if(vel == 0 || vel > 100 || vel < -100)
  {
    servo.write(90);
  }
  else if(vel < 0)
  {
    vel *= -1;
    int servoVal = ceil(vel * (90 - minim) / 100);
    Serial.println(servoVal);
    servo.write(servoVal);
  }
  else
  {
    int servoVal = 90 + floor(vel * (maxim - 90) / 100);
    servo.write(servoVal);
  }
}
