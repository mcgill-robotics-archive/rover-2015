/*
    Code for the Arduino Mega on the extremities board.
    
    HOW TO CONNECT THE MOTORS:
    
  Linear actuators (up +, down -):
    (Elbow)
      -Green: M2B
      -Black: M2A
    (Shoulder)
      -White: M1B
      -Red: M1A
      
  Banebot: TO BE DETERMINED (up + down -)
    (Wrist)
      -Black: M1B
      -White: M1A
      
  Large stepper (open +, close -):
    (Claw)
      -Black: B-
      -Green: B+
      -Red: A-
      -White: A+
      
  Wrist Servos (servo ports 0 and 1) (cw +, ccw -):
    (Wrist roration) POR
      -Black to Black
      -Red to Red
      -White to Black
      -Greem to Red
*/

#include <Servo.h>

int _EN_CLAW = 23; // ACTIVE LOW
int STEP_CLAW = 25;
int DIR_CLAW = 27;
int WRIST = 2;
int SHOULDER = 4;
int ELBOW = 5;
int WRIST_ROT_1 = 10; // Servo port 0
int WRIST_ROT_2 = 9; // Servo port 1
  
  
void setup()
{
  Serial.begin(9600); // WHY DO I NEED THIS???
  
  pinMode(_EN_CLAW, OUTPUT);
  pinMode(STEP_CLAW, OUTPUT);
  pinMode(DIR_CLAW, OUTPUT);
  
  digitalWrite(_EN_CLAW, LOW); // Enables claw
}

void loop()
{
  setServoVelocity(100, WRIST);
}

void setClawDisplacement(int disp)
{
  if(disp > 0)
  {
    digitalWrite(DIR_CLAW, LOW);
  }
  else if(disp < 0)
  {
    disp *= -1;
    digitalWrite(DIR_CLAW, HIGH);
  }
  else{}
  for(int i = 0; i < disp * 200; i++)
  {
    digitalWrite(STEP_CLAW, LOW);
    delayMicroseconds(1000);
    digitalWrite(STEP_CLAW, HIGH);
    delayMicroseconds(1000);
  }
}

void setWristRotationVelocity(int vel)
{
  setServoVelocity(vel, WRIST_ROT_1);
  setServoVelocity(vel, WRIST_ROT_2);
}

void setServoVelocity(int vel, int pin)
{
  int minim;
  int maxim;
  if(pin == WRIST || pin == SHOULDER || pin == ELBOW)
  {
    minim = 30;
    maxim = 150;
  }
  else
  {
    minim = 0;
    maxim = 180;
  }
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
    servo.write(servoVal);
  }
  else
  {
    int servoVal = 90 + floor(vel * (maxim - 90) / 100);
    servo.write(servoVal);
  }
}
