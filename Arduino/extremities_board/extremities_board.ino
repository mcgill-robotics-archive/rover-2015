/*
    Code for the Arduino Mega on the extremities board.
    
    HOW TO CONNECT THE MOTORS:
    
  Linear actuators (up +, down -):
    (Elbow- White)
      -Black: M2B
      -White: M2A
    (Shoulder- Blue)
      -White: M2A
      -Black: M2B
      
  Banebot:(up + down -)
    (Wrist- Blue)
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
int SHOULDER = 3;
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

  if(Serial.available() > 0)
  {
    String command = Serial.readString();
    if(command.charAt(0) == 's')
    {
      if(command.charAt(1) == 'u')
      {
        setServoVelocity(50, SHOULDER);
      }
      else if(command.charAt(1) == 'd')
      {
        setServoVelocity(-50, SHOULDER);
      }
      else
      {
        setServoVelocity(0, SHOULDER);
      }
    }
    else if(command.charAt(0) == 'e')
    {
      if(command.charAt(1) == 'u')
      {
        setServoVelocity(100, ELBOW);
      }
      else if(command.charAt(1) == 'd')
      {
        setServoVelocity(-100, ELBOW);
      }
      else
      {
        setServoVelocity(0, ELBOW);
      }
    }
    else if(command.charAt(0) == 'w')
    {
      if(command.charAt(1) == 'u')
      {
        setServoVelocity(25, WRIST);
      }
      else if(command.charAt(1) == 'd')
      {
        setServoVelocity(-25, WRIST);
      }
      else
      {
        setServoVelocity(0, WRIST);
      }
    }
    else if(command.charAt(0) == 'r') // Why isn't this working???
    {
      if(command.charAt(1) == 'w') // clock_W_ise
      {
        setWristRotationVelocity(100);
      }
      else if(command.charAt(1) == 'c') // _C_ounterclockwise
      {
        setWristRotationVelocity(-100);
      }
      else
      {
        setWristRotationVelocity(0);
      }
    }
    else if(command.charAt(0) == 'c')
    {
      if(command.charAt(1) == 'o')
      {
        setClawDisplacement(0.5);
      }
      else if(command.charAt(0) == 'c')
      {
        setClawDisplacement(-0.5);
      }
      else
      {
      }
    }
  }
}

void setClawDisplacement(float disp)
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
  for(int i = 0; i < int(disp * 200); i++)
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
    Serial.println(); // WHY
    servo.write(servoVal);
  }
  else
  {
    int servoVal = 90 + floor(vel * (maxim - 90) / 100);
    Serial.println(); // WHY
    servo.write(servoVal);
  }
}

