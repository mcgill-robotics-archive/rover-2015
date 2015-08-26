/*
  TODO:
Allow variable stepper speed.

  DIRECTIONS:
Arm links: up+, down -
Claw: open +, close -
Wrist rotation: cw +, ccw -
*/

#include <Servo.h>
#include <SPI.h>
#include <PID_v1.h>

// ENABLES OR DISABLES PID
bool PID_ON = false;

// ENABLES OR DISABLES SERIAL DEBUG INPUT
bool SERIAL_ON = true;

// Enable pins. Underscore prefix indicates that pin is active-low.
const int _EN_CLAW = 23;
const int _DRE_UNUSED = 32;   // Encoder port 3
const int _DRE_WRIST = 34;    // Encoder port 2
const int _DRE_SHOULDER = 36; // Encoder port 1
const int _DRE_ELBOW = 38;    // Encoder port 0

// Stepper motor pins
const int STEP_CLAW = 25;
const int DIR_CLAW = 27;

// Servo motor pins
const int WRIST = 5;
const int BASE = 4;
const int SHOULDER = 3;
const int ELBOW = 2;

// Wrist rotation pins
const int WRIST_ROT_0_DIR_A = 26;
const int WRIST_ROT_0_DIR_B = 28;
const int WRIST_ROT_0_PWM = 12;
const int WRIST_ROT_1_DIR_A = 22;
const int WRIST_ROT_1_DIR_B = 24;
const int WRIST_ROT_1_PWM = 11;

// Servo objects for motors.
Servo Wrist;
Servo Base;
Servo Shoulder;
Servo Elbow;

// Variables needed for PID control.
double wristInput;
double wristSetpoint;
double wristOutput;
double shoulderInput;
double shoulderSetpoint;
double shoulderOutput;
double elbowInput;
double elbowSetpoint;
double elbowOutput;

// PID object interacts with variables via pointers.
// Just need to call controller.Compute() and the value of output will be updated.  
PID WristController(&wristInput, &wristOutput, &wristSetpoint, 2, 0, 0, DIRECT);
PID ShoulderController(&shoulderInput, &shoulderOutput, &shoulderSetpoint, 5, 1, 0, DIRECT);
PID ElbowController(&elbowInput, &elbowOutput, &elbowSetpoint, 5, 1, 0, DIRECT);
  
void setup()
{
  // Serial communication setup
  Serial.begin(9600);
  Serial.setTimeout(50);
  while(!Serial){}
  
  // Claw setup
  pinMode(_EN_CLAW, OUTPUT);
  pinMode(STEP_CLAW, OUTPUT);
  pinMode(DIR_CLAW, OUTPUT);
  digitalWrite(_EN_CLAW, LOW); // Enables claw
  
  // Servo setup
  pinMode(WRIST, OUTPUT);
  pinMode(BASE, OUTPUT);
  pinMode(SHOULDER, OUTPUT);
  pinMode(ELBOW, OUTPUT);
  Wrist.attach(WRIST);
  Base.attach(BASE);
  Shoulder.attach(SHOULDER);
  Elbow.attach(ELBOW);
  
  // Wrist rotation setup
  pinMode(WRIST_ROT_0_DIR_A, OUTPUT);
  pinMode(WRIST_ROT_0_DIR_B, OUTPUT);
  pinMode(WRIST_ROT_0_PWM, OUTPUT);
  pinMode(WRIST_ROT_1_DIR_A, OUTPUT);
  pinMode(WRIST_ROT_1_DIR_B, OUTPUT);
  pinMode(WRIST_ROT_1_PWM, OUTPUT);
  
  // Encoder setup
  pinMode(_DRE_WRIST, OUTPUT);
  pinMode(_DRE_SHOULDER, OUTPUT);
  pinMode(_DRE_ELBOW, OUTPUT);
  digitalWrite(_DRE_WRIST, HIGH);
  digitalWrite(_DRE_SHOULDER, HIGH);
  digitalWrite(_DRE_ELBOW, HIGH);
  SPI.begin(); 
  SPI.setClockDivider(SPI_CLOCK_DIV8);
  SPI.setBitOrder(MSBFIRST); 
  SPI.setDataMode(SPI_MODE2);
  
  delay(10);
  
  // PID setup
  wristInput = readEncoder(WRIST);
  shoulderInput = readEncoder(SHOULDER);
  elbowInput = readEncoder(ELBOW);
  wristSetpoint = wristInput; // Make sure initial target is initial position -> zero correction.
  shoulderSetpoint = shoulderInput;
  elbowSetpoint = elbowInput;
  WristController.SetMode(AUTOMATIC);
  ShoulderController.SetMode(AUTOMATIC);
  ElbowController.SetMode(AUTOMATIC);
  WristController.SetOutputLimits(-100, 100);  
  ShoulderController.SetOutputLimits(-100, 100);  
  ElbowController.SetOutputLimits(-100, 100);
  WristController.SetSampleTime(1);
  ShoulderController.SetSampleTime(1);;
  ElbowController.SetSampleTime(1); 
  WristController.Compute();
  ShoulderController.Compute();
  ElbowController.Compute();  
}

void loop()
{
  if(SERIAL_ON)
  {
    if(Serial.available() > 0)
    {
      String command = Serial.readString();
      if(command.charAt(0) == 's')
      {
        if(command.charAt(1) == 'u')
        {
          setShoulderVel(100);
        }
        else if(command.charAt(1) == 'd')
        {
          setShoulderVel(-100);
        }
        else if(command.charAt(1) == 's')
        {
          setShoulderVel(0);
        }
        else
        {
          setShoulderPos(command.substring(1, 4).toInt());
        }
      }
      else if(command.charAt(0) == 'e')
      {
        if(command.charAt(1) == 'u')
        {
          setElbowVel(100);
        }
        else if(command.charAt(1) == 'd')
        {
          setElbowVel(-100);
        }
        else if(command.charAt(1) == 's')
        {
          setElbowVel(0);
        }
        else
        {
          setElbowPos(command.substring(1, 4).toInt());
        }
      }
      else if(command.charAt(0) == 'w')
      {
        if(command.charAt(1) == 'u')
        {
          setWristVel(100);
        }
        else if(command.charAt(1) == 'd')
        {
          setWristVel(-100);
        }
        else if(command.charAt(1) == 's')
        {
          setWristVel(0);
        }
        else
        {
          setWristPos(command.substring(1, 4).toInt());
        }
      }
      else if(command.charAt(0) == 'b')
      {
        if(command.charAt(1) == 'w') // clock_W_ise
        {
          setBaseVel(100);
        }
        else if(command.charAt(1) == 'c') // _C_ounterclockwise
        {
          setBaseVel(-100);
        }
        else
        {
          setBaseVel(0);
        }
      }
      else if(command.charAt(0) == 'r')
      {
        if(command.charAt(1) == 'w') // clock_W_ise
        {
          setRollVel(50);
        }
        else if(command.charAt(1) == 'c') // _C_ounterclockwise
        {
          setRollVel(-50);
        }
        else
        {
          setRollVel(0);
        }
      }
      else if(command.charAt(0) == 'c')
      {
        if(command.charAt(1) == 'o')
        {
          setClawDisplacement(0.5); //TODO UPDATE TO MORE CONVENIENT VALUE
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
    Serial.print("elbow ");
    Serial.print(elbowSetpoint, 3);
    Serial.print("\t");
    Serial.print(elbowInput, 3);
    Serial.print("\t");
    Serial.print(elbowOutput, 3);
    Serial.print("\t");
    Serial.print("\t");
    Serial.print("\t");
    Serial.print("shoulder ");
    Serial.print(shoulderSetpoint, 3);
    Serial.print("\t");
    Serial.print(shoulderInput, 3);
    Serial.print("\t");
    Serial.print(-shoulderOutput, 3);
    Serial.print("\t");
    Serial.print("\t");
    Serial.print("\t");
    Serial.print("wrist ");
    Serial.print(wristSetpoint, 3);
    Serial.print("\t");
    Serial.print(wristInput, 3);
    Serial.print("\t");
    Serial.println(-wristOutput, 3);
  }
  if(PID_ON)
  {
    stepPID();
  }
}

//PUBLIC METHODS

void setPID_ON(bool val)
{
  PID_ON = val;
}

void setSERIAL_ON(bool val)
{
  SERIAL_ON = val;
}

void setShoulderVel(int vel)
{
  setLinkVelocity(vel, Shoulder);
}

void setShoulderPos(int pos)
{
  shoulderSetpoint = constrain(pos, 150, 180);
}

double getShoulderPos()
{
  return readEncoder(SHOULDER);
}

void setElbowVel(int vel)
{
  setLinkVelocity(-vel, Elbow);
}

void setElbowPos(int pos)
{
  elbowSetpoint = constrain(pos, 10, 40);
}

double getElbowPos()
{
  return readEncoder(ELBOW);
}

void setWristVel(int vel)
{
  setLinkVelocity(vel, Wrist);
}

void setWristPos(int pos)
{
  wristSetpoint = constrain(pos, 75, 235);
}

double getWristPos()
{
  return readEncoder(WRIST);
}

void setBaseVel(int vel)
{
  setLinkVelocity(vel, Base); //TODO POSITIVE OR NEGATIVE?
}

void setRollVel(int vel)
{
  setWristRotVelocity(vel); //TODO VERIFY DIRECTION
}

void setClawDisp(int disp)
{
  setClawDisplacement(disp); //TODO MORE CONVENIENT UNITS
}

//PRIVATE METHODS

// Update PID outputs based on new inputs
void stepPID()
{
  elbowInput = readEncoder(ELBOW);
  ElbowController.Compute();
  setLinkVelocity(elbowOutput, Elbow);
  shoulderInput = readEncoder(SHOULDER);
  ShoulderController.Compute();
  setLinkVelocity(shoulderOutput, Shoulder);
  wristInput = readEncoder(WRIST);
  WristController.Compute();
  setLinkVelocity(wristOutput, Wrist);
  delay(1);
}

// Sets claw's displacement in rotations of the stepper.
void setClawDisplacement(float disp)
{
  if(disp > 0) // Choose a direction
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

// Sets velocity for the wrist rotation joint.
void setWristRotVelocity(int vel)
{
  if(vel == 0 || vel > 100 || vel < -100)
  {
    analogWrite(WRIST_ROT_0_PWM, 0);
    analogWrite(WRIST_ROT_1_PWM, 0);
  }
  else if(vel < 0)
  {
    vel *= -1;
    digitalWrite(WRIST_ROT_0_DIR_A, HIGH);
    digitalWrite(WRIST_ROT_0_DIR_B, LOW);
    digitalWrite(WRIST_ROT_1_DIR_A, HIGH);
    digitalWrite(WRIST_ROT_1_DIR_B, LOW);
    int output = map(vel, 0, 100, 0, 255);
    analogWrite(WRIST_ROT_0_PWM, output);
    analogWrite(WRIST_ROT_1_PWM, output);
  }
  else
  {
    digitalWrite(WRIST_ROT_0_DIR_A, LOW);
    digitalWrite(WRIST_ROT_0_DIR_B, HIGH);
    digitalWrite(WRIST_ROT_1_DIR_A, LOW);
    digitalWrite(WRIST_ROT_1_DIR_B, HIGH);
    int output = map(vel, 0, 100, 0, 255);
    analogWrite(WRIST_ROT_0_PWM, output);
    analogWrite(WRIST_ROT_1_PWM, output);
  }
}

// Sets velocity for a joint controlled by a sabertooth
void setLinkVelocity(int vel, Servo servo)
{
  setServoVelocity(vel, servo, 30, 150);
}

// Sets a specified servo to a specified velocity between -100 and 100.
// Minim and maxim are the highest and lowest raw velocities that actually work.
// They've been determined experimentally.
void setServoVelocity(int vel, Servo servo, int minim, int maxim)
{
  if(vel == 0 || vel > 100 || vel < -100)
  {
    servo.write(90);
  }
  /*else if(vel < 0) // This is the old code that was used to map speeds to PWM outputs.
  {
    vel *= -1;
    int servoVal = ceil(vel * (90 - minim) / 100);
    servo.write(servoVal);
  }
  else
  {
    int servoVal = 90 + floor(vel * (maxim - 90) / 100);
    servo.write(servoVal);
  }*/
  else
  {
    int servoVal = map(vel, -100, 100, minim, maxim);
    servo.write(servoVal);
  }
}

// Returns selected encoder's position in degrees.
float readEncoder(int pin)
{
  switch(pin) // Enables only the selected encoder.
  {
    case WRIST:
      digitalWrite(_DRE_WRIST, LOW);
      digitalWrite(_DRE_SHOULDER, HIGH);
      digitalWrite(_DRE_ELBOW, HIGH);
      break;
    case SHOULDER:
      digitalWrite(_DRE_WRIST, HIGH);
      digitalWrite(_DRE_SHOULDER, LOW);
      digitalWrite(_DRE_ELBOW, HIGH);
      break;
    case ELBOW:
      digitalWrite(_DRE_WRIST, HIGH);
      digitalWrite(_DRE_SHOULDER, HIGH);
      digitalWrite(_DRE_ELBOW, LOW);
      break;
    default: 
      digitalWrite(_DRE_WRIST, HIGH);
      digitalWrite(_DRE_SHOULDER, HIGH);
      digitalWrite(_DRE_ELBOW, HIGH);
  }
  byte dA = SPI.transfer(0x00);
  byte dB = SPI.transfer(0x00);
  int x= ((dA & 0x7F)<<6) | (dB>>2);
  float ax = x*359.956/8191.000;
  return ax;
}

