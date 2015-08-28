//
// Created by David Lavoie-Boutin on 15-08-28.
//

#include "Arduino.h"
#include "ArmController.h"
#include <SPI.h>
#include "../pid/PID_v1.h"

// ENABLES OR DISABLES PID
bool PID_ON = true;

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
const int SCIENCE = 10; // Servo port 0

// Wrist rotation pins
const int WRIST_ROT_0_DIR_A = 26;
const int WRIST_ROT_0_DIR_B = 28;
const int WRIST_ROT_0_PWM = 12;
const int WRIST_ROT_1_DIR_A = 22;
const int WRIST_ROT_1_DIR_B = 24;
const int WRIST_ROT_1_PWM = 11;

// Voltmeter pin
const int VOLT = A6;

// Humidity pin
const int HUMID = A7;

// Servo objects for motors.
Servo Wrist;
Servo Base;
Servo Shoulder;
Servo Elbow;
Servo Science;

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


//PUBLIC METHODS

void setPID_ON(bool val)
{
    PID_ON = val;
}


void setShoulderVel(int vel)
{
    setLinkVelocity(-vel, Shoulder);
}

void setShoulderPos(int pos)
{
    shoulderSetpoint = constrain(pos, 10, 60);
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
    elbowSetpoint = constrain(pos, 70, 140);
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
    wristSetpoint = constrain(pos, 100, 260);
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

// Sets position of the science servo.
void setSciencePos(int pos)
{
    Science.write(pos);
}

double getVoltage()
{
    /*
    Written by Olivier Lamarre on July 6th 2015

    Voltage reading (for 0-30V sources).
    Using a voltage divider circuit: 102k and 12k resistors (real resistances 102.1k and 12.2k)
    Arduino voltage measured across the 12k resistor, using VOLT and GND pins).
    Maximum voltage that the arduino will read is about 3.6V (when the source is at 30V). This is safely below the 5V limit.

    any problem with my code? DEAL WITH IT.
    */

    double maxSensorVal = 643.0;
    double sensorVal = analogRead(VOLT);
    sensorVal = constrain(sensorVal, 0.0, maxSensorVal);
    return sensorVal / maxSensorVal * 30.0;
}



float singlePHRead()
{
    Serial3.print("R\r");
    while (!Serial3.available()>0){;}
    if (Serial3.available()>0)
    {
        String Reading = Serial3.readStringUntil('\r');
        return atof(Reading.c_str());
    }
}

void setPHTemp(int temp) // Must be run every time the probe is turned on.
{
    String T = String(temp);
    Serial3.print("T,");
    Serial3.print(T);
    Serial3.print("\r");

    //uncomment all of this if you want to get a confirmation string from the device for new set temp
    //delay(50);
    //Serial3.print("T,?\r");
    //while (!Serial3.available()>0){;}
    //if (Serial3.available()>0)
    //{
    //String Reading = Serial3.readStringUntil('\r');
    //Serial.println(Reading);
    //}
}

float singleHumidRead()
{
    int res = analogRead(A7);
    // INSERT CODE HERE TO TRANSFORM READING INTO A MEANINGFUL UNIT
    return res;
}

//PRIVATE METHODS

// Update PID outputs based on new inputs
void stepPID()
{
    elbowInput = readEncoder(ELBOW);
    ElbowController.Compute();
    setLinkVelocity(-elbowOutput, Elbow);
    shoulderInput = readEncoder(SHOULDER);
    ShoulderController.Compute();
    setLinkVelocity(-shoulderOutput, Shoulder);
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

// Sets velocity for the wrist rotation joint (polulu) .
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
    switch(pin) // Enables only the selected encoder.
    {
        case WRIST:
            return ax;
            break;
        case SHOULDER:
            return 90.0 - ax;
            break;
        case ELBOW:
            return 360.0 - ax;
            break;
        default:
            return ax;
    }
}

void armSetup()
{
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
    pinMode(SCIENCE, OUTPUT);
    Wrist.attach(WRIST);
    Base.attach(BASE);
    Shoulder.attach(SHOULDER);
    Elbow.attach(ELBOW);
    Science.attach(SCIENCE);

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
    ShoulderController.SetSampleTime(1);
    ElbowController.SetSampleTime(1);
    WristController.Compute();
    ShoulderController.Compute();
    ElbowController.Compute();
}
 void armLoop()
 {
     if(PID_ON)
     {
         stepPID();
     }
 }