//This is a general PID controller for the arm
#include <ArduinoHardware.h>
#include <SPI.h>
#include <PID_v1.h>
#define toDeg 57.2957795

int PWM_A = 10;
int RESET_AB = 8;
int DTE = 7;
int DRE = 9;

//Declare ros node
ros::NodeHandle nh;

// Setup for PID
//upper/elbow motor
double inputU;
double setpointU;
double outputU; 
PID controllerU(&inputU, &outputU, &setpointU, 1, 0, 0, DIRECT);
//lower/shoulder/base motor
double inputL;
double setpointL;
double outputL;
PID controllerL(&inputL, &outputL, &setpointL, 1, 0, 0, DIRECT);


//Function gets the angle
void get_angle(const control_systems::ArmAngles& msg)
{
  setpointL = toDeg * msg.shoulderElevation;
  setpointU = toDeg * msg.elbow;
  //Do an angle check!!!
}

ros::Subscriber<control_systems::ArmAngles> sub("/arm", &get_angle);

void setup()
{
  //For communicating with master
  Serial.begin(9600);
  Serial.setTimeout(50);
  while (!Serial){}

  // Setup for motor output (AB)
  pinMode(RESET_AB, OUTPUT);
  pinMode(PWM_A, OUTPUT);
  digitalWrite(RESET_AB, HIGH);

  // Setup for encoder input (AB)
  pinMode(DRE, OUTPUT);
  pinMode(DTE, OUTPUT);
  digitalWrite(DRE, LOW);
  digitalWrite(DTE, HIGH);
  
  SPI.begin(); 
  SPI.setClockDivider(SPI_CLOCK_DIV8);
  SPI.setBitOrder(MSBFIRST); 
  SPI.setDataMode(SPI_MODE2);
  
  // Setup for PID
  input = readEncoderAB();
  controller.SetMode(AUTOMATIC);
  controller.SetOutputLimits(-100, 100);
}

void loop()
{    
nh.spinOnce();
//  Serial.println(readEncoderAB(), 3);
//  delay(10);

//Test - try to see if angle is correct
input = readEncoderAB();
controller.Compute();
//Serial.println(output, 3);
setSpeedAB(output);
delay(10);

}

void setSpeedAB(int spd) // Sets speed from -100 (full speed reverse) to 100 (full speed forward). 0 stops the motor.
{
  //  16000000 / (1 * 200 * 2 ) = 40 KHz
  int duty_cycle = 2 * floor((99 * spd) / 201 + 50);  // Scaling from [-100, 100] to [1, 99] (i.e. converts speed to duty cycle)
  TCCR1A = _BV (WGM10) | _BV (WGM11) | _BV (COM1B1);  // Phase Correct
  TCCR1B = _BV (WGM13) | _BV (CS10);                  // Phase Correct / Prescale 1
  OCR1A = 200;                                        // Sets Top to correspond to frequency 
  OCR1B = duty_cycle;                                 // Sets duty cycle (duty cycle = 0CR1B/OCR1A)
}

void setSpeedCD(int spd){}

float readEncoderAB() // Returns encoder position in degrees.
{
  byte dA = SPI.transfer(0x00);
  byte dB = SPI.transfer(0x00);
  int x= ((dA & 0x7F)<<6) | (dB>>2);
  float ax = x*359.956/8191.000;
  return ax;
}

