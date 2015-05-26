//This is a general PID controller for the arm
#include <ArduinoHardware.h>
#include <SPI.h>
#include <PID_v1.h>

int PWM_A = 10;
int RESET_AB = 8;
int DTE_A = 7;
int DRE_A = 9;
//For CD PID
int PWM_C = 14;
int RESET_CD = 12;
int DTE_C = 11;
int DRE_C = 13;

// Setup for PID
//motor controller
double input3;
double setpoint3;
double output3;

//Bounds on motor
double top3 = 70;
double bottom3 = 10;

// Setup for PID
//motor controller
double input4;
double setpoint4;
double output4;

//Bounds on motor
double top4 = 90;
double bottom4 = -90;

//Create PID Controllers
PID controller3(&input3, &output3, &setpoint3, 1, 0, 0, DIRECT);
PID controller4(&input4, &output4, &setpoint4, 1, 0, 0, DIRECT);

void setup()
{
  //For communicating with master
  Serial.begin(9600);
  Serial.setTimeout(50);
  while (!Serial){}
  
  //Set to zero
  setpoint3 = 0;
  setpoint4 = 0;

  // Setup for motor output (AB)
  pinMode(RESET_AB, OUTPUT);
  pinMode(PWM_A, OUTPUT);
  digitalWrite(RESET_AB, HIGH);
  
  // Setup for motor output (CD)
  pinMode(RESET_CD, OUTPUT);
  pinMode(PWM_C, OUTPUT);
  digitalWrite(RESET_CD, HIGH);
  

  // Setup for encoder input
  // (AB)
  pinMode(DRE_A, OUTPUT);
  pinMode(DTE_A, OUTPUT);
  digitalWrite(DRE_A, LOW);
  digitalWrite(DTE_A, HIGH);
  
  // (CD)
  pinMode(DRE_C, OUTPUT);
  pinMode(DTE_C, OUTPUT);
  digitalWrite(DRE_C, LOW);
  digitalWrite(DTE_C, HIGH);
  
  SPI.begin(); 
  SPI.setClockDivider(SPI_CLOCK_DIV8);
  SPI.setBitOrder(MSBFIRST); 
  SPI.setDataMode(SPI_MODE2);
  
  // Setup for PID
  input3 = readEncoderAB();
  input4 = readEncoderCD();
  controller3.SetMode(AUTOMATIC);
  controller4.SetOutputLimits(-100, 100);
}

void loop()
{
  //Listen until message is passed
  if (Serial.available())
  {
    //Check a message
    if(Serial.read()==167)
    {
      //Once message is started, read in all bytes
      char message[6];
      ///////////
      //May be a source of error in the future
      //once the 
      ///////////
      message[0] = Serial.read();
      message[1] = Serial.read();
      message[2] = Serial.read();
      message[3] = Serial.read();
      message[4] = Serial.read();
      message[5] = Serial.read();
      
      //Check if address is for motor
      if (message[0] == 13)
      {
        //Switch for the functions 
        switch(message[1])
        {
         case  5: //set angle function
           int result = 0;
           for (int n = 0; n < 4 ;n++)
           {
             result = (result << 8) + message[2+n];
           }
           //bound check
           double angle = 10*(double)result;
           //final bound check
           if (angle <= top3 && angle >= bottom3)
           {
             setpoint3 = angle;
           }
           break;
        }
        //Perform some calculation to calculate angle from the message
        //Check bounds
      }
      else if (message[0] == 14)
      {
        //Switch for the functions 
        switch(message[1])
        {
         case  5: //set angle function
           int result = 0;
           for (int n = 0; n < 4 ;n++)
           {
             result = (result << 8) + message[2+n];
           }
           //bound check
           double angle = 10*(double)result;
           //final bound check
           if (angle <= top4 && angle >= bottom4)
           {
             setpoint4 = angle;
           }
           break;
        }
      }
      
    }
  }
  //Arm Specific
  //Test - try to see if angle is correct
  input3 = readEncoderAB();
  input4 = readEncoderAB();
  //PID compututations
  controller3.Compute();
  controller4.Compute();
  
  setSpeedAB(output3);
  setSpeedCD(output4);
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

void setSpeedCD(int spd) // Sets speed from -100 (full speed reverse) to 100 (full speed forward). 0 stops the motor.
{
  //  16000000 / (1 * 200 * 2 ) = 40 KHz
  int duty_cycle = 2 * floor((99 * spd) / 201 + 50);  // Scaling from [-100, 100] to [1, 99] (i.e. converts speed to duty cycle)
  TCCR1A = _BV (WGM10) | _BV (WGM11) | _BV (COM1B1);  // Phase Correct
  TCCR1B = _BV (WGM13) | _BV (CS10);                  // Phase Correct / Prescale 1
  OCR1A = 200;                                        // Sets Top to correspond to frequency 
  OCR1B = duty_cycle;                                 // Sets duty cycle (duty cycle = 0CR1B/OCR1A)
}

float readEncoderAB() // Returns encoder position in degrees.
{
  byte dA = SPI.transfer(0x00);
  byte dB = SPI.transfer(0x00);
  int x= ((dA & 0x7F)<<6) | (dB>>2);
  float ax = x*359.956/8191.000;
  return ax;
}

float readEncoderCD() // Returns encoder position in degrees.
{
  byte dC = SPI.transfer(0x00);
  byte dD = SPI.transfer(0x00);
  int x= ((dC & 0x7F)<<6) | (dD>>2);
  float ax = x*359.956/8191.000;
  return ax;
}
