//This is a general PID controller for the arm
#include <ArduinoHardware.h>
#include <SPI.h>
#include <PID_v1.h>

int PWM_A = 10;
int RESET_AB = 8;
int DTE = 7;
int DRE = 9;

// Setup for PID
//motor controller
double input;
double setpoint;
double output;

//Bounds on motor
double top = 180;
double bottom = 0;

PID controller(&input, &output, &setpoint, 1, 0, 0, DIRECT);

void setup()
{
  //For communicating with master
  Serial.begin(9600);
  Serial.setTimeout(50);
  while (!Serial){}
  
  //Set to zero
  setpoint = 0;

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
  //Listen until message is passed
  if (Serial.available())
  {
  if(Serial.read()==167)
  {
    //Once message is started, read bytes
    char message[6];
    Serial.readBytes(message,6);
  /////
  //NEED FINAL BOUND CHECK!!!!
  /////
  //Check if address is for motor
  if (message[0] == 12)
  {
    //Switch for the functions 
    switch(message[1])
    {
     case  5: //set angle
       int result = 0;
       for (int n = 0; n < 4;n++)
       {
         result = (result << 8) + message[2+n];
       }
       //bound check
       
       setpoint = (double)result;
       break;
       
    }
    //Perform some calculation to calculate angle from the message
    
    //Check bounds
    
  }
  }
  }
  //Arm Specific
  //Test - try to see if angle is correct
  input = readEncoderAB();
  //PID compututations
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

float readEncoderAB() // Returns encoder position in degrees.
{
  byte dA = SPI.transfer(0x00);
  byte dB = SPI.transfer(0x00);
  int x= ((dA & 0x7F)<<6) | (dB>>2);
  float ax = x*359.956/8191.000;
  return ax;
}

