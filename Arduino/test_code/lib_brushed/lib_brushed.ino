#include <SPI.h>
#include <PID_v1.h>

int PWM_A = 10;
int RESET_AB = 8;
int DTE = 7;
int DRE = 9;

// Setup for PID
double input;
double setpoint;
double output;  
PID controller(&input, &output, &setpoint, 1, 0, 0, DIRECT);

void setup()
{
  Serial.begin(9600);
  Serial.setTimeout(50);
  while (!Serial){}
  
  pinMode(A2, OUTPUT);
  digitalWrite(A2, LOW);
  pinMode(A1, OUTPUT);
  digitalWrite(A2, HIGH);
  pinMode(A0, OUTPUT);
  digitalWrite(A2, HIGH);
//  a3 13 high

  pinMode(A3, OUTPUT);
  digitalWrite(A3, HIGH);
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);
  
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
  if (Serial.available()>0)
  {
      setSpeedAB(Serial.parseInt());
//      setpoint = Serial.parseInt();
  }
//  Serial.println(readEncoderAB(), 3);
//  delay(10);
//
input = readEncoderAB();
controller.Compute();
Serial.println(output, 3);
setSpeedAB(output);
delay(10);

}

void setSpeedAB(int spd) // Sets speed from -100 (full speed reverse) to 100 (full speed forward). 0 stops the motor.
{
  //  16000000 / (1 * 200 * 2 ) = 40 KHz
  int duty_cycle = 2 * floor((99 * spd) / 201 + 50);  // Scaling from [-100, 100] to [1, 99] (i.e. converts speed to duty cycle)
  TCCR3A = _BV (WGM30) | _BV (WGM31) | _BV (COM3A1);  // Phase Correct
  TCCR3B = _BV (WGM33) | _BV (CS30);                  // Phase Correct / Prescale 1
  OCR3A = 200;                                        // Sets Top to correspond to frequency 
  OCR3B = duty_cycle;                                 // Sets duty cycle (duty cycle = 0CR1B/OCR1A)
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
