#include <Stepper.h>


/*
 gps reader 
 arduino code, must run rosserial_python serial_node.py node to publish in ROS
 uses serial 3 to communicate with gps
 refresh rate 1Hz
 on arduino mega uses pins 14 and 15
 */


#include <TinyGPS++.h>
#include <ros.h>
#include <rover_msgs/GPS.h>
#include "motor.h"
#include <Servo.h>

#include <control_systems/ArmAngles.h>
#include <control_systems/SetPoints.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Bool.h>
#include <control_systems/PanTiltZoom.h>

byte initiation = 167;
byte address = 0;
byte function = 0;
byte argumentLo = 0;
byte argumentMid1 = 20;
byte argumentMid2 = 0;
byte argumentHi = 67;
byte termination = 255;
int messageComponents = 7;
int state = 2;
int last;
byte message[]= {address, function, argumentLo, argumentMid1, argumentMid2, argumentHi, termination};     //Constructing the message

#include <SPI.h>

const int EN = 24; //pin 25 - Active High
const int DR = 26; //29 - 0 Is Forward / 1 Is Reverse
const int BK = 22; //28 - Active High
const int SCS1 = 28; //1 - Slave Select 1

int tSCS=1;
int DataRec;

int ENB =0;
int DRV=0;
byte ADD0 = 0x00;
byte ADD2 = 0x02;
byte ADD3 = 0x03;
byte ADD4 = 0x04;
byte ADDA = 0x0A;
byte ADDB = 0x0B;

int REG01 = 289;
int REG21 = 1279;
int REG31 = 22528;
int REG41 = 32768;
int REGA1 = 61440;

int REG02 = 1313;
int REG22 = 1279;
int REG32 = 22528;
int REG42 = 32768;
int REGA2 = 61440;

int BRK;
byte TD1;
byte TD2;
String Data;

/*
 * The main idea here is to encode/decode the messages we're going to be sending over 
 * the RS485 communications protocol.
 * We'll be sending 7 bytes, 1 for address, 1 for function, 
 * 4 for argument and 1 for termination which will always have a value of 255
 */
 
ros::NodeHandle  nh;

static const uint32_t GPSBaud = 4800;
TinyGPSPlus gps;
int panServoPin = 50;
int tiltServoPin = 51;
Servo panServo;
Servo tiltServo;

const int stepsPerRevolution = 200;  // change this to fit the number of steps per revolution
// for your motor


// initialize the stepper library on pins 8 through 11: (CHANGE)
Stepper myStepper(stepsPerRevolution, 8, 9, 10, 11);
int stepCount = 0;  // number of steps the motor has taken
int motorSpeed = 20;

rover_msgs::GPS msg;
ros::Publisher publisher("raw_gps", &msg);
ros::Subscriber<control_systems::ArmAngles> armAngleSub ("/arm", arm_motor);
ros::Subscriber<std_msgs::Int16> driveSub ("/wheels", drive_motor);
ros::Subscriber<control_systems::PanTiltZoom> camSub ("/camera_orientation", camera_motor);
ros::Subscriber<std_msgs::Int16> clawSub ("/claw", claw);
int gpsOK = 1;

void setup()
{
  
  pinMode(A0, OUTPUT);
  pinMode(A1, OUTPUT);
  digitalWrite(A0, LOW);
  digitalWrite(A1, HIGH);
  panServo.attach(panServoPin);
  tiltServo.attach(tiltServoPin);

  pinMode(aBase, OUTPUT);
  pinMode(bBase, OUTPUT);
  pinMode(speedBase, OUTPUT);
  pinMode(aShoulder, OUTPUT);
  pinMode(bShoulder, OUTPUT);
  pinMode(speedShoulder, OUTPUT);
  pinMode(aElbow, OUTPUT);
  pinMode(bElbow, OUTPUT);
  pinMode(speedElbow, OUTPUT);
  pinMode(aWrist, OUTPUT);
  pinMode(bWrist, OUTPUT);
  pinMode(speedWrist, OUTPUT);
  
  Serial1.begin(9600);          //RS-485 Com port instantiation
  while(!Serial1){;}
  
  nh.initNode();
  nh.advertise(publisher);
  nh.subscribe(driveSub);
  nh.subscribe(armAngleSub);
  nh.subscribe(camSub);
  Serial3.begin(4800);

  SPI.begin();
  SPI.setClockDivider(SPI_CLOCK_DIV128); 
  SPI.setBitOrder(MSBFIRST); 
  SPI.setDataMode(SPI_MODE0);
  pinMode(EN, OUTPUT);
  pinMode(DR, OUTPUT);
  pinMode(BK, OUTPUT);
  pinMode(SCS1, OUTPUT);
  digitalWrite(EN, LOW);
  digitalWrite(DR, LOW);
  digitalWrite(BK, LOW);
  digitalWrite(SCS1, LOW);
    
  WriteRegister(ADD0, REG01,1);
  WriteRegister(ADD2, REG21,1);
  WriteRegister(ADD3, REG31,1);
  WriteRegister(ADD4, REG41,1);
  WriteRegister(ADDA, REGA1,1);
}

void loop()
{
  Serial1.flush();
  Serial1.write(initiation);
  
  while (Serial3.available() > 0)
    if (gps.encode(Serial3.read()))
      displayInfo();
      
  if (millis() > 5000 && gps.charsProcessed() < 10)
  {
    nh.logwarn("No GPS detected: check wiring.");
    while(true);
  }
  for (int i=0; i<messageComponents; i++){
    //while (Serial1.available()<2){
      Serial1.flush();
      Serial1.write(byte(i));
      Serial1.write(message[i]);
      delay(40);
    //}
  }
    
  nh.spinOnce();
  delay(10);
}

void WriteRegister(byte ADD, int DATA, int SCS){
  byte tADD=ADD;
  
  tSCS=SCS1;
  bitWrite(tADD,7,0);
  for (int i=0; i<8; i++){
    bitWrite(TD1, i , bitRead(DATA, i));
    bitWrite(TD2, i, bitRead(DATA, i+8));
  }
  digitalWrite(tSCS, HIGH);
  SPI.transfer(tADD);
  SPI.transfer(TD2);
  SPI.transfer(TD1);
  digitalWrite(tSCS, LOW);\
}
  
