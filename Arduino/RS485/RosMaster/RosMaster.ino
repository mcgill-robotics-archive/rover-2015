
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
#include <control_systems/Moving.h>
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

rover_msgs::GPS msg;
ros::Publisher publisher("raw_gps", &msg);
ros::Subscriber<control_systems::ArmAngles> armAngleSub ("/arm", arm_motor);
ros::Subscriber<control_systems::SetPoints> driveSub ("/wheels", drive_motor);
ros::Subscriber<control_systems::PanTiltZoom> camSub ("/camera_orientation", camera_motor);
//ros::Subscriber<control_systems::Moving> armAngleSub ("/movement", moving);
int gpsOK = 1;

void setup()
{
  
  pinMode(A0, OUTPUT);
  pinMode(A1, OUTPUT);
  digitalWrite(A0, LOW);
  digitalWrite(A1, HIGH);
  panServo.attach(panServoPin);
  tiltServo.attach(tiltServoPin);
  
  Serial1.begin(9600);          //RS-485 Com port instantiation
  while(!Serial1){;}
  
  nh.initNode();
  nh.advertise(publisher);
  nh.subscribe(driveSub);
  nh.subscribe(armAngleSub);
  Serial3.begin(4800);
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


