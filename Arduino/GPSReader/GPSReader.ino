#include <ArduinoHardware.h>
#include <ros.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include <rover_msgs/GPS.h>

static const int RXPin = 4, TXPin = 3;
static const uint32_t GPSBaud = 4800;

TinyGPSPlus gps;
SoftwareSerial ss(RXPin, TXPin);

ros::NodeHandle nh;
rover_msgs::GPS msg;
ros::Publisher publisher("raw_gps", &msg);
void setup()
{
  ss.begin(GPSBaud);
  
  nh.initNode();
  nh.advertise(publisher);
}

void loop()
{
  // This sketch displays information every time a new sentence is correctly encoded.
  while (ss.available() > 0)
    if (gps.encode(ss.read()))
      displayInfo();

  if (millis() > 5000 && gps.charsProcessed() < 10)
  {
    nh.logwarn("No GPS detected: check wiring.");
    while(true);
  }
}

void displayInfo()
{
  if (gps.location.isValid() && gps.date.isValid() && gps.time.isValid())
  {
    msg.isValid = true;
    
    msg.latitude = gps.location.lat();
    msg.longitude = gps.location.lng();
    msg.altitude = gps.altitude.meters();
    
    msg.month = gps.date.month();
    msg.day = gps.date.day();
    msg.year = gps.date.year();

    msg.hour = gps.time.hour();
 
    msg.minute = gps.time.minute();
 
    msg.sec = gps.time.second();
 
    msg.csec = gps.time.centisecond();
  }
  else
  {
    msg.isValid = false;
  }

  publisher.publish(&msg);
}
