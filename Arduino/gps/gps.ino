#include <TinyGPS++.h>
#include <ros.h>
#include <rover_msgs/GPS.h>

ros::NodeHandle  nh;

static const uint32_t GPSBaud = 4800;
TinyGPSPlus gps;

rover_msgs::GPS msg;
ros::Publisher publisher("raw_gps", &msg);

void setup()
{
  nh.initNode();
  nh.advertise(publisher);
  Serial3.begin(4800);
}

void displayInfo()
{
  if (gps.location.isValid())
  {
    msg.locationValid = true;
    msg.latitude = gps.location.lat();
    msg.longitude = gps.location.lng();
    msg.altitude = gps.altitude.meters();
  }
  else msg.locationValid = false;
  
  
  if (gps.date.isValid())
  {
    msg.dateValid = true;
    msg.month = gps.date.month();
    msg.day = gps.date.day();
    msg.year = gps.date.year();
  }
  else msg.dateValid = false;
  if (gps.time.isValid())
  {
    msg.timeValid = true;
    msg.hour = gps.time.hour();
    msg.minute = gps.time.minute();
    msg.sec = gps.time.second();
    msg.csec = gps.time.centisecond();
  }
  else
  {
    msg.timeValid = false;
  }

  publisher.publish(&msg);
}

void loop()
{
  while (Serial3.available() > 0)
    if (gps.encode(Serial3.read()))
      displayInfo();
      
  if (millis() > 5000 && gps.charsProcessed() < 10)
  {
    nh.logwarn("No GPS detected: check wiring.");
    while(true);
  }
  nh.spinOnce();
  delay(10);
}
