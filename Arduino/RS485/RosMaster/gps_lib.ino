
void displayInfo()
{
  if (gps.location.isValid())
  {
    msg.locationValid = true;
    msg.latitude = gps.location.lat();
    msg.longitude = gps.location.lng();
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

