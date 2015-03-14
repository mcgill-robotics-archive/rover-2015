/* 
 * rosserial::std_msgs::Time Test
 * Publishes current time
 */

#include <ros.h>
#include <ros/time.h>
#include <std_msgs/Time.h>
#include <control_systems/SetPoints.h>


ros::NodeHandle  nh;

std_msgs::Time test;



void update_wheel_values(const control_systems::SetPoints& cmd_msg)
{
  //cmd_msg.thetaFL;
  //cmd_msg.speedFL;
  String front = "A11";
  String middle= "B11";
  String rear  = "C11";
  int speedFL = abs(99.*(cmd_msg.speedFL)/9.8425);
  int speedFR = abs(99.*(cmd_msg.speedFR)/9.8425);
  int speedML = abs(99.*(cmd_msg.speedML)/9.8425);
  int speedMR = abs(99.*(cmd_msg.speedMR)/9.8425);
  int speedRL = abs(99.*(cmd_msg.speedRL)/9.8425);
  int speedRR = abs(99.*(cmd_msg.speedRR)/9.8425);
  //will eventually need capability to turn wheels
  //opposite directions!!!
  if (cmd_msg.speedFL > 0){front = front + "0";}
  else {front = front + "1";}
  if (cmd_msg.speedML > 0){middle = middle + "0";}
  else {middle = middle + "1";}
  if (cmd_msg.speedRL > 0){rear = rear + "0";}
  else {rear = rear + "1";}
  
  if (speedFL > 99){front = front + "99";}
  else {
  if(speedFL < 10){front = front + "0" + String(speedFL);}
  else {front = front + String(speedFL);}
  }
  
  if (speedFR > 99){front = front + "99";}
  else {
  if(speedFR < 10){front = front + "0" + String(speedFR);}
  else {front = front + String(speedFR);}
  }
  
  if (speedML > 99){middle = middle + "99";}
  else {
  if(speedML < 10){middle = middle + "0" + String(speedML);}
  else {middle = middle + String(speedML);}
  }
    
  if (speedMR > 99){middle = middle + "99";}
  else {
  if(speedMR < 10){middle = middle + "0" + String(speedMR);}
  else {middle = middle + String(speedMR);}
  }
  
  if (speedRL > 99){rear = rear + "99";}
  else {
  if(speedRL < 10){rear = rear + "0" + String(speedRL);}
  else {rear = rear + String(speedRL);}
  }
  
  if (speedRR > 99){rear = rear + "99";}
  else {
  if(speedRR < 10){rear = rear + "0" + String(speedRR);}
  else {rear = rear + String(speedRR);}
  }
  
  front = front + "Z";
  middle = middle + "Z";
  rear = rear + "Z";
  
  Serial1.print(front);
  Serial1.print(middle);
  Serial1.print(rear);
  
}
ros::Subscriber<control_systems::SetPoints> wheels_sub("/wheels",&update_wheel_values);

void setup()
{
  nh.initNode();
  nh.subscribe(wheels_sub);
}

void loop()
{  
  nh.spinOnce();
  delay(1);
}


