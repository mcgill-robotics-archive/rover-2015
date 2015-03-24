/* 
 * rosserial::std_msgs::Time Test
 * Publishes current time
 */

#include <ros.h>

#include <control_systems/SetPoints.h>


ros::NodeHandle  nh;


double pi = 3.14159265359;

void update_wheel_values(const control_systems::SetPoints& cmd_msg)
{
  //cmd_msg.thetaFL;
  //cmd_msg.speedFL;
  String front = "A11";
  String middle= "B11";
  String rear  = "C11";
  int speedFL = abs(89.*(cmd_msg.speedFL)/9.8425)+10;
  int speedFR = abs(89.*(cmd_msg.speedFR)/9.8425)+10;
  int speedML = abs(89.*(cmd_msg.speedML)/9.8425)+10;
  int speedMR = abs(89.*(cmd_msg.speedMR)/9.8425)+10;
  int speedRL = abs(89.*(cmd_msg.speedRL)/9.8425)+10;
  int speedRR = abs(89.*(cmd_msg.speedRR)/9.8425)+10;

  //will eventually need capability to turn wheels
  //opposite directions!!!
  if (cmd_msg.speedFL > 0){
    front = front + "0";
  }
  else {
    front = front + "1";
  }
  if (cmd_msg.speedML > 0){
    middle = middle + "0";
  }
  else {
    middle = middle + "1";
  }
  if (cmd_msg.speedRL > 0){
    rear = rear + "0";
  }
  else {
    rear = rear + "1";
  }

  if (speedFL > 99){
    front = front + "99";
  }
  else {
    front = front + String(speedFL);
  }

  if (speedFR > 99){
    front = front + "99";
  }
  else {
    front = front + String(speedFR);
  }

  if (speedML > 99){
    middle = middle + "99";
  }
  else {
    middle = middle + String(speedML);
  }

  if (speedMR > 99){
    middle = middle + "99";
  }
  else {
    middle = middle + String(speedMR);
  }

  if (speedRL > 99){
    rear = rear + "99";
  }
  else {
    rear = rear + String(speedRL);
  }

  if (speedRR > 99){
    rear = rear + "99";
  }
  else {
    rear = rear + String(speedRR);
  }

  front = front + "Z";
  middle = middle + "Z";
  rear = rear + "Z";

  //angles
  String steering = "S110";
  int thetaFL = abs(90.*(cmd_msg.thetaFL)/pi + 45);
  int thetaRL = abs(90.*(cmd_msg.thetaRL)/pi + 45);
  int thetaRR = abs(90.*(cmd_msg.thetaRR)/pi + 45);
  int thetaFR = abs(90.*(cmd_msg.thetaFR)/pi + 45);

  if (thetaFL > 90){
    steering = steering + "90";
  }
  else if (thetaFL < 10){
    steering = steering + "0" + String(thetaFL);
  }
  else {
    steering = steering + String(thetaFL);
  }

  if (thetaRL > 90){
    steering = steering + "90";
  }
  else if (thetaRL < 10){
    steering = steering + "0" + String(thetaRL);
  }
  else {
    steering = steering + String(thetaRL);
  }

  if (thetaRR > 90){
    steering = steering + "90";
  }
  else if (thetaRR < 10){
    steering = steering + "0" + String(thetaRR);
  }
  else {
    steering = steering + String(thetaRR);
  }

  if (thetaRL > 90){
    steering = steering + "90";
  }
  else if (thetaRL < 10){
    steering = steering + "0" + String(thetaRL);
  }
  else {
    steering = steering + String(thetaRL);
  }

  steering = steering + "Z";

  //
  //Serial1.print(front);
  //Serial1.print(middle);
  //Serial1.print(rear);
  //Serial1.print(steering);
  nh.loginfo(front.c_str());
  nh.loginfo(middle.c_str());
  nh.loginfo(rear.c_str());
  nh.loginfo(steering.c_str());
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



