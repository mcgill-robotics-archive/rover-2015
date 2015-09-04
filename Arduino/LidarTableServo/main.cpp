#include <ros.h>
#include <std_msgs/Int16.h>
#include <Servo.h>

#define SERVO_PIN 9
#define TOP_ANGLE 90
#define BOTTOM_ANGLE 45


ros::NodeHandle  nh;
Servo tiltServo;

std_msgs::Int16 str_msg;
ros::Publisher chatter("lidar_angle", &str_msg);

int currentAngle = 90;

void setup()
{
    tiltServo.attach(SERVO_PIN);
    tiltServo.write(currentAngle);

    pinMode(13, OUTPUT);
    nh.initNode();
    nh.advertise(chatter);
}

void loop()
{
    while (currentAngle > BOTTOM_ANGLE)
    {
        tiltServo.write(currentAngle --);
        delay(10);
        str_msg.data = currentAngle - 90;
        chatter.publish( &str_msg );
        nh.spinOnce();

    }

    while (currentAngle < TOP_ANGLE)
    {
        tiltServo.write(currentAngle ++);
        delay(10);
        str_msg.data = currentAngle - 90;
        chatter.publish( &str_msg );
        nh.spinOnce();
    }
}