#include "ros/ros.h"
#include "sensor_msgs/Image.h"

ros::Publisher im_pub;

void republish (sensor_msgs::Image data){
  im_pub.publish(data);

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "posePublisher");
  ros::NodeHandle n;
  std::string topic = "/camera_front_right/camera/image_raw";
  im_pub = n.advertise<sensor_msgs::Image>("camera_feed2", 1000);
  ros::Subscriber im_sub = n.subscribe(topic, 1000, republish);
  ros::spin();

  return 0;
}

