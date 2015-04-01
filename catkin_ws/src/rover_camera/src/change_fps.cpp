#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <iostream>

sensor_msgs::ImageConstPtr tmpImg;
sensor_msgs::ImageConstPtr lastImg;

void imageCallback (const sensor_msgs::ImageConstPtr& msg){
	ROS_INFO_STREAM("Recieved image");
	lastImg = msg;
}


int main(int argc, char ** argv)
{
	ros::init(argc, argv, "change_fps");
	ros::NodeHandle nh;
	ros::NodeHandle nhPrivate("~");
	image_transport::ImageTransport it(nh);
	image_transport::Subscriber sub = it.subscribe("/vid0/image_raw", 1, imageCallback);
	
	image_transport::Publisher pub = it.advertise("reduced_fps/image", 1);

	int fps = 15;
	nhPrivate.param<int>("fps", fps, 15);
	ROS_INFO_STREAM(fps);
	ros::Rate loopRate(fps);
	while (ros::ok()){
		ROS_INFO_STREAM("ros ok");
		if (lastImg){
			ROS_INFO_STREAM("image exists");
			if (tmpImg != lastImg){
				std::cout << "publishing";
				pub.publish(lastImg);
				tmpImg = lastImg;
			}
		} 
		loopRate.sleep();
	}
}