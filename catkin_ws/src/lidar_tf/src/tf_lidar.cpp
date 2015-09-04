#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/Int16.h>

void angleCallback(const std_msgs::Int16& msg){
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    transform.setOrigin( tf::Vector3(0, 0, 0) );
    tf::Quaternion q;
    q.setRPY(0, (-msg.data )/ 180.0 * 3.14159265 , 0);
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "laser"));
}

int main(int argc, char** argv){
    ros::init(argc, argv, "my_tf_broadcaster");
   
    ros::NodeHandle node;
    ros::Subscriber sub = node.subscribe("/lidar_angle", 10, &angleCallback);

    ros::spin();
    return 0;
};

