//
// Created by David Lavoie-Boutin on 15-08-28.
//
#include "Arduino.h"
#include "SPI.h"
#include "ros.h"
#include "LidarTable.h"

ros::NodeHandle nh;
tf::TransformBroadcaster br;

const char * tf_name_child = "laser";
const char * base_tf = "world";

void setupSPI() {
    // Encoder setup
    pinMode(_DRE_UNUSED, OUTPUT);
    digitalWrite(_DRE_UNUSED, HIGH);

    SPI.begin();
    SPI.setClockDivider(SPI_CLOCK_DIV8);
    SPI.setBitOrder(MSBFIRST);
    SPI.setDataMode(SPI_MODE2);
}

float readEncoder() {
    byte dA = SPI.transfer(0x00);
    byte dB = SPI.transfer(0x00);
    int x= ((dA & 0x7F)<<6) | (dB>>2);
    float ax = x*359.956/8191.000;
    return ax;
}

void sendTransform() {
    geometry_msgs::Transform trsf;
    trsf.translation.x = 0;
    trsf.translation.y = 0;
    trsf.translation.z = 0;
    trsf.rotation = fromPRY(0,readEncoderRAD(), 0); //TODO: confirm this is the proper axis

    geometry_msgs::TransformStamped transformStamped;
    transformStamped.transform = trsf;
    transformStamped.child_frame_id = tf_name_child;
    transformStamped.header.frame_id = base_tf;
    transformStamped.header.stamp = nh.now();


    br.sendTransform(transformStamped);
}

geometry_msgs::Quaternion fromPRY(float pitch, float roll, float yaw) {

    float c1 = cos(pitch / 2);
    float c2 = cos(roll / 2);
    float c3 = cos(yaw / 2);
    float s1 = sin(pitch / 2);
    float s2 = sin(roll / 2);
    float s3 = sin(yaw / 2);

    float w = c1 * c2 * c3 - s1 * s2 * s3;
    float x = s1 * s2 * c3 + c1 * c2 * s3;
    float y = s1 * c2 * c3 + c1 * s2 * s3;
    float z = c1 * s2 * c3 - s1 * c2 * s3;

    geometry_msgs::Quaternion quaternion;
    quaternion.w = w;
    quaternion.x = x;
    quaternion.y = y;
    quaternion.z = z;

    return quaternion;
}

float readEncoderRAD() {
    return readEncoder() * PI / 180.0;
}

void initROS() {
    nh.initNode();
    br.init(nh);
}

