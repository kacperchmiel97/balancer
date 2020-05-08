#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "std_msgs/Header.h"
#include "std_msgs/Float32.h"
#include "std_srvs/Empty.h"
#include "geometry_msgs/QuaternionStamped.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <angles/angles.h>

// How many samples to average over when calibrating
#define CALIBRATION_SAMPLES 20

ros::Publisher rotation_pub, pitch_pub;
tf2::Vector3 calibrationAxis;
float calibrationAngle;

// Calibration service
bool calibrate(std_srvs::Empty::Request &request, std_srvs::Empty::Response &response){
    float aX, aY, aZ, angle;
    tf2::Vector3 measured, reference, axis;
    sensor_msgs::Imu msg;
    boost::shared_ptr<sensor_msgs::Imu const> ptr;
    
    ROS_INFO("Filter calibration...\n");

    aX= aY= aZ= 0;

    // Take a few samples of acc measurements
    for(uint i= 0; i < CALIBRATION_SAMPLES; i++){
	ptr= ros::topic::waitForMessage<sensor_msgs::Imu>("imu/raw");
	if(ptr != NULL){
	    msg= *ptr;
	    aX+= msg.linear_acceleration.x;
	    aY+= msg.linear_acceleration.y;
	    aZ+= msg.linear_acceleration.z;
	}
    }

    // Average the samples
    measured.setX(aX/CALIBRATION_SAMPLES);
    measured.setY(aY/CALIBRATION_SAMPLES);
    measured.setZ(aZ/CALIBRATION_SAMPLES);
    measured.normalize();
    
    reference= tf2::Vector3(0, 0, 1);

    // Calculate the rotation from current gravity vector to base orientation
    calibrationAxis= measured.cross(reference);
    calibrationAngle= measured.angle(reference);
    
    ROS_INFO("X %f Y %f Z %f", measured.getX(), measured.getY(), measured.getZ());
    ROS_INFO("Calibration complete!");
    
    return true;
}

// Callback on receiving raw data
void onReceive(const sensor_msgs::Imu::ConstPtr &msg){
    static ros::Time last_time;
    ros::Time current_time;
    ros::Duration difference;
    static float roll= 0;
    static float pitch= 0;
    static float yaw= 0;
    float dt;
    float aX, aY, aZ;
    tf2::Vector3 raw;
    float alpha= 0.1;
    tf2::Quaternion result;
    geometry_msgs::QuaternionStamped result_msg;
    std_msgs::Float32 pitch_msg;

    // Calculate how much time passed since last measurement
    current_time= msg->header.stamp;
    difference= current_time - last_time;
    last_time= current_time;
    dt= difference.toSec();

    // Get acc measurement data
    raw.setX(msg->linear_acceleration.x);
    raw.setY(msg->linear_acceleration.y);
    raw.setZ(msg->linear_acceleration.z);
    raw.normalize();

    // Rotate it to our base
    raw= raw.rotate(calibrationAxis, calibrationAngle);

    // Calculate roll and pitch based on acc data
    aX= atan2(raw.getY() , raw.getZ());
    aY= atan2((-raw.getX()) , sqrt(raw.getY() * raw.getY() + raw.getZ() * raw.getZ()));
    aZ= 0;

    // Complementary filter
    roll= (1-alpha)*(roll + msg->angular_velocity.x * dt) + (alpha)*(aX);
    pitch= (1-alpha)*(pitch + msg->angular_velocity.y * dt) + (alpha)*(aY);
    // Z = (1-alpha)*(yaw + msg->angular_velocity.z * dt) + (alpha)*(aZ);

    // Publish pitch for the PIDs
    pitch_msg.data= angles::to_degrees(pitch);
    pitch_pub.publish(pitch_msg);

    // Calculate and publish the quaternion rotation for visualisation node
    result.setRPY(roll, pitch, yaw);
    result.normalize();

    result_msg.quaternion= tf2::toMsg(result);
    result_msg.header.stamp= ros::Time::now();
    rotation_pub.publish(result_msg);
}

int main(int argc, char *argv[]){
    ros::init(argc, argv, "imu_filter");
    ros::NodeHandle node;

    ros::Subscriber sub= node.subscribe("imu/raw", 1000, onReceive);
    ros::ServiceServer calibrationService= node.advertiseService("calibrateFilter", &calibrate);
    rotation_pub= node.advertise<geometry_msgs::QuaternionStamped>("imu/filtered/rotation", 1000);
    pitch_pub= node.advertise<std_msgs::Float32>("imu/filtered/pitch", 1000);
    
    ros::spin();
    return 0;
}
