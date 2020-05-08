#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "std_msgs/Header.h"
#include "std_msgs/Float32.h"


ros::Publisher pub, comp_pub;

void onReceive(const std_msgs::Float32::ConstPtr &msg){
    static ros::Time last_time;
    static float last_err;
    ros::Time current_time;
    ros::Duration difference;
    float pitch, dt;
    float P, I, D, I_limit, pval, dval;
    static float ival= 0;
    std_msgs::Float32 response;
    // std_msgs::Float32MultiArray components;
    
    // current_time= msg->header.stamp;
    // difference= current_time - last_time;
    // last_time= current_time;
    // dt= difference.toSec();
    dt= 1;

    ros::param::getCached("pid/p", P);
    ros::param::getCached("pid/i", I);
    ros::param::getCached("pid/d", D);
    ros::param::getCached("pid/i_limit", I_limit);

    pitch= msg->data;
    
    pval= P * pitch;
    dval= -D * (pitch - last_err) * dt;
    if(pitch > 0)
	if(ival < I_limit)
	    ival+= I * pitch * dt;

    if(pitch < 0)
	if(ival > -I_limit)
	    ival+= I * pitch * dt;
    
    
    response.data= pval + dval + ival;
    last_err= pitch;

    ROS_INFO("P: %f I: %f D: %f", pval, ival, dval);
/*
    components.data.clear();
    components.data.push_back(pval);
    components.data.push_back(ival);
    components.data.push_back(dval);
*/
    pub.publish(response);
    // comp_pub.publish(components);
}

int main(int argc, char *argv[]){
    ros::init(argc, argv, "pid_tilt");
    ros::NodeHandle node;

    ros::Subscriber sp_sub= node.subscribe("pid/speed/setpoint", 100, onSetpoint);
    ros::Subscriber fb_sub= node.subscribe("pid/speed/effort", 100, onFeedback);
    pub= node.advertise<std_msgs::Float32>("pid/pitch/setpoint", 1000);
    // comp_pub= node.advertise<std_msgs::Float64MultiArray>("pid/comp", 1000);
    
    ros::spin();
    return 0;
}
