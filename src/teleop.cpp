#include <ros/ros.h>
#include "sensor_msgs/Joy.h"
#include "balancer/MotorControl.h"
#include "std_msgs/Float32.h"

ros::Publisher pub_speed, pub_dir;

void joyCallback(const sensor_msgs::Joy &msg){
    std_msgs::Float32 speed, dir;

    speed.data= (20 * msg.axes[1]);
    pub_speed.publish(speed);

    dir.data= (-250 * msg.axes[3]);
    pub_dir.publish(dir);
}

int main( int argc, char** argv )
{
  ros::init(argc, argv, "teleop_test");
  ros::NodeHandle n;
  pub_speed= n.advertise<std_msgs::Float32>("speed_setpoint", 100);
  pub_dir= n.advertise<std_msgs::Float32>("direction", 100);
  ros::Subscriber sub= n.subscribe("joy", 1000, joyCallback);
  
  ros::spin();
  return 0;
}
