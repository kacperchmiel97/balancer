#include <ros/ros.h>
#include "sensor_msgs/Joy.h"
#include "balancer/MotorControl.h"
#include "std_msgs/Int16MultiArray.h"

ros::Publisher pub;

void joyCallback(const sensor_msgs::Joy &msg){
    std_msgs::Int16MultiArray command;

    command.data.clear();
    if(msg.axes[1] > 0){
	// command.left_dir= true;
	command.data.push_back(1000 * msg.axes[1]);
    }
    else{
	// command.left_dir= false;
	command.data.push_back(1000 * msg.axes[1]);
    }

    if(msg.axes[4] > 0){
	// command.right_dir= true;
	command.data.push_back(-1000 * msg.axes[4]);
    }
    else{
	// command.right_dir= false;
	command.data.push_back(-1000 * msg.axes[4]);
    }

    pub.publish(command);
}

int main( int argc, char** argv )
{
  ros::init(argc, argv, "teleop_raw");
  ros::NodeHandle n;
  pub= n.advertise<std_msgs::Int16MultiArray>("motorSpeed", 100);
  ros::Subscriber sub= n.subscribe("joy", 1000, joyCallback);
  
  ros::spin();
  return 0;
}
