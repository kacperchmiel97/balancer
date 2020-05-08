#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

visualization_msgs::Marker marker;
ros::Publisher marker_pub;

void filterCallback(const geometry_msgs::QuaternionStamped::ConstPtr &msg){
    marker.header.stamp = ros::Time::now();
    marker.pose.orientation.x = msg->quaternion.x;
    marker.pose.orientation.y = msg->quaternion.y;
    marker.pose.orientation.z = msg->quaternion.z;
    marker.pose.orientation.w = msg->quaternion.w;

    marker_pub.publish(marker);
}

int main( int argc, char** argv )
{
  ros::init(argc, argv, "visualization");
  ros::NodeHandle n;
  // ros::Rate r(100);
  marker_pub= n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
  ros::Subscriber sub= n.subscribe("imu/filtered/rotation", 1000, filterCallback);


  marker.header.frame_id = "/imu_v";
  marker.ns = "basic_shapes";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::CUBE;
  
  marker.action = visualization_msgs::Marker::ADD;
    
  // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
  marker.pose.position.x = 0;
  marker.pose.position.y = 0;
  marker.pose.position.z = 0;

  // Set the scale of the marker -- 1x1x1 here means 1m on a side
  marker.scale.x = 1.0;
  marker.scale.y = 1.0;
  marker.scale.z = 1.0;

  // Set the color -- be sure to set alpha to something non-zero!
  marker.color.r = 0.0f;
  marker.color.g = 1.0f;
  marker.color.b = 0.0f;
  marker.color.a = 1.0;

  marker.lifetime = ros::Duration();
  
  ros::spin();
  return 0;
}
