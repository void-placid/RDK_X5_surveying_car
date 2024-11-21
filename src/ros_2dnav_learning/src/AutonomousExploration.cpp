#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/OccupancyGrid.h"

ros::Publisher chatter_pub;

void chatterCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
    ROS_INFO("\n=====================================\nI heard map\n============================\n");
    for(auto data: msg->data){
        if(data != -1)
        ROS_INFO("%d", data);
    }

//   ROS_INFO("Middle heard: [%lf]", msg->linear.x);
//   ROS_INFO("Middle heard: [%lf]", msg->linear.y);
//   ROS_INFO("Middle heard: [%lf]", msg->linear.z);
//   ROS_INFO("Middle heard: [%lf]", msg->angular.x);
//   ROS_INFO("Middle heard: [%lf]", msg->angular.y);
//   ROS_INFO("Middle heard: [%lf]", msg->angular.z);

//   geometry_msgs::Twist msg_p;


//   msg_p.linear.x = msg->linear.x * 0.1;
//   msg_p.linear.y = msg->linear.y * 0.1;
//   msg_p.linear.z = msg->linear.z * 0.1;

//   msg_p.angular.x = msg->angular.x * 0.1;
//   msg_p.angular.y = msg->angular.y * 0.1;
//   msg_p.angular.z = msg->angular.z * 0.1;
//   chatter_pub.publish(msg_p);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "middle");
  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("map", 1000, chatterCallback);
  chatter_pub = n.advertise<geometry_msgs::Twist>("cmd_vel_cov", 1000);
  ros::Rate loop_rate(10);
  ros::spin();


  return 0;
}
