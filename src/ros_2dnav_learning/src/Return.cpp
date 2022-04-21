#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <queue>
#include <random>
#include <move_base_msgs/MoveBaseActionGoal.h>


class ReturnToStart
{
private:
  ros::NodeHandle nh_;
  ros::Subscriber odom_sub_;
  ros::Subscriber map_sub_;
  ros::Publisher goal_pub_;

  nav_msgs::OccupancyGrid current_map;

  
  bool is_moved;

  double moved_dist_thred = 0.1;   // m 

  // std::vector<double> robot_position;
  std::queue<double> robot_position[2];


  std::random_device rd_;
  std::mt19937 gen_{rd_()}; // 随机数生成器

public:
  ReturnToStart()
    : is_moved(false)
  {
    map_sub_ = nh_.subscribe("map", 1, &ReturnToStart::mapCallback, this);    // 绑定 map
    odom_sub_ = nh_.subscribe("odom", 1, &ReturnToStart::odomCallback, this); // 绑定 odometry

    goal_pub_ = nh_.advertise<move_base_msgs::MoveBaseActionGoal>("move_base/goal", 10);
    ROS_INFO("=====================================================");
    ROS_INFO("Return to base initialized.");
  }

  void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr &msg)
  {
    current_map = *msg;
    ROS_INFO("Map data received. width=%d, height=%d, resolution=%.4f",
             current_map.info.width, current_map.info.height, current_map.info.resolution);
  }


  void odomCallback(const nav_msgs::Odometry::ConstPtr &msg)
  {
    geometry_msgs::PoseStamped odom_pose;
    odom_pose.header = msg->header;
    odom_pose.pose = msg->pose.pose;

    auto robot_x = odom_pose.pose.position.x;
    auto robot_y = odom_pose.pose.position.y;

    if(robot_x < 0.08 && robot_y < 0.08) return;

    robot_position[0].push(robot_x);
    robot_position[1].push(robot_y);

    if (robot_position[0].size() > 100)   // 5s * 30Hz
    {
      double robot_x_ = robot_position[0].front();
      double robot_y_ = robot_position[1].front();

      robot_position[0].pop();
      robot_position[1].pop();

      double dx = robot_x - robot_x_;
      double dy = robot_y - robot_y_;
      
      double dist = std::sqrt(dx*dx + dy*dy);

      // 用三元表示
      is_moved = dist > moved_dist_thred ? true : false;
      if(!is_moved){
        ROS_ERROR("Robot has not moved, return to start point.");
        // 更改Goal格式为move_base_msgs/MoveBaseActionGoal
        move_base_msgs::MoveBaseActionGoal goal_msg;
        goal_msg.header.frame_id = "map";
        goal_msg.header.stamp = ros::Time::now();
        goal_msg.goal.target_pose.header = goal_msg.header;
        goal_msg.goal.target_pose.pose.position.x = 0;
        goal_msg.goal.target_pose.pose.position.y = 0;
        goal_msg.goal.target_pose.pose.orientation.w = getRandomAngle();

        ROS_INFO("Publishing goal point: x=%.2f, y=%.2f", 
                goal_msg.goal.target_pose.pose.position.x,
                goal_msg.goal.target_pose.pose.position.y);
        goal_pub_.publish(goal_msg);
        while(!robot_position[0].empty()) robot_position[0].pop();
        while(!robot_position[1].empty()) robot_position[1].pop();

      }
    }

  }

  double getRandomAngle()
  {
    std::uniform_real_distribution<double> angle(0, 3.1);
    return angle(gen_);
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "return");
  ReturnToStart rr;
  ros::spin();
  return 0;
}
