// AutonomousExploration.cpp
#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <vector>
#include <random>
#include <cmath>
#include <thread>
#include <mutex>

class RRTExploration
{
private:
  struct Node
  {
    double x, y;
    int parent;
    Node(double x_, double y_, int p_ = -1) : x(x_), y(y_), parent(p_) {}
  };

  ros::NodeHandle nh_;
  ros::Subscriber map_sub_;           // Sub map
  ros::Subscriber odom_sub_;          // Sub odometry
  ros::Publisher goal_pub_;           // Pub goal point
  tf::TransformListener tf_listener_; // TF

  // RTT tree
  std::vector<Node> tree_;
  nav_msgs::OccupancyGrid current_map_;

  // 机器人 map系
  double robot_x_;
  double robot_y_;
  bool has_odom_;
  bool map_received_;

  // RRT parameters
  double step_size_ = 0.5; // m
  double goal_bias_ = 0.2;
  int max_iterations_ = 100;

  std::random_device rd_;
  std::mt19937 gen_{rd_()}; // 随机数生成器

  // Exploration 线程
  std::thread explore_thread_;
  std::mutex tree_mutex_;
  bool exploring_;

public:
  RRTExploration()
      : has_odom_(false), map_received_(false), exploring_(false)
  {
    map_sub_ = nh_.subscribe("map", 1, &RRTExploration::mapCallback, this);    // 绑定 map
    odom_sub_ = nh_.subscribe("odom", 1, &RRTExploration::odomCallback, this); // 绑定 odometry

    goal_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("move_base/goal", 10); // 绑定 goal

    ROS_INFO("RRTExploration initialized.");
    ROS_INFO("Waiting for data... [map, odom]");
  }

  // Map 回调
  void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr &msg)
  {
    current_map_ = *msg;
    map_received_ = true;
    ROS_INFO("Map data received. width=%d, height=%d, resolution=%.4f",
             current_map_.info.width, current_map_.info.height, current_map_.info.resolution);

    // 如果odom有数据，且未探索，则开始探索
    if (has_odom_ && !exploring_)
    {
      startExploration();
    }
  }

  // Odom 回调
  void odomCallback(const nav_msgs::Odometry::ConstPtr &msg)
  {
    // Odom 系
    geometry_msgs::PoseStamped odom_pose;
    odom_pose.header = msg->header;
    odom_pose.pose = msg->pose.pose;

    // Map 系
    geometry_msgs::PoseStamped map_pose;
    try
    {
      tf_listener_.transformPose("map", odom_pose, map_pose);
      robot_x_ = map_pose.pose.position.x;
      robot_y_ = map_pose.pose.position.y;
      has_odom_ = true;
      ROS_INFO("Received odometry. x=%f, y=%f", robot_x_, robot_y_);

      // 如果map有数据，且未探索，则开始探索
      if (map_received_ && !exploring_)
      {
        startExploration();
      }
    }
    catch (tf::TransformException &ex)
    {
      ROS_ERROR("Transform failure: %s", ex.what());
    }
  }

  void startExploration()
  {
    if (exploring_)
    {
      ROS_WARN("Exploration already started. Ignoring start request.");
      return;
    }

    exploring_ = true;
    explore_thread_ = std::thread(&RRTExploration::explore, this);
  }

  bool isValidPoint(double x, double y)
  {
    // 检查在地图内
    if (x < 0 || y < 0 ||
        x >= current_map_.info.width * current_map_.info.resolution ||
        y >= current_map_.info.height * current_map_.info.resolution)
      return false;

    // 检查索引
    int map_x = static_cast<int>(x / current_map_.info.resolution);
    int map_y = static_cast<int>(y / current_map_.info.resolution);
    int index = map_y * current_map_.info.width + map_x;
    if (index < 0 || index >= static_cast<int>(current_map_.data.size()))
    {
      ROS_ERROR("Invalid Node - index out of range: x=%.2f, y=%.2f, index=%d", x, y, index);
      return false;
    }

    // 检查是否为未知区域


    return current_map_.data[index] == 0;
  }

  Node getRandomNode()
  {
    std::uniform_real_distribution<> dis_x(0, current_map_.info.width * current_map_.info.resolution);
    std::uniform_real_distribution<> dis_y(0, current_map_.info.height * current_map_.info.resolution);
    return Node(dis_x(gen_), dis_y(gen_));
  }

  int findNearest(const Node &target)
  {
    int nearest = 0;
    double min_dist = std::numeric_limits<double>::max();

    for (size_t i = 0; i < tree_.size(); i++)
    {
      double dist = distance(tree_[i], target);
      if (dist < min_dist)
      {
        min_dist = dist;
        nearest = i;
      }
    }
    return nearest;
  }

  double distance(const Node &a, const Node &b)
  {
    return std::sqrt(std::pow(a.x - b.x, 2) + std::pow(a.y - b.y, 2));
  }

  // 创建点
  Node steer(const Node &from, const Node &to)
  {
    double dist = distance(from, to);
    if (dist <= step_size_)
    {
      ROS_INFO("Steer: Distance %.2f <= step_size %.2f, returning target node", dist, step_size_);
      return to;
    }

    double theta = atan2(to.y - from.y, to.x - from.x);
    Node new_node(from.x + step_size_ * cos(theta),
                  from.y + step_size_ * sin(theta));
    ROS_INFO("Steer: Created new node at x=%.2f, y=%.2f", new_node.x, new_node.y);
    return new_node;
  }

  // 发布 Goal
  void publishGoal(const Node &target)
  {
    geometry_msgs::PoseStamped goal_msg;
    // header
    goal_msg.header.frame_id = "map";
    goal_msg.header.stamp = ros::Time::now();
    // pose
    goal_msg.pose.position.x = target.x;
    goal_msg.pose.position.y = target.y;
    goal_msg.pose.orientation.w = 0.0; // 旋转 无所谓

    ROS_INFO("Publishing goal point: x=%.2f, y=%.2f", goal_msg.pose.position.x, goal_msg.pose.position.y);
    goal_pub_.publish(goal_msg);
  }

  // RRT 
  // 随机发布最多100个点
  void explore()
  {
    tree_.clear();
    {
      std::lock_guard<std::mutex> lock(tree_mutex_);
      tree_.emplace_back(robot_x_, robot_y_); 
    }
    ROS_INFO("Starting RRT exploration, start point: x=%.2f, y=%.2f", robot_x_, robot_y_);

    ros::Rate rate(10); // 10 Hz
    for (int i = 0; i < max_iterations_; i++)
    {
      if (!ros::ok())
      {
        ROS_WARN("ROS shutdown, stopping exploration.");
        break;
      }

      // 随机点
      Node random_node = getRandomNode();
      int nearest_idx;
      {
        std::lock_guard<std::mutex> lock(tree_mutex_);
        nearest_idx = findNearest(random_node);
      }

      // 创建点
      Node new_node = steer(tree_[nearest_idx], random_node);

      // 检查点
      if (!isValidPoint(new_node.x, new_node.y))
      {
        continue;
      }

      // 存储点
      new_node.parent = nearest_idx;
      {
        std::lock_guard<std::mutex> lock(tree_mutex_);
        tree_.push_back(new_node);
      }

      // 发布点
      ROS_INFO("Publishing new node to Goal: x=%.2f, y=%.2f", new_node.x, new_node.y);
      publishGoal(new_node);
      ROS_INFO("New node published.");

      // 延时
      rate.sleep();
    }

    ROS_INFO("RRT exploration completed.");
    exploring_ = false;
  }

  ~RRTExploration()
  {
    // 清理线程
    if (explore_thread_.joinable())
    {
      explore_thread_.join();
    }
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "rrt_exploration");
  RRTExploration rrt_explorer;
  ros::spin();
  return 0;
}