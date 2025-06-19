// Copyright 2025 amsl

#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <nav_msgs/Path.h>
#include <urinal_map_msgs/EstimatedWall.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

class CleanupPathPlanner
{
public:

  // コンストラクタ
  CleanupPathPlanner(void);

  //メンバ関数
  void process();

  //メンバ変数
  int hz_;


protected:

  // pub sub
  ros::NodeHandle nh_;
  ros::Publisher pub_node_point_;
  ros::Publisher pub_path_;
  ros::Subscriber sub_estimated_wall_;

  // メンバ関数
  void estimated_wall_callback(const urinal_map_msgs::EstimatedWall::ConstPtr &msg);
  void create_path(const urinal_map_msgs::EstimatedWall::ConstPtr &msg);

  // その他のメンバ変数
  urinal_map_msgs::EstimatedWall::ConstPtr estimated_wall_;
  nav_msgs::Path path_;
  geometry_msgs::PointStamped node_point_;


};
