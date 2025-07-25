// Copyright 2025 amsl

#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Path.h>
#include <urinal_map_msgs/EstimatedWall.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

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
  ros::Publisher pub_path_;
  ros::Publisher pub_start_point_;
  ros::Subscriber sub_estimated_wall_;
  ros::Subscriber sub_pose_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  // メンバ関数
  void estimated_wall_callback(const urinal_map_msgs::EstimatedWall::ConstPtr &msg);
  void pose_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);
  bool side_judge();
  void create_path(const urinal_map_msgs::EstimatedWall::ConstPtr &msg);
  void judge_path_method();
  void create_approach_path();
  float calc_inclination();
  void path_tf_transformer();

  // その他のメンバ変数
  geometry_msgs::PoseStamped path_start_point_;
  geometry_msgs::PoseStamped pose_;
  urinal_map_msgs::EstimatedWall::ConstPtr estimated_wall_;
  nav_msgs::Path path_;
  nav_msgs::Path path_sub_;

  float dist_;
  float dx_;
  float tmp_start_point_x_;
  float tmp_start_point_y_;


};
