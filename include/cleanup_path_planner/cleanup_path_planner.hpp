// Copyright 2025 amsl

#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Path.h>
#include <urinal_cleaning_msgs/EstimatedWall.h>
#include <urinal_cleaning_msgs/GetUrinalMap.h>
#include <urinal_cleaning_msgs/PublishApproachPath.h>
#include <urinal_cleaning_msgs/PublishUrinalCleaningPath.h>
#include <urinal_cleaning_msgs/PublishReturnPath.h>
#include <urinal_cleaning_msgs/StopPublishingPath.h>
#include <urinal_cleaning_msgs/GetUrinalProximity.h>
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

  // ROS Interface
  ros::NodeHandle nh_;
  ros::Publisher pub_path_;
  ros::Publisher pub_start_point_; // デバッグ用
  ros::Subscriber sub_estimated_wall_;
  ros::Subscriber sub_pose_;
  ros::ServiceServer approach_path_svr_;
  ros::ServiceServer urinal_cleaning_path_svr_;
  ros::ServiceServer stop_path_svr_;
  ros::ServiceClient get_urinal_prox_clt_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  // メンバ関数
  void estimated_wall_callback(const urinal_cleaning_msgs::EstimatedWall::ConstPtr &msg);
  void pose_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);
  bool side_judge();
  void create_path();
  void judge_path_method();
  void create_approach_path();
  float calc_inclination();
  void path_tf_transformer();
  void calc_start_point();
  bool approach_path_check(urinal_cleaning_msgs::PublishApproachPath::Request& req, urinal_cleaning_msgs::PublishApproachPath::Response& res);
  bool urinal_cleaning_path_check(urinal_cleaning_msgs::PublishUrinalCleaningPath::Request& req, urinal_cleaning_msgs::PublishUrinalCleaningPath::Response& res);
  bool stop_path_check(urinal_cleaning_msgs::StopPublishingPath::Request& req, urinal_cleaning_msgs::StopPublishingPath::Response& res);

  // その他のメンバ変数
  geometry_msgs::PoseStamped path_start_point_;
  geometry_msgs::PoseStamped pose_;
  urinal_cleaning_msgs::EstimatedWall estimated_wall_;
  nav_msgs::Path path_;

  bool calc_start_point_called_ = false; // 清掃開始位置の計算が行われたかどうか
  int path_method_;
  float dx_;
  float tmp_start_point_x_;
  float tmp_start_point_y_;
  float target_offset_x_; // 清掃開始位置の指定用 offset
  float target_offset_y_; // 清掃開始位置の指定用 offset
  float start_cleaning_offset_; // 清掃開始位置のオフセット
  float dist_; // 推定壁からの距離


};
