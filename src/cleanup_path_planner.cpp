// Copyright 2025 amsl
#include "cleanup_path_planner/cleanup_path_planner.hpp"


CleanupPathPlanner::CleanupPathPlanner() : nh_("~")
{
  sub_estimated_wall_ = nh_.subscribe("/urinal_wall_estimator/estimated_wall", 1, &CleanupPathPlanner::estimated_wall_callback, this);

  pub_path_ = nh_.advertise<nav_msgs::Path>("/cleanup_path", 1);
  pub_path_sub_ = nh_.advertise<nav_msgs::Path>("/cleanup_path_sub", 1);
  // pub_node_point_ = nh_.advertise<geometry_msgs::PointStamped>("/cleanup_node_point", 1);

  // パラメータの読み込み
  nh_.param("dist", dist_, 0.5f); // デフォルト値を0.5に設定
  nh_.param("hz", hz_, 10); // デフォルト値を10に設定
}

void CleanupPathPlanner::process()
{
  ros::Rate loop_rate(hz_);
  while (ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }

}

void CleanupPathPlanner::estimated_wall_callback(const urinal_map_msgs::EstimatedWall::ConstPtr &msg)
{
  estimated_wall_ = msg;
  create_path(estimated_wall_);
  pub_path_.publish(path_);
  pub_path_sub_.publish(path_sub_);
}


void CleanupPathPlanner::create_path(const urinal_map_msgs::EstimatedWall::ConstPtr &estimated_wall)
{
  pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
  pcl::fromROSMsg(estimated_wall->wall_points, pcl_cloud);  // ←ここで変換

  path_.poses.clear();
  path_.header.frame_id = "map";
  // path_.header.stamp = ros::Time::now();
  path_.header.stamp = estimated_wall->header.stamp; // 受信したメッセージのタイムスタンプを使用

  // 法線方向の算出
  float tan_theta = estimated_wall->b / estimated_wall->a; // tan(theta) = b/a
  float theta = atan(tan_theta); // θ = arctan(b/a)

  // Pathの移動距離の計算
  float dist_x = dist_ * cos(theta);
  float dist_y = dist_ * sin(theta);
  float dist_z = 0.0; // z方向の移動は考慮しない

  geometry_msgs::PoseStamped pose;
  pose.header.frame_id = "map";
  // pose.header.stamp = ros::Time::now();
  pose.header.stamp = estimated_wall->header.stamp; // 受信したメッセージのタイムスタンプを使用

  for (const auto &point : pcl_cloud.points) {
    pose.pose.position.x = point.x + dist_x; // 法線方向に移動
    pose.pose.position.y = point.y + dist_y; // 法線方向に移動
    pose.pose.position.z = point.z + dist_z; // z方向の移動は考慮しない

    pose.pose.orientation.w = 1.0; // Set orientation to identity
    pose.pose.orientation.x = 0.0;
    pose.pose.orientation.y = 0.0;
    pose.pose.orientation.z = 0.0;

    path_.poses.push_back(pose);
    // …処理続く
  }


  path_sub_.poses.clear();
  path_sub_.header.frame_id = "map";
  // path_.header.stamp = ros::Time::now();
  path_sub_.header.stamp = estimated_wall->header.stamp; // 受信したメッセージのタイムスタンプを使用

  geometry_msgs::PoseStamped pose_sub;
  pose_sub.header.frame_id = "map";
  // pose.header.stamp = ros::Time::now();
  pose_sub.header.stamp = estimated_wall->header.stamp; // 受信したメッセージのタイムスタンプを使用

  for (const auto &point : pcl_cloud.points) {
    pose_sub.pose.position.x = point.x - dist_x; // 法線方向に移動
    pose_sub.pose.position.y = point.y - dist_y; // 法線方向に移動
    pose_sub.pose.position.z = point.z - dist_z; // z方向の移動は考慮しない

    pose_sub.pose.orientation.w = 1.0; // Set orientation to identity
    pose_sub.pose.orientation.x = 0.0;
    pose_sub.pose.orientation.y = 0.0;
    pose_sub.pose.orientation.z = 0.0;

    path_sub_.poses.push_back(pose_sub);
    // …処理続く
  }

}
