// Copyright 2025 amsl
#include "cleanup_path_planner/cleanup_path_planner.hpp"


CleanupPathPlanner::CleanupPathPlanner() : nh_("~"), tf_buffer_(), tf_listener_(tf_buffer_)
{
  sub_pose_ = nh_.subscribe("/amcl_pose", 1, &CleanupPathPlanner::pose_callback, this, ros::TransportHints().reliable().tcpNoDelay());
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
  // pub_path_sub_.publish(path_sub_);
}

void CleanupPathPlanner::pose_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
    pose_ = msg->pose.pose;
    ROS_ERROR_STREAM("pose_ x: " << pose_.position.x << ", y: " << pose_.position.y << ", z: " << pose_.position.z);
}

bool CleanupPathPlanner::side_judge()
{
  float  y = -1.0 * (pose_.position.x * estimated_wall_->a + estimated_wall_->c) / estimated_wall_->b; // y = -1 * (ax + c) / b

  if(pose_.position.y > y) {
    ROS_INFO("robot is on the up side of the wall");
    return true;
  } else {
    ROS_INFO("robot is on the down side of the wall");
    return false;
  }
}

void CleanupPathPlanner::create_path(const urinal_map_msgs::EstimatedWall::ConstPtr &estimated_wall)
{
  pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
  pcl::fromROSMsg(estimated_wall->wall_points, pcl_cloud);  // ←ここで変換

  path_.poses.clear();
  path_.header.frame_id = "base_link"; // base_link座標系に変更
  path_.header.stamp = estimated_wall->header.stamp;

  // 法線方向の算出
  float tan_theta = estimated_wall->b / estimated_wall->a; // tan(theta) = b/a
  float theta = atan(tan_theta); // θ = arctan(b/a)

  // Pathの移動距離の計算
  float dist_x = dist_ * cos(theta);
  float dist_y = dist_ * sin(theta);
  float dist_z = 0.0; // z方向の移動は考慮しない

  geometry_msgs::PoseStamped pose;
  pose.header.frame_id = "map";
  pose.header.stamp = estimated_wall->header.stamp;

  if(side_judge()) {
    // ロボットが壁の上側にいる場合
    dist_x = dist_x; // 法線方向に移動
    dist_y = dist_y; // 法線方向に移動
  } else {
    // ロボットが壁の下側にいる場合
    dist_x = -dist_x; // 法線方向と逆に移動
    dist_y = -dist_y; // 法線方向と逆に移動
  }

  for (const auto &point : pcl_cloud.points) {
    pose.pose.position.x = point.x + dist_x; // 法線方向に移動
    pose.pose.position.y = point.y + dist_y; // 法線方向に移動
    pose.pose.position.z = point.z + dist_z; // z方向の移動は考慮しない

    pose.pose.orientation.w = 1.0; // Set orientation to identity
    pose.pose.orientation.x = 0.0;
    pose.pose.orientation.y = 0.0;
    pose.pose.orientation.z = 0.0;

    // map→base_link座標系へ変換(tf2)
    geometry_msgs::PoseStamped pose_base_link;
    try{
      pose_base_link = tf_buffer_.transform(pose, "base_link", ros::Duration(0.1));
      path_.poses.push_back(pose_base_link);
    }catch (tf2::TransformException &ex)
    {
      ROS_WARN("tf2 Transform failed: %s", ex.what());
      continue;
    }
  }
}
