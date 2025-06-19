// Copyright 2025 amsl
#include "cleanup_path_planner/cleanup_path_planner.hpp"


CleanupPathPlanner::CleanupPathPlanner() : nh_("~")
{
  sub_estimated_wall_ = nh_.subscribe("/urinal_wall_estimator/estimated_wall", 1, &CleanupPathPlanner::estimated_wall_callback, this);

  pub_path_ = nh_.advertise<nav_msgs::Path>("/cleanup_path", 1);
  pub_node_point_ = nh_.advertise<geometry_msgs::PointStamped>("/cleanup_node_point", 1);

  hz_ = 10;
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
}


void CleanupPathPlanner::create_path(const urinal_map_msgs::EstimatedWall::ConstPtr &estimated_wall)
{
  pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
  pcl::fromROSMsg(estimated_wall->wall_points, pcl_cloud);  // ←ここで変換

  path_.poses.clear();
  path_.header.frame_id = "map";
  path_.header.stamp = ros::Time::now();

  for (const auto &point : pcl_cloud.points) {
    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = point.x;
    pose.pose.position.y = point.y;
    pose.pose.position.z = point.z;

    pose.pose.orientation.w = 1.0; // Set orientation to identity
    pose.pose.orientation.x = 0.0;
    pose.pose.orientation.y = 0.0;
    pose.pose.orientation.z = 0.0;
    pose.header.frame_id = "map";
    pose.header.stamp = ros::Time::now();

    path_.poses.push_back(pose);
    // …処理続く
  }
}
