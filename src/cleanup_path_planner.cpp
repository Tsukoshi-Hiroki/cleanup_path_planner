// Copyright 2025 amsl
#include "cleanup_path_planner/cleanup_path_planner.hpp"


CleanupPathPlanner::CleanupPathPlanner() : nh_("~"), tf_buffer_(), tf_listener_(tf_buffer_)
{
  sub_pose_ = nh_.subscribe("/amcl_pose", 1, &CleanupPathPlanner::pose_callback, this, ros::TransportHints().reliable().tcpNoDelay());
  sub_estimated_wall_ = nh_.subscribe("/urinal_wall_estimator/estimated_wall", 1, &CleanupPathPlanner::estimated_wall_callback, this);

  pub_path_ = nh_.advertise<nav_msgs::Path>("/wall_side_path", 1);
  pub_start_point_ = nh_.advertise<geometry_msgs::PoseStamped>("/wall_side_path_start_point", 1);
  // pub_node_point_ = nh_.advertise<geometry_msgs::PointStamped>("/cleanup_node_point", 1);

  // パラメータの読み込み
  nh_.getParam("dist", dist_);
  nh_.getParam("hz", hz_);
  nh_.getParam("dx", dx_);
  nh_.getParam("tmp_start_point_x", tmp_start_point_x_);
  nh_.getParam("tmp_start_point_y", tmp_start_point_y_);

  path_start_point_.header.frame_id = "map";
  path_start_point_.header.stamp = ros::Time::now();
  path_start_point_.pose.position.x = tmp_start_point_x_;
  path_start_point_.pose.position.y = tmp_start_point_y_;
  path_start_point_.pose.position.z = 0.0;
  path_start_point_.pose.orientation.x = 0.0;
  path_start_point_.pose.orientation.y = 0.0;
  path_start_point_.pose.orientation.z = 0.0;
  path_start_point_.pose.orientation.w = 0.0;
}

void CleanupPathPlanner::process()
{
  ros::Rate loop_rate(hz_);
  while (ros::ok())
  {
    judge_path_method();
    /* pub_path_.publish(path_); */
    pub_start_point_.publish(path_start_point_);
    ros::spinOnce();
    loop_rate.sleep();
  }

}

void CleanupPathPlanner::estimated_wall_callback(const urinal_map_msgs::EstimatedWall::ConstPtr &msg)
{
  estimated_wall_ = msg;
}

void CleanupPathPlanner::pose_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
  pose_.header = msg->header;
  pose_.pose = msg->pose.pose;
  ROS_INFO_STREAM("pose_ x: " << pose_.pose.position.x << ", y: " << pose_.pose.position.y << ", z: " << pose_.pose.position.z);
}

void CleanupPathPlanner::judge_path_method()
{
  // path の生成方法を決定するロジックをここに実装
  if(1)
  {
    // ここにpathの生成方法を決定するロジックを実装
    create_approach_path();
  }
  else if(0)
  {
    create_path(estimated_wall_);
  }
}

float CleanupPathPlanner::calc_inclination()
{
  // 自己位置から path_start_point_ までの傾きを計算
  float dx = path_start_point_.pose.position.x - pose_.pose.position.x;
  float dy = path_start_point_.pose.position.y - pose_.pose.position.y;
  if (dx == 0.0) {
    dx = 0.0001; // ゼロ除算を避ける
  }
  float inclination = dy / dx; // 傾きの計算
  ROS_INFO("Calculated inclination: %f", inclination);
  return inclination;
}

void CleanupPathPlanner::path_tf_transformer()
{
  // map座標系からbase_link座標系に変換
  // 変換後のPathメッセージを作成
  nav_msgs::Path transformed_path;
  transformed_path.header.frame_id = "base_link";
  transformed_path.header.stamp = path_.header.stamp;

  // tf2を使って座標変換を取得
  geometry_msgs::TransformStamped transform_stamped;
  try {
    transform_stamped = tf_buffer_.lookupTransform("base_link", path_.header.frame_id, path_.header.stamp, ros::Duration(0.1));
  } catch (tf2::TransformException &ex) {
    ROS_WARN("tf2 Transform failed: %s", ex.what());
    return;
  }

  // 各Poseを変換
  for (const auto &pose : path_.poses) {
    geometry_msgs::PoseStamped transformed_pose;
    tf2::doTransform(pose, transformed_pose, transform_stamped);
    transformed_path.poses.push_back(transformed_pose);
  }
  
  // 変換後のPathをパブリッシュ
  pub_path_.publish(transformed_path);
}

void CleanupPathPlanner::create_approach_path()
{
  path_.poses.clear();
  path_.header.frame_id = "map"; // base_link座標系に変更
  path_.header.stamp = pose_.header.stamp;

  // 自己位置を初期値に設定
  path_.poses.push_back(pose_);

  float inclination = calc_inclination();

  if(pose_.pose.position.x < path_start_point_.pose.position.x) {
    // ロボットが左側にいる場合
    for(int i=1; (pose_.pose.position.x + dx_*i) < path_start_point_.pose.position.x; ++i)
    {
      ROS_INFO("loop iteration: %d", i);
      geometry_msgs::PoseStamped pose;
      pose.header.frame_id = "map";
      pose.header.stamp = ros::Time::now();
      pose.pose.position.x = path_.poses.back().pose.position.x + dx_;
      pose.pose.position.y = path_.poses.back().pose.position.y + inclination * dx_; // 傾きに基づいてy座標を調整
      pose.pose.position.z = 0.0;
      pose.pose.orientation.w = 1.0; // Set orientation to identity
      pose.pose.orientation.x = dx_;
      pose.pose.orientation.y = inclination * dx_;
      pose.pose.orientation.z = 0.0;

      path_.poses.push_back(pose);
    }
    path_tf_transformer();

  } else {
    // ロボットが左側にいる場合
    for(int i=1; (pose_.pose.position.x - dx_*i) > path_start_point_.pose.position.x; ++i)
    {
      ROS_INFO("loop iteration: %d", i);
      geometry_msgs::PoseStamped pose;
      pose.header.frame_id = "map";
      pose.header.stamp = ros::Time::now();
      pose.pose.position.x = path_.poses.back().pose.position.x - dx_;
      pose.pose.position.y = path_.poses.back().pose.position.y - inclination * dx_; // 傾きに基づいてy座標を調整
      pose.pose.position.z = 0.0;
      pose.pose.orientation.w = 1.0; // Set orientation to identity
      pose.pose.orientation.x = dx_;
      pose.pose.orientation.y = inclination * dx_;
      pose.pose.orientation.z = 0.0;

      path_.poses.push_back(pose);
    }
    path_tf_transformer();

  }
}

bool CleanupPathPlanner::side_judge()
{
  float  y = -1.0 * (pose_.pose.position.x * estimated_wall_->a + estimated_wall_->c) / estimated_wall_->b; // y = -1 * (ax + c) / b

  if(pose_.pose.position.y > y) {
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
