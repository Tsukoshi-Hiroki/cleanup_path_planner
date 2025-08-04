// Copyright 2025 amsl
#include "cleanup_path_planner/cleanup_path_planner.hpp"


CleanupPathPlanner::CleanupPathPlanner() : nh_("~"), tf_buffer_(), tf_listener_(tf_buffer_)
{
  // subscriber
  sub_pose_ = nh_.subscribe("/amcl_pose", 1, &CleanupPathPlanner::pose_callback, this, ros::TransportHints().reliable().tcpNoDelay());
  sub_estimated_wall_ = nh_.subscribe("/urinal_wall_estimator/estimated_wall", 1, &CleanupPathPlanner::estimated_wall_callback, this);

  // publisher
  pub_path_ = nh_.advertise<nav_msgs::Path>("/wall_side_path", 1);
  pub_start_point_ = nh_.advertise<geometry_msgs::PoseStamped>("/wall_side_path_start_point", 1);

  // service server
  approach_path_svr_ = nh_.advertiseService("/publish_approach_path", &CleanupPathPlanner::approach_path_check, this);
  urinal_cleaning_path_svr_ = nh_.advertiseService("/publish_urinal_cleaning_path", &CleanupPathPlanner::urinal_cleaning_path_check, this);
  stop_path_svr_ = nh_.advertiseService("/stop_publishing_path", &CleanupPathPlanner::stop_path_check, this);

  // service client
  get_urinal_prox_clt_ = nh_.serviceClient<urinal_cleaning_msgs::GetUrinalProximity>("/get_urinal_proximity");

  // パラメータの読み込み
  nh_.getParam("path_method", path_method_);
  nh_.getParam("dist", dist_);
  nh_.getParam("hz", hz_);
  nh_.getParam("dx", dx_);
  nh_.getParam("tmp_start_point_x", tmp_start_point_x_);
  nh_.getParam("tmp_start_point_y", tmp_start_point_y_);
}

void CleanupPathPlanner::process()
{
  ros::Rate loop_rate(hz_);
  while (ros::ok())
  {
    /* pub_path_.publish(path_); */
    ros::spinOnce();
    judge_path_method();
    loop_rate.sleep();
  }

}

void CleanupPathPlanner::estimated_wall_callback(const urinal_cleaning_msgs::EstimatedWall::ConstPtr &msg)
{
  estimated_wall_ = *msg;
}

void CleanupPathPlanner::pose_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
  pose_.header = msg->header;
  pose_.pose = msg->pose.pose;
  ROS_INFO_STREAM("pose_ x: " << pose_.pose.position.x << ", y: " << pose_.pose.position.y << ", z: " << pose_.pose.position.z);
}


bool CleanupPathPlanner::approach_path_check(urinal_cleaning_msgs::PublishApproachPath::Request& req, 
  urinal_cleaning_msgs::PublishApproachPath::Response& res)
{
  // アプローチパスの生成を行う指示を受ける
  ROS_INFO("Approach path check called");
  path_method_ = 1; // アプローチパスを生成する方法を指定
  calc_start_point_called_ = true; // 清掃開始位置の計算を行わせる
  target_offset_x_ = req.target_offset_x;
  target_offset_y_ = req.target_offset_y;
  
  printf("Target offset: x = %f, y = %f\n", target_offset_x_, target_offset_y_);

  // レスポンスの設定
  res.success = true;
  
  return true;
}

bool CleanupPathPlanner::urinal_cleaning_path_check(urinal_cleaning_msgs::PublishUrinalCleaningPath::Request& req, 
  urinal_cleaning_msgs::PublishUrinalCleaningPath::Response& res)
{
  // アプローチパスの生成を行う
  ROS_INFO("Urinal cleaning path check called");
  path_method_ = 2; // アプローチパスを生成する方法を指定
  start_cleaning_offset_ = req.target_offset_x;
  dist_ = req.target_offset_y;
  
  ROS_INFO_STREAM("Start cleaning offset: x = " << start_cleaning_offset_ << ", y = " << dist_);

  // レスポンスの設定
  res.success = true;
  
  return true;
}

bool CleanupPathPlanner::stop_path_check(urinal_cleaning_msgs::StopPublishingPath::Request& req, 
  urinal_cleaning_msgs::StopPublishingPath::Response& res)
{
  // アプローチパスの生成を行う
  ROS_INFO("Stop path check called");
  path_method_ = 0; // アプローチパスを生成する方法を指定
  
  // レスポンスの設定
  res.success = true;
  
  return true;
}

// path の生成方法を決定する関数
void CleanupPathPlanner::judge_path_method()
{
  if(path_method_ == 1)
  {
    // 壁際清掃の開始位置を計算
    if(calc_start_point_called_)
    {
      calc_start_point();
    }
    pub_start_point_.publish(path_start_point_); //見た目用
    // 壁に接近するアプローチパスを生成
    create_approach_path();
  }
  else if(path_method_ == 2)
  {
    // 壁際のパスを生成
    create_path();
  }
  else if(path_method_ == 3)
  {
    // 通常清掃 Path への復帰用 Pathを生成
    
  }else
  {
    path_.poses.clear(); // 初期化
    path_start_point_ = geometry_msgs::PoseStamped(); // 初期化
    pub_path_.publish(path_);

    ROS_INFO("No path generation method selected, skipping path creation.");
  }

}

// 清掃開始位置の計算
void CleanupPathPlanner::calc_start_point()
{
  urinal_cleaning_msgs::GetUrinalProximity get_urinal_prox_srv;
  bool is_get_urinal_prox = false;
  is_get_urinal_prox = get_urinal_prox_clt_.call(get_urinal_prox_srv);

  if(!is_get_urinal_prox) {
    ROS_ERROR("Failed to call service get_urinal_proximity");
    return;
  }
  else {
    ROS_INFO("Successfully called service get_urinal_proximity");
    urinal_cleaning_msgs::UrinalProximity urinal_proximity = get_urinal_prox_srv.response.urinal_proximity;

    // 取得した urinal_proximity から path_start_point_ の座標を設定
    path_start_point_.header.frame_id = "map";
    path_start_point_.header.stamp = ros::Time::now();
    path_start_point_.pose.position.x = urinal_proximity.urinal_info.position.pose.position.x + target_offset_x_;
    path_start_point_.pose.position.y = urinal_proximity.urinal_info.position.pose.position.y + target_offset_y_;
    path_start_point_.pose.position.z = 0.0; // z座標は0に設定
    path_start_point_.pose.orientation.x = 0.0;
    path_start_point_.pose.orientation.y = 0.0;
    path_start_point_.pose.orientation.z = 0.0;
    path_start_point_.pose.orientation.w = 1.0; // 単位クォータニオン
    
    ROS_INFO_STREAM("Start point from urinal map: x = " << path_start_point_.pose.position.x 
                    << ", y = " << path_start_point_.pose.position.y);

    calc_start_point_called_ = false; // 清掃開始位置の計算が行われたので，次のspinで行われないように Flag を立てる
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
    // path の最後の座標をpath_start_point_に設定
    path_.poses.push_back(path_start_point_);

    // Pathの座標系をbase_linkに変換してパブリッシュ
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
    // path の最後の座標をpath_start_point_に設定
    path_.poses.push_back(path_start_point_);

    // Pathの座標系をbase_linkに変換してパブリッシュ
    path_tf_transformer();

  }
}

bool CleanupPathPlanner::side_judge()
{
  float  y = -1.0 * (pose_.pose.position.x * estimated_wall_.a + estimated_wall_.c) / estimated_wall_.b; // y = -1 * (ax + c) / b

  if(pose_.pose.position.y > y) {
    ROS_INFO("robot is on the up side of the wall");
    return true;
  } else {
    ROS_INFO("robot is on the down side of the wall");
    return false;
  }
}

void CleanupPathPlanner::create_path()
{
  pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
  pcl::fromROSMsg(estimated_wall_.wall_points, pcl_cloud);  // ←ここで変換

  path_.poses.clear();
  path_.header.frame_id = "map"; // base_link座標系に変更
  path_.header.stamp = estimated_wall_.header.stamp;

  // 法線方向の算出
  float tan_theta = estimated_wall_.b / estimated_wall_.a; // tan(theta) = b/a
  float theta = atan(tan_theta); // θ = arctan(b/a)

  // Pathの移動距離の計算
  float dist_x = dist_ * cos(theta);
  float dist_y = dist_ * sin(theta);
  float dist_z = 0.0; // z方向の移動は考慮しない


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
    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = "map";
    pose.header.stamp = estimated_wall_.header.stamp;
    pose.pose.position.x = point.x + dist_x; // 法線方向に移動
    pose.pose.position.y = point.y + dist_y; // 法線方向に移動
    pose.pose.position.z = point.z + dist_z; // z方向の移動は考慮しない
    pose.pose.orientation.w = 1.0; // Set orientation to identity
    pose.pose.orientation.x = 0.0;
    pose.pose.orientation.y = 0.0;
    pose.pose.orientation.z = 0.0;

    path_.poses.push_back(pose);
  }

  // Pathの座標系をbase_linkに変換してパブリッシュ
  path_tf_transformer();
}
