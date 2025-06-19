// Copyright 2025 amsl

#include "cleanup_path_planner/cleanup_path_planner.hpp"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "cleanup_path_planner");

  CleanupPathPlanner cupp;
  cupp.process();

  return 0;
}
