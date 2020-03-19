//
// Created by gzy on 3/18/20.
//

#include "cs_planner_ros.h"

#include <pluginlib/class_list_macros.h>
//register this planner as a BaseLocalPlanner plugin
PLUGINLIB_EXPORT_CLASS(cs_local_planner::CSPlannerROS, nav_core::BaseLocalPlanner)

namespace cs_local_planner {

CSPlannerROS::CSPlannerROS() {

}

CSPlannerROS::~CSPlannerROS() {
}

bool CSPlannerROS::computeVelocityCommands(geometry_msgs::Twist& cmd_vel) {
  return true;
}

void CSPlannerROS::initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros) {
}

bool CSPlannerROS::isGoalReached() {
  return true;
}

bool CSPlannerROS::setPlan(const std::vector< geometry_msgs::PoseStamped > &plan) {
  return true;
}

} // namespace cs_local_planner
