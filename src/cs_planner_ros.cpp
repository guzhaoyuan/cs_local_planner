//
// Created by gzy on 3/18/20.
//

#include "cs_planner_ros.h"

namespace cs_local_planner {

bool CSPlannerROS::computeVelocityCommands(geometry_msgs::Twist& cmd_vel) {
  return true;
}

void CSPlannerROS::initialize(std::string name, tf::TransformListener *tf, costmap_2d::Costmap2DROS *costmap_ros) {

}

bool CSPlannerROS::isGoalReached() {
  return true;
}

bool CSPlannerROS::setPlan(const std::vector< geometry_msgs::PoseStamped > &plan) {
  return true;
}

} // namespace cs_local_planner
