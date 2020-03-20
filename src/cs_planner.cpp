//
// Created by gzy on 3/19/20.
//

#include "cs_planner.h"

namespace cs_local_planner {

CSPlanner::CSPlanner(std::string name, base_local_planner::LocalPlannerUtil *planner_util)
: name_(name), planner_util_(planner_util) {
  ros::NodeHandle private_nh("~/" + name);
}

void CSPlanner::reconfigure(CSPlannerConfig &config) {

}

bool CSPlanner::checkTrajectory(Eigen::Vector3f pos, Eigen::Vector3f vel, Eigen::Vector3f vel_samples) {
  return true;
}


} // namespace cs_local_planner