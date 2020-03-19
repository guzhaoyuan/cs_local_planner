//
// Created by gzy on 3/18/20.
//

#ifndef SRC_CS_LOCAL_PLANNER_INCLUDE_CS_PLANNER_ROS_H_
#define SRC_CS_LOCAL_PLANNER_INCLUDE_CS_PLANNER_ROS_H_

#include <nav_core/base_local_planner.h>
#include <tf2_ros/buffer.h>

namespace cs_local_planner {

class CSPlannerROS : public nav_core::BaseLocalPlanner {
 public:
  CSPlannerROS();

  ~CSPlannerROS();

  bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel);
  void initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros);
  bool isGoalReached();
  bool setPlan(const std::vector< geometry_msgs::PoseStamped > &plan);

};

} // namespace cs_local_planner
#endif //SRC_CS_LOCAL_PLANNER_INCLUDE_CS_PLANNER_ROS_H_
