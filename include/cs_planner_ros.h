//
// Created by gzy on 3/18/20.
//

#ifndef SRC_CS_LOCAL_PLANNER_INCLUDE_CS_PLANNER_ROS_H_
#define SRC_CS_LOCAL_PLANNER_INCLUDE_CS_PLANNER_ROS_H_

#include <nav_core/base_local_planner.h>
#include <nav_msgs/Path.h>
#include <base_local_planner/local_planner_util.h>
#include <base_local_planner/latched_stop_rotate_controller.h>
#include <base_local_planner/odometry_helper_ros.h>

#include <tf2_ros/buffer.h>

#include <dynamic_reconfigure/server.h>
#include <cs_local_planner/CSPlannerConfig.h>
#include "cs_planner.h"

namespace cs_local_planner {

class CSPlannerROS : public nav_core::BaseLocalPlanner {
 public:
  CSPlannerROS();

  ~CSPlannerROS();

  bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel);
  void initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros);
  bool isGoalReached();
  bool setPlan(const std::vector< geometry_msgs::PoseStamped > &plan);

 private:
  // Callback to update the local planner's parameters based on dynamic reconfigure.
  void reconfigureCB(CSPlannerConfig &config, uint32_t level);

  bool initialized_;

  bool setup_;
  dynamic_reconfigure::Server<cs_local_planner::CSPlannerConfig>* server_ptr;
  cs_local_planner::CSPlannerConfig default_config_;

  ros::Publisher global_path_pub_;
  ros::Publisher local_path_pub_;

  base_local_planner::LocalPlannerUtil planner_util_;

  tf2_ros::Buffer* tf_;
  costmap_2d::Costmap2DROS* costmap_ros_;

  base_local_planner::LatchedStopRotateController latchedStopRotateController_;
  base_local_planner::OdometryHelperRos odom_helper_;

  CSPlanner* cp_;

};

} // namespace cs_local_planner
#endif //SRC_CS_LOCAL_PLANNER_INCLUDE_CS_PLANNER_ROS_H_
