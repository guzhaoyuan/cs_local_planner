//
// Created by gzy on 3/18/20.
//

#include "cs_planner_ros.h"

#include <pluginlib/class_list_macros.h>
#include <base_local_planner/goal_functions.h>

#include <tf2/utils.h>

//register this planner as a BaseLocalPlanner plugin
PLUGINLIB_EXPORT_CLASS(cs_local_planner::CSPlannerROS, nav_core::BaseLocalPlanner)

namespace cs_local_planner {

CSPlannerROS::CSPlannerROS() : initialized_(false), odom_helper_("odom"), setup_(false) {
}

CSPlannerROS::~CSPlannerROS() {
}

void CSPlannerROS::reconfigureCB(CSPlannerConfig &config, uint32_t level) {
  if (setup_ && config.restore_defaults) {
    config = default_config_;
    config.restore_defaults = false;
  }
  if ( ! setup_) {
    default_config_ = config;
    setup_ = true;
  }

  // Update generic local planner params.
  base_local_planner::LocalPlannerLimits limits;

  limits.max_vel_trans = config.max_vel_trans;
  limits.min_vel_trans = config.min_vel_trans;
  limits.max_vel_x = config.max_vel_x;
  limits.min_vel_x = config.min_vel_x;
  limits.max_vel_y = config.max_vel_y;
  limits.min_vel_y = config.min_vel_y;
  limits.max_vel_theta = config.max_vel_theta;
  limits.min_vel_theta = config.min_vel_theta;
  limits.acc_lim_x = config.acc_lim_x;
  limits.acc_lim_y = config.acc_lim_y;
  limits.acc_lim_theta = config.acc_lim_theta;
  limits.acc_lim_trans = config.acc_lim_trans;
  limits.xy_goal_tolerance = config.xy_goal_tolerance;
  limits.yaw_goal_tolerance = config.yaw_goal_tolerance;
  limits.prune_plan = config.prune_plan;
  limits.trans_stopped_vel = config.trans_stopped_vel;
  limits.theta_stopped_vel = config.theta_stopped_vel;

  planner_util_.reconfigureCB(limits, config.restore_defaults);

  // update dwa specific configuration
  cp_->reconfigure(config);
}

bool CSPlannerROS::computeVelocityCommands(geometry_msgs::Twist& cmd_vel) {
  // Dispatches to either use my control or stop and rotate control, depending on whether we are close enough to goal.
  geometry_msgs::PoseStamped current_pose_;

  if ( ! costmap_ros_->getRobotPose(current_pose_)) {
    ROS_ERROR("Could not get robot pose");
    return false;
  }
  std::vector<geometry_msgs::PoseStamped> transformed_plan;
  if ( ! planner_util_.getLocalPlan(current_pose_, transformed_plan)) {
    ROS_ERROR("Could not get local plan");
    return false;
  }
  // If the global plan passed in is empty, do nothing.
  if(transformed_plan.empty()) {
    ROS_WARN_NAMED("cs_local_planner", "Received an empty plan.");
    return false;
  }
  ROS_DEBUG_NAMED("cs_local_planner", "Received a transformed plan with %zu points.", transformed_plan.size());

  if (latchedStopRotateController_.isPositionReached(&planner_util_, current_pose_)) {
    ROS_DEBUG("Position reached.");
    // Publish an empty plan because we've reached our goal position.
    std::vector<geometry_msgs::PoseStamped> local_plan;
    std::vector<geometry_msgs::PoseStamped> transformed_plan;
    base_local_planner::publishPlan(transformed_plan, global_path_pub_);
    base_local_planner::publishPlan(local_plan, local_path_pub_);

    base_local_planner::LocalPlannerLimits limits = planner_util_.getCurrentLimits();

//    cmd_vel.angular.z = 0;
//    return true;
    return latchedStopRotateController_.computeVelocityCommandsStopRotate(
        cmd_vel,
        limits.getAccLimits(),
        cp_->getSimPeriod(),
        &planner_util_,
        odom_helper_,
        current_pose_,
        boost::bind(&CSPlanner::checkTrajectory, cp_, _1, _2, _3));
  } else {
    // We assume the global goal is the last point in the global plan.
    geometry_msgs::PoseStamped goal_pose;
    if ( ! planner_util_.getGoal(goal_pose)) {
      ROS_ERROR("Could not get goal pose");
      return false;
    }

    geometry_msgs::PoseStamped moving_target;

    if (base_local_planner::getGoalPositionDistance(current_pose_, goal_pose.pose.position.x, goal_pose.pose.position.y) < 0.5) {
      target_ID = transformed_plan.size()-1;
      moving_target = goal_pose;
    }
    while (target_ID < transformed_plan.size()-1) {
        moving_target = transformed_plan.at(target_ID);
        double distance = base_local_planner::getGoalPositionDistance(current_pose_, moving_target.pose.position.x, moving_target.pose.position.y);
        ROS_INFO("Distance to moving target: %.2f.", distance);
        if (distance > 0.2)
            break;
        target_ID ++;
    }
    pose_pub_.publish(moving_target);

    double curr_heading_to_goal_pos_W = std::atan2(moving_target.pose.position.y - current_pose_.pose.position.y,
                                                   moving_target.pose.position.x - current_pose_.pose.position.x);
    double curr_heading_to_goal_pos_O = base_local_planner::getGoalOrientationAngleDifference(current_pose_,
                                                                                              curr_heading_to_goal_pos_W);
    ROS_DEBUG("Angle to goal: %.2f.", curr_heading_to_goal_pos_O);

    geometry_msgs::PoseStamped robot_vel;
    odom_helper_.getRobotVel(robot_vel);
    // If the robot is stationary, try rotate towards goal first.
    if (std::sqrt(pow(robot_vel.pose.position.x,2)+pow(robot_vel.pose.position.y,2)) < 0.1) {
      std::vector<geometry_msgs::PoseStamped> local_plan;
      std::vector<geometry_msgs::PoseStamped> transformed_plan;
      base_local_planner::publishPlan(transformed_plan, global_path_pub_);
      base_local_planner::publishPlan(local_plan, local_path_pub_);

      if (std::abs(curr_heading_to_goal_pos_O) > 0.1) {
        cmd_vel.angular.z = curr_heading_to_goal_pos_O > 0 ? 0.3 : -0.3;
        return true;
      }
    }

    base_local_planner::publishPlan(transformed_plan, global_path_pub_);

    cmd_vel.linear.x = 0.5;
    // P control on heading direction, with dead zone.
    const double kp = 1;
    cmd_vel.angular.z = std::abs(curr_heading_to_goal_pos_O) < 0.1 ? 0 : kp * curr_heading_to_goal_pos_O;
  }

  return true;
}

void CSPlannerROS::initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros) {
  if (!initialized_) {
    ros::NodeHandle private_nh("~/" + name);
    global_path_pub_ = private_nh.advertise<nav_msgs::Path>("global_plan", 1);
    local_path_pub_ = private_nh.advertise<nav_msgs::Path>("local_plan", 1);
    pose_pub_ = private_nh.advertise<geometry_msgs::PoseStamped>("tracking_pose", 1);

    tf_ = tf;
    costmap_ros_ = costmap_ros;
    costmap_2d::Costmap2D* costmap = costmap_ros_->getCostmap();

    planner_util_.initialize(tf, costmap, costmap_ros_->getGlobalFrameID());

    odom_helper_.setOdomTopic( "/odom" );

    server_ptr = new dynamic_reconfigure::Server<CSPlannerConfig>(private_nh);
    dynamic_reconfigure::Server<CSPlannerConfig>::CallbackType cb = boost::bind(&CSPlannerROS::reconfigureCB, this,
        _1, _2);
    server_ptr->setCallback(cb);

    cp_ = new CSPlanner(name, &planner_util_);

    initialized_ = true;
  } else {
    ROS_WARN("This planner has already been initialized, doing nothing.");
  }
}

bool CSPlannerROS::isGoalReached() {
  geometry_msgs::PoseStamped current_pose_;

  if ( ! costmap_ros_->getRobotPose(current_pose_)) {
    ROS_ERROR("Could not get robot pose");
    return false;
  }

  if (latchedStopRotateController_.isGoalReached(&planner_util_, odom_helper_, current_pose_)) {
    ROS_INFO("Goal reached");
    return true;
  }

  return false;
}

bool CSPlannerROS::setPlan(const std::vector< geometry_msgs::PoseStamped > &plan) {

  if (!plan.empty()) {
    ROS_INFO("Got new plan.");
    latchedStopRotateController_.resetLatching();

    nav_msgs::Path global_path;
    global_path.header = plan.front().header;
    global_path.poses = plan;
    global_path_pub_.publish(global_path);
    planner_util_.setPlan(plan);
    target_ID = std::min(10, (int)plan.size()-1);
    return true;
  } else {
    ROS_ERROR("Got empty plan.");
  }

  return false;
}

} // namespace cs_local_planner
