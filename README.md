# cs_local_planner
This is a package containing a plugin for ROS navigation, serves as a local_planner that is inhereted from the `nav_core
::BaseLocalPlanner`. `cs_local_planner` means constant speed local planner.

The reason why I implement this planner is because the default DWA planner is so unpredictable and hard to tune
. It can easily get stuck or become hesitate to move. The planner tries to track the global plan without considering the smoothness
  of the local trajectory, meaning that the robot could stop and adjust its yaw before moving on. This may take more
   time but has a better path tracking performance.

The strategy is simple, when a global path comes, the robot start with an in-place-rotation to head towards the path
. Then it keep a constant velocity moving in segments of straight lines to track the path. 
When it reach the goal, it stop and perform a in-place-rotation again to align with the goal pose.

## TODO

- [x] Track global path instead of global goal.
- [ ] Making sure the global path tracking is able to make steep turns.
- [ ] Consider a global path planner using RRT as well as hand-specified waypoints.
- [ ] Add example usage.