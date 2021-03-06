# cs_local_planner

This is a package containing a plugin for ROS navigation, serves as a local_planner that is inhereted from the `nav_core
::BaseLocalPlanner`. `cs_local_planner` means constant speed local planner.

The reason why I implement this planner is because the default DWA planner is so unpredictable and hard to tune
. It can easily get stuck or become hesitate to move. The planner tries to track the global plan without considering the smoothness
  of the local trajectory, meaning that the robot could stop and adjust its yaw before moving on. This may take more
   time but has a better path tracking performance.

The strategy is simple. When a global path comes, the robot start with an in-place-rotation to head towards the path
. Then it keep a constant velocity to track the path. When it reaches the goal, it stops and performs a in-place
-rotation to align with the goal pose.

## Example

A [demo](https://github.com/guzhaoyuan/autolabor_simulation/blob/master/simulation_launch/launch/local_planner.launch
) of using cs_local_planner on a simulated differential drive car.

![example](meta/cs_local_planner.png)


## TODO

- [x] Solve the overshooting and using forward looking to get best tracking performance.
- [x] Track global path instead of global goal.
- [x] Add move_base config for mat6 robot.
- [ ] Add safety feature when the global plan directs to a obstacle, local planner should prevent this and stop.
- [x] Add joy stick control feature.
- [x] Add speed and acceleration limits.
- [x] Remove the local prediction trajectory after goal reached.
- [x] Solve the turning jiggering motion.
- [x] Making sure the global path tracking is able to make steep turns.
- [ ] Consider a global path planner using RRT as well as hand-specified waypoints.
- [x] Add example usage.
