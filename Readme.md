# Constrained Planning Experiment 

This is done for as part of the course requirement for CS6244: Robot Motion Planning and Control.

## Set up

You'll need our modified version of MoveIt which incorporates the line constraints. In your catkin workspace, execute the following commands:

```bash
git clone https://github.com/yewzijian/moveit.git
git clone https://github.com/yewzijian/moveit_msgs.git
git clone https://github.com/ros-planning/moveit_visual_tools.git

catkin build
```



## Speedup compile time

To make development easier, the MoveIt! is consolidated in a large code repository. If you would like to reduce the compile time, you can disable certain unneeded packages from being built using `catkin-tools`. 

```bash
catkin config --blacklist moveit_commander moveit_setup_assistant moveit_fake_controller_manager  moveit_ros_benchmarks moveit_controller_manager_example chomp_motion_planner moveit_planners_chomp
```



## Running simple demo

1. First make sure, you have sourced the workspace:

   ```bash
   source [catkin_ws]/devel/setup.bash
   ```

2. Open two terminals. In the first window:

   ```bash
   roslaunch panda_moveit_config demo.launch
   ```

3. In the second terminal:

   ```
   rosrun constrained_expt simple_demo
   ```

The demo will run through 3 planning stages:

* Unconstrained planning
* Trajectory constrained to a line
* Trajectory constrained such that the end effector remains upright.



## Running evaluation script

The code is set to evaluate only a single algorithm for speed. Also, note that some of the test cases might not have a solution.

1. Open two terminals. In the first window:

   ```bash
   roslaunch panda_moveit_config demo.launch
   ```

2. First generate the test cases. It'll be stored as "problems1.txt" in your workspace directory.

   ```
   rosrun constrained_expt generate_problems
   ```

3. Next run the evaluation code.

   ```
   rosrun constrained_expt run_eval
   ```


Currently, it is observed that some of the generated trajectories are not feasible. However we do not have a way to determine such cases and need manual verification.