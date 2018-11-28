/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, SRI International
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of SRI International nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Sachin Chitta, Dave Coleman, Mike Lautman */

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "move_group_interface_tutorial");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  // BEGIN_TUTORIAL
  //
  // Setup
  // ^^^^^
  //
  // MoveIt! operates on sets of joints called "planning groups" and stores them in an object called
  // the `JointModelGroup`. Throughout MoveIt! the terms "planning group" and "joint model group"
  // are used interchangably.
  static const std::string PLANNING_GROUP = "panda_arm";

  // The :move_group_interface:`MoveGroup` class can be easily
  // setup using just the name of the planning group you would like to control and plan for.
  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);

  // const std::string plannerStr = "RRTstarkConfigDefault";
  const std::string plannerStr = "RRTConnectkConfigDefault";
  // const std::string plannerStr = "PRMkConfigDefault";

  std::map<std::string, std::string> plannerParams = move_group.getPlannerParams(plannerStr, PLANNING_GROUP);
  std::cout << "Default parameters" << std::endl;
  std::cout << "==================" << std::endl;
  for (auto it : plannerParams){
    std::cout << it.first << ": " << it.second << std::endl;
  }
  plannerParams["range"] = "0.001";
  move_group.setPlannerParams(plannerStr, PLANNING_GROUP, plannerParams, true);

  // plannerParams = move_group.getPlannerParams(plannerStr, PLANNING_GROUP);
  // std::cout << "New parameters" << std::endl;
  // std::cout << "==================" << std::endl;
  // for (auto it : plannerParams){
  //   std::cout << it.first << ": " << it.second << std::endl;
  // }
  move_group.setPlannerId(plannerStr);

  // We will use the :planning_scene_interface:`PlanningSceneInterface`
  // class to add and remove collision objects in our "virtual world" scene
 // moveit::planning_interface::PlanningSceneInterface planning_scene_interface;


  ROS_INFO_STREAM("here 1");




  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
  ROS_INFO("Model frame: %s", kinematic_model->getModelFrame().c_str());

  planning_scene::PlanningScene planning_scene(kinematic_model);

  robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));
  kinematic_state->setToDefaultValues();
  const robot_state::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup("panda_arm");
  robot_state::RobotState& current_state = planning_scene.getCurrentStateNonConst();
  // Raw pointers are frequently used to refer to the planning group for improved performance.
//  const robot_state::JointModelGroup* joint_model_group =
//      move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);


  ROS_INFO_STREAM("here 2");

  // Visualization
  // ^^^^^^^^^^^^^
  //
  // The package MoveItVisualTools provides many capabilties for visualizing objects, robots,
  // and trajectories in RViz as well as debugging tools such as step-by-step introspection of a script
  namespace rvt = rviz_visual_tools;
  moveit_visual_tools::MoveItVisualTools visual_tools("panda_link0");
  visual_tools.deleteAllMarkers();

  // Remote control is an introspection tool that allows users to step through a high level script
  // via buttons and keyboard shortcuts in RViz
  visual_tools.loadRemoteControl();

  // RViz provides many types of markers, in this demo we will use text, cylinders, and spheres
  Eigen::Affine3d text_pose = Eigen::Affine3d::Identity();
  text_pose.translation().z() = 1.75;
  visual_tools.publishText(text_pose, "MoveGroupInterface Demo", rvt::WHITE, rvt::XLARGE);

  // Batch publishing is used to reduce the number of messages being sent to RViz for large visualizations
  visual_tools.trigger();

  moveit::core::RobotStatePtr current_state0 = move_group.getCurrentState();
  std::vector<double> joint_group_positions;
  current_state0->copyJointGroupPositions(joint_model_group, joint_group_positions);
  for (int i=0; i<joint_group_positions.size(); i++) {
    ROS_INFO_STREAM(i << " - " << joint_group_positions[i]);
  }
  joint_group_positions[0] = 0;
  joint_group_positions[1] = -0.785;
  joint_group_positions[2] = 0;
  joint_group_positions[3] = -2.356;
  joint_group_positions[4] = 0;
  joint_group_positions[5] = 1.571;
  joint_group_positions[6] = 0;
  move_group.setJointValueTarget(joint_group_positions);
  moveit::planning_interface::MoveGroupInterface::Plan my_plan0;
  bool success0 = (move_group.plan(my_plan0) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO_NAMED("tutorial", "Visualizing plan 2 (joint space goal) %s", success0 ? "" : "FAILED");
  move_group.move();

  // Getting Basic Information
  // ^^^^^^^^^^^^^^^^^^^^^^^^^
  //
  // We can print the name of the reference frame for this robot.
  ROS_INFO_NAMED("tutorial", "Reference frame: %s", move_group.getPlanningFrame().c_str());

  // We can also print the name of the end-effector link for this group.
  ROS_INFO_NAMED("tutorial", "End effector link: %s", move_group.getEndEffectorLink().c_str());

  // Start the demo
  // ^^^^^^^^^^^^^^^^^^^^^^^^^
  ROS_INFO_STREAM("Demo starting... First: Planning to a Pose goal");
  visual_tools.prompt("Press 'next' to plan to a poes goal");
  // Planning to a Pose goal
  // ^^^^^^^^^^^^^^^^^^^^^^^
  // We can plan a motion for this group to a desired pose for the
  // end-effector.
  geometry_msgs::Pose start_pose1 = move_group.getCurrentPose().pose;
  ROS_INFO_STREAM("Starting position: (" 
    << start_pose1.position.x << ", "
    << start_pose1.position.y << ", "
    << start_pose1.position.z << ")");

  geometry_msgs::Pose target_pose1 = start_pose1;
  target_pose1.position.x = 0.6; //target_pose1.position.x = 0.285075;
  target_pose1.position.y = 0.25; //target_pose1.position.y = 0.25;
  target_pose1.position.z = 0.5; //target_pose1.position.z = 0.5;
  move_group.setPoseTarget(target_pose1);
  move_group.setPlanningTime(20.0);

  // Now, we call the planner to compute the plan and visualize it.
  // Note that we are just planning, not asking move_group
  // to actually move the robot.
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;

  bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

  // Visualizing plans
  // ^^^^^^^^^^^^^^^^^
  // We can also visualize the plan as a line with markers in RViz.
  ROS_INFO_NAMED("tutorial", "Visualizing plan 1 as trajectory line");
  visual_tools.publishAxisLabeled(target_pose1, "pose1");
  visual_tools.publishText(text_pose, "Pose Goal", rvt::WHITE, rvt::XLARGE);
  visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
  visual_tools.trigger();

    move_group.move();

  // /*
  //  * Planning with straight line constraint
  //  */
  // visual_tools.prompt("Press 'next' to plan with line constraints");
  // geometry_msgs::Pose start_pose2 = move_group.getCurrentPose().pose;
  
<<<<<<< Updated upstream
  geometry_msgs::Pose target_pose2 = start_pose1;
  // target_pose2.position.x = 0.6; //target_pose2.position.x = 0.285075;
  // target_pose2.position.y = 0.25; //target_pose2.position.y = 0.25;
  // target_pose2.position.z = 0.5; //target_pose2.position.z = 0.837984;

  moveit_msgs::LineConstraint lcm;
  lcm.link_name = "panda_link8";
  lcm.header.frame_id = "panda_link0";
  lcm.line_start.x = start_pose2.position.x;
  lcm.line_start.y = start_pose2.position.y;
  lcm.line_start.z = start_pose2.position.z;
  lcm.line_end.x = target_pose2.position.x;
  lcm.line_end.y = target_pose2.position.y;
  lcm.line_end.z = target_pose2.position.z;
  lcm.tolerance = 0.01;
  lcm.weight = 1.0;

  moveit_msgs::OrientationConstraint ocm2;
  ocm2.link_name = "panda_link8";
  ocm2.header.frame_id = "panda_link0";
  ocm2.orientation.w = start_pose2.orientation.w;
  ocm2.orientation.x = start_pose2.orientation.x;
  ocm2.orientation.y = start_pose2.orientation.y;
  ocm2.orientation.z = start_pose2.orientation.z;
  ocm2.absolute_x_axis_tolerance = 0.01;
  ocm2.absolute_y_axis_tolerance = 0.01;
  ocm2.absolute_z_axis_tolerance = 0.01;
  ocm2.weight = 1.0;

  // Now, set it as the path constraint for the group.
  moveit_msgs::Constraints test_constraints2;
  test_constraints2.line_constraints.push_back(lcm);
  // test_constraints2.orientation_constraints.push_back(ocm2);
  move_group.setPathConstraints(test_constraints2);

  move_group.setStartStateToCurrentState();
  move_group.setPoseTarget(target_pose2);

  // Planning with constraints can be slow because every sample must call an inverse kinematics solver.
  // Lets increase the planning time from the default 5 seconds to be sure the planner has enough time to succeed.
  move_group.setPlanningTime(20.0);

  ROS_INFO_STREAM("Starting position: (" 
    << start_pose2.position.x << ", "
    << start_pose2.position.y << ", "
    << start_pose2.position.z << ")");
  ROS_INFO_STREAM("End position: (" 
    << target_pose2.position.x << ", "
    << target_pose2.position.y << ", "
    << target_pose2.position.z << ")");


  success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO_NAMED("tutorial", "Visualizing plan 2 (line constraints) %s", success ? "" : "FAILED");

  // Visualize the plan in RViz
  visual_tools.deleteAllMarkers();
  visual_tools.publishAxisLabeled(start_pose2, "start");
  visual_tools.publishAxisLabeled(target_pose2, "goal");
  visual_tools.publishText(text_pose, "Constrained Goal", rvt::WHITE, rvt::XLARGE);
  visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
  visual_tools.trigger();

  move_group.move();

  // When done with the path constraint be sure to clear it.
  move_group.clearPathConstraints();

  // Since we set the start state we have to clear it before planning other paths
  move_group.setStartStateToCurrentState();
=======
  // geometry_msgs::Pose target_pose2 = start_pose2;
  // target_pose2.position.x = 0.6;
  // target_pose2.position.y = 0.25;
  // target_pose2.position.z = 0.5;

  // moveit_msgs::LineConstraint lcm;
  // lcm.link_name = "panda_link8";
  // lcm.header.frame_id = "panda_link0";
  // lcm.line_start.x = start_pose2.position.x;
  // lcm.line_start.y = start_pose2.position.y;
  // lcm.line_start.z = start_pose2.position.z;
  // lcm.line_end.x = target_pose2.position.x;
  // lcm.line_end.y = target_pose2.position.y;
  // lcm.line_end.z = target_pose2.position.z;
  // lcm.tolerance = 0.01;
  // lcm.weight = 1.0;

  // moveit_msgs::OrientationConstraint ocm2;
  // ocm2.link_name = "panda_link8";
  // ocm2.header.frame_id = "panda_link0";
  // ocm2.orientation.w = start_pose2.orientation.w;
  // ocm2.orientation.x = start_pose2.orientation.x;
  // ocm2.orientation.y = start_pose2.orientation.y;
  // ocm2.orientation.z = start_pose2.orientation.z;
  // ocm2.absolute_x_axis_tolerance = 0.01;
  // ocm2.absolute_y_axis_tolerance = 0.01;
  // ocm2.absolute_z_axis_tolerance = 0.01;
  // ocm2.weight = 1.0;

  // // Now, set it as the path constraint for the group.
  // moveit_msgs::Constraints test_constraints2;
  // test_constraints2.line_constraints.push_back(lcm);
  // // test_constraints2.orientation_constraints.push_back(ocm2);
  // move_group.setPathConstraints(test_constraints2);

  // move_group.setStartStateToCurrentState();
  // move_group.setPoseTarget(target_pose2);

  // // Planning with constraints can be slow because every sample must call an inverse kinematics solver.
  // // Lets increase the planning time from the default 5 seconds to be sure the planner has enough time to succeed.
  // move_group.setPlanningTime(10.0);

  // ROS_INFO_STREAM("Starting position: (" 
  //   << start_pose2.position.x << ", "
  //   << start_pose2.position.y << ", "
  //   << start_pose2.position.z << ")");
  // ROS_INFO_STREAM("End position: (" 
  //   << target_pose2.position.x << ", "
  //   << target_pose2.position.y << ", "
  //   << target_pose2.position.z << ")");


  // success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  // ROS_INFO_NAMED("tutorial", "Visualizing plan 2 (line constraints) %s", success ? "" : "FAILED");

  // // Visualize the plan in RViz
  // visual_tools.deleteAllMarkers();
  // visual_tools.publishAxisLabeled(start_pose2, "start");
  // visual_tools.publishAxisLabeled(target_pose2, "goal");
  // visual_tools.publishText(text_pose, "Constrained Goal", rvt::WHITE, rvt::XLARGE);
  // visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
  // visual_tools.trigger();

  // // When done with the path constraint be sure to clear it.
  // move_group.clearPathConstraints();

  // // Since we set the start state we have to clear it before planning other paths
  // move_group.setStartStateToCurrentState();
>>>>>>> Stashed changes

  /*
   * Planning with orientation constraint
   */
  visual_tools.prompt("Press 'next' to plan with orientation constraints");
<<<<<<< Updated upstream
  geometry_msgs::Pose start_pose3 = move_group.getCurrentPose().pose;
=======

  // We will reuse the old goal that we had and plan to it.
  // Note that this will only work if the current state already
  // satisfies the path constraints. So, we need to set the start
  // state to a new pose.
  robot_state::RobotState start_state(*move_group.getCurrentState());
  geometry_msgs::Pose start_pose3;
  start_pose3.orientation.w = 0.0;
  start_pose3.orientation.x = 1.0;
  start_pose3.orientation.y = 0.0;
  start_pose3.orientation.z = 0.0;
  start_pose3.position.x = 0.6;
  start_pose3.position.y = -0.25;
  start_pose3.position.z = 0.5;
  start_state.setFromIK(joint_model_group, start_pose3);
  move_group.setStartState(start_state);


>>>>>>> Stashed changes
  moveit_msgs::OrientationConstraint ocm;
  ocm.link_name = "panda_link8";
  ocm.header.frame_id = "panda_link0";
  ocm.orientation.w = 0.0;
  ocm.orientation.x = 0.0;
  ocm.orientation.y = 1.0;
  ocm.orientation.z = 0.0;
  ocm.absolute_x_axis_tolerance = 0.1;
  ocm.absolute_y_axis_tolerance = 0.1;
  ocm.absolute_z_axis_tolerance = 3.14;
  ocm.weight = 1.0;

<<<<<<< Updated upstream
  geometry_msgs::Pose target_pose3 = start_pose3;
  target_pose3.position.x = start_pose2.position.x;
  target_pose3.position.y = start_pose2.position.y;
  target_pose3.position.z = start_pose2.position.z;
=======
  geometry_msgs::Pose target_pose3 = target_pose1;
  target_pose3.position.x = 0.25;
  target_pose3.position.y = 0.6;
  target_pose3.position.z = 0.5;
  target_pose3.orientation.w = 0.0;
  target_pose3.orientation.x = 1.0;
  target_pose3.orientation.y = 0.0;
  target_pose3.orientation.z = 0.0;
>>>>>>> Stashed changes


  moveit_msgs::LineConstraint lcm2;
  lcm2.link_name = "panda_link8";
  lcm2.header.frame_id = "panda_link0";
  lcm2.line_start.x = start_pose3.position.x;
  lcm2.line_start.y = start_pose3.position.y;
  lcm2.line_start.z = start_pose3.position.z;
  lcm2.line_end.x = target_pose3.position.x;
  lcm2.line_end.y = target_pose3.position.y;
  lcm2.line_end.z = target_pose3.position.z;
  lcm2.tolerance = 0.02;
  lcm2.weight = 1.0;

  // // Now, set it as the path constraint for the group.
  // moveit_msgs::Constraints test_constraints2;

  // Now, set it as the path constraint for the group.
  moveit_msgs::Constraints test_constraints;
  test_constraints.orientation_constraints.push_back(ocm);
  test_constraints.line_constraints.push_back(lcm2);
  move_group.setPathConstraints(test_constraints);

<<<<<<< Updated upstream
  // We will reuse the old goal that we had and plan to it.
  // Note that this will only work if the current state already
  // satisfies the path constraints. So, we need to set the start
  // state to a new pose.
  move_group.setStartStateToCurrentState();

=======
>>>>>>> Stashed changes
  // Now we will plan to the earlier pose target from the new
  // start state that we have just created.
  move_group.setPoseTarget(target_pose3);

  // Planning with constraints can be slow because every sample must call an inverse kinematics solver.
  // Lets increase the planning time from the default 5 seconds to be sure the planner has enough time to succeed.
  move_group.setPlanningTime(20.0);

  success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO_NAMED("tutorial", "Visualizing plan 3 (orientation constraints) %s", success ? "" : "FAILED");

  // Visualize the plan in RViz
  visual_tools.deleteAllMarkers();
  visual_tools.publishAxisLabeled(start_pose3, "start");
  visual_tools.publishAxisLabeled(target_pose3, "goal");
  visual_tools.publishText(text_pose, "Constrained Goal", rvt::WHITE, rvt::XLARGE);
  visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
  visual_tools.trigger();
  move_group.move();

  // When done with the path constraint be sure to clear it.
  move_group.clearPathConstraints();

  // Since we set the start state we have to clear it before planning other paths
  move_group.setStartStateToCurrentState();

  ros::shutdown();
  return 0;
}

