#include <moveit/move_group_interface/move_group_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

#include <iostream>
#include <fstream>
#include <vector>
#include <sstream>
#include <string>

#include <chrono>
#include <thread>

#define NUM_JOINTS 7

using namespace std;

struct Problem{
  vector<double> start_joints;
  vector<double> target_pose_position;
  vector<double> target_pose_orientation;
};

vector<double> tokenize(const string & str) {
  vector<double> vals;
  stringstream ss(str);

  double v;
  while (ss >> v)
    vals.push_back(v);   

  return vals;
}

vector<Problem> loadProblems(const string & path) {
  
  cout << "Loading problems from " << path << "..." << endl;
  vector<Problem> problems;
  ifstream fid(path);
  string line;

  getline(fid, line);
  while (fid.good()) {
    getline(fid, line);
    vector<double> vals = tokenize(line);
    if (vals.size() <= 1)
      break;

    Problem p;

    // Start joint positions
    p.start_joints.resize(NUM_JOINTS);
    for (int i=0; i<NUM_JOINTS; i++) {
      p.start_joints[i] = vals[i];
    }

    // Target positions
    p.target_pose_position.resize(3);
    for (int i=0; i<p.target_pose_position.size(); i++)
      p.target_pose_position[i] = vals[NUM_JOINTS+i];

    // Target orientations
    p.target_pose_orientation.resize(4);
    for (int i=0; i<p.target_pose_orientation.size(); i++)
      p.target_pose_orientation[i] = vals[NUM_JOINTS+p.target_pose_position.size()+i];

    problems.push_back(p);
  }

  return problems;
}

string vec2str(vector<double> vec) {
  stringstream ss;
  
  if (vec.empty())
    return "";

  ss << vec[0];
  for (int i=1; i<vec.size(); i++) {
    ss << ", " << vec[i];
  }

  return ss.str();
}

void runUnconstrainedExpt(const vector<string> & planners, const string & problemsPath,
                          moveit::planning_interface::MoveGroupInterface & move_group,
                          moveit_visual_tools::MoveItVisualTools & visual_tools,
                          const robot_state::JointModelGroup* joint_model_group)
{
  vector<Problem> problems = loadProblems("problems1.txt");
  for (string plannerStr : planners) {

    // Create file
    ofstream fid_out((string)"result_unconstrained_" + plannerStr + ".txt");
    ROS_INFO_STREAM("------------------------------------------------");
    ROS_INFO_STREAM("Unconstrained, Algo: " << plannerStr);
    ROS_INFO_STREAM("------------------------------------------------");

    move_group.setPlannerId(plannerStr);
    move_group.setPlanningTime(10.0);

    for (size_t iProb=0; iProb<problems.size() && ros::ok(); iProb++) {

      Problem p = problems[iProb];
      geometry_msgs::Pose start_pose, target_pose;
      
      // default values
      moveit::planning_interface::MoveGroupInterface::Plan computedPlan;
      computedPlan.planning_time_ = -1;
      double execTime_s = -1;

      // Move to starting position
      cout << endl;
      ROS_INFO_STREAM("Problem " << iProb << ", moving to joint positions:");

      move_group.setJointValueTarget(p.start_joints);
      bool success = (move_group.move() == moveit::planning_interface::MoveItErrorCode::SUCCESS);

      if (success) {
        ROS_INFO_STREAM("Moved to starting position");
      } else {
        ROS_WARN_STREAM("Failed to move to starting position");
      }
      
      // Plan
      if (success) {

        start_pose = move_group.getCurrentPose().pose;

        std::this_thread::sleep_for(std::chrono::milliseconds(1000));

        target_pose.orientation.w = p.target_pose_orientation[0];
        target_pose.orientation.x = p.target_pose_orientation[1];
        target_pose.orientation.y = p.target_pose_orientation[2];
        target_pose.orientation.z = p.target_pose_orientation[3];
        target_pose.position.x = p.target_pose_position[0];
        target_pose.position.y = p.target_pose_position[1];
        target_pose.position.z = p.target_pose_position[2];
        move_group.setPoseTarget(target_pose);

        visual_tools.deleteAllMarkers();
        visual_tools.publishAxisLabeled(start_pose, "start");
        visual_tools.publishAxisLabeled(target_pose, "goal");
        visual_tools.trigger();

        // Plan
        success = (move_group.plan(computedPlan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        ROS_INFO_STREAM("Planning result: " << ((success)? "SUCCESS" : "FAIL") 
                        << ", time taken: " << computedPlan.planning_time_);
      }

      // Visualize plan and execute
      if (success) {
        // Visualize the plan in RViz
        visual_tools.publishTrajectoryLine(computedPlan.trajectory_, joint_model_group);
        visual_tools.trigger();

        // Execute
        ros::Time beginExecTime = ros::Time::now();
        success = (move_group.execute(computedPlan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        ros::Duration execTime = ros::Time::now() - beginExecTime;
        execTime_s = execTime.toSec();
        ROS_INFO_STREAM("Execution result: " << ((success)? "SUCCESS" : "FAIL") << ", time taken: " << execTime_s);
      }

      fid_out
        << iProb
        << "\t" << computedPlan.planning_time_
        << "\t" << execTime_s << endl;

      std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }  
  }
}

void runOriConstrainedExpt(const vector<string> & planners, const string & problemsPath,
                          moveit::planning_interface::MoveGroupInterface & move_group,
                          moveit_visual_tools::MoveItVisualTools & visual_tools,
                          const robot_state::JointModelGroup* joint_model_group)
{
  vector<Problem> problems = loadProblems("problems1.txt");
  for (string plannerStr : planners) {

    // Create file
    ofstream fid_out((string)"result_oriconstrained_" + plannerStr + ".txt");
    ROS_INFO_STREAM("------------------------------------------------");
    ROS_INFO_STREAM("Orientation constrained, Algo: " << plannerStr);
    ROS_INFO_STREAM("------------------------------------------------");

    move_group.setPlannerId(plannerStr);
    move_group.setPlanningTime(20.0);

    for (size_t iProb=0; iProb<problems.size() && ros::ok(); iProb++) {

      Problem p = problems[iProb];
      geometry_msgs::Pose start_pose, target_pose;
      
      // default values
      moveit::planning_interface::MoveGroupInterface::Plan computedPlan;
      computedPlan.planning_time_ = -1;
      double execTime_s = -1;

      // Move to starting position
      cout << endl;
      ROS_INFO_STREAM("Problem " << iProb << ", moving to joint positions:");

      move_group.setJointValueTarget(p.start_joints);
      bool success = (move_group.move() == moveit::planning_interface::MoveItErrorCode::SUCCESS);

      if (success) {
        ROS_INFO_STREAM("Moved to starting position");
      } else {
        ROS_WARN_STREAM("Failed to move to starting position");
      }

      // Plan
      if (success) {

        start_pose = move_group.getCurrentPose().pose;

        std::this_thread::sleep_for(std::chrono::milliseconds(1000));

        target_pose.orientation.w = start_pose.orientation.w;
        target_pose.orientation.x = start_pose.orientation.x;
        target_pose.orientation.y = start_pose.orientation.y;
        target_pose.orientation.z = start_pose.orientation.z;
        target_pose.position.x = p.target_pose_position[0];
        target_pose.position.y = p.target_pose_position[1];
        target_pose.position.z = p.target_pose_position[2];
        move_group.setPoseTarget(target_pose);

        // Set constraints
        moveit_msgs::OrientationConstraint ocm;
        ocm.link_name = "panda_link8";
        ocm.header.frame_id = "panda_link0";
        ocm.orientation.w = start_pose.orientation.w;
        ocm.orientation.x = start_pose.orientation.x;
        ocm.orientation.y = start_pose.orientation.y;
        ocm.orientation.z = start_pose.orientation.z;
        ocm.absolute_x_axis_tolerance = 0.1;
        ocm.absolute_y_axis_tolerance = 0.1;
        ocm.absolute_z_axis_tolerance = 0.1;
        ocm.weight = 1.0;

        // Now, set it as the path constraint for the group.
        moveit_msgs::Constraints test_constraints;
        test_constraints.orientation_constraints.push_back(ocm);
        move_group.setPathConstraints(test_constraints);

        // Plan
        visual_tools.deleteAllMarkers();
        visual_tools.publishAxisLabeled(start_pose, "start");
        visual_tools.publishAxisLabeled(target_pose, "goal");
        visual_tools.trigger();
        success = (move_group.plan(computedPlan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        ROS_INFO_STREAM("Planning result: " << ((success)? "SUCCESS" : "FAIL") 
                        << ", time taken: " << computedPlan.planning_time_);
      }

      // Visualize plan and execute
      if (success) {
        // Visualize the plan in RViz
        visual_tools.publishTrajectoryLine(computedPlan.trajectory_, joint_model_group);
        visual_tools.trigger();

        // Execute
        ros::Time beginExecTime = ros::Time::now();
        success = (move_group.execute(computedPlan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        ros::Duration execTime = ros::Time::now() - beginExecTime;
        execTime_s = execTime.toSec();
        ROS_INFO_STREAM("Execution result: " << ((success)? "SUCCESS" : "FAIL") << ", time taken: " << execTime_s);
      }

      move_group.clearPathConstraints();
      fid_out
        << iProb
        << "\t" << computedPlan.planning_time_
        << "\t" << execTime_s << endl;

      std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    } 
  }
}

void runLineConstrainedExpt(const vector<string> & planners, const string & problemsPath,
                          moveit::planning_interface::MoveGroupInterface & move_group,
                          moveit_visual_tools::MoveItVisualTools & visual_tools,
                          const robot_state::JointModelGroup* joint_model_group)
{
  vector<Problem> problems = loadProblems("problems1.txt");
  for (string plannerStr : planners) {

    // Create file
    ofstream fid_out((string)"result_lineconstrained_" + plannerStr + ".txt");
    ROS_INFO_STREAM("------------------------------------------------");
    ROS_INFO_STREAM("Line constrained, Algo: " << plannerStr);
    ROS_INFO_STREAM("------------------------------------------------");

    move_group.setPlannerId(plannerStr);
    move_group.setPlanningTime(20.0);

    for (size_t iProb=0; iProb<problems.size() && ros::ok(); iProb++) {

      Problem p = problems[iProb];
      geometry_msgs::Pose start_pose, target_pose;
      
      // default values
      moveit::planning_interface::MoveGroupInterface::Plan computedPlan;
      computedPlan.planning_time_ = -1;
      double execTime_s = -1;

      // Move to starting position
      cout << endl;
      ROS_INFO_STREAM("Problem " << iProb << ", moving to joint positions:");

      move_group.setJointValueTarget(p.start_joints);
      bool success = (move_group.move() == moveit::planning_interface::MoveItErrorCode::SUCCESS);

      if (success) {
        ROS_INFO_STREAM("Moved to starting position");
      } else {
        ROS_WARN_STREAM("Failed to move to starting position");
      }

      // Plan
      if (success) {

        start_pose = move_group.getCurrentPose().pose;

        std::this_thread::sleep_for(std::chrono::milliseconds(1000));

        move_group.setPositionTarget(p.target_pose_position[0], 
                                     p.target_pose_position[1], 
                                     p.target_pose_position[2]);
        target_pose.orientation.w = start_pose.orientation.w; // only for visualization
        target_pose.orientation.x = start_pose.orientation.x;
        target_pose.orientation.y = start_pose.orientation.y;
        target_pose.orientation.z = start_pose.orientation.z;
        target_pose.position.x = p.target_pose_position[0];
        target_pose.position.y = p.target_pose_position[1];
        target_pose.position.z = p.target_pose_position[2];
        // move_group.setPoseTarget(target_pose);

        // Set constraints
        moveit_msgs::LineConstraint lcm;
        lcm.link_name = "panda_link8";
        lcm.header.frame_id = "panda_link0";
        lcm.line_start.x = start_pose.position.x;
        lcm.line_start.y = start_pose.position.y;
        lcm.line_start.z = start_pose.position.z;
        lcm.line_end.x = target_pose.position.x;
        lcm.line_end.y = target_pose.position.y;
        lcm.line_end.z = target_pose.position.z;
        lcm.tolerance = 0.02;
        lcm.weight = 1.0;
        moveit_msgs::OrientationConstraint ocm;
        ocm.link_name = "panda_link8";
        ocm.header.frame_id = "panda_link0";
        ocm.orientation.w = start_pose.orientation.w;
        ocm.orientation.x = start_pose.orientation.x;
        ocm.orientation.y = start_pose.orientation.y;
        ocm.orientation.z = start_pose.orientation.z;
        ocm.absolute_x_axis_tolerance = 0.1;
        ocm.absolute_y_axis_tolerance = 0.1;
        ocm.absolute_z_axis_tolerance = 0.1;
        ocm.weight = 1.0;

        // Now, set it as the path constraint for the group.
        moveit_msgs::Constraints test_constraints;
        test_constraints.line_constraints.push_back(lcm);
        test_constraints.orientation_constraints.push_back(ocm);
        move_group.setPathConstraints(test_constraints);

        // Plan
        visual_tools.deleteAllMarkers();
        visual_tools.publishAxisLabeled(start_pose, "start");
        visual_tools.publishAxisLabeled(target_pose, "goal");
        visual_tools.trigger();
        success = (move_group.plan(computedPlan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        ROS_INFO_STREAM("Planning result: " << ((success)? "SUCCESS" : "FAIL") 
                        << ", time taken: " << computedPlan.planning_time_);
      }

      // Visualize plan and execute
      if (success) {
        // Visualize the plan in RViz
        visual_tools.publishTrajectoryLine(computedPlan.trajectory_, joint_model_group);
        visual_tools.trigger();

        // Execute
        ros::Time beginExecTime = ros::Time::now();
        success = (move_group.execute(computedPlan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        ros::Duration execTime = ros::Time::now() - beginExecTime;
        execTime_s = execTime.toSec();
        ROS_INFO_STREAM("Execution result: " << ((success)? "SUCCESS" : "FAIL") << ", time taken: " << execTime_s);
      }

      move_group.clearPathConstraints();
      fid_out
        << iProb
        << "\t" << computedPlan.planning_time_
        << "\t" << execTime_s << endl;

      std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }  
  }
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "move_group_interface_tutorial");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  static const std::string PLANNING_GROUP = "panda_arm";
  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
  moveit_visual_tools::MoveItVisualTools visual_tools("panda_link0");
  const robot_state::JointModelGroup* joint_model_group =
    move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

  /*
   * Set planners and parameters here
   */
  vector<string> planners;
  // planners.push_back("RRTstarkConfigDefault");
  planners.push_back("RRTConnectkConfigDefault");
  // planners.push_back("ESTkConfigDefault");

  for (string plannerStr : planners) {
    std::map<std::string, std::string> plannerParams = move_group.getPlannerParams(plannerStr, PLANNING_GROUP);
    plannerParams["range"] = "0.001";
    move_group.setPlannerParams(plannerStr, PLANNING_GROUP, plannerParams, true);
  }

  /*
   * Run experiment
   */
  // runUnconstrainedExpt(planners, "problems1.txt",
  //                      move_group, visual_tools, joint_model_group);
  // runOriConstrainedExpt(planners, "problems1.txt",
  //                      move_group, visual_tools, joint_model_group);
  runLineConstrainedExpt(planners, "problems1.txt",
                       move_group, visual_tools, joint_model_group);


  ros::shutdown();
  return 0;
}
