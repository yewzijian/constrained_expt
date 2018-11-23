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

  vector<string> planners;
  // planners.push_back("RRTstarkConfigDefault");
  planners.push_back("RRTConnectkConfigDefault");
  // planners.push_back("ESTkConfigDefault");

  /*
   * Unconstrained expt
   */
  vector<Problem> problems = loadProblems("problems_unconstrained.txt");
  for (string plannerStr : planners) {

    // Create file
    ofstream fid_out((string)"result_unconstrained_" + plannerStr + ".txt");

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
      ROS_INFO_STREAM("Problem " << iProb << ", moving to joint positions:");
      ROS_INFO_STREAM(" - " << vec2str(p.start_joints));

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
        ROS_INFO_STREAM("Target pose:");
        ROS_INFO_STREAM(" - pos: " << vec2str(p.target_pose_position));
        ROS_INFO_STREAM(" - ori: " << vec2str(p.target_pose_orientation));

        target_pose.orientation.w = p.target_pose_orientation[0];
        target_pose.orientation.x = p.target_pose_orientation[1];
        target_pose.orientation.y = p.target_pose_orientation[2];
        target_pose.orientation.z = p.target_pose_orientation[3];
        target_pose.position.x = p.target_pose_position[0];
        target_pose.position.y = p.target_pose_position[1];
        target_pose.position.z = p.target_pose_position[2];
        move_group.setPoseTarget(target_pose);

        // Plan
        success = (move_group.plan(computedPlan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        ROS_INFO_STREAM("Planning result: " << ((success)? "SUCCESS" : "FAIL") 
                        << ", time taken: " << computedPlan.planning_time_);
      }

      // Visualize plan and execute
      if (success) {
        // Visualize the plan in RViz
        visual_tools.deleteAllMarkers();
        visual_tools.publishAxisLabeled(start_pose, "start");
        visual_tools.publishAxisLabeled(target_pose, "goal");
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

  ros::shutdown();
  return 0;
}
