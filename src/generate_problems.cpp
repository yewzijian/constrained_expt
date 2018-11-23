#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

#include <vector>
#include <geometry_msgs/PoseStamped.h>
#include <fstream>

using namespace std;

#define NUM_PROBLEMS 20

int main(int argc, char** argv)
{
  ros::init(argc, argv, "move_group_interface_tutorial");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  static const std::string PLANNING_GROUP = "panda_arm";
  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
  move_group.setPlanningTime(10.0);
  moveit::planning_interface::MoveGroupInterface::Plan computedPlan;

  ofstream outFile("problems1.txt");
  for (int iProb=0; iProb<NUM_PROBLEMS && ros::ok(); iProb++) {

    bool success = false;
    vector<double> randomJoints;
    while (!success) {
      // Generate random joint values as starting position
      randomJoints = move_group.getRandomJointValues();

      // Ensure starting state is reachable at least by planning
      move_group.setStartStateToCurrentState();
      move_group.setJointValueTarget(randomJoints);
      success = (move_group.plan(computedPlan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    }

    // Generate 
    geometry_msgs::PoseStamped randomPoseMsg = move_group.getRandomPose();

    if (iProb == 0) {
      for (size_t i=0; i<randomJoints.size(); i++) {
        outFile << "start_joint_" << i << "\t";
      }
      outFile 
        << "goal_X" << "\t"
        << "goal_Y" << "\t"
        << "goal_Z" << "\t"
        << "goal_q_w" << "\t"
        << "goal_q_x" << "\t"
        << "goal_q_y" << "\t"
        << "goal_q_z" << "\r\n";
    }

    for (size_t i=0; i<randomJoints.size(); i++ ) {
      outFile << randomJoints[i] << "\t";
    }
    outFile
      << randomPoseMsg.pose.position.x << "\t"
      << randomPoseMsg.pose.position.y << "\t"
      << randomPoseMsg.pose.position.z << "\t"
      << randomPoseMsg.pose.orientation.w << "\t"
      << randomPoseMsg.pose.orientation.x << "\t"
      << randomPoseMsg.pose.orientation.y << "\t"
      << randomPoseMsg.pose.orientation.z;

    outFile << "\r\n";

    cout << "Generated problem" << iProb << endl;
  }

  ros::shutdown();
  return 0;
}