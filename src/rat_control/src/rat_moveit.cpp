#include <ros/ros.h>
#include <memory>
// MoveitCpp
#include <moveit/moveit_cpp/moveit_cpp.h>
#include <moveit/moveit_cpp/planning_component.h>
#include <moveit/robot_state/conversions.h>

#include <geometry_msgs/PointStamped.h>

#include <moveit_visual_tools/moveit_visual_tools.h>
namespace rvt = rviz_visual_tools;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "rat_moveit");
  ros::NodeHandle nh("/tat_moveit");
  ros::AsyncSpinner spinner(4);
  spinner.start();

  // BEGIN_TUTORIAL
  //
  // Setup
  // ^^^^^
  //
  static const std::string PLANNING_GROUP = "rat_arm";
  static const std::string LOGNAME = "rat_moveit";
 /* Otherwise robot with zeros joint_states */
  ros::Duration(1.0).sleep();

  ROS_INFO_STREAM_NAMED(LOGNAME, "Starting RAT...");

  auto moveit_cpp_ptr = std::make_shared<moveit_cpp::MoveItCpp>(nh);
  moveit_cpp_ptr->getPlanningSceneMonitor()->providePlanningSceneService();

  auto planning_components = std::make_shared<moveit_cpp::PlanningComponent>(PLANNING_GROUP, moveit_cpp_ptr);
  auto robot_model_ptr = moveit_cpp_ptr->getRobotModel();
  auto robot_start_state = planning_components->getStartState();
  auto joint_model_group_ptr = robot_model_ptr->getJointModelGroup(PLANNING_GROUP);
  moveit_visual_tools::MoveItVisualTools visual_tools("Base", rvt::RVIZ_MARKER_TOPIC,
                                                      moveit_cpp_ptr->getPlanningSceneMonitor());
  visual_tools.deleteAllMarkers();
  visual_tools.loadRemoteControl();

  Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
  text_pose.translation().z() = 1.75;
  visual_tools.publishText(text_pose, "RAT Demo", rvt::WHITE, rvt::XLARGE);
  visual_tools.trigger();

  // Start the demo
  // ^^^^^^^^^^^^^^^^^^^^^^^^^
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");
  
  // END_TUTORIAL
  visual_tools.deleteAllMarkers();
  visual_tools.prompt("Press 'next' to end the demo");

  ROS_INFO_STREAM_NAMED(LOGNAME, "Shutting down.");
  ros::waitForShutdown();
}