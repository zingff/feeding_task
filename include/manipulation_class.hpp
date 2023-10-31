#ifndef MANIPULATION_CLASS_HPP
#define MANIPULATION_CLASS_HPP

#include <ros/ros.h>
#include <boost/filesystem.hpp>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit_msgs/PickupAction.h>

#include <std_msgs/Float32.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/PoseArray.h>

#include <tf/transform_listener.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <control_msgs/GripperCommandActionGoal.h>

#include <fstream>

#include <visualization_msgs/Marker.h>

typedef boost::shared_ptr<moveit::planning_interface::MoveGroupInterface> MoveGroupPtr;
typedef boost::shared_ptr<moveit::planning_interface::PlanningSceneInterface> PlanningScenePtr;

class Manipulation
{
private:
  ros::Subscriber grasp_config; 

  const double pi = std::acos(-1); 

  const std::string BASE_LINK_NAME  = "base_link";
  const std::string GRASP_LINK_NAME = "anygrasp/grasp_0";

  // Move arm functions:
  void setJointGroup(double j0, double j1, double j2,
                     double j3, double j4, double j5, double j6);

  void move(std::vector<double>);

  tf::TransformListener grasp_listener;


public:
  ros::Time begin;
  ros::Time end;

  ros::Publisher gripper_command;
  ros::Publisher grasps_visualization_pub_;

  Manipulation(ros::NodeHandle nodeHandle, std::string planning_group);

  moveit::core::RobotStatePtr current_state;
  std::vector<double> joint_group_positions;
  std::string PLANNING_GROUP;
  PlanningScenePtr planning_scene_ptr;
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  MoveGroupPtr move_group_ptr;
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  // todo: move to private
  void getCurrentState();

  // Hard coded arm goTo functions
  void goTop();
  void goDown();
  void goRight();
  void goLeft();
  void goVertical();
  void goWait();
  void goPlace();
  void goPostGrasp();
  void lastJointRotation(double rotate_angle);

  // Collision Objects
  void set_objects();
  void remove_objects();

  // Gripper commands, pick pipeline
  void closedGripper(trajectory_msgs::JointTrajectory &);
  void openGripper(trajectory_msgs::JointTrajectory &);

  // Gripper Action commands
  void close_gripper();
  void open_gripper();
  control_msgs::GripperCommandActionGoal gripper_cmd;

  void path_planning();
  void set_target_pose();
  void plan_pose_goal();
  void pick_and_place();
  void pickup();

  // For AnyGrasp
  void reach_anygrasp();
  void reach_handle();
  void reach_food_item();
  tf::StampedTransform getTransform(tf::TransformListener & listener, 
                                    std::string target_frame, 
                                    std::string source_frame);
  void goSnapshotPostion();
  void ompl_plan(double x, double y, double z);
  void bite_transfer();
  void get_utensil();
  void trajectory_replay(std::string waypoints_path);
  void goTemp(); // after get bowl handle

  bool getting_grasps = true; 
  bool pose_success;
  std_msgs::Float32 score;

  bool grabbed_object;  // Flags a good grasping plan
  
  // Planning variables
  geometry_msgs::Pose target_pose;
  geometry_msgs::Vector3 orientation;
  geometry_msgs::Vector3 axis;
  geometry_msgs::Point pose;
  geometry_msgs::Point position;
  geometry_msgs::Point pose_sample;
  geometry_msgs::Vector3 grasp_orientation;
  moveit_msgs::CollisionObject collision_object;
  tf2::Quaternion q;
};

#endif