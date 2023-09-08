/*****************************************************************
  Main Node for Gen3 GPD grasping
*****************************************************************/

#include "manipulation_class.hpp"
#include <anygrasp_generation/AnyGraspGeneration.h>
#include <tf2_ros/transform_broadcaster.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "anygrasp_multiple_objects");

  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(4);
  spinner.start();
  // ros::Duration(1.0).sleep();

  std::string planning_group = "arm"; // Gen3 planning group
  Manipulation manipulation(nh, planning_group);


  // Manipulation variables/setup
  manipulation.planning_scene_ptr = PlanningScenePtr(
      new moveit::planning_interface::PlanningSceneInterface());
  manipulation.move_group_ptr = MoveGroupPtr(
      new moveit::planning_interface::MoveGroupInterface(manipulation.PLANNING_GROUP));

  manipulation.move_group_ptr->setPlanningTime(5.0);
  manipulation.move_group_ptr->setMaxVelocityScalingFactor(0.8); // speed scaling
  manipulation.move_group_ptr->setPlannerId("RRT");

  // manipulation.place(0.15);
  manipulation.goTop();
  manipulation.open_gripper();
  
  manipulation.reach_anygrasp();
  ros::Duration(1).sleep();
  manipulation.goTop();
  manipulation.lastJointRotation(M_PI/3);
  ros::Duration(1).sleep();  
  manipulation.goDown();
  ros::Duration(1).sleep();  
  manipulation.open_gripper();
  manipulation.goTop();
  ros::shutdown();

  // ros::waitForShutdown();

  return 0;
}