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




  manipulation.goTop();
  ros::Duration(0.5).sleep();
  ros::shutdown();

  // ros::waitForShutdown();

  return 0;
}