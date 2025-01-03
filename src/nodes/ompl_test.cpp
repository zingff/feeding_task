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




  // manipulation.set_target_pose();
  // manipulation.bite_transfer();
  // manipulation.bite_transfer();

  manipulation.ompl_plan(0.348976, -0.321229, 0.276205);
  ros::Duration(2.0).sleep();
  manipulation.ompl_plan(0.357675,  -0.0815239, 0.13);
  ros::Duration(2.0).sleep();
  manipulation.open_gripper();
  ros::Duration(2).sleep();
  manipulation.goTop();
  ros::shutdown();

  // ros::waitForShutdown();

  return 0;
}