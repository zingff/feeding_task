#include <manipulation_class.hpp>
#include <tf/transform_listener.h>

void Manipulation::set_target_pose()
{
    this->q.setRPY(this->orientation.x, this->orientation.y, this->orientation.z); // x-pi
    // std::cout << this->orientation.x << std::endl;
    // std::cout << this->orientation.y << std::endl;
    // std::cout << this->orientation.z << std::endl;
    // std::cout << this->q.getZ() << std::endl;
    // std::cout << "target_orientation: " << this->orientation.x*57.3 << ", "<< this->orientation.y*57.3 << ", " << this->orientation.z*57.3 <<std::endl;
    // std::cout << "target_position: " << this->pose_sample.x << ", " << this->pose_sample.y << ", "  << this->pose_sample.z << std::endl;
    this->q.normalize();
    this->target_pose.orientation = tf2::toMsg(this->q);
    // std::cout << "target_orientation_again" << std::endl;
    // std::cout << this->target_pose.orientation.w << std::endl;
    // std::cout << this->target_pose.orientation.x << std::endl;
    // std::cout << this->target_pose.orientation.y << std::endl;
    // std::cout << this->target_pose.orientation.z << std::endl;
    // this->target_pose.position.x = this->pose_sample.x;
    // this->target_pose.position.y = this->pose_sample.y;
    // this->target_pose.position.z = this->pose_sample.z;
    this->target_pose.position.x = this->position.x;
    this->target_pose.position.y = this->position.y;
    this->target_pose.position.z = this->position.z;
}

void Manipulation::plan_pose_goal()
{
    this->move_group_ptr->setPoseTarget(this->target_pose);
    this->move_group_ptr->setGoalPositionTolerance(0.01);
    this->move_group_ptr->setGoalOrientationTolerance(0.002);
    this->move_group_ptr->setPlanningTime(5.0);
    this->move_group_ptr->setNumPlanningAttempts(30);
    this->pose_success = (this->move_group_ptr->plan(this->my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
}

// TODO: change the ompl name 
// TODO: change to upright way
// TODO: if planning not successful, shutdown
void Manipulation::ompl_plan(double x, double y, double z)
{
  auto current_pose = this->move_group_ptr->getCurrentPose();
  std::cout << "current quat:" << std::endl;
  std::cout << current_pose.pose.orientation << std::endl;
  this->target_pose.orientation = current_pose.pose.orientation;
  // this->target_pose.position = current_pose.pose.position;
  // this->target_pose.position.x = 0;
  // this->target_pose.position.y = -0.5;
  // this->target_pose.position.z = 0.3;
  // this->target_pose.orientation.x = 0.481745;
  // this->target_pose.orientation.y = -0.587967;
  // this->target_pose.orientation.z = 0.439422;
  // this->target_pose.orientation.w = 0.439422;
  // this->target_pose.position.x = 0.360;
  // this->target_pose.position.y = -0.146 + 0.0;
  // this->target_pose.position.z = 0.13;
  // this->target_pose.position.x = 0.357675;
  // this->target_pose.position.y = -0.0315239;
  // this->target_pose.position.z = 0.13;
  this->target_pose.position.x = x;
  this->target_pose.position.y = y;
  this->target_pose.position.z = z;
  // this->target_pose.position.x = 0.247481;
  // this->target_pose.position.y = -0.441109;
  // this->target_pose.position.z = 0.279088;
  this->move_group_ptr->setPoseTarget(this->target_pose);
  this->move_group_ptr->setPlanningTime(20.0);
  this->move_group_ptr->setMaxVelocityScalingFactor(0.5);
  this->move_group_ptr->setMaxAccelerationScalingFactor(0.5);
  // this->move_group_ptr->setPlannerId("RRTConnect");

  moveit_msgs::OrientationConstraint orientation_constraint;
  orientation_constraint.header.frame_id = this->move_group_ptr->getPoseReferenceFrame();
  orientation_constraint.link_name = this->move_group_ptr->getEndEffectorLink();
  // orientation_constraint.link_name = "end_effector_link";

  // current_pose = this->move_group_ptr->getCurrentPose();
  orientation_constraint.orientation = this->target_pose.orientation;
  // orientation_constraint.orientation.x = -0.566237;
  // orientation_constraint.orientation.y = -0.568729;
  // orientation_constraint.orientation.z = -0.454364;
  // orientation_constraint.orientation.w = 0.386622;
  orientation_constraint.absolute_x_axis_tolerance = 0.5;
  orientation_constraint.absolute_y_axis_tolerance = 0.5;
  orientation_constraint.absolute_z_axis_tolerance = 0.5;
  orientation_constraint.weight = 1.0;
  // moveit_msgs::Constraints orientation_constraints;
  // orientation_constraints.orientation_constraints.emplace_back(orientation_constraint);
  // this->move_group_ptr->setPathConstraints(orientation_constraints);

  this->pose_success = (this->move_group_ptr->plan(this->my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  std::cout << "Planning state: " << this->pose_success << std::endl;
  if (this->pose_success == false)
  {
    ros::shutdown();
  }
  this->move_group_ptr->move();
}

void Manipulation::bite_transfer()
{
  auto current_pose = this->move_group_ptr->getCurrentPose();
  std::cout << current_pose.pose.orientation << std::endl;
  // this->target_pose.orientation = current_pose.pose.orientation;
  // this->target_pose.position = current_pose.pose.position;

  // this->target_pose.position.x = 0.385684;
  // this->target_pose.position.y = 0.00854371;
  // this->target_pose.position.z = 0.246238-0.1;

  // this->target_pose.orientation.x = -0.42135;
  // this->target_pose.orientation.y = -0.386108;
  // this->target_pose.orientation.z = 0.474233;
  // this->target_pose.orientation.w = 0.669692;

  // this->target_pose.position.x = -0.0171463;
  // this->target_pose.position.y = 0.255903;
  // this->target_pose.position.z = 0.590837;

  this->target_pose.orientation.x = 0.384618;
  this->target_pose.orientation.y = -0.496789;
  this->target_pose.orientation.z = -0.604751;
  this->target_pose.orientation.w = 0.489433;

  this->target_pose.position.x = -0.0155496;
  this->target_pose.position.y = 0.265189;
  this->target_pose.position.z = 0.553932;
  
  this->move_group_ptr->setPoseTarget(this->target_pose);
  this->move_group_ptr->setPlanningTime(10.0);
  this->move_group_ptr->setMaxVelocityScalingFactor(0.8);
  // this->move_group_ptr->setPlannerId("RRT");

  moveit_msgs::OrientationConstraint orientation_constraint;
  orientation_constraint.header.frame_id = this->move_group_ptr->getPoseReferenceFrame();
  orientation_constraint.link_name = this->move_group_ptr->getEndEffectorLink();

  // orientation_constraint.orientation = current_pose.pose.orientation;

  // orientation_constraint.orientation.x = 0.384618;
  // orientation_constraint.orientation.y = -0.496789;
  // orientation_constraint.orientation.z = -0.604751;
  // orientation_constraint.orientation.w = 0.489433;

  // orientation_constraint.absolute_x_axis_tolerance = 0.4;
  // orientation_constraint.absolute_y_axis_tolerance = 0.4;
  // orientation_constraint.absolute_z_axis_tolerance = 0.5;
  // orientation_constraint.weight = 1.0;
  // moveit_msgs::Constraints orientation_constraints;
  // orientation_constraints.orientation_constraints.emplace_back(orientation_constraint);
  // this->move_group_ptr->setPathConstraints(orientation_constraints);

  this->pose_success = (this->move_group_ptr->plan(this->my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  this->move_group_ptr->move();
  trajectory_saving();
  // this->my_plan.trajectory_.joint_trajectory.joint_names

  ros::Duration(0.5).sleep();
  goTop();
}

// TODO: Add exception when a plan fails
//     this->target_pose.orientation = poses[0].orientation;
//     this->target_pose.position.x = poses[0].position.x;
//     this->target_pose.position.y = poses[0].position.y;
//     this->target_pose.position.z = poses[0].position.z;
//     std::cout << "orientation in gpd::pickup " << std::endl;
//     std::cout << poses[0].orientation << std::endl;
//     plan_pose_goal();
//     this->move_group_ptr->move();
//     ros::Duration(1.0).sleep();

//     // lastJointRotation(0.5*pi);

//     ROS_WARN_STREAM("Picking...");
//     this->target_pose.orientation = poses[0].orientation;
//     this->target_pose.position.x = poses[1].position.x;
//     this->target_pose.position.y = poses[1].position.y;
//     this->target_pose.position.z = poses[1].position.z;
//     plan_pose_goal();
//     this->move_group_ptr->move();

//     this->end = ros::Time::now();    // Get times, for testing

//     close_gripper();

//     ROS_WARN_STREAM("Retreating...");
//     this->target_pose.orientation = poses[0].orientation;
//     this->target_pose.position.x = poses[0].position.x;
//     this->target_pose.position.y = poses[0].position.y;
//     this->target_pose.position.z = poses[0].position.z;
//     plan_pose_goal();
//     this->move_group_ptr->move();

// }

// void Manipulation::place(float z_dist)
// {
//     ROS_WARN_STREAM("Placing... ");

//     // NOTE: All hard coded values, just places the object to the right of the arm base
//     this->q.setRPY(-pi, 0, 0);
//     this->target_pose.orientation = tf2::toMsg(this->q);
//     this->target_pose.position.x = 0;
//     this->target_pose.position.y = -0.28;
//     this->target_pose.position.z = z_dist;
//     plan_pose_goal();
//     this->move_group_ptr->move();
// /*
//     this->q.setRPY(-pi, 0, 0);
//     this->target_pose.orientation = tf2::toMsg(this->q);
//     this->target_pose.position.x = 0;
//     this->target_pose.position.y = -0.28;
//     this->target_pose.position.z = z_dist - 0.15;
//     plan_pose_goal();
//     this->move_group_ptr->move();
// */
//     open_gripper();

// /*
//     this->q.setRPY(-pi, 0, 0);
//     this->target_pose.orientation = tf2::toMsg(this->q);
//     this->target_pose.position.x = 0;
//     this->target_pose.position.y = -0.28;
//     this->target_pose.position.z = z_dist;
//     plan_pose_goal();
//     this->move_group_ptr->move();
//     */
// }

//     std::cout << "x: " <<tf_grasp_base.getRotation().getX() << std::endl;
//     std::cout << "y: " <<tf_grasp_base.getRotation().getY() << std::endl;
//     std::cout << "z: " <<tf_grasp_base.getRotation().getZ() << std::endl;
//     std::cout << "w: " <<tf_grasp_base.getRotation().getW() << std::endl;
//     // Useless transform
//     try
//     {
//         listener_.waitForTransform("base_link", "base_link", ros::Time(0), ros::Duration(3.0));
//         listener_.lookupTransform("base_link", "base_link", ros::Time(0), tf_base_odom);
//     }
//     catch (tf::TransformException ex)
//     {
//         ROS_ERROR("%s", ex.what());
//     }

//     // Define the offset from end-effector to gripper tip (the actual grasp point)
//     tf::Quaternion q_tip_grasp;
//     q_tip_grasp.setRPY(0, 0, -pi/2);
//     tf::Vector3 translation_tip_grasp(0, 0, -0.0);
//     tf::Transform tf_tip_grasp(q_tip_grasp, translation_tip_grasp);

//     // Find grasp pose
//   // Fetch specific rotaion values
//     tf::Transform tf_grasp_odom_(tf::Quaternion(0, 0, 0.0, 0.0), tf::Vector3(0, 0, -0.03));
//   // tf::Transform tf_grasp_odom = tf_base_odom * tf_grasp_base * tf_grasp_odom_;
//     tf::Transform tf_grasp_odom = tf_base_odom * tf_grasp_base * tf_tip_grasp;
//     tf::poseTFToMsg(tf_grasp_odom, pose);  // convert grasp from tf::Pose to geometry_msgs::Pose form

//     // TODO: publish the grasp in rviz
//     // TODO: to check if ros nodehandle works

//     // Find pre-grasp pose
//     tf::Transform tf_pregrasp_odom_(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0, 0, -0.15));  // change the pregrasp height
//     tf::Transform tf_pregrasp_odom = tf_grasp_odom * tf_pregrasp_odom_;
//     tf::poseTFToMsg(tf_pregrasp_odom, pre);

//     tf::StampedTransform tf_hand_odom;
//     try
//     {
//         listener_.waitForTransform("base_link", "end_effector_link", ros::Time(0), ros::Duration(3.0));
//         listener_.lookupTransform("base_link", "end_effector_link", ros::Time(0), tf_hand_odom);
//     }
//     catch (tf::TransformException ex)
//     {
//         ROS_ERROR("%s", ex.what());
//     }
//     Eigen::VectorXf hand_position(2);

//     hand_position << tf_hand_odom.getOrigin().getX(), tf_hand_odom.getOrigin().getY();
//     float distance = (grasp_position - hand_position).squaredNorm();

//     return poses;
// }