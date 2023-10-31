#ifndef ANYGRASP_PUBLISHER_H
#define ANYGRASP_PUBLISHER_H

#include "ros/ros.h"
#include "tf2_ros/transform_broadcaster.h"
#include <tf2_ros/buffer.h>
// #include "anygrasp_generation/AnyGraspGeneration.h"
#include <anygrasp_generation/AnyGraspGeneration.h>
#include "geometry_msgs/TransformStamped.h"
#include "tf2/LinearMath/Quaternion.h"

class AnyGraspPublisher
{
public:
    AnyGraspPublisher(ros::NodeHandle& nh);
    void publishAnyGrasp();

private:
    ros::NodeHandle nh_;
    ros::ServiceClient anygrasp_publisher_;
};

#endif // ANYGRASP_PUBLISHER_H
