#include "ros/ros.h"
#include "tf2_ros/transform_broadcaster.h"
#include "anygrasp_generation/AnyGraspGeneration.h"
#include "geometry_msgs/TransformStamped.h"
#include "tf2/LinearMath/Quaternion.h"

class AnyGraspPublisher
{
public:
    AnyGraspPublisher(ros::NodeHandle& nh) : nh_(nh)
    {
        anygrasp_publisher_ = nh_.serviceClient<anygrasp_generation::AnyGraspGeneration>("/grasp_generator");
    }

    void publishAnyGrasp()
    {
        anygrasp_generation::AnyGraspGeneration anygrasp_request;
        anygrasp_request.request.update_anygrasp = true;

        if (anygrasp_publisher_.call(anygrasp_request))
        {
            if (anygrasp_request.response.success)
            {
                ROS_INFO("Succeeded to call anygrasp!");

                tf2_ros::TransformBroadcaster anygrasp_tf_broadcaster;
                std::string grasp_namespace = "anygrasp";
                ros::Rate publish_rate(10);

                while (ros::ok())
                {
                    for (size_t i = 0; i < anygrasp_request.response.anygrasp_transforms.size(); ++i)
                    {
                        geometry_msgs::TransformStamped anygrasp_tf_msg;
                        anygrasp_tf_msg.header.stamp = ros::Time::now();
                        anygrasp_tf_msg.header.frame_id = "base_link";
                        anygrasp_tf_msg.child_frame_id = grasp_namespace + "/grasp_" + std::to_string(i);

                        anygrasp_tf_msg.transform.translation = anygrasp_request.response.anygrasp_transforms[i].translation;
                        anygrasp_tf_msg.transform.rotation = anygrasp_request.response.anygrasp_transforms[i].rotation;

                        anygrasp_tf_broadcaster.sendTransform(anygrasp_tf_msg);
                    }

                    publish_rate.sleep();
                }
            }
            else
            {
                ROS_ERROR("Failed to call anygrasp service.");
            }
        }
        else
        {
            ROS_ERROR("Failed to connect to anygrasp service.");
        }
    }

private:
    ros::NodeHandle nh_;
    ros::ServiceClient anygrasp_publisher_;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "anygrasp_publisher_node");
    ros::NodeHandle nh;

    AnyGraspPublisher grasp_publisher(nh);
    grasp_publisher.publishAnyGrasp();

    return 0;
}
