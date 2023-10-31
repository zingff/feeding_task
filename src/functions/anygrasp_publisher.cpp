#include "anygrasp_publisher.hpp"

AnyGraspPublisher::AnyGraspPublisher(ros::NodeHandle& nh) : nh_(nh)
{
    anygrasp_publisher_ = nh_.serviceClient<anygrasp_generation::AnyGraspGeneration>("/grasp_generator");
}

void AnyGraspPublisher::publishAnyGrasp()
{
    anygrasp_generation::AnyGraspGeneration anygrasp_request;
    anygrasp_request.request.update_anygrasp = true;

    if (anygrasp_publisher_.call(anygrasp_request))
    {
        if (anygrasp_request.response.success)
        {
            ROS_INFO("Succeeded to call anygrasp!");

            ros::NodeHandle private_nh("~");
            std::string grasp_namespace = "anygrasp";

            // Store transforms as ROS parameters
            for (size_t i = 0; i < anygrasp_request.response.anygrasp_transforms.size(); ++i)
            {
                std::string param_name = grasp_namespace + "/grasp_" + std::to_string(i);
                geometry_msgs::Transform transform = anygrasp_request.response.anygrasp_transforms[i];
                private_nh.setParam(param_name + "/translation/x", transform.translation.x);
                private_nh.setParam(param_name + "/translation/y", transform.translation.y);
                private_nh.setParam(param_name + "/translation/z", transform.translation.z);
                private_nh.setParam(param_name + "/rotation/x", transform.rotation.x);
                private_nh.setParam(param_name + "/rotation/y", transform.rotation.y);
                private_nh.setParam(param_name + "/rotation/z", transform.rotation.z);
                private_nh.setParam(param_name + "/rotation/w", transform.rotation.w);
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



// void AnyGraspPublisher::publishAnyGrasp()
// {
//     anygrasp_generation::AnyGraspGeneration anygrasp_request;
//     anygrasp_request.request.update_anygrasp = true;

//     if (anygrasp_publisher_.call(anygrasp_request))
//     {
//         if (anygrasp_request.response.success)
//         {
//             ROS_INFO("Succeeded to call anygrasp!");

//             tf2_ros::Buffer tf_buffer;
//             tf_buffer.setUsingDedicatedThread(true); // Disable TF2 cache

//             tf2_ros::TransformBroadcaster anygrasp_tf_broadcaster;
//             std::string grasp_namespace = "anygrasp";

//             // Create and send transforms
//             for (size_t i = 0; i < anygrasp_request.response.anygrasp_transforms.size(); ++i)
//             {
//                 geometry_msgs::TransformStamped anygrasp_tf_msg;
//                 anygrasp_tf_msg.header.stamp = ros::Time::now();
//                 anygrasp_tf_msg.header.frame_id = "base_link";
//                 anygrasp_tf_msg.child_frame_id = grasp_namespace + "/grasp_" + std::to_string(i);

//                 anygrasp_tf_msg.transform.translation = anygrasp_request.response.anygrasp_transforms[i].translation;
//                 anygrasp_tf_msg.transform.rotation = anygrasp_request.response.anygrasp_transforms[i].rotation;

//                 anygrasp_tf_broadcaster.sendTransform(anygrasp_tf_msg);
//             }
//         }
//         else
//         {
//             ROS_ERROR("Failed to call anygrasp service.");
//         }
//     }
//     else
//     {
//         ROS_ERROR("Failed to connect to anygrasp service.");
//     }
// }



// void AnyGraspPublisher::publishAnyGrasp()
// {
//     anygrasp_generation::AnyGraspGeneration anygrasp_request;
//     anygrasp_request.request.update_anygrasp = true;

//     if (anygrasp_publisher_.call(anygrasp_request))
//     {
//         if (anygrasp_request.response.success)
//         {
//             ROS_INFO("Succeeded to call anygrasp!");

//             tf2_ros::TransformBroadcaster anygrasp_tf_broadcaster;
//             std::string grasp_namespace = "anygrasp";
//             ros::Rate publish_rate(10);

//             while (ros::ok())
//             {
//                 for (size_t i = 0; i < anygrasp_request.response.anygrasp_transforms.size(); ++i)
//                 {
//                     geometry_msgs::TransformStamped anygrasp_tf_msg;
//                     anygrasp_tf_msg.header.stamp = ros::Time::now();
//                     anygrasp_tf_msg.header.frame_id = "base_link";
//                     anygrasp_tf_msg.child_frame_id = grasp_namespace + "/grasp_" + std::to_string(i);

//                     anygrasp_tf_msg.transform.translation = anygrasp_request.response.anygrasp_transforms[i].translation;
//                     anygrasp_tf_msg.transform.rotation = anygrasp_request.response.anygrasp_transforms[i].rotation;

//                     anygrasp_tf_broadcaster.sendTransform(anygrasp_tf_msg);
//                 }

//                 publish_rate.sleep();
//             }
//         }
//         else
//         {
//             ROS_ERROR("Failed to call anygrasp service.");
//         }
//     }
//     else
//     {
//         ROS_ERROR("Failed to connect to anygrasp service.");
//     }
// }
