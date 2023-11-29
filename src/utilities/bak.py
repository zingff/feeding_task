#!/home/zing/anaconda3/envs/sam/bin/python
import rospy
import open3d
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from your_package.srv import SkewerStatusCheck, SkewerStatusCheckResponse
# Include other necessary imports

# Your existing functions (convert_rgbUint32_to_tuple, convert_rgbFloat_to_tuple, convertCloudFromRosToOpen3d, etc.)

def skewer_status_service(req):
    try:
        # Wait for point cloud message
        point_cloud_topic = "/camera/depth_registered/points"
        raw_point_cloud = rospy.wait_for_message(point_cloud_topic, PointCloud2, 1.0)

        # Process point cloud
        points, colors = process_point_cloud(raw_point_cloud)
        visualize_prediction_result(points, colors)

        # Determine skewer status and point number (you need to implement the logic here)
        skewer_status = True  # Example status
        point_number = len(points)  # Example point number

        return SkewerStatusCheckResponse(skewer_status, point_number)
    except Exception as e:
        rospy.logerr(f"Service failed: {str(e)}")
        return SkewerStatusCheckResponse(False, 0)

if __name__ == '__main__':
    rospy.init_node('skewer_status_service')
    service = rospy.Service('skewer_status_check', SkewerStatusCheck, skewer_status_service)
    rospy.loginfo("Skewer Status Service Ready")
    rospy.spin()
