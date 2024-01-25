#! /home/zing/anaconda3/envs/anygrasp/bin/python

import rospy
import open3d
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from ctypes import *
import numpy as np
import time
from feeding_task.srv import SkewerStatusCheck, SkewerStatusCheckResponse
import sys
import os

parent_dir = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
sys.path.append(parent_dir)
from fsm.utilities import get_ros_param

# TODO: just use the most simple way to check, remove point cloud conversion
# TODO: since it is time-consuming

class StateOutputStyle:
    normal = '\033[1;38;5;189m'  # light purple
    success = '\033[1;38;5;192m'  # green
    failure = '\033[1;38;5;196m'  # red
    default = '\033[0m'  # default


def normal_loginfo(msg):
    rospy.loginfo(StateOutputStyle.normal + msg + StateOutputStyle.default)


def success_loginfo(msg):
    rospy.loginfo(StateOutputStyle.success + msg + StateOutputStyle.default)


def failure_loginfo(msg):
    rospy.loginfo(StateOutputStyle.failure + msg + StateOutputStyle.default)


def convert_rgbUint32_to_tuple(rgb_uint32):
    return (
        (rgb_uint32 & 0x00ff0000) >> 16,
        (rgb_uint32 & 0x0000ff00) >> 8,
        (rgb_uint32 & 0x000000ff)
    )
    
    
def convert_rgbFloat_to_tuple(rgb_float):
    return convert_rgbUint32_to_tuple(
        int(cast(pointer(c_float(rgb_float)), POINTER(c_uint32)).contents.value)
    )


def convertCloudFromRosToOpen3d(ros_cloud):
    field_names = [field.name for field in ros_cloud.fields]
    cloud_data = list(pc2.read_points(ros_cloud, skip_nans=False, field_names=field_names))

    open3d_cloud = open3d.geometry.PointCloud()
    if len(cloud_data) == 0:
        print("Converting an empty cloud")
        return None

    if "rgb" in field_names:
        IDX_RGB_IN_FIELD = 3
        xyz = [(x, y, z) for x, y, z, rgb in cloud_data]

        if type(cloud_data[0][IDX_RGB_IN_FIELD]) == float:  
            rgb = [convert_rgbFloat_to_tuple(rgb) for x, y, z, rgb in cloud_data]
        else:
            rgb = [convert_rgbUint32_to_tuple(rgb) for x, y, z, rgb in cloud_data]

        open3d_cloud.points = open3d.utility.Vector3dVector(np.array(xyz))
        open3d_cloud.colors = open3d.utility.Vector3dVector(np.array(rgb) / 255.0)
    else:
        xyz = [(x, y, z) for x, y, z in cloud_data]
        open3d_cloud.points = open3d.utility.Vector3dVector(np.array(xyz))

    # return
    return open3d_cloud


def process_point_cloud(
  point_cloud, 
  point_cloud_lower_limit, 
  point_cloud_depth,
  remove_outlier=True):
    cloud = convertCloudFromRosToOpen3d(point_cloud)
    points = np.asarray(cloud.points)
    colors = np.asarray(cloud.colors)
    image_shape = [point_cloud.height, point_cloud.width]  # 720*1280
    points_x = points[:, 0].reshape(image_shape)
    points_y = points[:, 1].reshape(image_shape)
    points_z = points[:, 2].reshape(image_shape)

    colors_x = colors[:, 0].reshape(image_shape)
    colors_y = colors[:, 1].reshape(image_shape)
    colors_z = colors[:, 2].reshape(image_shape)

    if remove_outlier:
        point_cloud_upper_limit = point_cloud_lower_limit + point_cloud_depth
        mask = (points_z > point_cloud_lower_limit) & (points_z < point_cloud_upper_limit)
        points = np.stack([points_x, points_y, points_z], axis=-1)
        colors = np.stack([colors_x, colors_y, colors_z], axis=-1)
        points = points[mask].astype(np.float32)
        colors = colors[mask].astype(np.float32)
        point_number = points.shape[0]

    return points, colors, point_number


# deprecated, for debugging
def visualize_prediction_result(points, colors):
    cloud_transform = np.array([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])
    cloud = open3d.geometry.PointCloud()
    cloud.points = open3d.utility.Vector3dVector(points)
    cloud.colors = open3d.utility.Vector3dVector(colors)
    cloud.transform(cloud_transform)
    visualizer = open3d.visualization.Visualizer()
    if visualizer.get_window_name() == "Open3D":
        visualizer.create_window(
            window_name='AnyGrasp Visualization',
            height=540,
            width=960)
        visualizer.add_geometry(cloud)
        visualizer.run()
    return True


def skewer_status_check_service(req):
  try:
    if (req.skewer_checker_flag):
      point_cloud_confidence = get_ros_param(
        "/foodSkewer/skewerStatusChecker/pointCloudConfidence", 
        200)
      point_cloud_lower_limit = get_ros_param(
        "/foodSkewer/skewerStatusChecker/pointCloudLowerLimit", 
        0.28)
      point_cloud_depth = get_ros_param(
        "/foodSkewer/skewerStatusChecker/pointCloudRange", 
        0.06)
      point_cloud_topic = "/camera/depth_registered/points"
      raw_point_cloud = rospy.wait_for_message(point_cloud_topic, PointCloud2, 1.0)
      
      _, _, point_number = process_point_cloud(raw_point_cloud, point_cloud_lower_limit, point_cloud_depth)
      skewer_status = False
      if (point_number > point_cloud_confidence):
        skewer_status = True
        success_loginfo("Skewer status: " + str(skewer_status))
        success_loginfo("Point number: " + str(point_number))
        return SkewerStatusCheckResponse(skewer_status, point_number)
      else:
        failure_loginfo("Skewer status: " + str(skewer_status))
        failure_loginfo("Point number: " + str(point_number))
        return SkewerStatusCheckResponse(skewer_status, 0)

  except Exception as e:
    failure_loginfo(f"Service failed: {str(e)}")
    return SkewerStatusCheckResponse(False, 0)
    

if __name__ == '__main__':
    rospy.init_node('skewer_status_check_service')
    service = rospy.Service('skewer_status_checker', SkewerStatusCheck, skewer_status_check_service)
    normal_loginfo("Skewer status check service is initialized!")
    rospy.spin()
