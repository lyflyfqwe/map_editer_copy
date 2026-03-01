from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
import os.path as os_path
import sys

current_dir = os_path.dirname(os_path.realpath(__file__))
sys.path.insert(0, os_path.join(current_dir, '.'))
from manager import AutoStarter

frame_map = "map"
frame_odom = "camera_init"
num_threads = 4
num_neighbour = 20
size_registered_leaf = 0.25
max_dist_sq = 2.0
path_global_pcd = "/home/qwelyflyf/Documents/point_cloud/flower_2.pcd"
topic_global_cloud = "/global_cloud"
topic_cloud_body = "/cloud_registered_body"
topic_odom = "/fast_lio_odom"

param_icp_relocalization = [
    {"frame_map": frame_map},
    {"frame_odom": frame_odom},
    {"num_threads": num_threads},
    {"num_neighbour": num_neighbour},
    {"size_registered_leaf": size_registered_leaf},
    {"max_dist_sq": max_dist_sq},
    {"path_global_pcd": path_global_pcd},
    {"topic_global_cloud": topic_global_cloud},
    {"topic_cloud_body": topic_cloud_body},
    {"topic_odom": topic_odom}
]

def generate_launch_description():
    ld = LaunchDescription()
    starter = AutoStarter(ld)
    
    starter.startLaunch("livox_ros_driver2", "launch_ROS2", "msg_MID360_launch.py")
    # starter.startLaunch("point_lio", "launch", "point_lio.launch.py")
    starter.startLaunch("fast_lio", "launch", "mapping.launch.py")
    starter.startNode("icp_relocalization", "icp_relocalization_sync_node", param=param_icp_relocalization)
    
    auto_start_pkg_path = get_package_share_directory("auto_start")
    starter.startNode("rviz2", "rviz2", args=["-d", os_path.join(auto_start_pkg_path, "rviz", "odom_icp.rviz")])
    
    return ld