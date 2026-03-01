from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
import os.path as os_path
import sys

current_dir = os_path.dirname(os_path.realpath(__file__))
sys.path.insert(0, os_path.join(current_dir, '.'))
from manager import AutoStarter

frame_lidar = "body"
frame_odom = "camera_init"
resolution = 0.01
box_mapping = [-10.0, 10.0, -11.0, 11.0, -1.0, 3.0] #单位m
FOV_range = [-7.0, 52.0, -45.0, 45.0] #角度制
max_range = 5.0
sliding_threshold = 0.2
box_visual = [-5.0, 5.0, -5.0, 5.0, -1.0, 2.0]
is_pub_FOV = True
topic_FOV_visual = "/FOV_range_visual"
topic_cloud_in_add = "/cloud_registered_body"
topic_cloud_in_del = "/m_detector_cloud_body"
topic_odom = "/fast_lio_odom"
topic_grid_out = "/grid_out_visual"
topic_save_pcd = "/occupied_mapping_save_pcd"
extrinsic_T = [0.0, 0.0, 0.0]
extrinsic_Q = [0.0, 0.0, 0.0, 1.0]

param_occupied_mapping = [
    {"frame_lidar": frame_lidar},
    {"frame_odom": frame_odom},
    {"resolution": resolution},
    {"box_mapping": box_mapping},
    {"FOV_range": FOV_range},
    {"max_range": max_range},
    {"sliding_threshold": sliding_threshold},
    {"box_visual": box_visual},
    {"is_pub_FOV": is_pub_FOV},
    {"topic_FOV_visual": topic_FOV_visual},
    {"topic_cloud_in_add": topic_cloud_in_add},
    {"topic_cloud_in_del": topic_cloud_in_del},
    {"topic_odom": topic_odom},
    {"topic_grid_out": topic_grid_out},
    {"topic_save_pcd": topic_save_pcd},
    {"extrinsic_T": extrinsic_T},
    {"extrinsic_Q": extrinsic_Q}
]

def generate_launch_description():
    ld = LaunchDescription()
    starter = AutoStarter(ld)
    
    starter.startLaunch("livox_ros_driver2", "launch_ROS2", "msg_MID360_launch.py")
    # starter.startLaunch("point_lio", "launch", "point_lio.launch.py")
    starter.startLaunch("fast_lio", "launch", "mapping.launch.py")
    # starter.startNode("occupied_mapping", "occupied_mapping_node", param=param_occupied_mapping)
    
    auto_start_pkg_path = get_package_share_directory("auto_start")
    starter.startNode("rviz2", "rviz2", args=["-d", os_path.join(auto_start_pkg_path, "rviz", "odom_mapping.rviz")])
    
    return ld