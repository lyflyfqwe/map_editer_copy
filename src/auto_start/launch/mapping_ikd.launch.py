from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
import os.path as os_path
import sys

current_dir = os_path.dirname(os_path.realpath(__file__))
sys.path.insert(0, os_path.join(current_dir, '.'))
from manager import AutoStarter

ikd_delete_param = 0.3
ikd_balance_param = 0.6
resolution = 0.01
frame_lidar = "body"
frame_odom = "camera_init"
extrinsic_T = [0.0, 0.0, 0.0]
extrinsic_Euler = [0.0, 0.0, 0.0]
range_mapping = [-40.0, 40.0, -40.0, 40.0, -2.0, 10.0]
range_visual = [-4.0, 4.0, -4.0, 4.0, -2.0, 2.0]
range_FOV = [-7.0, 52.0, -60.0, 60.0, 0.15, 100.0]
topic_cloud_in_add = "/cloud_registered_body"
topic_odom = "/fast_lio_odom"
topic_map_visual = "/ikd_mapping_result"
is_pub_FOV = True
visual_mil_timer = 200
visual_mil_time_thres = 40 
topic_FOV_visual = "/FOV_range_visual"
topic_save_pcd = "/ikd_mapping_save_pcd"

param_ikd_mapping = [
    {"ikd_delete_param": ikd_delete_param},
    {"ikd_balance_param": ikd_balance_param},
    {"resolution": resolution},
    {"frame_lidar": frame_lidar},
    {"frame_odom": frame_odom},
    {"extrinsic_T": extrinsic_T},
    {"extrinsic_Euler": extrinsic_Euler},
    {"range_mapping": range_mapping},
    {"range_visual": range_visual},
    {"range_FOV": range_FOV},
    {"topic_cloud_in_add": topic_cloud_in_add},
    {"topic_odom": topic_odom},
    {"topic_map_visual": topic_map_visual},
    {"is_pub_FOV": is_pub_FOV},
    {"topic_FOV_visual": topic_FOV_visual},
    {"topic_save_pcd": topic_save_pcd}
]

def generate_launch_description():
    ld = LaunchDescription()
    starter = AutoStarter(ld)
    starter.startLaunch("livox_ros_driver2", "launch_ROS2", "msg_MID360_launch.py")
    starter.startLaunch("fast_lio", "launch", "mapping.launch.py")
    starter.startNode("ikd_tree_mapping", "ikd_tree_mapping_node", params=param_ikd_mapping)
    auto_start_pkg_path = get_package_share_directory("auto_start")
    starter.startNode("rviz2", "rviz2", args=["-d", os_path.join(auto_start_pkg_path, "rviz", "mapping_ikd.rviz")])
    
    return ld