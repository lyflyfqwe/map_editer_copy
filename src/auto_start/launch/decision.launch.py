from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    '''获取YAML配置文件路径
    config_path = os.path.join(
        get_package_share_directory('auto_start'),
        'config',
        'region_points.yaml'
    )
    '''


    decision_node = Node(
        package='rm_decision',
        executable='rm_decision_node',
        name='rm_decision_node',
        parameters=[{
            #'region_config': config_path,
            'decision.lowwer_hp': 30,
            'decision.attack_distance': 5.0
        }],
        output='screen'  # 打印日志到终端
    )

    return LaunchDescription([
        decision_node
    ])